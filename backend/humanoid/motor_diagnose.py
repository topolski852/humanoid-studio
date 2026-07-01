"""Diagnostic + gravity-aware tuning routines for Recoil motor controllers.

Two workflows, both driven over the daemon proxy (zero-latency cached telemetry):

  run_diagnosis()    — Workflow A: classify *why* a joint misbehaves before tuning
                       gains (moves / stuck-torque-starved / commutation-fault /
                       Kp-starved / gravity-load / runaway, + stick-slip overlay),
                       and recommend a (confirmed, never auto-applied) remediation.

  run_gravity_tune() — Workflow B: asymmetric lift/drop characterization of a
                       gravity-loaded joint, sweep velocity_kp (Kd) for the descent
                       "slam", recommend Kp for droop, and probe Ki windup.

Every actuated routine is wrapped by _guarded_drive(), which aborts (disables the
motor) the instant position leaves a runaway band, clips targets to position
limits, and is always run under a conservative diagnosis torque limit that is
restored on exit.  No routine here performs an irreversible action on its own —
phase flip + recalibrate + flash are separate, explicitly-confirmed endpoints.
"""
from __future__ import annotations

import asyncio
import math
import time

from humanoid.daemon_client import DaemonActuatorProxy
from humanoid.motor_tune import (
    _MODE_POSITION,
    _current_motion_ratio,
    _descent_metrics,
    _hold_error,
    _motion_range,
    _torque_chatter_pp,
    _values,
    _velocity_reversals,
)

_R2D = 180.0 / math.pi
_LIMIT_MARGIN = 0.02   # rad — keep clear of firmware soft limits (matches run_step_test)


class RunawayAbort(Exception):
    """Raised (after disabling the motor) when position leaves the runaway band."""

    def __init__(self, pos: float, start: float, band: float) -> None:
        self.pos, self.start, self.band = pos, start, band
        super().__init__(
            f"Runaway abort: position {pos:+.3f} left ±{band:.3f} rad band "
            f"around start {start:+.3f} — motor disabled"
        )


# ---------------------------------------------------------------------------
# Low-level guarded actuation
# ---------------------------------------------------------------------------

def _require_position_mode(actuator: DaemonActuatorProxy):
    st = actuator.get_cached_state()
    if st is None:
        raise ValueError("Motor is OFFLINE — connect and enable it first")
    if st.mode != _MODE_POSITION:
        raise ValueError(
            f"Motor must be in POSITION mode (current mode 0x{st.mode:02X}) — "
            f"enable it from the panel first"
        )
    return st


def _limits(actuator: DaemonActuatorProxy) -> tuple[float, float]:
    lim = actuator.config.position_limits
    return lim.lower_bound + _LIMIT_MARGIN, lim.upper_bound - _LIMIT_MARGIN


def _available_range(actuator: DaemonActuatorProxy) -> float:
    lo, hi = _limits(actuator)
    return max(0.0, hi - lo)


def _clamp_target(actuator: DaemonActuatorProxy, target: float) -> float:
    lo, hi = _limits(actuator)
    return max(lo, min(hi, target))


def _torque_saturated(samples: list[dict], torque_limit: float, sat_frac: float = 0.95) -> bool:
    ts = [abs(t) for t in _values(samples, "torque")]
    return any(t >= torque_limit * sat_frac for t in ts) if ts else False


async def _guarded_drive(
    actuator: DaemonActuatorProxy,
    target_fn,
    *,
    duration_s: float,
    start_pos: float,
    runaway_band_rad: float,
    sample_dt: float = 0.01,
    on_sample=None,
) -> list[dict]:
    """Drive set_position(target_fn(frac)) for duration_s, sampling cached state.

    target_fn(frac in [0,1]) -> absolute target (rad).  Aborts + disables on
    runaway.  Returns run_step_test-shaped samples.
    """
    samples: list[dict] = []
    t0 = time.monotonic()
    while True:
        el = time.monotonic() - t0
        if el >= duration_s:
            break
        frac = (el / duration_s) if duration_s > 0 else 1.0
        target = target_fn(frac)
        await actuator.set_position(target)
        st = actuator.get_cached_state()
        if st is not None:
            sample = {
                "t_ms":        int(el * 1000),
                "commanded":   target,
                "position":    st.position,
                "velocity":    st.velocity,
                "torque":      st.torque,
                "current":     st.current,
                "bus_voltage": st.bus_voltage,
            }
            samples.append(sample)
            if on_sample is not None:
                on_sample(sample)
            if abs(st.position - start_pos) > runaway_band_rad:
                await actuator.disable()
                raise RunawayAbort(st.position, start_pos, runaway_band_rad)
        await asyncio.sleep(sample_dt)
    return samples


async def _hold(actuator, target, *, duration_s, start_pos, runaway_band_rad, sample_dt=0.01):
    return await _guarded_drive(
        actuator, lambda _f: target, duration_s=duration_s,
        start_pos=start_pos, runaway_band_rad=runaway_band_rad, sample_dt=sample_dt,
    )


async def _ramp(actuator, start_pos, delta, *, duration_s, runaway_band_rad, sample_dt=0.02):
    return await _guarded_drive(
        actuator,
        lambda f: _clamp_target(actuator, start_pos + delta * f),
        duration_s=duration_s, start_pos=start_pos,
        runaway_band_rad=runaway_band_rad, sample_dt=sample_dt,
    )


async def _restore(actuator, kp, ki, kd, tl, start_pos):
    """Best-effort: restore entry gains and return to the entry position."""
    try:
        await actuator.write_gains(kp, ki, kd, tl)
    except Exception:
        pass
    try:
        st = actuator.get_cached_state()
        if st is not None and st.mode == _MODE_POSITION:
            await actuator.set_position(_clamp_target(actuator, start_pos))
    except Exception:
        pass


# ---------------------------------------------------------------------------
# Commutation probe (shared by commissioning + diagnosis)
# ---------------------------------------------------------------------------

async def commutation_probe(
    actuator: DaemonActuatorProxy,
    *,
    step_rad: float = 0.12,
    move_s: float = 1.2,
    probe_torque_limit: float | None = None,
    runaway_band_rad: float | None = None,
    motion_stuck_rad: float = 0.05,
    min_fault_current_a: float = 1.0,
) -> dict:
    """Lightweight commutation check: one small guarded move under reduced torque.

    A commutation fault (wrong phase_order: current flows but produces no real
    torque) shows up as the encoder staying *frozen while the motor draws real
    current* in BOTH directions. We probe both ways because the joint is usually
    gravity-loaded (mounted): a healthy joint may be stuck going uphill yet still
    moves in the gravity-*assisted* direction — so "moves in either direction"
    means it commutates, while "stuck both ways while energized" is the fault. A
    merely Kp-starved joint is stuck both ways but draws little current.

    Returns {commutates, reason, moved_rad, max_current_a, runaway}. Restores
    entry gains/torque and returns to start on exit. Motor must be enabled in
    POSITION mode.
    """
    cfg = actuator.config
    start = _require_position_mode(actuator)
    start_pos = start.position
    kp0, ki0, kd0, tl0 = cfg.position_kp, cfg.position_ki, cfg.velocity_kp, cfg.torque_limit

    if probe_torque_limit is None:
        probe_torque_limit = min(tl0, max(0.5, 0.6 * tl0))
    avail = _available_range(actuator)
    if runaway_band_rad is None:
        runaway_band_rad = min(0.35, 0.5 * avail) if avail > 0 else 0.35

    moved = 0.0
    max_current = 0.0
    probed_any = False
    try:
        await actuator.write_gains(kp0, ki0, kd0, probe_torque_limit)
        for direction in (+1.0, -1.0):
            target = _clamp_target(actuator, start_pos + direction * step_rad)
            if abs(target - start_pos) < 0.5 * step_rad:
                continue  # no room this way — try the other direction
            probed_any = True
            try:
                samples = await _ramp(actuator, start_pos, target - start_pos,
                                      duration_s=move_s, runaway_band_rad=runaway_band_rad)
            except RunawayAbort:
                # Produced enough torque to leave the band — it commutates (the
                # loop sign / direction is a separate, gear_ratio concern).
                return {"commutates": True, "reason": "runaway",
                        "moved_rad": runaway_band_rad, "max_current_a": None,
                        "runaway": True}
            d = max((abs(s["position"] - start_pos) for s in samples), default=0.0)
            c = max((abs(s["current"]) for s in samples), default=0.0)
            moved = max(moved, d)
            max_current = max(max_current, c)
            # return to start before testing the other direction
            await _hold(actuator, _clamp_target(actuator, start_pos), duration_s=0.5,
                        start_pos=start_pos, runaway_band_rad=runaway_band_rad)
            if d >= motion_stuck_rad:
                break  # moved in this direction → it commutates; no need to test the other
    finally:
        await _restore(actuator, kp0, ki0, kd0, tl0, start_pos)

    if not probed_any:
        return {"commutates": True, "reason": "no_room",
                "moved_rad": 0.0, "max_current_a": round(max_current, 3), "runaway": False}

    stuck = moved < motion_stuck_rad          # stuck in every direction tried
    energized = max_current >= min_fault_current_a
    fault = stuck and energized               # frozen both ways while drawing real current
    return {
        "commutates": not fault,
        "reason": ("stuck_and_energized" if fault
                   else "moved" if not stuck else "stuck_no_current"),
        "moved_rad": round(moved, 4),
        "max_current_a": round(max_current, 3),
        "runaway": False,
    }


async def jog(
    actuator: DaemonActuatorProxy,
    *,
    step_rad: float = 0.15,
    move_s: float = 1.5,
    hold_s: float = 0.6,
    runaway_band_rad: float | None = None,
    return_to_start: bool = True,
    gentle: bool = True,
) -> dict:
    """Move the joint a small, visible amount so a human can see which way it
    goes, then return to start. Used by the Direction & Limits step: the user
    watches and confirms whether it moved in the +direction.

    gentle (default) caps Kp and the torque limit for the move so an UNtuned
    joint — e.g. one just commissioned, before PID tuning — can't slam; gains are
    restored on exit. The cap bounds the force, so the joint moves softly (it may
    not overcome gravity going uphill, but the hardstops already fixed direction;
    this jog is only a visual confirmation). Returns the signed motion (end-start).
    Motor must be enabled in POSITION mode."""
    cfg = actuator.config
    start = _require_position_mode(actuator)
    start_pos = start.position
    kp0, ki0, kd0, tl0 = cfg.position_kp, cfg.position_ki, cfg.velocity_kp, cfg.torque_limit
    avail = _available_range(actuator)
    if runaway_band_rad is None:
        runaway_band_rad = min(0.4, 0.6 * avail) if avail > 0 else 0.4

    lo, hi = _limits(actuator)
    step = step_rad if (start_pos + step_rad) <= hi else -step_rad
    try:
        if gentle:
            # Cap Kp so the approach force is bounded (no slam) and raise Kd for
            # damping, but keep a generous torque ceiling: a too-low torque cap
            # lets a gravity-loaded joint fall (and starves the Kd term that
            # would resist the fall). Peak force is then ~Kp*error, not the cap.
            await actuator.write_gains(min(kp0, 10.0), 0.0, max(kd0, 3.0),
                                       min(tl0, 8.0))
        samples = await _ramp(actuator, start_pos, step, duration_s=move_s,
                              runaway_band_rad=runaway_band_rad)
        end_pos = samples[-1]["position"] if samples else start_pos
        signed = end_pos - start_pos
        if return_to_start:
            await _hold(actuator, _clamp_target(actuator, start_pos),
                        duration_s=hold_s, start_pos=start_pos,
                        runaway_band_rad=runaway_band_rad)
    finally:
        if gentle:
            try:
                await actuator.write_gains(kp0, ki0, kd0, tl0)
            except Exception:
                pass
    return {
        "commanded_step_rad": round(step, 4),
        "signed_motion_rad": round(signed, 4),
        "moved_rad": round(abs(signed), 4),
    }


# ---------------------------------------------------------------------------
# Workflow A — diagnosis
# ---------------------------------------------------------------------------

def _recommendation(action, *, destructive, confirm_label, params=None, rationale=None):
    return {
        "action": action,
        "destructive": destructive,
        "confirm_label": confirm_label,
        "params": params or {},
        "rationale": rationale or [],
    }


async def run_diagnosis(
    actuator: DaemonActuatorProxy,
    *,
    step_rad: float = 0.15,
    hold_s: float = 1.0,
    ramp_rad: float = 0.25,
    ramp_s: float = 3.0,
    diag_torque_limit: float | None = None,
    runaway_band_rad: float | None = None,
    current_fault_a: float = 3.0,
    move_ok_frac: float = 0.5,
    motion_stuck_rad: float = 0.05,
    sat_frac: float = 0.95,
    reversal_count: int = 4,
    vel_eps: float = 0.05,
) -> dict:
    """Classify the root cause of a joint's behavior. Never acts destructively."""
    cfg = actuator.config
    start = _require_position_mode(actuator)
    start_pos = start.position
    kp0, ki0, kd0, tl0 = cfg.position_kp, cfg.position_ki, cfg.velocity_kp, cfg.torque_limit

    if diag_torque_limit is None:
        diag_torque_limit = min(tl0, max(0.5, 0.6 * tl0))
    avail = _available_range(actuator)
    if runaway_band_rad is None:
        runaway_band_rad = min(0.35, 0.5 * avail) if avail > 0 else 0.35

    thresholds = {
        "diag_torque_limit": round(diag_torque_limit, 3),
        "runaway_band_rad": round(runaway_band_rad, 3),
        "current_fault_a": current_fault_a,
        "move_ok_frac": move_ok_frac,
        "motion_stuck_rad": motion_stuck_rad,
        "sat_frac": sat_frac,
        "reversal_count": reversal_count,
    }
    rationale: list[str] = []
    phases: dict[str, list] = {}

    def result(classification, recommendation, evidence, flags=None):
        return {
            "classification": classification,
            "flags": flags or [],
            "evidence": evidence,
            "recommendation": recommendation,
            "rationale": rationale,
            "thresholds": thresholds,
            "samples_by_phase": phases,
            "entry_gains": {"position_kp": kp0, "position_ki": ki0,
                            "velocity_kp": kd0, "torque_limit": tl0},
        }

    try:
        await actuator.write_gains(kp0, ki0, kd0, diag_torque_limit)

        # -- Phase 1: move test ------------------------------------------------
        target = _clamp_target(actuator, start_pos + step_rad)
        if abs(target - start_pos) < motion_stuck_rad * 1.5:          # no room up; try down
            target = _clamp_target(actuator, start_pos - step_rad)
        commanded_delta = abs(target - start_pos)
        if commanded_delta < motion_stuck_rad:
            return result(
                "NO_ROOM",
                _recommendation("none", destructive=False,
                                confirm_label="", rationale=["Joint is at a limit — move it toward mid-range first"]),
                {"available_range_rad": round(avail, 3)},
            )
        try:
            s_move = await _hold(actuator, target, duration_s=hold_s,
                                 start_pos=start_pos, runaway_band_rad=runaway_band_rad)
        except RunawayAbort as ra:
            phases["move"] = []
            rationale.append(
                f"Position ran away past ±{runaway_band_rad:.2f} rad on a small command "
                f"— positive feedback (check phase / gear-ratio sign)."
            )
            return result(
                "RUNAWAY",
                _recommendation("none", destructive=False, confirm_label="",
                                rationale=["Inspect phase_inverted / gear_ratio sign; do not raise gains"]),
                {"abort_pos": round(ra.pos, 3)},
            )

        phases["move"] = s_move
        # Displacement from the start point (not intra-sample range) — robust to a
        # joint that snaps to target and then holds.
        moved = max((abs(s["position"] - start_pos)
                     for s in s_move if s.get("position") is not None), default=0.0)
        sat = _torque_saturated(s_move, diag_torque_limit, sat_frac)
        reversals = _velocity_reversals(s_move, vel_eps)
        chatter = _torque_chatter_pp(s_move)
        cm_move = _current_motion_ratio(s_move)

        flags = []
        if reversals >= reversal_count and moved < motion_stuck_rad * 2:
            flags.append("STICK_SLIP")

        evidence_common = {
            "move_motion_rad": round(moved, 4),
            "move_commanded_rad": round(commanded_delta, 4),
            "torque_saturated": sat,
            "velocity_reversals": reversals,
            "torque_chatter_pp_nm": chatter,
            "move_current": cm_move,
        }

        # -- moves fine --------------------------------------------------------
        if moved >= move_ok_frac * commanded_delta:
            rationale.append(
                f"Tracked {moved * _R2D:.1f}° of a {commanded_delta * _R2D:.1f}° command "
                f"(≥{move_ok_frac:.0%}) — joint moves under closed-loop control."
            )
            note = "Use the Gravity Tune tab to tune Kp/Kd if it slams or droops."
            if "STICK_SLIP" in flags:
                note = "Tracks, but velocity reversals/chatter seen — watch for stick-slip when tuning."
            return result("MOVES_OK",
                          _recommendation("none", destructive=False, confirm_label="",
                                          rationale=[note]),
                          evidence_common, flags)

        # -- stuck: torque-starved vs Kp-starved vs commutation ----------------
        rationale.append(
            f"Only moved {moved * _R2D:.2f}° of {commanded_delta * _R2D:.1f}° — stuck."
        )
        if sat:
            rationale.append(
                f"Torque saturated at the {diag_torque_limit:.1f} Nm diagnosis limit while stuck "
                f"— torque-starved; raising the torque limit may free it."
            )
            return result(
                "STUCK_TORQUE_STARVED",
                _recommendation("raise_torque_limit", destructive=False,
                                confirm_label=f"Raise torque limit to {round(tl0 * 1.5, 1)} Nm",
                                params={"torque_limit": round(min(tl0 * 1.5, tl0 + 4.0), 1)},
                                rationale=["Torque saturated while stuck"]),
                evidence_common, flags)

        # Commutation probe: ramp BOTH ways. A commutation/phase fault stays stuck
        # with HIGH current in BOTH directions (current flows, encoder frozen =
        # fictitious torque).  If it MOVES one way, that direction is just gravity-
        # /Kp-limited, not a fault.
        cm_probe = {}
        moved_any = False
        high_current_any = False
        for label, sign in (("ramp_up", +1.0), ("ramp_down", -1.0)):
            if _available_range(actuator) <= 0:
                continue
            delta = sign * ramp_rad
            try:
                s_ramp = await _ramp(actuator, start_pos, delta, duration_s=ramp_s,
                                     runaway_band_rad=runaway_band_rad)
            except RunawayAbort as ra:
                phases[label] = []
                return result(
                    "RUNAWAY",
                    _recommendation("none", destructive=False, confirm_label="",
                                    rationale=["Ran away during commutation ramp — check phase/sign"]),
                    {"abort_pos": round(ra.pos, 3)}, flags)
            phases[label] = s_ramp
            cm = _current_motion_ratio(s_ramp)
            cm_probe[label] = cm
            if cm["motion_range_rad"] >= motion_stuck_rad:
                moved_any = True
            if cm["max_current_a"] > current_fault_a:
                high_current_any = True

        evidence = {**evidence_common, "ramp_current": cm_probe}

        if not moved_any and high_current_any:
            rationale.append(
                f"Drew >{current_fault_a:.0f} A in BOTH directions but the encoder stayed frozen "
                f"(<{motion_stuck_rad * _R2D:.0f}°) — current flows without producing torque. "
                f"That is a closed-loop commutation/phase fault (motor spins fine in open-loop "
                f"calibration, but POSITION mode can't commutate)."
            )
            return result(
                "COMMUTATION_FAULT",
                _recommendation("phase_flip_and_recal", destructive=True,
                                confirm_label="Flip phase order + recalibrate flux (~90 s, motor will spin)",
                                rationale=["High current, no rotation, both directions = wrong phase order"]),
                evidence, flags)

        if not moved_any and not high_current_any:
            rationale.append(
                f"Stuck but current stayed low (<{current_fault_a:.0f} A) — the controller never "
                f"asked for enough torque to break free. Kp is too low for the load."
            )
            return result(
                "KP_STARVED",
                _recommendation("raise_kp", destructive=False,
                                confirm_label=f"Raise Kp to {round(kp0 * 1.5, 1)}",
                                params={"position_kp": round(kp0 * 1.5, 1)},
                                rationale=["Stuck with low current = Kp too low to break stiction/load"]),
                evidence, flags)

        # moved in one direction only ⇒ gravity / load asymmetry, not a fault
        rationale.append(
            "Moved in one direction but not the other while drawing current — gravity/load "
            "asymmetry, not a commutation fault. Tune Kp/Kd in the Gravity Tune tab."
        )
        return result(
            "GRAVITY_LOAD",
            _recommendation("none", destructive=False, confirm_label="",
                            rationale=["Asymmetric motion under load — use Gravity Tune"]),
            evidence, flags)

    finally:
        await _restore(actuator, kp0, ki0, kd0, tl0, start_pos)


# ---------------------------------------------------------------------------
# Workflow B — gravity-aware gain sweep
# ---------------------------------------------------------------------------

async def run_gravity_tune(
    actuator: DaemonActuatorProxy,
    *,
    kp: float,
    ki: float = 0.0,
    kd_values: list[float] | None = None,
    lift_rad: float = 0.25,
    lift_sign: float = 1.0,
    torque_limit: float,
    hold_s: float = 1.5,
    test_ki: bool = False,
    ki_probe: float = 5.0,
    runaway_band_rad: float | None = None,
    target_droop_deg: float = 2.0,
    knee_improve_frac: float = 0.10,
) -> dict:
    """Sweep Kd over a lift(hold)/drop cycle on a gravity-loaded joint.

    lift_sign = +1 lifts toward the upper limit, -1 toward the lower limit (the
    direction that opposes gravity — set from the UI per joint).  For each Kd:
    lift AGAINST gravity and measure droop (hold error); drop WITH gravity and
    measure descent peak velocity + overshoot (the slam).  Recommends the Kd at
    the damping knee and a Kp to cut droop; optionally probes Ki windup.
    """
    if kd_values is None:
        kd_values = [0.5, 1.0, 2.0, 4.0, 8.0]
    kd_values = [float(k) for k in kd_values if k > 0][:8]
    if not kd_values:
        raise ValueError("kd_values must contain at least one positive value")

    cfg = actuator.config
    start = _require_position_mode(actuator)
    start_pos = start.position
    kp0, ki0, kd0, tl0 = cfg.position_kp, cfg.position_ki, cfg.velocity_kp, cfg.torque_limit

    lift_sign = 1.0 if lift_sign >= 0 else -1.0
    hold_point = _clamp_target(actuator, start_pos + lift_sign * abs(lift_rad))
    drop_point = _clamp_target(actuator, start_pos)
    swing = abs(hold_point - drop_point)
    if swing < 0.05:
        raise ValueError(
            f"Lift range too small after clipping to limits ({swing * _R2D:.1f}°). "
            f"Move the joint away from its limit or pick the other lift direction."
        )
    avail = _available_range(actuator)
    if runaway_band_rad is None:
        runaway_band_rad = min(0.45, 0.6 * avail) if avail > 0 else 0.45
    runaway_band_rad = max(runaway_band_rad, swing + 0.1)   # don't abort on the intended swing

    rationale: list[str] = []

    async def lift_drop(kd_val):
        await actuator.write_gains(kp, ki, kd_val, torque_limit)
        s_lift = await _hold(actuator, hold_point, duration_s=hold_s,
                             start_pos=start_pos, runaway_band_rad=runaway_band_rad)
        droop = _hold_error(s_lift, hold_point)                 # signed (rad)
        s_drop = await _hold(actuator, drop_point, duration_s=hold_s,
                             start_pos=start_pos, runaway_band_rad=runaway_band_rad)
        dm = _descent_metrics(s_drop, drop_point)
        sat = _torque_saturated(s_drop, torque_limit) or _torque_saturated(s_lift, torque_limit)
        cur = max(_current_motion_ratio(s_lift)["max_current_a"],
                  _current_motion_ratio(s_drop)["max_current_a"])
        # Oscillation (velocity reversals) is the robust damping signal — it works
        # even on a low-gravity joint where the descent-overshoot is small/noisy.
        reversals = _velocity_reversals(s_lift, 0.1) + _velocity_reversals(s_drop, 0.1)
        return {
            "kd": round(kd_val, 3),
            "droop_rad": abs(droop),
            "descent_peak_velocity": dm["peak_velocity"],
            "descent_overshoot_rad": dm["overshoot_rad"],
            "reversals": reversals,
            "max_current_a": round(cur, 3),
            "torque_saturated": sat,
        }

    sweep: list[dict] = []
    windup = {"tested": False, "detected": False, "delta_pct": None, "ki_probe": ki_probe}

    async def recover_to_start():
        """After a runaway abort (motor disabled), re-enable and gently bring the
        joint back to the start with well-damped gains so it's ready for the next Kd."""
        try:
            await actuator.enable()   # POSITION; seeds target = current pos (no jump)
        except Exception:
            pass
        await asyncio.sleep(0.2)
        try:
            await actuator.write_gains(min(kp, 15.0), 0.0, max(kd0, 8.0), torque_limit)
            await _hold(actuator, drop_point, duration_s=1.5, start_pos=start_pos,
                        runaway_band_rad=runaway_band_rad * 2.0)
        except RunawayAbort:
            pass

    try:
        for kd_val in kd_values:
            try:
                sweep.append(await lift_drop(kd_val))
            except RunawayAbort:
                # Too under-damped at this Kd — it overshot past the guard. That IS
                # data (this Kd is too low); record it unstable and keep sweeping.
                sweep.append({
                    "kd": round(kd_val, 3), "unstable": True, "droop_rad": None,
                    "descent_peak_velocity": float("inf"),
                    "descent_overshoot_rad": float("inf"),
                    "reversals": 999,
                    "max_current_a": None, "torque_saturated": None,
                })
                await recover_to_start()

        stable = [r for r in sweep if not r.get("unstable")]
        if not stable:
            raise ValueError(
                "Every Kd in the sweep was too under-damped (ran away). This joint "
                "needs much more damping — raise the Kd values and re-run."
            )

        # Kd selection: prefer DAMPING, not descent speed. A well-damped Kd shows
        # little OSCILLATION (velocity reversals — the robust signal, works even on
        # low-gravity joints) and low descent overshoot. Pick the LOWEST such Kd;
        # more Kd past that only adds sluggishness. If none qualify, take the highest
        # stable Kd (the most damping we measured).
        overshoot_ok = max(swing * 0.15, 0.03)   # <=15% of swing (or ~1.7°)
        reversal_ok = 3                           # a damped move barely reverses
        well_damped = [r for r in stable
                       if r.get("reversals", 0) <= reversal_ok
                       and r["descent_overshoot_rad"] <= overshoot_ok]
        n_unstable = len(sweep) - len(stable)
        if well_damped:
            best = min(well_damped, key=lambda r: r["kd"])
            note = (f"{n_unstable} lower Kd ran away (too under-damped); "
                    if n_unstable else "")
            rationale.append(
                f"Kd={best['kd']:g} is the lowest well-damped move "
                f"({best.get('reversals', 0)} velocity reversals, overshoot "
                f"{best['descent_overshoot_rad'] * _R2D:.1f}°). {note}"
                f"Higher Kd only adds sluggishness."
            )
        else:
            # Fewest reversals, then lowest Kd — the closest to critically damped.
            best = min(stable, key=lambda r: (r.get("reversals", 0), r["kd"]))
            rationale.append(
                f"No Kd fully damped it; picked Kd={best['kd']:g} with the fewest "
                f"reversals ({best.get('reversals', 0)}) — consider even higher Kd."
            )
        selected_kd = best["kd"]

        # Kp recommendation from droop (droop ≈ gravity_torque / Kp)
        droop_rad = best["droop_rad"]
        target_droop_rad = target_droop_deg / _R2D
        if droop_rad > target_droop_rad and target_droop_rad > 0:
            kp_new = min(kp * 1.5, kp * droop_rad / target_droop_rad)
            kd_new = round(selected_kd * math.sqrt(kp_new / kp), 2)
            kp_new = round(kp_new, 1)
            rationale.append(
                f"Hold droop {droop_rad * _R2D:.1f}° at Kp={kp:g}; raising Kp→{kp_new:g} "
                f"(Kd→{kd_new:g} to keep damping) targets ~{target_droop_deg:g}° droop."
            )
        else:
            kp_new, kd_new = round(kp, 1), round(selected_kd, 2)
            rationale.append(f"Hold droop {droop_rad * _R2D:.1f}° already within ~{target_droop_deg:g}°.")

        # Ki windup probe (opt-in)
        if test_ki:
            windup["tested"] = True
            probe = await _lift_drop_ki(
                actuator, kp, ki_probe, selected_kd, hold_point, drop_point,
                hold_s, start_pos, runaway_band_rad, torque_limit,
            )
            base_v = best["descent_peak_velocity"]
            dv = (probe["descent_peak_velocity"] - base_v) / base_v if base_v > 1e-6 else 0.0
            windup["delta_pct"] = round(dv * 100, 1)
            windup["probe_row"] = probe
            if dv > 0.25:
                windup["detected"] = True
                rationale.append(
                    f"Adding Ki={ki_probe:g} raised descent velocity {dv * 100:.0f}% "
                    f"(integral windup → worse slam) — keep Ki≈0."
                )
            else:
                rationale.append(f"Ki={ki_probe:g} did not worsen the descent ({dv * 100:+.0f}%).")

        residual_droop_deg = round(best["droop_rad"] * _R2D, 2)
        return {
            "sweep": sweep,
            "selected_kd": selected_kd,
            "recommended": {"kp": kp_new, "kd": kd_new, "ki": 0.0},
            "windup": windup,
            "residual_droop_deg": residual_droop_deg,
            "rationale": rationale,
            "params_used": {
                "hold_point_rad": round(hold_point, 4),
                "drop_point_rad": round(drop_point, 4),
                "lift_sign": lift_sign,
                "torque_limit": torque_limit,
            },
        }
    finally:
        await _restore(actuator, kp0, ki0, kd0, tl0, start_pos)


# ---------------------------------------------------------------------------
# Breakaway torque discovery
# ---------------------------------------------------------------------------

async def find_breakaway_torque(
    actuator: DaemonActuatorProxy,
    *,
    step_rad: float = 0.15,
    torque_start: float = 0.5,
    torque_step: float = 0.5,
    torque_max: float | None = None,
    move_threshold_rad: float = 0.03,
    settle_s: float = 0.6,
    probe_s: float = 0.6,
    runaway_band_rad: float | None = None,
) -> dict:
    """Ramp the torque limit up until the joint breaks away and moves — the
    minimum torque that produces motion = gravity load + static friction.

    A high Kp is used so the torque LIMIT (not Kp·error) is the binding constraint,
    so the applied torque tracks the limit as we ramp it. Both directions are
    tested when there's room: the harder direction is uphill (gravity + friction),
    the easier is downhill (friction − gravity), which separates gravity (the
    up/down asymmetry) from friction (the common part). Motor must be in POSITION
    mode; entry gains and position are restored on exit.
    """
    cfg = actuator.config
    start = _require_position_mode(actuator)
    start_pos = start.position
    kp0, ki0, kd0, tl0 = cfg.position_kp, cfg.position_ki, cfg.velocity_kp, cfg.torque_limit
    if torque_max is None:
        torque_max = tl0
    avail = _available_range(actuator)
    if runaway_band_rad is None:
        runaway_band_rad = min(0.4, 0.6 * avail) if avail > 0 else 0.4
    runaway_band_rad = max(runaway_band_rad, abs(step_rad) + 0.1)

    # Kp high enough that the torque limit binds (applied torque = limit).
    kp_test = max(kp0, 2.0 * torque_max / max(abs(step_rad), 0.05))
    kd_test = max(kd0, 3.0)

    levels: list[float] = []
    t = torque_start
    while t < torque_max - 1e-6:
        levels.append(round(t, 3)); t += torque_step
    levels.append(round(torque_max, 3))

    lo, hi = _limits(actuator)
    dirs = []
    if (start_pos + abs(step_rad)) <= hi:
        dirs.append(("+", +1.0))
    if (start_pos - abs(step_rad)) >= lo:
        dirs.append(("-", -1.0))
    if not dirs:
        raise ValueError("No room to move in either direction — center the joint first.")

    async def return_to_start():
        # Bring the joint back to start with full torque + damping before the next probe.
        try:
            await actuator.write_gains(kp_test, 0.0, max(kd_test, 6.0), torque_max)
            await _hold(actuator, _clamp_target(actuator, start_pos), duration_s=settle_s,
                        start_pos=start_pos, runaway_band_rad=runaway_band_rad * 2.0)
        except RunawayAbort:
            try:
                await actuator.enable()
            except Exception:
                pass

    out: dict = {}
    try:
        for dname, sign in dirs:
            target = _clamp_target(actuator, start_pos + sign * abs(step_rad))
            breakaway = None
            per_level: list[dict] = []
            for tq in levels:
                await return_to_start()
                await actuator.write_gains(kp_test, 0.0, kd_test, tq)
                try:
                    s = await _hold(actuator, target, duration_s=probe_s,
                                    start_pos=start_pos, runaway_band_rad=runaway_band_rad)
                    moved = max((abs(x["position"] - start_pos) for x in s), default=0.0)
                    cur = max((abs(x["current"]) for x in s), default=0.0)
                except RunawayAbort:
                    moved, cur = runaway_band_rad, None
                    try:
                        await actuator.enable()
                    except Exception:
                        pass
                per_level.append({"torque": tq, "moved_rad": round(moved, 4),
                                  "max_current_a": round(cur, 3) if cur is not None else None})
                if moved >= move_threshold_rad:
                    breakaway = tq
                    break
            out[dname] = {"breakaway_torque": breakaway, "levels": per_level}
        await return_to_start()
    finally:
        await _restore(actuator, kp0, ki0, kd0, tl0, start_pos)

    # Analysis
    b = {d: out[d]["breakaway_torque"] for d in out}
    found = [v for v in b.values() if v is not None]
    analysis: dict = {"config_torque_limit": tl0}
    if len(found) == 2:
        up, down = max(found), min(found)
        analysis.update({
            "breakaway_uphill": up,
            "breakaway_downhill": down,
            "gravity_torque_est": round((up - down) / 2.0, 3),
            "friction_torque_est": round((up + down) / 2.0, 3),
        })
    working = max(found) if found else None      # worst-case torque needed to move
    analysis["working_torque"] = working
    if working is not None:
        rec = round(working * 1.4, 1)             # +40% headroom for control
        analysis["recommended_torque_limit"] = rec
        analysis["torque_limit_marginal"] = working > 0.85 * tl0
    else:
        analysis["recommended_torque_limit"] = None
        analysis["never_moved"] = True            # needs more torque than tested / binding

    return {
        "directions": out,
        "analysis": analysis,
        "torque_levels": levels,
        "kp_test": round(kp_test, 1),
        "kd_test": round(kd_test, 2),
    }


async def _lift_drop_ki(actuator, kp, ki, kd, hold_point, drop_point, hold_s,
                        start_pos, runaway_band_rad, torque_limit):
    await actuator.write_gains(kp, ki, kd, torque_limit)
    await _hold(actuator, hold_point, duration_s=hold_s,
                start_pos=start_pos, runaway_band_rad=runaway_band_rad)
    s_drop = await _hold(actuator, drop_point, duration_s=hold_s,
                         start_pos=start_pos, runaway_band_rad=runaway_band_rad)
    dm = _descent_metrics(s_drop, drop_point)
    return {
        "kd": round(kd, 3), "ki": round(ki, 3),
        "descent_peak_velocity": dm["peak_velocity"],
        "descent_overshoot_rad": dm["overshoot_rad"],
    }
