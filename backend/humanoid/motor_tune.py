"""Step-response PID auto-tuner for Recoil motor controllers."""
from __future__ import annotations

import asyncio
import time

from humanoid.daemon_client import DaemonActuatorProxy

# SDO parameter addresses (from recoil_protocol.hpp ParamId enum)
_PARAM_POSITION_KP  = 0x020
_PARAM_POSITION_KI  = 0x024
_PARAM_VELOCITY_KP  = 0x028
_PARAM_TORQUE_LIMIT = 0x030

# Firmware POSITION mode value (Mode.POSITION = 0x13)
_MODE_POSITION = 0x13


async def run_step_test(
    actuator: DaemonActuatorProxy,
    position_kp: float,
    position_ki: float,
    velocity_kp: float,
    torque_limit: float,
    center_rad: float,
    offset_rad: float = 0.45,
    step_hold_s: float = 1.5,
    num_steps: int = 4,
) -> dict:
    """
    Run a step-response test by commanding the motor between two positions.

    The motor steps between (center_rad - offset_rad) and (center_rad + offset_rad),
    dwelling at each position for step_hold_s seconds.  The test gains are written
    to the device via SDO before the motion starts.

    Returns:
        dict with keys "samples" (list of per-sample dicts) and "metrics".
    """
    offset_rad  = max(0.05, min(1.5,  float(offset_rad)))
    step_hold_s = max(0.3,  min(5.0,  float(step_hold_s)))
    num_steps   = max(1,    min(10,   int(num_steps)))

    # Motor must be in POSITION mode.
    state = actuator.get_cached_state()
    if state is None:
        raise ValueError("Motor is OFFLINE — cannot run step test")
    if state.mode != _MODE_POSITION:
        raise ValueError(
            f"Motor must be in POSITION mode to run step test "
            f"(current mode 0x{state.mode:02X})"
        )

    # Clip step range to stay within position limits so the motor doesn't
    # oscillate against a hard limit during the test.
    lim = actuator.config.position_limits
    lim_lo = lim.lower_bound
    lim_hi = lim.upper_bound
    margin = 0.02  # 2 rad safety margin from soft limits
    pos_a = max(center_rad - offset_rad, lim_lo + margin)
    pos_b = min(center_rad + offset_rad, lim_hi - margin)
    if pos_b - pos_a < 0.08:
        raise ValueError(
            f"Step range [{pos_a:.3f}, {pos_b:.3f}] rad is too small after clipping to "
            f"position limits [{lim_lo:.3f}, {lim_hi:.3f}]. "
            f"Move the motor closer to the center of its range before tuning."
        )
    # Recompute symmetric offset from clipped range.
    center_rad = (pos_a + pos_b) / 2.0
    offset_rad = (pos_b - pos_a) / 2.0

    # Write test gains via SDO before motion starts.
    for param, value in [
        (_PARAM_POSITION_KP,  position_kp),
        (_PARAM_POSITION_KI,  position_ki),
        (_PARAM_VELOCITY_KP,  velocity_kp),
        (_PARAM_TORQUE_LIMIT, torque_limit),
    ]:
        await actuator.sdo_write_f32(param, value)

    # Pre-settle at pos_a before sampling starts.
    await actuator.set_position(pos_a)
    await asyncio.sleep(step_hold_s)

    samples: list[dict] = []
    t_start = time.monotonic()

    for step_idx in range(num_steps):
        target = pos_b if (step_idx % 2 == 0) else pos_a
        await actuator.set_position(target)

        step_end = time.monotonic() + step_hold_s
        while time.monotonic() < step_end:
            s = actuator.get_cached_state()
            t_ms = int((time.monotonic() - t_start) * 1000)
            if s is not None:
                samples.append({
                    "t_ms":       t_ms,
                    "commanded":  target,
                    "position":   s.position,
                    "velocity":   s.velocity,
                    "torque":     s.torque,
                    "current":    s.current,
                    "bus_voltage": s.bus_voltage,
                    "step_index": step_idx,
                })
            await asyncio.sleep(0.01)

    # Return to center when done.
    await actuator.set_position(center_rad)

    return {
        "samples": samples,
        "metrics": _compute_metrics(samples, torque_limit, offset_rad),
        "params_used": {
            "center_rad": center_rad,
            "offset_rad": offset_rad,
            "pos_a": pos_a,
            "pos_b": pos_b,
        },
    }


def _compute_metrics(
    samples: list[dict],
    torque_limit: float,
    offset_rad: float,
) -> dict:
    empty = {
        "max_overshoot_rad": 0.0,
        "max_overshoot_pct": 0.0,
        "settling_time_ms": None,
        "steady_state_error_rad": None,
        "max_torque_nm": 0.0,
        "torque_saturated": False,
        "max_current_a": 0.0,
        "no_motion_detected": False,
    }
    if not samples:
        return empty

    # Check whether the motor moved at all across the entire test.
    all_positions = [s["position"] for s in samples if s.get("position") is not None]
    motion_range = (max(all_positions) - min(all_positions)) if len(all_positions) >= 2 else 0.0
    # Flag as no-motion if range is < 15% of the expected step size (2 × offset).
    no_motion_detected = motion_range < offset_rad * 0.30

    steps: dict[int, list[dict]] = {}
    for s in samples:
        steps.setdefault(s["step_index"], []).append(s)

    max_overshoot_rad = 0.0
    max_torque_nm     = 0.0
    max_current_a     = 0.0
    torque_saturated  = False
    settling_times: list[float] = []
    ss_errors: list[float]      = []
    threshold = offset_rad * 0.02  # 2% settling band

    for step_idx, step_samples in steps.items():
        if not step_samples:
            continue
        target   = step_samples[0]["commanded"]
        going_up = (step_idx % 2 == 0)   # step_idx 0 → pos_b (up); 1 → pos_a (down)
        t0_ms    = step_samples[0]["t_ms"]
        last_outside_idx = None

        # Detect degenerate steps where the motor was already at the target at t=0
        # (i.e. it never had to travel there). These can arise when the motor is stuck
        # and alternating steps happen to land on the motor's frozen position.
        first_pos = next((s["position"] for s in step_samples if s.get("position") is not None), None)
        pre_settled = first_pos is not None and abs(first_pos - target) <= threshold

        for i, s in enumerate(step_samples):
            pos = s.get("position")
            if pos is None:
                continue

            # Overshoot past the target in the direction of motion.
            overshoot = max(0.0, pos - target) if going_up else max(0.0, target - pos)
            max_overshoot_rad = max(max_overshoot_rad, overshoot)

            # Track last sample outside the settling band (for accurate settling time).
            if abs(pos - target) > threshold:
                last_outside_idx = i

            torque = s.get("torque")
            if torque is not None:
                t_abs = abs(torque)
                max_torque_nm = max(max_torque_nm, t_abs)
                if t_abs >= torque_limit * 0.95:
                    torque_saturated = True

            current = s.get("current")
            if current is not None:
                max_current_a = max(max_current_a, abs(current))

        # Settling time = time after which the motor stays continuously in the ±2% band.
        dwell_ms = float(step_samples[-1]["t_ms"] - t0_ms) if len(step_samples) > 1 else 0.0
        if pre_settled:
            # Motor was already at the target — not meaningful settling; skip this step.
            pass
        elif last_outside_idx is None:
            # Every sample was within the band from the start (genuine fast settling).
            settling_times.append(0.0)
        elif last_outside_idx < len(step_samples) - 1:
            settling_times.append(float(step_samples[last_outside_idx + 1]["t_ms"] - t0_ms))
        else:
            # Never settled within the dwell period — count as dwell_ms so the average
            # reflects the failure rather than silently omitting this step.
            settling_times.append(dwell_ms)

        # Steady-state error: mean over the last 20% of dwell samples.
        n_tail     = max(1, len(step_samples) // 5)
        valid_tail = [s for s in step_samples[-n_tail:] if s.get("position") is not None]
        if valid_tail and not pre_settled:
            ss_errors.append(
                sum(abs(s["position"] - target) for s in valid_tail) / len(valid_tail)
            )

    return {
        "max_overshoot_rad": round(max_overshoot_rad, 4),
        "max_overshoot_pct": round(max_overshoot_rad / (2 * offset_rad) * 100, 1) if offset_rad > 0 else 0.0,
        "settling_time_ms":  round(sum(settling_times) / len(settling_times)) if settling_times else None,
        "steady_state_error_rad": round(sum(ss_errors) / len(ss_errors), 4) if ss_errors else None,
        "max_torque_nm":    round(max_torque_nm, 3),
        "torque_saturated": torque_saturated,
        "max_current_a":    round(max_current_a, 3),
        "no_motion_detected": no_motion_detected,
    }


# ---------------------------------------------------------------------------
# Shared sample-analysis helpers (used by motor_tune AND motor_diagnose)
#
# Each operates on a list of sample dicts shaped like run_step_test() samples:
#   {"position": float, "velocity": float, "torque": float, "current": float, ...}
# Missing/None fields are skipped so partial telemetry never raises.
# ---------------------------------------------------------------------------

def _values(samples: list[dict], key: str) -> list[float]:
    return [s[key] for s in samples if s.get(key) is not None]


def _motion_range(samples: list[dict]) -> float:
    """Peak-to-peak position travel over the window (rad)."""
    ps = _values(samples, "position")
    return (max(ps) - min(ps)) if len(ps) >= 2 else 0.0


def _velocity_reversals(samples: list[dict], vel_eps: float = 0.05) -> int:
    """Count velocity sign changes where at least one side is real motion (> vel_eps).

    A high count with little net travel is the signature of stick-slip / limit-cycle
    chatter rather than smooth tracking.
    """
    vs = _values(samples, "velocity")
    return sum(
        1 for a, b in zip(vs, vs[1:])
        if a * b < 0 and (abs(a) > vel_eps or abs(b) > vel_eps)
    )


def _torque_chatter_pp(samples: list[dict]) -> float:
    """Peak-to-peak of measured torque over the window (Nm)."""
    ts = _values(samples, "torque")
    return round(max(ts) - min(ts), 3) if ts else 0.0


def _current_motion_ratio(samples: list[dict], motion_eps: float = 1e-4) -> dict:
    """Current drawn per radian of travel.

    High max_current with near-zero motion_range = "fictitious torque": the current
    loop is driving phase current but the rotor isn't turning — a closed-loop
    commutation/phase fault (the hip_roll case).  current_per_rad is inf when stuck.
    """
    cs = [abs(c) for c in _values(samples, "current")]
    max_cur = max(cs) if cs else 0.0
    motion = _motion_range(samples)
    ratio = (max_cur / motion) if motion > motion_eps else float("inf")
    return {
        "max_current_a":    round(max_cur, 3),
        "motion_range_rad": round(motion, 4),
        "current_per_rad":  ratio,
    }


def _hold_error(samples: list[dict], target: float, tail_frac: float = 0.2) -> float:
    """Signed steady-state hold error (droop): target - mean(position over last tail).

    On a gravity-loaded joint commanded to hold against gravity, this is the sag a
    pure-P loop can't remove (droop ~= gravity_torque / Kp).
    """
    ps = _values(samples, "position")
    if not ps:
        return 0.0
    n = max(1, int(len(ps) * tail_frac))
    mean_tail = sum(ps[-n:]) / n
    return round(target - mean_tail, 4)


def _descent_metrics(samples: list[dict], target: float) -> dict:
    """Peak speed and overshoot past target for a move toward `target`.

    Direction is inferred from the first sample, so it works for both the
    gravity-assisted "drop" and a normal step.  overshoot_rad is the worst
    excursion BEYOND the target in the direction of travel (the "slam").
    """
    ps = _values(samples, "position")
    vs = _values(samples, "velocity")
    if not ps:
        return {"peak_velocity": 0.0, "overshoot_rad": 0.0, "final_pos": None, "final_error": None}
    going_down = target < ps[0]
    peak_v = max((abs(v) for v in vs), default=0.0)
    overshoot = max(0.0, target - min(ps)) if going_down else max(0.0, max(ps) - target)
    return {
        "peak_velocity": round(peak_v, 3),
        "overshoot_rad": round(overshoot, 4),
        "final_pos":     round(ps[-1], 4),
        "final_error":   round(ps[-1] - target, 4),
    }
