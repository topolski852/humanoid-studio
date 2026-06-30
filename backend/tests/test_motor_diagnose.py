"""Self-contained tests for the diagnostic auto-tuner.

Runs without pytest:  python3 tests/test_motor_diagnose.py
(also pytest-compatible: functions are named test_*).

Covers the pure sample-analysis helpers, the run_diagnosis() state machine via a
scripted fake actuator, and a run_gravity_tune() smoke test (Kd-knee selection).
"""
import asyncio
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from humanoid.actuator import ActuatorState
from humanoid.robot_config import JointConfig, PositionLimits
from humanoid.motor_tune import (
    _velocity_reversals, _torque_chatter_pp, _current_motion_ratio,
    _hold_error, _descent_metrics, _motion_range,
)
from humanoid import motor_diagnose
from humanoid.motor_diagnose import run_diagnosis, run_gravity_tune, commutation_probe

_MODE_POSITION = 0x13


# ---------------------------------------------------------------------------
# Pure helpers
# ---------------------------------------------------------------------------

def _s(pos=None, vel=None, tq=None, cur=None):
    return {"position": pos, "velocity": vel, "torque": tq, "current": cur}


def test_velocity_reversals():
    # +,+,-,+  with motion > eps on each → two reversals (+→-, -→+)
    samples = [_s(vel=0.5), _s(vel=0.6), _s(vel=-0.4), _s(vel=0.3)]
    assert _velocity_reversals(samples, vel_eps=0.05) == 2
    # tiny jitter below eps must NOT count
    assert _velocity_reversals([_s(vel=0.01), _s(vel=-0.02)], vel_eps=0.05) == 0
    print("ok test_velocity_reversals")


def test_current_motion_ratio():
    # high current, frozen position -> inf ratio (commutation fault signature)
    stuck = [_s(pos=1.0, cur=4.0), _s(pos=1.0001, cur=5.0)]
    cm = _current_motion_ratio(stuck)
    assert cm["max_current_a"] == 5.0
    assert cm["motion_range_rad"] < 0.05
    assert cm["current_per_rad"] == float("inf")
    # moving: finite ratio
    moving = [_s(pos=0.0, cur=1.0), _s(pos=0.5, cur=1.0)]
    assert _current_motion_ratio(moving)["current_per_rad"] < 10
    print("ok test_current_motion_ratio")


def test_hold_error_and_descent():
    # holds at 0.9 while commanded 1.0 -> droop 0.1
    hold = [_s(pos=0.9)] * 10
    assert abs(_hold_error(hold, 1.0) - 0.1) < 1e-6
    # descend to 0.0, dips to -0.05 -> overshoot 0.05, peak vel 2.0
    drop = [_s(pos=0.5, vel=-2.0), _s(pos=0.0, vel=-0.5), _s(pos=-0.05, vel=0.0)]
    dm = _descent_metrics(drop, 0.0)
    assert dm["peak_velocity"] == 2.0
    assert abs(dm["overshoot_rad"] - 0.05) < 1e-6
    print("ok test_hold_error_and_descent")


def test_chatter_and_motion_range():
    assert _torque_chatter_pp([_s(tq=-1.0), _s(tq=2.0)]) == 3.0
    assert abs(_motion_range([_s(pos=0.1), _s(pos=0.4)]) - 0.3) < 1e-9
    print("ok test_chatter_and_motion_range")


# ---------------------------------------------------------------------------
# Fake actuator for the diagnosis state machine
# ---------------------------------------------------------------------------

class FakeActuator:
    """Scripts get_cached_state() responses per scenario so run_diagnosis can be
    driven deterministically and fast (small hold/ramp times)."""

    def __init__(self, scenario, start_pos=0.0, gravity_descent=None):
        self.scenario = scenario
        self.start_pos = start_pos
        self._pos = start_pos
        self._target = start_pos
        self._kd = 1.0
        self._tl = 6.0
        self._gravity_descent = gravity_descent or {}
        self._config = JointConfig(
            joint_name="fake", can_id=1, can_channel="can0",
            position_kp=20.0, position_ki=0.0, velocity_kp=2.0, torque_limit=6.0,
            position_limits=PositionLimits(min=-2.0, max=2.0),
        )

    # -- proxy surface used by motor_diagnose --
    @property
    def config(self):
        return self._config

    def update_config(self, cfg):
        self._config = cfg

    async def write_gains(self, kp, ki, kd, tl):
        self._kd, self._tl = kd, tl

    async def disable(self):
        pass

    async def enable(self, mode=None):
        pass

    async def set_position(self, target, *a, **k):
        self._target = target
        sc = self.scenario
        if sc == "moves":
            self._pos += 0.5 * (target - self._pos)          # tracks the command
        elif sc == "gravity":
            # lift (target above start) reaches with a fixed droop; drop returns to start
            if target > self.start_pos + 0.01:
                self._pos += 0.6 * ((target - 0.1) - self._pos)
            else:
                self._pos += 0.6 * (target - self._pos)
        # stuck scenarios (kp_starved/torque_starved/commutation): position frozen
        return None

    def get_cached_state(self):
        sc = self.scenario
        pos, vel, tq, cur = self._pos, 0.0, 0.0, 0.0
        if sc == "moves":
            vel = (self._target - self._pos) * 5.0
            tq, cur = 1.0, 0.6
        elif sc == "kp_starved":
            tq, cur = 1.0, 0.8                                # low current, not saturated
        elif sc == "torque_starved":
            tq, cur = self._tl, 5.0                           # torque pinned at the diag limit
        elif sc == "commutation":
            tq, cur = 0.4, 5.0                                # high current, frozen, no torque
        elif sc == "gravity":
            descending = self._target <= self.start_pos + 0.01
            vel = -self._gravity_descent.get(round(self._kd, 3), 1.0) if descending else 0.0
            tq, cur = 1.5, 1.0
        return ActuatorState(position=pos, velocity=vel, torque=tq, current=cur,
                             mode=_MODE_POSITION, error=0, bus_voltage=19.7)


async def _diagnose(scenario):
    act = FakeActuator(scenario)
    # tiny hold/ramp times keep the test ~instant
    return await run_diagnosis(act, hold_s=0.08, ramp_s=0.08, step_rad=0.15,
                               ramp_rad=0.25, current_fault_a=3.0)


def test_diagnosis_moves_ok():
    r = asyncio.run(_diagnose("moves"))
    assert r["classification"] == "MOVES_OK", r["classification"]
    assert r["recommendation"]["action"] == "none"
    print("ok test_diagnosis_moves_ok")


def test_diagnosis_kp_starved():
    r = asyncio.run(_diagnose("kp_starved"))
    assert r["classification"] == "KP_STARVED", r["classification"]
    assert r["recommendation"]["action"] == "raise_kp"
    assert r["recommendation"]["params"]["position_kp"] > 20.0
    print("ok test_diagnosis_kp_starved")


def test_diagnosis_torque_starved():
    r = asyncio.run(_diagnose("torque_starved"))
    assert r["classification"] == "STUCK_TORQUE_STARVED", r["classification"]
    assert r["recommendation"]["action"] == "raise_torque_limit"
    print("ok test_diagnosis_torque_starved")


def test_diagnosis_commutation_fault():
    r = asyncio.run(_diagnose("commutation"))
    assert r["classification"] == "COMMUTATION_FAULT", r["classification"]
    assert r["recommendation"]["action"] == "phase_flip_and_recal"
    assert r["recommendation"]["destructive"] is True
    print("ok test_diagnosis_commutation_fault")


def test_gravity_tune_knee():
    # descent velocity plateaus after Kd=2 -> the knee picker should select Kd=2
    descent = {0.5: 2.0, 1.0: 1.2, 2.0: 0.95, 4.0: 0.92, 8.0: 0.90}
    act = FakeActuator("gravity", gravity_descent=descent)
    r = asyncio.run(run_gravity_tune(
        act, kp=20.0, kd_values=[0.5, 1, 2, 4, 8], lift_rad=0.3, lift_sign=1.0,
        torque_limit=6.0, hold_s=0.08,
    ))
    assert len(r["sweep"]) == 5
    assert r["selected_kd"] == 2.0, r["selected_kd"]
    assert r["recommended"]["ki"] == 0.0
    assert r["recommended"]["kp"] >= 20.0
    print("ok test_gravity_tune_knee  (selected_kd=%s, rec=%s)" % (r["selected_kd"], r["recommended"]))


class _ProbeFake:
    """Minimal proxy whose reported current models torque saturation, so the
    commutation_probe()'s stuck-while-saturated logic can be exercised."""
    def __init__(self, *, moves, current, kt=0.0896, gear=15.0, tl=6.0):
        self._moves, self._current, self._pos, self._target = moves, current, 0.0, 0.0
        self._config = JointConfig(
            joint_name="f", can_id=1, can_channel="can0",
            torque_constant=kt, gear_ratio=gear, torque_limit=tl,
            position_kp=20.0, velocity_kp=2.0,
            position_limits=PositionLimits(min=-2.0, max=2.0))

    @property
    def config(self): return self._config
    def update_config(self, c): self._config = c
    async def write_gains(self, kp, ki, kd, tl): pass
    async def enable(self, mode=None): pass
    async def disable(self): pass
    async def set_position(self, t, *a, **k):
        self._target = t
        if self._moves:
            self._pos += 0.5 * (t - self._pos)
        return None
    def get_cached_state(self):
        return ActuatorState(position=self._pos, velocity=0.0, torque=0.0,
                             current=self._current, mode=_MODE_POSITION,
                             error=0, bus_voltage=19.7)


def test_commutation_probe_healthy():
    # moves freely with low current -> commutates
    r = asyncio.run(commutation_probe(_ProbeFake(moves=True, current=0.6), move_s=0.08))
    assert r["commutates"] is True, r
    print("ok test_commutation_probe_healthy")


def test_commutation_probe_fault():
    # stuck while drawing the (reduced) saturation current -> commutation fault.
    # probe_torque_limit = 0.6*6 = 3.6 ; sat_current = 3.6/0.0896/15 = 2.68 A
    r = asyncio.run(commutation_probe(_ProbeFake(moves=False, current=2.68), move_s=0.08))
    assert r["commutates"] is False, r
    assert r["saturated"] is True, r
    print("ok test_commutation_probe_fault")


def test_commutation_probe_kp_starved_not_fault():
    # stuck but LOW current (Kp too low) is NOT a commutation fault
    r = asyncio.run(commutation_probe(_ProbeFake(moves=False, current=0.5), move_s=0.08))
    assert r["commutates"] is True, r
    print("ok test_commutation_probe_kp_starved_not_fault")


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for fn in fns:
        fn()
    print(f"\nAll {len(fns)} tests passed.")
