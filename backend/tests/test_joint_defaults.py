"""Self-contained tests for the Berkeley Humanoid Lite default joint limits.

Runs without pytest:  python3 tests/test_joint_defaults.py
(also pytest-compatible: functions are named test_*).
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from humanoid.robot_config import RobotConfig, JointConfig, PositionLimits
from humanoid.joint_defaults import (
    JOINT_DEFAULT_POSITION_LIMITS,
    apply_default_limits,
    default_position_limits,
)


def test_table_covers_all_22_joints():
    assert len(JOINT_DEFAULT_POSITION_LIMITS) == 22, len(JOINT_DEFAULT_POSITION_LIMITS)
    # spot-check a few known values (degrees -> radians)
    knee = JOINT_DEFAULT_POSITION_LIMITS["left_knee_pitch_joint"]
    assert abs(knee[0] - 0.0) < 1e-9 and abs(knee[1] - math.radians(140)) < 1e-9
    roll = JOINT_DEFAULT_POSITION_LIMITS["left_ankle_roll_joint"]
    assert abs(roll[0] - math.radians(-15)) < 1e-9 and abs(roll[1] - math.radians(15)) < 1e-9
    print("ok test_table_covers_all_22_joints")


def test_lookup_known_and_unknown():
    assert default_position_limits("right_hip_pitch_joint") is not None
    assert default_position_limits("not_a_joint") is None
    print("ok test_lookup_known_and_unknown")


def test_apply_fills_null_only():
    cfg = RobotConfig(robot_name="t", joints={
        "left_knee_pitch_joint": JointConfig(joint_name="left_knee_pitch_joint", can_id=7),
        "left_hip_roll_joint": JointConfig(
            joint_name="left_hip_roll_joint", can_id=1,
            position_limits=PositionLimits(min=-0.1, max=0.2)),
        "mystery_joint": JointConfig(joint_name="mystery_joint", can_id=9),
    })
    filled = apply_default_limits(cfg)
    assert filled == ["left_knee_pitch_joint"], filled
    k = cfg.joints["left_knee_pitch_joint"].position_limits
    assert abs(k.min - 0.0) < 1e-9 and abs(k.max - math.radians(140)) < 1e-9
    # explicit limits are never overridden
    h = cfg.joints["left_hip_roll_joint"].position_limits
    assert h.min == -0.1 and h.max == 0.2
    # unknown joint left null
    m = cfg.joints["mystery_joint"].position_limits
    assert m.min is None and m.max is None
    print("ok test_apply_fills_null_only")


def test_apply_partial_limit_not_overridden():
    # a joint with only one bound set is considered "explicit" and left alone
    cfg = RobotConfig(robot_name="t", joints={
        "left_ankle_pitch_joint": JointConfig(
            joint_name="left_ankle_pitch_joint", can_id=11,
            position_limits=PositionLimits(min=None, max=0.5)),
    })
    filled = apply_default_limits(cfg)
    assert filled == [], filled
    lim = cfg.joints["left_ankle_pitch_joint"].position_limits
    assert lim.min is None and lim.max == 0.5
    print("ok test_apply_partial_limit_not_overridden")


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for fn in fns:
        fn()
    print(f"\nAll {len(fns)} tests passed.")
