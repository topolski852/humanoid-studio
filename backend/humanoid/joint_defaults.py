"""
Default per-joint software position limits for the Berkeley Humanoid Lite.

Source: Berkeley Humanoid Lite docs — Joint ID Mapping
https://berkeley-humanoid-lite.gitbook.io/docs/in-depth-contents/joint-id-mapping

Ranges below are the official joint position limits in DEGREES (joint frame);
they are converted to radians at import time. These are applied as DEFAULTS to
any joint whose position_limits are unset (min and max both null) — e.g. a motor
that has just been commissioned via the Flash Wizard. Joints that already carry
explicit limits are never overridden.

Keyed by the joint_name used in configs/humanoid_lite.json. Note the docs call
the CAN-id-9/10 arm joint "elbow_yaw"; this config names it "wrist_yaw".
"""
from __future__ import annotations

import logging
import math

_log = logging.getLogger(__name__)

# Official limits in degrees (lower, upper), joint frame.
_LIMITS_DEG: dict[str, tuple[float, float]] = {
    # ── Left arm ──────────────────────────────────────────────
    "left_shoulder_pitch_joint": (-90.0, 45.0),
    "left_shoulder_roll_joint":  (-90.0, 0.0),
    "left_shoulder_yaw_joint":   (-45.0, 45.0),
    "left_elbow_pitch_joint":    (-90.0, 0.0),
    "left_wrist_yaw_joint":      (-45.0, 45.0),   # docs: "elbow_yaw" (CAN id 9)
    # ── Right arm ─────────────────────────────────────────────
    "right_shoulder_pitch_joint": (-90.0, 45.0),
    "right_shoulder_roll_joint":  (0.0, 90.0),    # mirrored from left
    "right_shoulder_yaw_joint":   (-45.0, 45.0),
    "right_elbow_pitch_joint":    (-90.0, 0.0),
    "right_wrist_yaw_joint":      (-45.0, 45.0),
    # ── Left leg ──────────────────────────────────────────────
    "left_hip_roll_joint":    (-10.0, 90.0),
    "left_hip_yaw_joint":     (-56.25, 33.75),
    "left_hip_pitch_joint":   (-108.75, 56.25),
    "left_knee_pitch_joint":  (0.0, 140.0),
    "left_ankle_pitch_joint": (-45.0, 45.0),
    "left_ankle_roll_joint":  (-15.0, 15.0),
    # ── Right leg ─────────────────────────────────────────────
    "right_hip_roll_joint":    (-10.0, 90.0),
    "right_hip_yaw_joint":     (-33.75, 56.25),   # mirrored from left
    "right_hip_pitch_joint":   (-108.75, 56.25),
    "right_knee_pitch_joint":  (0.0, 140.0),
    "right_ankle_pitch_joint": (-45.0, 45.0),
    "right_ankle_roll_joint":  (-15.0, 15.0),
}

# Public table in radians, keyed by joint_name.
JOINT_DEFAULT_POSITION_LIMITS: dict[str, tuple[float, float]] = {
    name: (math.radians(lo), math.radians(hi))
    for name, (lo, hi) in _LIMITS_DEG.items()
}


def default_position_limits(joint_name: str) -> tuple[float, float] | None:
    """Return (min_rad, max_rad) defaults for a joint, or None if unknown."""
    return JOINT_DEFAULT_POSITION_LIMITS.get(joint_name)


def apply_default_limits(config) -> list[str]:
    """Fill in default position limits for any joint that has none.

    A joint is considered "unset" only when BOTH min and max are None — joints
    with any explicit limit are left untouched. Mutates ``config`` in place and
    returns the list of joint names that were filled (for logging/UX).
    """
    filled: list[str] = []
    for name, joint in config.joints.items():
        limits = joint.position_limits
        if limits.min is None and limits.max is None:
            default = JOINT_DEFAULT_POSITION_LIMITS.get(name)
            if default is None:
                continue
            limits.min, limits.max = default
            filled.append(name)
    if filled:
        _log.info("Applied default position limits to %d joint(s): %s",
                  len(filled), ", ".join(sorted(filled)))
    return filled
