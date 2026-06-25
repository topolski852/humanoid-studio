"""
Pydantic configuration models for the Berkeley Humanoid Lite robot.

Schema is the single source of truth shared by the firmware parameter map,
the JSON config file, and the FastAPI response models.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Iterator

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_serializer


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _coerce_float_or_none(v: Any) -> float:
    """Accept None → ±inf, strings, or plain numbers.
    NaN is used as a sentinel here because field_validator cannot return None for a float
    field directly — the _coerce validator below converts NaN back to None.
    """
    if v is None:
        return float("nan")  # sentinel; _coerce() below converts it back to None
    if isinstance(v, str):
        low = v.strip().lower()
        if low in ("inf", "+inf", "infinity"):
            return float("inf")
        if low in ("-inf", "-infinity"):
            return float("-inf")
        return float(low)
    return float(v)


def _serialize_float(v: float) -> float | None:
    """Serialize ±inf as None for JSON compatibility."""
    if math.isinf(v):
        return None
    return v


# ---------------------------------------------------------------------------
# Sub-models
# ---------------------------------------------------------------------------

_NO_LIMIT_RAD = 100.0  # ~±5730° — large enough to be practically unlimited for any joint


class PositionLimits(BaseModel):
    """
    Software position limits in radians.
    None serializes as JSON null and means "no limit".
    We send ±_NO_LIMIT_RAD to the firmware instead of ±inf: the STM32 position
    controller performs (upper + lower) / 2 which gives NaN with ±inf inputs.
    """
    model_config = ConfigDict(arbitrary_types_allowed=True)

    min: float | None = None
    max: float | None = None

    @field_validator("min", "max", mode="before")
    @classmethod
    def _coerce(cls, v: Any) -> float | None:
        if v is None:
            return None
        f = _coerce_float_or_none(v)
        if math.isnan(f):
            return None
        return f

    @property
    def lower_bound(self) -> float:
        return self.min if self.min is not None else -_NO_LIMIT_RAD

    @property
    def upper_bound(self) -> float:
        return self.max if self.max is not None else _NO_LIMIT_RAD

    def contains(self, value: float) -> bool:
        return self.lower_bound <= value <= self.upper_bound


# ---------------------------------------------------------------------------
# Per-joint configuration
# ---------------------------------------------------------------------------

class JointConfig(BaseModel):
    """Complete configuration for a single actuator joint."""

    # ------------------------------------------------------------------
    # Identity  (required by spec)
    # ------------------------------------------------------------------
    joint_name: str
    joint_type: str = "revolute"
    can_channel: str = "can0"
    can_id: int = Field(ge=1, le=63)

    # ------------------------------------------------------------------
    # Direction & calibration  (required by spec)
    # ------------------------------------------------------------------
    phase_inverted: bool = False
    electrical_offset: float = 0.0        # encoder flux_offset (rad, large range OK)
    gear_ratio: float = 1.0               # signed; negative flips position direction
    position_limits: PositionLimits = Field(default_factory=PositionLimits)

    # ------------------------------------------------------------------
    # Position controller
    # ------------------------------------------------------------------
    position_kp: float = 20.0
    position_ki: float = 0.0
    velocity_kp: float = 1.0              # acts as Kd in position mode
    velocity_ki: float = 0.0
    torque_limit: float = 2.0             # Nm
    velocity_limit: float = 20.0          # rad/s
    position_offset: float = 0.0          # rad
    torque_filter_alpha: float = 0.1454   # EMA α ≈ 50 Hz at 2 kHz loop

    # ------------------------------------------------------------------
    # Current controller
    # ------------------------------------------------------------------
    current_limit: float = 20.0           # A
    current_kp: float = 0.1664            # V/A
    current_ki: float = 5746.5            # 1/s

    # ------------------------------------------------------------------
    # Power stage
    # ------------------------------------------------------------------
    undervoltage_threshold: float = 0.0   # V; 0 = disabled
    overvoltage_threshold: float = 0.0    # V; 0 = disabled
    bus_voltage_filter_alpha: float = 0.2696

    # ------------------------------------------------------------------
    # Motor
    # ------------------------------------------------------------------
    pole_pairs: int = 14
    torque_constant: float = 0.0659       # Nm/A
    max_calibration_current: float = 3.0  # A

    # ------------------------------------------------------------------
    # Encoder (AS5600)
    # ------------------------------------------------------------------
    cpr: int = 4096
    encoder_position_offset: float = 0.0  # rad
    velocity_filter_alpha: float = 0.7154  # EMA α ≈ 2 kHz at 10 kHz loop

    # ------------------------------------------------------------------
    # System
    # ------------------------------------------------------------------
    watchdog_timeout: int = 1000          # ms
    fast_frame_frequency: int = 0         # Hz; 0 = disabled

    @field_validator('watchdog_timeout', 'fast_frame_frequency', 'pole_pairs', 'cpr', mode='before')
    @classmethod
    def _coerce_int_fields(cls, v: Any) -> Any:
        # Device READ_CONFIG returns integer SDO params as their f32 bit-patterns
        # (e.g. 1000 ms stored as firmware ticks → tiny denormalized float).
        # Accept ints directly; round floats only if they're exact integers.
        if isinstance(v, float):
            rounded = round(v)
            if abs(v - rounded) < 1e-6:
                return rounded
            # Non-integer float — return 0 rather than crashing the whole config load.
            return 0
        return v

    # ------------------------------------------------------------------
    # Derived properties (not persisted)
    # ------------------------------------------------------------------
    @property
    def phase_order(self) -> int:
        """Map phase_inverted bool → firmware int (+1 / -1)."""
        return -1 if self.phase_inverted else 1

    def with_electrical_offset(self, offset: float) -> "JointConfig":
        """Return a copy with the calibrated electrical offset applied."""
        return self.model_copy(update={"electrical_offset": offset})


# ---------------------------------------------------------------------------
# Top-level robot configuration
# ---------------------------------------------------------------------------

class RobotConfig(BaseModel):
    """Complete robot configuration — maps joint_name → JointConfig."""

    robot_name: str = "humanoid_lite"
    telemetry_hz: int = 10  # SDO poll rate for ws_telemetry; default 10 Hz
    # USB serial → limb name ("left_leg", "right_arm", …).  Written by can_adapter.
    # Must survive to_json() so motor config saves don't clear CAN assignments.
    can_assignments: dict[str, str] = Field(default_factory=dict)
    joints: dict[str, JointConfig] = Field(default_factory=dict)

    def __getitem__(self, joint_name: str) -> JointConfig:
        return self.joints[joint_name]

    def __iter__(self) -> Iterator[tuple[str, JointConfig]]:  # type: ignore[override]
        return iter(self.joints.items())

    def joint_names(self) -> list[str]:
        return list(self.joints.keys())

    def channels(self) -> set[str]:
        """Return the set of unique CAN channels used by the robot."""
        return {j.can_channel for j in self.joints.values()}

    def joints_on_channel(self, channel: str) -> dict[str, JointConfig]:
        return {name: j for name, j in self.joints.items() if j.can_channel == channel}

    # ------------------------------------------------------------------
    # Serialization with None-for-infinity
    # ------------------------------------------------------------------
    def to_dict(self) -> dict:
        raw = self.model_dump()
        for joint in raw["joints"].values():
            limits = joint.get("position_limits", {})
            if limits:
                min_v = limits.get("min")
                max_v = limits.get("max")
                if min_v is not None and math.isinf(min_v):
                    limits["min"] = None
                if max_v is not None and math.isinf(max_v):
                    limits["max"] = None
        return raw

    def to_json(self, path: str | Path, indent: int = 2) -> None:
        with open(path, "w") as f:
            json.dump(self.to_dict(), f, indent=indent)

    @classmethod
    def from_json(cls, path: str | Path) -> "RobotConfig":
        with open(path) as f:
            data = json.load(f)
        return cls.model_validate(data)

    @classmethod
    def empty(cls, robot_name: str = "unnamed") -> "RobotConfig":
        return cls(robot_name=robot_name, joints={})

    def add_joint(self, config: JointConfig) -> None:
        self.joints[config.joint_name] = config

    def remove_joint(self, name: str) -> None:
        self.joints.pop(name, None)
