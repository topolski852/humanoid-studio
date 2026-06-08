"""
Motor control endpoints.

GET  /motors/{joint_name}
POST /motors/{joint_name}/enable
POST /motors/{joint_name}/disable
POST /motors/{joint_name}/calibrate
POST /motors/{joint_name}/position
POST /motors/{joint_name}/position_offset
GET  /motors/{joint_name}/config_from_device
POST /motors/{joint_name}/apply_config
POST /motors/{joint_name}/store_to_flash
"""
from __future__ import annotations

import logging
from pathlib import Path

from fastapi import APIRouter, Request
from fastapi.responses import JSONResponse
from pydantic import BaseModel

from humanoid.daemon_client import DaemonError, DaemonNotSupportedError, Mode
from humanoid.robot_config import PositionLimits

_DEFAULT_CONFIG_PATH = Path(__file__).parents[3] / "configs" / "humanoid_lite.json"
_log = logging.getLogger(__name__)
_velocity_ff_warned = False

router = APIRouter(tags=["motors"])


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _ok(data: object) -> dict:
    return {"success": True, "data": data, "error": None}


def _err(msg: str, status: int = 400) -> JSONResponse:
    return JSONResponse(
        {"success": False, "data": None, "error": msg},
        status_code=status,
    )


def _resolve_actuator(request: Request, joint_name: str):
    robot = request.app.state.robot
    if robot is None:
        return None, _err("No robot config loaded — PUT /robot/config first", 503)
    if not robot.is_connected():
        return None, _err("Robot not connected — POST /robot/connect first", 503)
    actuator = robot.get_actuator_by_name(joint_name)
    if actuator is None:
        return None, _err(f"No joint named '{joint_name}'", 404)
    return actuator, None


# ---------------------------------------------------------------------------
# Request bodies
# ---------------------------------------------------------------------------

class EnableBody(BaseModel):
    mode: str = "POSITION"   # "POSITION" | "VELOCITY" | "TORQUE" | "CURRENT"


class PositionBody(BaseModel):
    position: float
    velocity_ff: float = 0.0
    torque_ff: float = 0.0


_MODE_MAP = {
    "POSITION": Mode.POSITION,
    "VELOCITY": Mode.VELOCITY,
    "TORQUE":   Mode.TORQUE,
    "CURRENT":  Mode.CURRENT,
    "IDLE":     Mode.IDLE,
    "DAMPING":  Mode.DAMPING,
}


# ---------------------------------------------------------------------------
# Routes
# ---------------------------------------------------------------------------

@router.get("/motors/{joint_name}", response_model=None)
async def get_motor(joint_name: str, request: Request) -> dict | JSONResponse:
    actuator, error = _resolve_actuator(request, joint_name)
    if error:
        return error
    try:
        state = await actuator.get_state()
        return _ok({
            "joint_name": joint_name,
            "can_id": actuator.config.can_id,
            "can_channel": actuator.config.can_channel,
            "state": state.model_dump(),
        })
    except DaemonError as exc:
        return _err(str(exc))


@router.post("/motors/{joint_name}/enable", response_model=None)
async def enable_motor(
    joint_name: str, body: EnableBody, request: Request
) -> dict | JSONResponse:
    actuator, error = _resolve_actuator(request, joint_name)
    if error:
        return error
    mode = _MODE_MAP.get(body.mode.upper(), Mode.POSITION)
    try:
        await actuator.enable(mode=mode)
        return _ok({"joint_name": joint_name, "enabled": True, "mode": body.mode.upper()})
    except DaemonError as exc:
        return _err(str(exc))


@router.post("/motors/{joint_name}/clear_error", response_model=None)
async def clear_motor_error(joint_name: str, request: Request) -> dict | JSONResponse:
    actuator, error = _resolve_actuator(request, joint_name)
    if error:
        return error
    try:
        await actuator.clear_error()
        return _ok({"joint_name": joint_name, "error_cleared": True})
    except DaemonError as exc:
        return _err(str(exc))


@router.post("/motors/{joint_name}/disable", response_model=None)
async def disable_motor(joint_name: str, request: Request) -> dict | JSONResponse:
    actuator, error = _resolve_actuator(request, joint_name)
    if error:
        return error
    try:
        await actuator.disable()
        return _ok({"joint_name": joint_name, "enabled": False})
    except DaemonError as exc:
        return _err(str(exc))


@router.post("/motors/{joint_name}/calibrate", response_model=None)
async def calibrate_motor(joint_name: str, request: Request) -> dict | JSONResponse:
    """
    Triggers flux-offset calibration and waits for it to complete.
    This can take up to 90 seconds — use a long HTTP timeout on the client side.
    The resulting flux_offset is returned and should be persisted to the config.
    """
    actuator, error = _resolve_actuator(request, joint_name)
    if error:
        return error
    try:
        flux_offset = await actuator.calibrate_offset(timeout=90.0)
        return _ok({"joint_name": joint_name, "flux_offset": flux_offset})
    except DaemonNotSupportedError as exc:
        return _err(str(exc), 503)
    except DaemonError as exc:
        return _err(str(exc))


@router.post("/motors/{joint_name}/position", response_model=None)
async def set_motor_position(
    joint_name: str, body: PositionBody, request: Request
) -> dict | JSONResponse:
    global _velocity_ff_warned
    actuator, error = _resolve_actuator(request, joint_name)
    if error:
        return error
    if body.velocity_ff != 0.0 and not _velocity_ff_warned:
        _velocity_ff_warned = True
        _log.warning(
            "velocity_ff is silently discarded in MODE_POSITION — "
            "the firmware control law ignores velocity_target; use torque_ff for feed-forward"
        )
    try:
        result = await actuator.set_position(
            body.position,
            velocity_ff=body.velocity_ff,
            torque_ff=body.torque_ff,
        )
        data: dict = {"joint_name": joint_name, "position_target": body.position}
        if result is not None:
            pos_measured, vel_measured = result
            data["position_measured"] = pos_measured
            data["velocity_measured"] = vel_measured
        return _ok(data)
    except DaemonError as exc:
        return _err(str(exc))


class PositionOffsetBody(BaseModel):
    position_offset: float  # rad, output-shaft


@router.post("/motors/{joint_name}/position_offset", response_model=None)
async def set_position_offset(
    joint_name: str, body: PositionOffsetBody, request: Request
) -> dict | JSONResponse:
    actuator, error = _resolve_actuator(request, joint_name)
    if error:
        return error
    # Daemon owns the CAN bus; update config in memory + persist to JSON.
    # The new position_offset takes effect on the device after daemon restart or APPLY_CONFIG.
    actuator.config.position_offset = body.position_offset
    config_path: Path = getattr(request.app.state, "config_path", _DEFAULT_CONFIG_PATH)
    request.app.state.robot.config.to_json(config_path)
    return _ok({"position_offset": body.position_offset})


# ---------------------------------------------------------------------------
# ESC config read / apply / persist
# ---------------------------------------------------------------------------

@router.get("/motors/{joint_name}/config_from_device", response_model=None)
async def get_motor_config_from_device(joint_name: str, request: Request) -> dict | JSONResponse:
    actuator, error = _resolve_actuator(request, joint_name)
    if error:
        return error
    try:
        raw = await actuator.read_config_from_device()
        return _ok(raw)
    except DaemonNotSupportedError as exc:
        return _err(str(exc), 503)
    except DaemonError as exc:
        return _err(str(exc))


class ApplyConfigBody(BaseModel):
    config: dict  # full merged key-value pairs


@router.post("/motors/{joint_name}/apply_config", response_model=None)
async def apply_motor_config(
    joint_name: str, body: ApplyConfigBody, request: Request
) -> dict | JSONResponse:
    actuator, error = _resolve_actuator(request, joint_name)
    if error:
        return error
    try:
        updated = actuator.config.model_copy(update=body.config)
        actuator.update_config(updated)
        request.app.state.robot.config.joints[joint_name] = updated
        config_path: Path = getattr(request.app.state, "config_path", _DEFAULT_CONFIG_PATH)
        request.app.state.robot.config.to_json(config_path)
        # Daemon applies its startup config to device RAM; store_to_flash not available via daemon.
        await actuator.apply_config()
        return _ok({"applied": True})
    except DaemonNotSupportedError as exc:
        return _err(str(exc), 503)
    except Exception as exc:
        return _err(str(exc))


class PositionCalibrateBody(BaseModel):
    hardstop_lower_rad: float   # recorded state.position at lower hardstop
    limits_min: float           # desired lower limit in rad
    limits_max: float           # desired upper limit in rad


@router.post("/motors/{joint_name}/position_calibrate", response_model=None)
async def position_calibrate(
    joint_name: str, body: PositionCalibrateBody, request: Request
) -> dict | JSONResponse:
    actuator, error = _resolve_actuator(request, joint_name)
    if error:
        return error
    try:
        # Always derive new offset from the authoritative live value in actuator.config
        # so repeated calibrations don't compound stale frontend state.
        new_offset = actuator.config.position_offset + (body.hardstop_lower_rad - body.limits_min)
        new_limits = PositionLimits(min=body.limits_min, max=body.limits_max)
        updated = actuator.config.model_copy(update={
            "position_offset": new_offset,
            "position_limits": new_limits,
        })
        actuator.update_config(updated)
        request.app.state.robot.config.joints[joint_name] = updated
        config_path: Path = getattr(request.app.state, "config_path", _DEFAULT_CONFIG_PATH)
        request.app.state.robot.config.to_json(config_path)
        # Daemon applies its startup config; store_to_flash not available via daemon.
        await actuator.apply_config()
        return _ok({
            "position_offset": new_offset,
            "limits_min": body.limits_min,
            "limits_max": body.limits_max,
        })
    except DaemonNotSupportedError as exc:
        return _err(str(exc), 503)
    except Exception as exc:
        return _err(str(exc))


@router.post("/motors/{joint_name}/estop", response_model=None)
async def estop_motor(joint_name: str, request: Request) -> dict | JSONResponse:
    actuator, error = _resolve_actuator(request, joint_name)
    if error:
        return error
    try:
        await actuator.estop()
        return _ok({"joint_name": joint_name, "mode": "DISABLED"})
    except DaemonError as exc:
        return _err(str(exc))


@router.post("/motors/{joint_name}/store_to_flash", response_model=None)
async def store_motor_to_flash(joint_name: str, request: Request) -> dict | JSONResponse:
    actuator, error = _resolve_actuator(request, joint_name)
    if error:
        return error
    try:
        await actuator.store_to_flash()
        return _ok({"stored": True})
    except DaemonNotSupportedError as exc:
        return _err(str(exc), 503)
    except DaemonError as exc:
        return _err(str(exc))
