"""
Robot configuration endpoints.

GET /robot/config
PUT /robot/config
"""
from __future__ import annotations

from pathlib import Path

from fastapi import APIRouter, Request
from fastapi.responses import JSONResponse

from humanoid.robot_config import RobotConfig
from humanoid.robot import Robot

router = APIRouter(tags=["robot"])

_DEFAULT_CONFIG_PATH = Path(__file__).parents[3] / "configs" / "humanoid_lite.json"


def _ok(data: object) -> dict:
    return {"success": True, "data": data, "error": None}


def _err(msg: str, status: int = 400) -> JSONResponse:
    return JSONResponse(
        {"success": False, "data": None, "error": msg},
        status_code=status,
    )


@router.get("/robot/config", response_model=None)
async def get_robot_config(request: Request) -> dict | JSONResponse:
    config: RobotConfig | None = request.app.state.config
    if config is None:
        return _err("No robot config loaded", 404)
    return _ok(config.to_dict())


@router.post("/robot/connect", response_model=None)
async def connect_robot(request: Request) -> dict | JSONResponse:
    robot: Robot | None = request.app.state.robot
    if robot is None:
        return _err("No robot config loaded — PUT /robot/config first", 503)
    if robot.is_connected():
        return _ok({"message": "Already connected"})
    try:
        await robot.connect()
        # Pull calibration values (flux_offset, encoder offsets) from ESC so apply_config()
        # won't overwrite them with the 0.0 defaults from the JSON config.
        await robot.read_calibration_from_devices()
        # Write design params (gear_ratio, PID, fast_frame_frequency) to ESC RAM.
        # Errors per-joint are caught and logged inside apply_all_configs.
        await robot.apply_all_configs()
        return _ok({"message": "Connected", "joint_count": len(robot.joint_names())})
    except Exception as exc:
        try:
            await robot.disconnect()
        except Exception:
            pass
        return _err(f"Connect failed: {exc}", 500)


@router.post("/robot/disconnect", response_model=None)
async def disconnect_robot(request: Request) -> dict | JSONResponse:
    robot: Robot | None = request.app.state.robot
    if robot is None or not robot.is_connected():
        return _ok({"message": "Already disconnected"})
    try:
        await robot.disconnect()
        return _ok({"message": "Disconnected"})
    except Exception as exc:
        return _err(f"Disconnect failed: {exc}", 500)


@router.put("/robot/config", response_model=None)
async def put_robot_config(request: Request) -> dict | JSONResponse:
    """
    Accept a full RobotConfig JSON body, validate it, persist to disk,
    and hot-reload the Robot instance (disconnects current connections).
    """
    try:
        body = await request.json()
    except Exception:
        return _err("Invalid JSON body", 400)

    try:
        new_config = RobotConfig.model_validate(body)
    except Exception as exc:
        return _err(f"Config validation failed: {exc}", 422)

    config_path: Path = getattr(request.app.state, "config_path", _DEFAULT_CONFIG_PATH)
    try:
        new_config.to_json(config_path)
    except OSError as exc:
        return _err(f"Failed to persist config: {exc}", 500)

    # Disconnect existing robot before replacing
    old_robot: Robot | None = request.app.state.robot
    if old_robot is not None:
        await old_robot.disconnect()

    new_robot = Robot(new_config)
    request.app.state.robot = new_robot
    request.app.state.config = new_config

    return _ok({
        "robot_name": new_config.robot_name,
        "joint_count": len(new_config.joints),
        "message": "Config updated — call connect endpoints to re-establish CAN links",
    })
