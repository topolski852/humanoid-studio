"""
Robot configuration endpoints.

GET /robot/config
PUT /robot/config
"""
from __future__ import annotations

import asyncio
from pathlib import Path

from fastapi import APIRouter, Request
from fastapi.responses import JSONResponse

from humanoid import settings as _app_settings
from humanoid.robot_config import RobotConfig
from humanoid.joint_defaults import apply_default_limits
from humanoid.daemon_client import DaemonClient, DaemonNotRunningError

router = APIRouter(tags=["robot"])

# Fallback for the rare case app.state.config_path is unset. parents[3] pointed
# one level above the repo root — a path that never existed — so go through the
# same resolver the backend starts from, which also stays writable when packaged.
_DEFAULT_CONFIG_PATH = _app_settings.resolve_config_path()


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
    robot: DaemonClient | None = request.app.state.robot
    if robot is None:
        return _err("No robot config loaded — PUT /robot/config first", 503)
    # Always call apply_all_configs — motors start OFFLINE after daemon launch.
    # apply_config sends fast_frame_frequency=100 which triggers PDO4 broadcast and
    # transitions motors OFFLINE → IDLE.  Skipping this when is_connected() is True
    # was the bug: the daemon broadcasts telemetry immediately, so is_connected() is
    # True before any config has been written to the ESCs.
    try:
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, robot.apply_all_configs)
        return _ok({"message": "Connected", "joint_count": len(robot.config.joint_names())})
    except DaemonNotRunningError:
        return _err("Daemon not running — click Connect again or restart the app", 503)
    except Exception as exc:
        return _err(f"Connect failed: {exc}", 500)


@router.post("/robot/disconnect", response_model=None)
async def disconnect_robot(request: Request) -> dict | JSONResponse:
    robot: DaemonClient | None = request.app.state.robot
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

    # Fill in the Berkeley Humanoid Lite default position limits for any joint
    # that has none yet (e.g. just commissioned via the Flash Wizard). Joints
    # with explicit limits are left untouched.
    apply_default_limits(new_config)

    config_path: Path = getattr(request.app.state, "config_path", _DEFAULT_CONFIG_PATH)
    try:
        new_config.to_json(config_path)
    except OSError as exc:
        return _err(f"Failed to persist config: {exc}", 500)

    # Update the running DaemonClient's config and rebuild per-joint proxies.
    # DaemonClient.disconnect() is a no-op so we skip it.
    client: DaemonClient = request.app.state.can_monitor  # always set; robot may be None
    client.config = new_config
    client._rebuild_proxies()
    request.app.state.robot  = client
    request.app.state.config = new_config

    return _ok({
        "robot_name": new_config.robot_name,
        "joint_count": len(new_config.joints),
        "message": "Config updated — call connect endpoints to re-establish CAN links",
    })
