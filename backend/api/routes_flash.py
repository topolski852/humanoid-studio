"""
Flash wizard endpoints.

POST /flash/start              body: {can_id, invert_phase, motor_profile, port?, can_channel?, firmware_dir?}
GET  /flash/status             returns state, progress, messages, flux_offset, error, updated_config
GET  /flash/step               returns step_index, total_steps, progress for UI progress strip
POST /flash/power_cycled       — called when user confirms ESC has been power-cycled
POST /flash/can_connected      — called when user confirms motor + CAN + encoder are connected
POST /flash/confirm_direction  body: {correct: bool}
GET  /flash/profiles           — list available motor profiles with specs
POST /flash/reset              — force state back to IDLE, cancels any in-progress session
"""
from __future__ import annotations

from pathlib import Path

from fastapi import APIRouter, Request
from fastapi.responses import JSONResponse
from pydantic import BaseModel

from humanoid.flash import FlashConfig, FlashError, MOTOR_PROFILES

router = APIRouter(tags=["flash"])

# Actual firmware project is one level deeper than the BESC repo root
_DEFAULT_FIRMWARE_DIR = (
    Path("/home/nse/Recoil-Motor-Controller-BESC")
    / "Recoil-Motor-Controller-B-G431B-ESC1"
)


def _ok(data: object) -> dict:
    return {"success": True, "data": data, "error": None}


def _err(msg: str, status: int = 400) -> JSONResponse:
    return JSONResponse(
        {"success": False, "data": None, "error": msg},
        status_code=status,
    )


# ---------------------------------------------------------------------------
# Request bodies
# ---------------------------------------------------------------------------

class FlashStartBody(BaseModel):
    can_id: int
    invert_phase: bool = False
    motor_profile: str = "MAD_5010_200KV"
    can_channel: str = "can0"
    port: str = "SWD"
    firmware_dir: str | None = None


class ConfirmDirectionBody(BaseModel):
    correct: bool


# ---------------------------------------------------------------------------
# Routes
# ---------------------------------------------------------------------------

@router.get("/flash/profiles")
async def flash_profiles() -> dict:
    """Return all motor profiles with specs for the UI profile selector."""
    profiles = []
    for key, data in MOTOR_PROFILES.items():
        profiles.append({
            "key":                     key,
            "define":                  data["define"],
            "torque_constant":         data["torque_constant"],
            "phase_resistance":        data["phase_resistance"],
            "phase_inductance":        data["phase_inductance"],
            "max_calibration_current": data["max_calibration_current"],
            "i_kp":                    data["i_kp"],
            "i_ki":                    data["i_ki"],
            "available":               data["torque_constant"] is not None,
        })
    return _ok({"profiles": profiles})


@router.post("/flash/start", response_model=None)
async def flash_start(body: FlashStartBody, request: Request) -> dict | JSONResponse:
    flash_manager = request.app.state.flash_manager
    firmware_dir = Path(body.firmware_dir) if body.firmware_dir else _DEFAULT_FIRMWARE_DIR

    if not firmware_dir.exists():
        return _err(f"Firmware directory not found: {firmware_dir}", 400)

    config = FlashConfig(
        firmware_dir=firmware_dir,
        can_channel=body.can_channel,
        can_id=body.can_id,
        invert_phase=body.invert_phase,
        motor_profile=body.motor_profile,
    )
    try:
        await flash_manager.start(port=body.port, config=config)
        return _ok({
            "state": flash_manager.status.state,
            "message": "Flash session started",
        })
    except FlashError as exc:
        return _err(str(exc), 409)


@router.get("/flash/status")
async def flash_status(request: Request) -> dict:
    flash_manager = request.app.state.flash_manager
    s = flash_manager.status
    return _ok({
        "state":          s.state,
        "progress":       s.progress,
        "messages":       s.messages,
        "flux_offset":    s.flux_offset,
        "error":          s.error,
        "updated_config": s.updated_config,
    })


@router.get("/flash/step")
async def flash_step(request: Request) -> dict:
    """Return step strip info for the frontend progress indicator."""
    flash_manager = request.app.state.flash_manager
    return _ok(flash_manager.get_step_info())


@router.post("/flash/power_cycled", response_model=None)
async def flash_power_cycled(request: Request) -> dict | JSONResponse:
    """Frontend calls this when the user has power-cycled the ESC after Pass 1."""
    flash_manager = request.app.state.flash_manager
    try:
        await flash_manager.power_cycled()
        return _ok({"message": "Power cycle acknowledged"})
    except FlashError as exc:
        return _err(str(exc), 409)


@router.post("/flash/can_connected", response_model=None)
async def flash_can_connected(request: Request) -> dict | JSONResponse:
    """Frontend calls this when user confirms motor + CAN + encoder are connected."""
    flash_manager = request.app.state.flash_manager

    # The robot's telemetry loop hammers the shared SocketCAN TX queue (txqueuelen=10)
    # with ~30 SDO frames/cycle, leaving no room for the flash wizard's socket.
    # Disconnect the robot before the flash wizard opens its own socket so the queue
    # is free. The user will reconnect from the sidebar after flashing completes.
    robot = request.app.state.robot
    if robot and robot.is_connected():
        await robot.disconnect()

    try:
        await flash_manager.can_connected()
        return _ok({"message": "CAN connection confirmed"})
    except FlashError as exc:
        return _err(str(exc), 409)


@router.post("/flash/confirm_direction", response_model=None)
async def flash_confirm_direction(
    body: ConfirmDirectionBody, request: Request
) -> dict | JSONResponse:
    flash_manager = request.app.state.flash_manager
    try:
        await flash_manager.confirm_direction(correct=body.correct)
        return _ok({"confirmed": body.correct})
    except FlashError as exc:
        return _err(str(exc), 409)


@router.post("/flash/can_ping", response_model=None)
async def flash_can_ping(request: Request) -> dict | JSONResponse:
    """
    Ping the target device CAN ID on the flash channel.
    Returns {reachable: bool, detail: str}.
    Only valid during WAITING_CAN_CONNECT.
    """
    flash_manager = request.app.state.flash_manager
    try:
        result = await flash_manager.can_ping()
        return _ok(result)
    except FlashError as exc:
        return _err(str(exc), 409)


@router.post("/flash/reset")
async def flash_reset(request: Request) -> dict:
    """Force the flash manager back to IDLE, cancelling any in-progress session."""
    await request.app.state.flash_manager.reset()
    return _ok({"state": "IDLE", "message": "Session reset"})
