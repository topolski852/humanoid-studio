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

import asyncio
from pathlib import Path

from fastapi import APIRouter, Request
from fastapi.responses import JSONResponse
from pydantic import BaseModel

from humanoid.flash import FlashConfig, FlashError, MOTOR_PROFILES

router = APIRouter(tags=["flash"])

# firmware/esc/ lives two directories above this file (backend/api/ → backend/ → repo root)
_DEFAULT_FIRMWARE_DIR = Path(__file__).parents[2] / "firmware" / "esc"


async def _bounce_can_interface(channel: str) -> None:
    """Bring the CAN interface down then up to flush the kernel TX queue."""
    async def _run(cmd: list[str]) -> None:
        p = await asyncio.create_subprocess_exec(
            'sudo', '-n', *cmd,
            stdout=asyncio.subprocess.DEVNULL,
            stderr=asyncio.subprocess.DEVNULL,
        )
        try:
            await asyncio.wait_for(p.wait(), timeout=5.0)
        except asyncio.TimeoutError:
            try:
                p.kill()
            except Exception:
                pass

    await _run(['/sbin/ip', 'link', 'set', channel, 'down'])
    await asyncio.sleep(0.1)
    await _run(['/sbin/ip', 'link', 'set', channel, 'type', 'can', 'bitrate', '1000000'])
    await _run(['/sbin/ip', 'link', 'set', channel, 'txqueuelen', '1000'])
    await _run(['/sbin/ip', 'link', 'set', channel, 'up'])


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
    skip_flash: bool = False


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
            "torque_constant":         data["torque_constant"],
            "phase_resistance":        data["phase_resistance"],
            "phase_inductance":        data["phase_inductance"],
            "max_calibration_current": data["max_calibration_current"],
            "i_kp":                    data["i_kp"],
            "i_ki":                    data["i_ki"],
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
        skip_flash=body.skip_flash,
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
    """No-op: power cycle step no longer required in the new single-pass flow."""
    return _ok({"message": "No power cycle required in this firmware version"})


@router.post("/flash/can_connected", response_model=None)
async def flash_can_connected(request: Request) -> dict | JSONResponse:
    """Frontend calls this when user confirms motor + CAN + encoder are connected."""
    flash_manager = request.app.state.flash_manager
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


@router.get("/flash/firmware_version")
async def flash_firmware_version(
    request: Request,
    firmware_dir: str | None = None,
) -> dict | JSONResponse:
    """
    Read ESC firmware version via SWD and compare to the bundled VERSION file.
    Returns match, esc_version_str, expected_version_str, error.
    Requires ST-LINK to be connected and ESC powered.
    """
    flash_manager = request.app.state.flash_manager
    fw_dir = Path(firmware_dir) if firmware_dir else _DEFAULT_FIRMWARE_DIR
    try:
        result = await flash_manager.check_firmware_version(fw_dir)
        return _ok(result)
    except Exception as exc:
        return _err(str(exc), 500)
