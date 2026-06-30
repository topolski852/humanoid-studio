"""
Motor control endpoints.

GET  /motors/{joint_name}
POST /motors/{joint_name}/connect
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

import asyncio
import logging
from pathlib import Path

from fastapi import APIRouter, Request
from fastapi.responses import JSONResponse
from pydantic import BaseModel

from humanoid.daemon_client import DaemonError, DaemonNotSupportedError, Mode
from humanoid.motor_tune import run_step_test
from humanoid.motor_diagnose import RunawayAbort, run_diagnosis, run_gravity_tune
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


async def _run_cancellable(request: Request, coro) -> dict | JSONResponse:
    """Run a long actuated routine, cancelling it if the client disconnects.

    Mirrors the step_test route's cancel pattern so a closed tab can't leave a
    diagnosis/sweep running as a ghost.  RunawayAbort surfaces as 409.
    """
    task = asyncio.create_task(coro)
    try:
        while not task.done():
            await asyncio.sleep(0.1)
            if await request.is_disconnected():
                task.cancel()
                try:
                    await task
                except BaseException:
                    pass
                return JSONResponse(
                    {"success": False, "data": None, "error": "cancelled"},
                    status_code=499,
                )
        return _ok(await task)
    except RunawayAbort as exc:
        task.cancel()
        return _err(str(exc), 409)
    except ValueError as exc:
        task.cancel()
        return _err(str(exc), 400)
    except DaemonError as exc:
        task.cancel()
        return _err(str(exc))


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

@router.post("/motors/{joint_name}/connect", response_model=None)
async def connect_motor_single(joint_name: str, request: Request) -> dict | JSONResponse:
    """Configure a single motor (OFFLINE → IDLE) without touching other motors."""
    robot = request.app.state.robot
    if robot is None:
        return _err("No robot config loaded — PUT /robot/config first", 503)
    if robot.get_actuator_by_name(joint_name) is None:
        return _err(f"No joint named '{joint_name}'", 404)
    try:
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, robot.connect_single, joint_name)
        return _ok({"joint_name": joint_name, "connected": True})
    except DaemonError as exc:
        return _err(str(exc))


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
    # Refuse early if motor is offline — avoids a 20-second wait while 27 SDO
    # writes all time out one-by-one against an unresponsive device.
    cached_state = actuator.get_cached_state()
    if cached_state is None:
        return _err(
            "Motor is OFFLINE — cannot apply config. "
            "Enable the motor (or at least connect it) before applying gains.",
            status=409,
        )
    try:
        updated = actuator.config.__class__.model_validate(
            {**actuator.config.model_dump(), **body.config}
        )
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


class WriteGainsBody(BaseModel):
    position_kp: float
    position_ki: float
    velocity_kp: float
    torque_limit: float


@router.post("/motors/{joint_name}/write_gains", response_model=None)
async def write_motor_gains(
    joint_name: str, body: WriteGainsBody, request: Request
) -> dict | JSONResponse:
    """
    Write position_kp, position_ki, velocity_kp (Kd), torque_limit to device RAM (~4 SDOs, ~20 ms).
    Does not persist to flash or update the JSON config file.
    Use apply_config for a full commit.
    """
    actuator, error = _resolve_actuator(request, joint_name)
    if error:
        return error
    cached_state = actuator.get_cached_state()
    if cached_state is None:
        return _err("Motor is OFFLINE — cannot write gains.", status=409)
    try:
        await actuator.write_gains(body.position_kp, body.position_ki,
                                   body.velocity_kp, body.torque_limit)
        return _ok({"applied": True})
    except DaemonError as exc:
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


# ---------------------------------------------------------------------------
# Step-response auto-tuner
# ---------------------------------------------------------------------------

class StepTestBody(BaseModel):
    position_kp: float
    position_ki: float
    velocity_kp: float
    torque_limit: float
    center_rad: float
    offset_rad: float = 0.45
    step_hold_s: float = 1.5
    num_steps: int = 4


@router.post("/motors/{joint_name}/step_test", response_model=None)
async def step_test_motor(
    joint_name: str, body: StepTestBody, request: Request
) -> dict | JSONResponse:
    """
    Run a step-response PID test on the motor.

    The motor must be in POSITION mode (enabled).  The total duration is
    step_hold_s * (num_steps + 1) seconds (including pre-settle at pos_a).
    Returns samples and step-response metrics.

    Cancels automatically if the client disconnects (tab close / navigation),
    preventing the step sequence from continuing as a ghost after the UI leaves.
    """
    actuator, error = _resolve_actuator(request, joint_name)
    if error:
        return error
    task = asyncio.create_task(run_step_test(actuator, **body.model_dump()))
    try:
        while not task.done():
            await asyncio.sleep(0.1)
            if await request.is_disconnected():
                task.cancel()
                try:
                    await task
                except (asyncio.CancelledError, Exception):
                    pass
                return JSONResponse(
                    {"success": False, "data": None, "error": "cancelled"},
                    status_code=499,
                )
        return _ok(await task)
    except ValueError as exc:
        task.cancel()
        return _err(str(exc), 400)
    except DaemonError as exc:
        task.cancel()
        return _err(str(exc))


# ---------------------------------------------------------------------------
# Diagnostic + gravity-aware auto-tuner (Workflow A / B)
# ---------------------------------------------------------------------------

class DiagnoseBody(BaseModel):
    step_rad: float = 0.15
    hold_s: float = 1.0
    ramp_rad: float = 0.25
    ramp_s: float = 3.0
    diag_torque_limit: float | None = None
    runaway_band_rad: float | None = None
    current_fault_a: float = 3.0


class GravityTuneBody(BaseModel):
    kp: float
    torque_limit: float
    ki: float = 0.0
    kd_values: list[float] = [0.5, 1.0, 2.0, 4.0, 8.0]
    lift_rad: float = 0.25
    lift_sign: float = 1.0
    hold_s: float = 1.5
    test_ki: bool = False


class RaiseTorqueBody(BaseModel):
    torque_limit: float
    confirm: bool = False


class PhaseRemediationBody(BaseModel):
    confirm: bool = False


@router.post("/motors/{joint_name}/diagnose", response_model=None)
async def diagnose_motor(
    joint_name: str, body: DiagnoseBody, request: Request
) -> dict | JSONResponse:
    """Workflow A — classify why a joint misbehaves (non-destructive).

    Runs a guarded move + commutation ramp under a low diagnosis torque limit and
    returns {classification, evidence, recommendation, rationale, thresholds}.
    The recommendation is advisory — remediation is a separate confirmed call.
    Motor must be enabled in POSITION mode.
    """
    actuator, error = _resolve_actuator(request, joint_name)
    if error:
        return error
    return await _run_cancellable(request, run_diagnosis(actuator, **body.model_dump()))


@router.post("/motors/{joint_name}/gravity_tune", response_model=None)
async def gravity_tune_motor(
    joint_name: str, body: GravityTuneBody, request: Request
) -> dict | JSONResponse:
    """Workflow B — lift/drop Kd sweep for a gravity-loaded joint (non-destructive).

    Gains are written to RAM transiently and restored on exit.  Returns the sweep
    table, selected Kd, recommended Kp/Kd, and a Ki-windup finding.
    Motor must be enabled in POSITION mode.
    """
    actuator, error = _resolve_actuator(request, joint_name)
    if error:
        return error
    return await _run_cancellable(request, run_gravity_tune(actuator, **body.model_dump()))


@router.post("/motors/{joint_name}/raise_torque_limit", response_model=None)
async def raise_torque_limit_route(
    joint_name: str, body: RaiseTorqueBody, request: Request
) -> dict | JSONResponse:
    """Write a higher torque_limit to RAM (confirmed remediation for torque-starved)."""
    actuator, error = _resolve_actuator(request, joint_name)
    if error:
        return error
    if not body.confirm:
        return _err("confirmation required", 400)
    if actuator.get_cached_state() is None:
        return _err("Motor is OFFLINE — cannot write gains.", status=409)
    try:
        cfg = actuator.config
        await actuator.write_gains(cfg.position_kp, cfg.position_ki,
                                   cfg.velocity_kp, body.torque_limit)
        return _ok({"torque_limit": body.torque_limit, "applied": True})
    except DaemonError as exc:
        return _err(str(exc))


@router.post("/motors/{joint_name}/remediate_phase", response_model=None)
async def remediate_phase(
    joint_name: str, body: PhaseRemediationBody, request: Request
) -> dict | JSONResponse:
    """Confirmed remediation for COMMUTATION_FAULT: flip phase_inverted, recalibrate
    flux (required after a phase flip), persist, then re-diagnose to confirm.

    This spins the motor (~90 s) and writes the new flux offset + phase to JSON and
    device RAM.  It does NOT store to flash — the UI offers a separate confirmed
    Persist-to-Flash step.  Wrapped so a failed recal restores the prior phase.
    """
    actuator, error = _resolve_actuator(request, joint_name)
    if error:
        return error
    if not body.confirm:
        return _err("confirmation required for phase flip + recalibration", 400)
    if actuator.get_cached_state() is None:
        return _err("Motor is OFFLINE — connect it first.", status=409)

    config_path: Path = getattr(request.app.state, "config_path", _DEFAULT_CONFIG_PATH)
    cfg0 = actuator.config
    flux_before = cfg0.electrical_offset
    phase_before = cfg0.phase_inverted

    def _persist(updated):
        actuator.update_config(updated)
        request.app.state.robot.config.joints[joint_name] = updated
        request.app.state.robot.config.to_json(config_path)

    try:
        # Calibration runs in IDLE; also avoids writing phase_order while enabled.
        await actuator.disable()
        await asyncio.sleep(0.2)

        # 1. flip phase_inverted, persist + push to RAM
        _persist(cfg0.model_copy(update={"phase_inverted": not cfg0.phase_inverted}))
        await actuator.apply_config()

        # 2. recalibrate flux for the new phase order (updates actuator.config)
        flux_after = await actuator.calibrate_offset(timeout=90.0)

        # 3. persist the freshly-calibrated offset (+ flipped phase) and push to RAM
        _persist(actuator.config)
        await actuator.apply_config()

        # 4. re-diagnose to confirm it now commutates (current should drop, it tracks)
        await actuator.enable(mode=Mode.POSITION)
        await asyncio.sleep(0.3)
        recheck = await run_diagnosis(actuator)
        return _ok({
            "phase_inverted": actuator.config.phase_inverted,
            "flux_before": flux_before,
            "flux_after": flux_after,
            "recheck": {
                "classification": recheck["classification"],
                "evidence": recheck["evidence"],
                "rationale": recheck["rationale"],
            },
        })
    except DaemonNotSupportedError as exc:
        return _err(str(exc), 503)
    except (DaemonError, ValueError) as exc:
        # Best-effort restore of the prior phase so a failed recal can't leave it
        # mis-commutated with the flipped phase.
        try:
            _persist(cfg0.model_copy(update={"phase_inverted": phase_before,
                                             "electrical_offset": flux_before}))
            await actuator.apply_config()
        except Exception:
            pass
        return _err(str(exc))
