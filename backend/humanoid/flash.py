"""
Flash wizard state machine for B-G431B-ESC1 boards.

Single-pass firmware procedure:

  1. FLASHING:
     Flash a pre-compiled .elf (one per motor profile) via openocd/ST-LINK.
     Device boots at commissioning CAN ID 127 (no valid flash config yet).

  2. COMMISSIONING:
     Send CAN SDO writes to ID 127: set device CAN ID, phase order, current gains,
     then SAVE_CONFIG + REBOOT.  Device reboots at its assigned CAN ID.

  3. CALIBRATING:
     At the real CAN ID: run encoder flux offset calibration.

  4. AWAITING_CONFIRMATION:
     User confirms motor direction is correct.
     If wrong: toggle phase order, save, re-calibrate.

  5. COMPLETE.
"""
from __future__ import annotations

import asyncio
import math
import shutil
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path

from pydantic import BaseModel

from .can_bus import CANBus, Parameter
from .actuator import Actuator
from .robot_config import JointConfig


# ---------------------------------------------------------------------------
# Motor profiles
# ---------------------------------------------------------------------------

_TWO_PI = 2.0 * math.pi

def _gains(R: float, L: float) -> tuple[float, float]:
    return round(1000.0 * _TWO_PI * L, 6), round(R / L, 3)

MOTOR_PROFILES: dict[str, dict] = {
    "MAD_M6C12_150KV": {
        "torque_constant":        0.08958,
        "phase_resistance":       0.13793,
        "phase_inductance":       3.039166e-5,
        "max_calibration_current": 5,
        "i_kp":                   _gains(0.13793, 3.039166e-5)[0],
        "i_ki":                   _gains(0.13793, 3.039166e-5)[1],
    },
    "MAD_5010_110KV": {
        "torque_constant":        0.1176,
        "phase_resistance":       0.6193,
        "phase_inductance":       8.50e-5,
        "max_calibration_current": 3,
        "i_kp":                   _gains(0.6193, 8.50e-5)[0],
        "i_ki":                   _gains(0.6193, 8.50e-5)[1],
    },
    "MAD_5010_200KV": {
        "torque_constant":        0.06588,
        "phase_resistance":       0.15227,
        "phase_inductance":       2.649166e-5,
        "max_calibration_current": 3,
        "i_kp":                   _gains(0.15227, 2.649166e-5)[0],
        "i_ki":                   _gains(0.15227, 2.649166e-5)[1],
    },
    "MAD_5010_310KV": {
        "torque_constant":        0.04212,
        "phase_resistance":       0.05735,
        "phase_inductance":       3.3256e-5,
        "max_calibration_current": 5,
        "i_kp":                   _gains(0.05735, 3.3256e-5)[0],
        "i_ki":                   _gains(0.05735, 3.3256e-5)[1],
    },
    "MAD_5010_370KV": {
        "torque_constant":        0.03529,
        "phase_resistance":       0.03000,
        "phase_inductance":       1.0717e-5,
        "max_calibration_current": 5,
        "i_kp":                   _gains(0.03000, 1.0717e-5)[0],
        "i_ki":                   _gains(0.03000, 1.0717e-5)[1],
    },
}

_COMMISSIONING_CAN_ID = 127


# ---------------------------------------------------------------------------
# Public types
# ---------------------------------------------------------------------------

class FlashError(Exception):
    pass


class FlashState(str, Enum):
    IDLE                  = "IDLE"
    FLASHING              = "FLASHING"              # openocd flash
    COMMISSIONING         = "COMMISSIONING"          # CAN SDO config at ID 127
    WAITING_CAN_CONNECT   = "WAITING_CAN_CONNECT"   # user connects motor + CAN + encoder
    CALIBRATING           = "CALIBRATING"            # encoder flux offset calibration
    AWAITING_CONFIRMATION = "AWAITING_CONFIRMATION"  # user confirms motor direction
    COMPLETE              = "COMPLETE"
    FAILED                = "FAILED"

    @property
    def step_index(self) -> int:
        _map = {
            FlashState.IDLE:                  0,
            FlashState.FLASHING:              1,
            FlashState.COMMISSIONING:         2,
            FlashState.WAITING_CAN_CONNECT:   3,
            FlashState.CALIBRATING:           4,
            FlashState.AWAITING_CONFIRMATION: 4,
            FlashState.COMPLETE:              5,
            FlashState.FAILED:               -1,
        }
        return _map.get(self, 0)


_FLASH_TOTAL_STEPS = 6


class FlashConfig(BaseModel):
    firmware_dir: Path
    can_channel: str = "can0"
    can_id: int
    invert_phase: bool = False
    motor_profile: str = "MAD_5010_200KV"

    def profile_data(self) -> dict:
        key = self.motor_profile
        if key not in MOTOR_PROFILES:
            raise FlashError(
                f"Unknown motor profile '{key}'. "
                f"Valid options: {', '.join(MOTOR_PROFILES)}"
            )
        return MOTOR_PROFILES[key]


@dataclass
class FlashStatus:
    state: FlashState = FlashState.IDLE
    progress: int = 0
    messages: list[str] = field(default_factory=list)
    error: str | None = None
    flux_offset: float | None = None
    updated_config: dict | None = None


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _check_tools() -> list[str]:
    missing = []
    if shutil.which("openocd") is None:
        missing.append("openocd")
    return missing


async def _run_subprocess(cmd: list[str], cwd: Path) -> tuple[int, str]:
    proc = await asyncio.create_subprocess_exec(
        *cmd,
        cwd=cwd,
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.STDOUT,
    )
    stdout, _ = await proc.communicate()
    return proc.returncode, stdout.decode(errors="replace")


async def _flash_prebuilt(elf_path: Path, firmware_dir: Path, log_fn) -> None:
    log_fn(f"Flashing {elf_path.name} via ST-LINK (openocd)...")
    rc, out = await _run_subprocess(
        [
            "openocd",
            "-f", "interface/stlink.cfg",
            "-f", "target/stm32g4x.cfg",
            "-c", f"program {elf_path} verify reset exit",
        ],
        cwd=firmware_dir,
    )
    if rc != 0:
        raise FlashError(f"Flash failed:\n{out[-3000:]}")
    log_fn("Flash written — device is resetting.")


# ---------------------------------------------------------------------------
# FlashManager
# ---------------------------------------------------------------------------

class FlashManager:
    """Drives the flash wizard through its state machine."""

    def __init__(self) -> None:
        self.status = FlashStatus()
        self._lock = asyncio.Lock()
        self._task: asyncio.Task | None = None
        self._confirm_event: asyncio.Event | None = None
        self._confirmed_correct: bool | None = None
        self._can_connect_event: asyncio.Event | None = None
        self._current_channel: str | None = None
        self._current_can_id: int | None = None

    @property
    def current_channel(self) -> str | None:
        return self._current_channel

    def _log(self, msg: str, progress: int | None = None) -> None:
        self.status.messages.append(msg)
        if progress is not None:
            self.status.progress = progress

    # ── Public API ────────────────────────────────────────────────────────────

    async def start(self, port: str, config: FlashConfig) -> None:
        async with self._lock:
            if self.status.state not in (
                FlashState.IDLE, FlashState.COMPLETE, FlashState.FAILED
            ):
                raise FlashError("Flash session already in progress")

            missing = _check_tools()
            if missing:
                raise FlashError(
                    f"Required tool not found on PATH: {', '.join(missing)}. "
                    "Install openocd and ensure it is on PATH."
                )

            config.profile_data()

            elf_path = config.firmware_dir / "prebuilt" / f"commissioning_{config.motor_profile}.elf"
            if not elf_path.exists():
                raise FlashError(f"Pre-compiled binary not found: {elf_path}")

            self.status = FlashStatus()
            self._current_channel = config.can_channel
            self._current_can_id = config.can_id

        self._task = asyncio.create_task(self._run_session(port, config))

    async def reset(self) -> None:
        async with self._lock:
            if self._task is not None and not self._task.done():
                self._task.cancel()
                self._task = None
            self.status = FlashStatus()
            self._current_channel = None
            self._current_can_id = None

    async def can_connected(self) -> None:
        if self.status.state != FlashState.WAITING_CAN_CONNECT:
            raise FlashError("Not waiting for CAN connection confirmation")
        if self._can_connect_event is not None:
            self._can_connect_event.set()

    async def can_ping(self) -> dict:
        if self.status.state != FlashState.WAITING_CAN_CONNECT:
            raise FlashError(
                f"CAN ping only available during WAITING_CAN_CONNECT (current: {self.status.state})"
            )
        channel = self._current_channel
        can_id = self._current_can_id
        if not channel or can_id is None:
            raise FlashError("No CAN channel or device ID configured")

        bus = CANBus(channel=channel)
        try:
            await bus.connect()
            version = await bus.read_parameter_u32(can_id, Parameter.FIRMWARE_VERSION, timeout=2.0)
            if version is not None:
                return {"reachable": True, "detail": f"firmware {version:#010x}"}
            return {
                "reachable": False,
                "detail": f"no SDO response from device {can_id} on {channel} within 2 s",
            }
        except Exception as exc:
            return {"reachable": False, "detail": str(exc)}
        finally:
            await bus.disconnect()

    async def confirm_direction(self, correct: bool) -> None:
        if self.status.state != FlashState.AWAITING_CONFIRMATION:
            raise FlashError("No direction confirmation pending")
        self._confirmed_correct = correct
        if self._confirm_event is not None:
            self._confirm_event.set()

    def get_step_info(self) -> dict:
        return {
            "state":       self.status.state,
            "step_index":  self.status.state.step_index,
            "total_steps": _FLASH_TOTAL_STEPS,
            "progress":    self.status.progress,
            "message":     self.status.messages[-1] if self.status.messages else "",
        }

    # ── Internal session ──────────────────────────────────────────────────────

    async def _run_session(self, port: str, config: FlashConfig) -> None:
        try:
            await self._do_session(port, config)
        except asyncio.TimeoutError:
            self.status.state = FlashState.FAILED
            self.status.error = "Timed out waiting for user action"
            self._log("FAILED: timeout waiting for user action")
        except FlashError as exc:
            self.status.state = FlashState.FAILED
            self.status.error = str(exc)
            self._log(f"FAILED: {exc}")
        except Exception as exc:
            self.status.state = FlashState.FAILED
            self.status.error = str(exc)
            self._log(f"FAILED (unexpected): {exc}")

    async def _do_session(self, port: str, config: FlashConfig) -> None:
        profile = config.profile_data()
        firmware_dir = config.firmware_dir
        invert_phase = config.invert_phase

        # ── 1. Flash pre-compiled binary ─────────────────────────────────────
        self.status.state = FlashState.FLASHING
        elf_path = firmware_dir / "prebuilt" / f"commissioning_{config.motor_profile}.elf"
        self._log(f"Flashing firmware for {config.motor_profile}...", progress=5)
        await _flash_prebuilt(elf_path, firmware_dir, self._log)
        self._log("Firmware flashed. Device booting at commissioning ID 127...", progress=20)

        # ── 2. Commission over CAN at ID 127 ─────────────────────────────────
        self.status.state = FlashState.COMMISSIONING
        self._log("Waiting for ESC to boot...", progress=22)
        await asyncio.sleep(3.0)

        self._log(
            f"Commissioning: setting CAN ID={config.can_id}, "
            f"phase_order={'inverted' if invert_phase else 'normal'}, "
            f"i_kp={profile['i_kp']:.4f}, i_ki={profile['i_ki']:.3f}...",
            progress=25,
        )
        bus = CANBus(channel=config.can_channel)
        await bus.connect()
        try:
            comm_id = _COMMISSIONING_CAN_ID
            await bus.write_parameter_u32(comm_id, Parameter.DEVICE_ID, config.can_id)
            await bus.write_parameter_i32(comm_id, Parameter.MOTOR_PHASE_ORDER, -1 if invert_phase else +1)
            await bus.write_parameter_f32(comm_id, Parameter.CURRENT_CONTROLLER_I_KP, profile["i_kp"])
            await bus.write_parameter_f32(comm_id, Parameter.CURRENT_CONTROLLER_I_KI, profile["i_ki"])
            await bus.write_parameter_u32(comm_id, Parameter.SAVE_CONFIG, 1)
            self._log("Config saved to flash. Rebooting device...", progress=30)
            await bus.write_parameter_u32(comm_id, Parameter.REBOOT, 1)
        finally:
            await bus.disconnect()

        self._log("Waiting for ESC to reboot at its assigned CAN ID...", progress=32)
        await asyncio.sleep(3.0)
        self._log(
            f"ESC rebooted. Now connect motor, CAN bus, and encoder, then click 'Motor Connected'.",
            progress=35,
        )

        # ── 3. Wait for motor + CAN hardware connection ───────────────────────
        self.status.state = FlashState.WAITING_CAN_CONNECT
        self._can_connect_event = asyncio.Event()
        self._log(
            "Waiting for motor + CAN + encoder connection. (600 s timeout)",
            progress=37,
        )
        await asyncio.wait_for(self._can_connect_event.wait(), timeout=600.0)
        self._log("Connection confirmed. Connecting to CAN bus...", progress=40)

        bus = CANBus(channel=config.can_channel)
        await bus.connect()
        self._log("Waiting for CAN bus to settle...", progress=42)
        await asyncio.sleep(2.0)

        joint_cfg = JointConfig(
            joint_name="__flash_target__",
            can_channel=config.can_channel,
            can_id=config.can_id,
            phase_inverted=invert_phase,
        )
        actuator = Actuator(bus, joint_cfg)

        try:
            while True:
                # ── 4. Calibrate ─────────────────────────────────────────────
                self.status.state = FlashState.CALIBRATING
                self._log("Setting fast_frame_frequency=100 Hz...", progress=45)
                await bus.write_parameter_u32(config.can_id, Parameter.FAST_FRAME_FREQUENCY, 100)
                self._log(
                    "Starting encoder flux offset calibration (MODE_CALIBRATION). "
                    "This takes ~15 s. Do NOT power off the ESC.",
                    progress=48,
                )

                flux_offset = await actuator.calibrate_offset(
                    timeout=90.0,
                    on_progress=self._log,
                )
                self.status.flux_offset = flux_offset
                self._log(
                    f"Calibration done: flux_offset = {flux_offset:.4f} rad "
                    f"({math.degrees(flux_offset):.2f}°). Saved to Flash.",
                    progress=75,
                )

                # ── 5. Confirm direction ──────────────────────────────────────
                self.status.state = FlashState.AWAITING_CONFIRMATION
                self._confirm_event = asyncio.Event()
                self._confirmed_correct = None
                self._log(
                    "Did the motor rotate correctly during calibration? Confirm direction. (120 s timeout)",
                    progress=78,
                )
                await asyncio.wait_for(self._confirm_event.wait(), timeout=120.0)
                direction_correct = bool(self._confirmed_correct)

                if direction_correct:
                    break

                # Toggle phase, save, and re-calibrate
                invert_phase = not invert_phase
                self._log(
                    f"Direction wrong — toggling phase_order, saving, and re-calibrating...",
                    progress=80,
                )
                self.status.state = FlashState.CALIBRATING
                await bus.write_parameter_i32(config.can_id, Parameter.MOTOR_PHASE_ORDER, -1 if invert_phase else +1)
                await bus.write_parameter_u32(config.can_id, Parameter.SAVE_CONFIG, 1)
                self._log("Phase order updated and saved. Re-running calibration...", progress=82)

        finally:
            await bus.disconnect()

        self.status.updated_config = {
            "torque_constant":         profile["torque_constant"],
            "max_calibration_current": float(profile["max_calibration_current"]),
            "current_kp":              profile["i_kp"],
            "current_ki":              profile["i_ki"],
            "electrical_offset":       self.status.flux_offset,
            "phase_inverted":          invert_phase,
            "fast_frame_frequency":    100,
        }

        self.status.state = FlashState.COMPLETE
        self._log("Flash wizard complete. Motor is commissioned and operational.", progress=100)
