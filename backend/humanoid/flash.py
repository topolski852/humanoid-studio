"""
Flash wizard state machine for B-G431B-ESC1 boards.

3-pass firmware procedure (each pass = patch conf.h → compile → openocd):

  Pass 1 INIT_FLASH:
    FIRST_TIME_BOOTUP=1, all LOAD flags=1, selected motor profile
    → programs STM32 Flash option bytes + writes default MotorController struct to Flash page 63
    → firmware enters infinite loop (halts); user must physically power-cycle the ESC

  Pass 2 PROGRAM_FLASH:
    FIRST_TIME_BOOTUP=0, all LOAD flags=0, correct CAN_ID and MOTOR_PHASE_ORDER
    → firmware writes defaults back to Flash (now Flash has correct CAN_ID and motor profile)
    → user then connects motor + CAN bus + encoder and powers on ESC (WAITING_CAN_CONNECT)
    → host sets fast_frame_frequency=100 via CAN SDO, then sends NMT MODE_CALIBRATION
    → firmware sweeps motor, computes flux_offset, saves full struct to Flash page 63

  Pass 3 FINALIZE_FLASH:
    FIRST_TIME_BOOTUP=0, all LOAD flags=1, same CAN_ID and MOTOR_PHASE_ORDER
    → operational firmware: loads CAN_ID + config + calibrated flux_offset from Flash every boot
    → PDO4 broadcast resumes at 100 Hz (loaded from Flash)
"""
from __future__ import annotations

import asyncio
import math
import re
import shutil
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Literal

from pydantic import BaseModel

from .can_bus import CANBus, Parameter
from .actuator import Actuator
from .robot_config import JointConfig


# ---------------------------------------------------------------------------
# Motor profiles
# ---------------------------------------------------------------------------

# Firmware define names exactly as they appear in motor_controller_conf.h
PROFILE_DEFINES = [
    "MOTORPROFILE_MAD_M6C12_150KV",
    "MOTORPROFILE_MAD_5010_110KV",
    "MOTORPROFILE_MAD_5010_200KV",
    "MOTORPROFILE_MAD_5010_310KV",
    "MOTORPROFILE_MAD_5010_370KV",
]

# Motor profile data: torque_constant, phase_resistance, phase_inductance, cal_current
# i_kp = 1000 * 2π * L  (CurrentController_init: bandwidth=1000 Hz)
# i_ki = R / L
_TWO_PI = 2.0 * math.pi

def _gains(R: float, L: float) -> tuple[float, float]:
    return round(1000.0 * _TWO_PI * L, 6), round(R / L, 3)

MOTOR_PROFILES: dict[str, dict] = {
    "MAD_M6C12_150KV": {
        "define":                 "MOTORPROFILE_MAD_M6C12_150KV",
        "torque_constant":        0.08958,
        "phase_resistance":       0.13793,
        "phase_inductance":       3.039166e-5,
        "max_calibration_current": 5,
        "i_kp":                   _gains(0.13793, 3.039166e-5)[0],
        "i_ki":                   _gains(0.13793, 3.039166e-5)[1],
    },
    "MAD_5010_110KV": {
        "define":                 "MOTORPROFILE_MAD_5010_110KV",
        "torque_constant":        0.1176,
        "phase_resistance":       0.6193,
        "phase_inductance":       8.50e-5,
        "max_calibration_current": 3,
        "i_kp":                   _gains(0.6193, 8.50e-5)[0],
        "i_ki":                   _gains(0.6193, 8.50e-5)[1],
    },
    "MAD_5010_200KV": {
        "define":                 "MOTORPROFILE_MAD_5010_200KV",
        "torque_constant":        0.06588,
        "phase_resistance":       0.15227,
        "phase_inductance":       2.649166e-5,
        "max_calibration_current": 3,
        "i_kp":                   _gains(0.15227, 2.649166e-5)[0],
        "i_ki":                   _gains(0.15227, 2.649166e-5)[1],
    },
    # 310KV and 370KV have no torque constant defined in firmware — not recommended
    "MAD_5010_310KV": {
        "define":                 "MOTORPROFILE_MAD_5010_310KV",
        "torque_constant":        None,   # undefined in firmware
        "phase_resistance":       0.05735,
        "phase_inductance":       3.3256e-5,
        "max_calibration_current": 5,
        "i_kp":                   _gains(0.05735, 3.3256e-5)[0],
        "i_ki":                   _gains(0.05735, 3.3256e-5)[1],
    },
    "MAD_5010_370KV": {
        "define":                 "MOTORPROFILE_MAD_5010_370KV",
        "torque_constant":        None,   # undefined in firmware
        "phase_resistance":       0.03000,
        "phase_inductance":       1.0717e-5,
        "max_calibration_current": 5,
        "i_kp":                   _gains(0.03000, 1.0717e-5)[0],
        "i_ki":                   _gains(0.03000, 1.0717e-5)[1],
    },
}


# ---------------------------------------------------------------------------
# Public types
# ---------------------------------------------------------------------------

class FlashError(Exception):
    pass


class FlashState(str, Enum):
    IDLE                  = "IDLE"
    INIT_FLASH            = "INIT_FLASH"            # Pass 1
    WAITING_POWER_CYCLE   = "WAITING_POWER_CYCLE"   # user power-cycles ESC
    PROGRAM_FLASH         = "PROGRAM_FLASH"          # Pass 2 (USB only)
    WAITING_CAN_CONNECT   = "WAITING_CAN_CONNECT"   # user connects motor + CAN + encoder
    CALIBRATING           = "CALIBRATING"            # CAN: flux offset calibration
    AWAITING_CONFIRMATION = "AWAITING_CONFIRMATION"  # user confirms motor direction
    REFLASHING            = "REFLASHING"             # re-run Pass 2 with toggled phase
    FINALIZE_FLASH        = "FINALIZE_FLASH"         # Pass 3 (USB only)
    COMPLETE              = "COMPLETE"
    FAILED                = "FAILED"

    @property
    def step_index(self) -> int:
        """0-based step index for the frontend step strip (8 steps total)."""
        _map = {
            FlashState.IDLE:                  0,
            FlashState.INIT_FLASH:            1,
            FlashState.WAITING_POWER_CYCLE:   2,
            FlashState.PROGRAM_FLASH:         3,
            FlashState.REFLASHING:            3,
            FlashState.WAITING_CAN_CONNECT:   4,
            FlashState.CALIBRATING:           5,
            FlashState.AWAITING_CONFIRMATION: 5,
            FlashState.FINALIZE_FLASH:        6,
            FlashState.COMPLETE:              7,
            FlashState.FAILED:               -1,
        }
        return _map.get(self, 0)


_FLASH_TOTAL_STEPS = 8


class FlashConfig(BaseModel):
    firmware_dir: Path
    can_channel: str = "can0"
    can_id: int
    invert_phase: bool = False
    motor_profile: str = "MAD_5010_200KV"   # key into MOTOR_PROFILES

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
    # Populated on COMPLETE so the frontend can sync humanoid_lite.json
    updated_config: dict | None = None


# ---------------------------------------------------------------------------
# conf.h patching helpers
# ---------------------------------------------------------------------------

_CONF_H_RELPATH = Path("Core/Inc/motor_controller_conf.h")

_FLAG_RE: dict[str, re.Pattern] = {
    "FIRST_TIME_BOOTUP":           re.compile(r"(#define\s+FIRST_TIME_BOOTUP\s+)\d+"),
    "LOAD_ID_FROM_FLASH":          re.compile(r"(#define\s+LOAD_ID_FROM_FLASH\s+)\d+"),
    "LOAD_CONFIG_FROM_FLASH":      re.compile(r"(#define\s+LOAD_CONFIG_FROM_FLASH\s+)\d+"),
    "LOAD_CALIBRATION_FROM_FLASH": re.compile(r"(#define\s+LOAD_CALIBRATION_FROM_FLASH\s+)\d+"),
    "DEVICE_CAN_ID":               re.compile(r"(#define\s+DEVICE_CAN_ID\s+)\d+"),
    "MOTOR_PHASE_ORDER":           re.compile(r"(#define\s+MOTOR_PHASE_ORDER\s+)[+-]?\d+"),
}

# Matches any motor profile define (commented or not)
_PROFILE_RE = re.compile(
    r"^(//)?#define\s+(MOTORPROFILE_MAD_\w+)\s*$",
    re.MULTILINE,
)


def _patch_conf_h(
    firmware_dir: Path,
    *,
    first_time_bootup: Literal[0, 1],
    load_id: Literal[0, 1],
    load_config: Literal[0, 1],
    load_calibration: Literal[0, 1],
    can_id: int,
    invert_phase: bool,
    motor_profile_define: str,
) -> str:
    """
    Patch motor_controller_conf.h in-place for one flash pass.
    Returns the original file contents so the caller can restore it.
    """
    conf_h = firmware_dir / _CONF_H_RELPATH
    original = conf_h.read_text()
    text = original

    flag_values = {
        "FIRST_TIME_BOOTUP":           str(first_time_bootup),
        "LOAD_ID_FROM_FLASH":          str(load_id),
        "LOAD_CONFIG_FROM_FLASH":      str(load_config),
        "LOAD_CALIBRATION_FROM_FLASH": str(load_calibration),
        "DEVICE_CAN_ID":               str(can_id),
        "MOTOR_PHASE_ORDER":           "-1" if invert_phase else "+1",
    }
    for name, value in flag_values.items():
        text, count = _FLAG_RE[name].subn(rf"\g<1>{value}", text)
        if count != 1:
            raise FlashError(
                f"Could not patch '{name}' in {conf_h}: "
                f"expected 1 match, found {count}. "
                "Has motor_controller_conf.h been modified?"
            )

    # Uncomment selected motor profile, comment all others
    def _replace_profile(m: re.Match) -> str:
        define_name = m.group(2)
        if define_name == motor_profile_define:
            return f"#define {define_name}"   # ensure uncommented
        else:
            return f"//#define {define_name}"  # ensure commented

    text, count = _PROFILE_RE.subn(_replace_profile, text)
    if count == 0:
        raise FlashError(
            f"No MOTORPROFILE_MAD_* defines found in {conf_h}. "
            "Has motor_controller_conf.h been modified?"
        )

    conf_h.write_text(text)
    return original


def _restore_conf_h(firmware_dir: Path, original: str) -> None:
    (firmware_dir / _CONF_H_RELPATH).write_text(original)


def _check_tools() -> list[str]:
    missing = []
    for tool in ("arm-none-eabi-gcc", "make", "openocd"):
        if shutil.which(tool) is None:
            missing.append(tool)
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


_UNSUPPORTED_GCC_FLAGS = ["-fcyclomatic-complexity"]


async def _compile_and_flash(
    firmware_dir: Path, port: str, log_fn: "Callable[[str], None]"
) -> None:
    """Compile and flash. Raises FlashError on failure."""
    build_dir = firmware_dir / "Debug"

    # Strip flags unsupported by older arm-none-eabi-gcc (e.g. -fcyclomatic-complexity
    # was added in GCC 12 but many distros ship GCC 10).  These flags live in
    # STM32CubeIDE-generated *.mk files in the build directory.
    for mk_file in build_dir.rglob("*.mk"):
        try:
            text = mk_file.read_text()
            patched = text
            for flag in _UNSUPPORTED_GCC_FLAGS:
                patched = patched.replace(f" {flag}", "")
            if patched != text:
                mk_file.write_text(patched)
        except OSError:
            pass

    log_fn("Compiling (make -j4)...")
    rc, out = await _run_subprocess(["make", "-j4", "all"], cwd=build_dir)
    if rc != 0:
        raise FlashError(f"Compilation failed:\n{out[-3000:]}")
    log_fn("Compilation OK.")

    elfs = list(build_dir.glob("*.elf"))
    if not elfs:
        raise FlashError(f"No .elf found in {build_dir}")
    elf_path = elfs[0]

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
    """Singleton that drives the flash wizard through its 3-pass state machine."""

    def __init__(self) -> None:
        self.status = FlashStatus()
        self._lock = asyncio.Lock()
        self._task: asyncio.Task | None = None
        self._confirm_event: asyncio.Event | None = None
        self._confirmed_correct: bool | None = None
        self._power_cycle_event: asyncio.Event | None = None
        self._can_connect_event: asyncio.Event | None = None
        self._current_channel: str | None = None
        self._current_can_id: int | None = None

    @property
    def current_channel(self) -> str | None:
        """CAN channel being used by the active flash session, or None if idle."""
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
                    f"Required tools not found on PATH: {', '.join(missing)}. "
                    "Run: sudo apt install openocd gcc-arm-none-eabi"
                )

            # Validate motor profile early
            config.profile_data()

            self.status = FlashStatus()
            self._current_channel = config.can_channel
            self._current_can_id = config.can_id

        self._task = asyncio.create_task(self._run_session(port, config))

    async def reset(self) -> None:
        """Force state back to IDLE, cancelling any in-progress session."""
        async with self._lock:
            if self._task is not None and not self._task.done():
                self._task.cancel()
                self._task = None
            self.status = FlashStatus()
            self._current_channel = None
            self._current_can_id = None

    async def power_cycled(self) -> None:
        """Called when the frontend has detected or the user confirms the ESC is back online."""
        if self.status.state != FlashState.WAITING_POWER_CYCLE:
            raise FlashError("Not waiting for a power cycle")
        if self._power_cycle_event is not None:
            self._power_cycle_event.set()

    async def can_connected(self) -> None:
        """Called when the frontend confirms motor + CAN + encoder are connected."""
        if self.status.state != FlashState.WAITING_CAN_CONNECT:
            raise FlashError("Not waiting for CAN connection confirmation")
        if self._can_connect_event is not None:
            self._can_connect_event.set()

    async def can_ping(self) -> dict:
        """
        Open a temporary CAN socket and check whether the target device responds
        to an SDO read.  Returns {reachable, detail}.
        Safe to call while state == WAITING_CAN_CONNECT (bus is not open yet).
        """
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
        profile_define = profile["define"]
        firmware_dir = config.firmware_dir
        invert_phase = config.invert_phase

        # ── PASS 1: INIT_FLASH (USB only) ────────────────────────────────────
        self.status.state = FlashState.INIT_FLASH
        self._log("Pass 1 — programming Flash option bytes...", progress=5)

        original = _patch_conf_h(
            firmware_dir,
            first_time_bootup=1, load_id=1, load_config=1, load_calibration=1,
            can_id=config.can_id, invert_phase=invert_phase,
            motor_profile_define=profile_define,
        )
        try:
            await _compile_and_flash(firmware_dir, port, self._log)
        finally:
            _restore_conf_h(firmware_dir, original)
            self._log("motor_controller_conf.h restored.")

        self._log(
            "Pass 1 complete. Firmware entered halt loop (FIRST_TIME_BOOTUP=1). "
            "Power cycle the ESC now (disconnect/reconnect motor power).",
            progress=18,
        )

        # ── WAITING_POWER_CYCLE ───────────────────────────────────────────────
        self.status.state = FlashState.WAITING_POWER_CYCLE
        self._power_cycle_event = asyncio.Event()
        self._log(
            "Waiting for power cycle. Click 'Power Cycled' when done. (300 s timeout)",
            progress=20,
        )
        await asyncio.wait_for(self._power_cycle_event.wait(), timeout=300.0)
        self._log("Power cycle confirmed. Waiting 2 s for boot...", progress=22)
        await asyncio.sleep(2.0)

        # Reflash loop — repeats if direction test fails
        while True:
            # ── PASS 2: PROGRAM_FLASH (USB only) ─────────────────────────────
            self.status.state = FlashState.PROGRAM_FLASH
            self._log(
                f"Pass 2 — writing CAN ID {config.can_id}, "
                f"phase_order={'inverted' if invert_phase else 'normal'}, "
                f"motor profile {profile_define}...",
                progress=25,
            )

            original = _patch_conf_h(
                firmware_dir,
                first_time_bootup=0, load_id=0, load_config=0, load_calibration=0,
                can_id=config.can_id, invert_phase=invert_phase,
                motor_profile_define=profile_define,
            )
            try:
                await _compile_and_flash(firmware_dir, port, self._log)
            finally:
                _restore_conf_h(firmware_dir, original)
                self._log("motor_controller_conf.h restored.")

            self._log(
                "Pass 2 complete. ESC running with correct CAN ID. "
                "Now connect motor, CAN bus, and encoder.",
                progress=45,
            )

            # ── WAITING_CAN_CONNECT ───────────────────────────────────────────
            self.status.state = FlashState.WAITING_CAN_CONNECT
            self._can_connect_event = asyncio.Event()
            self._log(
                "Waiting for motor + CAN + encoder connection. "
                "Click 'Motor connected' when ready. (600 s timeout)",
                progress=47,
            )
            await asyncio.wait_for(self._can_connect_event.wait(), timeout=600.0)
            self._log("CAN connection confirmed. Connecting to CAN bus...", progress=48)

            # Open CAN bus only after user confirms hardware is connected
            bus = CANBus(channel=config.can_channel)
            await bus.connect()
            # ESC floods the bus with PDO4 frames for ~1 s after boot; wait for that
            # burst to drain before sending SDO writes or the TX queue fills (ENOBUFS).
            self._log("Waiting for CAN bus to settle...", progress=49)
            await asyncio.sleep(2.0)
            joint_cfg = JointConfig(
                joint_name="__flash_target__",
                can_channel=config.can_channel,
                can_id=config.can_id,
                phase_inverted=invert_phase,
            )
            actuator = Actuator(bus, joint_cfg)

            try:
                # ── CALIBRATING ───────────────────────────────────────────────
                self.status.state = FlashState.CALIBRATING
                self._log("Setting fast_frame_frequency=100 Hz via CAN SDO...", progress=50)
                await bus.write_parameter_u32(config.can_id, Parameter.FAST_FRAME_FREQUENCY, 100)
                self._log(
                    "Starting encoder flux offset calibration (NMT MODE_CALIBRATION). "
                    "This takes ~15 s. Do NOT power off the ESC.",
                    progress=52,
                )

                def _on_cal_progress(msg: str) -> None:
                    self._log(msg)

                flux_offset = await actuator.calibrate_offset(timeout=90.0, on_progress=_on_cal_progress)
                self.status.flux_offset = flux_offset
                self._log(
                    f"Calibration done: flux_offset = {flux_offset:.4f} rad "
                    f"({math.degrees(flux_offset):.2f}°). Firmware has saved this to Flash.",
                    progress=70,
                )

                # ── AWAITING_CONFIRMATION ─────────────────────────────────────
                self.status.state = FlashState.AWAITING_CONFIRMATION
                self._confirm_event = asyncio.Event()
                self._confirmed_correct = None
                self._log(
                    "Did the motor rotate during calibration? Confirm direction in the UI. (120 s timeout)",
                    progress=72,
                )
                await asyncio.wait_for(self._confirm_event.wait(), timeout=120.0)
                direction_correct = bool(self._confirmed_correct)

            finally:
                await bus.disconnect()

            if direction_correct:
                break

            # Direction wrong — toggle phase and repeat from Pass 2
            self.status.state = FlashState.REFLASHING
            invert_phase = not invert_phase
            self._log(
                f"Direction wrong — re-running Pass 2 with invert_phase={invert_phase}...",
                progress=74,
            )

        # ── PASS 3: FINALIZE_FLASH (USB only) ────────────────────────────────
        self.status.state = FlashState.FINALIZE_FLASH
        self._log("Pass 3 — writing operational firmware (all LOAD flags=1)...", progress=84)

        original = _patch_conf_h(
            firmware_dir,
            first_time_bootup=0, load_id=1, load_config=1, load_calibration=1,
            can_id=config.can_id, invert_phase=invert_phase,
            motor_profile_define=profile_define,
        )
        try:
            await _compile_and_flash(firmware_dir, port, self._log)
        finally:
            _restore_conf_h(firmware_dir, original)
            self._log("motor_controller_conf.h restored.")

        self._log("Pass 3 complete. ESC will load calibration from Flash on every boot.", progress=94)

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
