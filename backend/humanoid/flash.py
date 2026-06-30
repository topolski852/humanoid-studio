"""
Flash wizard state machine for B-G431B-ESC1 boards.

Single-pass firmware procedure:

  1. FLASHING:
     Flash production_{motor_profile}.elf via openocd/ST-LINK, then write a
     valid config page so loadConfig succeeds.  The config page encodes
     device_id=target and all motor profile params (i_kp, i_ki, pole_pairs,
     cpr, phase_order, etc.) at the correct MotorController struct offsets.
     The ESC boots at the target CAN ID because the production firmware uses
     CAN_init(filter=0, mask=0) (accept-all FDCAN hardware filter) and reads
     device_id from the config page.

  2. COMMISSIONING:
     SDO-write all motor profile params plus device_id to the target CAN ID
     (firmware ACKs writes).  These writes reinforce what the config page
     encoded and correct any values that may be stale from a prior session.
     Save config to flash via FUNC_FLASH so all params are persistent.

  3. CALIBRATING:
     At the target CAN ID: run encoder flux offset calibration.

  4. AWAITING_CONFIRMATION:
     User confirms motor direction is correct.
     If wrong: toggle phase order, save, re-calibrate.

  5. COMPLETE.

NOTE: The prebuilt commissioning_*.elf files (old firmware) are NOT used.
They have a hardcoded FDCAN filter for device_id=127 and cannot communicate
at any other CAN ID.  The production ELF (compiled with CAN_init(0,0))
accepts all frames and routes by software device_id check.  A separate ELF
per motor profile ensures the correct compile-time defaults are present even
if the config page is ever erased.
"""
from __future__ import annotations

import asyncio
import math
import re
import shutil
import struct
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path

from pydantic import BaseModel

from .daemon_client import DaemonClient


# ---------------------------------------------------------------------------
# Recoil protocol constants (subset needed by flash wizard)
# ---------------------------------------------------------------------------

_PARAM_DEVICE_ID              = 0x000
_PARAM_WATCHDOG_TIMEOUT       = 0x008
_PARAM_FAST_FRAME_FREQUENCY   = 0x00C
_PARAM_MODE                   = 0x010
_PARAM_ERROR                  = 0x014
_PARAM_POSITION_GEAR_RATIO    = 0x01C
_PARAM_CURRENT_I_LIMIT        = 0x074
_PARAM_CURRENT_I_KP           = 0x078
_PARAM_CURRENT_I_KI           = 0x07C
_PARAM_MOTOR_POLE_PAIRS       = 0x104
_PARAM_MOTOR_TORQUE_CONSTANT  = 0x108
_PARAM_MOTOR_PHASE_ORDER      = 0x10C
_PARAM_MOTOR_MAX_CALIBRATION  = 0x110
_PARAM_ENCODER_CPR            = 0x120
_PARAM_ENCODER_FLUX_OFFSET    = 0x13C

_MODE_NAMES: dict[int, str] = {
    0x00: "DISABLED", 0x01: "IDLE", 0x02: "DAMPING",
    0x05: "CALIBRATION", 0x10: "CURRENT", 0x11: "TORQUE",
    0x12: "VELOCITY", 0x13: "POSITION", 0x80: "DEBUG",
}
_ERROR_BITS: dict[int, str] = {
    0x0001: "GENERAL", 0x0002: "ESTOP", 0x0004: "INITIALIZATION_ERROR",
    0x0008: "CALIBRATION_ERROR", 0x0010: "POWERSTAGE_ERROR",
    0x0020: "INVALID_MODE", 0x0040: "WATCHDOG_TIMEOUT",
    0x0080: "OVER_VOLTAGE", 0x0100: "OVER_CURRENT",
    0x0200: "OVER_TEMPERATURE", 0x0400: "CAN_RX_FAULT",
    0x0800: "CAN_TX_FAULT", 0x1000: "I2C_FAULT", 0x2000: "ENCODER_FAULT",
}

def _mode_name(byte: int) -> str:
    return _MODE_NAMES.get(byte, f"0x{byte:02X}")

def _error_str(bits: int) -> str:
    names = [n for bit, n in _ERROR_BITS.items() if bits & bit]
    return " | ".join(names) if names else "NO_ERROR"

def _parse_version_str(v: str) -> int | None:
    """Parse 'v3.0.1' or '3.0.1' into 0x03000001 (8-8-16: MAJOR·MINOR·PATCH)."""
    m = re.match(r'v?(\d+)\.(\d+)\.(\d+)', v.strip())
    if not m:
        return None
    major, minor, patch = int(m.group(1)), int(m.group(2)), int(m.group(3))
    return (major << 24) | (minor << 16) | patch


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
        "pole_pairs":             14,
        "cpr":                    4096,
    },
    "MAD_5010_110KV": {
        "torque_constant":        0.1176,
        "phase_resistance":       0.6193,
        "phase_inductance":       8.50e-5,
        "max_calibration_current": 3,
        "i_kp":                   _gains(0.6193, 8.50e-5)[0],
        "i_ki":                   _gains(0.6193, 8.50e-5)[1],
        "pole_pairs":             14,
        "cpr":                    4096,
    },
    "MAD_5010_200KV": {
        "torque_constant":        0.06588,
        "phase_resistance":       0.15227,
        "phase_inductance":       2.649166e-5,
        "max_calibration_current": 3,
        "i_kp":                   _gains(0.15227, 2.649166e-5)[0],
        "i_ki":                   _gains(0.15227, 2.649166e-5)[1],
        "pole_pairs":             14,
        "cpr":                    4096,
    },
    "MAD_5010_310KV": {
        "torque_constant":        0.04212,
        "phase_resistance":       0.05735,
        "phase_inductance":       3.3256e-5,
        "max_calibration_current": 5,
        "i_kp":                   _gains(0.05735, 3.3256e-5)[0],
        "i_ki":                   _gains(0.05735, 3.3256e-5)[1],
        "pole_pairs":             14,
        "cpr":                    4096,
    },
    "MAD_5010_370KV": {
        "torque_constant":        0.03529,
        "phase_resistance":       0.03000,
        "phase_inductance":       1.0717e-5,
        "max_calibration_current": 5,
        "i_kp":                   _gains(0.03000, 1.0717e-5)[0],
        "i_ki":                   _gains(0.03000, 1.0717e-5)[1],
        "pole_pairs":             14,
        "cpr":                    4096,
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
            FlashState.WAITING_CAN_CONNECT:   2,
            FlashState.COMMISSIONING:         3,
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
    skip_flash: bool = False

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


async def _check_can_interface_state(channel: str) -> str | None:
    """
    Read the host-side CAN interface state via `ip -details link show`.
    Returns 'BUS-OFF', 'ERROR-PASSIVE', 'ERROR-WARNING', or None (healthy).
    Any of these states can cause silent TX frame loss, causing SDO reads/writes
    to time out even when the ESC firmware is perfectly healthy.

    ERROR-WARNING is the most common culprit: the Linux CAN adapter's TX queue
    fills with frames that can't be sent (e.g., after repeated BUS-OFF recovery
    cycles), so all subsequent writes get ENOBUFS at the socket layer and never
    reach the bus.  Bouncing the interface (down + up) clears the queue.
    """
    try:
        proc = await asyncio.create_subprocess_exec(
            "ip", "-details", "link", "show", channel,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.DEVNULL,
        )
        stdout, _ = await proc.communicate()
        text = stdout.decode(errors="replace")
        if "BUS-OFF" in text or "bus-off" in text:
            return "BUS-OFF"
        if "ERROR-PASSIVE" in text or "error-passive" in text:
            return "ERROR-PASSIVE"
        if "ERROR-WARNING" in text or "error-warning" in text:
            return "ERROR-WARNING"
    except Exception:
        pass
    return None



_FLASH_CONFIG_ADDRESS = 0x0801F800   # STM32G431CB Bank1 page 63 — MotorController config
_FLASH_CONFIG_SIZE    = 0x800        # 2 KB (one page)

# RAM address of the `controller` global (MotorController struct).
# Offsets match the Parameter enum byte offsets (DEVICE_ID=0x000, MODE=0x010, ERROR=0x014).
_CONTROLLER_BASE = 0x20000210

_COMMISSIONING_CONFIG_PATH = Path("/tmp/esc_commissioning_config.bin")


def _make_commissioning_config_page(
    profile: dict,
    target_device_id: int = 127,
    invert_phase: bool = False,
) -> bytes:
    """
    Build a 2 KB valid STM32G431 flash config page for the production ELF.

    The firmware's storeConfig/loadConfig copies the entire MotorController
    struct to/from flash verbatim.  Byte offsets here MUST match the Parameter
    enum values in motor_controller.h (which equal the struct field byte offsets).

    Without this, a plain erase (all 0xFF = NaN) causes loadConfig to return
    HAL_ERROR on the first NaN check and the firmware enters an infinite error
    loop with no CAN response.
    """
    page = bytearray(_FLASH_CONFIG_SIZE)  # 2048 bytes, all 0x00

    torque_constant    = profile["torque_constant"]
    max_calib_current  = float(profile["max_calibration_current"])
    pole_pairs         = int(profile.get("pole_pairs", 14))
    cpr                = int(profile.get("cpr", 4096))
    i_kp               = float(profile["i_kp"])
    i_ki               = float(profile["i_ki"])
    phase_order        = -1 if invert_phase else 1

    # All offsets = Parameter enum values = MotorController struct byte offsets.
    struct.pack_into('<I', page, 0x000, target_device_id)    # device_id (uint8_t + 3B pad)
    # 0x004: firmware_version — 0x0 (firmware uses its compiled-in value, not this)
    struct.pack_into('<I', page, 0x008, 30000)                # watchdog_timeout (ms) — 30 s
    struct.pack_into('<I', page, 0x00C, 0)                    # fast_frame_frequency (set via SDO after boot)
    # 0x010: mode — 0x00 DISABLED; firmware transitions to IDLE after init
    # 0x014: error — 0 (no error)
    # 0x018: position_controller.update_counter — 0

    struct.pack_into('<f', page, 0x01C, 1.0)                  # gear_ratio = 1.0 (direct drive)
    # 0x020-0x06C: position PID gains, limits, integrators — 0.0 fine

    struct.pack_into('<f', page, 0x070, 0.1)                  # torque_filter_alpha

    # i_limit MUST be non-zero — calibration sweep cannot flow current otherwise
    struct.pack_into('<f', page, 0x074, max_calib_current * 2.0)   # i_limit
    struct.pack_into('<f', page, 0x078, i_kp)                       # i_kp
    struct.pack_into('<f', page, 0x07C, i_ki)                       # i_ki
    # 0x080-0x0D4: current controller measured/setpoint values — 0.0 fine
    # 0x0D8-0x0EC: HAL handles/ADC buffers — 0 fine (HAL reinitialises at boot)

    # 0x0F4: undervoltage_threshold — 0.0 disables undervoltage fault
    # 0x0F8: overvoltage_threshold  — 0.0 disables overvoltage fault
    struct.pack_into('<f', page, 0x0FC, 0.01)                 # bus_voltage_filter_alpha
    # 0x100: bus_voltage_measured — 0.0 fine

    struct.pack_into('<I', page, 0x104, pole_pairs)            # motor.pole_pairs
    struct.pack_into('<f', page, 0x108, torque_constant)       # motor.torque_constant
    struct.pack_into('<i', page, 0x10C, phase_order)           # motor.phase_order (int32)
    struct.pack_into('<f', page, 0x110, max_calib_current)     # motor.max_calibration_current

    # 0x114: encoder.hi2c  — HAL handle, set at boot; 0 fine
    # 0x118-0x11B: encoder.i2c_buffer — 0 fine
    # 0x11C: encoder.i2c_update_counter — 0

    struct.pack_into('<I', page, 0x120, cpr)                   # encoder.cpr
    # 0x124: encoder.position_offset — 0.0 (set after calibration)
    # 0x128: encoder.velocity_filter_alpha — 0.0 fine
    # 0x12C-0x138: encoder runtime values — 0.0 fine
    # 0x13C: encoder.flux_offset — 0.0 (written by calibration sequence)
    # 0x140+: flux offset lookup table — 0.0 fine

    return bytes(page)


async def _flash_prebuilt(
    elf_path: Path,
    firmware_dir: Path,
    log_fn,
    profile: dict,
    target_device_id: int = 127,
    invert_phase: bool = False,
) -> None:
    phase_str = "inverted (-1)" if invert_phase else "normal (+1)"
    log_fn(
        f"Flashing {elf_path.name} via ST-LINK (openocd) — "
        f"config page: device_id={target_device_id}, phase_order={phase_str}, "
        f"i_kp={profile['i_kp']:.4f}, i_ki={profile['i_ki']:.3f}. "
        f"ESC will boot at CAN ID {target_device_id}."
    )

    config_page = _make_commissioning_config_page(
        profile, target_device_id=target_device_id, invert_phase=invert_phase)
    _COMMISSIONING_CONFIG_PATH.write_bytes(config_page)

    # Three-step openocd sequence:
    #   1. Program the ELF (code pages only — does NOT touch the config page).
    #   2. Write a valid commissioning config page at 0x0801F800 so loadConfig
    #      succeeds at boot.  A plain erase (all 0xFF = NaN) causes loadConfig
    #      to return HAL_ERROR, which puts the firmware into an infinite UART
    #      loop with no CAN response.
    #   3. Verify the config page write to catch any silent flash failures.
    rc, out = await _run_subprocess(
        [
            "openocd",
            "-f", "interface/stlink.cfg",
            "-f", "target/stm32g4x.cfg",
            "-c", f"program {elf_path} verify",
            "-c", "halt",
            "-c", (
                f"flash write_image erase {_COMMISSIONING_CONFIG_PATH} "
                f"{_FLASH_CONFIG_ADDRESS:#010x} bin"
            ),
            "-c", (
                f"verify_image {_COMMISSIONING_CONFIG_PATH} "
                f"{_FLASH_CONFIG_ADDRESS:#010x} bin"
            ),
            "-c", "reset run",
            "-c", "shutdown",
        ],
        cwd=firmware_dir,
    )
    # Always log openocd output so flash write and verify steps are visible
    _openocd_summary = "\n".join(
        l for l in out.splitlines()
        if any(kw in l.lower() for kw in ("wrote", "verified", "error", "warn", "failed", "verify"))
    ) or out[-500:]
    log_fn(f"openocd output:\n{_openocd_summary}")
    if rc != 0:
        out_lower = out.lower()
        if ("stlink" in out_lower or "st-link" in out_lower) and \
           any(kw in out_lower for kw in ("outdated", "upgrade", "too old", "does not support")):
            raise FlashError("stlink-firmware-outdated")
        raise FlashError(f"Flash failed:\n{out[-3000:]}")
    log_fn(f"Flash written and config page verified — device is resetting (will boot at ID {target_device_id}).")


async def _read_controller_state_via_swd(firmware_dir: Path) -> dict | None:
    """
    Halt the ESC via SWD/ST-LINK and read controller struct fields from SRAM
    plus FDCAN1 hardware registers.  Returns a dict or None if openocd fails.

    SRAM fields (6 words at _CONTROLLER_BASE):
      device_id, firmware_version, watchdog_timeout, fast_frame_freq, mode, error

    FDCAN1 peripheral registers (STM32G431, base 0x40006400):
      ECR (0x40006440): TEC [7:0], REC [14:8]
      PSR (0x40006444): EP [5], EW [6], BO [7]
    """
    rc, out = await _run_subprocess(
        [
            "openocd",
            "-f", "interface/stlink.cfg",
            "-f", "target/stm32g4x.cfg",
            "-c", "init",
            "-c", "halt",
            "-c", f"mdw {_CONTROLLER_BASE:#010x} 6",
            "-c", "mdw 0x40006440 2",   # FDCAN1 ECR then PSR
            "-c", "resume",
            "-c", "shutdown",
        ],
        cwd=firmware_dir,
    )
    if rc != 0:
        return None

    m = re.search(r'0x20000210:\s+([0-9a-fA-F ]+)', out)
    if not m:
        return None
    words_str = m.group(1).split()[:6]
    if len(words_str) < 6:
        return None
    words = [int(w, 16) for w in words_str]
    result: dict = {
        "device_id":        words[0],
        "firmware_version": words[1],
        "watchdog_timeout": words[2],
        "fast_frame_freq":  words[3],
        "mode":             words[4],
        "error":            words[5],
    }

    m2 = re.search(r'0x40006440:\s+([0-9a-fA-F ]+)', out)
    if m2:
        fdcan = m2.group(1).split()[:2]
        if len(fdcan) >= 2:
            ecr, psr = int(fdcan[0], 16), int(fdcan[1], 16)
            result["can_tec"] = ecr & 0xFF
            result["can_rec"] = (ecr >> 8) & 0x7F
            result["can_ep"]  = bool((psr >> 5) & 1)
            result["can_ew"]  = bool((psr >> 6) & 1)
            result["can_bo"]  = bool((psr >> 7) & 1)

    return result


def _format_fdcan_state(swd: dict) -> str:
    """Return a short CAN hardware state string from SWD diagnostic dict, or ''."""
    if "can_bo" not in swd:
        return ""
    if swd["can_bo"]:
        return f"BUS_OFF (TEC={swd['can_tec']})"
    if swd["can_ep"]:
        return f"ERROR_PASSIVE (TEC={swd['can_tec']}, REC={swd['can_rec']})"
    if swd["can_ew"]:
        return f"ERROR_WARNING (TEC={swd['can_tec']}, REC={swd['can_rec']})"
    return f"OK (TEC={swd['can_tec']}, REC={swd['can_rec']})"


# ---------------------------------------------------------------------------
# FlashManager
# ---------------------------------------------------------------------------

class FlashManager:
    """Drives the flash wizard through its state machine."""

    def __init__(self, daemon_client: DaemonClient) -> None:
        self._daemon_client = daemon_client
        self.status = FlashStatus()
        self._lock = asyncio.Lock()
        self._task: asyncio.Task | None = None
        self._confirm_event: asyncio.Event | None = None
        self._confirmed_correct: bool | None = None
        self._can_connect_event: asyncio.Event | None = None
        self._current_channel: str | None = None
        self._current_can_id: int | None = None
        self._current_firmware_dir: Path | None = None

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

            if not config.skip_flash:
                missing = _check_tools()
                if missing:
                    raise FlashError(
                        f"Required tool not found on PATH: {', '.join(missing)}. "
                        "Install openocd and ensure it is on PATH."
                    )
                elf_path = config.firmware_dir / "prebuilt" / f"production_{config.motor_profile}.elf"
                if not elf_path.exists():
                    raise FlashError(
                        f"Firmware not found: {elf_path}. "
                        f"Expected firmware/esc/prebuilt/production_{config.motor_profile}.elf"
                    )

            config.profile_data()

            self.status = FlashStatus()
            self._current_channel = config.can_channel
            self._current_can_id = config.can_id
            self._current_firmware_dir = config.firmware_dir

        self._task = asyncio.create_task(self._run_session(port, config))

    async def reset(self) -> None:
        async with self._lock:
            if self._task is not None and not self._task.done():
                self._task.cancel()
                self._task = None
            self.status = FlashStatus()
            self._current_channel = None
            self._current_can_id = None
            self._current_firmware_dir = None

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
        if not channel:
            raise FlashError("No CAN channel configured")

        target_id = self._current_can_id
        dc = self._daemon_client
        loop = asyncio.get_running_loop()

        try:
            # Check commissioning ID (127) via heartbeat.
            hb = await loop.run_in_executor(
                None, dc.wait_heartbeat, channel, _COMMISSIONING_CAN_ID, 2000)
            if hb is not None:
                return {
                    "reachable": True,
                    "detail": (
                        f"ESC at commissioning ID {_COMMISSIONING_CAN_ID}: "
                        f"mode={_mode_name(hb['mode'])}, "
                        f"error={_error_str(hb['error'])}"
                    ),
                }

            # Check target ID as fallback (previously commissioned ESC).
            if target_id is not None and target_id != _COMMISSIONING_CAN_ID:
                hb2 = await loop.run_in_executor(
                    None, dc.wait_heartbeat, channel, target_id, 2000)
                if hb2 is not None:
                    return {
                        "reachable": True,
                        "detail": (
                            f"ESC at target ID {target_id} (prior commission): "
                            f"mode={_mode_name(hb2['mode'])}, "
                            f"error={_error_str(hb2['error'])}"
                        ),
                    }

            # No heartbeat — try SWD diagnostic while ST-LINK may be connected.
            swd_detail = ""
            fw_dir = self._current_firmware_dir
            if fw_dir is not None:
                swd = await _read_controller_state_via_swd(fw_dir)
                if swd is not None:
                    can_state = _format_fdcan_state(swd)
                    swd_detail = (
                        f" SWD: device_id={swd['device_id']}, "
                        f"firmware={swd['firmware_version']:#010x}, "
                        f"mode={_mode_name(swd['mode'])}, "
                        f"error={_error_str(swd['error'])}"
                        + (f", CAN={can_state}" if can_state else "")
                    )
            return {
                "reachable": False,
                "detail": (
                    f"no heartbeat from ESC at ID {_COMMISSIONING_CAN_ID}"
                    + (f" or {target_id}" if target_id else "")
                    + f" on {channel} within 2 s"
                    + swd_detail
                ),
            }
        except Exception as exc:
            return {"reachable": False, "detail": str(exc)}

    async def check_firmware_version(self, firmware_dir: Path) -> dict:
        """
        Read ESC firmware version via SWD and compare to the VERSION file.
        Returns dict: match, esc_version_str, esc_version_hex, expected_version_str,
        expected_version_hex, error (None if successful).
        Requires ST-LINK to be connected and ESC to be powered.
        """
        version_file = firmware_dir / "VERSION"
        expected_int: int | None = None
        expected_str = "unknown"
        if version_file.exists():
            raw = version_file.read_text().strip()
            expected_int = _parse_version_str(raw)
            expected_str = raw

        swd = await _read_controller_state_via_swd(firmware_dir)
        if swd is None:
            return {
                "match": False,
                "esc_version_str": None,
                "esc_version_hex": None,
                "expected_version_str": expected_str,
                "expected_version_hex": (
                    f"{expected_int:#010x}" if expected_int is not None else "unknown"
                ),
                "error": "SWD read failed — ST-LINK not connected or ESC not powered.",
            }

        esc_ver = swd["firmware_version"]
        major = (esc_ver >> 24) & 0xFF
        minor = (esc_ver >> 16) & 0xFF
        patch = esc_ver & 0xFFFF
        esc_ver_str = f"v{major}.{minor}.{patch}"
        match = expected_int is not None and esc_ver == expected_int

        return {
            "match": match,
            "esc_version_str": esc_ver_str,
            "esc_version_hex": f"{esc_ver:#010x}",
            "expected_version_str": expected_str,
            "expected_version_hex": (
                f"{expected_int:#010x}" if expected_int is not None else "unknown"
            ),
            "error": None,
        }

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
        self._commissioning_joint_name: str | None = None
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
        finally:
            # Re-enable slow-poll telemetry for all joints (we suspended all).
            dc = self._daemon_client
            if dc is not None:
                try:
                    loop = asyncio.get_running_loop()
                    await loop.run_in_executor(None, dc.enable_all_slow_poll)
                except Exception:
                    pass

    async def _do_session(self, port: str, config: FlashConfig) -> None:
        profile = config.profile_data()
        firmware_dir = config.firmware_dir
        invert_phase = config.invert_phase

        if not config.skip_flash:
            # ── 1. Flash pre-compiled binary ──────────────────────────────────
            self.status.state = FlashState.FLASHING
            elf_path = firmware_dir / "prebuilt" / f"production_{config.motor_profile}.elf"
            self._log(f"Flashing production firmware for {config.motor_profile}...", progress=5)
            await _flash_prebuilt(
                elf_path, firmware_dir, self._log, profile,
                target_device_id=config.can_id,
                invert_phase=invert_phase,
            )
            self._log(
                f"Firmware flashed. Device booting at target CAN ID {config.can_id} "
                f"(config page encodes device_id={config.can_id})...",
                progress=20)
        else:
            self._log(
                "Skipping flash step — using firmware already on the ESC.",
                progress=20,
            )

        # ── 2. Wait for motor + CAN hardware connection ───────────────────────
        if not config.skip_flash:
            # After SWD flash a full power-on reset (POR) is required so the
            # FDCAN peripheral initialises cleanly.  `openocd reset run` only
            # resets the CPU core via the debug interface and does not guarantee
            # the same peripheral state as POR.
            self._log(
                "Firmware flashed. Power cycle the ESC (disconnect then reconnect its "
                "power supply), then connect the CAN bus cable, motor phase wires, and "
                "encoder cable. Click 'Hardware connected' when ready.",
                progress=25,
            )
        else:
            self._log(
                "Power cycle the ESC (disconnect then reconnect its power supply), "
                "then connect the CAN bus cable, motor phase wires, and encoder cable. "
                "Click 'Hardware connected' when ready.",
                progress=25,
            )
        self.status.state = FlashState.WAITING_CAN_CONNECT
        self._can_connect_event = asyncio.Event()
        self._log(
            "Waiting for motor + CAN + encoder connection. (600 s timeout)",
            progress=27,
        )
        await asyncio.wait_for(self._can_connect_event.wait(), timeout=600.0)
        self._log("Connection confirmed. Commissioning ESC over CAN...", progress=40)

        # ── 3. Commission over CAN ───────────────────────────────────────────
        self.status.state = FlashState.COMMISSIONING
        loop = asyncio.get_running_loop()
        comm_id = _COMMISSIONING_CAN_ID
        dc = self._daemon_client

        # Feed watchdog at config.can_id — with production_{motor_profile}.elf the ESC
        # always boots at config.can_id (encoded in the config page), so comm_id=127 is
        # never the right address. Feeding the wrong ID causes WATCHDOG_TIMEOUT → DAMPING.
        feed_id = config.can_id
        self._log(
            f"Sending watchdog feed to ID {feed_id} on {config.can_channel}...",
            progress=41)
        await loop.run_in_executor(
            None, dc.feed_watchdog_generic, config.can_channel, feed_id)
        await asyncio.sleep(0.5)

        # Passive sniff: collect all CAN frames for 3 s.
        self._log(f"Sniffing CAN bus on {config.can_channel} for 3 s...", progress=42)
        raw_frames = await loop.run_in_executor(
            None, dc.sniff_bus, config.can_channel, 3000)

        _sniff_saw_comm_id = False
        _sniff_saw_target_id = False
        # EMCY (func_id=0x1 = FUNC_SYNC_EMCY) can come from ANY device_id, not just the target.
        # An ESC in DAMPING/error state broadcasts EMCY instead of heartbeats.
        # Key: device_id, value: first seen error_code (4-byte LE uint32).
        _sniff_emcy_by_device: dict[int, int] = {}
        _hb_count_comm = 0
        _hb_last_mode: int | None = None
        _hb_last_error: int | None = None
        _hb_target_mode: int | None = None
        _hb_target_error: int | None = None
        _frame_counts: dict[tuple[int, int], int] = {}
        for fr in raw_frames:
            dev  = fr.get("device_id", -1)
            func = fr.get("func_id", -1)
            _frame_counts[(dev, func)] = _frame_counts.get((dev, func), 0) + 1
            if dev == comm_id and func == 0xE:
                _sniff_saw_comm_id = True
                _hb_count_comm += 1
                if _hb_last_mode is None:
                    data_hex = fr.get("data", "")
                    if len(data_hex) >= 10:
                        try:
                            raw_bytes = bytes.fromhex(data_hex)
                            _hb_last_mode = raw_bytes[0]
                            _hb_last_error = struct.unpack_from("<I", raw_bytes, 1)[0]
                        except Exception:
                            pass
            if dev == config.can_id and func == 0xE and config.can_id != comm_id:
                _sniff_saw_target_id = True
                if _hb_target_mode is None:
                    data_hex = fr.get("data", "")
                    if len(data_hex) >= 10:
                        try:
                            raw_bytes = bytes.fromhex(data_hex)
                            _hb_target_mode = raw_bytes[0]
                            _hb_target_error = struct.unpack_from("<I", raw_bytes, 1)[0]
                        except Exception:
                            pass
            if func == 0x1 and dev not in _sniff_emcy_by_device:
                try:
                    raw_bytes = bytes.fromhex(fr.get("data", ""))
                    _sniff_emcy_by_device[dev] = (
                        struct.unpack_from("<I", raw_bytes, 0)[0]
                        if len(raw_bytes) >= 4 else 0
                    )
                except Exception:
                    _sniff_emcy_by_device[dev] = 0

        if raw_frames:
            _FUNC_NAMES = {0x1: "EMCY", 0x9: "PDO2", 0xA: "PDO3", 0xB: "SDO_TX",
                           0xC: "SDO_RX", 0xD: "FLASH", 0xE: "HB", 0xF: "NMT"}
            device_summary: dict[int, list[str]] = {}
            for (dev, func), count in sorted(_frame_counts.items()):
                label = _FUNC_NAMES.get(func, f"0x{func:X}")
                device_summary.setdefault(dev, []).append(
                    f"{label}×{count}" if count > 1 else label)
            parts = [f"ID {dev}:[{' '.join(fs)}]"
                     for dev, fs in sorted(device_summary.items())]
            self._log(f"Sniff result ({len(raw_frames)} frames): {', '.join(parts)}")
            if _sniff_saw_comm_id:
                mode_str = _mode_name(_hb_last_mode) if _hb_last_mode is not None else "?"
                err_str  = _error_str(_hb_last_error) if _hb_last_error is not None else "?"
                self._log(
                    f"  ↳ ID {comm_id} (unexpected — ESC booted with stale device_id=127): "
                    f"{_hb_count_comm} heartbeat(s) — mode={mode_str}, error={err_str}")
            if _sniff_saw_target_id:
                mode_str = _mode_name(_hb_target_mode) if _hb_target_mode is not None else "?"
                err_str  = _error_str(_hb_target_error) if _hb_target_error is not None else "?"
                self._log(f"  ↳ ID {config.can_id} (target): heartbeat seen — mode={mode_str}, error={err_str}")
            for emcy_dev, emcy_err in sorted(_sniff_emcy_by_device.items()):
                if emcy_dev == config.can_id:
                    self._log(
                        f"  ↳ ID {emcy_dev} (target): EMCY — "
                        f"error={_error_str(emcy_err)} (DAMPING/error state)")
                elif emcy_dev == comm_id:
                    self._log(
                        f"  ↳ ID {emcy_dev} (commissioning): EMCY — "
                        f"error={_error_str(emcy_err)}")
                else:
                    self._log(
                        f"  ↳ ID {emcy_dev} (unexpected): EMCY — "
                        f"error={_error_str(emcy_err)} "
                        f"(ESC has stale device_id={emcy_dev} from prior commissioning; "
                        f"will re-commission to target ID={config.can_id})")
        else:
            self._log(
                f"No CAN frames in 3 s on {config.can_channel}. "
                f"ESC may not be booted or CAN cable/termination is wrong.")

        # Determine which ID the ESC is currently at.
        active_id = comm_id
        esc_found = False
        esc_needs_idle_recovery = False  # True if ESC is in error/DAMPING — must reset before writes

        if _sniff_saw_target_id and config.can_id != comm_id:
            active_id = config.can_id
            esc_found = True
            self._log(
                f"ESC at target CAN ID {config.can_id} — "
                f"booted from config page (fresh flash) or previously commissioned.",
                progress=44)
        elif config.can_id in _sniff_emcy_by_device and config.can_id != comm_id:
            # ESC present at target ID but in error/DAMPING — broadcasts EMCY, not heartbeat.
            active_id = config.can_id
            esc_found = True
            esc_needs_idle_recovery = True
            self._log(
                f"ESC at target CAN ID {config.can_id} (detected via EMCY broadcast). "
                f"Will send NMT IDLE to recover before writing params.",
                progress=44)
        elif _sniff_saw_comm_id:
            esc_found = True
        else:
            # Check for EMCY from an unexpected device_id — ESC with stale device_id from a
            # prior commissioning run.  active_id is the current ID; code below will write
            # device_id=config.can_id and save to flash, re-commissioning without a re-flash.
            stale_ids = [d for d in _sniff_emcy_by_device
                         if d != config.can_id and d != comm_id]
            if stale_ids:
                active_id = stale_ids[0]   # use the first unexpected EMCY source
                esc_found = True
                esc_needs_idle_recovery = True
                self._log(
                    f"ESC found at unexpected CAN ID {active_id} (target={config.can_id}). "
                    f"ESC has stale device_id from prior commissioning. "
                    f"Will re-commission: NMT IDLE → write device_id={config.can_id} → save.",
                    progress=44)

        if not esc_found and config.can_id != comm_id:
            hb = await loop.run_in_executor(
                None, dc.wait_heartbeat, config.can_channel, config.can_id, 1000)
            if hb is not None:
                active_id = config.can_id
                esc_found = True
                self._log(
                    f"ESC at target CAN ID {config.can_id} — "
                    f"booted from config page (fresh flash) or previously commissioned.",
                    progress=44)

        if not esc_found:
            self._log(
                f"Waiting for ESC at commissioning ID {comm_id} on {config.can_channel}...",
                progress=43)
            hb = await loop.run_in_executor(
                None, dc.wait_heartbeat, config.can_channel, comm_id, 30000)
            if hb is not None:
                esc_found = True
            else:
                self._log("ESC not on CAN — reading controller state via SWD for diagnosis...")
                swd = await _read_controller_state_via_swd(firmware_dir)
                if swd is not None:
                    can_state = _format_fdcan_state(swd)
                    self._log(
                        f"SWD: device_id={swd['device_id']}, "
                        f"firmware={swd['firmware_version']:#010x}, "
                        f"mode={_mode_name(swd['mode'])}, "
                        f"error={_error_str(swd['error'])}"
                        + (f", CAN={can_state}" if can_state else ""))
                    if swd.get("can_bo"):
                        raise FlashError(
                            f"ESC FDCAN is in BUS_OFF (TEC={swd.get('can_tec', '?')}). "
                            f"Power cycle the ESC to recover.")
                else:
                    self._log("SWD diagnostic unavailable — ST-LINK may not be connected.")
                raise FlashError(
                    f"ESC did not appear on {config.can_channel} within 30 s — "
                    f"check CAN cable, ESC power, and that it was power-cycled after flashing.")

        # Detect active motors on this CAN channel (PDO4 fast-frame, func_id=0x9).
        # Recoil ESCs in IDLE mode use a permissive FDCAN hardware filter; fast-frame
        # traffic from neighbour motors can saturate the ESC's receive FIFO and silently
        # drop incoming SDO request frames, causing all SDO reads to return 0 responses.
        # Abort now — before wasting 6+ seconds on doomed SDO attempts — so the user can
        # disconnect the dashboard first.  Threshold 30 frames / 3 s (10 Hz) is well
        # above 2 Hz heartbeats and well below the 65–200 Hz fast-frame rate.
        _FAST_FRAME_FUNC = 0x9   # FUNC_TRANSMIT_PDO_4
        _active_ids = sorted({
            dev for (dev, func), cnt in _frame_counts.items()
            if func == _FAST_FRAME_FUNC and dev != active_id and cnt >= 30
        })
        if _active_ids:
            # Attempt to silence them directly via NMT IDLE before giving up.
            # 'Disconnect' in the dashboard sends SET_ALL_MODE IDLE through the daemon's
            # state machine, but motors that miss the single NMT frame end up in DAMPING
            # (watchdog timeout) rather than IDLE — and DAMPING still broadcasts PDO4.
            # Sending NMT IDLE directly here is more reliable: NMT has the highest CAN
            # priority and is a 2-byte frame that the ESC's FDCAN filter will accept even
            # under heavy load.  Send 3× over 1.5 s so at least one gets through.
            self._log(
                f"Detected active motors {_active_ids} on {config.can_channel} — "
                f"sending NMT DISABLED to silence fast-frame broadcasts...",
                progress=46)
            for _nmt_pass in range(3):
                for _mid in _active_ids:
                    try:
                        await loop.run_in_executor(
                            None, dc.send_nmt, config.can_channel, _mid, 0x00)  # MODE_DISABLED
                    except Exception:
                        pass
                await asyncio.sleep(0.5)

            # Re-sniff for 1 s to check whether the motors went quiet.
            _recheck_raw = await loop.run_in_executor(
                None, dc.sniff_bus, config.can_channel, 1000)
            _recheck_counts: dict[tuple[int, int], int] = {}
            for _rf in _recheck_raw:
                _rdev  = _rf.get("device_id", -1)
                _rfunc = _rf.get("func_id", -1)
                _recheck_counts[(_rdev, _rfunc)] = _recheck_counts.get((_rdev, _rfunc), 0) + 1
            _still_active = sorted({
                dev for (dev, func), cnt in _recheck_counts.items()
                if func == _FAST_FRAME_FUNC and dev != active_id and cnt >= 10
            })
            if _still_active:
                raise FlashError(
                    f"Motor(s) {_still_active} on {config.can_channel} are still "
                    f"broadcasting fast frames after 3 NMT DISABLED attempts. "
                    f"Their CAN traffic saturates the target ESC's receive FIFO and "
                    f"prevents SDO communication. "
                    f"Click 'Disconnect' in the dashboard to put all motors in DISABLED "
                    f"(silent) mode, wait 2 s, then retry. "
                    f"If this persists, power cycle motors {_still_active}.")
            self._log(
                f"Motors {_active_ids} silenced — continuing commissioning.",
                progress=47)

        # If the ESC is in an error/DAMPING state, attempt recovery before writing params.
        # Strategy: disable watchdog + clear error register via SDO (firmware accepts writes
        # to any param even in DAMPING), then send NMT IDLE.  The watchdog re-fires every
        # timeout period if left enabled, keeping the ESC stuck in DAMPING.
        if esc_needs_idle_recovery:
            self._log(
                f"Recovering ESC at ID {active_id} from DAMPING/error state...",
                progress=44)
            # 1. Disable watchdog so it stops retriggering DAMPING.
            ack_wd = await loop.run_in_executor(
                None, dc.generic_sdo_write,
                config.can_channel, active_id, _PARAM_WATCHDOG_TIMEOUT, "u32", 0)
            self._log(
                f"  SDO[ID {active_id}] watchdog_timeout=0 (disable) → "
                f"{'ACK' if ack_wd else 'NO_ACK'}")
            # 2. Clear error register directly — firmware accepts SDO writes to any param.
            ack_err = await loop.run_in_executor(
                None, dc.generic_sdo_write,
                config.can_channel, active_id, _PARAM_ERROR, "u32", 0)
            self._log(
                f"  SDO[ID {active_id}] error=0 (clear) → "
                f"{'ACK' if ack_err else 'NO_ACK'}")
            await asyncio.sleep(0.1)
            # 3. Send NMT IDLE — now that the error register is cleared and watchdog is off,
            #    the firmware should accept the mode transition.
            await loop.run_in_executor(
                None, dc.send_nmt, config.can_channel, active_id, 0x01)  # MODE_IDLE
            await asyncio.sleep(1.0)
            hb_idle = await loop.run_in_executor(
                None, dc.wait_heartbeat, config.can_channel, active_id, 3000)
            if hb_idle is not None:
                self._log(
                    f"ESC recovered: mode={_mode_name(hb_idle['mode'])}, "
                    f"error={_error_str(hb_idle['error'])}")
            else:
                self._log(
                    "No heartbeat after recovery (ESC may not broadcast in IDLE — continuing)")
        else:
            # Production firmware (v3.0.6+) auto-transitions to IDLE after init, so the
            # sniff may already show mode=IDLE.  Sending NMT IDLE to an already-IDLE ESC
            # triggers MotorController_reset() in the firmware, which can temporarily
            # disable SDO processing for > 300 ms and cause the diagnostic SDO read to
            # time out.  Only send NMT IDLE if the sniff confirmed DISABLED mode.
            _MODE_IDLE = 0x01
            if _hb_target_mode != _MODE_IDLE:
                self._log(
                    f"Sending NMT IDLE to ID {active_id} — "
                    f"waking ESC from MODE_DISABLED so SDO reads/writes are processed...",
                    progress=44)
                await loop.run_in_executor(
                    None, dc.send_nmt, config.can_channel, active_id, 0x01)  # 0x01 = MODE_IDLE
                await asyncio.sleep(0.3)
            else:
                self._log(
                    f"ESC at ID {active_id} already in IDLE (confirmed by sniff) — "
                    f"skipping NMT IDLE to avoid firmware state reset.",
                    progress=44)

        self._log(
            f"ESC at ID {active_id}. "
            f"Writing params via SDO: target_id={config.can_id}, "
            f"phase_order={'inverted (-1)' if invert_phase else 'normal (+1)'}, "
            f"i_kp={profile['i_kp']:.4f}, i_ki={profile['i_ki']:.3f}",
            progress=45)

        # Write all critical motor parameters via SDO.
        # These match what the config page contains, but SDO writes ensure correctness
        # even when skip_flash=True (no fresh config page) or if the page was corrupt.
        # DEVICE_ID must go last (changes the software device_id routing filter).
        phase_val = -1 if invert_phase else +1
        i_limit_val = float(profile["max_calibration_current"]) * 2.0

        # Look up the configured gear_ratio for this joint so it is persisted to
        # ESC flash correctly.  Fall back to 1.0 (direct drive) only if the joint
        # is not yet in the robot config (e.g. first-time commissioning of a new ESC).
        gear_ratio = 1.0
        robot_cfg = self._daemon_client.config if self._daemon_client else None
        if robot_cfg is not None:
            for jname, jcfg in robot_cfg.joints.items():
                if jcfg.can_channel == config.can_channel and jcfg.can_id == config.can_id:
                    gear_ratio = float(jcfg.gear_ratio)
                    self._commissioning_joint_name = jname
                    self._log(f"  Using gear_ratio={gear_ratio:+.1f} from robot config")
                    break
            else:
                self._log(
                    f"  No joint config for {config.can_channel} ID {config.can_id} "
                    f"— using gear_ratio=1.0 (direct drive)")

        # Suspend slow-poll SDO reads for ALL motors.
        # The daemon sends periodic SDO READ requests (~3 Hz per motor) for telemetry.
        # Responses from any motor on the same bus are FUNC_TRANSMIT_SDO frames that
        # can interfere with GENERIC_SDO_WRITE futures and —critically— with the
        # diagnostic SDO READ below.  Suspending all motors eliminates this race.
        if dc:
            await loop.run_in_executor(None, dc.disable_all_slow_poll)
            self._log("  Slow-poll SDO suspended for all joints")
            if self._commissioning_joint_name:
                self._log(f"  (commissioning joint: {self._commissioning_joint_name})")
            # Wait two control-loop ticks (10 ms) for any in-flight slow-poll SDO
            # responses already in the kernel receive buffer to be drained.
            await asyncio.sleep(0.05)

        # Host-side CAN interface health check.  ERROR-WARNING / ERROR-PASSIVE / BUS-OFF
        # all cause the kernel TX queue to fill (ENOBUFS), silently dropping every outgoing
        # CAN frame so SDO reads return no response even though the ESC firmware is healthy.
        # Root cause: repeated BUS-OFF cycles (from prior SDO failures or bus errors) leave
        # old frames stuck in the kernel queue; a down/up cycle flushes the queue.
        can_iface_state = await _check_can_interface_state(config.can_channel)
        if can_iface_state:
            # Attempt automatic recovery: bounce the interface to flush the TX queue.
            _reset_ok = False
            try:
                import subprocess as _sp
                _sp.run(["ip", "link", "set", config.can_channel, "down"],
                        check=True, timeout=3, capture_output=True)
                _sp.run(["ip", "link", "set", config.can_channel, "type", "can",
                         "bitrate", "1000000"],
                        check=False, timeout=3, capture_output=True)
                _sp.run(["ip", "link", "set", config.can_channel,
                         "txqueuelen", "1000"],
                        check=True, timeout=3, capture_output=True)
                _sp.run(["ip", "link", "set", config.can_channel, "up"],
                        check=True, timeout=3, capture_output=True)
                _reset_ok = True
            except Exception:
                pass
            if _reset_ok:
                self._log(
                    f"  CAN interface {config.can_channel} was in {can_iface_state} "
                    f"(TX queue clogged from prior errors) — automatically reset to "
                    f"ERROR-ACTIVE. Waiting 300 ms for motors to reconnect...")
                await asyncio.sleep(0.3)
            else:
                self._log(
                    f"  WARNING: CAN interface {config.can_channel} is in {can_iface_state} "
                    f"state and could not be auto-reset (run as root?). "
                    f"TX frames will be silently dropped — SDO reads will return no response. "
                    f"Run manually: sudo ip link set {config.can_channel} down && "
                    f"sudo ip link set {config.can_channel} type can bitrate 1000000 && "
                    f"sudo ip link set {config.can_channel} txqueuelen 1000 && "
                    f"sudo ip link set {config.can_channel} up")

        # Pre-commission SDO diagnostic: confirm the ESC responds to SDO reads before
        # starting the 27-param write sequence.  Try device_id (0x000) first; fall back to
        # watchdog_timeout (0x008) in case the firmware marks 0x000 as non-readable.
        # Timeout is 3 s (3 attempts × 1 s each).  If this times out the ESC is wedged:
        #   • Old commissioning ELF with FDCAN filter fixed at device_id=127
        #   • Firmware in a wedged state (power-cycle + Flash+Commission required)
        diag_result = None
        _diag_sniff_count = -1
        _diag_param_used = _PARAM_DEVICE_ID
        for _diag_param_try in (_PARAM_DEVICE_ID, _PARAM_WATCHDOG_TIMEOUT):
            _r, _sniff = await loop.run_in_executor(
                None, dc.generic_sdo_read_diag,
                config.can_channel, active_id, _diag_param_try, 3.0)
            if _r is not None:
                diag_result = _r
                _diag_param_used = _diag_param_try
                break
            _diag_sniff_count = _sniff  # capture sniff count from last attempt
        if diag_result is None:
            if _diag_sniff_count == 0:
                _diag_hint = (
                    "The daemon received no SDO response frames from the ESC at all — "
                    "the ESC firmware is not responding to SDO requests. "
                    "This usually means the FDCAN receive filter is misconfigured "
                    "or the firmware is in a wedged state. ")
            elif _diag_sniff_count > 0:
                _diag_hint = (
                    f"The daemon received {_diag_sniff_count} SDO frame(s) from the ESC "
                    f"but the response matcher did not fire — please file a bug report. ")
            else:
                _diag_hint = "The firmware is likely in a wedged state. "
            raise FlashError(
                f"ESC at ID {active_id} is NOT responding to SDO reads — "
                f"commissioning aborted to avoid calibration with wrong parameters. "
                f"Tried params 0x{_PARAM_DEVICE_ID:03X} and 0x{_PARAM_WATCHDOG_TIMEOUT:03X}, "
                f"3 s each, with all slow-poll suspended. "
                f"{_diag_hint}"
                f"Try: (1) power-cycle the ESC and retry, "
                f"(2) disconnect the dashboard (click Disconnect) before commissioning, "
                f"(3) use 'Flash + Commission' to re-flash from scratch.")
        if _diag_param_used == _PARAM_DEVICE_ID:
            stored_id = int(diag_result["value_u32"] or 0)
            self._log(f"  SDO diagnostic: ESC device_id={stored_id} — SDO responding ✓")
        else:
            wdt_ms = int(diag_result["value_u32"] or 0)
            self._log(
                f"  SDO diagnostic: watchdog_timeout={wdt_ms} ms — SDO responding ✓ "
                f"(device_id param 0x000 timed out, used fallback 0x{_PARAM_WATCHDOG_TIMEOUT:03X})")

        sdo_writes = [
            (_PARAM_POSITION_GEAR_RATIO,   "f32", gear_ratio,                   f"gear_ratio={gear_ratio:+.1f}"),
            (_PARAM_CURRENT_I_LIMIT,       "f32", i_limit_val,                  f"i_limit={i_limit_val:.2f}"),
            (_PARAM_CURRENT_I_KP,          "f32", profile["i_kp"],              f"i_kp={profile['i_kp']:.4f}"),
            (_PARAM_CURRENT_I_KI,          "f32", profile["i_ki"],              f"i_ki={profile['i_ki']:.3f}"),
            (_PARAM_MOTOR_POLE_PAIRS,      "u32", int(profile.get("pole_pairs", 14)), f"pole_pairs={profile.get('pole_pairs', 14)}"),
            (_PARAM_MOTOR_TORQUE_CONSTANT, "f32", profile["torque_constant"],   f"torque_constant={profile['torque_constant']:.5f}"),
            (_PARAM_MOTOR_PHASE_ORDER,     "i32", phase_val,                    f"phase_order={phase_val:+d}"),
            (_PARAM_MOTOR_MAX_CALIBRATION, "f32", float(profile["max_calibration_current"]), f"max_calib_current={profile['max_calibration_current']:.1f}"),
            (_PARAM_ENCODER_CPR,           "u32", int(profile.get("cpr", 4096)), f"cpr={profile.get('cpr', 4096)}"),
        ]
        for param, dtype, value, label in sdo_writes:
            ack = await loop.run_in_executor(
                None, dc.generic_sdo_write,
                config.can_channel, active_id, param, dtype, value)
            self._log(f"  SDO[ID {active_id}] {label} → {'ACK' if ack else 'NO_ACK'}")

        if active_id != config.can_id:
            ack_id = await loop.run_in_executor(
                None, dc.generic_sdo_write,
                config.can_channel, active_id, _PARAM_DEVICE_ID, "u32", config.can_id)
            self._log(
                f"  SDO[ID {active_id}] device_id → {config.can_id} → "
                f"{'ACK' if ack_id else 'NO_ACK'}")
            await asyncio.sleep(0.1)

        self._log(
            f"Sending flash store to ID {active_id} "
            f"(target={config.can_id})...")
        await loop.run_in_executor(
            None, dc.send_flash_store, config.can_channel, active_id)
        await asyncio.sleep(0.2)

        self._log(
            f"Verification: waiting for heartbeat from ID {config.can_id} (3 s)...",
            progress=50)
        hb_new = await loop.run_in_executor(
            None, dc.wait_heartbeat, config.can_channel, config.can_id, 3000)
        if hb_new is None:
            self._log(f"No heartbeat at target ID {config.can_id} within 3 s.")
            if active_id != config.can_id:
                self._log(
                    f"Checking if ESC is still at source ID {active_id} (1.5 s)...")
                hb_old = await loop.run_in_executor(
                    None, dc.wait_heartbeat, config.can_channel, active_id, 1500)
                if hb_old is not None:
                    self._log(
                        f"ESC still at ID {active_id}: "
                        f"mode={_mode_name(hb_old['mode'])}, error={_error_str(hb_old['error'])}")
                    raise FlashError(
                        f"ESC still at source ID {active_id} after DEVICE_ID write — "
                        f"SDO write was not ACK'd. The commissioning ELF may require "
                        f"DEVICE_ID to be encoded in the config page before flashing.")
            raise FlashError(
                f"No response from ESC at CAN ID {config.can_id} — "
                f"check CAN cable and ESC power.")

        self._log(
            f"ESC confirmed at CAN ID {config.can_id}. "
            f"Config saved to flash. ESC is live.", progress=54)

        # Verify ESC is in a healthy state before calibration.
        hb_mode = hb_new['mode']
        hb_error = hb_new['error']
        self._log(
            f"ESC state: mode={_mode_name(hb_mode)}, error={_error_str(hb_error)}",
            progress=55)
        if hb_error != 0 or hb_mode not in (0x00, 0x01):
            self._log(
                f"ESC in unexpected state before calibration "
                f"({_mode_name(hb_mode)}, {_error_str(hb_error)}). "
                "Disabling watchdog + clearing errors via SDO then sending NMT IDLE...")
            ack_wd2 = await loop.run_in_executor(
                None, dc.generic_sdo_write,
                config.can_channel, config.can_id, _PARAM_WATCHDOG_TIMEOUT, "u32", 0)
            self._log(
                f"  SDO[ID {config.can_id}] watchdog_timeout=0 → "
                f"{'ACK' if ack_wd2 else 'NO_ACK'}")
            ack_err2 = await loop.run_in_executor(
                None, dc.generic_sdo_write,
                config.can_channel, config.can_id, _PARAM_ERROR, "u32", 0)
            self._log(
                f"  SDO[ID {config.can_id}] error=0 (clear) → "
                f"{'ACK' if ack_err2 else 'NO_ACK'}")
            await asyncio.sleep(0.1)
            await loop.run_in_executor(None, dc.send_nmt, config.can_channel, config.can_id, 0x01)
            await asyncio.sleep(1.5)
            hb_cleared = await loop.run_in_executor(
                None, dc.wait_heartbeat, config.can_channel, config.can_id, 3000)
            if hb_cleared is not None:
                self._log(
                    f"After recovery: mode={_mode_name(hb_cleared['mode'])}, "
                    f"error={_error_str(hb_cleared['error'])}")
                if hb_cleared['error'] != 0:
                    self._log(
                        f"ESC still has errors ({_error_str(hb_cleared['error'])}). "
                        "Calibration may fail — check encoder and motor wiring.")
            else:
                self._log("No heartbeat after recovery (continuing).")

        # ── 4. Calibration ───────────────────────────────────────────────────
        self._log("Waiting for CAN bus to settle...", progress=56)
        await asyncio.sleep(2.0)

        while True:
            self.status.state = FlashState.CALIBRATING
            ack_freq = await loop.run_in_executor(
                None, dc.generic_sdo_write,
                config.can_channel, config.can_id, _PARAM_FAST_FRAME_FREQUENCY, "u32", 100)
            self._log(
                f"SDO[ID {config.can_id}] fast_frame_frequency=100 Hz → "
                f"{'ACK' if ack_freq else 'NO_ACK'}",
                progress=60)

            # Pre-calibration: verify ESC mode and error via SDO before triggering calibration.
            pre_err_result  = await loop.run_in_executor(
                None, dc.generic_sdo_read, config.can_channel, config.can_id, _PARAM_ERROR)
            pre_mode_result = await loop.run_in_executor(
                None, dc.generic_sdo_read, config.can_channel, config.can_id, _PARAM_MODE)
            pre_error = int(pre_err_result["value_u32"] or 0) if pre_err_result else None
            pre_mode  = int(pre_mode_result["value_u32"] or 0) if pre_mode_result else None
            self._log(
                f"Pre-calibration: mode={_mode_name(pre_mode) if pre_mode is not None else '?'}, "
                f"error={_error_str(pre_error) if pre_error is not None else '?'}",
                progress=61)
            if pre_error:
                self._log(
                    f"ESC has errors ({_error_str(pre_error)}) — "
                    "clearing via SDO then NMT IDLE before calibration...")
                await loop.run_in_executor(
                    None, dc.generic_sdo_write,
                    config.can_channel, config.can_id, _PARAM_WATCHDOG_TIMEOUT, "u32", 0)
                await loop.run_in_executor(
                    None, dc.generic_sdo_write,
                    config.can_channel, config.can_id, _PARAM_ERROR, "u32", 0)
                await asyncio.sleep(0.1)
                await loop.run_in_executor(None, dc.send_nmt, config.can_channel, config.can_id, 0x01)
                await asyncio.sleep(1.0)

            self._log(
                "Starting encoder flux offset calibration (MODE_CALIBRATION). "
                "This takes ~15 s. Do NOT power off the ESC.",
                progress=62)

            cal_result = await loop.run_in_executor(
                None, dc.calibrate_device,
                config.can_channel, config.can_id, 90000)

            if cal_result["status"] == "ERROR":
                raise FlashError(
                    f"Calibration failed: error_code=0x{cal_result['error_code']:04X}")
            if cal_result["status"] == "TIMEOUT":
                self._log("Calibration timed out after 90 s. Reading SWD state for diagnosis...",
                          progress=65)
                swd_diag = await _read_controller_state_via_swd(firmware_dir)
                swd_detail = ""
                if swd_diag is not None:
                    can_state = _format_fdcan_state(swd_diag)
                    swd_detail = (
                        f" ESC state: mode={_mode_name(swd_diag['mode'])}, "
                        f"error={_error_str(swd_diag['error'])}"
                        + (f", CAN={can_state}" if can_state else "")
                    )
                    self._log(f"SWD diagnostic:{swd_detail}")
                raise FlashError(
                    "Calibration timed out after 90 s." + swd_detail +
                    " Check motor phase wires and encoder connection.")

            # After calibration the ESC transitions CALIBRATION → IDLE.
            # Wait 800 ms before SDO reads: the firmware writes the flux_offset
            # into its internal struct and the ESC may not respond to SDO reads
            # for a brief window immediately after returning to IDLE.
            await asyncio.sleep(0.8)

            # Read flux_offset from the device — retry once with a long timeout.
            fo_result = await loop.run_in_executor(
                None, dc.generic_sdo_read,
                config.can_channel, config.can_id, _PARAM_ENCODER_FLUX_OFFSET, 3.0)
            if fo_result is None:
                self._log(
                    "flux_offset SDO read timed out on first attempt — waiting 1 s and retrying...")
                await asyncio.sleep(1.0)
                fo_result = await loop.run_in_executor(
                    None, dc.generic_sdo_read,
                    config.can_channel, config.can_id, _PARAM_ENCODER_FLUX_OFFSET, 5.0)
            if fo_result is None:
                raise FlashError(
                    f"Calibration completed but flux_offset SDO read timed out "
                    f"(CAN ID {config.can_id} on {config.can_channel}). "
                    "Check CAN wiring and that the ESC is still powered."
                )
            flux_offset = fo_result["value_f32"]
            self.status.flux_offset = flux_offset
            self._log(
                f"Calibration done: flux_offset = {flux_offset:.4f} rad "
                f"({math.degrees(flux_offset):.2f}°). Saved to Flash.",
                progress=75)

            # ── 5. Confirm direction ──────────────────────────────────────────
            self.status.state = FlashState.AWAITING_CONFIRMATION
            self._confirm_event = asyncio.Event()
            self._confirmed_correct = None
            self._log(
                "Did the motor rotate correctly during calibration? "
                "Confirm direction. (120 s timeout)",
                progress=78)
            await asyncio.wait_for(self._confirm_event.wait(), timeout=120.0)
            direction_correct = bool(self._confirmed_correct)

            if direction_correct:
                break

            # Toggle phase, save, and re-calibrate.
            invert_phase = not invert_phase
            new_phase_val = -1 if invert_phase else +1
            self._log(
                f"Direction wrong — toggling phase_order to {new_phase_val:+d}, "
                f"saving, and re-calibrating...",
                progress=80)
            self.status.state = FlashState.CALIBRATING
            ack_tog = await loop.run_in_executor(
                None, dc.generic_sdo_write,
                config.can_channel, config.can_id, _PARAM_MOTOR_PHASE_ORDER,
                "i32", new_phase_val)
            self._log(
                f"SDO[ID {config.can_id}] phase_order={new_phase_val:+d} → "
                f"{'ACK' if ack_tog else 'NO_ACK'}")
            await loop.run_in_executor(
                None, dc.send_flash_store, config.can_channel, config.can_id)
            self._log("Phase order saved. Re-running calibration...", progress=82)

        self.status.updated_config = {
            "torque_constant":         profile["torque_constant"],
            "max_calibration_current": float(profile["max_calibration_current"]),
            "current_kp":              profile["i_kp"],
            "current_ki":              profile["i_ki"],
            "electrical_offset":       self.status.flux_offset,
            "phase_inverted":          invert_phase,
            "fast_frame_frequency":    100,
            "gear_ratio":              gear_ratio,
        }

        # Put the commissioned ESC into DISABLED (silent resting state).
        # The motor is fully configured and stored to flash; it should not broadcast
        # until the robot is next connected via the dashboard.
        await loop.run_in_executor(None, dc.send_nmt, config.can_channel, active_id, 0x00)

        self.status.state = FlashState.COMPLETE
        self._log(
            "Flash wizard complete. Motor is commissioned, stored to flash, "
            "and placed in DISABLED mode.",
            progress=100)
