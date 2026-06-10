"""
DaemonClient — thin UDP bridge from Python to the C++ control daemon.

Two ports:
  9001  Python → daemon  JSON request / response
  9000  daemon → Python  JSON telemetry push (fire-and-forget)

This is the ONLY Python file that knows the daemon UDP protocol.
All other backend files import this module and call DaemonClient methods.
"""
from __future__ import annotations

import asyncio
import json
import logging
import socket
import threading
import time
import uuid
from pathlib import Path
from typing import Callable

from enum import IntEnum

from humanoid.actuator import ActuatorState
from humanoid.robot_config import JointConfig, RobotConfig


class Mode(IntEnum):
    DISABLED            = 0x00
    IDLE                = 0x01
    DAMPING             = 0x02
    CALIBRATION         = 0x05
    CURRENT             = 0x10
    TORQUE              = 0x11
    VELOCITY            = 0x12
    POSITION            = 0x13
    VABC_OVERRIDE       = 0x20
    VALPHABETA_OVERRIDE = 0x21
    VQD_OVERRIDE        = 0x22
    DEBUG               = 0x80

_log = logging.getLogger(__name__)

_CANONICAL_BUSES = ['can_left_leg', 'can_right_leg', 'can_left_arm', 'can_right_arm']
_SYSFS_NET = Path('/sys/class/net')


def _sysfs_iface_state(ifname: str) -> str:
    """Read kernel operstate from sysfs without spawning a subprocess.
    Returns 'UP', 'DOWN', or 'UNCONFIGURED' (interface not present in OS at all).
    """
    operstate_path = _SYSFS_NET / ifname / 'operstate'
    try:
        operstate = operstate_path.read_text().strip()
        return 'UP' if operstate == 'up' else 'DOWN'
    except OSError:
        return 'UNCONFIGURED'

_DAEMON_HOST = "127.0.0.1"
_CMD_TIMEOUT = 5.0   # seconds
_RUNNING_MAX_AGE = 2.0  # seconds before is_running() returns False


# ---------------------------------------------------------------------------
# Exceptions
# ---------------------------------------------------------------------------

class DaemonError(Exception):
    """Base class for daemon communication errors."""


class DaemonNotRunningError(DaemonError):
    """Daemon is not reachable (no telemetry in the last 2 s, or socket error)."""


class DaemonCommandError(DaemonError):
    """Daemon returned an ERROR response to a command."""


class DaemonNotSupportedError(DaemonError):
    """Operation requires direct CAN access; not available while daemon owns the bus."""


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _mode_name(mode_int: int) -> str:
    try:
        return Mode(mode_int).name
    except ValueError:
        return f"UNKNOWN(0x{mode_int:02X})"


def _daemon_state_to_actuator(d: dict) -> ActuatorState | None:
    """Convert a daemon state dict into an ActuatorState pydantic object.

    Returns None for OFFLINE joints so the frontend null-checks correctly
    distinguish unreachable devices from reachable-but-idle ones.

    Telemetry uses "state" key; GET_STATE response uses "joint_state" key.
    """
    joint_state = d.get("state") or d.get("joint_state")
    if joint_state == "OFFLINE":
        return None
    mode_int = int(d.get("mode", 0))
    bv = d.get("bus_voltage")
    return ActuatorState(
        position=float(d.get("position", 0.0)),
        velocity=float(d.get("velocity", 0.0)),
        torque=float(d.get("torque", 0.0)),
        current=float(d.get("current", 0.0)),
        mode=mode_int,
        mode_name=_mode_name(mode_int),
        error=int(d.get("error", 0)),
        bus_voltage=float(bv) if bv is not None else None,
        timestamp=time.time(),
    )


# ---------------------------------------------------------------------------
# DaemonActuatorProxy
# ---------------------------------------------------------------------------

class DaemonActuatorProxy:
    """
    Drop-in async replacement for Actuator, backed by DaemonClient.

    routes_motors.py calls ``await actuator.get_state()``, ``await actuator.enable()``,
    etc.  This proxy translates those calls into daemon UDP commands.

    Operations not supported by the daemon (load_from_flash) raise
    DaemonNotSupportedError so callers can return a meaningful HTTP 503.
    """

    def __init__(
        self,
        name: str,
        can_id: int,
        can_channel: str,
        config: JointConfig,
        client: DaemonClient,
    ) -> None:
        self._name = name
        self._can_id = can_id
        self._can_channel = can_channel
        self._config = config
        self._client = client

    # -- Properties mirroring Actuator --

    @property
    def device_id(self) -> int:
        return self._can_id

    @property
    def name(self) -> str:
        return self._name

    @property
    def config(self) -> JointConfig:
        return self._config

    def update_config(self, config: JointConfig) -> None:
        self._config = config

    # -- Async actuator interface (supported by daemon) --

    async def get_state(self, passive: tuple | None = None) -> ActuatorState:
        """Query daemon for fresh joint state. `passive` arg is ignored."""
        loop = asyncio.get_running_loop()
        data = await loop.run_in_executor(None, self._client.get_state, self._name)
        return _daemon_state_to_actuator(data)

    async def enable(self, mode: Mode = Mode.POSITION) -> None:
        """Set joint to active control mode (daemon maps all active modes to ENABLED)."""
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, self._client.set_mode, self._name, "POSITION")

    async def disable(self) -> None:
        """Set joint to IDLE (daemon NMT MODE_IDLE)."""
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, self._client.set_mode, self._name, "IDLE")

    async def estop(self) -> None:
        """Emergency stop — daemon maps DISABLED to IDLE (motor stops, firmware stays alive)."""
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, self._client.set_mode, self._name, "DISABLED")

    async def clear_error(self) -> None:
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, self._client.clear_error, self._name)

    async def set_position(
        self,
        position: float,
        velocity_ff: float = 0.0,
        torque_ff: float = 0.0,
        timeout: float = 0.005,
    ) -> tuple[float, float] | None:
        """
        Send a position target to the daemon.
        Returns None — daemon SET_POSITION ACKs without measured feedback.
        """
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, self._client.set_position, self._name, position)
        return None

    async def apply_config(self) -> None:
        """Tell daemon to write this joint's live config to device RAM."""
        loop = asyncio.get_running_loop()
        cfg = self._config
        config_dict = {
            "gear_ratio":               float(cfg.gear_ratio),
            "position_kp":              float(cfg.position_kp),
            "position_ki":              float(cfg.position_ki),
            "velocity_kp":              float(cfg.velocity_kp),
            "velocity_ki":              float(cfg.velocity_ki),
            "torque_limit":             float(cfg.torque_limit),
            "velocity_limit":           float(cfg.velocity_limit),
            "position_limit_min":       float(cfg.position_limits.lower_bound),
            "position_limit_max":       float(cfg.position_limits.upper_bound),
            "position_offset":          float(cfg.position_offset),
            "torque_filter_alpha":      float(cfg.torque_filter_alpha),
            "current_limit":            float(cfg.current_limit),
            "current_kp":               float(cfg.current_kp),
            "current_ki":               float(cfg.current_ki),
            "undervoltage_threshold":   float(cfg.undervoltage_threshold),
            "overvoltage_threshold":    float(cfg.overvoltage_threshold),
            "bus_voltage_filter_alpha": float(cfg.bus_voltage_filter_alpha),
            "torque_constant":          float(cfg.torque_constant),
            "max_calibration_current":  float(cfg.max_calibration_current),
            "encoder_position_offset":  float(cfg.encoder_position_offset),
            "velocity_filter_alpha":    float(cfg.velocity_filter_alpha),
            "electrical_offset":        float(cfg.electrical_offset),
            "fast_frame_frequency":     int(cfg.fast_frame_frequency),
            "watchdog_timeout":         int(cfg.watchdog_timeout),
            "pole_pairs":               int(cfg.pole_pairs),
            "cpr":                      int(cfg.cpr),
            "phase_inverted":           bool(cfg.phase_inverted),
        }
        await loop.run_in_executor(None, self._client.apply_config, self._name, config_dict)

    async def feed_watchdog(self) -> None:
        """No-op — daemon feeds watchdogs from its 200 Hz control loop."""

    def get_cached_state(self) -> ActuatorState | None:
        """Return the latest telemetry-pushed state without a UDP round-trip.

        Returns None when the joint is OFFLINE or no telemetry has been received yet.
        Safe to call from any thread (reads a lock-protected cache).
        """
        d = self._client.get_cached_joint_state(self._name)
        return _daemon_state_to_actuator(d) if d is not None else None

    async def sdo_write_f32(self, param_id: int, value: float, timeout: float = 1.0) -> bool:
        """Write a float32 parameter directly to the device via SDO (async wrapper)."""
        loop = asyncio.get_running_loop()
        return await loop.run_in_executor(
            None,
            lambda: self._client.generic_sdo_write(
                self._can_channel, self._can_id, param_id, 'f32', float(value), timeout
            ),
        )

    # -- Unsupported operations (require direct CAN bus access) --

    async def calibrate_offset(
        self,
        timeout: float = 90.0,
        on_progress: Callable[[str], None] | None = None,
    ) -> float:
        """Run flux-offset calibration via the daemon's CALIBRATE_DEVICE command."""
        loop = asyncio.get_running_loop()
        timeout_ms = int(timeout * 1000)

        result = await loop.run_in_executor(
            None, self._client.calibrate_device,
            self._can_channel, self._can_id, timeout_ms,
        )
        if result["status"] == "TIMEOUT":
            raise DaemonError(
                f"{self._name}: calibration timed out after {timeout:.0f} s"
            )
        if result["status"] == "ERROR":
            raise DaemonError(
                f"{self._name}: calibration failed "
                f"(firmware error=0x{result['error_code']:04X})"
            )

        # Read back the measured flux offset from device RAM.
        _PARAM_ENCODER_FLUX_OFFSET = 0x13C
        sdo = await loop.run_in_executor(
            None, self._client.generic_sdo_read,
            self._can_channel, self._can_id, _PARAM_ENCODER_FLUX_OFFSET,
        )
        if sdo is None:
            raise DaemonError(
                f"{self._name}: calibration succeeded but flux offset read timed out"
            )

        flux_offset: float = sdo["value_f32"]
        self._config = self._config.with_electrical_offset(flux_offset)
        return flux_offset

    async def store_to_flash(self) -> None:
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, self._client.store_joint_to_flash, self._name)

    async def load_from_flash(self) -> None:
        raise DaemonNotSupportedError(
            "load_from_flash requires direct CAN access; "
            "the daemon owns the bus — stop the daemon first"
        )

    async def read_config_from_device(self) -> dict:
        loop = asyncio.get_running_loop()
        return await loop.run_in_executor(None, self._client.read_device_config, self._name)

    def __repr__(self) -> str:
        return f"DaemonActuatorProxy(name={self._name!r}, can_id={self._can_id})"


# ---------------------------------------------------------------------------
# DaemonClient
# ---------------------------------------------------------------------------

class DaemonClient:
    """
    Manages the Python ↔ daemon UDP protocol.  Replaces both Robot and CanMonitor
    in the FastAPI backend.

    Thread model:
      - Telemetry thread (daemon-telemetry): receives JSON pushes on port 9000,
        updates the cached joint-state + bus-health snapshot.
      - Any asyncio thread: sends commands on port 9001 via _send_command()
        (blocking, protected by a threading.Lock, run in executor from async callers).
      - Main asyncio thread: reads cached snapshot without blocking.
    """

    def __init__(
        self,
        config: RobotConfig | None,
        cmd_port: int = 9001,
        tel_port: int = 9000,
        daemon_host: str = _DAEMON_HOST,
        cmd_timeout: float = _CMD_TIMEOUT,
    ) -> None:
        self.config = config   # exposed; routes read/write app.state.robot.config

        self._cmd_port    = cmd_port
        self._tel_port    = tel_port
        self._daemon_host = daemon_host
        self._cmd_timeout = cmd_timeout

        # Command socket (synchronous, one at a time)
        self._cmd_sock: socket.socket | None = None
        self._cmd_lock = threading.Lock()

        # Telemetry receive thread
        self._tel_sock: socket.socket | None = None
        self._tel_thread: threading.Thread | None = None
        self._stop_event = threading.Event()

        # Telemetry snapshot (written by tel_thread, read from any thread)
        self._tel_lock      = threading.Lock()
        self._joint_states: dict[str, dict] = {}   # name → daemon joint dict
        self._bus_health:   dict[str, dict] = {}   # ifname → {open, tx_dropped, rx_frames}
        self._last_tel_time: float = 0.0           # monotonic time of last good frame

        # Drop-event queue (daemon doesn't push these; kept for API compat)
        self._drop_lock   = threading.Lock()
        self._drop_events: list[dict] = []

        # Per-joint proxies built from the loaded config
        self._proxies: dict[str, DaemonActuatorProxy] = {}
        self._rebuild_proxies()

    # ------------------------------------------------------------------ #
    # Lifecycle                                                            #
    # ------------------------------------------------------------------ #

    async def start(self) -> None:
        """Open sockets and start the telemetry receive thread (async entry point)."""
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, self._start_sync)

    async def stop(self) -> None:
        """Stop telemetry thread and close sockets (async entry point)."""
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, self._stop_sync)

    def _start_sync(self) -> None:
        self._stop_event.clear()

        # Command socket — ephemeral port; daemon responds to sender address
        self._cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._cmd_sock.settimeout(self._cmd_timeout)

        # Telemetry listen socket on port 9000
        self._tel_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._tel_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._tel_sock.bind(("", self._tel_port))
        self._tel_sock.settimeout(1.0)   # allows clean shutdown checks

        self._tel_thread = threading.Thread(
            target=self._tel_receive_loop,
            daemon=True,
            name="daemon-telemetry",
        )
        self._tel_thread.start()
        _log.info(
            "DaemonClient started (cmd→%s:%d  tel←:%d)",
            self._daemon_host, self._cmd_port, self._tel_port,
        )

    def _stop_sync(self) -> None:
        self._stop_event.set()
        if self._tel_thread and self._tel_thread.is_alive():
            self._tel_thread.join(timeout=3.0)
        for sock in (self._cmd_sock, self._tel_sock):
            if sock is not None:
                try:
                    sock.close()
                except Exception:
                    pass
        self._cmd_sock = None
        self._tel_sock = None
        _log.info("DaemonClient stopped")

    # ------------------------------------------------------------------ #
    # State queries                                                        #
    # ------------------------------------------------------------------ #

    def is_running(self) -> bool:
        """True if telemetry was received within the last 2 seconds."""
        return (time.monotonic() - self._last_tel_time) < _RUNNING_MAX_AGE

    def is_connected(self) -> bool:
        """Alias for is_running() — satisfies the Robot interface used by routes."""
        return self.is_running()

    # ------------------------------------------------------------------ #
    # Telemetry receive thread                                             #
    # ------------------------------------------------------------------ #

    def _tel_receive_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                data, _ = self._tel_sock.recvfrom(65535)
            except socket.timeout:
                continue
            except Exception as exc:
                if not self._stop_event.is_set():
                    _log.debug("Telemetry recv error: %s", exc)
                continue

            try:
                msg = json.loads(data)
            except json.JSONDecodeError:
                continue

            if msg.get("type") != "TELEMETRY":
                continue

            with self._tel_lock:
                self._joint_states  = msg.get("joints", {})
                self._bus_health    = msg.get("bus_health", {})
                self._last_tel_time = time.monotonic()

    # ------------------------------------------------------------------ #
    # Command send / receive                                               #
    # ------------------------------------------------------------------ #

    def _send_command(self, msg: dict, timeout: float | None = None) -> dict:
        """
        Send a JSON command to the daemon and wait for a matching response.
        Thread-safe via _cmd_lock.  Raises on timeout or daemon error.
        Pass timeout to override the default _cmd_timeout for slow commands.
        """
        if self._cmd_sock is None:
            raise DaemonNotRunningError("DaemonClient not started — call start() first")

        msg.setdefault("id", str(uuid.uuid4()))
        payload = json.dumps(msg).encode()

        with self._cmd_lock:
            effective_timeout = timeout if timeout is not None else self._cmd_timeout
            try:
                self._cmd_sock.settimeout(effective_timeout)
                self._cmd_sock.sendto(payload, (self._daemon_host, self._cmd_port))
                raw, _ = self._cmd_sock.recvfrom(65535)
            except socket.timeout:
                raise DaemonNotRunningError(
                    f"Daemon did not respond within {effective_timeout}s "
                    f"(type={msg.get('type')})"
                )
            except OSError as exc:
                raise DaemonNotRunningError(f"Daemon socket error: {exc}") from exc
            finally:
                self._cmd_sock.settimeout(self._cmd_timeout)

        try:
            resp = json.loads(raw)
        except json.JSONDecodeError as exc:
            raise DaemonCommandError(f"Invalid JSON response: {raw!r}") from exc

        if resp.get("type") == "ERROR":
            raise DaemonCommandError(resp.get("msg", "unknown daemon error"))

        return resp

    # ------------------------------------------------------------------ #
    # Daemon commands (synchronous — run via executor from async callers) #
    # ------------------------------------------------------------------ #

    def ping(self) -> dict:
        return self._send_command({"type": "PING"})

    def get_state(self, joint_name: str) -> dict:
        """Fetch fresh state for one joint from daemon. Returns the inner state dict."""
        resp = self._send_command({"type": "GET_STATE", "joint_name": joint_name})
        return resp.get("state", {})

    def get_all_states_raw(self) -> dict[str, dict]:
        """Fetch states for all joints from daemon (blocking)."""
        resp = self._send_command({"type": "GET_ALL_STATES"})
        return resp.get("states", {})

    def get_cached_joint_state(self, joint_name: str) -> dict | None:
        """Return the most recent telemetry-cached state for joint_name without a UDP round-trip."""
        with self._tel_lock:
            return self._joint_states.get(joint_name)

    def set_mode(self, joint_name: str, mode: str) -> None:
        self._send_command({"type": "SET_MODE", "joint_name": joint_name, "mode": mode})

    def set_all_mode(self, mode: str) -> None:
        self._send_command({"type": "SET_ALL_MODE", "mode": mode})

    def set_position(self, joint_name: str, position_rad: float) -> None:
        self._send_command({
            "type": "SET_POSITION",
            "joint_name": joint_name,
            "position_rad": position_rad,
        })

    def clear_error(self, joint_name: str) -> None:
        self._send_command({"type": "CLEAR_ERROR", "joint_name": joint_name})

    def apply_config(self, joint_name: str, config: dict | None = None) -> None:
        cmd: dict = {"type": "APPLY_CONFIG", "joint_name": joint_name}
        if config is not None:
            cmd["config"] = config
        self._send_command(cmd)

    def apply_all_configs(self) -> None:
        # Daemon applies config to every joint on every open CAN bus.
        # Worst case: N_joints × 27 SDO writes × 500 ms/write = several seconds.
        # Use a 60 s timeout so partial-bus setups (some buses unavailable) don't
        # trigger a false DaemonNotRunningError in the Python caller.
        resp = self._send_command({"type": "APPLY_ALL_CONFIGS"}, timeout=60.0)
        configured = resp.get("configured", 0)
        skipped    = resp.get("skipped", 0)
        failed     = resp.get("failed", 0)
        if configured == 0 and skipped > 0:
            _log.warning("apply_all_configs: no motors configured (%d skipped, %d failed)",
                         skipped, failed)
        else:
            _log.info("apply_all_configs: %d configured, %d skipped, %d failed",
                      configured, skipped, failed)

    def store_joint_to_flash(self, joint_name: str) -> None:
        self._send_command({"type": "STORE_TO_FLASH", "joint_name": joint_name})

    def read_device_config(self, joint_name: str) -> dict:
        # READ_CONFIG reads 23 params with up to 100ms each = ~2.3s; use a generous timeout.
        resp = self._send_command({"type": "READ_CONFIG", "joint_name": joint_name}, timeout=30.0)
        if resp.get("type") != "CONFIG":
            raise DaemonError(f"READ_CONFIG failed: {resp}")
        return resp.get("config", {})

    def daemon_shutdown(self) -> None:
        """Ask the daemon to shut itself down gracefully."""
        try:
            self._send_command({"type": "SHUTDOWN"})
        except DaemonError:
            pass  # daemon may already be gone

    def disable_slow_poll(self, joint_name: str) -> None:
        """Stop slow-poll SDO telemetry reads for a joint.
        Call before Flash Wizard commissioning to prevent SDO read responses
        from consuming generic_listener_ futures registered for SDO writes.
        """
        self._send_command({"type": "DISABLE_SLOW_POLL", "joint_name": joint_name})

    def enable_slow_poll(self, joint_name: str) -> None:
        """Resume slow-poll SDO telemetry reads for a joint."""
        self._send_command({"type": "ENABLE_SLOW_POLL", "joint_name": joint_name})

    # ------------------------------------------------------------------ #
    # Generic CAN passthrough — flash wizard and commissioning            #
    # ------------------------------------------------------------------ #

    def generic_sdo_write(
        self,
        channel: str,
        device_id: int,
        param_id: int,
        value_type: str,
        value: float | int,
        timeout: float = 1.0,
    ) -> bool:
        """
        Write a parameter to any device_id on the bus (not just configured joints).
        value_type: 'u32', 'i32', or 'f32'.
        Returns True if the ESC ACK'd, False if no-ACK (commissioning ELF may not respond).
        Raises DaemonNotRunningError if the daemon is unreachable.
        """
        resp = self._send_command({
            "type": "GENERIC_SDO_WRITE",
            "channel": channel,
            "device_id": device_id,
            "param_id": param_id,
            "value_type": value_type,
            "value": value,
            "timeout_ms": int(timeout * 1000),
        }, timeout=timeout + 1.0)
        return resp.get("status") == "OK"

    def generic_sdo_read(
        self,
        channel: str,
        device_id: int,
        param_id: int,
        timeout: float = 1.0,
    ) -> dict | None:
        """
        Read a parameter from any device_id.
        Returns dict with value_u32, value_f32, value_i32, or None on timeout.
        """
        resp = self._send_command({
            "type": "GENERIC_SDO_READ",
            "channel": channel,
            "device_id": device_id,
            "param_id": param_id,
            "timeout_ms": int(timeout * 1000),
        }, timeout=timeout + 1.0)
        if resp.get("status") == "OK":
            return {
                "value_u32": resp.get("value_u32"),
                "value_f32": resp.get("value_f32"),
                "value_i32": resp.get("value_i32"),
            }
        return None

    def wait_heartbeat(
        self,
        channel: str,
        device_id: int,
        timeout_ms: int = 3000,
    ) -> dict | None:
        """
        Block until a heartbeat is received from device_id, or timeout.
        Returns dict with 'mode' (int) and 'error' (int), or None on timeout.
        """
        resp = self._send_command({
            "type": "WAIT_HEARTBEAT",
            "channel": channel,
            "device_id": device_id,
            "timeout_ms": timeout_ms,
        }, timeout=timeout_ms / 1000.0 + 1.0)
        if resp.get("status") == "OK":
            return {"mode": resp.get("mode", 0), "error": resp.get("error", 0)}
        return None

    def sniff_bus(
        self,
        channel: str,
        duration_ms: int = 3000,
    ) -> list[dict]:
        """
        Collect all CAN frames on the channel for duration_ms milliseconds.
        Returns list of {device_id, func_id, dlc, data} dicts.
        """
        resp = self._send_command({
            "type": "SNIFF_BUS",
            "channel": channel,
            "duration_ms": duration_ms,
        }, timeout=duration_ms / 1000.0 + 2.0)
        return resp.get("frames", [])

    def send_nmt(self, channel: str, device_id: int, mode: int) -> None:
        """Send an NMT mode-change frame to any device_id."""
        self._send_command({
            "type": "SEND_NMT",
            "channel": channel,
            "device_id": device_id,
            "mode": mode,
        })

    def send_flash_store(self, channel: str, device_id: int) -> None:
        """Send FUNC_FLASH store command; blocks ~300 ms for flash write to complete."""
        self._send_command({
            "type": "SEND_FLASH_STORE",
            "channel": channel,
            "device_id": device_id,
        }, timeout=2.0)

    def feed_watchdog_generic(self, channel: str, device_id: int) -> None:
        """Send a heartbeat (watchdog feed) frame to any device_id."""
        self._send_command({
            "type": "FEED_WATCHDOG",
            "channel": channel,
            "device_id": device_id,
        })

    def calibrate_device(
        self,
        channel: str,
        device_id: int,
        timeout_ms: int = 90000,
    ) -> dict:
        """
        Enter MODE_CALIBRATION on device_id and wait for it to complete.
        Blocks until the device heartbeats back to IDLE/DISABLED, or timeout.
        Returns dict with 'status' ('OK', 'TIMEOUT', 'ERROR') and 'error_code'.
        """
        resp = self._send_command({
            "type": "CALIBRATE_DEVICE",
            "channel": channel,
            "device_id": device_id,
            "timeout_ms": timeout_ms,
        }, timeout=timeout_ms / 1000.0 + 5.0)
        return {
            "status": resp.get("status", "TIMEOUT"),
            "error_code": resp.get("error_code", 0),
        }

    # ------------------------------------------------------------------ #
    # Robot-like interface for routes                                      #
    # ------------------------------------------------------------------ #

    def get_actuator_by_name(self, name: str) -> DaemonActuatorProxy | None:
        return self._proxies.get(name)

    async def get_all_states(
        self,
        passive_kinematics: dict | None = None,
    ) -> dict[str, ActuatorState | None]:
        """
        Return ActuatorState for every joint.
        Uses the latest telemetry snapshot (avoids blocking the event loop).
        Falls back to a daemon GET_ALL_STATES command when no telemetry has arrived yet.
        ``passive_kinematics`` is accepted for interface compat but ignored.
        """
        with self._tel_lock:
            joint_snap = dict(self._joint_states)
            snap_age   = time.monotonic() - self._last_tel_time

        if not joint_snap or snap_age > _RUNNING_MAX_AGE:
            # No fresh telemetry — query daemon synchronously
            loop = asyncio.get_running_loop()
            try:
                joint_snap = await loop.run_in_executor(None, self.get_all_states_raw)
            except DaemonError:
                all_names = list(self.config.joints) if self.config else []
                return {n: None for n in all_names}

        result: dict[str, ActuatorState | None] = {}
        for name, d in joint_snap.items():
            try:
                result[name] = _daemon_state_to_actuator(d)
            except Exception:
                result[name] = None

        # Ensure joints present in config but absent from telemetry appear as None
        if self.config:
            for name in self.config.joints:
                result.setdefault(name, None)

        return result

    async def feed_all_watchdogs(self) -> None:
        """No-op — daemon feeds watchdogs from its 200 Hz control loop."""

    async def connect(self) -> None:
        """
        'Connect' in daemon mode = apply all configs so joints transition from
        OFFLINE → IDLE (daemon wakes each device and writes its config).
        """
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, self.apply_all_configs)

    async def read_calibration_from_devices(self) -> None:
        """No-op — calibration is loaded from the JSON config file, not via SDO."""

    async def disconnect(self) -> None:
        """
        No-op for normal disconnect.  routes_flash.py calls daemon_shutdown() explicitly
        when it needs the CAN bus back for the flash wizard.
        """

    def _rebuild_proxies(self) -> None:
        """Recreate per-joint proxies from the current config (call after config update)."""
        self._proxies.clear()
        if self.config is None:
            return
        for name, jc in self.config.joints.items():
            self._proxies[name] = DaemonActuatorProxy(
                name=name,
                can_id=jc.can_id,
                can_channel=jc.can_channel,
                config=jc,
                client=self,
            )

    # ------------------------------------------------------------------ #
    # CanMonitor-like interface for routes_devices and main.py            #
    # ------------------------------------------------------------------ #

    def get_interface_stats(self) -> list:
        """
        Return per-interface health dicts for all four canonical CAN buses.

        Always emits all four buses (can_left_leg, can_right_leg, can_left_arm,
        can_right_arm) so the frontend never falls back to a stale UNKNOWN default.

        State is derived by cross-checking daemon bus_health with sysfs operstate:
          - UNCONFIGURED: interface not present in OS (no udev rule / never plugged in)
          - DOWN:         interface exists in OS but link is down
          - UP:           interface is up AND daemon has an open socket

        This prevents the "shows UP when STOPPED" bug where the daemon can successfully
        bind a socket to a STOPPED interface and incorrectly report open=True.
        """
        with self._tel_lock:
            health      = dict(self._bus_health)
            joint_snap  = dict(self._joint_states)

        # Build per-bus joint lists from config + latest telemetry snapshot
        joints_by_bus: dict[str, list] = {}
        if self.config:
            for jname, jc in self.config.joints.items():
                bus = jc.can_channel
                if bus not in joints_by_bus:
                    joints_by_bus[bus] = []
                sd = joint_snap.get(jname, {})
                js = sd.get("state", "OFFLINE")
                status = "ONLINE" if js in ("IDLE", "ENABLED", "CALIBRATING") else "OFFLINE"
                joints_by_bus[bus].append({
                    "name":          jname,
                    "can_id":        jc.can_id,
                    "status":        status,
                    "position_rad":  sd.get("position"),
                    "velocity_rads": sd.get("velocity"),
                    "last_seen_ms":  None,
                })

        stats = []
        for ifname in _CANONICAL_BUSES:
            bh         = health.get(ifname, {})
            sysfs_state = _sysfs_iface_state(ifname)

            if sysfs_state == 'UNCONFIGURED':
                # Interface object doesn't exist in the OS — adapter not plugged in
                # or udev rule never written.
                final_state     = 'UNCONFIGURED'
                bus_error_state = 'UNCONFIGURED'
            elif sysfs_state == 'DOWN':
                # Interface exists (kernel has it) but link is down/stopped.
                # Daemon socket may still report open=True if it bound before the
                # interface went down — trust the kernel, not the daemon.
                final_state     = 'DOWN'
                bus_error_state = 'DOWN'
            else:
                # Kernel says UP; trust daemon's open flag for CAN-socket status.
                is_open         = bh.get("open", False)
                final_state     = 'UP' if is_open else 'DOWN'
                bus_error_state = 'ERROR-ACTIVE' if is_open else 'DOWN'

            bus_joints = joints_by_bus.get(ifname, [])
            online_cnt = sum(1 for j in bus_joints if j["status"] == "ONLINE")
            stats.append({
                "name":             ifname,
                "state":            final_state,
                "bus_error_state":  bus_error_state,
                "bitrate":          1_000_000,
                "rx_packets":       bh.get("rx_frames", 0),
                "tx_packets":       0,
                "rx_errors":        0,
                "tx_errors":        0,
                "rx_dropped":       0,
                "tx_dropped":       bh.get("tx_dropped", 0),
                "message_rate":     0.0,
                "rate_history":     [],
                "joints_online":    online_cnt,
                "joints_total":     len(bus_joints),
                "joints":           bus_joints,
                "usb_path":         "",
            })
        return stats

    def get_traffic(self) -> dict:
        """Return empty traffic dict — daemon does not expose per-ID traffic."""
        return {}

    async def pop_drop_events(self) -> list:
        """Return and clear any queued drop events (daemon doesn't push these)."""
        with self._drop_lock:
            evts = self._drop_events[:]
            self._drop_events.clear()
        return evts

    def get_passive_kinematics(self) -> dict:
        """No passive kinematics needed — daemon owns all PDO4 data."""
        return {}
