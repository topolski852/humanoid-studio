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
from typing import Callable

from humanoid.actuator import ActuatorState
from humanoid.can_bus import Mode
from humanoid.robot_config import JointConfig, RobotConfig

_log = logging.getLogger(__name__)

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


def _daemon_state_to_actuator(d: dict) -> ActuatorState:
    """Convert a daemon state dict into an ActuatorState pydantic object."""
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

    Operations not supported by the daemon (calibrate_offset, store_to_flash,
    read_config_from_device, load_from_flash) raise DaemonNotSupportedError so
    callers can return a meaningful HTTP 503.
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
        """Tell daemon to write this joint's config (from loaded JSON) to device RAM."""
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, self._client.apply_config, self._name)

    async def feed_watchdog(self) -> None:
        """No-op — daemon feeds watchdogs from its 200 Hz control loop."""

    # -- Unsupported operations (require direct CAN bus access) --

    async def calibrate_offset(
        self,
        timeout: float = 90.0,
        on_progress: Callable[[str], None] | None = None,
    ) -> float:
        raise DaemonNotSupportedError(
            "calibrate_offset requires direct CAN access; "
            "the daemon owns the bus — stop the daemon first"
        )

    async def store_to_flash(self) -> None:
        raise DaemonNotSupportedError(
            "store_to_flash requires direct CAN access; "
            "the daemon owns the bus — stop the daemon first"
        )

    async def load_from_flash(self) -> None:
        raise DaemonNotSupportedError(
            "load_from_flash requires direct CAN access; "
            "the daemon owns the bus — stop the daemon first"
        )

    async def read_config_from_device(self) -> dict:
        raise DaemonNotSupportedError(
            "read_config_from_device requires direct CAN access; "
            "the daemon owns the bus — stop the daemon first"
        )

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

    def _send_command(self, msg: dict) -> dict:
        """
        Send a JSON command to the daemon and wait for a matching response.
        Thread-safe via _cmd_lock.  Raises on timeout or daemon error.
        """
        if self._cmd_sock is None:
            raise DaemonNotRunningError("DaemonClient not started — call start() first")

        msg.setdefault("id", str(uuid.uuid4()))
        payload = json.dumps(msg).encode()

        with self._cmd_lock:
            try:
                self._cmd_sock.sendto(payload, (self._daemon_host, self._cmd_port))
                raw, _ = self._cmd_sock.recvfrom(65535)
            except socket.timeout:
                raise DaemonNotRunningError(
                    f"Daemon did not respond within {self._cmd_timeout}s "
                    f"(type={msg.get('type')})"
                )
            except OSError as exc:
                raise DaemonNotRunningError(f"Daemon socket error: {exc}") from exc

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

    def apply_config(self, joint_name: str) -> None:
        self._send_command({"type": "APPLY_CONFIG", "joint_name": joint_name})

    def apply_all_configs(self) -> None:
        self._send_command({"type": "APPLY_ALL_CONFIGS"})

    def daemon_shutdown(self) -> None:
        """Ask the daemon to shut itself down gracefully."""
        try:
            self._send_command({"type": "SHUTDOWN"})
        except DaemonError:
            pass  # daemon may already be gone

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
        Return a list of interface-stat dicts built from the latest bus_health telemetry.
        Matches the shape that routes_devices.py / the can_health WS message expects.
        """
        with self._tel_lock:
            health = dict(self._bus_health)

        stats = []
        for ifname, bh in health.items():
            stats.append({
                "name":          ifname,
                "open":          bh.get("open", False),
                "tx_dropped":    bh.get("tx_dropped", 0),
                "rx_frames":     bh.get("rx_frames", 0),
                # Fields CanMonitor tracked but daemon doesn't — zero-fill for compat
                "tx_frames":     0,
                "rx_errors":     0,
                "tx_errors":     0,
                "joints_online": [],
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
