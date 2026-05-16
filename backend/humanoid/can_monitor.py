"""
CAN bus health monitoring.

Two background tasks per interface:
  - poll_loop  : reads sysfs + ip-details every 2 s for packet/error counters
  - sniff_loop : opens a read-only socketcan socket to record per-ID frame rates

Collected data is accessed via get_interface_stats() and get_traffic().
State-change events (UP ↔ DOWN) are queued via pop_drop_events().
"""
from __future__ import annotations

import asyncio
import datetime
import json
import logging
import os
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

import can

from .robot_config import RobotConfig
from . import recoil_protocol as proto

_log = logging.getLogger(__name__)

ALL_BUSES       = ['can_left_leg', 'can_right_leg', 'can_left_arm', 'can_right_arm']
_SYSFS_NET      = Path('/sys/class/net')
_TRAFFIC_WINDOW = 5.0    # seconds for rate calculation
_POLL_INTERVAL  = 2.0    # seconds between sysfs reads
_STALE_CUTOFF   = 10.0   # seconds after which a traffic entry is hidden
_MAX_RATE_HIST  = 30     # rate samples kept per interface (~60 s at 2 s interval)

# Passive joint detection thresholds
_JOINT_ONLINE_CUTOFF = 2.0   # last-seen age → ONLINE
_JOINT_STALE_CUTOFF  = 10.0  # last-seen age → STALE (beyond = OFFLINE)

_FUNC_NAMES          = proto.FUNC_NAMES
_TELEMETRY_FUNC_IDS  = proto.TELEMETRY_FUNC_IDS


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

@dataclass
class IfaceStats:
    name: str
    state: str           = 'UNKNOWN'        # UP / DOWN / UNKNOWN / UNCONFIGURED
    bus_error_state: str = 'UNKNOWN'        # ERROR-ACTIVE / ERROR-WARNING / ERROR-PASSIVE / BUS-OFF
    bitrate: int         = 0
    rx_packets: int      = 0
    tx_packets: int      = 0
    rx_errors:  int      = 0
    tx_errors:  int      = 0
    rx_dropped: int      = 0
    tx_dropped: int      = 0
    message_rate: float  = 0.0
    rate_history: list   = field(default_factory=list)
    joints_online: int   = 0
    joints_total:  int   = 0
    usb_path: str        = ''

    def to_dict(self) -> dict:
        return {
            'name':            self.name,
            'state':           self.state,
            'bus_error_state': self.bus_error_state,
            'bitrate':         self.bitrate,
            'rx_packets':      self.rx_packets,
            'tx_packets':      self.tx_packets,
            'rx_errors':       self.rx_errors,
            'tx_errors':       self.tx_errors,
            'rx_dropped':      self.rx_dropped,
            'tx_dropped':      self.tx_dropped,
            'message_rate':    self.message_rate,
            'rate_history':    self.rate_history[-_MAX_RATE_HIST:],
            'joints_online':   self.joints_online,
            'joints_total':    self.joints_total,
            'usb_path':        self.usb_path,
        }


@dataclass
class TrafficEntry:
    arb_id:       int
    bus_name:     str
    joint_name:   str | None
    func_name:    str
    device_id:    int
    last_data:    bytes  = b''
    last_seen:    float  = field(default_factory=time.time)
    timestamps:   deque  = field(default_factory=deque)   # ring of recent timestamps
    position_rad: float | None = None   # decoded from TX_PDO2/TX_PDO4
    velocity_rads: float | None = None  # decoded from TX_PDO2/TX_PDO4

    @property
    def message_rate(self) -> float:
        now = time.time()
        while self.timestamps and now - self.timestamps[0] > _TRAFFIC_WINDOW:
            self.timestamps.popleft()
        return len(self.timestamps) / _TRAFFIC_WINDOW

    def to_dict(self) -> dict:
        now = time.time()
        R2D = 180.0 / 3.141592653589793
        return {
            'arb_id':        self.arb_id,
            'arb_id_hex':    f'0x{self.arb_id:03X}',
            'node_id':       self.device_id,
            'joint_name':    self.joint_name,
            'func_name':     self.func_name,
            'message_rate':  round(self.message_rate, 1),
            'last_data':     self.last_data.hex(' ') if self.last_data else '',
            'last_seen':     self.last_seen,
            'last_seen_ago': round(now - self.last_seen, 2),
            'position_deg':  round(self.position_rad * R2D, 2) if self.position_rad is not None else None,
            'velocity_dps':  round(self.velocity_rads * R2D, 2) if self.velocity_rads is not None else None,
        }


@dataclass
class DropEvent:
    interface:  str
    timestamp:  str   # ISO 8601
    event:      str   # 'down' or 'up'
    rx_errors:  int = 0
    tx_errors:  int = 0

    def to_dict(self) -> dict:
        return {
            'type':      'can_drop_event',
            'interface': self.interface,
            'timestamp': self.timestamp,
            'event':     self.event,
            'rx_errors': self.rx_errors,
            'tx_errors': self.tx_errors,
        }


# ---------------------------------------------------------------------------
# Sysfs helpers
# ---------------------------------------------------------------------------

def _read_sysfs_int(path: Path) -> int:
    try:
        return int(path.read_text().strip())
    except (OSError, ValueError):
        return 0


def _read_usb_path(iface_name: str) -> str:
    try:
        real = os.path.realpath(f'/sys/class/net/{iface_name}')
        # real ends with  .../usb-port/usb-dev/usb-dev:1.0/net/can_xxx
        # USB device node is 3 levels up from the tail
        parts = real.split('/')
        idx = next((i for i, p in enumerate(parts) if p == 'net'), None)
        if idx and idx >= 2:
            return '/'.join(parts[:idx - 1])
        return real
    except Exception:
        return ''


async def _ip_details_json(iface_name: str) -> dict:
    """Run `ip -details -json link show {iface}` and return first element."""
    try:
        proc = await asyncio.create_subprocess_exec(
            'ip', '-details', '-json', 'link', 'show', iface_name,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.DEVNULL,
        )
        stdout, _ = await asyncio.wait_for(proc.communicate(), timeout=3.0)
        data = json.loads(stdout.decode())
        return data[0] if isinstance(data, list) and data else {}
    except Exception:
        return {}


# ---------------------------------------------------------------------------
# CanMonitor
# ---------------------------------------------------------------------------

class CanMonitor:
    """
    Background health monitor for all four CAN buses.

    Call await monitor.start() to launch background tasks.
    Call await monitor.stop() to cancel them cleanly.
    """

    def __init__(self, config: RobotConfig | None) -> None:
        self._stats: dict[str, IfaceStats] = {n: IfaceStats(name=n) for n in ALL_BUSES}
        self._traffic: dict[str, dict[int, TrafficEntry]] = {n: {} for n in ALL_BUSES}
        self._drop_events: list[DropEvent] = []
        self._drop_lock = asyncio.Lock()
        self._prev_states: dict[str, str] = {}

        # (bus_name, device_id) → joint_name
        self._joint_lookup: dict[tuple[str, int], str] = {}
        joints_per_bus: dict[str, int] = {n: 0 for n in ALL_BUSES}
        if config:
            for joint_name, jcfg in config:
                self._joint_lookup[(jcfg.can_channel, jcfg.can_id)] = joint_name
                joints_per_bus[jcfg.can_channel] = joints_per_bus.get(jcfg.can_channel, 0) + 1
        for name in ALL_BUSES:
            self._stats[name].joints_total = joints_per_bus.get(name, 0)

        # Passive joint detection: last timestamp a frame was seen from each joint
        self._joint_last_seen: dict[tuple[str, int], float] = {}
        # Decoded telemetry from passive broadcast frames
        self._joint_telemetry: dict[tuple[str, int], dict] = {}

        self._running = False
        self._tasks: list[asyncio.Task] = []

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    async def start(self) -> None:
        self._running = True
        self._tasks.append(asyncio.create_task(self._poll_loop(), name='can_poll'))
        for bus in ALL_BUSES:
            self._tasks.append(asyncio.create_task(
                self._sniff_bus(bus), name=f'can_sniff_{bus}'
            ))
        _log.info('CanMonitor started')

    async def stop(self) -> None:
        self._running = False
        for t in self._tasks:
            t.cancel()
        await asyncio.gather(*self._tasks, return_exceptions=True)
        self._tasks.clear()
        _log.info('CanMonitor stopped')

    # ── Public API ────────────────────────────────────────────────────────────

    def _joint_status_entry(self, bus_name: str, device_id: int, joint_name: str, now: float) -> dict:
        last_ts = self._joint_last_seen.get((bus_name, device_id))
        if last_ts is None:
            status = 'OFFLINE'
            last_seen_ms = None
        else:
            age = now - last_ts
            if age < _JOINT_ONLINE_CUTOFF:
                status = 'ONLINE'
            elif age < _JOINT_STALE_CUTOFF:
                status = 'STALE'
            else:
                status = 'OFFLINE'
            last_seen_ms = round(age * 1000)
        t = self._joint_telemetry.get((bus_name, device_id), {})
        return {
            'can_id':        device_id,
            'name':          joint_name,
            'status':        status,
            'last_seen_ms':  last_seen_ms,
            'position_rad':  t.get('position'),
            'velocity_rads': t.get('velocity'),
        }

    def get_interface_stats(self) -> list[dict]:
        now = time.time()
        result = []
        for bus_name in ALL_BUSES:
            stats = self._stats[bus_name].to_dict()
            joints = []
            for (b, dev_id), jname in self._joint_lookup.items():
                if b != bus_name:
                    continue
                joints.append(self._joint_status_entry(bus_name, dev_id, jname, now))
            joints.sort(key=lambda j: j['can_id'])
            online_count = sum(1 for j in joints if j['status'] == 'ONLINE')
            stats['joints'] = joints
            stats['joints_online'] = online_count  # override with passive detection
            result.append(stats)
        return result

    def get_traffic(self) -> dict[str, list[dict]]:
        now = time.time()
        result: dict[str, list[dict]] = {}
        for bus in ALL_BUSES:
            entries = sorted(
                (e.to_dict() for e in self._traffic[bus].values()
                 if now - e.last_seen < _STALE_CUTOFF),
                key=lambda e: e['arb_id'],
            )
            result[bus] = entries
        return result

    async def pop_drop_events(self) -> list[dict]:
        async with self._drop_lock:
            out = [e.to_dict() for e in self._drop_events]
            self._drop_events.clear()
        return out

    def update_joints_online(self, online_joints: set[str]) -> None:
        """Called by the WebSocket loop to keep per-bus online counts fresh."""
        for bus in ALL_BUSES:
            count = sum(
                1 for (b, _), jn in self._joint_lookup.items()
                if b == bus and jn in online_joints
            )
            self._stats[bus].joints_online = count

    # ── Sysfs poll loop ───────────────────────────────────────────────────────

    async def _poll_loop(self) -> None:
        while self._running:
            try:
                await asyncio.gather(*[self._poll_iface(n) for n in ALL_BUSES])
            except Exception as exc:
                _log.debug('Poll error: %s', exc)
            await asyncio.sleep(_POLL_INTERVAL)

    async def _poll_iface(self, name: str) -> None:
        stats = self._stats[name]
        sysfs = _SYSFS_NET / name

        if not sysfs.exists():
            await self._check_state_change(name, stats, 'UNCONFIGURED')
            stats.state = 'UNCONFIGURED'
            stats.bus_error_state = 'UNKNOWN'
            return

        loop = asyncio.get_running_loop()

        def _read() -> dict:
            stat_dir = sysfs / 'statistics'
            return {
                'operstate': (sysfs / 'operstate').read_text().strip(),
                'rx_packets': _read_sysfs_int(stat_dir / 'rx_packets'),
                'tx_packets': _read_sysfs_int(stat_dir / 'tx_packets'),
                'rx_errors':  _read_sysfs_int(stat_dir / 'rx_errors'),
                'tx_errors':  _read_sysfs_int(stat_dir / 'tx_errors'),
                'rx_dropped': _read_sysfs_int(stat_dir / 'rx_dropped'),
                'tx_dropped': _read_sysfs_int(stat_dir / 'tx_dropped'),
            }

        try:
            c = await loop.run_in_executor(None, _read)
        except OSError:
            stats.state = 'UNKNOWN'
            return

        new_state = 'UP' if c['operstate'] == 'up' else 'DOWN'
        stats.rx_packets = c['rx_packets']
        stats.tx_packets = c['tx_packets']
        stats.rx_errors  = c['rx_errors']
        stats.tx_errors  = c['tx_errors']
        stats.rx_dropped = c['rx_dropped']
        stats.tx_dropped = c['tx_dropped']

        if new_state == 'UP':
            ip_data   = await _ip_details_json(name)
            info_data = ip_data.get('linkinfo', {}).get('info_data', {})
            raw_state = info_data.get('state', '').upper()
            stats.bus_error_state = raw_state if raw_state in {
                'ERROR-ACTIVE', 'ERROR-WARNING', 'ERROR-PASSIVE', 'BUS-OFF'
            } else 'UNKNOWN'
            stats.bitrate = info_data.get('bittiming', {}).get('bitrate', 0)
        else:
            stats.bus_error_state = 'UNKNOWN'

        if not stats.usb_path:
            stats.usb_path = await loop.run_in_executor(None, _read_usb_path, name)

        # Compute message rate from traffic timestamps
        now = time.time()
        ts_count = sum(
            1 for entry in self._traffic[name].values()
            for ts in entry.timestamps if now - ts <= _TRAFFIC_WINDOW
        )
        stats.message_rate = round(ts_count / _TRAFFIC_WINDOW, 1)
        stats.rate_history.append(stats.message_rate)
        if len(stats.rate_history) > _MAX_RATE_HIST:
            stats.rate_history.pop(0)

        await self._check_state_change(name, stats, new_state)
        stats.state = new_state

    async def _check_state_change(self, name: str, stats: IfaceStats, new_state: str) -> None:
        prev = self._prev_states.get(name)
        if prev is not None and prev != new_state:
            # Don't emit drop events when the interface is simply not configured yet
            if 'UNCONFIGURED' not in (prev, new_state):
                ts = datetime.datetime.utcnow().isoformat() + 'Z'
                event = DropEvent(
                    interface=name,
                    timestamp=ts,
                    event='down' if new_state == 'DOWN' else 'up',
                    rx_errors=stats.rx_errors,
                    tx_errors=stats.tx_errors,
                )
                async with self._drop_lock:
                    self._drop_events.append(event)
            _log.debug('CAN %s: %s → %s', name, prev, new_state)
        self._prev_states[name] = new_state

    # ── Traffic sniffer ───────────────────────────────────────────────────────

    async def _sniff_bus(self, bus_name: str) -> None:
        loop = asyncio.get_running_loop()
        while self._running:
            # Don't attempt to open a socket if the interface doesn't exist yet.
            # This avoids ERROR log spam when adapters are not yet renamed by udev.
            if not (_SYSFS_NET / bus_name).exists():
                await asyncio.sleep(5.0)
                continue

            try:
                raw_bus = await loop.run_in_executor(
                    None,
                    lambda: can.interface.Bus(interface='socketcan', channel=bus_name),
                )
            except Exception as exc:
                _log.debug('Sniff open %s failed (%s) — retry in 5 s', bus_name, exc)
                await asyncio.sleep(5.0)
                continue

            _log.info('Traffic sniffer active: %s', bus_name)

            def _recv():
                return raw_bus.recv(timeout=0.1)

            try:
                while self._running:
                    msg = await loop.run_in_executor(None, _recv)
                    if msg is None:
                        continue
                    if msg.is_error_frame:
                        _log.debug('Error frame on %s arb_id=0x%X', bus_name, msg.arbitration_id)
                        continue
                    self._record_frame(bus_name, msg)
            except Exception as exc:
                if self._running:
                    _log.debug('Sniff %s error: %s — reconnecting', bus_name, exc)
            finally:
                try:
                    raw_bus.shutdown()
                except Exception:
                    pass

            if self._running:
                await asyncio.sleep(2.0)

    def _record_frame(self, bus_name: str, msg: can.Message) -> None:
        arb_id     = msg.arbitration_id
        device_id, func_id = proto.decode_arb_id(arb_id)
        func_name  = _FUNC_NAMES.get(func_id, f'FN:0x{func_id:X}')
        joint_name = self._joint_lookup.get((bus_name, device_id))

        entry = self._traffic[bus_name].get(arb_id)
        if entry is None:
            entry = TrafficEntry(
                arb_id=arb_id, bus_name=bus_name,
                joint_name=joint_name, func_name=func_name, device_id=device_id,
            )
            self._traffic[bus_name][arb_id] = entry

        now = time.time()
        entry.last_data = bytes(msg.data)
        entry.last_seen = now
        entry.timestamps.append(now)

        # Prune old timestamps to keep the deque bounded
        cutoff = now - _TRAFFIC_WINDOW
        while entry.timestamps and entry.timestamps[0] < cutoff:
            entry.timestamps.popleft()

        # Decode position/velocity from telemetry frames
        if func_id in _TELEMETRY_FUNC_IDS:
            tel = proto.decode_telemetry(bytes(msg.data))
            if tel is not None:
                pos, vel = tel
                entry.position_rad  = pos
                entry.velocity_rads = vel
                # Passive joint detection
                if joint_name is not None:
                    key = (bus_name, device_id)
                    self._joint_last_seen[key] = now
                    self._joint_telemetry[key] = {'position': pos, 'velocity': vel}
        elif joint_name is not None:
            # Non-telemetry frame still counts as activity for joint detection
            self._joint_last_seen[(bus_name, device_id)] = now
