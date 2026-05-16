"""
Device / CAN interface endpoints.

GET  /devices                        — enumerate CAN interfaces (original)
GET  /devices/can/status             — per-interface health (sysfs + ip details)
GET  /devices/can/traffic            — per-ID live traffic for all buses
GET  /devices/can/adapters           — discover raw adapters + assignment status
POST /devices/can/assign             — assign USB serial → limb (live rename + udev rule)
POST /devices/can/unassign           — remove assignment from config
POST /devices/can/{name}/ping        — passive 2-second listen, returns responding device IDs
POST /devices/can/{name}/up          — bring an interface up at 1 Mbit/s
POST /ui/dismiss-setup               — mark the setup banner as permanently dismissed
GET  /devices/usb                    — list flash-relevant USB devices (ST-LINK, STM32 DFU)

NOTE: POST /devices/can/assign and POST /devices/can/{name}/up require
CAP_NET_ADMIN or sudo.  Grant it without full sudo:
    sudo setcap cap_net_admin+ep $(readlink -f $(which python3))
Or run the backend with:
    sudo -E python3 main.py
"""
from __future__ import annotations

import asyncio
import re
import subprocess
import time
from pathlib import Path

import can

from fastapi import APIRouter, Request
from fastapi.responses import JSONResponse
from pydantic import BaseModel

from humanoid import can_adapter

router = APIRouter(tags=["devices"])

_SYSFS_NET  = Path("/sys/class/net")
_SAFE_IFACE = re.compile(r'^[a-z][a-z0-9_]{1,14}$')   # e.g. can_left_leg


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _ok(data: object) -> dict:
    return {"success": True, "data": data, "error": None}


def _err(msg: str, status: int = 400) -> JSONResponse:
    return JSONResponse({"success": False, "data": None, "error": msg}, status_code=status)


async def _can_interfaces() -> list[dict]:
    """Read /sys/class/net and return basic info for each CAN interface."""
    def _scan() -> list[dict]:
        result = []
        if not _SYSFS_NET.exists():
            return result
        for entry in sorted(_SYSFS_NET.iterdir()):
            try:
                uevent = (entry / "uevent").read_text()
                if "DEVTYPE=can" not in uevent:
                    continue
                operstate = (entry / "operstate").read_text().strip()
                result.append({"name": entry.name, "operstate": operstate, "up": operstate == "up"})
            except OSError:
                continue
        return result

    return await asyncio.get_running_loop().run_in_executor(None, _scan)


# ---------------------------------------------------------------------------
# Request bodies
# ---------------------------------------------------------------------------

class _AssignBody(BaseModel):
    usb_serial: str
    limb: str       # e.g. "left_leg", "right_arm"


class _UnassignBody(BaseModel):
    usb_serial: str


# ---------------------------------------------------------------------------
# Routes — fixed paths first (must precede parameterised routes)
# ---------------------------------------------------------------------------

@router.get("/devices")
async def list_devices() -> dict:
    """List all CAN network interfaces found on this host."""
    try:
        interfaces = await _can_interfaces()
        return _ok({"interfaces": interfaces})
    except Exception as exc:
        return {"success": False, "data": None, "error": str(exc)}


@router.get("/devices/can/status", response_model=None)
async def can_status(request: Request) -> dict | JSONResponse:
    """Return per-interface health (stats + CAN error state) for all four buses."""
    monitor = getattr(request.app.state, 'can_monitor', None)
    if monitor is None:
        return _err("CAN monitor not initialized", 503)
    return _ok(monitor.get_interface_stats())


@router.get("/devices/can/traffic", response_model=None)
async def can_traffic(request: Request) -> dict | JSONResponse:
    """Return live per-CAN-ID traffic statistics for all buses."""
    monitor = getattr(request.app.state, 'can_monitor', None)
    if monitor is None:
        return _err("CAN monitor not initialized", 503)
    return _ok(monitor.get_traffic())


@router.get("/devices/can/adapters", response_model=None)
async def list_can_adapters(request: Request) -> dict | JSONResponse:
    """
    Discover all CAN adapters on the system and return their assignment status.

    Runs: ip -j link show type can  +  udevadm info per adapter.
    """
    config_path = getattr(request.app.state, 'config_path', None)
    if config_path is None:
        return _err("Config path not available", 503)
    try:
        data = await can_adapter.discover_adapters(config_path)
        return _ok(data)
    except Exception as exc:
        return _err(str(exc), status=500)


@router.post("/devices/can/assign", response_model=None)
async def assign_can_adapter(body: _AssignBody, request: Request) -> dict | JSONResponse:
    """
    Assign a USB-CAN adapter to a robot limb.

    Looks up the adapter's current interface name by USB serial, then:
      1. Brings it down
      2. Renames it to can_{limb}
      3. Sets bitrate 1000000
      4. Brings it up
      5. Writes a udev rule for persistence
      6. Saves assignment to the robot config
    """
    config_path = getattr(request.app.state, 'config_path', None)
    if config_path is None:
        return _err("Config path not available", 503)

    # Discover to get current interface name for this serial
    try:
        discovery = await can_adapter.discover_adapters(config_path)
    except Exception as exc:
        return _err(f"Discovery failed: {exc}", status=500)

    adapter = next(
        (a for a in discovery['adapters'] if a['usb_serial'] == body.usb_serial),
        None,
    )
    if adapter is None:
        return _err(f"No adapter found with serial {body.usb_serial!r}", status=404)

    try:
        result = await can_adapter.assign_adapter(
            adapter['current_name'], body.usb_serial, body.limb, config_path
        )
        return _ok(result)
    except (RuntimeError, ValueError) as exc:
        return _err(str(exc), status=500)


@router.post("/devices/can/unassign", response_model=None)
async def unassign_can_adapter(body: _UnassignBody, request: Request) -> dict | JSONResponse:
    """Remove a serial→limb assignment from the robot config (keeps udev rule)."""
    config_path = getattr(request.app.state, 'config_path', None)
    if config_path is None:
        return _err("Config path not available", 503)
    try:
        result = await can_adapter.unassign_adapter(body.usb_serial, config_path)
        return _ok(result)
    except ValueError as exc:
        return _err(str(exc), status=404)
    except Exception as exc:
        return _err(str(exc), status=500)


@router.post("/ui/dismiss-setup", response_model=None)
async def dismiss_setup(request: Request) -> dict | JSONResponse:
    """Permanently dismiss the setup banner (stored in configs/ui_state.json)."""
    config_path = getattr(request.app.state, 'config_path', None)
    if config_path is None:
        return _err("Config path not available", 503)
    try:
        can_adapter.dismiss_setup_banner(config_path)
        return _ok({"dismissed": True})
    except Exception as exc:
        return _err(str(exc), status=500)


# ---------------------------------------------------------------------------
# Parameterised routes — must come after all fixed /devices/can/* paths
# ---------------------------------------------------------------------------

@router.post("/devices/can/{name}/scan", response_model=None)
async def scan_bus(name: str, request: Request) -> dict | JSONResponse:
    """
    Actively ping every CAN ID on a bus (IDs 1-20 plus any expected from config).

    For each ID: sends RECEIVE_PDO_1 with magic byte 0xCA, waits up to 100ms
    for TRANSMIT_PDO_1 echoing 0xCA back.  Returns which IDs responded, timing,
    and whether each was expected per the robot config.
    """
    if not _SAFE_IFACE.match(name):
        return _err(f"Invalid interface name: {name!r}")

    if not (_SYSFS_NET / name).exists():
        return _err(f"Interface {name!r} does not exist", status=404)

    # Build expected-ID → joint_name lookup from loaded config
    config = getattr(request.app.state, 'config', None)
    expected: dict[int, str] = {}
    if config is not None:
        for joint in config.joints.values():
            if joint.can_channel == name:
                expected[joint.can_id] = joint.joint_name

    scan_ids = sorted(set(expected.keys()) | set(range(1, 21)))

    # RECEIVE_PDO_1 = 0x4, TRANSMIT_PDO_1 = 0x3
    _RECV_PDO1_BASE = 0x4 << 7   # 0x200 — arb_id prefix we send to
    _XMIT_PDO1_BASE = 0x3 << 7   # 0x180 — arb_id prefix we listen for

    loop = asyncio.get_running_loop()

    def _do_scan() -> list[dict]:
        try:
            bus = can.interface.Bus(interface='socketcan', channel=name)
        except Exception as exc:
            raise RuntimeError(str(exc))

        results: list[dict] = []
        try:
            for device_id in scan_ids:
                arb_tx = _RECV_PDO1_BASE | device_id
                arb_rx = _XMIT_PDO1_BASE | device_id

                t0 = time.perf_counter()
                bus.send(can.Message(
                    arbitration_id=arb_tx,
                    is_extended_id=False,
                    data=b'\xCA',
                ))

                deadline = time.perf_counter() + 0.1
                responded = False
                response_time_ms = None
                raw_response = None

                while time.perf_counter() < deadline:
                    remaining = deadline - time.perf_counter()
                    msg = bus.recv(timeout=max(0.0, remaining))
                    if msg is None:
                        break
                    if msg.is_error_frame:
                        continue
                    if msg.arbitration_id == arb_rx and len(msg.data) >= 1 and msg.data[0] == 0xCA:
                        response_time_ms = round((time.perf_counter() - t0) * 1000, 1)
                        raw_response = ' '.join(f'{b:02X}' for b in msg.data)
                        responded = True
                        break
                    # Non-matching frame (e.g. autonomous PDO4 broadcasts) — keep waiting

                results.append({
                    'can_id':          device_id,
                    'joint_name':      expected.get(device_id),
                    'expected':        device_id in expected,
                    'responded':       responded,
                    'response_time_ms': response_time_ms,
                    'raw_response':    raw_response,
                })
        finally:
            bus.shutdown()
        return results

    try:
        t_start = loop.time()
        results = await loop.run_in_executor(None, _do_scan)
        scan_ms = round((loop.time() - t_start) * 1000)
    except RuntimeError as exc:
        return _err(str(exc), status=500)

    expected_found    = sum(1 for r in results if r['expected'] and r['responded'])
    expected_missing  = sum(1 for r in results if r['expected'] and not r['responded'])
    unexpected_found  = sum(1 for r in results if not r['expected'] and r['responded'])

    return _ok({
        'bus':              name,
        'scan_duration_ms': scan_ms,
        'results':          results,
        'summary': {
            'expected_found':   expected_found,
            'expected_missing': expected_missing,
            'unexpected_found': unexpected_found,
        },
    })


@router.post("/devices/can/{name}/ping", response_model=None)
async def ping_bus(name: str) -> dict | JSONResponse:
    """
    Passively listen on a CAN bus for 2 seconds and return all device IDs that
    transmitted at least one frame.  Useful to verify an adapter is wired up.
    """
    if not _SAFE_IFACE.match(name):
        return _err(f"Invalid interface name: {name!r}")

    if not (_SYSFS_NET / name).exists():
        return _err(f"Interface {name!r} does not exist — assign and bring up first", status=404)

    loop = asyncio.get_running_loop()

    def _listen() -> list[int]:
        try:
            bus = can.interface.Bus(interface='socketcan', channel=name)
        except Exception as exc:
            raise RuntimeError(str(exc))
        device_ids: set[int] = set()
        deadline = time.monotonic() + 2.0
        try:
            while time.monotonic() < deadline:
                remaining = deadline - time.monotonic()
                msg = bus.recv(timeout=max(0.0, remaining))
                if msg is None or msg.is_error_frame:
                    continue
                device_ids.add(msg.arbitration_id & 0x7F)
        finally:
            bus.shutdown()
        return sorted(device_ids)

    try:
        device_ids = await loop.run_in_executor(None, _listen)
        return _ok({"interface": name, "device_ids": device_ids, "count": len(device_ids)})
    except RuntimeError as exc:
        return _err(str(exc), status=500)


@router.post("/devices/can/{name}/up", response_model=None)
async def bring_interface_up(name: str) -> dict | JSONResponse:
    """
    Bring a CAN interface up at 1 Mbit/s (uses sudo — passwordless rule required).
    """
    if not _SAFE_IFACE.match(name):
        return _err(f"Invalid interface name: {name!r}")

    async def _run(cmd: list[str]) -> tuple[int, str]:
        p = await asyncio.create_subprocess_exec(
            'sudo', '-n', *cmd,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
        )
        try:
            _, stderr = await asyncio.wait_for(p.communicate(), timeout=10.0)
        except asyncio.TimeoutError:
            try:
                p.kill()
                await p.wait()
            except Exception:
                pass
            return -1, 'command timed out'
        return p.returncode, stderr.decode().strip()

    await _run(['/sbin/ip', 'link', 'set', name, 'down'])

    rc, err = await _run(['/sbin/ip', 'link', 'set', name, 'type', 'can', 'bitrate', '1000000'])
    if rc != 0:
        return _err(err or "ip link set type can failed", status=500)

    rc, err = await _run(['/sbin/ip', 'link', 'set', name, 'up'])
    if rc != 0:
        return _err(err or "ip link set up failed", status=500)

    return _ok({"interface": name, "state": "UP", "bitrate": 1_000_000})


# ---------------------------------------------------------------------------
# USB device detection
# ---------------------------------------------------------------------------

# VID:PID → (type, label)
_USB_KNOWN: dict[str, tuple[str, str]] = {
    "0483:374b": ("stlink", "ST-LINK/V2.1 — SWD programmer ready"),
    "0483:3748": ("stlink", "ST-LINK/V2 — SWD programmer ready"),
    "0483:df11": ("dfu",    "STM32 ESC in DFU mode — ready to flash"),
}

_LSUSB_LINE_RE = re.compile(
    r"Bus\s+(\d+)\s+Device\s+(\d+):\s+ID\s+([0-9a-fA-F]{4}:[0-9a-fA-F]{4})\s*(.*)"
)


@router.get("/devices/usb")
async def get_usb_devices() -> dict:
    """Return flash-relevant USB devices: ST-LINK programmers and STM32 DFU-mode ESCs."""
    try:
        result = subprocess.run(
            ["lsusb"], capture_output=True, text=True, timeout=5
        )
        lines = result.stdout.splitlines()
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return _ok({"devices": []})

    devices = []
    for line in lines:
        m = _LSUSB_LINE_RE.match(line)
        if not m:
            continue
        bus, dev, vid_pid, desc = m.group(1), m.group(2), m.group(3).lower(), m.group(4).strip()
        if vid_pid in _USB_KNOWN:
            kind, label = _USB_KNOWN[vid_pid]
            devices.append({
                "type":    kind,
                "vid_pid": vid_pid,
                "label":   label,
                "bus":     int(bus),
                "device":  int(dev),
            })

    return _ok({"devices": devices})
