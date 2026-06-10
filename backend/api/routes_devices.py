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
from pathlib import Path

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
    Active CAN bus scan.
    Not available while the C++ daemon owns the CAN sockets.
    Device reachability is visible via GET /devices/can/status (daemon telemetry).
    """
    return _err(
        "Active bus scan requires direct CAN access; "
        "the daemon currently owns all CAN sockets. "
        "Use GET /devices/can/status to check device reachability.",
        503,
    )


@router.post("/devices/can/{name}/ping", response_model=None)
async def ping_bus(name: str) -> dict | JSONResponse:
    """
    Passive CAN bus listen.
    Not available while the C++ daemon owns the CAN sockets.
    Device reachability is visible via GET /devices/can/status (daemon telemetry).
    """
    return _err(
        "Passive bus listen requires direct CAN access; "
        "the daemon currently owns all CAN sockets. "
        "Use GET /devices/can/status to check device reachability.",
        503,
    )


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

    # Try with restart-ms 100 first; some adapters (e.g. gs_usb/CANable) don't
    # support hardware bus-off restart and will reject the parameter.
    rc, err = await _run(['/sbin/ip', 'link', 'set', name, 'type', 'can',
                          'bitrate', '1000000', 'restart-ms', '100'])
    restart_ms = 100
    if rc != 0:
        rc, err = await _run(['/sbin/ip', 'link', 'set', name, 'type', 'can',
                               'bitrate', '1000000'])
        restart_ms = 0
        if rc != 0:
            return _err(err or "ip link set type can failed", status=500)

    rc, err = await _run(['/sbin/ip', 'link', 'set', name, 'txqueuelen', '1000'])
    if rc != 0:
        return _err(err or "ip link set txqueuelen failed", status=500)

    rc, err = await _run(['/sbin/ip', 'link', 'set', name, 'up'])
    if rc != 0:
        return _err(err or "ip link set up failed", status=500)

    return _ok({"interface": name, "state": "UP", "bitrate": 1_000_000,
                "restart_ms": restart_ms, "txqueuelen": 1000})


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
