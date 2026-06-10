"""
CAN adapter discovery and assignment.

Discovers raw CAN interfaces (can0, can1, ...) via 'ip -j link show type can'.
Reads USB serial numbers via udevadm to produce stable identifiers.
Assigns adapters to limbs by live-renaming the interface and writing persistent
udev rules to /etc/udev/rules.d/99-humanoid-can.rules.

Assignments are persisted in configs/humanoid_lite.json under 'can_assignments':
  { "USB_SERIAL": "left_leg", ... }

Requires sudo for ip link set name / ip link set up.
Long-term: grant cap_net_admin to the python binary instead.
"""
from __future__ import annotations

import asyncio
import json
import logging
import re
from pathlib import Path

_log = logging.getLogger(__name__)

LIMBS = ['left_leg', 'right_leg', 'left_arm', 'right_arm']
_UDEV_RULES_PATH = Path('/etc/udev/rules.d/99-humanoid-can.rules')
_SAFE_LIMB = re.compile(r'^(left|right)_(leg|arm)$')


# ---------------------------------------------------------------------------
# Shell helpers
# ---------------------------------------------------------------------------

async def _run(*cmd: str, sudo: bool = False, timeout: float = 10.0) -> tuple[int, str, str]:
    # -n (non-interactive): fail immediately instead of prompting for a password.
    # Passwordless sudo is required — see /etc/sudoers.d/humanoid-can.
    full = (['sudo', '-n'] if sudo else []) + list(cmd)
    proc = None
    try:
        proc = await asyncio.create_subprocess_exec(
            *full,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
        )
        stdout, stderr = await asyncio.wait_for(proc.communicate(), timeout=timeout)
        return proc.returncode, stdout.decode().strip(), stderr.decode().strip()
    except asyncio.TimeoutError:
        if proc is not None:
            try:
                proc.kill()
                await proc.wait()
            except Exception:
                pass
        return -1, '', 'command timed out'
    except Exception as exc:
        return -1, '', str(exc)


async def _get_usb_info(iface_name: str) -> dict:
    """Return USB serial and port path for a CAN interface via udevadm."""
    rc, out, _ = await _run(
        '/usr/bin/udevadm', 'info', f'/sys/class/net/{iface_name}', '--query=property'
    )
    info: dict = {'usb_serial': None, 'usb_path': None}
    if rc != 0:
        return info
    for line in out.splitlines():
        k, _, v = line.partition('=')
        if k == 'ID_SERIAL_SHORT':
            info['usb_serial'] = v
        elif k == 'ID_PATH':
            info['usb_path'] = v
    return info


# ---------------------------------------------------------------------------
# Config helpers
# ---------------------------------------------------------------------------

def _load_assignments(config_path: Path) -> dict[str, str]:
    """Load {usb_serial: limb} from the robot config.  Returns {} on error."""
    try:
        return json.loads(config_path.read_text()).get('can_assignments', {})
    except Exception:
        return {}


def _save_assignments(config_path: Path, assignments: dict[str, str]) -> None:
    try:
        data = json.loads(config_path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"Failed to read config: {exc}") from exc
    data['can_assignments'] = assignments
    content = json.dumps(data, indent=2)
    tmp = config_path.with_name(config_path.name + '.tmp')
    try:
        tmp.write_text(content)
        tmp.replace(config_path)
    except OSError as exc:
        tmp.unlink(missing_ok=True)
        raise RuntimeError(f"Failed to save assignments: {exc}") from exc


def _joints_per_limb(config_path: Path) -> dict[str, int]:
    counts: dict[str, int] = {limb: 0 for limb in LIMBS}
    try:
        data = json.loads(config_path.read_text())
        for joint in data.get('joints', {}).values():
            ch = joint.get('can_channel', '')
            for limb in LIMBS:
                if ch == f'can_{limb}':
                    counts[limb] += 1
    except Exception:
        pass
    return counts


def _load_ui_state(config_path: Path) -> dict:
    try:
        return json.loads((config_path.parent / 'ui_state.json').read_text())
    except Exception:
        return {}


def _save_ui_state(config_path: Path, state: dict) -> None:
    (config_path.parent / 'ui_state.json').write_text(json.dumps(state, indent=2))


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

async def discover_adapters(config_path: Path) -> dict:
    """
    Enumerate all CAN interfaces and their assignment status.

    Returns a dict ready to serialise as the GET /devices/can/adapters response.
    """
    rc, out, _ = await _run('ip', '-j', 'link', 'show', 'type', 'can')
    raw_ifaces: list[dict] = []
    if rc == 0 and out:
        try:
            raw_ifaces = json.loads(out)
        except json.JSONDecodeError:
            pass

    assignments = _load_assignments(config_path)  # {serial: limb}

    limb_slots: dict[str, str | None] = {limb: None for limb in LIMBS}
    for serial, limb in assignments.items():
        if limb in limb_slots:
            limb_slots[limb] = serial

    adapters = []
    for iface in raw_ifaces:
        name = iface.get('ifname', '')
        state = 'UP' if iface.get('operstate', '').upper() == 'UP' else 'DOWN'
        usb_info = await _get_usb_info(name)
        usb_serial = usb_info['usb_serial']
        # Renamed interfaces (can_left_leg, etc.) may not return serial via udevadm
        # because udev's property DB doesn't always update after `ip link set name`.
        # Recover by reverse-lookup: if the interface name matches a known assignment, use that serial.
        if usb_serial is None:
            for serial, limb in assignments.items():
                if name == f'can_{limb}':
                    usb_serial = serial
                    break
        assigned_limb = assignments.get(usb_serial) if usb_serial else None

        # Fallback: if the interface name already matches a known limb (e.g. can_left_leg),
        # treat it as assigned even when can_assignments is empty or the serial lookup failed.
        # This handles the case where the JSON entry was lost (e.g. overwritten by a config save)
        # but the udev rule is still in place and the interface was auto-renamed on plug-in.
        if assigned_limb is None:
            for limb in LIMBS:
                if name == f'can_{limb}':
                    assigned_limb = limb
                    # Re-sync: if we have the serial now, write it back so future lookups work
                    if usb_serial and usb_serial not in assignments:
                        assignments[usb_serial] = limb
                        _save_assignments(config_path, assignments)
                    break

        adapters.append({
            'current_name':  name,
            'state':         state,
            'usb_serial':    usb_serial,
            'usb_path':      usb_info['usb_path'],
            'assigned_limb': assigned_limb,
            'assigned_name': f'can_{assigned_limb}' if assigned_limb else None,
        })

    udev_exists = _UDEV_RULES_PATH.exists()
    ui_state = _load_ui_state(config_path)

    return {
        'adapters':          adapters,
        'unassigned_count':  sum(1 for a in adapters if a['assigned_limb'] is None),
        'limb_slots':        limb_slots,
        'joints_per_limb':   _joints_per_limb(config_path),
        'udev_rules_exist':  udev_exists,
        'udev_rules_content': _UDEV_RULES_PATH.read_text() if udev_exists else '',
        'banner_dismissed':  ui_state.get('banner_dismissed', False),
    }


def _build_udev_rule(usb_serial: str, limb: str) -> str:
    name = f'can_{limb}'
    # ATTRS{serial} is the sysfs attribute (from udevadm --attribute-walk);
    # ID_SERIAL_SHORT is a udev ENV property and does not work in ATTRS{} match.
    # Full path for ip is required — udev RUN has no PATH.
    # restart-ms 100 is tried first; drivers that don't support it (e.g. gs_usb/CANable)
    # fall back silently so the interface still comes up with correct bitrate and txqueuelen.
    return (
        f'SUBSYSTEM=="net", ACTION=="add", ATTRS{{serial}}=="{usb_serial}", '
        f'NAME="{name}", '
        f'RUN+="/bin/sh -c \''
        f'/usr/bin/ip link set {name} type can bitrate 1000000 restart-ms 100 2>/dev/null || '
        f'/usr/bin/ip link set {name} type can bitrate 1000000; '
        f'/usr/bin/ip link set {name} txqueuelen 1000; '
        f'/usr/bin/ip link set {name} up\'"'
    )


async def assign_adapter(
    current_name: str,
    usb_serial: str,
    limb: str,
    config_path: Path,
) -> dict:
    """
    Live-rename {current_name} → can_{limb}, bring it up, write a udev rule,
    and persist the serial→limb mapping in the robot config.

    Raises RuntimeError on any step failure.
    NOTE: the interface must not have an open socket (disconnect the robot first).
    """
    if not _SAFE_LIMB.match(limb):
        raise ValueError(f'Invalid limb name: {limb!r}')

    new_name = f'can_{limb}'

    # Step 1: take down
    await _run('ip', 'link', 'set', current_name, 'down', sudo=True)

    # Step 2: rename (skip if already has the correct name)
    if current_name != new_name:
        rc, _, err = await _run('ip', 'link', 'set', current_name, 'name', new_name, sudo=True)
        if rc != 0:
            raise RuntimeError(f'Rename {current_name!r} → {new_name!r} failed: {err}')

    # Step 3: set CAN parameters; try with restart-ms 100 first (not all drivers support it)
    rc, _, err = await _run(
        'ip', 'link', 'set', new_name, 'type', 'can', 'bitrate', '1000000',
        'restart-ms', '100', sudo=True
    )
    if rc != 0:
        rc, _, err = await _run(
            'ip', 'link', 'set', new_name, 'type', 'can', 'bitrate', '1000000', sudo=True
        )
        if rc != 0:
            raise RuntimeError(f'Set CAN parameters on {new_name!r} failed: {err}')

    # Step 4: bring up
    rc, _, err = await _run('ip', 'link', 'set', new_name, 'up', sudo=True)
    if rc != 0:
        raise RuntimeError(f'Bring up {new_name!r} failed: {err}')

    # Step 4b: increase TX queue depth (default 10 is too small for concurrent sockets)
    await _run('ip', 'link', 'set', new_name, 'txqueuelen', '1000', sudo=True)

    # Step 5: write udev rule
    existing = _UDEV_RULES_PATH.read_text() if _UDEV_RULES_PATH.exists() else ''
    kept_lines = [
        line for line in existing.splitlines()
        # Remove old entries for this serial (either attribute form) or this limb name
        if f'serial}}=="{usb_serial}"' not in line
        and f'ID_SERIAL_SHORT}}=="{usb_serial}"' not in line
        and f'NAME="can_{limb}"' not in line
        and line.strip()
    ]
    kept_lines.append(_build_udev_rule(usb_serial, limb))
    new_content = '\n'.join(kept_lines) + '\n'

    proc = await asyncio.create_subprocess_exec(
        'sudo', 'tee', str(_UDEV_RULES_PATH),
        stdin=asyncio.subprocess.PIPE,
        stdout=asyncio.subprocess.DEVNULL,
        stderr=asyncio.subprocess.PIPE,
    )
    try:
        _, tee_err = await asyncio.wait_for(proc.communicate(input=new_content.encode()), timeout=5.0)
        if proc.returncode != 0:
            _log.warning('udev rule write failed: %s', tee_err.decode())
    except asyncio.TimeoutError:
        _log.warning('udev rule write timed out')

    rc, _, err = await _run('/usr/bin/udevadm', 'control', '--reload-rules', sudo=True)
    if rc != 0:
        _log.warning('udevadm reload-rules failed (rc=%d): %s', rc, err)

    # Step 6: persist assignment in config
    assignments = _load_assignments(config_path)
    assignments = {s: l for s, l in assignments.items() if l != limb}  # clear old owner of this slot
    assignments[usb_serial] = limb
    _save_assignments(config_path, assignments)

    return {'new_name': new_name, 'state': 'UP'}


async def unassign_adapter(usb_serial: str, config_path: Path) -> dict:
    """Remove serial→limb mapping from the config (udev rule is kept)."""
    assignments = _load_assignments(config_path)
    if usb_serial not in assignments:
        raise ValueError(f'Serial {usb_serial!r} is not assigned')
    limb = assignments.pop(usb_serial)
    _save_assignments(config_path, assignments)
    return {'removed_limb': limb}


def dismiss_setup_banner(config_path: Path) -> None:
    ui_state = _load_ui_state(config_path)
    ui_state['banner_dismissed'] = True
    _save_ui_state(config_path, ui_state)
