#!/usr/bin/env python3
"""Check firmware version on every ESC via SDO read.
Run from the repo root with the daemon running and all motors powered on.
"""
import json, socket, uuid, sys
from pathlib import Path

DAEMON_HOST         = "127.0.0.1"
DAEMON_PORT         = 9001
PARAM_FIRMWARE_VER  = 0x004
TARGET_VERSION      = 0x03000008  # v3.0.8

def sdo_read(channel: str, device_id: int, param_id: int, timeout: float = 1.5):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(timeout + 1.0)
    req = json.dumps({
        "type":       "GENERIC_SDO_READ",
        "id":         str(uuid.uuid4()),
        "channel":    channel,
        "device_id":  device_id,
        "param_id":   param_id,
        "timeout_ms": int(timeout * 1000),
    }).encode()
    try:
        sock.sendto(req, (DAEMON_HOST, DAEMON_PORT))
        data, _ = sock.recvfrom(4096)
        resp = json.loads(data)
        if resp.get("status") == "OK":
            return resp.get("value_u32")
        return None
    except Exception:
        return None
    finally:
        sock.close()

cfg_path = Path(__file__).parent.parent / "configs" / "humanoid_lite.json"
cfg = json.loads(cfg_path.read_text())

seen    = set()
results = []
for name, j in cfg["joints"].items():
    key = (j["can_channel"], j["can_id"])
    if key in seen:
        continue
    seen.add(key)
    val = sdo_read(j["can_channel"], j["can_id"], PARAM_FIRMWARE_VER)
    if val is not None:
        major = (val >> 24) & 0xFF
        minor = (val >> 16) & 0xFF
        patch = val & 0xFFFF
        ver   = f"v{major}.{minor}.{patch}"
        ok    = val == TARGET_VERSION
    else:
        ver, ok = "OFFLINE", False
    results.append((name, j["can_channel"], j["can_id"], ver, ok))

header = f"{'Joint':<35} {'Channel':<18} {'ID':>4}  {'Firmware':<12}  Status"
print(header)
print("-" * len(header))
needs_flash = []
for name, ch, did, ver, ok in results:
    status = "OK" if ok else ("OFFLINE" if ver == "OFFLINE" else f"NEEDS FLASH ({ver})")
    print(f"{name:<35} {ch:<18} {did:>4}  {ver:<12}  {status}")
    if not ok and ver != "OFFLINE":
        needs_flash.append((name, ch, did, ver))

if needs_flash:
    print(f"\n{len(needs_flash)} ESC(s) need reflashing to v3.0.8:")
    for name, ch, did, ver in needs_flash:
        print(f"  {name}  ({ch} ID {did})  currently {ver}")
    sys.exit(1)
elif all(ok for *_, ok in results):
    print("\nAll online ESCs are running v3.0.8.")
else:
    print("\nSome ESCs are offline — power on and retry.")
    sys.exit(2)
