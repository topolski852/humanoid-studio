"""
Recoil motor controller CAN protocol — ID layout and payload decode.

CAN ID bit layout (11-bit standard frame):
    bits [10:7]  function code  (4 bits, 0x0–0xE)
    bits  [6:0]  node ID        (7 bits, 1–127)

    arb_id = (func_code << 7) | node_id

Example: arb_id 0x48C
    node_id   = 0x48C & 0x7F = 12   (right_ankle_pitch_joint)
    func_code = 0x48C >> 7   =  9   (TX_PDO4 — autonomous broadcast)
"""
from __future__ import annotations

import struct

# ── ID layout ─────────────────────────────────────────────────────────────────

NODE_ID_MASK   = 0x7F    # bits 6:0  (7-bit node/device ID)
FUNC_ID_SHIFT  = 7       # bits 10:7 (4-bit function code)


def decode_arb_id(arb_id: int) -> tuple[int, int]:
    """Return (node_id, func_code) from a raw CAN arbitration ID."""
    return arb_id & NODE_ID_MASK, arb_id >> FUNC_ID_SHIFT


def encode_arb_id(node_id: int, func_code: int) -> int:
    """Build a CAN arbitration ID from node_id and function code."""
    return (func_code << FUNC_ID_SHIFT) | (node_id & NODE_ID_MASK)


# ── Function codes ────────────────────────────────────────────────────────────

class Func:
    NMT           = 0x0   # mode change: host → motor; data = <BB> mode, node_id
    SYNC_EMCY     = 0x1
    TIME          = 0x2
    TX_PDO1       = 0x3   # ping response: motor → host; data[0] echoes magic byte
    RX_PDO1       = 0x4   # ping request: host → motor; data[0] = PING_MAGIC (0xCA)
    TX_PDO2       = 0x5   # position echo: motor → host; 8B = <ff> pos_rad, vel_rads
    RX_PDO2       = 0x6   # position command: host → motor; 8B = <ff> pos_target, vel_ff
    TX_PDO3       = 0x7
    RX_PDO3       = 0x8
    TX_PDO4       = 0x9   # autonomous broadcast: motor → host; 8B = <ff> pos_rad, vel_rads
    RX_PDO4       = 0xA
    TX_SDO        = 0xB   # SDO response: motor → host; data[3:0] = parameter value
    RX_SDO        = 0xC   # SDO request: host → motor; read or write a parameter
    FLASH         = 0xD   # firmware flash operations
    HEARTBEAT     = 0xE   # watchdog feed: host → motor; no data required


FUNC_NAMES: dict[int, str] = {
    Func.NMT:       'NMT',
    Func.SYNC_EMCY: 'SYNC',
    Func.TIME:      'TIME',
    Func.TX_PDO1:   'TX_PDO1',
    Func.RX_PDO1:   'RX_PDO1',
    Func.TX_PDO2:   'TX_PDO2',
    Func.RX_PDO2:   'RX_PDO2',
    Func.TX_PDO3:   'TX_PDO3',
    Func.RX_PDO3:   'RX_PDO3',
    Func.TX_PDO4:   'TX_PDO4',
    Func.RX_PDO4:   'RX_PDO4',
    Func.TX_SDO:    'TX_SDO',
    Func.RX_SDO:    'RX_SDO',
    Func.FLASH:     'FLASH',
    Func.HEARTBEAT: 'HEARTBEAT',
}

# Function codes where the 8 payload bytes are two little-endian float32s:
#   bytes [0:4] = position_rad  (output shaft, after gear ratio)
#   bytes [4:8] = velocity_rads (output shaft, after gear ratio)
# Note: no current field — Iq is only available via SDO reads.
TELEMETRY_FUNC_IDS = frozenset({Func.TX_PDO2, Func.TX_PDO4})

PING_MAGIC = 0xCA  # echoed back in TX_PDO1 to confirm liveness


# ── Payload helpers ───────────────────────────────────────────────────────────

def decode_telemetry(data: bytes) -> tuple[float, float] | None:
    """
    Decode 8-byte PDO2 / PDO4 payload.

    Returns (position_rad, velocity_rads) or None if data is too short.
    Both values are output-shaft quantities (already multiplied by gear_ratio
    inside the firmware).
    """
    if len(data) < 8:
        return None
    try:
        pos, vel = struct.unpack_from('<ff', data)
        return pos, vel
    except struct.error:
        return None
