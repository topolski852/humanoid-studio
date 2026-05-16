"""
Debug / diagnostic endpoints.

GET /debug/decode_frame?arb_id=0x48C&data=E887 8DBD FFE8 09BF
  Decodes any raw CAN frame through the Recoil protocol layer.
  Returns node_id, func_code, frame type, and decoded payload.

GET /debug/can_stats
  Returns per-device SDO lock contention stats (how often concurrent
  SDO reads were serialised — non-zero means the race was being hit).
"""
from __future__ import annotations

import struct

from fastapi import APIRouter, Query
from fastapi.responses import JSONResponse

from humanoid import recoil_protocol as proto

router = APIRouter(tags=["debug"])


def _decode_f32(raw: bytes) -> float:
    (v,) = struct.unpack("<f", raw[:4])
    return v


def _decode_u32(raw: bytes) -> int:
    (v,) = struct.unpack("<L", raw[:4])
    return v


@router.get("/debug/decode_frame", response_model=None)
async def decode_frame(
    arb_id: str = Query(..., description="Hex arbitration ID, e.g. 0x48C or 0x58C"),
    data: str = Query("", description="Hex data bytes, spaces optional, e.g. 'E887 8DBD FFE8 09BF'"),
) -> dict | JSONResponse:
    """
    Decode a raw CAN frame using the Recoil protocol.

    Useful for verifying:
    - Broadcast frame (TX_PDO4) decoding: arb_id=0x48C, data=<8 bytes>
    - SDO response (TX_SDO)   decoding: arb_id=0x58C, data=<4 bytes>
    """
    try:
        arb_id_int = int(arb_id, 16)
    except ValueError:
        return JSONResponse({"error": f"Invalid arb_id: {arb_id!r}"}, status_code=400)

    try:
        data_bytes = bytes.fromhex(data.replace(" ", ""))
    except ValueError:
        return JSONResponse({"error": f"Invalid hex data: {data!r}"}, status_code=400)

    node_id, func_code = proto.decode_arb_id(arb_id_int)
    func_name = proto.FUNC_NAMES.get(func_code, f"UNKNOWN(0x{func_code:X})")

    result: dict = {
        "arb_id":     f"0x{arb_id_int:03X}",
        "node_id":    node_id,
        "func_code":  f"0x{func_code:X}",
        "func_name":  func_name,
        "data_hex":   data_bytes.hex(" ") if data_bytes else "",
        "data_len":   len(data_bytes),
        "decoded":    {},
    }

    # TX_PDO4 / TX_PDO2 — autonomous broadcast or position echo: [pos_f32, vel_f32]
    if func_code in proto.TELEMETRY_FUNC_IDS:
        tel = proto.decode_telemetry(data_bytes)
        if tel is not None:
            pos_rad, vel_rads = tel
            result["decoded"] = {
                "type":         "telemetry",
                "position_rad": round(pos_rad, 6),
                "position_deg": round(pos_rad * 180 / 3.141592653589793, 4),
                "velocity_rads": round(vel_rads, 6),
                "velocity_dps": round(vel_rads * 180 / 3.141592653589793, 4),
            }
        else:
            result["decoded"] = {"error": "data too short for telemetry (need 8 bytes)"}

    # TX_SDO — SDO response from ESC: 4 bytes of raw parameter value
    elif func_code == proto.Func.TX_SDO:
        if len(data_bytes) >= 4:
            raw4 = data_bytes[:4]
            f32_val, = struct.unpack("<f", raw4)
            u32_val, = struct.unpack("<L", raw4)
            i32_val, = struct.unpack("<l", raw4)
            result["decoded"] = {
                "type":    "sdo_response",
                "raw_hex": raw4.hex(" "),
                "as_f32":  f32_val,
                "as_u32":  u32_val,
                "as_i32":  i32_val,
                "note":    "4 raw value bytes — no param_id echo in firmware response",
            }
        else:
            result["decoded"] = {"error": "TX_SDO response should be 4 bytes"}

    # RX_SDO — SDO request from host: [cmd, param_id_lo, param_id_hi, sub, ...]
    elif func_code == proto.Func.RX_SDO:
        if len(data_bytes) >= 3:
            cmd = data_bytes[0]
            param_id = struct.unpack_from("<H", data_bytes, 1)[0]
            ccs = cmd >> 5
            direction = {1: "write", 2: "read"}.get(ccs, f"unknown(ccs={ccs})")
            result["decoded"] = {
                "type":      "sdo_request",
                "direction": direction,
                "param_id":  f"0x{param_id:03X}",
                "cmd_byte":  f"0x{cmd:02X}",
            }
            if ccs == 1 and len(data_bytes) >= 8:
                val_bytes = data_bytes[4:8]
                f32, = struct.unpack("<f", val_bytes)
                u32, = struct.unpack("<L", val_bytes)
                result["decoded"]["write_value_f32"] = f32
                result["decoded"]["write_value_u32"] = u32
        else:
            result["decoded"] = {"error": "data too short for SDO request"}

    # TX_PDO1 — ping echo
    elif func_code == proto.Func.TX_PDO1:
        result["decoded"] = {
            "type":       "ping_response",
            "magic_byte": f"0x{data_bytes[0]:02X}" if data_bytes else "empty",
            "ok":         len(data_bytes) >= 1 and data_bytes[0] == proto.PING_MAGIC,
        }

    return result
