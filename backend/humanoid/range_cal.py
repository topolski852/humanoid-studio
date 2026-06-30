"""
Hardstop range calibration: from the two hardstop encoder positions, derive the
gear_ratio SIGN (direction / backwards-encoder detection), the position_offset
(zero reference), and the joint limits.

Procedure (see routes /motors/{j}/range_cal_start + /range_cal_apply):
  1. At the START of calibration, write position_offset = 0 to the ESC. The two
     hardstop captures are then RAW positions (position = encoder / gear_ratio,
     no offset), so the offset math below is direct — no old-offset bookkeeping,
     and a gear_ratio sign flip stays simple (position just negates with gear).
  2. The user moves the joint to each hardstop and records lower_pos / upper_pos.
  3. compute_range_calibration() maps lower_pos -> min_rad and upper_pos -> max_rad.

Direction: the lower hardstop must read LESS than the upper after calibration. If
the raw upper_pos < lower_pos, the encoder counts backward for this joint, so the
gear_ratio sign is flipped (which negates the measured position) to make the joint
increase toward max_rad. position_offset is then chosen so the lower hardstop reads
exactly min_rad.
"""
from __future__ import annotations


def compute_range_calibration(
    lower_pos: float,
    upper_pos: float,
    min_rad: float,
    max_rad: float,
    gear_ratio: float,
    *,
    max_range_error_rad: float = 0.35,   # ~20 deg, matches the Motor Cal page tolerance
) -> dict:
    """Compute gear_ratio sign, position_offset, and limits from two hardstops.

    lower_pos / upper_pos are RAW positions captured with position_offset = 0 on
    the ESC, in the joint's current gear_ratio frame. min_rad / max_rad are the
    joint's known angular limits (lower hardstop -> min_rad, upper -> max_rad).

    Returns a dict with the values to write to the ESC plus diagnostics:
      gear_ratio, position_offset, limits{min,max}, flipped,
      measured_range_rad, range_error_rad, range_ok.
    """
    if max_rad <= min_rad:
        raise ValueError(f"min_rad ({min_rad}) must be < max_rad ({max_rad})")

    # Backwards encoder: the upper hardstop reads lower than the lower one.
    flipped = upper_pos < lower_pos
    new_gear = -gear_ratio if flipped else gear_ratio

    # Position = encoder / gear_ratio, so flipping the gear sign negates position.
    pl = -lower_pos if flipped else lower_pos
    pu = -upper_pos if flipped else upper_pos
    # After the (possible) flip, pu >= pl.

    # Offset so the lower hardstop reads exactly min_rad:
    #   displayed = position - offset  ->  min_rad = pl - offset.
    offset = pl - min_rad

    measured_range = pu - pl              # >= 0, == |upper_pos - lower_pos|
    expected_range = max_rad - min_rad
    range_error = abs(measured_range - expected_range)

    return {
        "gear_ratio": new_gear,
        "position_offset": offset,
        "limits": {"min": min_rad, "max": max_rad},
        "flipped": flipped,
        "measured_range_rad": measured_range,
        "range_error_rad": range_error,
        "range_ok": range_error <= max_range_error_rad,
    }
