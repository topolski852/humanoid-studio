"""Self-contained tests for hardstop range calibration math.

Runs without pytest:  python3 tests/test_range_cal.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from humanoid.range_cal import compute_range_calibration

KNEE_MIN, KNEE_MAX = 0.0, math.radians(140)   # 0 .. 2.4435 rad


def _displayed(pos_raw, r):
    """Reproduce firmware displayed position for a raw (gear=orig) capture:
    in the new gear frame the raw position negates iff flipped, then minus offset."""
    p = -pos_raw if r["flipped"] else pos_raw
    return p - r["position_offset"]


def test_forward_encoder_zero_offset():
    # lower stop at 0, upper at +2.44 (encoder forward, already aligned)
    r = compute_range_calibration(0.0, KNEE_MAX, KNEE_MIN, KNEE_MAX, gear_ratio=15.0)
    assert r["flipped"] is False
    assert r["gear_ratio"] == 15.0
    assert abs(r["position_offset"] - 0.0) < 1e-9
    assert r["range_ok"] is True
    assert abs(_displayed(0.0, r) - KNEE_MIN) < 1e-9
    assert abs(_displayed(KNEE_MAX, r) - KNEE_MAX) < 1e-9
    print("ok test_forward_encoder_zero_offset")


def test_forward_encoder_with_offset():
    # whole range shifted by +0.5 rad in raw frame
    lo, hi = 0.5, 0.5 + KNEE_MAX
    r = compute_range_calibration(lo, hi, KNEE_MIN, KNEE_MAX, gear_ratio=15.0)
    assert r["flipped"] is False
    assert abs(r["position_offset"] - 0.5) < 1e-9
    assert abs(_displayed(lo, r) - KNEE_MIN) < 1e-9
    assert abs(_displayed(hi, r) - KNEE_MAX) < 1e-9
    print("ok test_forward_encoder_with_offset")


def test_backwards_encoder_flips_gear():
    # upper hardstop reads LOWER than the lower one -> encoder backwards
    lo, hi = 0.5, 0.5 - KNEE_MAX            # hi < lo
    r = compute_range_calibration(lo, hi, KNEE_MIN, KNEE_MAX, gear_ratio=15.0)
    assert r["flipped"] is True
    assert r["gear_ratio"] == -15.0
    assert r["range_ok"] is True
    # after flip+offset, lower stop reads min, upper reads max
    assert abs(_displayed(lo, r) - KNEE_MIN) < 1e-9
    assert abs(_displayed(hi, r) - KNEE_MAX) < 1e-9
    print("ok test_backwards_encoder_flips_gear")


def test_range_mismatch_flagged():
    # measured range way smaller than expected -> range_ok False
    r = compute_range_calibration(0.0, 1.0, KNEE_MIN, KNEE_MAX, gear_ratio=15.0)
    assert r["range_ok"] is False
    assert r["range_error_rad"] > 0.35
    print("ok test_range_mismatch_flagged")


def test_bad_limits_raises():
    try:
        compute_range_calibration(0.0, 1.0, 1.0, 1.0, gear_ratio=15.0)
    except ValueError:
        print("ok test_bad_limits_raises")
        return
    raise AssertionError("expected ValueError for min_rad >= max_rad")


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for fn in fns:
        fn()
    print(f"\nAll {len(fns)} tests passed.")
