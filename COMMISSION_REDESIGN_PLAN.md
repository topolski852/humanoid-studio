# Plan: decouple commutation from direction in commissioning

Status: **planned, approved approach (2026-06-30).** Reworks the Flash Wizard so
`phase_order` is chosen for **commutation only** (auto-checked), and joint
**direction** is set via `gear_ratio` sign in a new jog-based step that also
applies the official joint limits.

## Why

Today the wizard asks "did the motor move the right direction?" and, if no,
**toggles `phase_order` + recalibrates**. This conflates two independent things:

- `phase_order` = **commutation** (swaps PWM output *and* current sensing,
  powerstage.c:39-55/100-125). Firmware evidence (hip_roll, sessions 11–12):
  only one phase_order calibrates into clean closed-loop commutation; the wrong
  one buzzes/stalls. It is **not** a free direction knob.
- `gear_ratio` **sign** = **direction**. Firmware divides position/velocity by
  it and divides torque→i_q by it (motor_controller.c:389,390,397,414), so
  flipping the sign reverses the joint *and* keeps the loop stable — **no
  recal**. This is the correct direction mechanism.

Net: stop using phase for direction. Pick phase by commutation quality; set
direction with gear_ratio sign.

## Phase 1 — Backend: auto commutation-check (replaces the direction toggle)

In `flash.py` `_do_session`, replace the `AWAITING_CONFIRMATION` direction loop
(lines ~1426-1457) with an **automatic commutation check** after calibration:

1. Reduce torque limit (SDO) for safety; record current position.
2. Via the daemon control path (`dc.set_mode(joint, "POSITION")` +
   `dc.set_position(joint, pos±step)`), command a small guarded move
   (~0.12 rad), sampling position + i_q from `dc.get_all_states_raw()[joint]`
   for ~1.5 s. Runaway guard: disable if |pos-start| exceeds a band.
3. Classify (reuse motor_diagnose thresholds): `|motion| < ~0.05 rad` AND
   `max|i_q| > ~3 A` ⇒ **commutation fault**.
4. On fault: toggle `phase_order` (SDO) → `send_flash_store` → recalibrate →
   re-check. **Max 2 attempts** (only two phase values). If both fail ⇒
   `FlashError("commutation failed on both phase orders — check motor/encoder
   wiring")`.
5. On clean: restore torque limit, return joint to IDLE/DISABLED, proceed.

Notes: needs the commissioned joint to be in the robot config (have
`_commissioning_joint_name`); firmware v3.1.1+ seeds position_target on POSITION
entry so enable doesn't jump. `updated_config` keeps `phase_inverted` (the
commutation-correct value); no direction toggle. Remove
`AWAITING_CONFIRMATION` state + `confirm_direction()` + the frontend confirm UI,
or repurpose AWAITING_CONFIRMATION to a non-blocking "checking commutation…".

## Phase 2 — Backend: new "Direction & Limits" step

A post-commission calibration step keyed by joint (own endpoints, not in the
flash state machine — usable standalone for re-checks):

- `POST /motors/{joint}/jog_direction` — enable POSITION, jog +`step` from
  current pos, return measured motion sign. (Guarded; returns to start.)
- `POST /motors/{joint}/set_direction {reversed: bool}` — if `reversed`, flip
  `gear_ratio` sign in config → `apply_config` (RAM) → persist JSON → optional
  `store_to_flash`. No recalibration.
- Apply default limits via `apply_default_limits` (already exists) and persist.

## Phase 3 — Frontend: FlashWizard + new step UI

- Remove the "No, invert ↻ / Yes, correct ✓" direction prompt; the Commission
  step shows an automatic "Checking commutation…" → pass/fail (auto-retry).
- Add a **Direction & Limits** step after commission: a "Jog +" button, then
  "Correct ✓ / Reversed ↺" → calls set_direction; shows the joint's applied
  limits (from defaults), editable.
- `STEP_LABELS`: Configure · Flash · Connect · Commission · Calibrate ·
  **Direction & Limits** · Done.

## Verification (HIL, left leg)
1. Commission a joint that needs phase=true: the auto-check toggles from a
   wrong initial phase and lands on clean commutation without user input.
2. Jog step: a joint mounted "backwards" reports reversed motion → set_direction
   flips gear_ratio sign → re-jog confirms +direction, no recal.
3. Limits from defaults applied + persisted; daemon clamps to them.
4. Both-phases-fail path raises a clear error (e.g. disconnected encoder).

## Open risk
- Commutation-check current threshold needs HIL calibration vs gravity-loaded
  joints (a loaded joint draws current to hold). Mitigate by keying on
  motion-vs-current ratio (stuck + high current), not absolute current, and by
  using a small reduced-torque move — same approach as the Diagnose tool.
