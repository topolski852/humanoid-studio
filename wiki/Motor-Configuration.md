# Motor Configuration

This page explains the three types of calibration, when each is needed, and how to perform each one using the Calibration tab in the motor panel.

---

## Overview: three types of calibration

### 1. Electrical offset calibration (flux offset)

The FOC (field-oriented control) algorithm needs to know the alignment between the encoder's zero position and the motor's magnetic zero. This value is called the `electrical_offset` (also called `flux_offset` in firmware) and is measured in radians.

**When to run:** After flashing new firmware to a board, or if a motor is replaced. The value is stored in firmware Flash and survives power cycles. You do not need to re-run this after normal reboots.

**What happens physically:** The firmware ramps up a controlled voltage, rotates the motor forward and backward through a full electrical cycle, and computes the alignment offset from the encoder readings. The motor must be able to rotate freely — remove any load or hold the joint in the air so the output shaft can spin without hitting a hardstop.

**Duration:** Up to 90 seconds.

### 2. Position limit calibration (hardstop procedure)

The position limit calibration sets two values:
- `position_offset` — the encoder's zero reference (in rad, applied in the firmware position controller)
- `position_limits.min` / `position_limits.max` — the software travel limits enforced by the firmware

**When to run:** After electrical calibration when commissioning a new joint, or whenever the robot's mechanical zero has changed (e.g., after disassembly and reassembly). This calibration can be repeated safely as many times as needed.

**What happens physically:** You move the joint manually to each mechanical hardstop and record the encoder readings. The app computes the correct offset and limits from those readings and writes them to the ESC.

**Duration:** 2–5 minutes of manual work.

### 3. Firmware flash

Flashing replaces the firmware program on the STM32 microcontroller. After flashing, all calibration data in Flash is erased and must be redone.

**When to run:** Only when a motor is fighting commands (phase inversion), or when commissioning a brand-new ESC board that has never had firmware loaded. Do not flash to fix sign inversion — that is handled by the gear ratio sign in the config. See the [Sign Inversion vs Phase Inversion](#sign-inversion-vs-phase-inversion) section.

---

## Step by step: commissioning a new motor

### Step 1: Connect the ESC

1. Connect the ESC to your computer via USB (for flashing) or power on the motor on the CAN bus (for calibration after flashing).
2. Verify the ESC appears in the CAN Monitor — it should show traffic on the appropriate bus.

### Step 2: Flash firmware (if needed)

If this is a fresh board, open the **Flash Wizard** tab. See [Flash Wizard](Flash-Wizard) for the full procedure. After flashing, proceed to Step 3.

### Step 3: Open the motor's Calibration tab

1. On the Dashboard, click the motor card for the joint you want to calibrate.
2. In the motor panel, click the **Calibration** tab.

### Step 4: Run electrical offset calibration

1. Hold the joint so the output shaft can rotate freely (suspend it in air or remove any linkage).
2. Click **Run Flux Calibration**.
3. Wait up to 90 seconds. The button shows a spinner. Do not power off the ESC.
4. On completion, the result shows the computed `flux_offset` in radians. A typical value is a large number like `-16.16` or `68.04` rad — this is normal; it represents cumulative electrical rotations.

The firmware saves the flux_offset to Flash automatically. The app reads it back and updates the config file.

### Step 5: Run position limit calibration

1. Click **Enable Idle** to put the motor in IDLE mode. The motor will be compliant (no torque) but still powered.
2. Enter the expected joint travel limits in the **Lower limit** and **Upper limit** fields (in degrees). These are the angles the joint should read at its lower and upper mechanical hardstops.
   - Example for hip roll: lower = −10°, upper = 90°
   - If you do not know the limits, leave the fields blank and use only the hardstop recording method.
3. Physically move the joint to its **lower mechanical hardstop**. Click **Record Lower**.
4. Physically move the joint to its **upper mechanical hardstop**. Click **Record Upper**.
5. The app shows the measured range in degrees. If limits were entered, it also compares to the expected range. A mismatch > 20° triggers a warning.
6. Click **Apply Calibration**.

The app writes the new `position_offset` and `position_limits` to the ESC RAM, saves them to `configs/humanoid_lite.json`, and stores to ESC Flash.

### Step 6: Verify

Enable the motor in Position mode. Jog it to 0° — it should hold at the mechanical reference position. Jog to the limit values and verify the motor stops at the hardstops.

---

## Sign Inversion vs Phase Inversion

This distinction is the most common point of confusion during commissioning. Getting it wrong causes unnecessary reflashing.

### Sign inversion (software fix — no reflash)

**What it looks like:**
- Moving the joint in what should be the positive direction causes the encoder reading to decrease
- Moving toward the lower limit (more negative degrees) causes the reading to increase
- The motor responds to commands but in the wrong direction

**Why it happens:**
- The motor is mounted in the opposite orientation from the convention expected by the firmware
- The gear arrangement reverses the output shaft direction

**How the app handles it:**
During position limit calibration, the app compares the recorded lower hardstop reading to the recorded upper hardstop reading. If `calUpper < calLower` (the "upper" hardstop reads a smaller number than the "lower" hardstop), the encoder is reading backward.

**Fix:** Negate the `gear_ratio` in the Tune tab. For example, change `−15.0` to `+15.0`. This inverts the position direction without touching the firmware. No reflash is needed.

**CAN Monitor shows:** The position values climb when you expected them to decrease, and vice versa.

### Phase inversion (firmware fix — reflash required)

**What it looks like:**
- The motor makes a grinding or screaming noise when a position command is sent
- The motor draws high current without moving correctly, or vibrates without rotating
- The motor "fights" any attempt to hold position, rather than tracking it
- This is a violent symptom — the motor will shake and potentially skip

**Why it happens:**
The three-phase wiring order (A, B, C phases) is reversed relative to what the firmware expects. The FOC controller computes torque vectors assuming a specific phase order. With the wrong order, the torque vector points in the wrong direction and the motor fights itself.

**Fix:** Reflash the firmware with the phase order inverted. In the Flash Wizard, enable the **Invert Phase** option. The wizard will also run the direction verification test after flashing.

**Distinct from sign inversion:** If you observe the wrong direction but no grinding noise and normal current draw, it is sign inversion (gear ratio fix). If you observe grinding, vibration, or high current draw, it is phase inversion (reflash fix). Do not reflash for a sign inversion issue — it is unnecessary and takes 10–15 minutes.

---

## Joint direction conventions

The table below shows the positive direction for each joint based on the Berkeley Humanoid Lite configuration. The `gear_ratio` sign in the config determines direction: a negative gear ratio inverts the encoder direction relative to the output shaft convention.

| Joint | Bus | CAN ID | Gear Ratio | Phase Inverted | Positive limit |
|---|---|---|---|---|---|
| left_hip_roll_joint | can_left_leg | 1 | +15.0 | No | 1.571 rad (90°) |
| left_hip_yaw_joint | can_left_leg | 3 | −15.0 | No | — (not set) |
| left_hip_pitch_joint | can_left_leg | 5 | −15.0 | No | — (not set) |
| left_knee_pitch_joint | can_left_leg | 7 | −15.0 | No | — (not set) |
| left_ankle_pitch_joint | can_left_leg | 11 | −15.0 | No | — (not set) |
| left_ankle_roll_joint | can_left_leg | 13 | −15.0 | Yes | — (not set) |
| right_hip_roll_joint | can_right_leg | 2 | −15.0 | Yes | — (not set) |
| right_hip_yaw_joint | can_right_leg | 4 | −15.0 | Yes | — (not set) |
| right_hip_pitch_joint | can_right_leg | 6 | −15.0 | Yes | — (not set) |
| right_knee_pitch_joint | can_right_leg | 8 | −15.0 | Yes | — (not set) |
| right_ankle_pitch_joint | can_right_leg | 12 | −15.0 | Yes | — (not set) |
| right_ankle_roll_joint | can_right_leg | 14 | −15.0 | No | — (not set) |
| left_shoulder_pitch_joint | can_left_arm | 1 | −15.0 | Yes | — (not set) |
| left_shoulder_roll_joint | can_left_arm | 3 | −15.0 | Yes | — (not set) |
| left_shoulder_yaw_joint | can_left_arm | 5 | −15.0 | Yes | — (not set) |
| left_elbow_pitch_joint | can_left_arm | 7 | −15.0 | Yes | — (not set) |
| left_wrist_yaw_joint | can_left_arm | 9 | −15.0 | Yes | — (not set) |
| right_shoulder_pitch_joint | can_right_arm | 2 | −15.0 | Yes | — (not set) |
| right_shoulder_roll_joint | can_right_arm | 4 | −15.0 | Yes | — (not set) |
| right_shoulder_yaw_joint | can_right_arm | 6 | −15.0 | Yes | — (not set) |
| right_elbow_pitch_joint | can_right_arm | 8 | −15.0 | No | — (not set) |
| right_wrist_yaw_joint | can_right_arm | 10 | −15.0 | Yes | — (not set) |

Position limits shown as `— (not set)` indicate that the `position_limits.min` and `position_limits.max` fields are `null` in the config, meaning no software travel limit is enforced by the firmware. Set limits by running the position limit calibration procedure.

The `left_hip_roll_joint` is the only joint with confirmed limits: min = −0.175 rad (−10°), max = 1.571 rad (90°). These were set during hardware testing.

---

## How the position offset is computed

The position calibration formula computes a new offset based on the authoritative live offset already stored in the ESC, not the potentially stale value shown in the UI:

```
new_offset = current_offset_from_ESC + (recorded_lower_hardstop - desired_min_limit)
```

This means running the calibration multiple times is safe. Each run reads the current offset from the ESC's live RAM and adjusts from that baseline. The old bug (where the frontend used a cached offset that could be stale, causing compounding errors on repeat calibration) was fixed in the backend.
