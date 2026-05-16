# Flash Wizard

The Flash Wizard programs Recoil Motor Controller firmware onto B-G431B-ESC1 ESC boards. It compiles the firmware from source, patches the configuration header, flashes via ST-LINK, runs electrical calibration, and verifies the motor direction — all within the app.

---

## When to use the Flash Wizard

Use the Flash Wizard when:
- You are commissioning a **brand-new ESC board** that has never had firmware loaded
- A motor **fights commands** (grinding noise, high current, vibration without proper movement) indicating phase inversion
- You need to change the **CAN ID** of an ESC on a bus where SDO writes to the ID register are not surviving (e.g., the board has never had a config stored to Flash)

Do **not** use the Flash Wizard when:
- A motor is running in the wrong direction without grinding — this is sign inversion, fixable by negating `gear_ratio` in the Tune tab, no reflash needed
- You want to change PID gains, position limits, or other runtime parameters — all of these are writable via SDO without reflashing
- The motor has an ESC error — clear the error first and investigate before reflashing

---

## Prerequisites

### ARM toolchain

The Flash Wizard compiles the firmware from source. You need:

```bash
sudo apt-get install gcc-arm-none-eabi make openocd
```

Verify:

```bash
arm-none-eabi-gcc --version
openocd --version
```

### Firmware source

The wizard expects the Recoil Motor Controller firmware source tree at:

```
/home/nse/Recoil-Motor-Controller-BESC/Recoil-Motor-Controller-B-G431B-ESC1/
```

This path is hardcoded in `backend/api/routes_flash.py`. If your firmware is in a different location, update `_DEFAULT_FIRMWARE_DIR` in that file.

### ST-LINK connection

The B-G431B-ESC1 has an integrated STLINK-V3 debugger accessible via its USB-C connector. Connect the USB-C cable from the ESC to your computer. openocd will connect via the SWD interface automatically.

---

## How the wizard works

The Flash Wizard runs a 3-pass firmware procedure:

### Pass 1 — Initialize Flash option bytes

The STM32G431 must have its Flash option bytes configured before it can store a config struct to Flash page 63. Pass 1 programs these option bytes and writes default motor controller values to Flash.

After Pass 1, the firmware enters an infinite loop (halted state). **You must physically power-cycle the ESC** — disconnect and reconnect the motor power supply. Do not just reset via USB.

### Pass 2 — Write operational firmware with correct CAN ID and phase order

Pass 2 compiles the firmware with:
- The CAN ID you specified
- The phase order you specified (normal or inverted)
- All LOAD flags set to 0 (loads defaults, not Flash values)

After Pass 2, the firmware boots with the correct CAN ID and broadcasts TX_PDO4 frames. At this point you need to:
- Connect the motor mechanically (attach the output shaft to the joint)
- Connect the CAN bus to the appropriate bus adapter
- Click **Motor Connected** in the wizard

The wizard then:
1. Opens the CAN bus socket
2. Waits 2 seconds for the ESC boot traffic to settle (prevents ENOBUFS TX queue overflow)
3. Writes `fast_frame_frequency = 100` Hz via SDO
4. Sends NMT MODE_CALIBRATION to start flux-offset calibration
5. Polls until calibration completes (up to 90 seconds)
6. Asks you to verify the motor direction

### Direction verification loop

After calibration, the wizard asks: **"Did the motor rotate during calibration?"**

- If **yes** and the direction looked correct, click **Correct** — the wizard proceeds to Pass 3
- If the direction was **wrong** (motor spun backward from expected), click **Wrong** — the wizard automatically re-runs Pass 2 with the phase order toggled and repeats calibration

This loop repeats until you confirm the direction is correct.

### Pass 3 — Finalize operational firmware

Pass 3 compiles the firmware with all LOAD flags set to 1, meaning the firmware will load its CAN ID, config, and calibration data from Flash on every boot. This is the operational configuration.

After Pass 3, the ESC is fully commissioned and the wizard displays the measured `flux_offset`.

---

## Step by step walkthrough

1. Open the motor's tab (click it on the Dashboard)
2. In the motor panel, go to the **Calibration** tab
3. Click **Flash Wizard** at the bottom of the Calibration tab
4. In the Flash Wizard dialog:
   - Select the **CAN ID** for this motor (must match the bus wiring plan)
   - Select the **Motor Profile** (MAD_M6C12_150KV for hip roll, MAD_5010_200KV for most others)
   - Select the **CAN Channel** this motor belongs to
   - Enable **Invert Phase** only if you know from prior testing that this motor has phase inversion
5. Click **Start Flash**
6. Wait for Pass 1 to complete (~2 minutes for compile + flash)
7. When prompted, **power-cycle the ESC** (disconnect motor power, wait 2 seconds, reconnect)
8. Click **Power Cycled** in the wizard
9. Wait for Pass 2 to complete (~2 minutes)
10. Connect the motor to the CAN bus and mechanical linkage
11. Click **Motor Connected**
12. Wait for calibration (~15–30 seconds; up to 90 seconds)
13. Observe the motor rotating during calibration
14. Click **Correct** or **Wrong** based on what you saw
15. If wrong, wait for Pass 2 to re-run with inverted phase, then repeat steps 12–14
16. When direction is correct, wait for Pass 3 (~2 minutes)
17. The wizard shows `flash_wizard complete` and the measured `flux_offset`

---

## What "invert phase" does

The firmware has a `MOTOR_PHASE_ORDER` compile-time constant (+1 or −1). This define controls which two phases are swapped in the SVPWM output. With the wrong order, the FOC torque vector points opposite to the encoder velocity, causing the motor to fight itself.

At runtime, `MOTOR_PHASE_ORDER` maps to the `phase_order` field in the `MotorController` struct. This field can also be written via SDO (parameter 0x10C, signed int32). The Flash Wizard patches the compile-time define and then stores the value to Flash so it persists across power cycles.

The `phase_inverted` boolean in `humanoid_lite.json` controls what value the app writes to this parameter during `apply_config()`. For most joints, `phase_inverted = true` (phase_order = −1).

---

## If flashing fails

**Compilation error:** Check that `arm-none-eabi-gcc` and `make` are installed and on PATH. The build runs from the `Debug/` subdirectory of the firmware source.

**openocd cannot connect:** The STLINK-V3 may need a moment after USB plug-in. Try replugging the USB-C cable. Make sure no other program (STM32CubeIDE) has the ST-LINK interface open.

**Power cycle not recognized:** The wizard waits for you to click the button — it cannot detect the power cycle automatically. After clicking, it waits 2 seconds before proceeding. If the ESC did not actually boot (LED not lit), the CAN connection step will time out.

**Motor connected timeout:** The wizard waits up to 600 seconds (10 minutes) for you to click Motor Connected. This is a long window intentionally — it gives time to physically connect the CAN cable and motor wires.

**ENOBUFS after motor connected:** Historically, the first SDO write after CAN connect could fail with "No buffer space available" because the ESC's boot broadcast frames filled the TX queue. The backend now waits 2 seconds after `bus.connect()` before sending any SDO, and retries with backoff on ENOBUFS. If you still see this error, it may be a very slow machine — the 2-second delay may need to be extended in `flash.py`.
