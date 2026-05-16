# Humanoid Studio — Session Handoff Document

**Purpose:** This document is the complete context for a new Claude Code session picking up this
project. It was generated at the end of a session that (1) deeply analyzed the Recoil firmware and
Berkeley Humanoid Lite Python library, then (2) built the entire `humanoid-studio/` project from
scratch, then (3) fixed critical bugs and completed the motor-endpoint refactor.

**Project root:** `/home/nse/humanoid-studio/`
**Firmware source:** `/home/nse/Recoil-Motor-Controller-BESC/Recoil-Motor-Controller-B-G431B-ESC1/`
**Original Python lib:** `/home/nse/Berkeley-Humanoid-Lite/source/berkeley_humanoid_lite_lowlevel/`

---

## 1. Firmware Findings

### 1.1 motor_controller_conf.h — Every Configurable Parameter

File location: `Core/Inc/motor_controller_conf.h`

**Build-time defines (require recompile + reflash to change):**

| Define | Current value | Meaning |
|---|---|---|
| `FIRMWARE_VERSION` | `0x20250226` | Date-stamped version (YYYYmmdd) |
| `DEVICE_CAN_ID` | `14` | CAN node ID [1–63]. Must be unique per bus. |
| `FIRST_TIME_BOOTUP` | `0` | Set to `1` only when programming a virgin chip to program Flash option bytes. Halts in while(1) after programming — must reflash with 0. |
| `LOAD_ID_FROM_FLASH` | `1` | `1` = use CAN ID stored in Flash; `0` = use DEVICE_CAN_ID from source |
| `LOAD_CONFIG_FROM_FLASH` | `1` | `1` = load all PID/limits/motor params from Flash; `0` = use compile-time defaults |
| `LOAD_CALIBRATION_FROM_FLASH` | `1` | `1` = load encoder flux_offset from Flash; `0` = re-calibrate every boot |
| `SAFETY_WATCHDOG_ENABLED` | `1` | `1` = watchdog active; motor falls to DAMPING if no PDO within 1000 ms |
| `ENCODER_DIRECTION` | `+1` | Encoder count direction. Never changed in practice; encoder_cpr = direction * 4096 |
| `ENCODER_PRECISION_BITS` | `12` | AS5600 12-bit encoder → 4096 CPR |
| `MOTOR_PHASE_ORDER` | `+1` | **Phase/direction control.** `+1` = forward; `-1` = inverted. This is the ONLY way to invert motor direction at the firmware level. |
| `NOMINAL_BUS_VOLTAGE` | `12.0f` | Used as calibration voltage ceiling guard (volts). If `voltage_setpoint > NOMINAL_BUS_VOLTAGE + 10`, calibration aborts. |
| `COMMUTATION_FREQ` | `10000.0f` | FOC inner loop rate (Hz) = 160 MHz / TIM_AAR(4000) / TIM_REPETITION(4) |
| `POSITION_UPDATE_FREQ` | `2000.0f` | Position/velocity controller update rate (Hz) |
| `ENCODER_LUT_ENTRIES` | `128` | Number of entries in the flux offset lookup table |
| `ADC_RESOLUTION` | `4096` | 12-bit ADC |

**Motor profile selection (one `#define` must be uncommented):**

| Profile define | Pole pairs | Torque constant | Phase resistance | Phase inductance | Cal current |
|---|---|---|---|---|---|
| `MOTORPROFILE_MAD_M6C12_150KV` | 14 | 0.08958 Nm/A | 0.13793 Ω | 3.039e-5 H | 5 A |
| `MOTORPROFILE_MAD_5010_110KV` | 14 | 0.1176 Nm/A | 0.6193 Ω | 8.50e-5 H | 3 A |
| **`MOTORPROFILE_MAD_5010_200KV`** | **14** | **0.06588 Nm/A** | **0.15227 Ω** | **2.649e-5 H** | **3 A** |
| `MOTORPROFILE_MAD_5010_310KV` | 14 | (undefined) | 0.05735 Ω | 3.326e-5 H | 5 A |
| `MOTORPROFILE_MAD_5010_370KV` | 14 | (undefined) | 0.03000 Ω | 1.072e-5 H | 5 A |

**Currently active profile: `MOTORPROFILE_MAD_5010_200KV`**

---

### 1.2 Motor Phase/Direction Inversion

**Compile-time control (requires reflash):**
```c
// In Core/Inc/motor_controller_conf.h, line 78:
#define MOTOR_PHASE_ORDER  +1   // change to -1 to invert
```

This define becomes `motor.phase_order` (type `int8_t`) in the `Motor` struct. It is passed to
`PowerStage_setOutputPWM()` and `PowerStage_setOutputVoltage()` to swap phase A and phase C
wiring in the SVPWM output.

**Runtime control (via CAN SDO, survives until power cycle):**

Write a signed 32-bit integer to byte offset `0x10C` in the MotorController struct:
- `+1` (0x00000001) = forward
- `-1` (0xFFFFFFFF) = inverted

```python
# Python: write -1 to invert
import struct
raw = struct.pack('<i', -1)   # little-endian signed int32
await bus._sdo_write(device_id, 0x10C, raw)
```

**To make it persistent** you must call store-to-flash after writing (FUNC_FLASH function=0xD,
data byte 0 = `0x01`).

**Our `JointConfig` field:** `phase_inverted: bool`. The `phase_order` property converts:
`return -1 if self.phase_inverted else 1`. The `apply_config()` method writes this to
`Parameter.MOTOR_PHASE_ORDER` (0x10C) as a signed i32.

**Which joints have phase inverted in the robot:**
- `right_ankle_roll_joint` (can1, id=14): `phase_inverted=False` (phase_order=+1)
- `right_elbow_pitch_joint` (can3, id=8): `phase_inverted=False` (phase_order=+1)
- All other 20 joints: `phase_inverted=True` (phase_order=-1)

---

### 1.3 Initialization Sequence (motor_controller.c: MotorController_init)

Called from `APP_init()` in `app.c`. Executed once at boot from main().

```
Step 1:  controller->mode = MODE_DISABLED; controller->error = ERROR_NO_ERROR;
         Set watchdog_timeout=1000ms, fast_frame_frequency=0, device_id=DEVICE_CAN_ID

Step 2:  CAN_init(&hfdcan1, 0, 0)
         Initialize FDCAN1 peripheral. Filter ID=0 means accept all IDs.

Step 3:  Encoder_init(&controller->encoder, &hi2c1)
         Initialize AS5600 I2C encoder on I2C1. Sets CPR = ENCODER_DIRECTION * 4096.

Step 4:  PowerStage_init(&controller->powerstage, &htim1, &hadc1, &hadc2)
         Configure TIM1 for 3-phase PWM center-aligned, link ADC1+ADC2.

Step 5:  Motor_init(&controller->motor)
         Load compile-time motor profile values (pole_pairs, torque_constant, etc.)

Step 6:  CurrentController_init(&controller->current_controller)
         Initialize d/q PI controllers and set default i_limit, i_kp, i_ki.

Step 7:  PositionController_init(&controller->position_controller)
         Initialize position/velocity PI controllers with default gains.

Step 8:  MotorController_loadConfig(controller)
         Read MotorController struct from Flash at 0x0801F800 (Bank 1, Page 63).
         - If LOAD_CALIBRATION_FROM_FLASH=1: loads encoder.flux_offset. If NaN, returns HAL_ERROR.
         - If LOAD_ID_FROM_FLASH=1: loads device_id from Flash (overrides DEVICE_CAN_ID).
         - If LOAD_CONFIG_FROM_FLASH=1: loads all PID gains, limits, motor params.
           Validates each float — if NaN, returns HAL_ERROR (init fails).
         - firmware_version is ALWAYS set from the FIRMWARE_VERSION define (not from Flash).

Step 9:  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1)
         Start the LED PWM timer. LED brightness indicates mode.

Step 10: HAL_OPAMP_Start(&hopamp1/2/3)
         Start 3 OPAMPs that amplify phase current sense signals (gain x16).

Step 11: HAL_ADCEx_InjectedStart(&hadc1), HAL_ADCEx_InjectedStart(&hadc2)
         Start injected ADC conversions for phase current measurement.

Step 12: HAL_TIM_Base_Start_IT(&htim2)   → safety watchdog timer (1kHz tick, 1000ms timeout)
         HAL_TIM_Base_Start(&htim6)       → time keeper (monotonic reference)
         HAL_TIM_Base_Start_IT(&htim8)    → fast-frame timer (triggers TRANSMIT_PDO_4)
         Set htim2 ARR = watchdog_timeout * 10 - 1 (100Hz tick × timeout_ms = period)
         Set htim8 ARR = 10000/fast_frame_frequency - 1 (or 10000/100-1 if frequency=0)

Step 13: PowerStage_start(&controller->powerstage)
         Enable PWM outputs. Phase currents now active.

Step 14: If any step returned HAL_ERROR:
         Set ERROR_INITIALIZATION_ERROR, go to MODE_DISABLED, blink LED fast, loop UART prints.

Step 15: If any LOAD_*_FROM_FLASH flag = 0:
         MotorController_storeConfig(controller) — write defaults back to Flash.

Step 16: HAL_Delay(100)
         Wait for OPAMPs and ADC to settle.

Step 17: PowerStage_calibratePhaseCurrentOffset()
         Measure ADC zero-current offset (averages N readings with motor de-energized).

Step 18: MotorController_clearError(controller)
Step 19: MotorController_setMode(controller, MODE_IDLE)
         Motor is now live in IDLE. PWM enabled. Watchdog timer running.
```

**FIRST_TIME_BOOTUP path (only for virgin chip):**
```
APP_init() → APP_initFlashOption() → HAL_FLASH_Unlock() → HAL_FLASH_OB_Unlock()
→ FLASH->OPTR = 0xFBEFF8AA (boot from Flash, not DFU)
→ FLASH_CR_OPTSTRT → HAL_FLASH_Lock() → HAL_FLASH_OB_Launch() (triggers reset)
→ MotorController_storeConfig() → while(1)  ← halts; must reflash with FIRST_TIME_BOOTUP=0
```

**ISR routing (app.c HAL callbacks):**
- `htim1` → 10 kHz → `MotorController_update()` (full FOC loop)
- `htim2` → watchdog → if no watchdog reset within timeout, set DAMPING + ERROR_WATCHDOG_TIMEOUT
- `htim8` → fast-frame → if `fast_frame_frequency != 0`, transmit PDO4 with position+velocity
- `FDCAN_RxFifo0` → `MotorController_handleCANMessage()`
- `APP_main()` (main loop, HAL_Delay 50ms) → `MotorController_updateService()` → runs calibration FSM

---

### 1.4 CAN ID Assignment

**Compile-time:** `#define DEVICE_CAN_ID 14` in `motor_controller_conf.h`

**Flash persistence:**
- If `LOAD_ID_FROM_FLASH=1` (default), the device ID is read from Flash offset 0x000
  (`PARAM_DEVICE_ID`) at boot, overriding the compile-time value.
- To set a CAN ID over CAN: write a uint32 to offset 0x000, then store to Flash
  (FUNC_FLASH, byte 0 = 0x01).

**CAN frame address scheme:**
```
11-bit standard CAN ID = (func_id << 7) | device_id
device_id: bits [6:0]  (7 bits, range 1–63)
func_id:   bits [10:7] (4 bits, 16 function codes)
```

**Multiple devices on one bus are assigned IDs 1, 2, 3... within that bus. IDs are NOT globally
unique across buses — the same ID can appear on can0 and can1 simultaneously.**

---

### 1.5 Parameters: Runtime CAN vs Requires Reflash

**Can be changed at runtime over CAN (SDO write, survives until power-cycle; persistent only if
store-to-Flash is called afterward):**

All parameters accessible via `FUNC_RECEIVE_SDO` (func_id=0xC):
- `device_id` (0x000) — uint32
- `watchdog_timeout` (0x008) — uint32, milliseconds
- `fast_frame_frequency` (0x00C) — uint32, Hz
- `mode` (0x010) — uint32 (prefer NMT for mode changes)
- All position controller gains and limits (0x01C–0x070)
- All current controller gains (0x074–0x07C)
- All powerstage thresholds and filter (0x0F4–0x0FC)
- `motor.pole_pairs` (0x104) — uint32
- `motor.torque_constant` (0x108) — float32
- `motor.phase_order` (0x10C) — int32 signed (+1 or -1)
- `motor.max_calibration_current` (0x110) — float32
- `encoder.cpr` (0x120) — uint32
- `encoder.position_offset` (0x124) — float32
- `encoder.velocity_filter_alpha` (0x128) — float32
- `encoder.flux_offset` (0x13C) — float32

**Requires reflash (compile-time only):**
- `MOTOR_PHASE_ORDER` in source — this is copied into `motor.phase_order` at init only if
  `LOAD_CONFIG_FROM_FLASH=0`. When loading from Flash, the Flash-stored value is used.
  **In practice**: reflash IS required to set phase_order on a fresh board that has never
  stored a config, because the Flash contains uninitialized data (NaN). Our flash wizard
  patches the `#define`, recompiles, reflashes, then calibrates and stores to Flash.
- `MOTOR_PROFILE` selection (pole_pairs, torque_constant, etc.)
- `FIRMWARE_VERSION` (always overwritten from define at boot)
- `COMMUTATION_FREQ`, `POSITION_UPDATE_FREQ` (timer hardware config)
- `FIRST_TIME_BOOTUP` (only for virgin chips)
- `ENCODER_DIRECTION` (always use +1)
- `NOMINAL_BUS_VOLTAGE` (calibration safety limit)

**Flash storage:** Full `MotorController` struct written to STM32G431 Flash Bank 1, Page 63,
address `0x0801F800`. Page size 2KB. Written double-word (uint64) at a time.

---

### 1.6 STM32CubeProgrammer Flash Command

```bash
STM32_Programmer_CLI -c port=SWD -w Debug/Recoil-Motor-Controller-B-G431B-ESC1.elf -v -rst
```

- `-c port=SWD` — connect via ST-Link SWD (the B-G431B-ESC1 has an integrated STLINK-V3)
- `-w <elf>` — write the ELF file (programmer resolves load addresses from ELF segments)
- `-v` — verify after writing
- `-rst` — reset MCU after flashing so it boots into the new firmware

**Alternative (USB DFU mode — hold BOOT0 high at reset):**
```bash
STM32_Programmer_CLI -c port=USB1 -w Debug/Recoil-Motor-Controller-B-G431B-ESC1.elf -v -rst
```

**Build command (run from `Debug/` directory):**
```bash
cd /home/nse/Recoil-Motor-Controller-BESC/Recoil-Motor-Controller-B-G431B-ESC1/Debug
make -j4 all
```
Produces `Debug/Recoil-Motor-Controller-B-G431B-ESC1.elf`.

---

## 2. Python Library Findings

### 2.1 CAN Message Format

**11-bit standard CAN ID construction:**
```
bits [10:7] = func_id  (4 bits)
bits [6:0]  = device_id (7 bits)

CAN ID = (func_id << 7) | device_id
```

**Function codes (FrameFunction enum):**

| Value | Name | Direction | Use |
|---|---|---|---|
| 0x0 | NMT | host→node | Mode change command |
| 0x1 | SYNC_EMCY | host→node | Ping (echoes device_id back) |
| 0x2 | TIME | — | Unused |
| 0x3 | TRANSMIT_PDO_1 | node→host | Echo of PDO1 data |
| 0x4 | RECEIVE_PDO_1 | host→node | Send 8 bytes, receive echo, resets watchdog |
| 0x5 | TRANSMIT_PDO_2 | node→host | [position_measured f32, velocity_measured f32] |
| 0x6 | RECEIVE_PDO_2 | host→node | [position_target f32, velocity_ff f32], resets watchdog |
| 0x7 | TRANSMIT_PDO_3 | node→host | [position_measured f32, torque_measured f32] |
| 0x8 | RECEIVE_PDO_3 | host→node | [position_target f32, torque_target f32], resets watchdog |
| 0x9 | TRANSMIT_PDO_4 | node→host | Fast-frame: [position f32, velocity f32] from htim8 ISR |
| 0xA | RECEIVE_PDO_4 | host→node | No-op |
| 0xB | TRANSMIT_SDO | node→host | SDO read response: 4 bytes of value |
| 0xC | RECEIVE_SDO | host→node | SDO read/write request |
| 0xD | FLASH | host→node | Store (byte 0=1) or Load (byte 0=2) config Flash |
| 0xE | HEARTBEAT | host→node | Resets watchdog only |

**PDO2 — primary real-time frame (used for position control at 200Hz):**
```
TX (host → node):
  CAN ID: (0x6 << 7) | device_id  = 0x300 | device_id
  DLC: 8
  data[0:4]: float32 LE  position_target  (rad, output-side after gear ratio)
  data[4:8]: float32 LE  velocity_ff      (rad/s, feedforward)

RX (node → host):
  CAN ID: (0x5 << 7) | device_id  = 0x280 | device_id
  DLC: 8
  data[0:4]: float32 LE  position_measured (rad)
  data[4:8]: float32 LE  velocity_measured (rad/s)
```

**SDO Write (download, host → node):**
```
CAN ID: (0xC << 7) | device_id  = 0x600 | device_id
DLC: 8
data[0]:   0x20  (CCS=1 download, bits [7:5]=001, expedited=1, size=4, n=0)
data[1:3]: uint16 LE  parameter_id  (byte offset into MotorController struct)
data[3]:   0x00
data[4:8]: uint32 LE  value (interpreted as float or int depending on parameter)

No response frame is sent by the device.
```

**SDO Read (upload, host → node):**
```
Request:
  CAN ID: (0xC << 7) | device_id  = 0x600 | device_id
  DLC: 8
  data[0]:   0x40  (CCS=2 upload initiate, bits [7:5]=010)
  data[1:3]: uint16 LE  parameter_id
  data[3:7]: 0x00 0x00 0x00 0x00

Response (node → host):
  CAN ID: (0xB << 7) | device_id  = 0x580 | device_id
  DLC: 4
  data[0:4]: uint32 LE  value
```

**NMT (mode change):**
```
CAN ID: 0x000 (broadcast) or (0x0 << 7) | device_id
DLC: 2
data[0]: uint8  mode value (e.g., 0x13 = MODE_POSITION, 0x05 = MODE_CALIBRATION)
data[1]: uint8  addressed_node (device_id; if 0, broadcast to all)
```

**FLASH (store/load config):**
```
CAN ID: (0xD << 7) | device_id  = 0x680 | device_id
DLC: 1
data[0]: 0x01 = store MotorController struct to Flash
         0x02 = load MotorController struct from Flash
```

**HEARTBEAT (watchdog feed only):**
```
CAN ID: (0xE << 7) | device_id  = 0x700 | device_id
DLC: 0
(no data, resets htim2 counter)
```

**Data encoding:**
- All float parameters: IEEE 754 float32, **little-endian**
- All uint parameters: uint32, **little-endian**
- Phase order: int32 signed, **little-endian** (+1 = 0x00000001 LE)
- **There is NO Q8.8 or fixed16 encoding in the actual protocol.** The `fixed16.py` file
  exists in the original library but is never used anywhere. All values are raw float32 or int32.
- Position and velocity in PDO2 are **output-side radians** (already divided by gear_ratio by firmware)

---

### 2.2 Commands Sent by Original Scripts

**calibrate_electrical_offset.py:**
```python
bus = recoil.Bus(channel='can0', bitrate=1000000)
bus.set_mode(joint_id, recoil.Mode.CALIBRATION)
# NMT frame: CAN ID=device_id, data=[0x05, device_id]
time.sleep(20)  # wait for firmware to complete the ~15s calibration
bus.set_mode(joint_id, recoil.Mode.IDLE)
# Calibration result (flux_offset) is auto-stored to Flash by firmware
```

**move_actuator.py (200 Hz position loop):**
```python
bus.set_mode(device_id, recoil.Mode.POSITION)  # NMT [0x13, device_id]
while True:
    position_target = ...  # some trajectory
    bus.transmit_pdo_2(device_id, position_target=pos, velocity_target=0.0)
    # PDO2 RX: CAN 0x600|id, 8 bytes [float32 pos, float32 0.0]
    pos_meas, vel_meas = bus.receive_pdo_2(device_id)
    # PDO2 TX: CAN 0x500|id, 8 bytes [float32 pos_meas, float32 vel_meas]
    time.sleep(1/200)
# BUG: no explicit watchdog feed; PDO2 resets watchdog implicitly
```

**configure_parameter.py (hardcoded MAD 5010-200KV):**
```python
# All 9 SDO writes, then store-to-Flash:
bus.write_gear_ratio(id, -15.0)          # SDO write 0x01C: float32(-15.0)
bus.write_position_kp(id, 25.0)          # SDO write 0x020
bus.write_velocity_kp(id, 0.023)         # SDO write 0x028
bus.write_torque_limit(id, 4.0)          # SDO write 0x030
bus.write_velocity_limit(id, 20.0)       # SDO write 0x034
bus.write_current_limit(id, 20.0)        # SDO write 0x074
bus.write_motor_pole_pairs(id, 14)       # SDO write 0x104
bus.write_motor_torque_constant(id, 0.06588) # SDO write 0x108
bus.write_motor_phase_order(id, -1)      # SDO write 0x10C: int32(-1)
bus.write_motor_calibration_current(id, 3.0) # SDO write 0x110
# Then: store-to-flash (FUNC_FLASH, data=[0x01])
# BUG: gear_ratio is silently negated in write_gear_ratio (writes -(-15.0)=+15.0 to firmware)
# Actually corrected: looking at core.py source the -GEAR_RATIO was in configure_parameter.py
```

**write_configurations.py (robot-wide config write):**
```python
# BUG: Only operates on one hardcoded joint:
entry = (recoil.Bus(channel='can1', bitrate=1000000), 14, "right_ankle_roll_joint")
# Should iterate robot.joints like read_configurations.py does
# Also missing: watchdog_timeout write
# Has 100ms sleep between each of the ~15 SDO writes per joint
```

---

### 2.3 robot_configuration.json Schema (Original Format)

The original file at `Berkeley-Humanoid-Lite/source/berkeley_humanoid_lite_lowlevel/robot_configuration.json`
uses a **nested** structure (not the flat structure used in our new schema):

```json
{
  "left_hip_roll_joint": {
    "device_id": 1,
    "firmware_version": "0x20250226",
    "watchdog_timeout": 1000,
    "fast_frame_frequency": 0,
    "position_controller": {
      "gear_ratio": -15.0,
      "position_kp": 20.0,
      "position_ki": 0.0,
      "velocity_kp": 1.0,
      "velocity_ki": 0.0,
      "torque_limit": 2.0,
      "velocity_limit": 20.0,
      "position_limit_upper": Infinity,   ← non-standard JSON! (Python json.dump artifact)
      "position_limit_lower": -Infinity,
      "position_offset": 0.0,
      "torque_filter_alpha": 0.1454
    },
    "current_controller": {
      "i_limit": 20.0,
      "i_kp": 0.1664,
      "i_ki": 5746.5
    },
    "powerstage": {
      "undervoltage_threshold": 0.0,
      "overvoltage_threshold": 0.0,
      "bus_voltage_filter_alpha": 0.2696
    },
    "motor": {
      "pole_pairs": 14,
      "torque_constant": 0.0659,
      "phase_order": -1,      ← int, -1 or +1
      "max_calibration_current": 3.0
    },
    "encoder": {
      "cpr": 4096,
      "position_offset": 0.0,
      "velocity_filter_alpha": 0.7154,
      "flux_offset": -32.2362   ← large float, can be negative, units = radians (electrical)
    }
  }
}
```

**Known issues with original format:**
- `Infinity`/`-Infinity` are not valid JSON (Python's json.dump produces them; most parsers reject them)
- `device_id` is NOT unique globally (same ID on different buses: `left_shoulder_pitch` and
  `left_shoulder_roll` both had device_id=1, but they are on different buses)
- `phase_order` is int in original; we replaced with `phase_inverted: bool`

---

### 2.4 Code Quality Issues in Original Scripts

1. **`recoil/can.py` is a dead stub** that imports `DataFrame` from core.py and re-declares
   `CANFrame` identically. Never imported by any script. Can be deleted.

2. **`recoil/fixed16.py` is dead code.** Q8.8 class defined, never used anywhere in any script.
   The protocol does not use fixed-point encoding — everything is float32.

3. **`filter_device_id=0` bug in `Bus.receive_pdo_2()`:** The check `if filter_device_id:` is
   False when device_id=0, skipping all filtering. Fixed in new library with explicit
   `if self.device_id is not None`.

4. **Race condition in SDO read:** Original code transmits the request first, then registers
   the receive filter. Frame could arrive and be dropped before filter is registered. Fixed in
   new library by registering the `asyncio.Future` waiter BEFORE transmitting.

5. **`receive_pdo_2` missing func_id filter:** Original code only filters by device_id, not
   func_id. If any other PDO from the same device arrives first, it would be returned as the
   response. Fixed in new library.

6. **`write_configurations.py` only writes one joint:** Line 15 hardcodes
   `entry = (recoil.Bus(channel='can1', bitrate=1000000), 14, "right_ankle_roll_joint")`.
   The `for entry in robot.joints:` loop from `read_configurations.py` was not used.

7. **Non-standard JSON:** `json.dump()` without custom encoder produces `Infinity` for float('inf').
   Fixed by using `None` in our Pydantic model (serializes as JSON `null`).

8. **100ms sleep per SDO write:** `write_configurations.py` has `time.sleep(0.1)` between
   every SDO write. With ~15 writes per joint × 22 joints = 33 seconds. No sleep needed;
   SDO is synchronous request-response.

9. **Blocking synchronous I/O:** All original scripts use synchronous `Bus.recv()` which blocks
   the thread. No async, no concurrent joint updates possible.

10. **`move_actuator.py` no watchdog feed:** The loop relies on PDO2 implicitly resetting the
    watchdog. Fine in practice but the intent is not clear.

---

### 2.5 Rewritten Python Library Class Hierarchy

**`backend/humanoid/can_bus.py`:**
```python
class Function(IntEnum): NMT=0, SYNC_EMCY=1, ..., HEARTBEAT=0xE
class Mode(IntEnum): DISABLED=0x00, IDLE=0x01, DAMPING=0x02, CALIBRATION=0x05,
                     CURRENT=0x10, TORQUE=0x11, VELOCITY=0x12, POSITION=0x13, ...
class ErrorCode(IntEnum): NO_ERROR=0, GENERAL=1, ESTOP=2, ..., ENCODER_FAULT=0x2000
class Parameter(IntEnum): DEVICE_ID=0x000, ..., ENCODER_FLUX_OFFSET_TABLE=0x140
class CANBusError(Exception): pass

@dataclass
class CANFrame:
    device_id: int; func_id: int; data: bytes

class CANBus:
    async def connect(self) -> None        # opens SocketCAN, starts _receive_loop task
    async def disconnect(self) -> None
    async def __aenter__/aexit__
    async def transmit(frame: CANFrame)    # asyncio.Lock protected
    async def receive(filter_device_id, filter_func, timeout) -> CANFrame | None
    async def _sdo_read(device_id, param_id, timeout) -> bytes | None
    async def _sdo_write(device_id, param_id, raw: bytes) -> None
    async def read_parameter_f32/i32/u32(device_id, param) -> float/int | None
    async def write_parameter_f32/i32/u32(device_id, param, value) -> None
    async def ping(device_id, timeout) -> bool
    async def feed_watchdog(device_id) -> None
    async def set_mode(device_id, mode: Mode) -> None
    async def store_to_flash(device_id) -> None
    async def load_from_flash(device_id) -> None
    async def send_pdo2(device_id, position, velocity_ff) -> None
    async def recv_pdo2(device_id, timeout) -> tuple[float,float] | None
    async def send_recv_pdo2(device_id, position, velocity_ff, timeout) -> tuple | None
```

**`backend/humanoid/actuator.py`:**
```python
class ActuatorError(Exception): pass
class ActuatorTimeoutError(ActuatorError): pass
class ActuatorCalibrationError(ActuatorError): pass

class ActuatorState(BaseModel):
    position: float; velocity: float; torque: float
    mode: int; mode_name: str; error: int; bus_voltage: float; timestamp: float

class Actuator:
    def __init__(bus: CANBus, config: JointConfig)
    async def ping(timeout=0.1) -> bool
    async def connect(timeout=1.0) -> bool          # 3 retries
    async def enable(mode=Mode.POSITION) -> None
    async def disable() -> None                     # → IDLE
    async def damp() -> None                        # → DAMPING
    async def feed_watchdog() -> None
    async def read_mode() -> Mode
    async def read_error() -> int
    async def read_firmware_version() -> str
    async def read_bus_voltage() -> float | None
    async def read_flux_offset() -> float | None
    async def get_state() -> ActuatorState          # 6 SDO reads, for polling
    async def set_position(pos, velocity_ff=0, torque_ff=0, timeout=0.005) -> tuple|None
    async def set_velocity(velocity) -> None
    async def set_torque(torque) -> None
    async def set_current(i_q, i_d=0.0) -> None
    async def calibrate_offset(timeout=90.0, on_progress=None) -> float
    async def apply_config() -> None                # 22 f32 + 4 u32 + 1 i32 SDO writes
    async def store_to_flash() -> None
    async def load_from_flash() -> None
```

**`backend/humanoid/robot_config.py`:**
```python
class PositionLimits(BaseModel):
    min: float | None = None    # None → lower_bound=-inf
    max: float | None = None    # None → upper_bound=+inf
    @property lower_bound / upper_bound -> float

class JointConfig(BaseModel):
    # Identity
    joint_name: str; joint_type: str; can_channel: str; can_id: int [1,63]
    # Direction
    phase_inverted: bool; electrical_offset: float; gear_ratio: float
    position_limits: PositionLimits
    # PID gains
    position_kp, position_ki, velocity_kp, velocity_ki: float
    torque_limit, velocity_limit, position_offset, torque_filter_alpha: float
    # Current controller
    current_limit, current_kp, current_ki: float
    # Powerstage
    undervoltage_threshold, overvoltage_threshold, bus_voltage_filter_alpha: float
    # Motor
    pole_pairs: int; torque_constant, max_calibration_current: float
    # Encoder
    cpr: int; encoder_position_offset, velocity_filter_alpha: float
    # System
    watchdog_timeout: int; fast_frame_frequency: int
    @property phase_order -> int   # -1 if phase_inverted else +1

class RobotConfig(BaseModel):
    robot_name: str; joints: dict[str, JointConfig]
    def channels() -> set[str]
    def to_dict/to_json/from_json()
```

**`backend/humanoid/robot.py`:**
```python
class Robot:
    def __init__(config: RobotConfig)
    async def connect/disconnect()
    async def __aenter__/aexit__
    def get_actuator(joint_name) -> Actuator
    def get_actuator_by_name(joint_name) -> Actuator | None  # primary lookup
    def get_actuator_by_id(can_id) -> Actuator | None        # legacy; first-match only
    def joint_names() -> list[str]
    def is_connected() -> bool
    async def ping_all(timeout) -> dict[str, bool]
    async def connect_all(timeout) -> dict[str, bool]
    async def enable_all(mode) / disable_all() / damp_all()
    async def get_all_states() -> dict[str, ActuatorState | None]  # fault-tolerant
    async def feed_all_watchdogs()
    async def apply_all_configs() / store_all_to_flash()
```

**`backend/humanoid/flash.py`:**
```python
class FlashError(Exception): pass
class FlashState(str, Enum):
    IDLE, CONNECTING, FLASHING, CALIBRATING, TESTING,
    AWAITING_CONFIRMATION, REFLASHING, COMPLETE, FAILED

class FlashConfig(BaseModel):
    firmware_dir: Path; can_channel: str; can_id: int; invert_phase: bool

@dataclass
class FlashStatus:
    state: FlashState; progress: int; messages: list[str]
    error: str | None; flux_offset: float | None

class FlashManager:
    async def start(port: str, config: FlashConfig) -> None
    async def confirm_direction(correct: bool) -> None
    # Internal: _flash_calibrate_confirm_loop() with while True for REFLASHING
```

---

## 3. What Was Built in This Session

### 3.1 Backend Python Files

| File | Purpose |
|---|---|
| `backend/humanoid/robot_config.py` | Pydantic models: `PositionLimits`, `JointConfig`, `RobotConfig`. Flat schema, JSON serialization with None=±inf. |
| `backend/humanoid/can_bus.py` | Async CAN transport. All protocol enums, async receive loop with ThreadPoolExecutor, race-free SDO read (waiter registered before transmit), asyncio.Lock for transmit. |
| `backend/humanoid/actuator.py` | Single-joint controller. All public methods listed above. `calibrate_offset()` polls mode until IDLE, checks error bits. `apply_config()` writes 22+4+1 SDO params with no delays. |
| `backend/humanoid/robot.py` | Multi-joint coordinator. Opens one `CANBus` per unique channel. `get_actuator_by_name()` is the primary lookup (unique). `get_actuator_by_id()` is legacy. `get_all_states()` is fault-tolerant (per-actuator try/except). |
| `backend/humanoid/flash.py` | Flash wizard state machine. Patches `motor_controller_conf.h` regex, runs `make -j4`, runs `STM32_Programmer_CLI`, then calibrates and tests. Loop for REFLASHING if direction is wrong. |
| `backend/humanoid/__init__.py` | Package exports for all public symbols. |
| `backend/api/__init__.py` | Imports all four route modules. |
| `backend/api/routes_devices.py` | `GET /devices` — scans `/sys/class/net/` for interfaces with `DEVTYPE=can` in uevent. |
| `backend/api/routes_motors.py` | `GET/POST /motors/{joint_name}` — enable, disable, calibrate, set_position. Uses `joint_name` (string) as path param, resolves via `robot.get_actuator_by_name()`. All return `{success, data, error}`. |
| `backend/api/routes_flash.py` | `POST /flash/start`, `GET /flash/status`, `POST /flash/confirm_direction`. Reads `firmware_dir` from request or defaults to `../../Recoil-Motor-Controller-B-G431B-ESC1`. |
| `backend/api/routes_robot.py` | `GET/PUT /robot/config`, `POST /robot/connect`, `POST /robot/disconnect`. PUT validates, persists to disk, creates new Robot instance. connect/disconnect open/close CAN buses. |
| `backend/main.py` | FastAPI on `localhost:8765`. Lifespan loads config from `configs/humanoid_lite.json`. Creates `Robot(config)` but does NOT auto-connect (call `POST /robot/connect` after startup). `/ws/telemetry` WebSocket at 50Hz. |
| `backend/requirements.txt` | `fastapi`, `uvicorn[standard]`, `python-can`, `pydantic>=2.7`, `websockets` |

### 3.2 Configuration

| File | Purpose |
|---|---|
| `configs/humanoid_lite.json` | 22-joint flat-schema config. can0=left leg (IDs 1,3,5,7,11,13), can1=right leg (IDs 2,4,6,8,12,14), can2=left arm (IDs 1,3,5,7,9), can3=right arm (IDs 2,4,6,8,10). Calibrated electrical_offset values from original robot_configuration.json. gear_ratio=-15.0 for all. |

### 3.3 Electron + React App

| File | Purpose |
|---|---|
| `app/package.json` | Vite + Electron 30 + React 18 + Tailwind + React Router. `npm run dev` starts both. |
| `app/vite.config.js` | `base: './'` for Electron file:// compatibility. Port 5173. |
| `app/tailwind.config.js` | Custom tokens: `surface` (#0f1117), `accent` (#3b82f6), `online` (#22c55e). JetBrains Mono for data. |
| `app/postcss.config.cjs` | CJS format (not ESM) so it works without `"type":"module"` in package.json. |
| `app/index.html` | Vite entry. CSP allows localhost:8765. Loads JetBrains Mono + Inter from Google Fonts. |
| `app/electron/main.js` | Spawns `python3 main.py` from backend/. Polls `/devices` every 500ms (40 attempts = 20s). Shows Restart/Quit dialog on crash. SIGTERM on quit, SIGKILL after 3s. |
| `app/electron/preload.js` | Exposes `window.electron.platform`. No IPC needed; app talks directly to localhost:8765. |
| `app/src/main.jsx` | React 18 `createRoot`. |
| `app/src/index.css` | Tailwind directives + scrollbar styling + component classes (`card`, `btn`, `data-value`, `data-label`). |
| `app/src/App.jsx` | HashRouter (required for file:// in production). Tab state management. `openMotorTab(canId, jointName)` uses joint_name as the URL key and tab ID. Route is `/motor/:jointName`. |
| `app/src/api.js` | Fetch wrapper. Motor methods use `encodeURIComponent(jointName)` in URL. All calls throw on `success=false`. |
| `app/src/context/TelemetryContext.jsx` | WebSocket to `/ws/telemetry`. Auto-reconnects every 2s. Provides `{states, robotConnected, wsConnected}` via context. |
| `app/src/components/Sidebar.jsx` | 240px dark sidebar. Logo + connection status dot. **Connect/Disconnect button** that calls `POST /robot/connect` or `POST /robot/disconnect`. Device list from `GET /robot/config` with live green/grey dots from WS. |
| `app/src/components/TabBar.jsx` | Tab strip. Active tab has blue bottom border. Closeable motor tabs (Dashboard is not closeable). |
| `app/src/components/StatusDot.jsx` | Green pulsing dot (online) or grey dot (offline). |
| `app/src/components/MotorCard.jsx` | Dashboard grid card. Shows CAN channel, ID, joint name, position, mode. Blue hover accent bar. |
| `app/src/components/MotorTab.jsx` | Route `/motor/:jointName`. `useParams().jointName` → decodes → looks up `rc.joints[decodedName]` directly (not by can_id). Passes `jointName` to ControlPanel and `canId`+`canChannel` to FlashWizard. |
| `app/src/components/TelemetryTable.jsx` | Left column: position (large), velocity, torque, voltage, mode, error. All monospace. |
| `app/src/components/ControlPanel.jsx` | Accepts `jointName` prop (not `canId`). All API calls use `jointName`. Enable/disable, position jog (±0.1 buttons + slider + 5 presets), calibration button, Flash Wizard button. |
| `app/src/components/FlashWizard.jsx` | Modal. Takes `{ canId, canChannel, jointName, onClose }`. Passes both `canId` and `canChannel` to `api.flashStart()`. Step strip, log pane, direction confirm. |
| `app/src/pages/Dashboard.jsx` | Responsive grid 2–4 cols. Empty state when no config. Online/offline summary counts. |
| `app/src/pages/RobotConfig.jsx` | Table view (expandable rows per joint, two-column field layout) + JSON editor with save. |
| `app/src/pages/Settings.jsx` | Connection status display. WS/API URL display. No editable settings yet. |

### 3.4 Architecture Decisions Made

- **`joint_name` as primary URL and API key:** Motor REST endpoints use `/motors/{joint_name}`
  (string), not `/motors/{can_id}` (int). This is the only globally unique identifier for a
  joint since can_ids repeat across buses (can0 id=1 = left_hip_roll, can2 id=1 = left_shoulder_pitch).
  Frontend routes are `/motor/:jointName`, tab IDs are `motor-${jointName}`.

- **`robot.connect()` is explicit, not automatic:** Backend startup creates `Robot(config)` but
  does not open CAN buses. The Sidebar's Connect button calls `POST /robot/connect` to open
  them. This avoids errors on machines without CAN hardware.

- **HashRouter for Electron:** `file://` URLs don't support pushState. HashRouter makes routes
  like `file://...#/motor/left_hip_roll_joint` work in both dev and prod.

- **No `"type":"module"` in package.json:** Would break Electron's `main.js` and `preload.js`
  which use CommonJS `require()`. PostCSS config uses `.cjs` extension to avoid the ESM warning.

---

## 4. What the New Session Needs to Do Next

### Priority 1 — Testing the backend (no hardware needed)

**Install Python dependencies:**
```bash
cd /home/nse/humanoid-studio/backend
pip install fastapi "uvicorn[standard]" python-can "pydantic>=2.7.0" websockets
```

**Run backend:**
```bash
cd /home/nse/humanoid-studio/backend
python3 main.py
```
Expected: Uvicorn starts on http://localhost:8765, logs "Loaded robot config: humanoid_lite (22 joints)".

**Test API (no CAN hardware needed):**
```bash
curl http://localhost:8765/devices
curl http://localhost:8765/robot/config | python3 -m json.tool | head -50
curl http://localhost:8765/flash/status
# Connect robot (will fail without CAN hardware, that's expected):
curl -X POST http://localhost:8765/robot/connect
# Motor endpoint now uses joint_name:
curl http://localhost:8765/motors/left_hip_roll_joint
```

**Test WS telemetry:**
```bash
python3 -c "import asyncio, websockets, json
async def t():
    async with websockets.connect('ws://localhost:8765/ws/telemetry') as ws:
        for _ in range(3):
            print(json.loads(await ws.recv())['connected'])
asyncio.run(t())"
```

### Priority 2 — Testing the Electron app

**Node.js PATH** must include nvm node (not in system PATH):
```bash
export PATH="$HOME/.nvm/versions/node/v20.20.2/bin:$PATH"
```

**Dev mode (requires backend running separately in another terminal):**
```bash
cd /home/nse/humanoid-studio/app
npm run dev
```
This starts Vite on port 5173 and Electron in dev mode. Electron will try to spawn the backend
(and fail if it's already running on 8765, or succeed if not). In dev, may want to comment out
`startBackend()` call and run backend manually.

**Production build (creates AppImage):**
```bash
cd /home/nse/humanoid-studio/app
npm run build
# Output: app/release/Humanoid Studio-0.1.0.AppImage
```

### Priority 3 — Hardware testing (requires B-G431B-ESC1 board)

**CAN setup:**
```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

**Verify a single motor:**
```python
import asyncio
import sys; sys.path.insert(0, '/home/nse/humanoid-studio/backend')
from humanoid.can_bus import CANBus

async def test():
    async with CANBus(channel='can0') as bus:
        ok = await bus.ping(1, timeout=0.5)
        print('ping:', ok)
asyncio.run(test())
```

**Connect via app then use the Sidebar Connect button:**
After the backend starts, click "Connect" in the Sidebar (next to the status dot). This calls
`POST /robot/connect` which opens CAN buses for all channels in the config. The telemetry WS
will start showing joint states as green dots.

**Flash a fresh board:**
```bash
curl -X POST http://localhost:8765/flash/start \
  -H 'Content-Type: application/json' \
  -d '{"can_id": 1, "invert_phase": false, "port": "SWD", "can_channel": "can0"}'
```
Then poll `GET /flash/status` and confirm direction with `POST /flash/confirm_direction {"correct": true}`.

### Priority 4 — Known open questions

1. **`electrical_offset` vs `encoder.flux_offset`:** In `configs/humanoid_lite.json`, the
   `electrical_offset` field is populated with the values from the original
   `robot_configuration.json`. These are pre-calibrated flux offsets. Confirm these are still
   valid before using them (re-calibrate if motors were disturbed).

2. **`position_limits` in config:** All joints have `{"min": null, "max": null}` (unlimited).
   The firmware uses `position_limit_lower` and `position_limit_upper`. When `null`, the
   Python code sends `float('-inf')` and `float('inf')` as IEEE 754 representations. **Verify
   that the firmware accepts ±inf as the "no limit" sentinel.** If not, use a large value like
   ±6.28.

3. **`position_kp: 20.0` for all joints:** The original `configure_parameter.py` used 25.0.
   The `read_configurations.py` output showed 20.0 (already stored in Flash). Confirm which
   is current.

4. **`gear_ratio: -15.0` convention:** The original library writes `gear_ratio = -GEAR_RATIO`
   where `GEAR_RATIO = 15`. So the negative sign is intentional — it flips position direction
   in firmware (output_position = encoder_position / gear_ratio). Combined with phase_inverted,
   both direction mechanisms are in play. Make sure you understand which one controls what for
   each joint before changing either.

5. **STM32_Programmer_CLI installation:** The flash wizard calls `STM32_Programmer_CLI`. Confirm
   it is installed and on PATH. Typically installed with STM32CubeProgrammer at
   `/opt/st/stm32cubeprog/bin/STM32_Programmer_CLI`.

6. **Arm joints (can2/can3) not physically connected:** The current running robot only uses
   can0/can1 (legs). The arm joints exist in our config but the robot doesn't have the hardware
   connected. Keep them in config for future use but don't try to connect them.

7. **Settings page has no editable fields yet.** It shows status only. Useful additions: CAN
   interface selection, API URL configuration, connect/disconnect trigger (currently only in Sidebar).

### Dependencies to Install

**Python (backend):**
```bash
pip install fastapi "uvicorn[standard]" python-can "pydantic>=2.7.0" websockets
```

**Node.js (already installed via nvm at `~/.nvm/versions/node/v20.20.2`):**
```bash
export PATH="$HOME/.nvm/versions/node/v20.20.2/bin:$PATH"
cd /home/nse/humanoid-studio/app
npm install  # (already done; node_modules exists)
```

**System (for CAN hardware):**
```bash
sudo apt-get install can-utils  # provides ip link, candump, cansend
```

### Hardcoded Values to Confirm

| Location | Value | What it controls |
|---|---|---|
| `backend/main.py:26` | `_CONFIG_PATH` points to `../../configs/humanoid_lite.json` | Auto-loaded config on startup |
| `backend/main.py:27` | `_TELEMETRY_HZ = 50` | WebSocket broadcast rate |
| `backend/api/routes_flash.py:16` | `_DEFAULT_FIRMWARE_DIR` points 3 levels up to `Recoil-Motor-Controller-B-G431B-ESC1` | Flash wizard firmware source path |
| `app/electron/main.js:8` | `BACKEND_PORT = 8765` | Port backend listens on |
| `app/electron/main.js:9,10` | `BACKEND_DIR` = `../../backend` (relative to `app/electron/`) | Where Electron spawns Python from |
| `app/src/api.js:1` | `BASE = 'http://localhost:8765'` | API base URL |
| `app/src/context/TelemetryContext.jsx:3` | `WS_URL = 'ws://localhost:8765/ws/telemetry'` | WebSocket URL |
| `configs/humanoid_lite.json` | All CAN channels `can0`–`can3` | Must match actual interface names |

---

## 5. Full Project Structure

```
/home/nse/humanoid-studio/app/electron/main.js
/home/nse/humanoid-studio/app/electron/preload.js
/home/nse/humanoid-studio/app/index.html
/home/nse/humanoid-studio/app/package.json
/home/nse/humanoid-studio/app/package-lock.json
/home/nse/humanoid-studio/app/postcss.config.cjs
/home/nse/humanoid-studio/app/src/api.js
/home/nse/humanoid-studio/app/src/App.jsx
/home/nse/humanoid-studio/app/src/components/ControlPanel.jsx
/home/nse/humanoid-studio/app/src/components/FlashWizard.jsx
/home/nse/humanoid-studio/app/src/components/MotorCard.jsx
/home/nse/humanoid-studio/app/src/components/MotorTab.jsx
/home/nse/humanoid-studio/app/src/components/Sidebar.jsx
/home/nse/humanoid-studio/app/src/components/StatusDot.jsx
/home/nse/humanoid-studio/app/src/components/TabBar.jsx
/home/nse/humanoid-studio/app/src/components/TelemetryTable.jsx
/home/nse/humanoid-studio/app/src/context/TelemetryContext.jsx
/home/nse/humanoid-studio/app/src/index.css
/home/nse/humanoid-studio/app/src/main.jsx
/home/nse/humanoid-studio/app/src/pages/Dashboard.jsx
/home/nse/humanoid-studio/app/src/pages/RobotConfig.jsx
/home/nse/humanoid-studio/app/src/pages/Settings.jsx
/home/nse/humanoid-studio/app/tailwind.config.js
/home/nse/humanoid-studio/app/vite.config.js
/home/nse/humanoid-studio/backend/api/__init__.py
/home/nse/humanoid-studio/backend/api/routes_devices.py
/home/nse/humanoid-studio/backend/api/routes_flash.py
/home/nse/humanoid-studio/backend/api/routes_motors.py
/home/nse/humanoid-studio/backend/api/routes_robot.py
/home/nse/humanoid-studio/backend/humanoid/actuator.py
/home/nse/humanoid-studio/backend/humanoid/can_bus.py
/home/nse/humanoid-studio/backend/humanoid/flash.py
/home/nse/humanoid-studio/backend/humanoid/__init__.py
/home/nse/humanoid-studio/backend/humanoid/robot_config.py
/home/nse/humanoid-studio/backend/humanoid/robot.py
/home/nse/humanoid-studio/backend/main.py
/home/nse/humanoid-studio/backend/requirements.txt
/home/nse/humanoid-studio/configs/humanoid_lite.json
/home/nse/humanoid-studio/HANDOFF.md
/home/nse/humanoid-studio/README.md
```

### configs/humanoid_lite.json (full content)

```json
{
  "robot_name": "humanoid_lite",
  "joints": {
    "left_hip_roll_joint":    { "joint_name": "left_hip_roll_joint",    "joint_type": "revolute", "can_channel": "can0", "can_id": 1,  "phase_inverted": true,  "electrical_offset": -32.2362, "gear_ratio": -15.0, "position_limits": {"min": null, "max": null}, "position_kp": 20.0, "position_ki": 0.0, "velocity_kp": 1.0, "velocity_ki": 0.0, "torque_limit": 2.0, "velocity_limit": 20.0, "position_offset": 0.0, "torque_filter_alpha": 0.1454, "current_limit": 20.0, "current_kp": 0.1664, "current_ki": 5746.5, "undervoltage_threshold": 0.0, "overvoltage_threshold": 0.0, "bus_voltage_filter_alpha": 0.2696, "pole_pairs": 14, "torque_constant": 0.0659, "max_calibration_current": 3.0, "cpr": 4096, "encoder_position_offset": 0.0, "velocity_filter_alpha": 0.7154, "watchdog_timeout": 1000, "fast_frame_frequency": 0 },
    "left_hip_yaw_joint":     { "joint_name": "left_hip_yaw_joint",     "joint_type": "revolute", "can_channel": "can0", "can_id": 3,  "phase_inverted": true,  "electrical_offset": -44.2742, "gear_ratio": -15.0, "position_limits": {"min": null, "max": null}, "position_kp": 20.0, "position_ki": 0.0, "velocity_kp": 1.0, "velocity_ki": 0.0, "torque_limit": 2.0, "velocity_limit": 20.0, "position_offset": 0.0, "torque_filter_alpha": 0.1454, "current_limit": 20.0, "current_kp": 0.1664, "current_ki": 5746.5, "undervoltage_threshold": 0.0, "overvoltage_threshold": 0.0, "bus_voltage_filter_alpha": 0.2696, "pole_pairs": 14, "torque_constant": 0.0659, "max_calibration_current": 3.0, "cpr": 4096, "encoder_position_offset": 0.0, "velocity_filter_alpha": 0.7154, "watchdog_timeout": 1000, "fast_frame_frequency": 0 },
    "left_hip_pitch_joint":   { "joint_name": "left_hip_pitch_joint",   "joint_type": "revolute", "can_channel": "can0", "can_id": 5,  "phase_inverted": true,  "electrical_offset":   1.0842, "gear_ratio": -15.0, "position_limits": {"min": null, "max": null}, "position_kp": 20.0, "position_ki": 0.0, "velocity_kp": 1.0, "velocity_ki": 0.0, "torque_limit": 2.0, "velocity_limit": 20.0, "position_offset": 0.0, "torque_filter_alpha": 0.1454, "current_limit": 20.0, "current_kp": 0.1664, "current_ki": 5746.5, "undervoltage_threshold": 0.0, "overvoltage_threshold": 0.0, "bus_voltage_filter_alpha": 0.2696, "pole_pairs": 14, "torque_constant": 0.0659, "max_calibration_current": 3.0, "cpr": 4096, "encoder_position_offset": 0.0, "velocity_filter_alpha": 0.7154, "watchdog_timeout": 1000, "fast_frame_frequency": 0 },
    "left_knee_pitch_joint":  { "joint_name": "left_knee_pitch_joint",  "joint_type": "revolute", "can_channel": "can0", "can_id": 7,  "phase_inverted": true,  "electrical_offset": -19.9311, "gear_ratio": -15.0, "position_limits": {"min": null, "max": null}, "position_kp": 20.0, "position_ki": 0.0, "velocity_kp": 1.0, "velocity_ki": 0.0, "torque_limit": 2.0, "velocity_limit": 20.0, "position_offset": 0.0, "torque_filter_alpha": 0.1454, "current_limit": 20.0, "current_kp": 0.1664, "current_ki": 5746.5, "undervoltage_threshold": 0.0, "overvoltage_threshold": 0.0, "bus_voltage_filter_alpha": 0.2696, "pole_pairs": 14, "torque_constant": 0.0659, "max_calibration_current": 3.0, "cpr": 4096, "encoder_position_offset": 0.0, "velocity_filter_alpha": 0.7154, "watchdog_timeout": 1000, "fast_frame_frequency": 0 },
    "left_ankle_pitch_joint": { "joint_name": "left_ankle_pitch_joint", "joint_type": "revolute", "can_channel": "can0", "can_id": 11, "phase_inverted": true,  "electrical_offset":  55.3687, "gear_ratio": -15.0, "position_limits": {"min": null, "max": null}, "position_kp": 20.0, "position_ki": 0.0, "velocity_kp": 1.0, "velocity_ki": 0.0, "torque_limit": 2.0, "velocity_limit": 20.0, "position_offset": 0.0, "torque_filter_alpha": 0.1454, "current_limit": 20.0, "current_kp": 0.1664, "current_ki": 5746.5, "undervoltage_threshold": 0.0, "overvoltage_threshold": 0.0, "bus_voltage_filter_alpha": 0.2696, "pole_pairs": 14, "torque_constant": 0.0659, "max_calibration_current": 3.0, "cpr": 4096, "encoder_position_offset": 0.0, "velocity_filter_alpha": 0.7154, "watchdog_timeout": 1000, "fast_frame_frequency": 0 },
    "left_ankle_roll_joint":  { "joint_name": "left_ankle_roll_joint",  "joint_type": "revolute", "can_channel": "can0", "can_id": 13, "phase_inverted": true,  "electrical_offset":   5.3749, "gear_ratio": -15.0, "position_limits": {"min": null, "max": null}, "position_kp": 20.0, "position_ki": 0.0, "velocity_kp": 1.0, "velocity_ki": 0.0, "torque_limit": 2.0, "velocity_limit": 20.0, "position_offset": 0.0, "torque_filter_alpha": 0.1454, "current_limit": 20.0, "current_kp": 0.1664, "current_ki": 5746.5, "undervoltage_threshold": 0.0, "overvoltage_threshold": 0.0, "bus_voltage_filter_alpha": 0.2696, "pole_pairs": 14, "torque_constant": 0.0659, "max_calibration_current": 3.0, "cpr": 4096, "encoder_position_offset": 0.0, "velocity_filter_alpha": 0.7154, "watchdog_timeout": 1000, "fast_frame_frequency": 0 },
    "right_hip_roll_joint":    { "joint_name": "right_hip_roll_joint",    "joint_type": "revolute", "can_channel": "can1", "can_id": 2,  "phase_inverted": true,  "electrical_offset": -12.8011, "gear_ratio": -15.0, "position_limits": {"min": null, "max": null}, "position_kp": 20.0, "position_ki": 0.0, "velocity_kp": 1.0, "velocity_ki": 0.0, "torque_limit": 2.0, "velocity_limit": 20.0, "position_offset": 0.0, "torque_filter_alpha": 0.1454, "current_limit": 20.0, "current_kp": 0.1664, "current_ki": 5746.5, "undervoltage_threshold": 0.0, "overvoltage_threshold": 0.0, "bus_voltage_filter_alpha": 0.2696, "pole_pairs": 14, "torque_constant": 0.0659, "max_calibration_current": 3.0, "cpr": 4096, "encoder_position_offset": 0.0, "velocity_filter_alpha": 0.7154, "watchdog_timeout": 1000, "fast_frame_frequency": 0 },
    "right_hip_yaw_joint":     { "joint_name": "right_hip_yaw_joint",     "joint_type": "revolute", "can_channel": "can1", "can_id": 4,  "phase_inverted": true,  "electrical_offset": -25.6714, "gear_ratio": -15.0, "position_limits": {"min": null, "max": null}, "position_kp": 20.0, "position_ki": 0.0, "velocity_kp": 1.0, "velocity_ki": 0.0, "torque_limit": 2.0, "velocity_limit": 20.0, "position_offset": 0.0, "torque_filter_alpha": 0.1454, "current_limit": 20.0, "current_kp": 0.1664, "current_ki": 5746.5, "undervoltage_threshold": 0.0, "overvoltage_threshold": 0.0, "bus_voltage_filter_alpha": 0.2696, "pole_pairs": 14, "torque_constant": 0.0659, "max_calibration_current": 3.0, "cpr": 4096, "encoder_position_offset": 0.0, "velocity_filter_alpha": 0.7154, "watchdog_timeout": 1000, "fast_frame_frequency": 0 },
    "right_hip_pitch_joint":   { "joint_name": "right_hip_pitch_joint",   "joint_type": "revolute", "can_channel": "can1", "can_id": 6,  "phase_inverted": true,  "electrical_offset":  24.0968, "gear_ratio": -15.0, "position_limits": {"min": null, "max": null}, "position_kp": 20.0, "position_ki": 0.0, "velocity_kp": 1.0, "velocity_ki": 0.0, "torque_limit": 2.0, "velocity_limit": 20.0, "position_offset": 0.0, "torque_filter_alpha": 0.1454, "current_limit": 20.0, "current_kp": 0.1664, "current_ki": 5746.5, "undervoltage_threshold": 0.0, "overvoltage_threshold": 0.0, "bus_voltage_filter_alpha": 0.2696, "pole_pairs": 14, "torque_constant": 0.0659, "max_calibration_current": 3.0, "cpr": 4096, "encoder_position_offset": 0.0, "velocity_filter_alpha": 0.7154, "watchdog_timeout": 1000, "fast_frame_frequency": 0 },
    "right_knee_pitch_joint":  { "joint_name": "right_knee_pitch_joint",  "joint_type": "revolute", "can_channel": "can1", "can_id": 8,  "phase_inverted": true,  "electrical_offset":  35.6913, "gear_ratio": -15.0, "position_limits": {"min": null, "max": null}, "position_kp": 20.0, "position_ki": 0.0, "velocity_kp": 1.0, "velocity_ki": 0.0, "torque_limit": 2.0, "velocity_limit": 20.0, "position_offset": 0.0, "torque_filter_alpha": 0.1454, "current_limit": 20.0, "current_kp": 0.1664, "current_ki": 5746.5, "undervoltage_threshold": 0.0, "overvoltage_threshold": 0.0, "bus_voltage_filter_alpha": 0.2696, "pole_pairs": 14, "torque_constant": 0.0659, "max_calibration_current": 3.0, "cpr": 4096, "encoder_position_offset": 0.0, "velocity_filter_alpha": 0.7154, "watchdog_timeout": 1000, "fast_frame_frequency": 0 },
    "right_ankle_pitch_joint": { "joint_name": "right_ankle_pitch_joint", "joint_type": "revolute", "can_channel": "can1", "can_id": 12, "phase_inverted": true,  "electrical_offset":  40.3929, "gear_ratio": -15.0, "position_limits": {"min": null, "max": null}, "position_kp": 20.0, "position_ki": 0.0, "velocity_kp": 1.0, "velocity_ki": 0.0, "torque_limit": 2.0, "velocity_limit": 20.0, "position_offset": 0.0, "torque_filter_alpha": 0.1454, "current_limit": 20.0, "current_kp": 0.1664, "current_ki": 5746.5, "undervoltage_threshold": 0.0, "overvoltage_threshold": 0.0, "bus_voltage_filter_alpha": 0.2696, "pole_pairs": 14, "torque_constant": 0.0659, "max_calibration_current": 3.0, "cpr": 4096, "encoder_position_offset": 0.0, "velocity_filter_alpha": 0.7154, "watchdog_timeout": 1000, "fast_frame_frequency": 0 },
    "right_ankle_roll_joint":  { "joint_name": "right_ankle_roll_joint",  "joint_type": "revolute", "can_channel": "can1", "can_id": 14, "phase_inverted": false, "electrical_offset":   6.4591, "gear_ratio": -15.0, "position_limits": {"min": null, "max": null}, "position_kp": 20.0, "position_ki": 0.0, "velocity_kp": 1.0, "velocity_ki": 0.0, "torque_limit": 2.0, "velocity_limit": 20.0, "position_offset": 0.0, "torque_filter_alpha": 0.1454, "current_limit": 20.0, "current_kp": 0.1664, "current_ki": 5746.5, "undervoltage_threshold": 0.0, "overvoltage_threshold": 0.0, "bus_voltage_filter_alpha": 0.2696, "pole_pairs": 14, "torque_constant": 0.0659, "max_calibration_current": 3.0, "cpr": 4096, "encoder_position_offset": 0.0, "velocity_filter_alpha": 0.7154, "watchdog_timeout": 1000, "fast_frame_frequency": 0 },
    "left_shoulder_pitch_joint": { "joint_name": "left_shoulder_pitch_joint", "joint_type": "revolute", "can_channel": "can2", "can_id": 1,  "phase_inverted": true,  "electrical_offset":  68.0395, "gear_ratio": -15.0, "position_limits": {"min": null, "max": null}, "position_kp": 20.0, "position_ki": 0.0, "velocity_kp": 1.0, "velocity_ki": 0.0, "torque_limit": 2.0, "velocity_limit": 20.0, "position_offset": 0.0, "torque_filter_alpha": 0.1454, "current_limit": 20.0, "current_kp": 0.1664, "current_ki": 5746.5, "undervoltage_threshold": 0.0, "overvoltage_threshold": 0.0, "bus_voltage_filter_alpha": 0.2696, "pole_pairs": 14, "torque_constant": 0.0659, "max_calibration_current": 3.0, "cpr": 4096, "encoder_position_offset": 0.0, "velocity_filter_alpha": 0.7154, "watchdog_timeout": 1000, "fast_frame_frequency": 0 },
    "left_shoulder_roll_joint":  { "joint_name": "left_shoulder_roll_joint",  "joint_type": "revolute", "can_channel": "can2", "can_id": 3,  "phase_inverted": true,  "electrical_offset":  37.8376, "gear_ratio": -15.0, "position_limits": {"min": null, "max": null}, "position_kp": 20.0, "position_ki": 0.0, "velocity_kp": 1.0, "velocity_ki": 0.0, "torque_limit": 2.0, "velocity_limit": 20.0, "position_offset": 0.0, "torque_filter_alpha": 0.1454, "current_limit": 20.0, "current_kp": 0.1664, "current_ki": 5746.5, "undervoltage_threshold": 0.0, "overvoltage_threshold": 0.0, "bus_voltage_filter_alpha": 0.2696, "pole_pairs": 14, "torque_constant": 0.0659, "max_calibration_current": 3.0, "cpr": 4096, "encoder_position_offset": 0.0, "velocity_filter_alpha": 0.7154, "watchdog_timeout": 1000, "fast_frame_frequency": 0 },
    "left_shoulder_yaw_joint":   { "joint_name": "left_shoulder_yaw_joint",   "joint_type": "revolute", "can_channel": "can2", "can_id": 5,  "phase_inverted": true,  "electrical_offset":   4.9700, "gear_ratio": -15.0, "position_limits": {"min": null, "max": null}, "position_kp": 20.0, "position_ki": 0.0, "velocity_kp": 1.0, "velocity_ki": 0.0, "torque_limit": 2.0, "velocity_limit": 20.0, "position_offset": 0.0, "torque_filter_alpha": 0.1454, "current_limit": 20.0, "current_kp": 0.1664, "current_ki": 5746.5, "undervoltage_threshold": 0.0, "overvoltage_threshold": 0.0, "bus_voltage_filter_alpha": 0.2696, "pole_pairs": 14, "torque_constant": 0.0659, "max_calibration_current": 3.0, "cpr": 4096, "encoder_position_offset": 0.0, "velocity_filter_alpha": 0.7154, "watchdog_timeout": 1000, "fast_frame_frequency": 0 },
    "left_elbow_pitch_joint":    { "joint_name": "left_elbow_pitch_joint",    "joint_type": "revolute", "can_channel": "can2", "can_id": 7,  "phase_inverted": true,  "electrical_offset":  78.9262, "gear_ratio": -15.0, "position_limits": {"min": null, "max": null}, "position_kp": 20.0, "position_ki": 0.0, "velocity_kp": 1.0, "velocity_ki": 0.0, "torque_limit": 2.0, "velocity_limit": 20.0, "position_offset": 0.0, "torque_filter_alpha": 0.1454, "current_limit": 20.0, "current_kp": 0.1664, "current_ki": 5746.5, "undervoltage_threshold": 0.0, "overvoltage_threshold": 0.0, "bus_voltage_filter_alpha": 0.2696, "pole_pairs": 14, "torque_constant": 0.0659, "max_calibration_current": 3.0, "cpr": 4096, "encoder_position_offset": 0.0, "velocity_filter_alpha": 0.7154, "watchdog_timeout": 1000, "fast_frame_frequency": 0 },
    "left_wrist_yaw_joint":      { "joint_name": "left_wrist_yaw_joint",      "joint_type": "revolute", "can_channel": "can2", "can_id": 9,  "phase_inverted": true,  "electrical_offset":   2.4043, "gear_ratio": -15.0, "position_limits": {"min": null, "max": null}, "position_kp": 20.0, "position_ki": 0.0, "velocity_kp": 1.0, "velocity_ki": 0.0, "torque_limit": 2.0, "velocity_limit": 20.0, "position_offset": 0.0, "torque_filter_alpha": 0.1454, "current_limit": 20.0, "current_kp": 0.1664, "current_ki": 5746.5, "undervoltage_threshold": 0.0, "overvoltage_threshold": 0.0, "bus_voltage_filter_alpha": 0.2696, "pole_pairs": 14, "torque_constant": 0.0659, "max_calibration_current": 3.0, "cpr": 4096, "encoder_position_offset": 0.0, "velocity_filter_alpha": 0.7154, "watchdog_timeout": 1000, "fast_frame_frequency": 0 },
    "right_shoulder_pitch_joint": { "joint_name": "right_shoulder_pitch_joint", "joint_type": "revolute", "can_channel": "can3", "can_id": 2,  "phase_inverted": true,  "electrical_offset":  75.3983, "gear_ratio": -15.0, "position_limits": {"min": null, "max": null}, "position_kp": 20.0, "position_ki": 0.0, "velocity_kp": 1.0, "velocity_ki": 0.0, "torque_limit": 2.0, "velocity_limit": 20.0, "position_offset": 0.0, "torque_filter_alpha": 0.1454, "current_limit": 20.0, "current_kp": 0.1664, "current_ki": 5746.5, "undervoltage_threshold": 0.0, "overvoltage_threshold": 0.0, "bus_voltage_filter_alpha": 0.2696, "pole_pairs": 14, "torque_constant": 0.0659, "max_calibration_current": 3.0, "cpr": 4096, "encoder_position_offset": 0.0, "velocity_filter_alpha": 0.7154, "watchdog_timeout": 1000, "fast_frame_frequency": 0 },
    "right_shoulder_roll_joint":  { "joint_name": "right_shoulder_roll_joint",  "joint_type": "revolute", "can_channel": "can3", "can_id": 4,  "phase_inverted": true,  "electrical_offset":  81.8248, "gear_ratio": -15.0, "position_limits": {"min": null, "max": null}, "position_kp": 20.0, "position_ki": 0.0, "velocity_kp": 1.0, "velocity_ki": 0.0, "torque_limit": 2.0, "velocity_limit": 20.0, "position_offset": 0.0, "torque_filter_alpha": 0.1454, "current_limit": 20.0, "current_kp": 0.1664, "current_ki": 5746.5, "undervoltage_threshold": 0.0, "overvoltage_threshold": 0.0, "bus_voltage_filter_alpha": 0.2696, "pole_pairs": 14, "torque_constant": 0.0659, "max_calibration_current": 3.0, "cpr": 4096, "encoder_position_offset": 0.0, "velocity_filter_alpha": 0.7154, "watchdog_timeout": 1000, "fast_frame_frequency": 0 },
    "right_shoulder_yaw_joint":   { "joint_name": "right_shoulder_yaw_joint",   "joint_type": "revolute", "can_channel": "can3", "can_id": 6,  "phase_inverted": true,  "electrical_offset":  83.6760, "gear_ratio": -15.0, "position_limits": {"min": null, "max": null}, "position_kp": 20.0, "position_ki": 0.0, "velocity_kp": 1.0, "velocity_ki": 0.0, "torque_limit": 2.0, "velocity_limit": 20.0, "position_offset": 0.0, "torque_filter_alpha": 0.1454, "current_limit": 20.0, "current_kp": 0.1664, "current_ki": 5746.5, "undervoltage_threshold": 0.0, "overvoltage_threshold": 0.0, "bus_voltage_filter_alpha": 0.2696, "pole_pairs": 14, "torque_constant": 0.0659, "max_calibration_current": 3.0, "cpr": 4096, "encoder_position_offset": 0.0, "velocity_filter_alpha": 0.7154, "watchdog_timeout": 1000, "fast_frame_frequency": 0 },
    "right_elbow_pitch_joint":    { "joint_name": "right_elbow_pitch_joint",    "joint_type": "revolute", "can_channel": "can3", "can_id": 8,  "phase_inverted": false, "electrical_offset":  56.7337, "gear_ratio": -15.0, "position_limits": {"min": null, "max": null}, "position_kp": 20.0, "position_ki": 0.0, "velocity_kp": 1.0, "velocity_ki": 0.0, "torque_limit": 2.0, "velocity_limit": 20.0, "position_offset": 0.0, "torque_filter_alpha": 0.1454, "current_limit": 20.0, "current_kp": 0.1664, "current_ki": 5746.5, "undervoltage_threshold": 0.0, "overvoltage_threshold": 0.0, "bus_voltage_filter_alpha": 0.2696, "pole_pairs": 14, "torque_constant": 0.0659, "max_calibration_current": 3.0, "cpr": 4096, "encoder_position_offset": 0.0, "velocity_filter_alpha": 0.7154, "watchdog_timeout": 1000, "fast_frame_frequency": 0 },
    "right_wrist_yaw_joint":      { "joint_name": "right_wrist_yaw_joint",      "joint_type": "revolute", "can_channel": "can3", "can_id": 10, "phase_inverted": true,  "electrical_offset":  78.1506, "gear_ratio": -15.0, "position_limits": {"min": null, "max": null}, "position_kp": 20.0, "position_ki": 0.0, "velocity_kp": 1.0, "velocity_ki": 0.0, "torque_limit": 2.0, "velocity_limit": 20.0, "position_offset": 0.0, "torque_filter_alpha": 0.1454, "current_limit": 20.0, "current_kp": 0.1664, "current_ki": 5746.5, "undervoltage_threshold": 0.0, "overvoltage_threshold": 0.0, "bus_voltage_filter_alpha": 0.2696, "pole_pairs": 14, "torque_constant": 0.0659, "max_calibration_current": 3.0, "cpr": 4096, "encoder_position_offset": 0.0, "velocity_filter_alpha": 0.7154, "watchdog_timeout": 1000, "fast_frame_frequency": 0 }
  }
}
```

---

## 6. CAN Protocol — Confirmed from Firmware

Source files read: `motor_controller_conf.h`, `motor_controller.c`, `can.c`, `can.h` from
`/home/nse/Recoil-Motor-Controller-BESC/Recoil-Motor-Controller-B-G431B-ESC1/`.
Also read: `recoil/can.py`, `recoil/core.py`, `recoil/fixed16.py` from Berkeley-Humanoid-Lite.

### 6.1 11-bit CAN Arbitration ID Bit Layout

Confirmed directly from `motor_controller.c` lines 599–604:

```c
uint16_t device_id = (rx_frame->id) & 0b1111111;   // bits [6:0]  — 7-bit node ID
uint16_t func_id   = (rx_frame->id) >> 7;           // bits [10:7] — 4-bit function code

// Encoding:
arb_id = (func_code << 7) | node_id
```

**Example: 0x48C**
```
0x48C = 1164 decimal
node_id   = 1164 & 0x7F = 12  → right_ankle_pitch_joint (can_right_leg / can1)
func_code = 1164 >> 7   =  9  → FUNC_TRANSMIT_PDO_4 (autonomous broadcast)
```

### 6.2 All Function Codes (confirmed from FrameFunction enum in motor_controller_conf.h)

| Hex  | Firmware name            | recoil_protocol.py | Direction  | Payload                                             |
|------|--------------------------|--------------------|------------|-----------------------------------------------------|
| 0x0  | FUNC_NMT                 | NMT                | host→motor | `<BB>` mode, node_id; node_id=0 → broadcast all   |
| 0x1  | FUNC_SYNC_EMCY           | SYNC_EMCY          | —          | unused                                              |
| 0x2  | FUNC_TIME                | TIME               | —          | unused                                              |
| 0x3  | FUNC_TRANSMIT_PDO_1      | TX_PDO1            | motor→host | echoes all 8 bytes of host ping; data[0]=0xCA      |
| 0x4  | FUNC_RECEIVE_PDO_1       | RX_PDO1            | host→motor | ping; data[0]=0xCA; resets watchdog                |
| 0x5  | FUNC_TRANSMIT_PDO_2      | TX_PDO2            | motor→host | `<ff>` pos_measured_rad, vel_measured_rads          |
| 0x6  | FUNC_RECEIVE_PDO_2       | RX_PDO2            | host→motor | `<ff>` pos_target_rad, vel_ff_rads; resets watchdog|
| 0x7  | FUNC_TRANSMIT_PDO_3      | TX_PDO3            | motor→host | `<ff>` pos_measured_rad, torque_measured            |
| 0x8  | FUNC_RECEIVE_PDO_3       | RX_PDO3            | host→motor | `<ff>` pos_target_rad, torque_target                |
| 0x9  | FUNC_TRANSMIT_PDO_4      | TX_PDO4            | motor→host | `<ff>` pos_rad, vel_rads (htim8 ISR auto-broadcast)|
| 0xA  | FUNC_RECEIVE_PDO_4       | RX_PDO4            | host→motor | no-op                                               |
| 0xB  | FUNC_TRANSMIT_SDO        | TX_SDO             | motor→host | 4-byte parameter value (uint32 LE)                  |
| 0xC  | FUNC_RECEIVE_SDO         | RX_SDO             | host→motor | read (0x40) or write (0x20) a parameter             |
| 0xD  | FUNC_FLASH               | FLASH              | host→motor | byte 0: 0x01=store to Flash, 0x02=load from Flash   |
| 0xE  | FUNC_HEARTBEAT           | HEARTBEAT          | host→motor | no data; resets watchdog only                       |

### 6.3 Broadcast Payload Byte Map

All broadcast PDOs use **IEEE 754 float32, little-endian**. No Fixed16/Q8.8 in any broadcast frame.

```
PDO2 TX (func=0x5):
  bytes [0:4]  float32 LE  position_measured_rad   (output-shaft, after gear ratio)
  bytes [4:8]  float32 LE  velocity_measured_rads  (output-shaft, after gear ratio)

PDO3 TX (func=0x7):
  bytes [0:4]  float32 LE  position_measured_rad
  bytes [4:8]  float32 LE  torque_measured

PDO4 TX (func=0x9, autonomous broadcast via htim8):
  bytes [0:4]  float32 LE  position_rad            (output-shaft)
  bytes [4:8]  float32 LE  velocity_rads           (output-shaft)
```

No current/Iq field exists in any broadcast frame. Iq is available only via SDO read.

**Decode in Python:**
```python
import struct
pos, vel = struct.unpack_from('<ff', data)   # works for PDO2 TX and PDO4 TX
```

**Hardware verification:** Sample bytes `E8 87 8D BD FF E8 09 BF` from can_right_leg →
```
pos = struct.unpack('<f', bytes.fromhex('E8878DBD'))[0] = -0.0693 rad ≈ -3.97°
vel = struct.unpack('<f', bytes.fromhex('FFE809BF'))[0] = -0.538 rad/s
```

### 6.4 Discrepancies Between Firmware and Berkeley core.py

| Item | Firmware | Berkeley core.py | Impact |
|------|----------|------------------|--------|
| PDO3 (func 0x7/0x8) | Fully implemented: position+torque RX/TX | **Not implemented** — no send_pdo3, no recv_pdo3 | Cannot use torque control mode via Berkeley lib |
| PDO1 echo size | Firmware echoes all 8 bytes of the ping frame (copies two uint32 words from rx to tx) | Host only sends 1 byte (`b'\xCA'`); bytes 1-7 in the echo will be 0x00 | Still works: check `data[0] == 0xCA` is valid |
| NMT wildcard | Accepts node_id=0 to address all devices on bus | core.py sends with specific node_id; wildcard path not exercised | Minor: broadcast NMT untested in Python lib |
| Fixed16 class | Not used in protocol at any point | `fixed16.py` exists but is never imported by any script | Dead code — safe to ignore |

### 6.5 recoil_protocol.py Status

`backend/humanoid/recoil_protocol.py` is **correct as written** — all values confirmed against firmware:
- `NODE_ID_MASK = 0x7F` ✓
- `FUNC_ID_SHIFT = 7` ✓
- All 15 `Func.*` constants match FrameFunction enum values ✓
- `TELEMETRY_FUNC_IDS = frozenset({Func.TX_PDO2, Func.TX_PDO4})` ✓ (PDO3 excluded intentionally)
- `decode_telemetry()` using `struct.unpack_from('<ff', data)` ✓
- `PING_MAGIC = 0xCA` ✓

---

*End of handoff document. Updated 2026-05-14.*

---

## 7. Joint ID Mapping (Berkeley Humanoid Lite)

All joints use gear_ratio magnitude = 15.0. The sign (±) controls encoder direction and
must be verified per joint using the Position Calibration direction check in the motor tab.
Left and right joints are physically mirrored — right-side limits are sign-inverted relative
to left-side limits unless verified otherwise.

Convention: negative angle = inward/backward motion, positive = outward/forward motion.
Example: Hip Roll −10° = leg rotates inward, +90° = leg rotates outward.

Source: https://berkeley-humanoid-lite.gitbook.io/docs/in-depth-contents/joint-id-mapping

| Joint Name                 | CAN ID | Channel        | Range (deg)       | Notes                        |
|----------------------------|--------|----------------|-------------------|------------------------------|
| left_hip_roll_joint        | 1      | can_left_leg   | [−10, 90]         | −10° inward, +90° outward    |
| right_hip_roll_joint       | 2      | can_right_leg  | [−90, 10]         | Mirrored from left           |
| left_hip_yaw_joint         | 3      | can_left_leg   | [−56.25, 33.75]   |                              |
| right_hip_yaw_joint        | 4      | can_right_leg  | [−33.75, 56.25]   | Mirrored from left           |
| left_hip_pitch_joint       | 5      | can_left_leg   | [TBD]             |                              |
| right_hip_pitch_joint      | 6      | can_right_leg  | [TBD]             |                              |
| left_knee_joint            | 7      | can_left_leg   | [TBD]             | ~140° range                  |
| right_knee_joint           | 8      | can_right_leg  | [TBD]             |                              |
| left_ankle_roll_joint      | 9      | can_left_leg   | [−15, 15]         |                              |
| right_ankle_roll_joint     | 10     | can_right_leg  | [−15, 15]         |                              |
| left_ankle_pitch_joint     | 11     | can_left_leg   | [TBD]             |                              |
| right_ankle_pitch_joint    | 12     | can_right_leg  | [TBD]             |                              |
| left_shoulder_roll_joint   | 13     | can_left_arm   | [TBD]             |                              |
| right_shoulder_roll_joint  | 14     | can_right_arm  | [TBD]             |                              |
| left_shoulder_pitch_joint  | 15     | can_left_arm   | [TBD]             |                              |
| right_shoulder_pitch_joint | 16     | can_right_arm  | [TBD]             |                              |
| left_shoulder_yaw_joint    | 17     | can_left_arm   | [TBD]             |                              |
| right_shoulder_yaw_joint   | 18     | can_right_arm  | [TBD]             |                              |
| left_elbow_joint           | 19     | can_left_arm   | [TBD]             |                              |
| right_elbow_joint          | 20     | can_right_arm  | [TBD]             |                              |

TBD ranges should be filled in as each joint is verified using the position calibration
direction check in the motor tab. Right-side mirroring should be confirmed with hardware.

*Updated 2026-05-15.*

---

## 8. Parameter Address Audit — 2026-05-15

### 8.1 Audit Result: No Address Mismatches

A complete audit of all CAN SDO parameter addresses was performed comparing:
- `apply_config()` write addresses (backend/humanoid/actuator.py)
- `read_config_from_device()` read-back addresses (actuator.py)
- `get_state()` telemetry read addresses (actuator.py)
- Flash wizard CAN writes (backend/humanoid/flash.py)

**All 29 configurable parameters use identical `Parameter` enum addresses for write and read-back.** The flash wizard only writes `FAST_FRAME_FREQUENCY = 100` via CAN; all other parameters are applied via `apply_config()` on robot connect.

### 8.2 Bug Found: ±∞ Position Limits → NaN in Firmware

**Symptom:** After connecting, all motors error out immediately with bus voltage drops, position spikes, and velocity spikes.

**Root cause:** All joints have `position_limits: {"min": null, "max": null}`. The Python code converted `null` to `float(±inf)` before writing to `POSITION_CONTROLLER_POSITION_LIMIT_LOWER/UPPER` (0x038/0x03C). The STM32 firmware performs arithmetic on these limits (e.g., `(upper + lower) / 2`), and `inf + (-inf) = NaN`. NaN propagates through the entire position controller, producing undefined torque outputs and triggering POWERSTAGE faults.

**Fix applied:** `backend/humanoid/robot_config.py` — `PositionLimits.lower_bound` and `upper_bound` now return `±100 rad` (~±5730°) instead of `±inf` when `min`/`max` is `None`. This is physically unreachable for any joint but avoids NaN arithmetic in the firmware.

### 8.3 Bug Found: SDO Velocity Is Motor-Shaft, Not Output-Shaft

**Symptom:** Active telemetry velocity reads 15× too large compared to actual joint speed; appears correlated with position value.

**Root cause:** `POSITION_CONTROLLER_VELOCITY_MEASURED` (0x054), read via SDO in `get_state()`, stores **motor-shaft rad/s** — the firmware does not apply gear ratio to this parameter. By contrast:
- `POSITION_CONTROLLER_POSITION_MEASURED` (0x060) IS output-shaft (gear ratio applied by firmware)
- PDO4 fast-frame velocity IS output-shaft (confirmed from firmware source in Section 6)

With `gear_ratio = −15`, motor-shaft velocity is 15× larger than output-shaft and inverted in sign.

**Fix applied:** `backend/humanoid/actuator.py` `get_state()` — divide raw SDO velocity by `gear_ratio` to produce output-shaft velocity consistent with the position reading.

### 8.4 Passive (PDO4) vs Active (SDO) Telemetry

| Field | PDO4 passive | SDO active (get_state) | Notes |
|-------|-------------|------------------------|-------|
| position | output-shaft rad | output-shaft rad | Both consistent after gear ratio applied by firmware |
| velocity | output-shaft rad/s | **was** motor-shaft rad/s | Fixed by dividing by gear_ratio in get_state() |
| current | N/A | A (Iq) | Only available via SDO |
| bus voltage | N/A | V | Only available via SDO |

---

## SDO Race Condition Fix — 2026-05-15

### Root Cause Found

Random garbage error-register values ("all errors set simultaneously, then clear in 1 frame") were caused by **concurrent SDO reads to the same ESC device from two different asyncio coroutines** — for example, two WebSocket telemetry loops running simultaneously, or the WebSocket telemetry loop racing with a `GET /motors/{joint}` HTTP request.

The Recoil firmware SDO response format is 4 raw value bytes with **no param_id echo**. When two coroutines both register a `_Waiter(device_id, TRANSMIT_SDO)` simultaneously, whichever waiter fires first gets the other coroutine's response bytes. If telemetry's ERROR read gets position bytes, interpreting a float like `1.57 rad → 0x3FC90FDB` as a uint32 error bitmask sets bits 0,1,6,7,8,10,22,23,25 all at once — which looks like every possible error flag.

**What it was NOT:** The TX_PDO4 autonomous broadcast (0x9) and TX_SDO response (0xB) have different function codes → different CAN IDs → they cannot match the same waiter. The broadcast was never the problem.

### Firmware CAN ID Bit Layout (Confirmed from Source)

`arb_id = (func_code << 7) | node_id`

| bits [10:7] | bits [6:0] |
|---|---|
| function code (4 bits) | node_id (7 bits) |

Verification: TX_PDO4 for node 12 = (9<<7)|12 = **0x48C** ✓ (matches observed candump)

| Frame Type | Func | CAN ID (node 12) | Direction |
|---|---|---|---|
| TX_PDO4 broadcast | 0x9 | 0x48C | ESC → host |
| TX_SDO response | 0xB | 0x58C | ESC → host |
| RX_SDO request | 0xC | 0x60C | host → ESC |
| TX_PDO2 echo | 0x5 | 0x28C | ESC → host |
| RX_PDO2 command | 0x6 | 0x30C | host → ESC |
| HEARTBEAT | 0xE | 0x70C | host → ESC |

### SDO Data Format (Firmware Ground Truth)

**SDO read request** (host → ESC, func=RX_SDO):
```
data[0]   = 0x40 (ccs=2, upload request)
data[1:3] = param_id (little-endian uint16)
data[3]   = 0x00 (sub-index)
data[4:8] = 0x00 (padding)
```

**SDO read response** (ESC → host, func=TX_SDO):
```
data[0:4] = parameter value (raw 4 bytes, little-endian)
data[4:8] = undefined (firmware only sends DLC=4)
```
No param_id echo. The firmware simply does:
`*(uint32_t*)tx_frame->data = *(uint32_t*)((uint8_t*)controller + parameter_id)`

### Fix Applied — `backend/humanoid/can_bus.py`

Added `_device_sdo_locks: dict[int, asyncio.Lock]` — a per-device SDO serialization lock.

`_sdo_read()` and `_sdo_write()` now acquire `_sdo_lock(device_id)` before proceeding. This ensures only one SDO transaction is in flight per device at a time, regardless of how many concurrent coroutines try to access the same device.

The lock is created lazily on first use per device_id. It serialises:
- Two concurrent `get_state()` calls (two WebSocket clients)
- Telemetry `get_all_states()` + API `GET /motors/{joint}` or `read_config_from_device()`

### Verified Decode of 0x48C Frame

Sample: `E8 87 8D BD FF E8 09 BF`
- node_id = 12, func_code = 0x9 (TX_PDO4) ✓
- Position = -0.0691 rad (-3.96°)   ← physically plausible
- Velocity = -0.5387 rad/s (-30.87°/s)

Decode function in `recoil_protocol.py` is correct. No changes needed.

### Debug Endpoint Added

`GET http://localhost:8765/debug/decode_frame?arb_id=0x48C&data=E8878DBD FFE809BF`

Returns JSON with fully decoded frame including position_deg, velocity_dps.

---

## Wiki Repository

The GitHub Wiki is a separate git repo cloned at:
  /home/nse/humanoid-studio-wiki/

The remote is set to SSH: git@github.com:topolski852/humanoid-studio.wiki.git

To update wiki pages:
  cp /home/nse/humanoid-studio/wiki/PAGENAME.md /home/nse/humanoid-studio-wiki/
  cd /home/nse/humanoid-studio-wiki
  git add .
  git commit -m "Update wiki: description of change"
  git push origin master

NOTE: Push requires the SSH key to be added to GitHub account:
  ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAICTnbkcReV7LV59jcLiRqy+/3pw+SdbklvCv+13v5sGa kelly.topolski@warlocks1507.com
  Add at: https://github.com/settings/ssh/new

Source wiki files live in /home/nse/humanoid-studio/wiki/ (tracked in main repo).
The wiki repo at /home/nse/humanoid-studio-wiki/ is the publishing copy.
