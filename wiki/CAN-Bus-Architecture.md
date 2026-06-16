# CAN Bus Architecture

This page is a technical reference for the CAN communication layer. It covers the network topology, the Recoil protocol frame formats, and important implementation details in the C++ daemon and Python bridge.

---

## Network topology

### The 4-bus architecture

The Berkeley Humanoid Lite uses four separate CAN buses, one per limb. Each bus runs at 1,000,000 bps (1 Mbit/s). All buses operate independently — a problem on one bus does not affect the others.

| Bus name | Limb | Joints (CAN ID → joint name) |
|---|---|---|
| `can_left_leg` | Left leg | 1 → left_hip_roll, 3 → left_hip_yaw, 5 → left_hip_pitch, 7 → left_knee_pitch, 11 → left_ankle_pitch, 13 → left_ankle_roll |
| `can_right_leg` | Right leg | 2 → right_hip_roll, 4 → right_hip_yaw, 6 → right_hip_pitch, 8 → right_knee_pitch, 12 → right_ankle_pitch, 14 → right_ankle_roll |
| `can_left_arm` | Left arm | 1 → left_shoulder_pitch, 3 → left_shoulder_roll, 5 → left_shoulder_yaw, 7 → left_elbow_pitch, 9 → left_wrist_yaw |
| `can_right_arm` | Right arm | 2 → right_shoulder_pitch, 4 → right_shoulder_roll, 6 → right_shoulder_yaw, 8 → right_elbow_pitch, 10 → right_wrist_yaw |

**Total: 22 joints.**

### Why CAN IDs are not globally unique

CAN IDs are only unique within a single bus. `can_left_leg` has a motor with ID 1 (`left_hip_roll_joint`) and `can_left_arm` also has a motor with ID 1 (`left_shoulder_pitch_joint`). They are physically on separate wires and do not interfere with each other.

This is why the app addresses every motor as `(bus_name, can_id)` rather than by `can_id` alone. The REST API uses `joint_name` (a globally unique string like `"left_hip_roll_joint"`) as the URL key, which maps to the correct `(bus, can_id)` pair in the robot config.

---

## Recoil Protocol

### CAN ID bit layout

The Recoil firmware uses 11-bit standard CAN frames. The arbitration ID encodes both the function code and the node ID:

```
bits [10:7]  function code   (4 bits, 0x0–0xE)
bits  [6:0]  node ID         (7 bits, 1–127)

arbitration_id = (func_code << 7) | node_id
```

Example: arbitration ID `0x48C`
- `node_id   = 0x48C & 0x7F = 12`  → right_ankle_pitch_joint
- `func_code = 0x48C >> 7   =  9`  → TX_PDO4 (autonomous broadcast)

### Function codes

| Code | Name | Direction | Purpose |
|---|---|---|---|
| 0x0 | NMT | host → motor | Mode change command |
| 0x1 | SYNC_EMCY | — | Unused in this firmware |
| 0x2 | TIME | — | Unused in this firmware |
| 0x3 | TX_PDO1 | motor → host | Echo response to ping; data[0] reflects magic byte 0xCA |
| 0x4 | RX_PDO1 | host → motor | Ping request; send 0xCA, expect TX_PDO1 echo; also resets watchdog |
| 0x5 | TX_PDO2 | motor → host | Position echo: [float32 pos_rad, float32 vel_rads] |
| 0x6 | RX_PDO2 | host → motor | Position command: [float32 pos_target, float32 vel_ff]; also resets watchdog |
| 0x7 | TX_PDO3 | motor → host | Unused in current firmware |
| 0x8 | RX_PDO3 | host → motor | Unused in current firmware |
| 0x9 | TX_PDO4 | motor → host | Autonomous broadcast at configured Hz: [float32 pos_rad, float32 vel_rads] |
| 0xA | RX_PDO4 | host → motor | No-op |
| 0xB | TX_SDO | motor → host | SDO read response: 4 raw bytes of value |
| 0xC | RX_SDO | host → motor | SDO read or write request |
| 0xD | FLASH | host → motor | Store (byte 0=1) or Load (byte 0=2) config from Flash |
| 0xE | HEARTBEAT | host → motor | Feed watchdog only; no data required |

### TX_PDO4 — the autonomous broadcast frame

Each ESC transmits TX_PDO4 autonomously at the rate configured in `fast_frame_frequency` (default 100 Hz after commissioning, 0 Hz on a fresh flash). This is the primary source of passive telemetry that the CAN Monitor and Dashboard use.

```
CAN ID: (0x9 << 7) | node_id
DLC: 8
data[0:4]: float32 little-endian  position_rad   (output shaft, after gear ratio)
data[4:8]: float32 little-endian  velocity_rads  (output shaft, after gear ratio)
```

Both values are output-shaft quantities. The firmware divides by `gear_ratio` internally before transmitting. No current field is included in this frame — quadrature current (Iq) is only available via SDO read.

### SDO write (download, host → motor)

Used to write any parameter into the ESC's RAM. The parameter address is the byte offset into the firmware `MotorController` struct.

```
CAN ID: (0xC << 7) | node_id   = 0x600 | node_id
DLC: 8
data[0]:    0x20  (CCS=1 download, expedited, size=4)
data[1:3]:  uint16 little-endian  parameter_id
data[3]:    0x00
data[4:8]:  4 bytes little-endian value (float32 or int32 or uint32 depending on parameter)
```

The motor sends no acknowledgement. The write is fire-and-forget.

### SDO read (upload, host → motor)

Used to read any parameter from the ESC's RAM.

```
Request:
  CAN ID: (0xC << 7) | node_id
  DLC: 8
  data[0]:    0x40  (CCS=2 upload initiate)
  data[1:3]:  uint16 little-endian  parameter_id
  data[3:7]:  0x00 0x00 0x00 0x00

Response (TX_SDO) — current firmware (8-byte):
  CAN ID: (0xB << 7) | node_id
  DLC: 8
  data[0]:    0x43  (SDO_READ_RESP)
  data[1:3]:  uint16 little-endian  parameter_id (echo of the requested param)
  data[3]:    0x00
  data[4:8]:  4 raw bytes (interpret as float32 or uint32 or int32)

Response (TX_SDO) — legacy firmware (4-byte, fallback):
  CAN ID: (0xB << 7) | node_id
  DLC: 4
  data[0:4]:  4 raw bytes only, no parameter_id echo
```

Current firmware echoes the parameter ID in the response, eliminating the race condition described below. The daemon handles both formats — 8-byte responses are matched by param_id; 4-byte responses rely on the mutex/condition-variable mailbox to ensure only one SDO transaction is in flight per motor at a time.

### Data encoding

All values in the Recoil protocol are raw IEEE 754 float32 or integer types, little-endian. There is no fixed-point encoding in the protocol.

- Float parameters: `struct.pack('<f', value)` / `struct.unpack('<f', raw)`
- Unsigned int parameters: `struct.pack('<L', value)` / `struct.unpack('<L', raw)`
- Signed int parameters (phase_order): `struct.pack('<l', value)` / `struct.unpack('<l', raw)`
- Position and velocity are always in radians at the output shaft

### NMT mode change frame

```
CAN ID: (0x0 << 7) | node_id
DLC: 2
data[0]: uint8  mode value
data[1]: uint8  addressed node ID

Mode values:
  0x00 = DISABLED   (PWM fully off; motor coasts; recovery requires error clear)
  0x01 = IDLE       (PWM off; no torque; can transition directly to POSITION)
  0x02 = DAMPING    (PWM on; coils shorted; regenerative braking)
  0x05 = CALIBRATION (flux-offset calibration sequence; autonomous, takes ~15 s)
  0x13 = POSITION   (active position control)
```

### FLASH frame (store/load config)

```
CAN ID: (0xD << 7) | node_id
DLC: 1
data[0]: 0x01 = store MotorController struct to Flash page 63
         0x02 = load MotorController struct from Flash page 63
```

### Watchdog keepalive requirement

The firmware has a safety watchdog timer with a default timeout of 1000 ms. If no PDO2 command or HEARTBEAT frame is received within the timeout window, the firmware transitions the motor to DAMPING mode and sets the `WATCHDOG_TIMEOUT` error bit.

In Humanoid Studio, the **C++ daemon** feeds the watchdog from its 200 Hz control loop — it sends HEARTBEAT frames to all IDLE joints every 200 ms (5 Hz), and PDO2 position commands to all ENABLED joints at 200 Hz. This is independent of the Python backend, WebSocket clients, or any asyncio scheduling. Python has no watchdog responsibility.

If you stop the **daemon** process, the watchdog will fire approximately 1 second later and all motors will enter DAMPING mode. Stopping the Python backend alone does not trigger the watchdog — the daemon continues running.

---

## SDO Race Condition (historical, now fixed)

### What the bug was

Early in development, random garbage values appeared in the error register displayed in the motor tabs. A motor with no fault would briefly show WATCHDOG_TIMEOUT or ENCODER_FAULT and then clear. This happened multiple times per second.

### Why it happened

The root cause was concurrent SDO reads to the same motor from two different asyncio coroutines — the WebSocket telemetry loop and an incoming GET `/motors/{joint}` request.

The firmware SDO response frame (`TX_SDO`, func_code 0xB) contains only 4 raw bytes of value. There is no echo of the requested parameter ID. When two coroutines both registered a waiter for `(device_id, TRANSMIT_SDO)` and both transmitted their requests, each waiter consumed whichever response arrived first — regardless of which parameter it was for. One coroutine got the other coroutine's response bytes. Reading 4 bytes of a float32 as a uint32 error register produced arbitrary bit patterns.

### The fix

The C++ daemon's `actuator.cpp` uses a mutex and condition variable to serialize SDO transactions per motor:

```cpp
std::mutex      sdo_ack_mutex_;
std::condition_variable sdo_ack_cv_;
```

Before transmitting an SDO read, the daemon sets a pending flag and waits on `sdo_ack_cv_` with a timeout. When the CAN reader thread receives a TX_SDO response for this motor, it stores the value and notifies the CV. Only one SDO transaction is in flight per motor at a time — the control loop processes them sequentially per actuator tick.

With current firmware (8-byte responses that echo the parameter_id), the daemon additionally validates that the incoming param_id matches what was requested, catching any stale or misrouted responses before they reach the caller.
