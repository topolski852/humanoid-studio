# Humanoid Studio — Technical Reference

**Audience:** an engineer about to write **new robot-control code** (e.g. a learned-policy
runner) against the Berkeley Humanoid Lite ESC firmware + control daemon in this repo.

**What this doc is:** a from-scratch reference for the *current* system. Every constant,
address, port, command name, and field below was verified against source (see the "Verified
from" notes). If something is not yet built, it is marked **PLANNED**.

**Golden rule for new control code:** you talk **UDP → the C++ daemon**. You do **not** open
CAN sockets, and you do not talk to the ESCs directly. The daemon owns every SocketCAN
interface and runs the hard real-time loop.

---

## 1. System architecture

```
        ┌────────────────────────────────────────────────────────────────┐
        │  YOUR NEW CONTROL CODE (learned-policy runner, trainer, etc.)    │
        │  - speaks the daemon UDP protocol (JSON) directly, or via        │
        │    backend/humanoid/daemon_client.py (DaemonClient)              │
        │  - NEVER opens a CAN socket                                       │
        └───────────────┬───────────────────────────▲────────────────────┘
                        │ cmd (JSON req/resp)        │ telemetry (JSON push)
                        │ UDP 127.0.0.1:9001         │ UDP :9000  (+ :9002 ESTOP)
        ┌───────────────▼───────────────────────────┴────────────────────┐
        │  C++ DAEMON  (daemon/, single process, owns ALL SocketCAN)       │
        │  - control loop: 200 Hz, SCHED_FIFO prio 80, pinned to CPU 0     │
        │  - per-joint Actuator state machine                              │
        │  - telemetry push loop: 1–100 Hz (config telemetry_hz, def 100)  │
        │  - CanBusManager: 4 SocketCAN buses @ 1 Mbit                     │
        └───────────────┬─────────────────────────────────────────────────┘
                        │ SocketCAN (raw CAN frames, Recoil protocol)
        ┌───────────────▼─────────────────────────────────────────────────┐
        │  22× Recoil ESC nodes (STM32G431 B-G431B-ESC1), 4 CAN buses      │
        │  each runs a 10 kHz FOC loop + 2 kHz position loop               │
        └─────────────────────────────────────────────────────────────────┘

  Side channel (not required for control):
        ┌─────────────────────────────────────────────────────────────────┐
        │  Python FastAPI backend  (backend/, localhost:8765)              │
        │  - thin proxy: wraps DaemonClient, exposes REST + WS to the UI    │
        │  - does NOT own CAN; it is just another daemon UDP client         │
        └───────────────▲─────────────────────────────────────────────────┘
                        │ REST / WebSocket
        ┌───────────────┴─────────────────────────────────────────────────┐
        │  Electron + React desktop app  (app/)                            │
        └─────────────────────────────────────────────────────────────────┘
```

**Where new control code plugs in:** at the daemon UDP boundary, alongside the FastAPI
backend. Two equally valid options:

1. **Import `DaemonClient`** from `backend/humanoid/daemon_client.py` (Python) — gives you a
   ready-made client with reconnect, telemetry cache, and a per-joint proxy.
2. **Speak the raw UDP/JSON protocol** yourself (any language). It is small; see §6.

Only **one** process should send position commands at a time. If your runner and the FastAPI
backend both stream `SET_POSITION`, they will fight. In practice: idle the UI's control before
running a policy, or drive everything through one process.

**Verified from:** `daemon/src/main.cpp` (ports/prio/cpu defaults), `daemon/src/control/robot.hpp`
& `robot.cpp` (thread model, 200 Hz, ports 9000/9001/9002), `daemon/src/control/control_loop.hpp`
(`period_s = 0.005` → 200 Hz, `SCHED_FIFO`), `backend/humanoid/daemon_client.py` (ports),
`backend/main.py` (FastAPI on 8765).

### Ports (all UDP, all on 127.0.0.1)

| Port | Direction | Purpose | Source |
|---|---|---|---|
| 9001 | client → daemon | JSON command request; daemon replies to sender | `main.cpp` `--cmd-port`, `daemon_client.py` |
| 9000 | daemon → client | JSON telemetry push, fire-and-forget | `main.cpp` `--tel-port`, `daemon_client.py` |
| 9002 | client → daemon | **ESTOP only**, never blocked by an in-flight command on 9001 | `robot.hpp` `priority_server_`, `daemon_client.py estop_all()` |
| 8765 | HTTP/WS | FastAPI backend (UI side channel, not the control path) | `backend/main.py` |

Control loop runs at **200 Hz** (`period_s = 0.005`), `SCHED_FIFO` priority **80** (needs
`cap_sys_nice` or root; falls back to `SCHED_OTHER` otherwise), CPU affinity **0**. Telemetry
push runs at `telemetry_hz` (config value, currently **100**; CLI `--tel-hz`, clamped 1–100).

---

## 2. Firmware reference

Firmware source of record is the new repo **`humanoid-esc-firmware`**
(github.com/topolski852/humanoid-esc-firmware). A working tree is also at
`/home/nse/Recoil-Motor-Controller-BESC/…`. Current version: **`0x03020000` = v3.2.0**
(`FIRMWARE_VERSION` in `Core/Inc/motor_controller_conf.h`).

### Motor modes (firmware `MotorMode` enum)

| Mode | Value | Notes |
|---|---|---|
| `MODE_DISABLED` | `0x00` | PWM off, motor coasts. **Boot default** (v3.2.0 boots DISABLED, silent — no heartbeat, no PDO4, watchdog off). Must be woken to IDLE before it will accept SDO writes or mode changes. |
| `MODE_IDLE` | `0x01` | PWM enabled, zero torque, safe. Accepts SDO. |
| `MODE_DAMPING` | `0x02` | Regenerative braking. The watchdog timeout drops the motor here. |
| `MODE_CALIBRATION` | `0x05` | Runs the autonomous encoder flux-offset sweep. |
| `MODE_CURRENT` | `0x10` | Direct d/q current command. |
| `MODE_TORQUE` | `0x11` | Direct torque command. |
| `MODE_VELOCITY` | `0x12` | Velocity control. |
| `MODE_POSITION` | `0x13` | Position control (the mode a policy runner uses). |
| `MODE_VABC/VALPHABETA/VQD_OVERRIDE` | `0x20/0x21/0x22` | Low-level voltage overrides (debug). |
| `MODE_DEBUG` | `0x80` | Debug. |

**Verified from:** `daemon/src/motor/recoil_protocol.hpp` `MotorMode`, mirrored in
`daemon_client.py` `Mode` and firmware `motor_controller.c`.

### Position control law (MODE_POSITION)

Runs in the firmware's 2 kHz position controller (`Core/Src/position_controller.c`,
current conversion in `Core/Src/motor_controller.c`):

```
position_setpoint = clamp(position_target, position_limit_lower, position_limit_upper)
position_error    = position_setpoint - position_measured
velocity_error    = 0 - velocity_measured          # NOTE: target velocity is always 0

position_integrator = clamp(position_integrator + position_ki*position_error,
                            -torque_limit, +torque_limit)

torque_target = position_kp * position_error
              + velocity_kp * velocity_error        # velocity_kp acts as Kd
              + position_integrator
              + torque_target_ff                    # SDO feed-forward torque (usually 0)

# EMA filter, then clamp to torque_limit:
torque_setpoint = clamp( ema(torque_target, alpha=torque_filter_alpha),
                         -torque_limit, +torque_limit )

# Convert torque -> quadrature current for the inner FOC loop:
i_q_target = torque_setpoint / torque_constant / gear_ratio
i_d_target = 0
```

An inner **10 kHz FOC current loop** (`current_controller.c`) then drives `i_q_target`.
`position_measured`/`velocity_measured` are already divided by `gear_ratio`, so they are
**output-shaft** (post-gearbox) radians and rad/s.

> **Two facts that differ from older docs, verified in source:**
> - The velocity target in POSITION mode is hard-wired to **0**. `velocity_kp` is a pure
>   damping (Kd) term. The `velocity_ff` field in PDO2 (bytes 4–7) is **ignored** in POSITION
>   mode — do not rely on it.
> - `i_q_target` divides by the **signed** `gear_ratio` (not `|gear_ratio|`). Commutation
>   phase-swap already makes `+i_q` produce `+encoder` torque regardless of `phase_order`, so
>   the closed-loop sign is independent of `phase_order`. (A historical bug multiplied by
>   `phase_order` here and caused POSITION-mode runaway; removed as of v3.1.2.)

### Error codes (`ErrorCode` bitmask)

Bitwise-OR of any of: `NO_ERROR 0x0000`, `GENERAL 0x0001`, `ESTOP 0x0002`,
`INITIALIZATION_ERROR 0x0004`, `CALIBRATION_ERROR 0x0008`, `POWERSTAGE_ERROR 0x0010`,
`INVALID_MODE 0x0020`, `WATCHDOG_TIMEOUT 0x0040`, `OVER_VOLTAGE 0x0080`,
`OVER_CURRENT 0x0100`, `OVER_TEMPERATURE 0x0200`, `CAN_RX_FAULT 0x0400`,
`CAN_TX_FAULT 0x0800`, `I2C_FAULT 0x1000`, `ENCODER_FAULT 0x2000`.
Clear by writing `0` to the `ERROR` register (SDO param `0x014`), or via the daemon
`CLEAR_ERROR` command. **Verified from:** `recoil_protocol.hpp` `ErrorCode`.

### Firmware version encoding

`uint32` packed **`0xMMmmPPrr`** (major, minor, patch, reserved). e.g. `0x03020000` → `v3.2.0`.
The daemon decodes with `(fw>>24, fw>>16, fw>>8) & 0xFF`. Read via SDO param
`PARAM_FIRMWARE_VERSION 0x004`. **Verified from:** `robot.cpp firmware_version_json()`,
`motor_controller_conf.h`.

### Watchdog → DAMPING

The firmware runs a safety watchdog timer (`htim2`, autoreload = `watchdog_timeout*10 − 1`;
`watchdog_timeout` = **1000 ms** by default). Any of `FUNC_RECEIVE_PDO_1/2/3` (0x4/0x6/0x8)
**or** `FUNC_HEARTBEAT` (0xE) resets the counter. If nothing arrives within the timeout **and
the motor is in a motion mode**, the firmware drops to `MODE_DAMPING` and sets
`ERROR_WATCHDOG_TIMEOUT`. The watchdog does **not** fire in IDLE/DISABLED/DAMPING.
**Verified from:** `motor_controller.c` (htim2 setup + PDO/heartbeat `__HAL_TIM_SET_COUNTER`),
`app.c` (`MODE_DAMPING` + `ERROR_WATCHDOG_TIMEOUT` on timeout).

> When a joint is ENABLED, the daemon's 200 Hz tick sends a PDO2 every cycle, which feeds the
> firmware watchdog for you. You never feed it manually.

---

## 3. CAN protocol

- **11-bit standard CAN ID:** `arb_id = (func_id << 7) | device_id`
  (`device_id` = bits [6:0], 7 bits, range 1–63; `func_id` = bits [10:7], 4 bits).
- **Bitrate:** 1 Mbit on every bus.
- **All multi-byte payloads are little-endian**; floats are IEEE-754 `float32`. There is **no**
  fixed-point encoding anywhere in the live protocol.

**Verified from:** `recoil_protocol.hpp` (`make_arb_id`, packed structs + `static_assert`s),
`daemon_client.py`, firmware `motor_controller.c`.

### Function codes (`FuncCode`)

| Value | Name | Dir | Payload |
|---|---|---|---|
| 0x0 | `NMT` | →node | `[mode u8, addressed_device_id u8]` (id 0 = broadcast) |
| 0x1 | `SYNC_EMCY` | node→ | EMCY fault broadcast: `[error_code u32]` |
| 0x2 | `TIME` | — | unused |
| 0x3 | `TRANSMIT_PDO_1` | node→ | ping echo |
| 0x4 | `RECEIVE_PDO_1` | →node | ping request (`0xCA` magic byte); feeds watchdog |
| 0x5 | `TRANSMIT_PDO_2` | node→ | `[position_measured f32, velocity_measured f32]` |
| 0x6 | `RECEIVE_PDO_2` | →node | `[position_target f32, velocity_ff f32]`; feeds watchdog |
| 0x7 | `TRANSMIT_PDO_3` | node→ | calibration status frames |
| 0x8 | `RECEIVE_PDO_3` | →node | `[position_target f32, torque_target f32]`; feeds watchdog |
| 0x9 | `TRANSMIT_PDO_4` | node→ | **autonomous** feedback `[position f32, velocity f32]` at `fast_frame_frequency` Hz |
| 0xA | `RECEIVE_PDO_4` | →node | no-op |
| 0xB | `TRANSMIT_SDO` | node→ | SDO response (8-byte CANopen upload response) |
| 0xC | `RECEIVE_SDO` | →node | SDO read/write request |
| 0xD | `FLASH` | →node | `[op u8]` (1 = store, 2 = load) |
| 0xE | `HEARTBEAT` | both | node broadcasts `[mode u8, error u32]`; also feeds watchdog when received by node |

### PDO2 — the real-time position frame

**TX (host → node), `RECEIVE_PDO_2`, arb `(0x6<<7)|id`, DLC 8:**
```
data[0:4]  float32 LE  position_target   (output-side rad; firmware adds position_offset)
data[4:8]  float32 LE  velocity_ff       (rad/s; IGNORED in POSITION mode)
```
**RX (node → host), `TRANSMIT_PDO_2`, arb `(0x5<<7)|id`, DLC 8:**
```
data[0:4]  float32 LE  position_measured (output-side rad)
data[4:8]  float32 LE  velocity_measured (output-side rad/s)
```

### PDO4 — autonomous feedback

`TRANSMIT_PDO_4`, arb `(0x9<<7)|id`, DLC 8, same layout as PDO2 RX
(`[position f32, velocity f32]`). The node emits it on its own timer at
`fast_frame_frequency` Hz (config: **100** for all joints; `0` disables it). This is the
daemon's primary position feedback and liveness signal.

### SDO read / write (`RECEIVE_SDO` 0xC, arb `(0xC<<7)|id`, DLC 8)

Request layout `[cmd u8, param_id u16 LE, pad u8, value[4]]`:
- **Write (download):** `cmd = 0x20`, `value` = float32/uint32/int32 LE. ACK: response byte0 `0x60`.
- **Read (upload):** `cmd = 0x40`, `value` = zeros. Response (`TRANSMIT_SDO` 0xB,
  `(0xB<<7)|id`, DLC 8): `[0x43, param_id u16 LE, pad, value[4] LE]`.

`param_id` is the byte offset into the firmware `MotorController` struct — fully enumerated in
`recoil_protocol.hpp` `ParamId` (and `backend/humanoid/can_bus.py` `Parameter`). Key ones:
`ERROR 0x014`, `MODE 0x010`, `GEAR_RATIO 0x01C`, `POSITION_KP 0x020`, `VELOCITY_KP 0x028`,
`TORQUE_LIMIT 0x030`, `POSITION_OFFSET 0x040`, `TORQUE_CONSTANT 0x108`, `PHASE_ORDER 0x10C`,
`ENCODER_FLUX_OFFSET 0x13C`, `BUS_VOLTAGE_MEASURED 0x100`, `I_Q_MEASURED 0x0C0`.

### NMT, FLASH, HEARTBEAT, EMCY

- **NMT** (`0x0`, arb `(0x0<<7)|id` or `0x000` broadcast, DLC 2): `[mode u8, device_id u8]`.
  Preferred way to change mode.
- **FLASH** (`0xD`, DLC 1): `[1]` = store RAM config → Flash page 63 (`0x0801F800`);
  `[2]` = load Flash → RAM.
- **HEARTBEAT** (`0xE`): node → host `[mode u8, error u32]` (5 bytes) as a mode-change ACK and
  periodic liveness; host → node (0 or with data) feeds the watchdog. Do **not** transmit on
  `(0xE<<7)|id` toward an IDLE motor — that arb is the motor's own broadcast ID and collides.
- **EMCY** (`0x1`): node → host `[error_code u32]` fault broadcast.

---

## 4. Joint model (22 joints, 4 CAN buses)

Joints are addressed by their **globally-unique `joint_name`**. Device IDs are **only unique
per bus** (e.g. `can_left_leg id=1` and `can_left_arm id=1` are different joints), so never key
control code on `can_id` alone.

Canonical bus names (from `daemon_client.py _CANONICAL_BUSES`): `can_left_leg`,
`can_right_leg`, `can_left_arm`, `can_right_arm`.

**Mixed motor set — the leg's 8 big joints (hip roll/yaw/pitch + knee) use a 150 KV motor
(`MAD_M6C12`, Kt ≈ 0.08958 Nm/A); the 14 ankle + arm joints use a 200 KV motor
(`MAD_5010`, Kt ≈ 0.06588 Nm/A).** Verified per-joint from `configs/humanoid_lite.json`
`torque_constant` (arm joints store the rounded value `0.0659`).

| Bus | id | joint_name | gear_ratio | phase_inverted | Kt (Nm/A) | motor |
|---|---|---|---|---|---|---|
| can_left_leg | 1 | left_hip_roll_joint | +15.0 | true | 0.08958 | 150KV M6C12 |
| can_left_leg | 3 | left_hip_yaw_joint | −15.0 | true | 0.08958 | 150KV M6C12 |
| can_left_leg | 5 | left_hip_pitch_joint | −15.0 | true | 0.08958 | 150KV M6C12 |
| can_left_leg | 7 | left_knee_pitch_joint | +15.0 | true | 0.08958 | 150KV M6C12 |
| can_left_leg | 11 | left_ankle_pitch_joint | +15.0 | true | 0.06588 | 200KV 5010 |
| can_left_leg | 13 | left_ankle_roll_joint | +15.0 | true | 0.06588 | 200KV 5010 |
| can_right_leg | 2 | right_hip_roll_joint | −15.0 | true | 0.08958 | 150KV M6C12 |
| can_right_leg | 4 | right_hip_yaw_joint | +15.0 | true | 0.08958 | 150KV M6C12 |
| can_right_leg | 6 | right_hip_pitch_joint | +15.0 | true | 0.08958 | 150KV M6C12 |
| can_right_leg | 8 | right_knee_pitch_joint | −15.0 | true | 0.08958 | 150KV M6C12 |
| can_right_leg | 12 | right_ankle_pitch_joint | −15.0 | true | 0.06588 | 200KV 5010 |
| can_right_leg | 14 | right_ankle_roll_joint | +15.0 | **false** | 0.06588 | 200KV 5010 |
| can_left_arm | 1 | left_shoulder_pitch_joint | −15.0 | true | 0.0659 | 200KV 5010 |
| can_left_arm | 3 | left_shoulder_roll_joint | −15.0 | true | 0.0659 | 200KV 5010 |
| can_left_arm | 5 | left_shoulder_yaw_joint | −15.0 | true | 0.0659 | 200KV 5010 |
| can_left_arm | 7 | left_elbow_pitch_joint | −15.0 | true | 0.0659 | 200KV 5010 |
| can_left_arm | 9 | left_wrist_yaw_joint | −15.0 | true | 0.0659 | 200KV 5010 |
| can_right_arm | 2 | right_shoulder_pitch_joint | −15.0 | true | 0.0659 | 200KV 5010 |
| can_right_arm | 4 | right_shoulder_roll_joint | −15.0 | true | 0.0659 | 200KV 5010 |
| can_right_arm | 6 | right_shoulder_yaw_joint | −15.0 | true | 0.0659 | 200KV 5010 |
| can_right_arm | 8 | right_elbow_pitch_joint | −15.0 | **false** | 0.0659 | 200KV 5010 |
| can_right_arm | 10 | right_wrist_yaw_joint | −15.0 | true | 0.0659 | 200KV 5010 |

> `phase_inverted` is `true` on 20 of 22 joints; only `right_ankle_roll_joint` and
> `right_elbow_pitch_joint` are `false`. These flags are per-motor commutation calibration,
> **not** a control convention — do not "correct" them in a policy runner.

### Direction & frame conventions

- **`gear_ratio` (signed):** the firmware divides `position/velocity_measured` by it, so a
  negative `gear_ratio` flips the sign of the output-side position/velocity you read and
  command. It aligns a joint's positive direction with the URDF/policy convention. Magnitude is
  15.0 for every joint.
- **`phase_inverted` (bool):** maps to firmware `phase_order` (`+1` if false, `−1` if true).
  Purely commutation/direction of the FOC phase wiring; set during flashing/calibration.
- **Frame stack** for a commanded/reported angle:
  - **wire frame** = raw firmware `position_measured` (gearbox-divided, offset-subtracted).
  - **display frame** = what the daemon reports in telemetry and accepts in `SET_POSITION`
    (`wire − position_offset` on read; the firmware adds `position_offset` back on a PDO2
    command). This is your policy-facing frame.
  - `position_offset` (rad) is the zero-point calibration; `encoder_position_offset` and
    `electrical_offset` (flux offset) are separate encoder calibrations (see §6.5).

---

## 5. `JointConfig` schema

Source of truth: `backend/humanoid/robot_config.py` (`JointConfig`, `PositionLimits`).
Serialized in `configs/humanoid_lite.json`. Field → default → unit:

**Identity**
| Field | Default | Notes |
|---|---|---|
| `joint_name` | (required) | globally unique key |
| `joint_type` | `"revolute"` | |
| `can_channel` | `"can0"` | one of the 4 canonical bus names in the real config |
| `can_id` | (required, 1–63) | unique per bus only |

**Direction & calibration**
| Field | Default | Unit / notes |
|---|---|---|
| `phase_inverted` | `false` | → firmware `phase_order` (+1/−1) |
| `electrical_offset` | `0.0` | encoder **flux_offset**, rad (large range OK) |
| `gear_ratio` | `1.0` | signed; negative flips output direction |
| `position_limits` | `{min:null, max:null}` | `PositionLimits`; `null` = no limit |

`PositionLimits.min/max` are rad or `null`. `null` serializes as JSON `null` and maps to a
`±100.0 rad` sentinel (`_NO_LIMIT_RAD`) when sent to firmware (±inf would give NaN in the
firmware's `(upper+lower)/2`).

**Position controller**
| Field | Default | Unit |
|---|---|---|
| `position_kp` | `20.0` | Kp |
| `position_ki` | `0.0` | Ki |
| `velocity_kp` | `1.0` | acts as **Kd** in POSITION mode |
| `velocity_ki` | `0.0` | |
| `torque_limit` | `2.0` | Nm (clamps the output torque) |
| `velocity_limit` | `20.0` | rad/s (used in VELOCITY mode) |
| `position_offset` | `0.0` | rad (zero-point) |
| `torque_filter_alpha` | `0.1454` | EMA α (~50 Hz at 2 kHz) |

**Current controller**
| Field | Default | Unit |
|---|---|---|
| `current_limit` | `20.0` | A |
| `current_kp` | `0.1664` | V/A |
| `current_ki` | `5746.5` | 1/s |

**Power stage**
| Field | Default | Unit |
|---|---|---|
| `undervoltage_threshold` | `0.0` | V (0 = disabled) |
| `overvoltage_threshold` | `0.0` | V (0 = disabled) |
| `bus_voltage_filter_alpha` | `0.2696` | EMA α |

**Motor**
| Field | Default | Unit |
|---|---|---|
| `pole_pairs` | `14` | |
| `torque_constant` | `0.0659` | Nm/A (see §4 for the real per-joint mix) |
| `max_calibration_current` | `3.0` | A |

**Encoder (AS5600)**
| Field | Default | Unit |
|---|---|---|
| `cpr` | `4096` | 12-bit |
| `encoder_position_offset` | `0.0` | rad |
| `velocity_filter_alpha` | `0.7154` | EMA α (~2 kHz at 10 kHz) |

**System**
| Field | Default | Unit |
|---|---|---|
| `watchdog_timeout` | `1000` | ms |
| `fast_frame_frequency` | `0` | Hz (PDO4 rate; **100** in the live config) |

Derived (not persisted): `phase_order` → `−1 if phase_inverted else +1`.
`RobotConfig` adds `robot_name`, `telemetry_hz` (default 10; **100** live), `can_assignments`
(USB serial → limb label), and `joints: dict[str, JointConfig]`. The daemon's own
`JointConfig` (`daemon/src/config/config_loader.hpp`) mirrors these fields (note it names the
watchdog field `watchdog_timeout_ms`).

---

## 6. How to write control code

### 6.1 DaemonClient command set

All verified against `backend/humanoid/daemon_client.py`. Each row is a `DaemonClient` method
and the wire-level command `type` it sends (the daemon replies `ACK`/`ERROR` or a typed
response). Commands are UDP JSON with an added `"id"` for request/response matching.

| DaemonClient method | Wire `type` | Purpose |
|---|---|---|
| `ping()` | `PING` → `PONG` | liveness / daemon version |
| `get_state(joint_name)` | `GET_STATE` | fresh state for one joint (`resp["state"]`) |
| `get_all_states_raw()` | `GET_ALL_STATES` | all joints (`resp["states"]`) |
| `get_cached_joint_state(name)` | — (reads telemetry cache) | no round-trip |
| `set_mode(name, mode)` | `SET_MODE` | `mode` is a string: `"POSITION"`, `"IDLE"`, `"DISABLED"`, … |
| `set_all_mode(mode)` | `SET_ALL_MODE` | all joints |
| `set_position(name, position_rad)` | `SET_POSITION` | display-frame rad target |
| `clear_error(name)` | `CLEAR_ERROR` | zero the error register |
| `apply_config(name, config, timeout)` | `APPLY_CONFIG` | write one joint's params to device RAM |
| `apply_all_configs()` | `APPLY_ALL_CONFIGS` | write every joint; this is "Connect" |
| `write_gains(name, kp, ki, velocity_kp, torque_limit)` | `WRITE_GAINS` | fast (~4 SDOs) gain retune |
| `store_joint_to_flash(name)` | `STORE_TO_FLASH` | persist RAM → Flash |
| `read_device_config(name)` | `READ_CONFIG` → `CONFIG` | read back all params |
| `calibrate_device(chan, id, timeout_ms)` | `CALIBRATE_DEVICE` | run flux-offset calibration |
| `estop_all()` | `ESTOP` (port **9002**) | priority stop → all joints to IDLE |
| `daemon_shutdown()` | `SHUTDOWN` | stop the daemon |
| generic passthrough | `GENERIC_SDO_WRITE`, `GENERIC_SDO_READ`, `SEND_NMT`, `SEND_FLASH_STORE`, `FEED_WATCHDOG`, `WAIT_HEARTBEAT`, `SNIFF_BUS` | flash-wizard / commissioning; not needed by a policy runner |
| slow-poll toggles | `DISABLE_SLOW_POLL`, `ENABLE_SLOW_POLL`, `DISABLE_ALL_SLOW_POLL`, `ENABLE_ALL_SLOW_POLL` | mute SDO telemetry during commissioning |

**Per-joint proxy** (`DaemonActuatorProxy`, from `client.get_actuator_by_name(name)`):
`await set_position(pos, ...)`, `await enable(mode=Mode.POSITION)` (→ `SET_MODE POSITION`),
`await disable()` (→ `SET_MODE IDLE`), `await estop()`, `await get_state()`,
`get_cached_state()` (no round-trip), `await clear_error()`, `await write_gains(...)`,
`await apply_config()`, `await calibrate_offset(...)`, `await store_to_flash()`.
`feed_watchdog()` is a **no-op** — the daemon feeds watchdogs from its 200 Hz loop.

### 6.2 Telemetry frame (pushed on :9000)

```jsonc
{
  "type": "TELEMETRY",
  "seq": <uint>,
  "timestamp_us": <uint>,
  "joints": {
    "left_hip_roll_joint": {
      "state": "OFFLINE"|"DISABLED"|"IDLE"|"ENABLED"|"CALIBRATING"|"FAULT",
      "position": <float>,          // display-frame rad
      "velocity": <float>,          // rad/s
      "torque":   <float>,          // Nm (estimated)
      "current":  <float>,          // A (Iq)
      "mode":     <int>,            // firmware MotorMode
      "error":    <uint>,           // firmware error bitmask
      "bus_voltage": <float|null>,
      "firmware_version": "v3.2.0"|null
    }, ...
  },
  "bus_health": { "can_left_leg": {"open":bool,"tx_dropped":int,"rx_frames":int}, ... }
}
```
**Verified from:** `robot.cpp build_telemetry_json()`. A `GET_STATE` response uses key
`"joint_state"` instead of `"state"` for the same fields (see `_daemon_state_to_actuator`).

### 6.3 Minimal position-control loop (Python, via DaemonClient)

```python
import asyncio, time
from humanoid.daemon_client import DaemonClient   # Mode enum also here if needed
from humanoid.robot_config import RobotConfig

async def main():
    cfg = RobotConfig.from_json("configs/humanoid_lite.json")
    client = DaemonClient(cfg)          # daemon must already be running
    await client.start()                # opens UDP sockets, starts telemetry thread

    # 1. Connect: wake every motor DISABLED->IDLE and push its config to device RAM.
    await client.connect()              # == apply_all_configs()

    # 2. Enable the joints you will drive (POSITION mode).
    leg = [n for n in cfg.joint_names() if "leg" in cfg.joints[n].can_channel]
    for name in leg:
        client.set_mode(name, "POSITION")

    # 3. Stream targets. The daemon's 200 Hz loop paces PDO2 to the ESCs; you can push
    #    targets at your policy rate (e.g. 50-100 Hz).
    try:
        while True:
            for name in leg:
                st = client.get_cached_joint_state(name)   # no round-trip
                if st is None:
                    continue                                # joint offline
                target = st["position"]                    # replace with policy output (rad)
                client.set_position(name, target)          # display-frame rad
            await asyncio.sleep(1/100)                      # 100 Hz policy tick
    finally:
        for name in leg:
            client.set_mode(name, "IDLE")                  # or client.estop_all()
        await client.stop()

asyncio.run(main())
```

Notes:
- `set_*` / `get_state` are **blocking** UDP calls; in a hot async loop prefer
  `get_cached_joint_state()` (reads the telemetry cache) and wrap blocking calls in
  `run_in_executor` if you need them.
- You do not feed watchdogs; the daemon does while a joint is ENABLED.
- To stop: `set_mode(..., "IDLE")` for a clean stop, or `estop_all()` for the priority path.

### 6.4 Actuator state machine (daemon-side)

`JointState` (daemon, `actuator.hpp`), distinct from the firmware `MotorMode`:

```
OFFLINE ──(PDO4/heartbeat seen)──► IDLE ──(SET_MODE POSITION)──► ENABLED
   ▲                                 │                              │
   │  no PDO4/heartbeat > 1500 ms    │  SET_MODE IDLE               │ SET_MODE IDLE / ESTOP
   └─────────────────────────────────┴──────────────────────────────┘
 DISABLED  (NMT MODE_DISABLED; silent, OFFLINE timer suppressed — used by disconnect())
 CALIBRATING  (IDLE → CALIBRATE_DEVICE)      FAULT  (firmware error; CLEAR_ERROR → IDLE)
```

- A motor boots DISABLED (silent). The daemon marks it OFFLINE until it hears a PDO4 or
  heartbeat, then wakes it to IDLE (NMT IDLE) so SDO writes/reads work.
- **`apply_all_configs()` / `connect()` is the required "wake + configure" step** before
  enabling. Firmware silently drops SDO writes while DISABLED.
- ENABLED sends a PDO2 every 200 Hz tick (position hold or your last `SET_POSITION`).
- OFFLINE detection: no PDO4 **or** heartbeat for **1500 ms** (daemon side). The **firmware**
  watchdog (→ DAMPING) is separate, at **1000 ms** of no PDO/heartbeat in a motion mode.

### 6.5 Calibration workflow (three distinct offsets — don't conflate them)

1. **`electrical_offset` (flux_offset, SDO `0x13C`):** the FOC commutation offset. Measured by
   the autonomous firmware sweep (`MODE_CALIBRATION`). Trigger with
   `calibrate_device(chan, id, timeout_ms)` (or proxy `calibrate_offset()`), then read back
   `0x13C`. Persist with `store_to_flash`. Required for any torque production.
2. **`gear_ratio` direction (signed, SDO `0x01C`):** sets output-shaft sign. Chosen during
   flashing/commissioning (Flash Wizard "direction confirm") so `+position` matches the URDF.
   Not measured — a config choice per joint (all `±15.0`).
3. **`position_offset` (SDO `0x040`):** the joint zero-point (display frame). Set by jogging to
   a known pose and capturing the angle. Independent from `encoder_position_offset` (`0x124`),
   which is a raw-encoder zero.

A policy runner should treat all three as already-calibrated inputs from
`configs/humanoid_lite.json`; it applies them via `apply_all_configs()` (`connect`) and never
re-derives them at runtime.

---

## 7. PLANNED — IMU integration (contract)

**Status: not yet implemented.** The robot currently has **no IMU**, and there are no IMU
fields anywhere in firmware, daemon, or backend today (verified: no `imu`/`quaternion`/
`projected_gravity`/`base` references in `daemon/src` or `backend/humanoid`).

The IMU will be added **at the daemon level** — an external serial/USB sensor read by the
daemon. **No ESC-firmware change.** So control/trainer code should target the daemon telemetry,
not CAN.

**Intended telemetry addition** (design so your code can target it now): the `TELEMETRY` frame
(§6.2) gains a top-level `"base"` block:

```jsonc
"base": {
  "quaternion":        [w, x, y, z],   // base orientation (order/convention TBD at impl)
  "angular_velocity":  [wx, wy, wz],   // rad/s, base frame
  "projected_gravity": [gx, gy, gz]    // gravity unit vector rotated into the base frame
}
```

`projected_gravity` is the standard learned-locomotion observation (gravity direction expressed
in the base frame; ≈ `[0,0,-1]` when upright). **TODO at implementation time:** confirm the
exact quaternion order/handedness, units, sensor mounting frame, and update rate — treat the
field names above as the stable contract and the numeric conventions as provisional.

---

## 8. Appendix — legacy reference (historical)

This project began by porting the original Berkeley Humanoid Lite Python scripts
(`Berkeley-Humanoid-Lite/source/berkeley_humanoid_lite_lowlevel/`). Those scripts are
**historical** and are **not** how the system works today — the C++ daemon replaced the
direct-CAN Python bus. Do not model new control code on them. What still matters:

- The original `robot_configuration.json` used a **nested** schema and emitted non-standard
  JSON `Infinity`/`-Infinity`; the current schema is **flat** (`JointConfig`, §5) and uses
  `null` for "no limit".
- The original scripts opened SocketCAN directly and ran a synchronous 200 Hz `move_actuator`
  loop with no daemon. Today all CAN access goes through the daemon; direct-CAN Python
  (`backend/humanoid/can_bus.py`, `actuator.py`, `robot.py`) still exists but is used **only**
  for offline flashing/commissioning when the daemon is stopped. `DaemonActuatorProxy` raises
  `DaemonNotSupportedError` for operations that need the raw bus (e.g. `load_from_flash`).
- The old critique of individual buggy scripts (`write_configurations.py` one-joint bug, dead
  `fixed16.py`, SDO race, etc.) is obsolete — those files are not in the control path.

---

## 9. Next steps

- **`humanoid-control` (new repo, to create):** the learned-policy runner — start with a
  **legs-only stand-up policy**. Plug in at the daemon UDP boundary per §6. (The repo does not
  exist in this tree yet as of this writing.)
- **IMU work (§7):** add the external serial/USB IMU at the daemon level and emit the `base`
  telemetry block. Confirm quaternion/gravity conventions when implementing.
- **Firmware:** source of record is **`humanoid-esc-firmware`**
  (github.com/topolski852/humanoid-esc-firmware). Current firmware is **v3.2.0** (boots
  DISABLED, watchdog → DAMPING, POSITION-mode sign fixes described in §2).
```
