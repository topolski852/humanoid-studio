# Humanoid Studio — C++ CAN Daemon Specification (AS-BUILT)

**Status: SHIPPED / IMPLEMENTED.** The daemon described here is fully implemented,
built, and running in production. This document is the authoritative protocol/API
reference for control code that talks to the daemon. All constants, ports, command
names, fields, and enums below are verified against the daemon source under
`daemon/src/` and the Python client `backend/humanoid/daemon_client.py`.

> Source of truth (verify here before changing this doc):
> - `daemon/src/main.cpp` — CLI parsing, signal handling, startup
> - `daemon/src/control/robot.{hpp,cpp}` — command dispatch, telemetry JSON, threads
> - `daemon/src/control/control_loop.hpp` — real-time loop utility
> - `daemon/src/motor/actuator.{hpp,cpp}` — per-joint state machine, SDO, tick
> - `daemon/src/motor/recoil_protocol.hpp` — CAN constants, SDO map, enums, frames
> - `daemon/src/config/config_loader.hpp` — JointConfig / RobotConfig structs
> - `daemon/src/can/*` — SocketCAN wrapper, bus manager, generic listener
> - `daemon/src/ipc/*` — UDP server / broadcaster
> - `daemon/Makefile` — build (GNU make + g++, C++17; no CMake)
> - `backend/humanoid/daemon_client.py` — Python client / protocol mirror

---

## 1. Purpose & Design Rationale (shipped design)

The Python `python-can` transport layer was replaced by a standalone C++ daemon
process that owns all four SocketCAN interfaces. The FastAPI Python layer is a thin
proxy: it talks to the daemon exclusively via JSON-over-UDP on localhost. Python
never opens a CAN socket during normal operation (the Flash Wizard, which uses
OpenOCD/SWD rather than CAN, is the only exception; live CAN commissioning is routed
through the daemon's generic passthrough commands).

Why the daemon owns CAN:

- **No GIL on the CAN hot path.** The control loop runs at 200 Hz in C++ with
  optional `SCHED_FIFO` real-time scheduling and CPU affinity, independent of the
  Python web server.
- **Multi-bus from one process.** One `CanBusManager` owns up to four CAN interfaces
  and fans frames in/out per bus; a stopped/absent interface is non-fatal.
- **Crash isolation.** The control layer and web layer restart and upgrade
  independently; the daemon can be shut down (e.g. by the Flash Wizard) and respawned.
- **Foundation for a direct control/policy UDP interface** that bypasses the web
  layer (see §9, PLANNED IMU extension, for the next contract addition).

---

## 2. CLI Flags & Startup

Binary: `daemon/build/humanoid_daemon` (built by `daemon/Makefile`).

```
Usage: humanoid_daemon [OPTIONS]
  --config PATH      JSON config file (default: ../configs/humanoid_lite.json)
  --cmd-port PORT    UDP command port           (default: 9001)
  --tel-port PORT    UDP telemetry port          (default: 9000)
  --tel-hz HZ        Telemetry rate 1-100 Hz     (default: 10)
  --rt-prio PRIO     SCHED_FIFO priority 0-99    (default: 80; 0 = SCHED_OTHER)
  --cpu CPU          CPU affinity for control loop (default: 0)
  --help             Show usage
```

Verified flag/default facts (from `main.cpp` and `robot.hpp RobotOptions`):

- Default command port **9001**, telemetry port **9000**.
- Priority E-Stop port **9002** is **hard-coded** (not configurable), see §5.4.
- `--tel-hz` default is **10**, but `main.cpp` overrides it from the config file's
  `telemetry_hz` *only when the CLI value is still the default 10*. The shipped
  `configs/humanoid_lite.json` sets `telemetry_hz: 100`, so the daemon publishes at
  **100 Hz** unless `--tel-hz` is passed a non-10 value. (The telemetry thread also
  independently re-reads `cfg_.telemetry_hz` and clamps to 1–100, preferring the
  config value when in range — so config `telemetry_hz` is authoritative for the push
  rate.)
- `--rt-prio` maps to the control-loop `SCHED_FIFO` priority; `0` selects
  `SCHED_OTHER`. Real-time scheduling requires `cap_sys_nice` (or root); on failure
  the loop logs a warning and continues at normal priority.
- Unknown args print usage and exit 1. `--help`/`-h` prints usage and exits 0.

**Startup sequence** (`main.cpp` → `Robot::start()`):
1. Parse args, print the effective config/ports/hz/prio/cpu to stderr.
2. `load_config(path)` → `RobotConfig` (fail-fast, exit 1 on error).
3. Install `SIGINT`/`SIGTERM` handlers.
4. Construct `Robot`; open one CAN socket per unique `can_channel` (missing
   interfaces logged, non-fatal).
5. Build one `Actuator` per joint.
6. Start UDP command server (9001) and priority E-Stop server (9002).
7. Start telemetry thread and the 200 Hz control loop.
8. Block on `pause()` until the first signal, then `Robot::stop()`.

**Signals:** first `SIGINT`/`SIGTERM` requests graceful shutdown; a **second** signal
calls `_exit(1)` immediately (async-signal-safe force-exit). There is no timed
2-second force window — it is strictly "second signal = force exit."

**Build:** `cd daemon && make` (or `make -j`). Uses `g++ -std=c++17 -O2 -pthread`,
`nlohmann/json` vendored at `daemon/third_party/json.hpp`. No CMake, no FetchContent,
no external package dependencies. Grant real-time scheduling without root via:
`sudo setcap cap_sys_nice+ep build/humanoid_daemon`.

---

## 3. Control-Loop Timing (verified)

Implemented by `ControlLoop` (`control/control_loop.hpp`), started from
`Robot::start()` with:

- **Period:** `1.0/200.0` s → **200 Hz** (5 ms budget).
- **Scheduling:** `SCHED_FIFO` at `--rt-prio` (default **80**) via
  `pthread_setschedparam`; falls back to `SCHED_OTHER` if not permitted.
- **CPU affinity:** `--cpu` (default **0**) via `pthread_setaffinity_np`.
- **Overrun handling:** timing uses `steady_clock`. If a tick overruns by more than
  **50% of the period** (> 2.5 ms), an overrun counter increments and a message is
  logged to stderr; the loop re-bases `next_wake` to `now + period` to catch up.
  Overruns under 50% are absorbed silently.

**Per-tick sequence** (`Robot::control_tick()`), in order:

1. **E-Stop check first.** If `estop_pending_` is set (from the priority port),
   send `NMT MODE_IDLE` to *every* actuator and `request_state(IDLE)` for each, then
   clear the flag.
2. **Drain all buses** (`bus_mgr_->drain_all`, capped at **64 frames/tick**).
   For each frame: match `device_id` + `can_channel` to an `Actuator` and call
   `on_rx_frame()`; also feed the `GenericListener` (for Flash-Wizard passthrough
   futures/sniffs).
3. **Tick each actuator** (`Actuator::tick`): apply any pending state change (send
   NMT), and for `ENABLED` joints send one PDO2 position command. `IDLE`/`OFFLINE`/
   `CALIBRATING`/`FAULT`/`DISABLED` emit no motion frames.

**Watchdog note (important, differs from earlier plans):** the daemon does **not**
send `FUNC_HEARTBEAT` frames to IDLE joints. `FUNC_HEARTBEAT` (`0xE`, arb
`0x700+device_id`) is the *motor's own outgoing* broadcast arb-id; sending on it from
the host causes bus collisions. The firmware watchdog only fires in motion modes, so
IDLE joints need no feed. When a "feed" is required for a specific device (Flash
Wizard), the daemon sends `NMT MODE_IDLE` on arb `0x000+id` instead (see
`FEED_WATCHDOG`).

**Slow-poll telemetry:** inside `tick()`, for `ENABLED`/`IDLE` joints the daemon
round-robins one SDO read per phase across a 60-tick window (~3.3 Hz each at 200 Hz):
bus voltage (`0x100`), i_q measured (`0xC0`), torque measured (`0x048`). These update
`bus_voltage`/`current`/`torque` in the telemetry snapshot. Disabled per-joint or
globally by the `*_SLOW_POLL` commands during commissioning.

---

## 4. CAN Protocol Reference (`motor/recoil_protocol.hpp`)

### 4.1 CAN ID layout

```
arb_id = (func_id << 7) | device_id
device_id = arb_id & 0x7F          (7 bits)
func_id   = (arb_id >> 7) & 0xF    (4 bits)
```

### 4.2 Function codes (`FuncCode`)

| Name | Code | Direction / meaning |
|---|---|---|
| `FUNC_NMT` | `0x0` | host→node: mode change (2 bytes: mode, device_id) |
| `FUNC_SYNC_EMCY` | `0x1` | node→host: EMCY error broadcast (4 bytes) |
| `FUNC_TIME` | `0x2` | unused |
| `FUNC_TRANSMIT_PDO_1` | `0x3` | node→host: ping echo |
| `FUNC_RECEIVE_PDO_1` | `0x4` | host→node: ping request (magic `0xCA`) |
| `FUNC_TRANSMIT_PDO_2` | `0x5` | node→host: [pos_measured f32, vel_measured f32] |
| `FUNC_RECEIVE_PDO_2` | `0x6` | host→node: [pos_target f32, vel_ff f32] |
| `FUNC_TRANSMIT_PDO_3` | `0x7` | node→host: calibration status frames |
| `FUNC_RECEIVE_PDO_3` | `0x8` | host→node: unused |
| `FUNC_TRANSMIT_PDO_4` | `0x9` | node→host: autonomous fast-frame [pos f32, vel f32] |
| `FUNC_RECEIVE_PDO_4` | `0xA` | host→node: no-op |
| `FUNC_TRANSMIT_SDO` | `0xB` | node→host: SDO response (8-byte CANopen) |
| `FUNC_RECEIVE_SDO` | `0xC` | host→node: SDO read/write request |
| `FUNC_FLASH` | `0xD` | host→node: store(1) / load(2) flash |
| `FUNC_HEARTBEAT` | `0xE` | node→host: heartbeat/ACK (5 bytes: mode + err); host must NOT transmit |

### 4.3 Motor modes (`MotorMode`)

| Name | Value | | Name | Value |
|---|---|---|---|---|
| `MODE_DISABLED` | `0x00` | | `MODE_CURRENT` | `0x10` |
| `MODE_IDLE` | `0x01` | | `MODE_TORQUE` | `0x11` |
| `MODE_DAMPING` | `0x02` | | `MODE_VELOCITY` | `0x12` |
| `MODE_CALIBRATION` | `0x05` | | `MODE_POSITION` | `0x13` |
| `MODE_VABC_OVERRIDE` | `0x20` | | `MODE_VALPHABETA_OVERRIDE` | `0x21` |
| `MODE_VQD_OVERRIDE` | `0x22` | | `MODE_DEBUG` | `0x80` |

### 4.4 Error codes (`ErrorCode`, uint32 bitmask)

| Name | Value | | Name | Value |
|---|---|---|---|---|
| `ERROR_NO_ERROR` | `0x0000` | | `ERROR_OVER_VOLTAGE` | `0x0080` |
| `ERROR_GENERAL` | `0x0001` | | `ERROR_OVER_CURRENT` | `0x0100` |
| `ERROR_ESTOP` | `0x0002` | | `ERROR_OVER_TEMPERATURE` | `0x0200` |
| `ERROR_INITIALIZATION_ERROR` | `0x0004` | | `ERROR_CAN_RX_FAULT` | `0x0400` |
| `ERROR_CALIBRATION_ERROR` | `0x0008` | | `ERROR_CAN_TX_FAULT` | `0x0800` |
| `ERROR_POWERSTAGE_ERROR` | `0x0010` | | `ERROR_I2C_FAULT` | `0x1000` |
| `ERROR_INVALID_MODE` | `0x0020` | | `ERROR_ENCODER_FAULT` | `0x2000` |
| `ERROR_WATCHDOG_TIMEOUT` | `0x0040` | | | |

### 4.5 SDO command bytes

| Name | Value | Meaning |
|---|---|---|
| `SDO_CMD_WRITE` | `0x20` | host→node expedited download (write) request |
| `SDO_CMD_READ` | `0x40` | host→node upload initiate (read) request |
| `SDO_WRITE_ACK` | `0x60` | node→host download response byte 0 (write ACK, DLC 8) |
| `SDO_READ_RESP` | `0x43` | node→host upload response byte 0 (read result, DLC 8) |
| `PING_MAGIC` | `0xCA` | ping payload byte for `FUNC_RECEIVE_PDO_1` |

### 4.6 Frame layouts

- **SDO request** (8 bytes): `cmd(1) | param_id_lo(1) | param_id_hi(1) | pad(1) | value(4 LE)`.
- **SDO response** (8 bytes): `cmd(1=0x43/0x60) | param_id(2 LE) | pad(1) | value(4 LE)`.
  (Commissioning ELF v3.0.0 returns a 4-byte response with the value at bytes 0–3 —
  the daemon handles both.)
- **PDO2 TX / RX**, **PDO4 RX**, **PDO3 RX**: two `float32` LE (8 bytes total).
- **NMT** (2 bytes): `mode(1) | device_id(1)`; device_id 0 = broadcast.
- **Heartbeat RX** (5 bytes, new firmware): `mode(1) | error_code(4 LE)`.
  Older firmware sends an 8-byte zero frame.
- **EMCY** (4 bytes): `error_code(4 LE)`.
- **Flash cmd** (1 byte): `op` (1 = store, 2 = load).

### 4.7 SDO parameter address map (`ParamId`)

Values are byte offsets into the firmware `MotorController` struct; the low 16 bits go
into the SDO `param_id` field. Type column: `f32` unless noted; `u32`/`i32` where the
daemon writes integers. (Full enum is in `recoil_protocol.hpp`; the subset the daemon
reads/writes is marked ●.)

| Param | Offset | Type | Daemon use |
|---|---|---|---|
| `PARAM_DEVICE_ID` | `0x000` | u32 | |
| `PARAM_FIRMWARE_VERSION` | `0x004` | u32 | ● read on connect (packed `0xMMmmPPrr`) |
| `PARAM_WATCHDOG_TIMEOUT` | `0x008` | u32 | ● write (config) |
| `PARAM_FAST_FRAME_FREQUENCY` | `0x00C` | u32 | ● write (config) |
| `PARAM_MODE` | `0x010` | u32 | |
| `PARAM_ERROR` | `0x014` | u32 | ● write 0 on clear_fault |
| `PARAM_POSITION_CONTROLLER_GEAR_RATIO` | `0x01C` | f32 | ● write / read |
| `PARAM_POSITION_CONTROLLER_POSITION_KP` | `0x020` | f32 | ● write / read |
| `PARAM_POSITION_CONTROLLER_POSITION_KI` | `0x024` | f32 | ● write / read |
| `PARAM_POSITION_CONTROLLER_VELOCITY_KP` | `0x028` | f32 | ● write / read (this is "Kd") |
| `PARAM_POSITION_CONTROLLER_VELOCITY_KI` | `0x02C` | f32 | ● write / read |
| `PARAM_POSITION_CONTROLLER_TORQUE_LIMIT` | `0x030` | f32 | ● write / read |
| `PARAM_POSITION_CONTROLLER_VELOCITY_LIMIT` | `0x034` | f32 | ● write / read |
| `PARAM_POSITION_CONTROLLER_POSITION_LIMIT_LOWER` | `0x038` | f32 | ● write (offset-adjusted) / read |
| `PARAM_POSITION_CONTROLLER_POSITION_LIMIT_UPPER` | `0x03C` | f32 | ● write (offset-adjusted) / read |
| `PARAM_POSITION_CONTROLLER_POSITION_OFFSET` | `0x040` | f32 | ● write / read |
| `PARAM_POSITION_CONTROLLER_TORQUE_MEASURED` | `0x048` | f32 | ● slow-poll read → `torque` |
| `PARAM_POSITION_CONTROLLER_TORQUE_FILTER_ALPHA` | `0x070` | f32 | ● write / read |
| `PARAM_CURRENT_CONTROLLER_I_LIMIT` | `0x074` | f32 | ● write / read (current_limit) |
| `PARAM_CURRENT_CONTROLLER_I_KP` | `0x078` | f32 | ● write / read (current_kp) |
| `PARAM_CURRENT_CONTROLLER_I_KI` | `0x07C` | f32 | ● write / read (current_ki) |
| `PARAM_CURRENT_CONTROLLER_I_Q_MEASURED` | `0x0C0` | f32 | ● slow-poll read → `current` |
| `PARAM_POWERSTAGE_UNDERVOLTAGE_THRESHOLD` | `0x0F4` | f32 | ● write / read |
| `PARAM_POWERSTAGE_OVERVOLTAGE_THRESHOLD` | `0x0F8` | f32 | ● write / read |
| `PARAM_POWERSTAGE_BUS_VOLTAGE_FILTER_ALPHA` | `0x0FC` | f32 | ● write / read |
| `PARAM_POWERSTAGE_BUS_VOLTAGE_MEASURED` | `0x100` | f32 | ● slow-poll read → `bus_voltage` |
| `PARAM_MOTOR_POLE_PAIRS` | `0x104` | u32 | ● write (config) |
| `PARAM_MOTOR_TORQUE_CONSTANT` | `0x108` | f32 | ● write / read |
| `PARAM_MOTOR_PHASE_ORDER` | `0x10C` | i32 | ● write (`+1` normal, `-1` if phase_inverted) |
| `PARAM_MOTOR_MAX_CALIBRATION_CURRENT` | `0x110` | f32 | ● write / read |
| `PARAM_ENCODER_CPR` | `0x120` | u32 | ● write (config) |
| `PARAM_ENCODER_POSITION_OFFSET` | `0x124` | f32 | ● write / read |
| `PARAM_ENCODER_VELOCITY_FILTER_ALPHA` | `0x128` | f32 | ● write / read |
| `PARAM_ENCODER_FLUX_OFFSET` | `0x13C` | f32 | ● write (electrical_offset) / read after calibrate |

Other offsets present in the enum (current/voltage setpoints, integrators, powerstage
HTIM/HADC handles, flux-offset table, etc.) are defined for completeness but not
exercised by the daemon; consult `recoil_protocol.hpp` for the full list.

---

## 5. UDP API Reference (authoritative)

Transport: JSON datagrams over UDP on `127.0.0.1`.

- **Port 9001** — command/response RPC (Python → daemon; daemon replies to the sender).
- **Port 9000** — telemetry push (daemon → Python; fire-and-forget, no request).
- **Port 9002** — priority E-Stop only (see §5.4).

**Envelope:** every command may include an `"id"` correlation string; the daemon
echoes it in the response. The Python client sets `id` to a UUID and discards any
response whose `id` does not match (drops stale buffered responses). Command socket
timeout on the Python side is 5 s by default (per-command overrides exist).

Every response has a `"type"`. Generic success is `{"type":"ACK","id":...}`.
Errors are `{"type":"ERROR","id":...,"msg":"..."}` (unknown joint, closed bus, bad
params, unknown command type, or a caught exception).

### 5.1 Core joint commands (port 9001)

| `type` | Request fields | Response |
|---|---|---|
| `PING` | — | `{type:"PONG", id, daemon_version:"1.0"}` |
| `GET_STATE` | `joint_name` | `{type:"STATE", id, state:{...}}` — see §5.2 |
| `GET_ALL_STATES` | — | `{type:"ALL_STATES", id, states:{name→{...}}}` |
| `SET_MODE` | `joint_name`, `mode` | `ACK` / `ERROR` |
| `SET_ALL_MODE` | `mode` | `ACK` / `ERROR` |
| `SET_POSITION` | `joint_name`, `position_rad` | `ACK` / `ERROR` |
| `CLEAR_ERROR` | `joint_name` | `ACK` — SDO-writes error reg 0, sets IDLE |
| `APPLY_CONFIG` | `joint_name`, optional `config`{…} | `ACK` / `ERROR` |
| `APPLY_ALL_CONFIGS` | — | `{type:"ACK", id, configured, skipped, failed}` |
| `WRITE_GAINS` | `joint_name`, `position_kp`, `position_ki`, `velocity_kp`, `torque_limit` | `ACK` / `ERROR` |
| `READ_CONFIG` | `joint_name` | `{type:"CONFIG", id, config:{…}}` — reads ~24 params via SDO |
| `STORE_TO_FLASH` | `joint_name` | `ACK` (blocks ~150 ms) |
| `DISABLE_SLOW_POLL` / `ENABLE_SLOW_POLL` | `joint_name` | `ACK` |
| `DISABLE_ALL_SLOW_POLL` / `ENABLE_ALL_SLOW_POLL` | — | `ACK` |
| `SHUTDOWN` | — | `ACK`, then graceful stop ~100 ms later |

**`mode` string mapping** (`SET_MODE` / `SET_ALL_MODE`): `"POSITION"` or `"ENABLED"`
→ `ENABLED`; `"IDLE"` → `IDLE`; `"DISABLED"` → `DISABLED`. Any other value → `ERROR`.
There is no `SET_TORQUE`/`SET_VELOCITY`/`CALIBRATE` joint-name command in the shipped
daemon — velocity/torque control and calibration go through the generic passthrough
commands (§5.3) keyed by `channel`+`device_id`.

**`APPLY_CONFIG` config object** (all optional; absent keys keep the loaded value):
`gear_ratio, position_kp, position_ki, velocity_kp, velocity_ki, torque_limit,
velocity_limit, position_limit_min, position_limit_max, position_offset,
torque_filter_alpha, current_limit, current_kp, current_ki, undervoltage_threshold,
overvoltage_threshold, bus_voltage_filter_alpha, torque_constant,
max_calibration_current, encoder_position_offset, velocity_filter_alpha,
electrical_offset, fast_frame_frequency, watchdog_timeout, pole_pairs, cpr,
phase_inverted`. Writes are **delta writes** (only changed params are sent) and
fail-fast on the first SDO timeout (default 500 ms/write). `APPLY_ALL_CONFIGS` first
transitions every joint on an open bus to IDLE (wake), waits 300 ms, then applies;
joints on closed buses are counted in `skipped`.

### 5.2 Telemetry & state schema

**`GET_STATE` / `GET_ALL_STATES`** per-joint `state` object (fields from
`Robot::handle_command`):

```json
{
  "position": 0.123,          // display-frame rad (firmware subtracts position_offset)
  "velocity": -0.01,          // rad/s, output side
  "torque": 1.2,              // Nm (slow-poll SDO)
  "current": 3.1,             // i_q A (slow-poll SDO)
  "mode": 19,                 // firmware MotorMode as int (e.g. 0x13 = POSITION)
  "error": 0,                 // firmware error bitmask (uint32)
  "joint_state": "ENABLED",   // daemon JointState string
  "bus_voltage": 23.8,        // float, or null if unknown (<0)
  "firmware_version": "v3.1.2"// string "vMAJ.MIN.PATCH", or null if not yet read
}
```

**Telemetry push** on port 9000 (`Robot::build_telemetry_json`), published at the
configured rate (**100 Hz** with the shipped config; see §2):

```json
{
  "type": "TELEMETRY",
  "seq": 12345,
  "timestamp_us": 1716652800000000,
  "joints": {
    "left_hip_yaw": {
      "state": "ENABLED",         // NOTE: key is "state" here (not "joint_state")
      "position": 0.123,
      "velocity": -0.01,
      "torque": 1.2,
      "current": 3.1,
      "mode": 19,
      "error": 0,
      "bus_voltage": 23.8,
      "firmware_version": "v3.1.2"
    }
  },
  "bus_health": {
    "can_left_leg":  {"open": true,  "tx_dropped": 0, "rx_frames": 12345},
    "can_right_leg": {"open": true,  "tx_dropped": 0, "rx_frames": 12000},
    "can_left_arm":  {"open": false, "tx_dropped": 0, "rx_frames": 0},
    "can_right_arm": {"open": false, "tx_dropped": 0, "rx_frames": 0}
  }
}
```

Field notes:
- The per-joint state key is **`state`** in the telemetry push and **`joint_state`**
  in `GET_STATE`/`GET_ALL_STATES`. The Python client accepts either
  (`d.get("state") or d.get("joint_state")`).
- `bus_voltage` is `null` until a slow-poll read succeeds (daemon uses `-1.0` sentinel
  internally, serialized as JSON `null`).
- `firmware_version` is a formatted string (or `null`); it is decoded from the packed
  `0xMMmmPPrr` u32 as `v{byte3}.{byte2}.{byte1}` (the low byte is reserved/ignored).
- `bus_health` reports per-bus **`open`**, **`tx_dropped`**, and **`rx_frames`**.
  There is no per-joint `calibration_progress` field in the shipped telemetry (PDO3
  calibration frames are currently ignored by the actuator; calibration status is
  reported via the `CALIBRATE_DEVICE` command result instead).
- Canonical bus names: `can_left_leg`, `can_right_leg`, `can_left_arm`,
  `can_right_arm`.

### 5.3 Generic CAN passthrough commands (Flash Wizard / commissioning, port 9001)

These operate on arbitrary `channel`+`device_id` (not limited to configured joints)
so Python can do SDO/NMT/sniff/calibration without opening its own CAN socket. Backed
by the daemon's `GenericListener` (one-shot futures + bus sniffs).

| `type` | Request fields | Response |
|---|---|---|
| `GENERIC_SDO_WRITE` | `channel`, `device_id`, `param_id`, `value_type`("u32"/"i32"/"f32"), `value`, `timeout_ms`(500) | `{type:"SDO_WRITE_RESULT", status:"OK"/"NO_ACK"/"BAD_ACK"}` |
| `GENERIC_SDO_READ` | `channel`, `device_id`, `param_id`, `timeout_ms`(500) | `{type:"SDO_READ_RESULT", status:"OK"/"TIMEOUT"/"BAD_RESPONSE", value_u32, value_f32, value_i32, sniff_count}` (up to 3 attempts) |
| `WAIT_HEARTBEAT` | `channel`, `device_id`, `timeout_ms`(3000) | `{type:"HEARTBEAT_RESULT", status:"OK"/"TIMEOUT", mode, error}` |
| `SNIFF_BUS` | `channel`, `duration_ms`(3000, capped 30000) | `{type:"SNIFF_RESULT", status:"OK", frames:[{device_id,func_id,dlc,data(hex)}]}` |
| `SEND_NMT` | `channel`, `device_id`, `mode` | `ACK` |
| `SEND_FLASH_STORE` | `channel`, `device_id` | `ACK` (sends `FUNC_FLASH` op=1, blocks 300 ms) |
| `FEED_WATCHDOG` | `channel`, `device_id` | `ACK` (sends `NMT IDLE`, not `FUNC_HEARTBEAT`) |
| `CALIBRATE_DEVICE` | `channel`, `device_id`, `timeout_ms`(90000) | `{type:"CALIBRATE_RESULT", status:"OK"/"TIMEOUT"/"ERROR", error_code}` |

`CALIBRATE_DEVICE` sends `NMT MODE_CALIBRATION`, waits (≤5 s) for the CALIBRATION
heartbeat ACK, then waits for the motor to heartbeat back to `IDLE`/`DISABLED`
(nudging with `NMT IDLE` after ~20 s), reporting `ERROR` if
`ERROR_CALIBRATION_ERROR` is set.

### 5.4 Priority E-Stop (port 9002)

A dedicated `UdpServer` bound to `127.0.0.1:9002` handles **only** `{"type":"ESTOP"}`,
returning `{"type":"ACK"}` and setting `estop_pending_` atomically. Any other message
returns `{"type":"ERROR","msg":"priority port: ESTOP only"}`. This port exists so the
E-Stop is never blocked behind an in-progress `apply_config` holding the command lock
on 9001. The control loop consumes the flag at the top of the very next tick and
drives all joints to IDLE. If the priority server fails to bind, it is non-fatal
(E-Stop via 9001 `SET_ALL_MODE IDLE` still works). Port 9002 is hard-coded.

---

## 6. Actuator State Machine (`motor/actuator.{hpp,cpp}`)

**States** (`enum class JointState`): `OFFLINE`, `DISABLED`, `IDLE`, `ENABLED`,
`CALIBRATING`, `FAULT`. (Serialized names match the enum.)

**Transitions (as implemented):**

```
OFFLINE      → IDLE         first PDO4 or heartbeat received (sets needs_idle_wakeup_,
                            forces a full config rewrite on next apply)
IDLE         → ENABLED      SET_MODE POSITION/ENABLED: pre-send NMT IDLE, seed hold
                            position via PDO2, send NMT POSITION, mark ENABLED
any (non-OFFLINE/DISABLED) → OFFLINE   no PDO4/heartbeat for > 1500 ms
ENABLED      → IDLE         SET_MODE IDLE (NMT IDLE)
*            → DISABLED     SET_MODE DISABLED (NMT DISABLED); OFFLINE timer suppressed
ENABLED/CALIBRATING → FAULT EMCY frame received
CALIBRATING  → IDLE         heartbeat reports MODE_IDLE (calibration complete)
FAULT        → IDLE         CLEAR_ERROR (SDO error reg ← 0, local state ← IDLE)
```

Notes:
- `DISABLED` is intentionally silent (used by `disconnect()` for commissioning); its
  OFFLINE timer is suppressed so it does not oscillate to OFFLINE. It only leaves
  DISABLED via an explicit `SET_MODE IDLE`.
- On `OFFLINE→IDLE`, `needs_idle_wakeup_` fires an `NMT IDLE` on the next tick to wake
  firmware from boot-time `MODE_DISABLED` so SDO reads respond (firmware boots
  DISABLED since v3.2.0).
- `firmware_version` is preserved across brief OFFLINE windows and re-read on the next
  `apply_config` after a fresh connect.

**Position frame:** `position` on the wire and in telemetry is display-frame; the
firmware applies `position_offset` internally (both for PDO2 commands and PDO4/PDO2
feedback). `SET_POSITION position_rad` is a display-frame target.

---

## 7. Config Loading (`config/config_loader.{hpp,cpp}` + `configs/humanoid_lite.json`)

Loaded with the vendored `nlohmann/json`. Top-level JSON keys: `robot_name`,
`telemetry_hz`, `can_assignments` (USB-serial → bus-label map, e.g.
`"...": "left_leg"`), and `joints` (map of 22 joints). Null `position_limits` become
±`NO_LIMIT_RAD` (**100.0** rad), matching the Python `_NO_LIMIT_RAD` sentinel.

Per-joint JSON fields (verified against the shipped config and `JointConfig`):
`joint_name, joint_type, can_channel, can_id, phase_inverted, electrical_offset,
gear_ratio, position_limits{min,max}, position_kp, position_ki, velocity_kp,
velocity_ki, torque_limit, velocity_limit, position_offset, torque_filter_alpha,
current_limit, current_kp, current_ki, undervoltage_threshold, overvoltage_threshold,
bus_voltage_filter_alpha, pole_pairs, torque_constant, max_calibration_current, cpr,
encoder_position_offset, velocity_filter_alpha, watchdog_timeout, fast_frame_frequency`.

`load_config()` throws (fail-fast, daemon exits 1) on a missing file or required field.

---

## 8. CAN / IPC Internals (as-built)

- **`SocketCan`** (`can/socket_can.*`): one non-blocking raw `PF_CAN`/`CAN_RAW` socket
  per interface (`SIOCGIFINDEX` + bind). `recv()` is a non-blocking single-frame read;
  `send()` returns false on `ENOBUFS` and increments `tx_dropped`. There is **no**
  per-bus reader thread and **no** epoll/ring-buffer — frames are drained inline in the
  control loop (`drain_all`, ≤64 frames/tick). Tracks `tx_dropped` and `rx_frames`.
- **`CanBusManager`** (`can/can_bus_manager.*`): owns the sockets keyed by ifname.
  Missing interfaces are non-fatal; `is_open(ifname)` reflects link state. Auto-reopen
  with a 1 s back-off; "send on closed bus" logs are rate-limited to 5 s. Exposes
  per-bus `BusStats{open, tx_dropped, rx_frames}` via `stats()`.
- **`GenericListener`** (`can/generic_listener.*`): one-shot futures and bus sniffs
  used by the §5.3 passthrough commands.
- **`UdpServer`** (`ipc/udp_server.*`): blocking `recvfrom` with a 100 ms `SO_RCVTIMEO`
  so the receive loop can observe shutdown; replies to the sender's address;
  `SO_REUSEADDR` for fast restart. Runs at normal priority.
- **`UdpBroadcaster`** (`ipc/udp_broadcaster.*`): fire-and-forget datagram send to
  `127.0.0.1:<tel-port>`.

**Graceful shutdown** (`Robot::stop()`): stop the control loop; send `NMT DAMPING` to
every `ENABLED`/`CALIBRATING` joint; wait 500 ms; send `NMT IDLE` to all joints; wait
200 ms; stop the UDP servers; join the telemetry thread. (A second signal during this
window force-exits via `main.cpp`.)

---

## 9. PLANNED: External IMU extension (NOT YET IMPLEMENTED)

> This section is a **forward contract stub** so upcoming control/policy code can
> target the schema now. None of it exists in the daemon source yet — do not treat
> any name below as verified. Implement and then move these facts into §2/§5/§7.

**Concept:** an external serial/USB IMU (e.g. a 9-DoF board on `/dev/ttyUSBx`) read by
a dedicated daemon reader thread. No ESC-firmware change; the IMU is not on the CAN
bus. The reader thread parses the IMU stream, maintains the latest orientation
snapshot (behind a lock, like the telemetry snapshot), and the telemetry builder adds
a top-level `base` block to each push.

**Proposed telemetry addition** (top level, alongside `joints` and `bus_health`):

```json
"base": {
  "quaternion":         [w, x, y, z],   // orientation, world→base (order TBD in impl)
  "angular_velocity":   [wx, wy, wz],   // rad/s, base frame
  "projected_gravity":  [gx, gy, gz]    // unit gravity vector in base frame
}
```

**Proposed config additions** (top level or under a new `imu` object in
`humanoid_lite.json`):

- `imu.device` — serial device path (e.g. `/dev/ttyUSB0`)
- `imu.baud` — baud rate (e.g. `115200`)
- `imu.mounting_rotation` — quaternion or 3×3 rotating IMU frame → base frame
- (optional) `imu.rate_hz`, `imu.enabled`

**Proposed CLI additions:** `--imu-device`, `--imu-baud` (mirroring the config).

**Open decisions for the implementer:** quaternion ordering/convention (wxyz vs xyzw,
world→base vs base→world); whether `projected_gravity` is computed on the daemon or by
the policy; how a missing/disconnected IMU is signaled (e.g. `base: null` vs a
`base.valid` flag); and whether the policy consumes `base` from the 9000 telemetry push
or from a new dedicated high-rate port.

---

## 10. Implementation Status (COMPLETED)

All phases below are done and running; this section is a historical record of the
build-out, reframed as-built.

- **Daemon skeleton** — DONE. Config loader, UDP server, signal handling, `main.cpp`;
  responds to `PING`, clean shutdown on `SIGINT`.
- **CAN layer** — DONE. `SocketCan`, `CanBusManager`, `recoil_protocol.hpp` frame
  helpers, `Actuator` SDO state machine, `GET_STATE`.
- **Control loop + full command set** — DONE. 200 Hz `SCHED_FIFO` loop, per-tick drain
  → tick, telemetry push on 9000, all §5 commands including generic passthrough and the
  9002 priority E-Stop.
- **Python migration** — DONE. `backend/humanoid/daemon_client.py` implements the
  client + `DaemonActuatorProxy`; FastAPI routes call the client. `python-can` is off
  the live path (a few Python modules are retained only for enums/type definitions used
  by the Flash Wizard, which uses OpenOCD/SWD, not CAN).
- **Integration/perf validation** — DONE. Verified on hardware across multiple firmware
  revisions (see `MEMORY.md` session notes).

### As-built verification checklist

- [x] `humanoid_daemon --config ../configs/humanoid_lite.json` starts and loads 22 joints
- [x] `GET_ALL_STATES` returns all joints (OFFLINE for absent, IDLE for powered)
- [x] `APPLY_ALL_CONFIGS` delta-writes params; skips closed buses; returns
      `{configured, skipped, failed}`
- [x] `SET_MODE <joint> POSITION` → `ENABLED`; `SET_POSITION` → PDO2 on the bus
- [x] PDO4/PDO2 feedback updates `position`/`velocity`; slow-poll fills
      `bus_voltage`/`current`/`torque`
- [x] EMCY → `FAULT`; `CLEAR_ERROR` → error reg 0 + `IDLE`
- [x] Priority E-Stop on 9002 drives all joints IDLE at the next tick
- [x] Generic passthrough (`GENERIC_SDO_*`, `SNIFF_BUS`, `CALIBRATE_DEVICE`, …) drives
      Flash Wizard commissioning
- [x] `SIGINT` → DAMPING → IDLE graceful shutdown; second signal force-exits
- [x] No Python-originated CAN frames during normal operation (daemon owns the bus)
