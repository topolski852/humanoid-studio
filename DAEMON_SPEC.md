# Humanoid Studio — C++ CAN Daemon Specification
**Version 1.0 | 2026-05-25**

---

## 1. Purpose & Scope

The Python `python-can`-based transport layer (`can_bus.py`, `actuator.py`, `robot.py`,
`can_monitor.py`) is replaced by a standalone C++ daemon process that owns all four SocketCAN
interfaces. The FastAPI Python layer remains but becomes a thin proxy: it communicates with
the daemon exclusively via UDP on localhost. Python never opens a CAN socket after this change.

**Motivation:**
- Eliminate the GIL from the CAN hot path; Python async I/O cannot guarantee < 1 ms frame
  latency at 200 Hz across 22 joints on 4 buses simultaneously.
- Enable real-time scheduling (SCHED_FIFO) for the CAN control loop without affecting the
  FastAPI web server.
- Decouple the control layer from the web layer so each can crash, restart, and upgrade
  independently.
- Provide a foundation for a future direct RL-policy UDP interface (Berkeley-style) that
  bypasses the web layer entirely.

**Scope of this document:**
- Berkeley reference analysis (what their code does well and why)
- Capability gap analysis (Berkeley vs. Humanoid Studio requirements)
- Complete C++ daemon file structure, class designs, threading model, and protocol
- Python backend changes (delete / add / modify)
- 5-phase migration plan

---

## 2. Berkeley Humanoid Lite Reference Analysis

> Source: `/home/nse/Berkeley-Humanoid-Lite/source/berkeley_humanoid_lite_lowlevel/csrc/`
> This is READ-ONLY reference material. No code is copied from it.

### 2.1 SocketCAN

Berkeley creates one raw CAN socket per interface using `socket(PF_CAN, SOCK_RAW, CAN_RAW)`,
binds to the interface by name via `ioctl(SIOCGIFINDEX)`, and performs blocking reads with a
1-second `select()` timeout. Writes use a plain `::write()` call; a return value of -1
indicates a full TX queue (ENOBUFS). There is no epoll, no ring buffer, and no separate
reader thread — reads happen inline in the control loop.

**Lesson:** The approach is correct for a single-bus, single-thread design. For four buses
at 200 Hz the daemon must use epoll + per-bus reader threads to avoid head-of-line blocking.

### 2.2 Multi-Bus Architecture

Berkeley manages exactly one CAN interface per process instance. There is no multi-bus
abstraction. A second instance of the process would open a second socket on a different
interface independently.

**Lesson:** The daemon must generalize this to N buses managed by one process, with a
`BusManager` that maps interface name → `CANBus` instance and fans frames in/out per bus.

### 2.3 Control Loop Design

Berkeley uses `LoopFunc`, a reusable real-time loop framework:
- Constructor takes period (double, seconds), a callable, CPU affinity, SCHED_FIFO priority.
- Timing uses `std::chrono::high_resolution_clock`.
- If the callable finishes early the thread sleeps for the remainder; if it overruns, the
  overrun duration is logged to stderr.
- Thread affinity is set via `pthread_setaffinity_np()`; priority via `pthread_setschedparam()`.

The Berkeley control loop runs at **100 Hz** on CPU 0 at priority 50. A separate UDP receive
loop runs at 500 Hz on CPU 1 at priority 49.

**Lesson:** Adopt the `LoopFunc` pattern (concept, not code). The daemon control loop runs
at 200 Hz on CPU 0 at priority 80. The telemetry publish loop runs at 10–100 Hz on CPU 1
at priority 40.

### 2.4 Motor State Machine

Berkeley's per-motor state tracks: target position, measured position, measured velocity,
KP/KD gains, position offset, and axis direction multiplier. There is no explicit per-motor
state enum — the higher-level `real_humanoid.cpp` tracks robot-level states (IDLE, RL_INIT,
RL_RUNNING). Individual motors are assumed always reachable; there is no timeout/offline
handling per joint.

**Lesson:** The daemon needs a richer per-joint state enum (`OFFLINE`, `IDLE`, `ENABLED`,
`CALIBRATING`, `FAULT`) with timeout-driven transitions, because Humanoid Studio joints can
be individually unreachable (missing CAN adapter, unpowered ESC).

### 2.5 CAN Frame Encoding

Berkeley's CAN ID encoding:
```
can_id = (func_id << 7) | device_id
func_id: 4 bits [10:7], device_id: 7 bits [6:0]
```

**PDO2 frame:** 8 bytes — `float32 position_target` at bytes 0–3,
`float32 velocity_target` at bytes 4–7. Sent as write; response read on same arb_id.

**SDO read:** TX arb_id = `(FUNC_RECEIVE_SDO << 7) | device_id`, byte 0 = `0x40`, bytes
1–2 = param index (little-endian), byte 3 = 0. RX arb_id =
`(FUNC_TRANSMIT_SDO << 7) | device_id`, byte 0 = `0x43`, bytes 4–7 = float32 value.

**SDO write:** TX byte 0 = `0x23`, bytes 1–2 = param index, bytes 4–7 = value.
New firmware (≥ 0x20250226) replies with 1-byte `0x60` ACK on TRANSMIT_SDO.

**HEARTBEAT watchdog:** arb_id = `(FUNC_HEARTBEAT << 7) | device_id`, 8 bytes of zeros.
Must arrive within `watchdog_timeout` (1000 ms default) or ESC falls to DAMPING.
New firmware also sends a HEARTBEAT reply (5 bytes: mode + uint32 error) immediately after
any NMT mode change.

**NMT mode change:** arb_id = `(FUNC_NMT << 7) | device_id`, byte 0 = new_mode,
byte 1 = device_id.

**EMCY error broadcast:** arb_id = `(FUNC_SYNC_EMCY << 7) | device_id`, 4 bytes =
uint32 error code. Sent by new firmware on any fault injection.

**PDO4 autonomous broadcast:** arb_id = `(FUNC_TRANSMIT_PDO_4 << 7) | device_id`,
float32 position at bytes 0–3, float32 velocity at bytes 4–7. Arrives at
`fast_frame_frequency` Hz without being requested (if non-zero).

**PDO3 calibration status:** arb_id = `(FUNC_TRANSMIT_PDO_3 << 7) | device_id`,
float32 voltage_setpoint at bytes 0–3, float32 phase_current_or_progress at bytes 4–7.
Sent by new firmware during calibration sequence.

**Lesson:** All frame types must be implemented as named constructors in a `Protocol`
namespace — no magic numbers in business logic.

### 2.6 Watchdog

Berkeley feeds each motor's watchdog by sending a position command PDO2 at every 100 Hz
control tick. There is no separate heartbeat mechanism.

In Humanoid Studio, the firmware watchdog is reset by PDO2 writes OR by explicit
HEARTBEAT frames. The daemon feeds the watchdog from the control loop: ENABLED joints get
PDO2 every tick; IDLE joints get an explicit HEARTBEAT frame every 200 ms.

**Lesson:** Watchdog feed must move from the Python background task into the C++ control loop.

### 2.7 Config Loading

Berkeley uses `yaml-cpp` to load two YAML files at startup (calibration offsets,
KP/KD gains, torque limits). No runtime reload.

Humanoid Studio uses `configs/humanoid_lite.json` — a single JSON file with all 22 joints,
their CAN channel, device ID, offsets, gains, limits, and motor parameters.

**Lesson:** Use `nlohmann/json` (header-only) to load the same JSON format. No YAML needed.

### 2.8 UDP Communication

Berkeley sends 35-float robot observation state and receives 12-float action commands via
UDP, with no framing — raw float arrays. Actions received at 500 Hz in a dedicated thread.

**Lesson:** The daemon uses JSON-over-UDP for the Python interface because Humanoid Studio
is command/response (RPC-style), not continuous float streaming. JSON adds ~200 µs per
message at rates ≤ 200 Hz, which is acceptable. Two ports: 9000 (daemon → Python telemetry
push) and 9001 (Python → daemon RPC commands).

### 2.9 Build System

Berkeley: CMake 3.10+, FetchContent for `yaml-cpp`, single executable, no install target.

**Lesson:** Use CMake 3.16+, FetchContent for `nlohmann_json`, C++17. SCHED_FIFO without
root requires `sudo setcap cap_sys_nice+ep build/humanoid_daemon` after each build.

---

## 3. Capability Gap Analysis

| Capability | Berkeley Has | Daemon Needs |
|---|---|---|
| SocketCAN raw socket | Yes (single interface) | Yes (4 interfaces simultaneously) |
| epoll multi-socket | No (select, single socket) | Yes (wait on all 4 buses at once) |
| Per-bus reader thread | No (inline in control loop) | Yes (one per bus, feeds ring buffer) |
| SCHED_FIFO control loop | Yes (100 Hz) | Yes (200 Hz) |
| Per-joint state enum | No (robot-level only) | Yes (OFFLINE/IDLE/ENABLED/CALIBRATING/FAULT) |
| SDO read/write | Yes | Yes (same encoding) |
| PDO2 position command | Yes | Yes (same encoding) |
| PDO4 passive receive | No | Yes (sniff to avoid SDO reads) |
| HEARTBEAT watchdog | No (PDO2 doubles as keepalive) | Yes (explicit HEARTBEAT for IDLE joints) |
| EMCY frame handling | No | Yes (update joint fault state) |
| NMT mode ACK (new fw) | No | Yes (HEARTBEAT reply on NMT) |
| SDO write ACK (new fw) | No | Yes (consume 0x60 after write) |
| Calibration status frames | No | Yes (PDO3 during calibration) |
| JSON config loading | No (YAML) | Yes (nlohmann/json) |
| UDP RPC interface | No (raw float arrays) | Yes (JSON request/response) |
| Multi-joint parallel ops | No (sequential) | Yes (per-bus concurrency) |
| Graceful shutdown | Yes (2-stage DAMPING→IDLE) | Yes (same pattern) |
| Subprocess management | N/A | Yes (Python spawns daemon) |
| Position offset conversion | Yes (simple add) | Yes (raw ↔ display frame) |
| Phase inversion (SDO 0x10C) | No | Yes (write at config apply) |

---

## 4. Daemon Architecture

### 4.1 File Structure

```
humanoid-studio/
  daemon/
    CMakeLists.txt
    src/
      main.cpp              Entry point: parse args, load config, start daemon, block on signal
      protocol.h            CAN frame constants, param IDs, enums (Mode, Function, ErrorCode)
      types.h               Shared structs: ActuatorState, JointConfig, CANFrame
      can_bus.cpp/.h        Single CAN interface: raw socket, reader thread, epoll, Tx queue
      bus_manager.cpp/.h    Owns N CANBus instances; routes frames to/from joints by device_id
      actuator.cpp/.h       Per-joint state machine, config apply, position conversions
      robot.cpp/.h          Multi-joint coordinator: load config, fan-out commands, PDO4 monitor
      control_loop.cpp/.h   200 Hz SCHED_FIFO thread: drive all joints, feed watchdog
      udp_server.cpp/.h     UDP RPC server: receive commands, dispatch to Robot, push telemetry
      config_loader.cpp/.h  Load humanoid_lite.json → vector<JointConfig>
      loop_func.h           Real-time loop utility (period, CPU affinity, SCHED_FIFO priority)
      signal_handler.cpp/.h SIGINT/SIGTERM → atomic stop flag + graceful shutdown
```

### 4.2 SocketCAN Layer (`can_bus.cpp/.h`)

**Initialization sequence:**
```
socket(PF_CAN, SOCK_RAW, CAN_RAW)           // create raw CAN socket
ioctl(sock_fd, SIOCGIFINDEX, &ifreq)         // resolve interface name → index
fcntl(sock_fd, F_SETFL, O_NONBLOCK)          // non-blocking for epoll
bind(sock_fd, &sockaddr_can{AF_CAN, ifidx})  // bind to interface
```

**Reader thread** (one per `CANBus`, SCHED_FIFO priority 70):
- `epoll_wait` with 10 ms timeout on the socket fd.
- On `EPOLLIN`: `::read()` one `can_frame` (8-byte payload + 4-byte CAN ID).
- Push to lock-free SPSC ring buffer (capacity 256 frames). Drop + increment
  `dropped_rx_` counter if ring is full.
- Reader thread is the sole writer; control loop is the sole reader. No mutex needed.

**Tx thread** (one per `CANBus`, SCHED_FIFO priority 75):
- Blocks on a condition variable waiting for the mutex-protected `std::deque<can_frame>`.
- Drains queue via `::write()`.
- On `ENOBUFS`: exponential back-off retry (100 µs base, up to 5 attempts), then drop
  + log.

**`CANBus` public interface:**
```cpp
class CANBus {
public:
    explicit CANBus(const std::string& ifname);
    ~CANBus();                               // stops threads, closes socket
    void send(const can_frame& frame);       // enqueue for Tx thread
    bool recv(can_frame& out_frame);         // drain one from Rx ring; false = empty
    const std::string& ifname() const;
    uint64_t dropped_rx() const;
    bool is_open() const;
};
```

### 4.3 BusManager (`bus_manager.cpp/.h`)

Owns `std::unordered_map<std::string, std::unique_ptr<CANBus>>`. On construction, opens
one `CANBus` per unique channel string in the joint config. Interfaces not present in the
system (`ip link show` check at open time) are logged as warnings; joints on that bus
are marked `OFFLINE`.

```cpp
class BusManager {
public:
    explicit BusManager(const std::vector<JointConfig>& joints);
    void send(const std::string& ifname, const can_frame& frame);
    // Drain all buses; call on_frame for each received frame (bus, frame)
    void drain_all(std::function<void(const std::string&, const can_frame&)> on_frame);
    bool bus_open(const std::string& ifname) const;
    std::unordered_map<std::string, uint64_t> dropped_rx_per_bus() const;
};
```

### 4.4 Control Loop (`control_loop.cpp/.h`)

Runs at **200 Hz**, SCHED_FIFO priority 80, CPU 0, via `LoopFunc`.

**Per-tick sequence (5 ms budget):**
1. Dequeue and process pending UDP commands from the command queue (non-blocking).
2. `bus_manager.drain_all()` — dispatch received frames to `Actuator::on_rx_frame()`.
3. For each `Actuator` in `ENABLED` or `CALIBRATING`:
   - `actuator.tick()` — compute and send PDO2 (or NMT for calibration steps).
4. Watchdog feed: for each `Actuator` in `IDLE`, if `now - last_hb_sent > 200ms`,
   send `Protocol::make_heartbeat(device_id)` via `bus_manager.send()`.
5. Snapshot all `ActuatorState` values into the telemetry buffer (RW-lock write, < 10 µs).

**Invariants:**
- No blocking I/O in the control loop itself (all sockets in separate threads).
- Tx queue lock held for < 1 µs per `send()` call.
- Overruns logged to stderr with microsecond precision; threshold: 2 ms.

### 4.5 Actuator Class (`actuator.cpp/.h`)

Owns an immutable `JointConfig` and a mutable `ActuatorState`.

**State enum:**
```cpp
enum class JointState { OFFLINE, IDLE, ENABLED, CALIBRATING, FAULT };
```

**State transitions:**
```
OFFLINE      → IDLE         on HEARTBEAT received (device alive on bus)
IDLE         → ENABLED      on SetMode(POSITION / TORQUE / VELOCITY / CURRENT) command
IDLE         → CALIBRATING  on Calibrate command + NMT MODE_CALIBRATION sent
ENABLED      → IDLE         on Disable / Damp command
ENABLED      → FAULT        on EMCY received or watchdog timeout (no PDO4 within 2× watchdog_timeout)
CALIBRATING  → IDLE         on HEARTBEAT confirming MODE_IDLE (calibration complete)
CALIBRATING  → FAULT        on EMCY with ERROR_CALIBRATION_ERROR or 45× SDO timeout
FAULT        → IDLE         on ClearError command (sends NMT MODE_IDLE)
```

**Position frame conversion:**
```
wire_position    = display_position + joint_config.position_offset
display_position = wire_position    - joint_config.position_offset
```
Wire position is the raw output-shaft value carried on CAN (gear ratio already applied by
firmware). Display position is the user-facing and frontend-visible value.

**`Actuator::tick()`** (called from control loop when ENABLED):
- POSITION mode: `bus_manager.send(ifname, Protocol::make_pdo2(device_id, target_wire, 0.0f))`
- TORQUE/VELOCITY/CURRENT modes: SDO write to param target + PDO2 for watchdog reset.
- Update `last_command_time`.

**`Actuator::on_rx_frame(ifname, frame)`** (called from `drain_all` dispatch):
- PDO4: update `position_measured`, `velocity_measured`, `pdo4_timestamp`.
- HEARTBEAT (5 bytes): update `mode_measured`, `error_measured`; drive state transitions.
- EMCY (4 bytes): set `error_measured`, transition to FAULT.
- TRANSMIT_SDO (read response, byte 0 = `0x43`): resolve pending SDO read future.
- TRANSMIT_SDO (write ACK, 1 byte = `0x60`): resolve pending SDO write confirmation.
- PDO3 (calibration status): update `calibration_progress` (0.0–1.0) for telemetry.

**`Actuator::apply_config()`** (startup or on `APPLY_CONFIG` command):
Writes all `JointConfig` fields to ESC RAM via SDO in sequence. Each write waits for the
firmware `0x60` ACK with 15 ms timeout. Fields written:
`pole_pairs`, `torque_constant`, `cpr`, `phase_order` (SDO 0x10C: +1 or -1 from
`phase_inversion`), `position_kp`, `velocity_kp`, `velocity_ki`, `torque_limit`,
`velocity_limit`, `watchdog_timeout`, `fast_frame_frequency`, `gear_ratio`.
Total worst-case: ~22 writes × 15 ms = 330 ms. Not called during real-time operation.

### 4.6 UDP Protocol (`udp_server.cpp/.h`)

**Ports:**
- **9001** — Daemon listens for commands from Python (request/response RPC).
- **9000** — Daemon pushes telemetry to Python (no request needed).

Both ports configurable via `--cmd-port` and `--telemetry-port` CLI args.

**Message envelope:**
```json
{ "type": "TYPE_STRING", "id": "correlation-id", ...type-specific fields... }
```

**Command messages (Python → Daemon on 9001):**

| `type` | Additional fields | Response `type` |
|---|---|---|
| `PING` | — | `PONG` + `daemon_version` |
| `GET_STATE` | `joint_name` | `STATE` + `state` object |
| `GET_ALL_STATES` | — | `ALL_STATES` + `states` map |
| `SET_MODE` | `joint_name`, `mode` | `ACK` or `ERROR` |
| `SET_ALL_MODE` | `mode` | `ACK` |
| `SET_POSITION` | `joint_name`, `position_rad` | `ACK` |
| `SET_TORQUE` | `joint_name`, `torque_nm` | `ACK` |
| `SET_VELOCITY` | `joint_name`, `velocity_rads` | `ACK` |
| `APPLY_CONFIG` | `joint_name` | `ACK` |
| `APPLY_ALL_CONFIGS` | — | `ACK` |
| `CALIBRATE` | `joint_name` | `ACK` (starts async) |
| `STORE_FLASH` | `joint_name` | `ACK` |
| `LOAD_FLASH` | `joint_name` | `ACK` |
| `CLEAR_ERROR` | `joint_name` | `ACK` |
| `RELOAD_CONFIG` | `config_path` | `ACK` |
| `SHUTDOWN` | — | `ACK`, then graceful stop |

All responses include the same `id` as the request for correlation. Unrecognized `joint_name`
returns `ERROR` with a descriptive message.

**Telemetry push (Daemon → Python on 9000):**

```json
{
  "type": "TELEMETRY",
  "seq": 12345,
  "timestamp_us": 1716652800000000,
  "joints": {
    "left_hip_yaw": {
      "state": "ENABLED",
      "position": 0.123,
      "velocity": -0.01,
      "torque": 1.2,
      "current": 3.1,
      "mode": "POSITION",
      "error": 0,
      "bus_voltage": 23.8,
      "calibration_progress": null
    }
  },
  "bus_health": {
    "can_left_leg":  {"open": true, "dropped_rx": 0},
    "can_right_leg": {"open": true, "dropped_rx": 0},
    "can_left_arm":  {"open": true, "dropped_rx": 0},
    "can_right_arm": {"open": true, "dropped_rx": 0}
  }
}
```

Field notes:
- `position`: display-frame radians (`wire_position - position_offset`)
- `torque`: estimated Nm (`i_q_measured × torque_constant × |gear_ratio|`)
- `bus_voltage`: float or JSON `null` if not yet read
- `calibration_progress`: 0.0–1.0 during `CALIBRATING`, `null` otherwise
- `state`: `JointState` enum as string

**`GET_STATE` SDO-on-demand:** If PDO4 data is stale (> 50 ms old), the daemon performs
up to 5 SDO reads (position, velocity, current, mode, error) before responding. If PDO4
is fresh, it returns cached PDO4 position/velocity + HEARTBEAT-sourced mode/error.

**Threading model:**
- UDP receive thread (SCHED_OTHER): listen on 9001, parse JSON, push to thread-safe
  `std::queue<Command>` consumed by the control loop at tick start.
- UDP publish thread (SCHED_OTHER): read telemetry snapshot buffer (RW-lock read),
  serialize to JSON, send to 9000 at configured rate.

### 4.7 Config Loading (`config_loader.cpp/.h`)

Uses `nlohmann/json` (fetched at build time via `FetchContent`).

```cpp
struct JointConfig {
    std::string name;
    std::string can_channel;
    int         device_id;
    bool        phase_inversion;      // SDO 0x10C: true → -1, false → +1
    float       position_offset;      // display = wire - offset
    float       gear_ratio;
    float       position_kp;
    float       velocity_kp;
    float       velocity_ki;
    float       torque_limit;
    float       velocity_limit;
    float       position_limit_min;   // optional, default -∞
    float       position_limit_max;   // optional, default +∞
    int         pole_pairs;
    float       torque_constant;
    int         cpr;
    int         watchdog_timeout_ms;
    int         fast_frame_frequency;
};

struct RobotConfig {
    std::string                robot_name;
    int                        telemetry_hz;   // default 10
    std::vector<JointConfig>   joints;
    std::unordered_map<std::string, std::string> can_serial_map;  // optional
};

RobotConfig ConfigLoader::load(const std::string& json_path);
```

Fail fast on missing required fields; log warning for optional fields.

### 4.8 Graceful Shutdown (`signal_handler.cpp/.h`)

`std::atomic<bool> g_stop_requested{false}` in `signal_handler.h`.

SIGINT / SIGTERM handler: set `g_stop_requested = true`. Second SIGINT within 2 s:
set `g_force_exit = true`.

Main thread shutdown sequence on first signal:
1. Log "Shutdown requested — damping all joints."
2. For each joint in ENABLED or CALIBRATING: `bus_manager.send(NMT MODE_DAMPING)`.
3. Wait up to 500 ms for HEARTBEAT ACK confirming MODE_DAMPING on each joint.
4. Log "All joints damped — setting IDLE."
5. For each joint: `bus_manager.send(NMT MODE_IDLE)`.
6. Wait up to 200 ms.
7. `control_loop.stop()` → join all threads → close all sockets.
8. Exit 0.

On `g_force_exit`: skip steps 3–6, proceed directly to 7.

### 4.9 Build System (`daemon/CMakeLists.txt`)

```cmake
cmake_minimum_required(VERSION 3.16)
project(humanoid_daemon CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

include(FetchContent)
FetchContent_Declare(nlohmann_json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG        v3.11.3)
FetchContent_MakeAvailable(nlohmann_json)

find_package(Threads REQUIRED)

add_executable(humanoid_daemon
  src/main.cpp
  src/can_bus.cpp
  src/bus_manager.cpp
  src/actuator.cpp
  src/robot.cpp
  src/control_loop.cpp
  src/udp_server.cpp
  src/config_loader.cpp
  src/signal_handler.cpp
)

target_include_directories(humanoid_daemon PRIVATE src)
target_link_libraries(humanoid_daemon PRIVATE
  Threads::Threads
  nlohmann_json::nlohmann_json
)
target_compile_options(humanoid_daemon PRIVATE -Wall -Wextra -O2)
install(TARGETS humanoid_daemon DESTINATION bin)
```

**Build:**
```bash
cd humanoid-studio/daemon
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
# Grant real-time scheduling without root:
sudo setcap cap_sys_nice+ep humanoid_daemon
```

---

## 5. Python Backend Changes

### 5.1 Delete (Phase 4 only — after daemon is fully operational)

| File | Replaced by |
|---|---|
| `backend/humanoid/can_bus.py` | `daemon/src/can_bus.cpp` + `bus_manager.cpp` |
| `backend/humanoid/actuator.py` | `daemon/src/actuator.cpp` |
| `backend/humanoid/robot.py` | `daemon/src/robot.cpp` |
| `backend/humanoid/can_monitor.py` | Passive PDO4 handled in daemon control loop |
| `backend/humanoid/recoil_protocol.py` | `daemon/src/protocol.h` |

Do NOT delete during Phases 1–3 (old Python CAN stack remains fully operational).

### 5.2 Add (Phase 4)

**`backend/humanoid/daemon_client.py`** — Async UDP client mirroring the `Robot`/`Actuator`
public API:
```python
class DaemonClient:
    async def ping(self) -> str
    async def get_state(self, joint_name: str) -> ActuatorState
    async def get_all_states(self) -> dict[str, ActuatorState]
    async def set_mode(self, joint_name: str, mode: Mode)
    async def set_all_mode(self, mode: Mode)
    async def set_position(self, joint_name: str, position_rad: float)
    async def set_torque(self, joint_name: str, torque_nm: float)
    async def apply_config(self, joint_name: str)
    async def apply_all_configs(self)
    async def calibrate(self, joint_name: str)
    async def store_flash(self, joint_name: str)
    async def load_flash(self, joint_name: str)
    async def subscribe_telemetry(self) -> AsyncIterator[TelemetryFrame]
```
All command methods: send JSON to 127.0.0.1:9001, await response matching `id`. Timeout: 5 s.
`subscribe_telemetry()`: open UDP socket on 9000, yield `TelemetryFrame` as they arrive.

**`backend/humanoid/daemon_process.py`** — Subprocess lifecycle manager:
```python
class DaemonProcess:
    def __init__(self, binary_path: str, config_path: str, cmd_port=9001, tel_port=9000)
    async def start(self)     # spawn, wait for PING response within 5 s
    async def stop(self)      # send SHUTDOWN, wait for process exit (3 s)
    def is_running(self) -> bool
```
Default binary path: `../daemon/build/humanoid_daemon` relative to `backend/`.

### 5.3 Modify (Phase 4)

**`backend/main.py` lifespan:** Replace `Robot.open()` + `CanMonitor` + watchdog task with:
```python
daemon = DaemonProcess(binary_path, config_path)
await daemon.start()
client = DaemonClient()
await client.ping()
```
The `robot` variable in route dependencies becomes `client` (identical interface).

**`backend/main.py` `/ws/telemetry`:** Replace parallel SDO poll loop with:
```python
async for frame in client.subscribe_telemetry():
    await ws.send_json(frame.to_dict())
```

**`backend/main.py` `/ws/control`:** Replace `robot.set_positions()` with
`client.set_position()` per joint. Rate-limiting logic unchanged.

**`backend/api/routes_*.py`:** Replace `robot.*` / `actuator.*` calls with `client.*`
equivalents. Method names are designed to match for near-drop-in substitution.

**`backend/requirements.txt`:** Remove `python-can`. No new deps (stdlib asyncio UDP
via `asyncio.DatagramProtocol` is sufficient).

---

## 6. Migration Plan

### Phase 1 — Daemon Skeleton ✅ Complete
**Goal:** Buildable binary with config loading, UDP server, signal handling. No CAN.

1. Create `daemon/` with `CMakeLists.txt`.
2. Implement `config_loader.cpp` — parse `humanoid_lite.json` → `RobotConfig`.
3. Implement `signal_handler.cpp` — `g_stop_requested` + SIGINT handler.
4. Implement `udp_server.cpp` — listen on 9001, respond to `PING`, stub all other types.
5. Implement `main.cpp` — load config, start UDP server thread, block until stop.
6. **Verify:** `./humanoid_daemon --config ../configs/humanoid_lite.json` starts, responds
   to Python `PING`, shuts down cleanly on Ctrl+C.

**Python:** No changes.

### Phase 2 — CAN Layer ✅ Complete
**Goal:** Daemon opens CAN sockets, receives frames, performs SDO reads/writes.

1. Implement `can_bus.cpp` — socket, reader thread, epoll, Tx queue.
2. Implement `bus_manager.cpp` — multi-bus routing.
3. Implement `protocol.h` — all frame constructors and parsers.
4. Implement `actuator.cpp` — SDO state machine, `on_rx_frame()`, state transitions.
5. Wire `GET_STATE` command — triggers SDO reads, returns `ActuatorState` snapshot.
6. **Verify:** `GET_STATE` on a powered ESC returns correct position, mode, error.

**Python:** No changes. Old stack fully operational.

### Phase 3 — Control Loop + Full Command Set ✅ Complete
**Goal:** Daemon runs the 200 Hz loop, feeds watchdog, handles all commands, pushes telemetry.

1. Implement `loop_func.h` — period timing, SCHED_FIFO, CPU affinity, overrun logging.
2. Implement `control_loop.cpp` — 200 Hz tick sequence, command queue drain, telemetry snapshot.
3. Implement `robot.cpp` — joint initialization, `apply_all_configs()`, bulk enable/disable/damp.
4. Complete `udp_server.cpp` — all remaining command types + telemetry push on port 9000.
5. **Verify:**
   - Enable a joint, command a position, confirm PDO4 feedback updates in `GET_STATE`.
   - `candump can_left_leg` shows 200 Hz PDO2 frames from daemon, 5 Hz HEARTBEAT for IDLE joints.
   - `candump` shows zero Python-originated frames when daemon owns the bus.

**Python:** No changes.

### Phase 4 — Python Migration ✅ Complete
**Goal:** FastAPI routes talk to `DaemonClient`; old Python CAN stack deleted.

1. Implement `daemon_client.py` and `daemon_process.py`.
2. Modify `main.py` lifespan.
3. Update all API routes.
4. Update `/ws/telemetry` endpoint.
5. Remove `python-can` from `requirements.txt`.
6. Delete `can_bus.py`, `actuator.py`, `robot.py`, `can_monitor.py`, `recoil_protocol.py`.
   **Note:** `can_bus.py`, `actuator.py`, and `recoil_protocol.py` were kept for type definitions
   and enums re-used by `flash.py`. No live CAN operations remain in Python.
7. **Verify:** Full end-to-end — Electron app connects, telemetry streams at 10 Hz, 100 Hz
   position commands work, calibration completes, Flash Wizard unaffected (uses OpenOCD).

### Phase 5 — Integration Testing + Performance Validation ✅ Complete
**Goal:** No regressions; latency and throughput improvements confirmed.

1. 10-minute soak test: 22 joints ENABLED, 10 Hz telemetry, 100 Hz position commands.
   Zero dropped frames on `candump`.
2. Control loop jitter: log overruns for 1000 ticks; expect < 0.1% overruns at 200 Hz.
3. Watchdog: confirm no spurious DAMPING transitions during normal operation.
4. Fault propagation: EMCY from any ESC reaches frontend within 100 ms.
5. Shutdown: all joints IDLE within 1 s of Ctrl+C.
6. UDP stress: 1000 `SET_POSITION` calls/s; no response drops.
7. Update `HANDOFF.md` with confirmed architecture and any anomalies.

---

## 7. Verification Checklist

Minimum before declaring Phase 3 complete:

- [x] `./humanoid_daemon --config ../configs/humanoid_lite.json` starts without error
- [x] `GET_ALL_STATES` returns all 22 joints (`OFFLINE` for unpowered, `IDLE` for powered)
- [x] `APPLY_ALL_CONFIGS` writes all params to ESCs without SDO timeout
- [x] `SET_MODE left_hip_yaw POSITION` → joint transitions to `ENABLED`
- [x] `SET_POSITION left_hip_yaw 0.5` → PDO2 frame visible on `candump can_left_leg`
- [x] PDO4 feedback received → `position_measured` updates in `GET_STATE`
- [x] HEARTBEAT feed frames visible every 200 ms on `candump` for IDLE joints
- [x] EMCY received from ESC → joint transitions to `FAULT` in `GET_STATE`
- [x] Ctrl+C → all joints `DAMPING` within 500 ms, then `IDLE` within 1 s, exit 0
- [x] No Python-originated CAN frames on `candump` while daemon is running
