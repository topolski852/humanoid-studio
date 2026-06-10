# Codebase Architecture

This page is a developer reference. It explains the repository structure, how the three related codebases relate to each other, and the key design decisions made when building Humanoid Studio.

---

## Repository structure

```
humanoid-studio/
├── app/                    Electron + React frontend
│   ├── electron/
│   │   ├── main.js         Electron main process; spawns daemon then backend, creates window
│   │   └── preload.js      Context bridge (exposes window.electron.platform only)
│   ├── src/
│   │   ├── App.jsx         HashRouter, tab state management
│   │   ├── api.js          Fetch wrapper for all REST calls to localhost:8765
│   │   ├── constants.js    Shared BUSES constant (name/limb/label for all four CAN buses)
│   │   ├── main.jsx        React 18 createRoot entry point
│   │   ├── index.css       Tailwind directives + scrollbar + component classes
│   │   ├── context/
│   │   │   └── TelemetryContext.jsx  WebSocket state distribution (states, canHealth, passiveTelemetry)
│   │   ├── hooks/
│   │   │   └── useControlWebSocket.js  Low-latency WebSocket hook for position commands
│   │   ├── components/
│   │   │   ├── MotorControlsPanel.jsx  Enable/Idle/E-STOP, position jog, sine wave
│   │   │   ├── MotorCalibrationPanel.jsx  Flux cal, position cal, Flash Wizard link
│   │   │   ├── MotorConfigPanel.jsx  Tune tab: PID gains, limits, SDO sync
│   │   │   ├── AutoTunePanel.jsx     Auto tab: step test, metrics, gain suggestion
│   │   │   ├── MotorTab.jsx          Route /motor/:jointName, tab container
│   │   │   ├── MotorCard.jsx         Dashboard grid card per joint
│   │   │   ├── FlashWizard.jsx       Flash modal: step strip, log, direction confirm
│   │   │   ├── TelemetryTable.jsx    Left-column live data display
│   │   │   ├── Sidebar.jsx           Navigation, Connect/Disconnect button, motor list
│   │   │   ├── RobotDiagram.jsx      Schematic robot view (body segments, joint dots)
│   │   │   ├── TabBar.jsx            Tab strip with closeable motor tabs
│   │   │   ├── StatusDot.jsx         Green pulsing or grey static indicator
│   │   │   └── ErrorLogPanel.jsx     In-session error log
│   │   ├── pages/
│   │   │   ├── Dashboard.jsx         Responsive grid of motor cards
│   │   │   ├── CanMonitor.jsx        CAN health + traffic display (daemon telemetry)
│   │   │   ├── CanSetup.jsx          USB adapter assignment UI
│   │   │   ├── EscSetup.jsx          ESC scan / ping page
│   │   │   ├── RobotConfig.jsx       Config table + JSON editor
│   │   │   └── Settings.jsx          App settings (config path, theme)
│   │   └── utils/
│   │       └── canDisplay.js         CAN ID decode helpers for UI display
│   ├── index.html          Vite entry; CSP allows localhost:8765; Google Fonts
│   ├── package.json        Electron 30, React 18, Vite 5, Tailwind 3
│   ├── vite.config.js      base: './' for Electron file:// compat; port 5173
│   └── tailwind.config.js  Custom tokens: surface, accent, online, danger
├── backend/                Python FastAPI backend (thin proxy layer)
│   ├── main.py             FastAPI app on localhost:8765; WebSocket /ws/telemetry
│   ├── requirements.txt    fastapi, uvicorn[standard], python-can, pydantic>=2.7, websockets
│   ├── humanoid/           Hardware bridge library
│   │   ├── daemon_client.py   UDP bridge to C++ daemon (ports 9001/9000); replaces Robot+CanMonitor
│   │   ├── robot_config.py    Pydantic models: JointConfig, RobotConfig, PositionLimits
│   │   ├── recoil_protocol.py CAN ID decode; function codes; telemetry decode (used by debug route)
│   │   ├── can_adapter.py     USB-CAN adapter discovery, assignment, udev rule writing
│   │   ├── can_bus.py         SocketCAN transport (retained for Flash Wizard use only)
│   │   ├── actuator.py        Single-joint controller (retained for Flash Wizard use only)
│   │   ├── flash.py           Flash wizard 3-pass state machine; conf.h patching
│   │   └── settings.py        App settings load/save (~/.config/humanoid-studio/)
│   └── api/
│       ├── routes_motors.py   GET/POST /motors/{joint_name}/*
│       ├── routes_robot.py    GET/PUT /robot/config, POST /robot/connect|disconnect
│       ├── routes_devices.py  GET /devices, CAN interface info, USB devices
│       ├── routes_flash.py    POST /flash/start, GET /flash/status, /flash/step etc.
│       ├── routes_settings.py GET/PUT /settings
│       └── routes_debug.py    Debug utilities
├── daemon/                 C++ real-time CAN daemon (owns all SocketCAN interfaces)
│   ├── Makefile            C++17, -O2; builds to daemon/build/humanoid_daemon
│   ├── third_party/        nlohmann/json header (vendored)
│   ├── build/
│   │   └── humanoid_daemon Compiled binary (run cd daemon && make -j$(nproc))
│   └── src/
│       ├── main.cpp             Entry point; parse args, load config, start daemon
│       ├── can/
│       │   ├── socket_can.cpp/.h      Single CAN interface: raw socket, epoll, Tx queue
│       │   ├── can_bus_manager.cpp/.h Owns N SocketCAN instances; drain_all(); routes frames
│       │   └── generic_listener.cpp/.h  Passive frame listener (for CAN Monitor, passive PDO poll)
│       ├── config/
│       │   └── config_loader.cpp/.h   Load humanoid_lite.json → vector<JointConfig>
│       ├── control/
│       │   ├── robot.cpp/.h     Multi-joint coordinator: load config, fan-out commands
│       │   └── control_loop.hpp 200 Hz SCHED_FIFO real-time loop utility
│       ├── ipc/
│       │   ├── udp_server.cpp/.h      UDP RPC server: receive JSON commands, dispatch to Robot
│       │   └── udp_broadcaster.cpp/.h Push telemetry JSON to port 9000 at configured Hz
│       └── motor/
│           ├── actuator.cpp/.h        Per-joint state machine, config apply, position conversions
│           └── recoil_protocol.hpp    CAN frame constants, param IDs, enums (Mode, Function, ErrorCode)
├── configs/
│   ├── humanoid_lite.json  22-joint robot configuration (flat schema)
│   └── 99-humanoid-can.rules  Stable udev rules written by the CAN Setup page
└── wiki/                   This documentation
```

---

## The three related codebases

Humanoid Studio relates to two upstream codebases that are **read-only reference material** — never modified:

### 1. Recoil-Motor-Controller-BESC

Location: `/home/nse/Recoil-Motor-Controller-BESC/Recoil-Motor-Controller-B-G431B-ESC1/`

The STM32G431 firmware that runs on each ESC. Humanoid Studio has no control over this code at runtime — it can only communicate with the firmware via the CAN protocol and reflash it via the Flash Wizard.

Key files referenced during development:
- `Core/Inc/motor_controller_conf.h` — all build-time parameters (CAN ID, motor profile, LOAD flags)
- `Core/Src/motor_controller.c` — FOC control loop, mode state machine, PDO handlers
- `Core/Src/app.c` — ISR routing, TX_PDO4 timer callback
- `Core/Src/position_controller.c` — position offset pipeline

### 2. Berkeley-Humanoid-Lite

Location: `/home/nse/Berkeley-Humanoid-Lite/source/berkeley_humanoid_lite_lowlevel/`

The original Python scripts and C++ lowlevel code for the robot project. These were studied to understand the CAN protocol, joint mapping, and calibration values. They were not used as a dependency — Humanoid Studio has its own library and daemon.

Key issues with the original scripts that the rewrite fixes:
- Synchronous blocking I/O (no concurrent joint updates)
- SDO race condition (waiter registered after transmit)
- Non-standard JSON (Python's `Infinity` is not valid JSON)
- Hardcoded to write only one joint in `write_configurations.py`
- 100 ms sleep between each of ~15 SDO writes per joint (33 seconds for all joints; no sleep needed)

The C++ Berkeley lowlevel was analyzed as read-only reference for the daemon architecture (real-time loop pattern, CAN frame encoding, SCHED_FIFO usage). No Berkeley code was copied.

### 3. humanoid-studio (this project)

A full rewrite of the low-level library plus a complete desktop application. The library (`backend/humanoid/`) is a thin proxy bridge; the C++ daemon owns all real-time CAN operations.

---

## Process architecture

Humanoid Studio runs as three cooperating processes:

```
┌─────────────────────────────────────────────────────────────────┐
│  Electron (app/electron/main.js)                                │
│  - Spawns daemon first, waits for PING response                 │
│  - Then spawns Python backend, waits for /devices health check  │
│  - Shows window; on quit sends SIGTERM to both children         │
└────────────────┬────────────────────────────────────────────────┘
                 │  spawns
    ┌────────────▼─────────────────┐    ┌──────────────────────────────┐
    │  Python FastAPI (port 8765)  │    │  C++ Daemon                  │
    │  backend/main.py             │◄──►│  daemon/build/humanoid_daemon │
    │  - REST API + WebSocket      │UDP │  - 200 Hz CAN control loop   │
    │  - Thin proxy to daemon      │9001│  - All 4 SocketCAN interfaces │
    │  - Flash Wizard (CAN bypass) │9000│  - Watchdog feed             │
    └──────────────────────────────┘    └──────────────────────────────┘
```

**Startup sequence:**
1. Electron spawns `humanoid_daemon --config configs/humanoid_lite.json`
2. Electron pings daemon on port 9001 every 500 ms (up to 10 s)
3. Once daemon responds, Electron spawns `python3 main.py`
4. Python polls `http://localhost:8765/devices` every 500 ms (up to 20 s)
5. Window opens

**UDP ports:**
- **9001**: Python → daemon commands (JSON request/response)
- **9000**: daemon → Python telemetry push (10 Hz default, max 100 Hz)

---

## Backend architecture

### Why FastAPI

FastAPI provides automatic JSON schema validation via Pydantic, native async/await support, WebSocket support, and automatic OpenAPI docs at `/docs`. The alternative (Flask, Django) would require significantly more boilerplate for async and WebSocket support.

### DaemonClient — the sole CAN path

`backend/humanoid/daemon_client.py` is the only Python object that communicates with hardware. It:
- Sends JSON commands to port 9001 and awaits responses with the same `id` field
- Listens on port 9000 for telemetry push frames from the daemon
- Maintains a thread-safe snapshot of the latest joint states and bus health
- Exposes a `get_interface_stats()` method that synthesises the CAN Monitor data from the telemetry snapshot

`DaemonActuatorProxy` is a per-joint facade that gives API routes the same async interface they previously used to talk to `Actuator` objects, but routes all calls through `DaemonClient`.

All FastAPI routes use `app.state.client` (a `DaemonClient` instance). There is no direct CAN access from Python for normal operation.

### Flash Wizard — CAN bypass

The Flash Wizard (`flash.py`) is the single exception to the DaemonClient rule. It uses OpenOCD over SWD for firmware reflash, not CAN. However, during the OpenOCD procedure it also needs a raw CAN socket for the post-flash parameter write.

When the Flash Wizard starts, it calls `client.daemon_shutdown()` to stop the daemon's CAN control loop. The daemon process stays alive (it continues responding to PING) but does not transmit any CAN frames. The Flash Wizard then opens its own `python-can` socket directly.

After the Flash Wizard completes, the user reconnects via the sidebar, which triggers a daemon restart.

`can_bus.py` and `actuator.py` are kept in the backend for this reason alone. They are not used by any other route.

### WebSocket telemetry

The `/ws/telemetry` WebSocket endpoint broadcasts two types of messages:

1. **Motor telemetry** at 20 Hz: reads the cached joint state snapshot from `DaemonClient` (no CAN traffic generated). Sends `{"connected": bool, "actuators": {joint_name: state_dict, ...}}`.

2. **CAN health** every 200 ms (every 4th motor frame): sends `{"type": "can_health", "interfaces": [...]}` from `daemon_client.get_interface_stats()`.

3. **CAN drop events** immediately when they occur, forwarded from `daemon_client.pop_drop_events()`.

Because the daemon pushes telemetry at 10 Hz, the Python WS telemetry re-broadcasts cached data at 20 Hz with no additional CAN traffic.

### No watchdog task in Python

The daemon feeds the HEARTBEAT watchdog for all IDLE joints at 5 Hz from its control loop. Python no longer has a background watchdog task. This was a critical improvement: in the old architecture, if the Python asyncio loop was busy (e.g., during a Flash Wizard operation), the watchdog could fire.

---

## C++ Daemon architecture

### Why a separate C++ daemon

Python's GIL and asyncio cannot guarantee < 1 ms CAN frame latency at 200 Hz across 22 joints on 4 buses simultaneously. The daemon:
- Runs its control loop at SCHED_FIFO priority 80 (real-time, not preempted by Python GC)
- Uses one reader thread per CAN bus with epoll (no blocking the control loop on slow buses)
- Feeds the firmware watchdog from the control loop itself (not from Python)
- Owns all SocketCAN sockets exclusively (no contention with Python)

### Threading model

```
Main thread           — config load, signal handling, graceful shutdown
Control loop (200 Hz) — drain Rx rings, tick all actuators, watchdog feed, telemetry snapshot
  CPU 0, SCHED_FIFO prio 80
Reader threads (×4)   — one per CAN bus; epoll → SPSC ring buffer
  CPU 0, SCHED_FIFO prio 70
Tx threads (×4)       — drain Tx deque via ::write(); retry on ENOBUFS
  CPU 0, SCHED_FIFO prio 75
UDP command thread    — UdpServer on 9001; parse JSON; dispatch Robot methods directly
  SCHED_OTHER
UDP telemetry thread  — UdpBroadcaster; read snapshot; push JSON to 9000 at configured Hz
  SCHED_OTHER
```

### Actuator state machine

Each joint has a `JointState` enum: `OFFLINE`, `IDLE`, `ENABLED`, `CALIBRATING`, `FAULT`.

- `OFFLINE` → `IDLE`: HEARTBEAT received from device (device is alive)
- `IDLE` → `ENABLED`: user `SET_MODE POSITION/TORQUE/VELOCITY` command
- `ENABLED` → `FAULT`: EMCY frame received or watchdog timeout
- `FAULT` → `IDLE`: user `CLEAR_ERROR` + `SET_MODE IDLE` command

### Position frame conversion

The firmware tracks raw output-shaft radians with respect to an ESC-internal reference. The config stores a `position_offset` per joint that converts between the firmware's frame and the display frame:

```
wire_position    = display_position + joint_config.position_offset
display_position = wire_position    - joint_config.position_offset
```

This conversion is applied in the daemon and also in the Python flash wizard for consistency.

---

## Frontend architecture

### Electron shell

`app/electron/main.js` first spawns the daemon binary, then spawns `python3 main.py` from the `backend/` directory as a child process. It polls `http://localhost:8765/devices` every 500 ms (up to 40 attempts = 20 seconds) before showing the window. On backend crash, it shows a Restart/Quit dialog.

On quit, it sends SIGTERM to both child processes and follows up with SIGKILL after 3 seconds if the processes have not exited.

**Note:** When launching from a VS Code terminal, VS Code sets `ELECTRON_RUN_AS_NODE=1` in the environment. The `npm run dev` script uses `cross-env ELECTRON_RUN_AS_NODE=` (empty string) to clear this variable before launching Electron; without this, `require('electron')` returns undefined and the app fails silently.

### HashRouter for Electron

Electron serves the app as a `file://` URL in production. Browser history `pushState` does not work with `file://` URLs because there is no server to handle the URL. The app uses React Router's `HashRouter`, which encodes routes as URL fragments: `file://...#/motor/left_hip_roll_joint`.

### TelemetryContext

`TelemetryContext.jsx` maintains the WebSocket connection to `/ws/telemetry`. It auto-reconnects every 2 seconds on disconnect. It distributes `{states, robotConnected, wsConnected}` to all consuming components via React Context. Motor cards, the dashboard, and the robot diagram all read from this context.

### joint_name as the primary key

Every motor is identified by its `joint_name` string (e.g., `"left_hip_roll_joint"`). This is the only globally unique identifier — `can_id` integers repeat across buses. The REST API uses `joint_name` in URLs (`/motors/left_hip_roll_joint`), the frontend routes use it (`/motor/left_hip_roll_joint`), and tab IDs are `motor-left_hip_roll_joint`.

---

## Data flow

```
C++ daemon control loop (200 Hz, CPU 0, SCHED_FIFO):
  └─► drain_all() reads Rx ring buffers from all 4 CAN buses
  └─► actuator.on_rx_frame(): PDO4 updates position_measured/velocity_measured
                               HEARTBEAT updates mode/error, drives state machine
                               EMCY sets FAULT state
  └─► actuator.tick(): ENABLED joints → send PDO2 position command
                        IDLE joints  → send HEARTBEAT every 200 ms (5 Hz watchdog)
  └─► copy ActuatorState snapshots to telemetry buffer (RW lock)

Daemon telemetry thread (10 Hz):
  └─► read telemetry snapshot buffer
  └─► send JSON {"type": "TELEMETRY", "joints": {...}, "bus_health": {...}} to 127.0.0.1:9000

Python DaemonClient telemetry listener:
  └─► recv() on port 9000
  └─► parse TELEMETRY frame → update _joint_states, _bus_health (lock)

Python WebSocket /ws/telemetry (20 Hz):
  └─► read _joint_states snapshot from DaemonClient (no CAN I/O)
  └─► build ActuatorState per joint
  └─► {"connected": true, "actuators": {...}} sent to all WebSocket clients
  └─► every 4th frame: {"type": "can_health", "interfaces": get_interface_stats()}

Frontend:
  TelemetryContext.jsx receives JSON, stores in React state
  └─► Dashboard: reads states, renders motor cards with live position and mode
  └─► MotorTab: reads state for specific joint, renders TelemetryTable
  └─► RobotDiagram: reads all states, renders joint status dots
  └─► CanMonitor: reads can_health messages, displays per-bus stats and joint lists
```

---

## Key design decisions

### Library rewrite instead of wrapping the original

The original Berkeley Humanoid Lite Python scripts use blocking synchronous I/O and cannot support concurrent joint updates. Wrapping them in an async layer would require per-joint threads and a complex synchronization scheme. The rewrite was cleaner and took less time than wrapping would have.

### (bus_name, can_id) addressing

`can_id` integers are not globally unique — left_leg ID 1 and left_arm ID 1 are different motors. Using `joint_name` as the primary key in the REST API, and resolving to `(bus_name, can_id)` via the robot config, avoids this ambiguity entirely.

### Explicit connect, not automatic

The backend creates a `DaemonClient` on startup but does not attempt CAN operations until the user clicks Connect. This allows the app to start and display the config on machines without any CAN hardware attached, and avoids errors on startup when adapters are not plugged in. The daemon is always running (it is spawned by Electron); Connect merely signals it to apply config and begin telemetry.

### 20 Hz WebSocket telemetry (not faster)

50 Hz was tried with the old Python stack and caused USB-CAN adapter instability. With the daemon architecture the bottleneck is gone (daemon pushes at 10 Hz; Python broadcasts cached data at 20 Hz with no CAN traffic). 20 Hz is kept as the WebSocket rate to avoid unnecessary React re-renders.

### Position limit: None serializes as null, not ±infinity

The original Berkeley scripts used Python's `float('inf')` and serialized it as the JSON literal `Infinity` — which is not valid JSON and causes many parsers to fail. The Pydantic models use `None` for "no limit", which serializes as JSON `null`. When sending to the firmware, `None` is converted to `±100.0 rad` (≈ ±5730°) — large enough to be practically unlimited. The firmware's position controller computes `(upper + lower) / 2` as the midpoint for limit enforcement; using actual ±infinity would produce NaN.

### Daemon is non-negotiable for position control; Python-can is retained for flash only

The daemon eliminates GIL jitter from the control path. The python-can dependency is kept solely because the Flash Wizard performs a raw SDO write after reflash, and spawning a second daemon instance for that single write would add more complexity than the dependency costs. If the flash path is ever refactored to go through the daemon, python-can can be removed.
