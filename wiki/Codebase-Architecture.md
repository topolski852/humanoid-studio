# Codebase Architecture

This page is a developer reference. It explains the repository structure, how the three related codebases relate to each other, and the key design decisions made when building Humanoid Studio.

---

## Repository structure

```
humanoid-studio/
├── app/                    Electron + React frontend
│   ├── electron/
│   │   ├── main.js         Electron main process; spawns backend, creates window
│   │   └── preload.js      Context bridge (exposes window.electron.platform only)
│   ├── src/
│   │   ├── App.jsx         HashRouter, tab state management
│   │   ├── api.js          Fetch wrapper for all REST calls to localhost:8765
│   │   ├── main.jsx        React 18 createRoot entry point
│   │   ├── index.css       Tailwind directives + scrollbar + component classes
│   │   ├── context/
│   │   │   └── TelemetryContext.jsx  WebSocket state distribution
│   │   ├── components/
│   │   │   ├── MotorControlsPanel.jsx  Enable/Idle/E-STOP, position jog, sine wave
│   │   │   ├── MotorCalibrationPanel.jsx  Flux cal, position cal, Flash Wizard link
│   │   │   ├── MotorConfigPanel.jsx  Tune tab: PID gains, limits, SDO sync
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
│   │   │   ├── CanMonitor.jsx        CAN health + traffic display
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
├── backend/                Python FastAPI backend
│   ├── main.py             FastAPI app on localhost:8765; WebSocket /ws/telemetry
│   ├── requirements.txt    fastapi, uvicorn[standard], python-can, pydantic>=2.7, websockets
│   ├── humanoid/           Hardware abstraction library (rewritten from scratch)
│   │   ├── can_bus.py      Async SocketCAN transport; SDO read/write; waiter dispatch
│   │   ├── actuator.py     Single-joint controller; calibration; apply_config
│   │   ├── robot.py        Multi-joint coordinator; opens one CANBus per channel
│   │   ├── robot_config.py Pydantic models: JointConfig, RobotConfig, PositionLimits
│   │   ├── recoil_protocol.py  CAN ID decode; function codes; telemetry decode
│   │   ├── can_adapter.py  USB-CAN adapter discovery, assignment, udev rule writing
│   │   ├── can_monitor.py  Background sysfs poll + traffic sniffer for all 4 buses
│   │   ├── flash.py        Flash wizard 3-pass state machine; conf.h patching
│   │   └── settings.py     App settings load/save (~/.config/humanoid-studio/)
│   └── api/
│       ├── routes_motors.py   GET/POST /motors/{joint_name}/*
│       ├── routes_robot.py    GET/PUT /robot/config, POST /robot/connect|disconnect
│       ├── routes_devices.py  GET /devices, CAN interface info, USB devices
│       ├── routes_flash.py    POST /flash/start, GET /flash/status, /flash/step etc.
│       ├── routes_settings.py GET/PUT /settings
│       └── routes_debug.py    Debug utilities
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

The original Python scripts written for the robot project. These were studied to understand the CAN protocol, joint mapping, and calibration values. They were not used as a dependency — Humanoid Studio has its own library.

Key issues with the original scripts that the rewrite fixes:
- Synchronous blocking I/O (no concurrent joint updates)
- SDO race condition (waiter registered after transmit)
- Non-standard JSON (Python's `Infinity` is not valid JSON)
- Hardcoded to write only one joint in `write_configurations.py`
- 100 ms sleep between each of ~15 SDO writes per joint (33 seconds for all joints; no sleep needed)

### 3. humanoid-studio (this project)

A full rewrite of the low-level library plus a complete desktop application. The library (`backend/humanoid/`) is designed for concurrent async operation and is thoroughly separated from the UI.

---

## Backend architecture

### Why FastAPI

FastAPI provides automatic JSON schema validation via Pydantic, native async/await support, WebSocket support, and automatic OpenAPI docs at `/docs`. The alternative (Flask, Django) would require significantly more boilerplate for async and WebSocket support.

### The asyncio concurrency model

The backend is a single Python process with a single asyncio event loop. All CAN I/O runs in this loop via `python-can`'s SocketCAN interface.

The receive path uses a `ThreadPoolExecutor` with one thread per CAN bus. This thread calls `raw_bus.recv(timeout=0.01)` — a blocking call — and returns to the asyncio loop when a frame arrives. This avoids blocking the event loop while waiting for frames.

The transmit path uses an `asyncio.Lock` so concurrent sends never interleave frame bytes on the wire.

### Per-device SDO lock

The firmware's SDO read response (TX_SDO, func_code 0xB) contains only 4 raw bytes of the parameter value with no parameter ID echo. If two coroutines concurrently send SDO read requests to the same motor, each registers a waiter for `(device_id, TRANSMIT_SDO)`. Whichever response arrives first will be consumed by whichever waiter was registered first — which may not be the coroutine that sent the corresponding request.

The fix is a per-device asyncio lock (`_device_sdo_locks: dict[int, asyncio.Lock]`). Before any SDO read or write, the coroutine acquires the lock for that device ID. This serializes all SDO traffic per motor while allowing concurrent operations to different motors.

### WebSocket telemetry broadcasting at 20 Hz

The `/ws/telemetry` WebSocket endpoint broadcasts two types of messages:

1. **Motor telemetry** at 20 Hz: calls `robot.get_all_states()` which reads position, velocity, torque, current, mode, error, and bus voltage from each motor via individual SDO reads. Applies velocity EMA smoothing (α=0.2). Sends `{"connected": bool, "actuators": {joint_name: state_dict, ...}}`.

2. **CAN health** every 200 ms (every 4th motor frame): sends `{"type": "can_health", "interfaces": [...]}` from the CanMonitor.

3. **CAN drop events** immediately when they occur: forwarded as `{"type": "can_drop_event", ...}`.

### Background watchdog task

A separate asyncio task (`_watchdog_task`) runs at 5 Hz (200 ms interval) and calls `robot.feed_all_watchdogs()`. This sends a HEARTBEAT frame to every connected motor. It runs independently of WebSocket clients — motors stay alive even when no browser tab is open.

---

## Frontend architecture

### Electron shell

`app/electron/main.js` spawns `python3 main.py` from the `backend/` directory as a child process. It polls `http://localhost:8765/devices` every 500 ms (up to 40 attempts = 20 seconds) before showing the window. On backend crash, it shows a Restart/Quit dialog.

On quit, it sends SIGTERM to the backend process and follows up with SIGKILL after 3 seconds if the process has not exited.

### HashRouter for Electron

Electron serves the app as a `file://` URL in production. Browser history `pushState` does not work with `file://` URLs because there is no server to handle the URL. The app uses React Router's `HashRouter`, which encodes routes as URL fragments: `file://...#/motor/left_hip_roll_joint`.

### TelemetryContext

`TelemetryContext.jsx` maintains the WebSocket connection to `/ws/telemetry`. It auto-reconnects every 2 seconds on disconnect. It distributes `{states, robotConnected, wsConnected}` to all consuming components via React Context. Motor cards, the dashboard, and the robot diagram all read from this context.

### joint_name as the primary key

Every motor is identified by its `joint_name` string (e.g., `"left_hip_roll_joint"`). This is the only globally unique identifier — `can_id` integers repeat across buses. The REST API uses `joint_name` in URLs (`/motors/left_hip_roll_joint`), the frontend routes use it (`/motor/left_hip_roll_joint`), and tab IDs are `motor-left_hip_roll_joint`.

---

## Data flow

```
ESC broadcasts TX_PDO4 at ~100 Hz per motor (up to 995 msg/s per bus)
  └─► SocketCAN kernel socket receives frame
  └─► can_monitor.py _sniff_bus() reads frame in executor thread
  └─► recoil_protocol.decode_arb_id() extracts (node_id, func_code)
  └─► recoil_protocol.decode_telemetry() extracts (position_rad, velocity_rads)
  └─► CanMonitor stores per-(bus, device_id) telemetry and timestamps

WebSocket client connected:
  main.py ws_telemetry() runs every 50 ms (20 Hz)
  └─► robot.get_all_states() fires N SDO reads per joint (position, velocity, mode, error, voltage)
  └─► Each SDO read: transmit request, wait for TRANSMIT_SDO response, decode 4 bytes
  └─► ActuatorState built per joint; position_offset subtracted to match firmware getter
  └─► EMA smoothing applied to velocity
  └─► {"connected": true, "actuators": {...}} sent to all WebSocket clients

Frontend:
  TelemetryContext.jsx receives JSON, stores in React state
  └─► Dashboard: reads states, renders motor cards with live position and mode
  └─► MotorTab: reads state for specific joint, renders TelemetryTable
  └─► RobotDiagram: reads all states, renders joint status dots
```

---

## Key design decisions

### Library rewrite instead of wrapping the original

The original Berkeley Humanoid Lite Python scripts use blocking synchronous I/O and cannot support concurrent joint updates. Wrapping them in an async layer would require per-joint threads and a complex synchronization scheme. The rewrite was cleaner and took less time than wrapping would have.

### (bus_name, can_id) addressing

`can_id` integers are not globally unique — left_leg ID 1 and left_arm ID 1 are different motors. Using `joint_name` as the primary key in the REST API, and resolving to `(bus_name, can_id)` via the robot config, avoids this ambiguity entirely.

### Explicit connect, not automatic

The backend creates a `Robot(config)` object on startup but does not open the CAN buses. The user must click Connect in the Sidebar. This allows the app to start and display the config on machines without any CAN hardware attached, and avoids errors on startup when adapters are not plugged in.

### 20 Hz WebSocket telemetry (not faster)

50 Hz was tried and caused USB-CAN adapter instability. The adapter's internal buffer filled faster than the kernel could drain it. 20 Hz (one telemetry frame per 50 ms, covering all 22 joints) is a comfortable rate that leaves headroom for command frames and does not stress the USB path.

### Position limit: None serializes as null, not ±infinity

The original Berkeley scripts used Python's `float('inf')` and serialized it as the JSON literal `Infinity` — which is not valid JSON and causes many parsers to fail. The Pydantic models use `None` for "no limit", which serializes as JSON `null`. When sending to the firmware, `None` is converted to `±100.0 rad` (≈ ±5730°) — large enough to be practically unlimited. The firmware's position controller computes `(upper + lower) / 2` as the midpoint for limit enforcement; using actual ±infinity would produce NaN.
