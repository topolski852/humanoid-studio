# Humanoid Studio

A desktop application for controlling, calibrating, and monitoring the [Berkeley Humanoid Lite](https://github.com/HybridRobotics/Berkeley-Humanoid-Lite) robot. Humanoid Studio provides a complete hardware interface layer — connecting to 22 joints across 4 CAN buses, running motor calibration procedures, flashing ESC firmware, and displaying live telemetry — without the blocking I/O and race conditions of the original Python scripts.

<!--
Screenshot placeholder — add a screenshot of the Dashboard here once the robot is fully assembled.
-->

---

## Hardware requirements

- **Linux** (Ubuntu 20.04 or later) — required for SocketCAN
- **Berkeley Humanoid Lite** robot hardware — B-G431B-ESC1 ESCs running Recoil firmware
- **USB-CAN adapters** — up to 4 (one per limb: left leg, right leg, left arm, right arm)
- **ST-LINK** — only needed for the Flash Wizard (firmware flashing)

---

## Quick start

```bash
# 1. Install Python dependencies
cd humanoid-studio/backend
pip install fastapi "uvicorn[standard]" python-can "pydantic>=2.7.0" websockets

# 2. Install Node dependencies
cd ../app
npm install

# 3. Start the app (dev mode — opens Electron window)
npm run dev
```

The Electron window opens automatically once the backend is ready. The backend API is available at `http://localhost:8765/docs`.

---

## Documentation

Full documentation is in the [GitHub Wiki](../../wiki):

- [Installation](../../wiki/Installation) — prerequisites, setup, and troubleshooting first-run issues
- [Hardware Setup](../../wiki/Hardware-Setup) — CAN adapters, udev rules, wiring, termination resistors
- [Motor Configuration](../../wiki/Motor-Configuration) — calibration procedures, sign vs phase inversion
- [Flash Wizard](../../wiki/Flash-Wizard) — 3-pass firmware flashing procedure
- [CAN Monitor](../../wiki/CAN-Monitor) — interface status, traffic table, drop log
- [CAN Bus Architecture](../../wiki/CAN-Bus-Architecture) — Recoil protocol, frame formats, SDO, NMT
- [Codebase Architecture](../../wiki/Codebase-Architecture) — repo structure, backend design, data flow
- [Troubleshooting](../../wiki/Troubleshooting) — known issues and fixes from hardware testing
- [Roadmap](../../wiki/Roadmap) — what works, what's in progress, and future plans

---

## Related projects

- [Berkeley Humanoid Lite](https://github.com/HybridRobotics/Berkeley-Humanoid-Lite) — the robot hardware and original Python control scripts
- [Recoil Motor Controller](https://github.com/rxdu/Recoil-Motor-Controller-B-G431B-ESC1) — the STM32G431 ESC firmware
