# Humanoid Studio

A desktop application for controlling, calibrating, and monitoring the [Berkeley Humanoid Lite](https://github.com/HybridRobotics/Berkeley-Humanoid-Lite) robot. Humanoid Studio provides a complete hardware interface layer — connecting to 22 joints across 4 CAN buses, running motor calibration procedures, flashing ESC firmware, running Auto-Tune step tests, and displaying live telemetry — without the blocking I/O and race conditions of the original Python scripts.

<!--
Screenshot placeholder — add a screenshot of the Dashboard here once the robot is fully assembled.
-->

---

## Hardware requirements

- **Linux** (Ubuntu 22.04 or 24.04 recommended) — required for SocketCAN
- **Berkeley Humanoid Lite** robot hardware — B-G431B-ESC1 ESCs running Recoil firmware
- **USB-CAN adapters** — up to 4 (one per limb: left leg, right leg, left arm, right arm)
- **ST-LINK** — only needed for the Flash Wizard (firmware flashing)

---

## Quick start

### From release (no compilation)

Download the AppImage from the [releases page](https://github.com/topolski852/humanoid-studio/releases/latest), then:

```bash
sudo apt-get install libfuse2 can-utils iproute2
pip install fastapi "uvicorn[standard]" python-can "pydantic>=2.7" websockets
chmod +x "Humanoid Studio-0.1.0.AppImage"
./"Humanoid Studio-0.1.0.AppImage"
```

### From source (development)

```bash
# 1. Build the C++ daemon
cd humanoid-studio/daemon
make -j$(nproc)

# 2. Install Python dependencies
cd ../backend
pip install -r requirements.txt

# 3. Install Node dependencies
cd ../app
npm install

# 4. Start the app (dev mode — opens Electron window)
npm run dev
```

The Electron window opens automatically once the backend is ready. The backend API is available at `http://localhost:8765/docs`.

---

## Documentation

Full documentation is in the [GitHub Wiki](https://github.com/topolski852/humanoid-studio/wiki):

- [Installation](https://github.com/topolski852/humanoid-studio/wiki/Installation) — release install, developer setup, troubleshooting first-run issues
- [Hardware Setup](https://github.com/topolski852/humanoid-studio/wiki/Hardware-Setup) — CAN adapters, udev rules, wiring, termination resistors
- [Motor Configuration](https://github.com/topolski852/humanoid-studio/wiki/Motor-Configuration) — calibration procedures, sign vs phase inversion
- [Flash Wizard](https://github.com/topolski852/humanoid-studio/wiki/Flash-Wizard) — 3-pass firmware flashing procedure
- [CAN Monitor](https://github.com/topolski852/humanoid-studio/wiki/CAN-Monitor) — interface status, traffic table, drop log
- [CAN Bus Architecture](https://github.com/topolski852/humanoid-studio/wiki/CAN-Bus-Architecture) — Recoil protocol, frame formats, SDO, NMT
- [Codebase Architecture](https://github.com/topolski852/humanoid-studio/wiki/Codebase-Architecture) — repo structure, backend design, data flow
- [Troubleshooting](https://github.com/topolski852/humanoid-studio/wiki/Troubleshooting) — known issues and fixes from hardware testing
- [Roadmap](https://github.com/topolski852/humanoid-studio/wiki/Roadmap) — what works, what's in progress, and future plans
- [Future Improvements](https://github.com/topolski852/humanoid-studio/wiki/Future-Improvements) — non-critical improvement backlog

---

## Related projects

- [Berkeley Humanoid Lite](https://github.com/HybridRobotics/Berkeley-Humanoid-Lite) — the robot hardware and original Python control scripts
- [Recoil Motor Controller](https://github.com/rxdu/Recoil-Motor-Controller-B-G431B-ESC1) — the STM32G431 ESC firmware
