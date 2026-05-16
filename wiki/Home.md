# Humanoid Studio

Humanoid Studio is a desktop application for commissioning, calibrating, and operating the [Berkeley Humanoid Lite](https://github.com/HybridRobotics/Berkeley-Humanoid-Lite) robot. It provides a graphical interface for communicating with Recoil Motor Controller ESC boards over CAN bus, replacing the original collection of one-shot Python scripts with a persistent, state-aware control environment.

[Screenshot: Dashboard view — add screenshot here after first full hardware session]

---

## What it can do right now

| Feature | Status |
|---|---|
| Real-time telemetry (position, velocity, torque, mode, error) for all 22 joints | Working |
| Per-joint motor enable / idle / e-stop controls | Working |
| Position jog with slider, +/− buttons, presets, and Run-to-Position command | Working |
| Sine wave test with configurable frequency, amplitude, and center offset | Working |
| CAN bus health monitoring (packet rates, error states, drop events) | Working |
| USB-CAN adapter assignment and persistent udev rule generation | Working |
| Flux-offset (electrical) calibration | Working |
| Position limit calibration via hardstop recording | Working |
| ESC config read-back, parameter tuning, and flash persistence | Working |
| Flash wizard for firmware programming and motor commissioning | Working |
| Robot configuration editor (JSON + table view) | Working |
| Passive motor detection (shows motors that are broadcasting without connecting) | Working |

## What is planned but not yet implemented

| Feature | Notes |
|---|---|
| Right leg / arm calibration | Hardware not connected during development; code is ready |
| Multi-motor coordinated motion | Single-joint control only in current UI |
| Limb enable/disable as a group | Planned for Phase 2 |
| Hold-position mode after re-enable | No automatic re-enable from IDLE |
| 3D URDF visualization with live joint angles | Phase 3; STL meshes exist at `Berkeley-Humanoid-Lite/source/berkeley_humanoid_lite_assets/` |
| RL policy deployment interface | Phase 4 |
| Windows / macOS support | Linux only; no Windows/Mac support is planned |

---

## Hardware requirements

- **Berkeley Humanoid Lite robot** (or hardware with compatible B-G431B-ESC1 ESC boards)
- **B-G431B-ESC1 ESC boards** running Recoil Motor Controller firmware (build date ≥ 2025-02-26)
- **USB-CAN adapters** — one per limb, up to four simultaneously (left/right leg, left/right arm)
- **Linux computer** — Ubuntu 22.04 or 24.04 recommended; other distros may need minor dependency adjustments
- **USB-C cables** — cable quality matters significantly. A marginal cable causes the interface to go DOWN intermittently, which appears as a CAN bus drop event and motor disconnection. Use cables rated for data, not just charging.
- **ST-LINK/V2 or built-in STLINK-V3** — only required for the Flash Wizard; the B-G431B-ESC1 board has an integrated STLINK-V3
- **ARM toolchain** (`arm-none-eabi-gcc`, `make`, `openocd`) — only required for the Flash Wizard

---

## Quick links

**Getting started**
- [Installation](Installation) — install from zero, run in dev mode, troubleshoot the GLIBCXX error
- [Hardware Setup](Hardware-Setup) — CAN adapter assignment, udev rules, interface bring-up

**Using the app**
- [Motor Configuration](Motor-Configuration) — electrical calibration, position calibration, sign vs phase inversion
- [Flash Wizard](Flash-Wizard) — when and how to reflash an ESC
- [CAN Monitor](CAN-Monitor) — bus health indicators, traffic table, drop log

**Technical reference**
- [CAN Bus Architecture](CAN-Bus-Architecture) — protocol details, frame formats, SDO race condition fix
- [Codebase Architecture](Codebase-Architecture) — developer orientation, data flow, design decisions

**Other**
- [Troubleshooting](Troubleshooting) — organized by symptom
- [Roadmap](Roadmap) — current status and future plans

---

## Project status

Active development. The left leg (6 joints) has been fully commissioned: electrical calibration complete, telemetry clean, position jog tested. The right leg, left arm, and right arm exist in the configuration file but have not yet been calibrated — the hardware was not connected during the development sessions that built this codebase.

The core communication stack is stable and production-quality. The UI is functional and has been tested end-to-end with real hardware. Flash wizard, position calibration, and ESC config sync have all been verified working.
