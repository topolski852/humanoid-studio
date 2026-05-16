# Roadmap

This page describes the current state of the project, what is actively being worked on, and what is planned for future phases. All items marked as working have been confirmed on real hardware.

---

## Currently working

These features are functional and have been tested with physical hardware.

**CAN communication**
- All four SocketCAN buses (`can_left_leg`, `can_right_leg`, `can_left_arm`, `can_right_arm`) open and receive traffic
- CAN Monitor displays live message rates, per-ID traffic tables, and drop log
- Per-device SDO lock prevents the race condition that caused garbage error values in concurrent reads
- HEARTBEAT watchdog feeding at 5 Hz keeps motors alive without a WebSocket client

**Motor control — left leg**
- All six left leg joints connect, respond to SDO reads, and report telemetry at 20 Hz
- Position mode enable, position jog (slider, numeric entry, +/− step), and sine wave work
- E-STOP sends Mode 0 (DISABLED) and blocks further commands until the error is cleared
- IDLE transition returns the motor to no-torque state

**Calibration — left hip roll joint**
- Flux offset calibration (electrical calibration) runs via NMT MODE_CALIBRATION and saves to Flash
- Position limit calibration (hardstop recording) correctly computes the new offset from the live authoritative value in the ESC, not a cached frontend value. Repeat calibration is safe and does not compound
- Position limits confirmed: min = −10°, max = 90°

**Flash Wizard**
- Full 3-pass procedure confirmed working end to end on the left hip roll joint
- Phase inversion detection and direction verification loop work correctly
- ENOBUFS (TX queue overflow) after motor boot is handled with 2-second settling delay and 4-attempt retry
- `flux_offset` is correctly saved to Flash and persists across power cycles

**Config and settings**
- `configs/humanoid_lite.json` loads at startup and saves on calibration
- App Settings page stores config path and loads correctly on restart
- JSON editor in the Robot Config page allows direct config editing with validation

**Application shell**
- Electron + React frontend starts cleanly from `npm run dev` on Linux
- Backend spawns from Electron main process; app waits for health check before showing window
- On backend crash, Restart/Quit dialog appears
- Tab system with closeable motor tabs works correctly for left leg joints

---

## In progress

**Right leg — not yet calibrated**

The right leg CAN bus (`can_right_leg`) is wired and the interfaces come up correctly. All six right leg joints appear in the config and show up on the Dashboard. They have not been through the full calibration procedure (flux offset + position limits) because hardware access was focused on the left leg first. The joints will respond to SDO reads but position control has not been verified.

**Flash Wizard — right leg and arm boards**

The Flash Wizard has been confirmed on one ESC. The remaining 21 ESCs (right leg, both arms) need to be flashed through the full 3-pass procedure. There are no software changes expected — the wizard handles any CAN ID and motor profile already.

**Position limit calibration — all joints except left hip roll**

All joints except `left_hip_roll_joint` show `null` position limits in the config. The mechanical hardstop calibration procedure needs to be run per joint once the robot is fully assembled and each joint can be moved through its range safely.

---

## Phase 2 — multi-limb operation

Phase 2 covers extending reliable single-joint operation to all 22 joints simultaneously.

**Multi-motor position control**

The backend already supports concurrent operations across different joints (per-device SDO lock). The missing piece is a frontend control surface for commanding multiple joints at once. Planned additions:
- Limb enable/disable: single button to enable or idle all joints on one CAN bus
- Hold position: command all joints to hold their current position (read position, send it back as target)
- Joint group presets: save and recall positions for common poses (e.g., stand, crouch, zero)

**Full arm calibration**

Both arms (`can_left_arm`, `can_right_arm`) need flux and position limit calibration. The arm joints use the same motor type and protocol as the legs. No software changes are expected — the existing Calibration tab handles them.

**Config sync verification**

After all 22 joints are calibrated, a verification pass will confirm that `configs/humanoid_lite.json` correctly reflects all 22 joints' `electrical_offset`, `position_offset`, and `position_limits`. A config audit tool in the Robot Config page is planned: highlight joints with null limits or zero offsets that look unset.

---

## Phase 3 — 3D visualization

Phase 3 adds a live 3D view of the robot using Three.js with the actual URDF STL meshes.

**URDF loader with STL meshes**

The Berkeley Humanoid Lite assets include STL files for every link at:

```
/home/nse/Berkeley-Humanoid-Lite/source/berkeley_humanoid_lite_assets/data/robots/berkeley_humanoid/berkeley_humanoid_lite/meshes/
```

Available meshes include: `base_visual.stl`, `leg_left_hip_roll_visual.stl`, `leg_left_hip_yaw_visual.stl`, `leg_left_hip_pitch_visual.stl`, `leg_left_knee_pitch_visual.stl`, `leg_left_ankle_pitch_visual.stl`, `leg_left_ankle_roll_visual.stl`, and corresponding right leg, left arm, right arm, and hand meshes.

The planned implementation loads these STL files via Three.js STLLoader, parents them according to the URDF joint tree, and applies the live joint angles from `TelemetryContext` at 20 Hz to update joint rotations in real time.

**Robot view page**

A new top-level page (accessible from the sidebar) will show the 3D robot. Planned features:
- Orbit camera (drag to rotate, scroll to zoom, right-drag to pan)
- Per-joint color coding by mode (POSITION = green, IDLE = grey, DISABLED = red, DAMPING = orange)
- Click on a joint dot to open its motor tab
- Toggle between solid mesh view and wireframe + joint axes view

**Joint angle overlay**

The 3D view will show numeric joint angle readouts positioned near each joint. A toggle switches between degrees and radians.

---

## Phase 4 — locomotion and policy

Phase 4 integrates Humanoid Studio with the reinforcement learning stack.

**RL policy runner**

The Berkeley Humanoid Lite project includes trained locomotion policies. Phase 4 adds a "Policy" page that:
- Loads a policy checkpoint (ONNX or TorchScript format)
- Reads IMU + joint state observations at the correct frequency
- Runs inference and sends joint position targets

**Locomotion controller**

A PD impedance controller running in the backend will sit between the policy output and the raw position commands, allowing safe operation with configurable gains and velocity limits.

**Teleoperation**

Teleoperation support via keyboard or gamepad for direct joint control during development and debugging. This is lower priority than the policy runner.

---

## Known limitations

**Linux only**

The application requires Linux because it depends on SocketCAN, which is a Linux kernel subsystem. There is no equivalent on macOS or Windows. The Electron and React frontend could run on other platforms, but the backend CAN interface is Linux-specific. A future simulation mode could work on other platforms.

**Requires physical hardware to operate**

The backend starts without hardware, and the frontend can display the config and settings. But all motor control, calibration, and monitoring features require the actual CAN adapters and powered ESCs. There is no simulation or mock mode.

**Flash Wizard requires an ST-LINK connection**

The Flash Wizard uses openocd over the SWD interface exposed by the B-G431B-ESC1's built-in STLINK-V3 debugger. Firmware cannot be flashed over CAN alone (the Recoil firmware does not implement a CAN bootloader).

**Single config file**

The app currently manages one robot configuration at a time (`configs/humanoid_lite.json`). The Settings page allows pointing to a different path, but switching configurations requires a backend restart. Multi-robot support or config profiles are not planned.

**No authentication or access control**

The FastAPI backend listens on `localhost:8765` with no authentication. This is intentional for a local desktop app. Do not expose this port to a network.
