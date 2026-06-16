# Troubleshooting

Organized by symptom. Every issue listed here was encountered during actual development and testing of this project.

---

## App won't start

### Daemon fails to start

Electron spawns the daemon before the Python backend. If the daemon binary is missing or fails, Electron will report "Failed to Start Backend" even though Python is fine.

1. **Daemon not built (dev mode only):** In dev mode Electron looks for `daemon/build/humanoid_daemon`. Build it first:
   ```bash
   cd humanoid-studio/daemon
   make -j$(nproc)
   ```
   In the packaged AppImage the daemon binary is bundled and does not need to be built.

2. **Daemon binary not found:** In dev mode, confirm the binary exists:
   ```bash
   ls -lh humanoid-studio/daemon/build/humanoid_daemon
   ```

3. **Daemon ports already in use:** A previous daemon did not exit cleanly. Free the ports:
   ```bash
   pkill -9 humanoid_daemon 2>/dev/null
   fuser -k 9000/udp 2>/dev/null
   fuser -k 9001/udp 2>/dev/null
   fuser -k 9002/udp 2>/dev/null
   ```

4. **SocketCAN interfaces not up:** The daemon tries to open the CAN interfaces listed in `humanoid_lite.json`. Missing interfaces are logged as warnings and those joints are marked OFFLINE — the daemon still starts. If the daemon exits immediately, check its stderr output for a fatal error (e.g., invalid config JSON).

### Backend fails to start — "Failed to Start Backend"

The Electron window shows an error dialog saying the backend failed to start. Check:

1. **Python is not on PATH:** The Electron main process spawns `python3`. Verify with:
   ```bash
   which python3
   python3 --version
   ```
   If Python is installed via nvm or a non-standard location, make sure it is on PATH before launching the app.

2. **Missing Python dependencies:** The backend imports `fastapi`, `uvicorn`, `can`, `pydantic`, and `websockets`. A missing package causes the process to exit immediately with a traceback. Install:
   ```bash
   cd humanoid-studio/backend
   pip install -r requirements.txt
   ```

3. **Port 8765 already in use:** A previous backend process did not exit cleanly and is still holding the port. Find and kill it:
   ```bash
   sudo lsof -ti :8765 | xargs kill -9
   ```
   Or look for the process:
   ```bash
   ps aux | grep "main.py"
   kill <pid>
   ```

4. **Config file missing or invalid:** The backend tries to load `configs/humanoid_lite.json` at startup. If this file is missing or has invalid JSON, the backend logs a warning but continues. However, if Pydantic validation fails for a field, the backend may exit. Check the terminal output for the specific error.

### GLIBCXX_3.4.29 not found

```
/snap/core20/current/lib/x86_64-linux-gnu/libstdc++.so.6: version 'GLIBCXX_3.4.29' not found
```

This is a Snap-vs-system library version conflict. Electron requires a newer `libstdc++` than what Snap's runtime provides.

**Fix 1 — use nvm instead of snap node:**
```bash
nvm install 20
nvm use 20
npm run dev
```

**Fix 2 — override LD_LIBRARY_PATH:**
```bash
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
npm run dev
```

**Fix 3 — install libstdc++ from system:**
```bash
sudo apt-get install libstdc++6
```

If none of these work, check whether your system's `/usr/lib/x86_64-linux-gnu/libstdc++.so.6` actually provides `GLIBCXX_3.4.29`:
```bash
strings /usr/lib/x86_64-linux-gnu/libstdc++.so.6 | grep GLIBCXX | tail -5
```

---

## CAN issues

### Interface not found after replug

After unplugging and replugging a USB-CAN adapter, the interface may come back as `can0` instead of `can_left_leg`. This means the udev rules were not written, or were written but are not loading.

**Check if the rules file exists:**
```bash
cat /etc/udev/rules.d/99-humanoid-can.rules
```

If the file is missing, open the CAN Setup page in the app and assign the adapter again. The assignment process writes the rules file and triggers `udevadm control --reload-rules`.

If the file exists, force-reload udev rules:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Then unplug and replug the adapter. It should come back as `can_left_leg`.

**Manual fallback (no udev):**
```bash
sudo ip link set can0 down
sudo ip link set can0 name can_left_leg
sudo ip link set can_left_leg type can bitrate 1000000
sudo ip link set can_left_leg up
```

### Interface UP but 0 traffic

The interface is UP (confirmed by `ip link show type can`) but the CAN Monitor shows 0 msg/s and no traffic entries.

Possible causes and checks:

1. **Motors not powered.** The ESC needs main power to boot and transmit. Check that the power supply is on and delivering the correct voltage.

2. **Missing termination resistor.** A CAN bus without end termination may appear completely silent. Check both ends of the CAN chain for 120 Ω termination. This was the cause of the left leg silent bus symptom during development.

3. **CAN-H and CAN-L swapped.** Check the wiring at every connector between the adapter and the first ESC.

4. **Wrong CAN channel.** The adapter may be wired to the right leg bus while you are looking at the left leg panel. Check the physical wiring.

5. **CAN chain break.** A broken wire or connector anywhere in the chain between the adapter and the motors causes silence. Test with one motor physically connected directly to the adapter.

### Only one motor responding on a multi-motor bus

If the traffic table shows only one motor's TX_PDO4 and the others are silent:

1. **Other motors not powered.** Each ESC has its own power connector. Verify all are connected and receiving power.

2. **CAN chain fault.** The CAN chain daisy-chains ESCs. A broken wire between ESC #1 and ESC #2 means ESC #2 and later are invisible. Remove and reconnect each connector to isolate.

3. **Wrong CAN IDs.** If two ESCs have the same CAN ID on the same bus, their frames collide and one or both will appear unreliable. Check each ESC's ID individually by connecting them one at a time.

### Adapter shows wrong limb name

The CAN Setup page shows the adapter assigned to `left_leg` but it is physically wired to the right leg.

Reassign it: open the CAN Setup page, click Unassign, then Assign with the correct limb. The app will rename the interface and update the udev rule.

---

## Motor issues

### Motor goes to DAMPING mode after 1 second

The firmware safety watchdog timer has a 1000 ms timeout. If no PDO2 command or HEARTBEAT frame arrives within 1 second, the firmware transitions the motor to DAMPING mode and sets the `WATCHDOG_TIMEOUT` error bit.

The **daemon** feeds the watchdog at 5 Hz (every 200 ms) from its 200 Hz control loop. It sends HEARTBEAT frames for all IDLE joints and PDO2 commands for all ENABLED joints. If you are seeing watchdog timeouts:

1. **Daemon not running:** Check with `pgrep humanoid_daemon`. If it is not running, start the app or spawn the daemon manually.
2. **Robot not connected:** The daemon feeds the watchdog for joints in IDLE or ENABLED state. Joints remain OFFLINE until a HEARTBEAT is received from the ESC, which requires the motor to be powered. Click Connect in the sidebar to trigger `apply_config` and move joints from OFFLINE to IDLE.
3. **Bus errors interrupting heartbeat:** If the CAN bus is in a bad state, HEARTBEAT frames may not be delivered. Check the CAN Monitor for error counts or run `candump can_left_leg` to confirm daemon traffic is visible.

### CAN Monitor shows all buses as UNKNOWN or grey

The CAN Monitor reads bus health from daemon telemetry. If the daemon has not sent any telemetry yet (e.g., robot not connected), `get_interface_stats()` returns an empty list and the monitor shows "No interfaces".

1. **Daemon not running:** Check `pgrep humanoid_daemon`. Restart the app if the daemon is not present.
2. **Telemetry not flowing:** The daemon pushes telemetry at 10 Hz. If no `TELEMETRY` messages arrive on port 9000, check whether the daemon is in a fault state (run it manually from a terminal to see its stderr).
3. **Config not applied:** Connect the robot via the sidebar to trigger `APPLY_ALL_CONFIGS`, which causes the daemon to begin cycling the state machines and sending telemetry even for unpowered joints.

### Random garbage error values (historical — now fixed)

In earlier builds, the error register displayed in the motor tab would briefly show nonsensical values (WATCHDOG_TIMEOUT, ENCODER_FAULT) on motors with no actual fault. This was the SDO race condition: two concurrent SDO reads to the same motor consumed each other's responses.

This is fixed in the C++ daemon. The daemon's `actuator.cpp` uses a mutex + condition variable mailbox (`sdo_ack_mutex_`, `sdo_ack_cv_`) to serialize SDO transactions per motor. If you see this symptom, you are running an old build. Update to the current version.

### Motor makes grinding noise or vibrates when enabled

Phase inversion. The three-phase wiring order is backward relative to what the firmware expects. The FOC controller's torque vector points in the wrong direction, causing violent oscillation.

**Fix:** Reflash the firmware via the Flash Wizard with the **Invert Phase** option enabled. Do not attempt to operate the motor in this state — the high current draw and mechanical stress can damage the motor or gearbox.

### Motor runs in the wrong direction without grinding

Sign inversion. The encoder is reading backward relative to the position convention.

**Fix:** In the Tune tab, negate the `gear_ratio` value. For example, change `−15.0` to `+15.0`. No reflash is needed. Run the position limit calibration after changing the gear ratio to re-establish the correct zero reference.

### Encoder reads thousands of degrees

The `gear_ratio` in the config does not match the actual gear ratio of the motor assembly. The firmware divides encoder position by `gear_ratio` to produce the output-shaft angle. If `gear_ratio = 1.0` but the actual ratio is 15:1, the output reads 15× the expected value.

Check the config in the Tune tab and set the correct gear ratio. For the MAD 5010 and M6C12 motors in the Berkeley Humanoid Lite, the gear ratio is 15 (positive or negative depending on mounting direction).

### Position jumps when enabling the motor

The electrical offset calibration has not been run, or the stored `electrical_offset` value is incorrect. When the motor first goes into POSITION mode, it computes its current position using the raw encoder reading. If the flux offset is wrong, the FOC controller applies torque in the wrong phase relationship, which can cause the rotor to jump to the nearest stable position.

Run the electrical offset calibration (Flux Calibration button in the Calibration tab) before using position control.

### ESC error persists after clearing

Clicking the **Clear ESC Error** button in the motor panel sends a write of 0 to the `ERROR` register via SDO. This should clear all error bits in the firmware. If the error returns immediately, the underlying fault condition is still present.

Common causes:
- `WATCHDOG_TIMEOUT` returns if the backend is not feeding the watchdog
- `CALIBRATION_ERROR` returns if the electrical calibration was run without motor power
- `INITIALIZATION_ERROR` means the firmware detected a bad Flash config on boot — reflash required

If the ESC enters DISABLED mode via E-STOP (the red E-STOP button in the Controls tab), the firmware also sets `ESTOP` error. The recovery sequence is: Clear ESC Error → Enable (Position). The firmware falls back from DISABLED to IDLE with an error, then allows re-enable after the error is cleared.

---

## Flash wizard issues

### Flash fails immediately — "Compilation failed"

`arm-none-eabi-gcc` or `make` is not installed or not on PATH.

```bash
# Install
sudo apt-get install gcc-arm-none-eabi make

# Verify
arm-none-eabi-gcc --version
make --version
```

Also verify that `openocd` is installed:
```bash
sudo apt-get install openocd
openocd --version
```

### openocd cannot connect to ST-LINK

```
Error: open failed
Error: libusb_open() failed with LIBUSB_ERROR_ACCESS
```

The USB device permission is wrong. Add your user to the `plugdev` group or install the ST-LINK udev rules:

```bash
# Install ST-LINK udev rules
sudo cp /usr/share/openocd/contrib/60-openocd.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
# Then replug the ST-LINK adapter
```

### Power cycle prompt — what to do

After Pass 1 completes, the wizard shows: "Power cycle the ESC now (disconnect/reconnect motor power)."

This is a physical step you must perform:
1. Disconnect the motor power cable from the ESC (not the USB cable — only the main power)
2. Wait 2 seconds
3. Reconnect motor power
4. Click the **Power Cycled** button in the wizard

The firmware enters an infinite loop after Pass 1. The power cycle is required to get back to a bootable state. USB reset alone is not sufficient.

### Motor still wrong direction after reflash

If you reflashed with **Invert Phase** enabled and the motor is still running backward, it may have been a sign inversion issue all along, not a phase inversion. Verify by checking: does the motor run smoothly after enabling? If yes (no grinding, no vibration), it was sign inversion and you should negate the `gear_ratio` in the Tune tab instead.

If the motor still grinds after reflash with both phase settings, there may be a hardware fault (damaged phase winding, broken ESC output stage, or encoder fault).
