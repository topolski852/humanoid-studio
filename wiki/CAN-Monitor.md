# CAN Monitor

The CAN Monitor page shows real-time health information for all four CAN buses. Bus health data comes from daemon telemetry (the daemon's `bus_health` JSON pushed on port 9000). A passive traffic sniffer runs independently of the robot connection state to populate the traffic table.

---

## Interface status indicators

Each bus panel shows a status. The daemon reports whether each CAN socket is open and tracks TX drops; the Python backend checks `/sys/class/net/{ifname}/operstate` for final UP/DOWN resolution. The app maps these to the following labels:

| App label | Meaning |
|---|---|
| **Healthy** | Interface is UP and the daemon has an open socket on it. |
| **Down** | The Linux interface exists but is not in UP state — not brought up, or dropped after a USB disconnect. |
| **Not Configured** | The interface name (e.g., `can_left_arm`) does not exist in `/sys/class/net/`. The udev rule has not been written or the adapter is not plugged in. |

Note: The kernel CAN error state machine states (ERROR-WARNING, ERROR-PASSIVE, BUS-OFF) are not currently reported by the daemon — TEC/REC counters are not surfaced. If a bus enters BUS-OFF you will see it go DOWN rather than a specific BUS-OFF label.

### What to do for each state

**Healthy:** No action needed.

**Degraded:** Check CAN termination resistors. Check cable quality (especially any connector that was recently handled). The bus is still working but the error rate may increase.

**High Errors:** Stop sending commands. Check all CAN-H and CAN-L wiring. Check for a shorted wire or a damaged ESC. Running in this state for extended periods can cause BUS-OFF.

**Bus Off:** The interface must be reset. Unplug and replug the USB-CAN adapter, or run:
```bash
sudo ip link set can_left_leg down
sudo ip link set can_left_leg up
```

**Down:** Bring the interface up. If it was brought up before but dropped, the adapter may have been unplugged or the USB connection dropped. Replug the adapter — the udev rule will bring it back up automatically.

**Not Configured:** Open the CAN Setup page and assign the adapter for this limb.

---

## Message rate

Each bus panel shows the current message rate in msg/s from daemon telemetry. At 6 joints × 100 Hz TX_PDO4, a fully active left leg or right leg bus carries approximately 600 msg/s of motor broadcasts plus host command frames.

A sudden drop to 0 msg/s indicates motors powered down, a CAN chain break, or the USB adapter losing contact.

---

## Traffic table

The traffic table shows every unique CAN arbitration ID seen in the last 10 seconds, along with:

| Column | Meaning |
|---|---|
| Arb ID | Raw arbitration ID in hex. Decodes as (func_code << 7) \| node_id. |
| Node ID | The device ID extracted from the arbitration ID (bits 6:0). |
| Joint name | The joint name from the config, if any joint maps to this (bus, node_id) pair. Unknown IDs appear without a name. |
| Func | The function code name (TX_PDO4, RX_SDO, HEARTBEAT, etc.). |
| Rate | Message rate in msg/s, computed over a 5-second sliding window. |
| Last data | The most recent 8 bytes of frame data, shown as hex pairs. |
| Age | Seconds since the last frame was seen. Entries older than 10 seconds are hidden. |
| Position | For TX_PDO2 and TX_PDO4 frames: the decoded position in degrees. |
| Velocity | For TX_PDO2 and TX_PDO4 frames: the decoded velocity in deg/s. |

Position and velocity in the traffic table are raw output-shaft values decoded from the frame payload, without any position_offset subtraction. These differ from the values shown in the motor tab, which apply the position_offset to produce the calibrated joint angle.

### Reading the traffic for a single motor

A healthy powered motor on the left leg (e.g., ID 1) produces TX_PDO4 frames at ~100 Hz. You will see one row with:
- Arb ID: `0x480` (= 0x9 << 7 | 1 = 0x480)
- Func: `TX_PDO4`
- Rate: ~100 msg/s
- Position and velocity updating every 10 ms

When you send a position command, additional rows appear:
- `0x601` (RX_SDO from host, if parameter writes were made)
- `0x680` (FLASH, if store-to-flash was triggered)
- `0x481` (RX_PDO1 ping, if connect/ping was run)

---

## Drop log

The drop log records every time a CAN interface changes state: UP → DOWN, DOWN → UP, or UNCONFIGURED → UP.

Each event records:
- **Interface:** which bus (e.g., `can_left_leg`)
- **Timestamp:** ISO 8601 UTC
- **Event:** `down` or `up`

The drop log is in-memory and resets when the backend restarts. It is intended for diagnosing intermittent USB cable problems and CAN chain reliability issues during a session.

---

## Debugging guide

### Interface UP but 0 msg/s

Motors are not powered or the CAN chain is broken. Possible causes:

1. Main power supply is off — the ESC needs motor power to boot and transmit
2. Missing termination resistor — the left leg silent bus during development was traced to this
3. CAN-H and CAN-L wires are swapped at one connector
4. A broken connector in the CAN chain between the adapter and the ESC

### Interface goes DOWN intermittently

USB cable quality problem. The kernel drops the SocketCAN interface when the USB device disconnects. Replace the USB cable with a high-quality data cable. Short cables (< 0.5 m) are more reliable.

### Only one motor is responding on a multi-motor bus

Other motors may not be powered, may have wrong CAN IDs, or may have a CAN chain fault (broken wire between that ESC and the next one in the chain).

Check each motor individually by physically disconnecting them from the chain and testing one at a time.

### Motor appears on unexpected CAN ID

If a motor shows up in the traffic table with a different ID than expected (e.g., ID 14 instead of ID 1), the Flash wizard previously wrote a different ID to Flash. You can correct this via the Flash Wizard (re-flash with the correct ID) or by writing the correct ID via SDO and storing to Flash.

### Random garbage error values in motor tab (historical)

This was caused by the SDO race condition where two concurrent SDO reads to the same motor consumed each other's responses. It has been fixed in the C++ daemon using per-motor mutex/condition-variable mailboxes. If you see this symptom, upgrade to the current version.
