# Hardware Setup

Before the app can communicate with motors, the CAN adapters must be physically connected, named correctly by Linux, and configured at 1 Mbit/s. This page explains every step.

---

## CAN Adapter Setup

### Compatible adapters

Any USB-to-CAN adapter that presents as a SocketCAN interface on Linux will work. The project has been developed and tested with adapters that expose themselves as `can0`, `can1`, etc. under the `net/` subsystem. The adapter must support 1 Mbit/s CAN bitrate.

Common compatible adapters:
- PEAK PCAN-USB
- Canable / Canable Pro (SocketCAN firmware)
- USB2CAN

### Why cable quality matters

The CAN adapter's USB cable is a reliability-critical component. A marginal USB cable causes the kernel's USB controller to drop the device intermittently. When this happens, the SocketCAN interface goes DOWN and the app loses contact with all motors on that bus.

Symptoms of a bad cable:
- CAN Monitor shows the interface cycling DOWN → UP repeatedly
- Drop events appear in the drop log even when you are not moving anything
- Telemetry briefly shows motors going OFFLINE and coming back
- In extreme cases, the Python backend throws a `Failed to open can_left_leg` error

Use a cable rated for data transfer, not a charge-only cable. Shorter cables are generally more reliable. If you experience intermittent drops, the cable is the first thing to replace.

### Physical connection to the robot's CAN chain

Each limb has a dedicated CAN bus. Connect one USB-CAN adapter per limb you want to control:

| Adapter assignment | Controls |
|---|---|
| `can_left_leg` | 6 left leg joints (hip roll, hip yaw, hip pitch, knee pitch, ankle pitch, ankle roll) |
| `can_right_leg` | 6 right leg joints (same joint types, right side) |
| `can_left_arm` | 5 left arm joints (shoulder pitch, shoulder roll, shoulder yaw, elbow pitch, wrist yaw) |
| `can_right_arm` | 5 right arm joints (same joint types, right side) |

Connect the adapter's CAN-H wire to the bus CAN-H line and CAN-L to CAN-L. All ESCs on a single limb share the same two-wire CAN bus.

### CAN termination resistors

CAN buses require 120 Ω termination resistors at each physical end of the bus. Without termination:

- The bus may appear UP with low message rates
- Frames reflect and cause bit errors, visible as elevated rx_errors in the CAN Monitor
- Some adapters tolerate this on short buses; others produce a completely silent bus

If a bus shows UP in the CAN Monitor but 0 msg/s when the motors are powered, missing termination is a possible cause. The left leg silent bus symptom during development was traced to a missing termination resistor at one end of the CAN chain.

Verify by checking both physical ends of the CAN chain: the adapter's internal termination (some adapters have a switch or jumper) and the last ESC in the chain.

---

## CAN Interface Configuration

### How Linux names CAN interfaces

When you plug in a USB-CAN adapter, Linux assigns it a name like `can0`, `can1`, `can2`, or `can3`. These names are assigned by kernel enumeration order, which is not stable across reboots or unplugs. The same physical adapter may be `can0` one day and `can1` the next.

Humanoid Studio uses stable names based on limb assignment: `can_left_leg`, `can_right_leg`, `can_left_arm`, `can_right_arm`. These names are set by udev rules written the first time you assign an adapter in the app.

### Assigning adapters in the app

1. Open Humanoid Studio
2. Navigate to the **CAN Setup** page (gear icon in the sidebar)
3. The app shows all detected CAN interfaces and their USB serial numbers
4. Click **Assign** next to each adapter and select the limb it is connected to
5. The app renames the interface immediately and writes a persistent udev rule

After assignment, the interface is available as `can_left_leg` (or whichever limb you chose) and will be brought up automatically at 1 Mbit/s on every future plug-in.

### What the udev rules do

The udev rule written for each adapter looks like this:

```
SUBSYSTEM=="net", ACTION=="add", ATTRS{serial}=="ABC123DEF",
NAME="can_left_leg",
RUN+="/bin/sh -c '/usr/bin/ip link set can_left_leg type can bitrate 1000000 &&
/usr/bin/ip link set can_left_leg up'"
```

This rule fires when the USB device with serial `ABC123DEF` is plugged in. It renames the raw `canN` interface to `can_left_leg`, sets the bitrate to 1 Mbit/s, and brings the interface up. The rules file is written to `/etc/udev/rules.d/99-humanoid-can.rules`.

After writing new rules, the app triggers `udevadm control --reload-rules` so the new rules take effect without a reboot.

### Verify the interface is up

```bash
ip link show type can
```

A healthy interface looks like:

```
4: can_left_leg: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP mode DEFAULT group default qlen 10
    link/can
```

The key fields are `UP` and `LOWER_UP`. If you see `DOWN`, the interface needs to be brought up.

### Manual bring-up (fallback)

If the udev rule has not been written yet or you are testing without a persistent assignment, bring the interface up manually:

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

Replace `can0` with whatever name the kernel assigned. The app will still work with raw `canN` names — you just need to make sure the channel name in your robot config matches.

### Passwordless sudo for CAN operations

The CAN adapter assignment process requires `sudo` for `ip link` and `udevadm` commands. If your user is not in the sudoers file for these commands, the rename will fail.

To grant passwordless access for these specific commands, add a rule to `/etc/sudoers.d/humanoid-can`:

```
your_username ALL=(ALL) NOPASSWD: /usr/bin/ip, /usr/bin/udevadm, /usr/bin/tee
```

Replace `your_username` with your Linux username.

---

## Powering the robot

### Motors must be powered to transmit

The Recoil ESC firmware starts broadcasting TX_PDO4 frames (position + velocity) at 100 Hz as soon as the board boots. This requires the main motor power supply to be on. USB power alone is not sufficient for CAN communication.

### Signs that motors are powered

- CAN Monitor shows `~995 msg/s` per active bus (6 motors × ~100 Hz per motor plus command frames)
- Individual motor entries in the traffic table appear immediately after power-on
- The dashboard shows motor cards with green ONLINE dots within 2 seconds of power-on

### Signs that motors are not powered

- CAN interface shows `UP` in the monitor (the adapter is connected)
- Message rate is `0 msg/s`
- No entries appear in the traffic table
- All motors show OFFLINE in the dashboard

If you see UP with 0 msg/s, check: (1) main power supply is on, (2) CAN termination resistors are in place, (3) CAN chain is physically connected end-to-end.
