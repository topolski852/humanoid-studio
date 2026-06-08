# Flash Wizard — Bug and Fix Log

A running record of every bug observed and every fix attempted.
Status lifecycle: **[UNTESTED]** → **[FAILED]** → **[WORKING]**

A fix is only marked **[WORKING]** when the user explicitly confirms the full
end-to-end Flash Wizard flow (flash + commission + calibration) completed
successfully.

---

## BUG-001 — ESC invisible in passive sniff after flash + power cycle

**Symptom:** Passive CAN sniff shows zero frames from `dev=127`; no other
devices drop off the bus during the wizard.

**Log output:**
```
No CAN frames received in 3 s on can_left_leg.
ESC may not be booted or CAN cable/termination is wrong.
FAILED: ESC did not appear at commissioning ID 127 ...
```

**Root cause:** `_bounce_can_interface()` was called inside
`flash_can_connected()` — *after* the ESC was already live on the bus.
During the interface-down period the ESC's TX frames receive no ACK from the
host; every un-ACK'd frame increments the FDCAN Transmit Error Counter (TEC)
by 8. After ~32 un-ACK'd frames TEC exceeds 128 (ERROR_PASSIVE) and the ESC
stops retrying transmissions. If TEC reaches 256 the controller enters
BUS_OFF and refuses to transmit at all until a power cycle.

---

### FIX-001-a — Move CAN bounce to before WAITING_CAN_CONNECT [UNTESTED]

Moved `_bounce_can_interface(config.can_channel)` from
`routes_flash.py::flash_can_connected()` to `flash.py::_do_session()`,
immediately after `_flash_prebuilt()` completes and before the state
transitions to `WAITING_CAN_CONNECT`. The bounce now runs while the ESC is
powered off and not yet connected to the bus, so no frames are lost.
Commission-only mode (`skip_flash=True`) skips the bounce entirely — the bus
is already live when the user clicks "Hardware connected".

---

## BUG-002 — ESC in BUS_OFF from prior failed attempt; power cycle needed

**Symptom:** After a failed FIX-001 attempt the SWD diagnostic shows
`mode=IDLE, error=NO_ERROR` (healthy application state) but zero CAN frames
arrive from the ESC even though other devices are visible.

**Log output:**
```
SWD diagnostic: device_id=127, firmware=0x00030000, mode=IDLE, error=NO_ERROR (0x0000)
FAILED: ESC did not appear...
```

**Root cause:** A prior wizard run (before FIX-001-a) ran the late bounce
while the ESC was live. TEC accumulated to BUS_OFF (≥ 256). The STM32G431
FDCAN peripheral stays in BUS_OFF until a power cycle; the application code
reports `error=NO_ERROR` because it reads an application-layer error field,
not the FDCAN hardware register. The SWD diagnostic did not read FDCAN ECR/PSR,
so BUS_OFF was invisible.

---

### FIX-002-a — Read FDCAN1 ECR+PSR in SWD diagnostic; targeted BUS_OFF error message [UNTESTED]

Added `mdw 0x40006440 2` to the openocd command in
`_read_controller_state_via_swd()`. Parses the two 32-bit words:
- `ECR` (0x40006440): `TEC = ECR & 0xFF`, `REC = (ECR >> 8) & 0x7F`
- `PSR` (0x40006444): `EP = bit 5`, `EW = bit 6`, `BO = bit 7`

`_format_fdcan_state()` helper formats the result as `"BUS_OFF (TEC=N)"`,
`"ERROR_PASSIVE (TEC=N, REC=N)"`, `"ERROR_WARNING (TEC=N, REC=N)"`, or
`"OK (TEC=N, REC=N)"`.

When `swd.get("can_bo")` is True, raises `FlashError` with a targeted
message: *"ESC application is healthy … but the FDCAN controller is in
BUS_OFF state … Power cycle the ESC to recover."*

Updated WAITING_CAN_CONNECT instructions in both `flash.py` log messages
and `FlashWizard.jsx` to explicitly require a power cycle in both
flash+commission and commission-only modes.

---

## BUG-003 — ESC heartbeat visible on CAN but all SDO reads time out

**Symptom:** Passive sniff sees `dev=127 func=0xE` (ESC is alive and
broadcasting), but all 60 `read_parameter_u32` attempts return `None` over
~60 seconds.

**Log output:**
```
CAN activity detected: dev=5 func=0x9, dev=7 func=0x9, dev=3 func=0x9,
  dev=5 func=0x9, dev=7 func=0x9, dev=3 func=0x9, dev=5 func=0x9, dev=127 func=0xE
Waiting for ESC at commissioning ID 127 on can_left_leg...
Still waiting for ESC at ID 127...
FAILED: ESC did not appear at commissioning ID 127 or target ID 1 on can_left_leg within 60 s
```

**Root cause:** `func_id=0xE` is `HEARTBEAT` in the Recoil protocol. The ESC
commissioning firmware is configured with `watchdog_timeout=1000 ms`. The
ESC boots at POR, sends heartbeats, and waits for a watchdog feed
(`func=0xE, 0 bytes`, host→node). The previous code opened the CAN socket
*after* a 1-second settle sleep: by the time the socket opened the watchdog
had already expired. The ESC entered `WATCHDOG_TIMEOUT` error state and
stopped processing SDO commands — but continued broadcasting heartbeats
(heartbeats are driven by a separate timer, not gated on watchdog health).

---

### FIX-003-a — Feed watchdog immediately on bus open; continuous feeds through sniff and pre-check loops [UNTESTED]

In `flash.py::_do_session()`:

1. Moved `bus = CANBus(...)` and `await bus.connect()` to *before* the settle
   sleep (previously they were after it).
2. Call `await bus.feed_watchdog(comm_id)` immediately after `bus.connect()`,
   before any sleep.
3. Reduced settle sleep from 1.0 s to 0.5 s.
4. Added `await bus.feed_watchdog(comm_id)` at the top of the passive sniff
   `while` loop (every ≤ 0.25 s receive cycle, feeds happen continuously).
5. Added `await bus.feed_watchdog(comm_id)` at the top of the SDO pre-check
   `for` loop; reduced the inter-attempt sleep from 0.5 s to 0.25 s so the
   total per-iteration time stays well under 1000 ms.
6. Decode heartbeat payload in the sniff loop: when
   `frame.device_id == comm_id and frame.func_id == 0xE and len(frame.data) >= 5`,
   unpack `mode (1 byte) + error_code (4 bytes, little-endian)` and log
   `ESC heartbeat: mode=<MODE_NAME>, error=<ERROR_FLAGS>`. This makes
   WATCHDOG_TIMEOUT immediately visible in future logs.

---

## BUG-004 — `Transmit buffer full` crash; ESC not found at commissioning ID 127

**Symptom:** Two consecutive commission-only runs fail identically. Passive sniff
shows only `dev=5, dev=3, dev=7` (the other three left-leg joints) — the target
ESC never appears. Pre-check loop runs ~30 attempts, then crashes:

**Log output:**
```
CAN activity detected: dev=5 func=0x9, dev=3 func=0x9, dev=7 func=0x9, ...
Waiting for ESC at commissioning ID 127 on can_left_leg...
Still waiting for ESC at ID 127...
Still waiting for ESC at ID 127...
FAILED (unexpected): Transmit failed on can_left_leg: Transmit buffer full
```

**Root cause (two issues):**

*Issue A — crash*: `await bus.feed_watchdog(comm_id)` in the pre-check loop
is placed OUTSIDE the `try/except CANBusError` block. When the kernel CAN TX
queue fills (ENOBUFS → `can.CanError` → `CANBusError` after 4 retries), the
exception propagates past the inner bounce handler all the way to the outer
`except Exception` → `"FAILED (unexpected)"`. The same exposure exists in the
sniff loop.

*Issue B — ESC not at 127*: The commissioning firmware boots at ID 127 only
when the flash config page has `device_id=127`. After a successful prior
commissioning run, `device_id=target_id` was stored to flash. On the next power
cycle the ESC boots at `target_id`. The code did not try `target_id` until after
60 failed attempts at 127, and when it found the ESC there it wrongly set
`skip_commission = True` — skipping all parameter writes and just running
calibration.

---

### FIX-004-a — Guard feed_watchdog with try/except; check target ID first; introduce active_id [UNTESTED]

**Issue A fix:** Wrapped `await bus.feed_watchdog(...)` in the sniff loop in a
bare `try/except CANBusError: pass`. In the pre-check loop, moved
`feed_watchdog` INSIDE the existing `try/except CANBusError` block so ENOBUFS
now triggers the bounce-and-retry path instead of crashing.

**Issue B fix:** Introduced `active_id` (defaults to `comm_id=127`). Before the
60-attempt wait loop, a quick check is performed at `config.can_id` (the target
ID). If the ESC responds there (previously commissioned), `active_id` is set to
`config.can_id` and the wait loop is skipped entirely. The wait loop itself now
polls `active_id` (not the hardcoded `comm_id`). The `skip_commission = True`
path was replaced with `active_id = config.can_id` + full commissioning writes.

All SDO writes in the commissioning section now use `active_id`. The
`DEVICE_ID` write is guarded by `if active_id != config.can_id` — it is skipped
when the ESC is already at its target ID (writing DEVICE_ID to itself would
switch the CAN filter and drop the subsequent `store_to_flash` frame).

---

## BUG-005 — ENOBUFS on the very first CAN transmit after daemon shutdown

**Symptom:** Fails immediately after "Connection confirmed" — before the passive sniff
or pre-check loop even start.

**Log output:**
```
Connection confirmed. Commissioning ESC over CAN...
FAILED (unexpected): Transmit failed on can_left_leg: Failed to transmit: No buffer space available [Error Code 105]
```

**Root cause (two issues):**

*Issue A — TX queue full at socket open time*: The daemon communicates at 200 Hz with
all production ESCs on `can_left_leg`. When the user clicks "Hardware connected",
`routes_flash.py::flash_can_connected()` sends SHUTDOWN to the daemon via UDP and sleeps
300 ms. The daemon ACKs SHUTDOWN immediately but its SocketCAN socket and netdev TX queue
are not released within 300 ms. In commission-only mode, `_bounce_can_interface()` is
never called anywhere, so `txqueuelen` may be at the kernel default (as low as 10). When
the flash wizard opens its `CANBus` socket and immediately calls `feed_watchdog()`, the
netdev TX queue is still full → ENOBUFS → all 4 retries (300 ms total) fail.

*Issue B — initial `feed_watchdog` not in try/except*: The first `await bus.feed_watchdog(comm_id)`
call in `_do_session()` (right after `bus.connect()`) is not wrapped in any try/except.
When it raises `CANBusError`, the exception propagates directly to the outer `except Exception`
handler → `"FAILED (unexpected)"`. The bounce-and-retry path added in FIX-004-a only covers
the pre-check loop, not this earlier call.

---

### FIX-005-a — Bounce in `can_connected()` for all modes; wrap initial `feed_watchdog` [UNTESTED]

**Issue A fix:** Added `await _bounce_can_interface(self._current_channel)` inside
`FlashManager.can_connected()` (in `flash.py`) before setting `_can_connect_event`. This
runs for ALL modes (flash+commission and commission-only). It flushes the daemon's netdev
TX queue and sets `txqueuelen=1000` before the flash socket opens. The bounce is safe in
commission-only mode because the commissioning ESC's heartbeats are ACKed by the 3
production ESCs on the same channel during the ~100 ms down period.

**Issue B fix:** Wrapped the initial `await bus.feed_watchdog(comm_id)` in `_do_session()`
in a try/except. On `CANBusError`, it disconnects, bounces the interface again, waits 300 ms,
reconnects, and retries the feed once. This is defense-in-depth for edge cases where the
bounce in `can_connected()` wasn't quite sufficient.

---

## BUG-006 — All SDO reads return None; commissioning firmware uses 4-byte SDO response format

**Symptom:** After FIX-005-a resolves the ENOBUFS crash, the commission flow proceeds to
the pre-check loop but all 60 `read_parameter_u32` attempts return `None` over ~45 seconds.
SWD diagnostic confirms the ESC is healthy: `mode=IDLE, error=NO_ERROR, CAN=OK (TEC=0, REC=0)`.

**Log output:**
```
Connection confirmed. Commissioning ESC over CAN...
Watchdog fed — letting ESC settle...
Listening for CAN frames on can_left_leg for 3 s...
CAN activity detected: dev=5 func=0x9, dev=3 func=0x9, dev=7 func=0x9, ...
Waiting for ESC at commissioning ID 127 on can_left_leg...
Still waiting for ESC at ID 127...
FAILED: ESC did not appear at commissioning ID 127 or target ID 1 on can_left_leg within 60 s
SWD diagnostic: device_id=127, firmware=0x00030000, mode=IDLE, error=NO_ERROR (0x0000), CAN=OK (TEC=0, REC=0)
```

**Root cause (two issues):**

*Issue A — SDO response format mismatch*: The commissioning ELF (`firmware=0x00030000`,
v3.0.0) sends **4-byte SDO read responses** with the raw value at `data[0:4]` and
`tx_frame->size = 4`. Confirmed by disassembling `commissioning_MAD_5010_200KV.elf`:

```asm
800823c: ldr.w r3, [r0, ip]     ; r3 = *(controller + parameter_id) = value
8008240: str r3, [r2, #8]       ; tx_frame->data[0:4] = value
8008242: movs r1, #4
8008244: strh r1, [r2, #6]      ; tx_frame->size = 4 (DLC=4)
```

Production firmware sends 8-byte CANopen upload responses with the value at `data[4:8]`.
`_sdo_read()` in `can_bus.py` unconditionally read `bytes(rx.data[4:8])` — for a 4-byte
frame this slice returns `b''` (empty), causing `struct.unpack("<L", b'')` in
`read_parameter_u32` to raise `struct.error`. The exception propagates up through
`_do_session()` and all 60 pre-check attempts return `None`.

*Issue B — sniff loop exits too early on busy bus*: The 3-second passive sniff loop had
`if len(_sniff) >= 8: break`. With 3 production ESCs sending at 100+ Hz, 8 frames fill
in ~27 ms — the loop exits long before the commissioning ESC's 500 ms heartbeat interval
can be observed. This meant the log never showed `dev=127` even though the ESC was live.

---

### FIX-006-a — Handle 4-byte SDO responses; remove 8-frame sniff cap [UNTESTED]

**Issue A fix:** Updated `_sdo_read()` in `can_bus.py` to branch on response length:

```python
if len(rx.data) >= 8:
    return bytes(rx.data[4:8])   # production: 8-byte CANopen, value at [4:8]
elif len(rx.data) == 4:
    return bytes(rx.data[0:4])   # commissioning ELF v3.0.0: 4-byte raw, value at [0:4]
else:
    return None
```

**Issue B fix:** Removed `if len(_sniff) >= 8: break` from the sniff loop in
`flash.py::_do_session()`. The loop now runs the full 3-second window regardless of
traffic volume, so the commissioning ESC's heartbeat at 500 ms intervals can be logged.

---

## BUG-007 — ESC detected via heartbeat in sniff but all SDO reads still time out; commissioning never starts

**Symptom:** After FIX-006-a, the sniff loop correctly logs `ESC heartbeat: mode=IDLE,
error=NO_ERROR` six times (once every ~500 ms over 3 s) and `dev=127 func=0xE` appears
in the CAN activity list. But the pre-check loop then runs 60 SDO reads over 45 seconds,
all returning `None`, and the wizard fails.

**Log output:**
```
ESC heartbeat: mode=IDLE, error=NO_ERROR   (×6)
CAN activity detected: ... dev=127 func=0xE ...
Waiting for ESC at commissioning ID 127 on can_left_leg...
Still waiting for ESC at ID 127 on can_left_leg...
Still waiting for ESC at ID 127 on can_left_leg...
ESC not on CAN — reading controller state via SWD for diagnosis...
SWD diagnostic unavailable — ST-LINK may not be connected or ESC is unpowered.
FAILED: ESC did not appear at commissioning ID 127 or target ID 1 on can_left_leg within 60 s
```

**Root cause:** The commissioning firmware (v3.0.0, `firmware=0x00030000`) does NOT respond
to SDO read requests (`func=0xC, ccs=2`). The sniff proves the ESC is alive and correctly
identified at ID 127, but every subsequent call to `read_parameter_u32` in the pre-check
loop times out because no SDO response arrives. The post-commissioning verification step
(lines 839–868) uses the same SDO read mechanism and would also fail.

The firmware DOES send heartbeats (`func=0xE, DLC=5, data=[mode(1) + error(4)]`) every
~500 ms, which is the correct mechanism to confirm presence on a firmware that does not
implement SDO reads.

---

### FIX-007-a — Heartbeat-based ESC detection; replace all SDO reads in commissioning path [UNTESTED]

**Pre-check fix:** Added `_sniff_saw_comm_id` and `_sniff_saw_target_id` flags in the
sniff loop. If a heartbeat from `comm_id` (127) was seen during the sniff, `pre_ver` is
set to `0` (sentinel) immediately and the 60-attempt SDO wait loop is skipped entirely.
If a heartbeat from `config.can_id` was seen (previously commissioned ESC), `active_id`
is switched and the entire wait is skipped.

If the sniff didn't catch a heartbeat (bus was unfavourable or ESC booted late), a
post-sniff quick check uses `bus.receive(filter_device_id=N, filter_func=0xE, timeout=0.6)`
instead of SDO. The 60-attempt fallback loop also uses heartbeat receive (600 ms per
attempt) instead of `read_parameter_u32`.

**Post-commissioning verification fix:** The 5-attempt verification loop now tries
`bus.receive(filter_device_id=config.can_id, filter_func=0xE, timeout=0.6)` first and
falls back to SDO read for production firmware that supports reads. The `if not confirmed`
fallback check also uses heartbeat rather than SDO to determine if the ESC is still at
the old ID.

**Log message fix:** `pre_ver = 0` (heartbeat sentinel) prints `"commissioning firmware"`
instead of the misleading `"firmware 0x00000000"`.

---

## BUG-008 — DEVICE_ID write silently fails; ESC stays at ID 127 after commissioning

**Symptom:** After FIX-007-a correctly detects the ESC via heartbeat and proceeds through
all commissioning writes, the post-commissioning verification fails — the ESC is still at
ID 127 and never appears at the target CAN ID.

**Log output:**
```
ESC found at ID 127 (commissioning firmware).
Verifying ESC at CAN ID 1...
FAILED: ESC still at source ID 127 after DEVICE_ID write — SDO write may have failed.
```

**Root cause — dual SocketCAN socket conflict:**

The C++ daemon opens a SocketCAN socket on every configured CAN interface at startup and
holds it for the lifetime of the process. The Python flash wizard (`can_bus.py`) opens a
second independent SocketCAN socket on the same interface during commissioning.

Two SocketCAN sockets on the same `netdev` share the kernel TX queue. The daemon sends
CAN frames at 200 Hz to production ESCs on `can_left_leg`. When the flash wizard also
writes frames, the shared netdev TX queue fills (ENOBUFS). Python's `_sdo_write()` in
`can_bus.py` internally retries and catches `CANBusError`, but returns silently with no
error propagated to the caller. The DEVICE_ID SDO write (and all preceding writes —
phase order, i_kp, i_ki) are discarded by the kernel. The ESC never receives the write
frame and stays at ID 127.

Secondary issue: `can_bus.py`'s `_sdo_write()` registers an ACK waiter but on timeout
returns `None` silently. Callers in `flash.py` receive no indication that the write
failed, so the wizard continues to the "verify at target ID" step — which can never pass.

Additionally, the commissioning ELF (v3.0.0) does not send SDO write ACKs at all. Even if
the ENOBUFS problem were solved, write success was previously unverifiable. The first
indication of failure was the post-commissioning heartbeat check at the target ID.

---

### FIX-008-a — Unified daemon CAN: all commissioning traffic routed through the C++ daemon [UNTESTED]

**Architecture change:** The Python `can_bus.py` socket is no longer opened during the
Flash Wizard. The C++ daemon is the single owner of all SocketCAN sockets for the
lifetime of the process. Flash wizard CAN operations are sent as UDP commands to the
daemon, which executes them on its already-open socket.

**C++ daemon extensions (`daemon/src/`):**

- **`can/generic_listener.hpp/.cpp`** — Thread-safe one-shot future and multi-frame sniff
  collector. The daemon's 200 Hz `drain_all()` callback calls
  `generic_listener_.on_frame()` for every received frame. A fast-path atomic check
  (`pending_ == 0`) skips the lock entirely when nothing is registered, adding zero
  latency to the control loop.

- **8 new UDP command handlers in `robot.cpp`:**
  - `GENERIC_SDO_WRITE` — Sends an SDO write frame and waits up to 500 ms for an ACK.
    Returns `{status:"OK"}` or `{status:"NO_ACK"}` explicitly. No more silent failures.
  - `GENERIC_SDO_READ` — Sends an SDO read frame and returns the 4-byte value in u32/f32/i32.
  - `WAIT_HEARTBEAT` — Blocks until a FUNC_HEARTBEAT frame arrives from the specified
    device, or times out. Returns `{mode, error}` from the heartbeat payload.
  - `SNIFF_BUS` — Collects all frames on a channel for a specified duration. Capped at
    1000 frames.
  - `SEND_NMT` — Sends a bare NMT mode-change frame to any device ID.
  - `SEND_FLASH_STORE` — Sends the FUNC_FLASH store command and waits 300 ms.
  - `FEED_WATCHDOG` — Sends an 8-byte zero heartbeat (watchdog feed) to any device ID.
  - `CALIBRATE_DEVICE` — Sends NMT MODE_CALIBRATION, then polls FUNC_HEARTBEAT until
    the device returns to IDLE/DISABLED. Blocks up to 90 seconds.

**Python changes:**

- **`daemon_client.py`** — 8 new methods: `generic_sdo_write()`, `generic_sdo_read()`,
  `wait_heartbeat()`, `sniff_bus()`, `send_nmt()`, `send_flash_store()`,
  `feed_watchdog_generic()`, `calibrate_device()`. All use the existing UDP command socket.

- **`flash.py`** — All `CANBus` imports removed. Commissioning section rewritten:
  sniff → `dc.sniff_bus()`, heartbeat detection → `dc.wait_heartbeat()`,
  parameter writes → `dc.generic_sdo_write()` (with explicit "NO_ACK" warnings),
  flash store → `dc.send_flash_store()`, verification → `dc.wait_heartbeat()` at target ID,
  calibration → `dc.calibrate_device()` (blocking daemon command).

- **`routes_flash.py`** — Daemon shutdown/restart dance removed from
  `flash_can_connected()`. The daemon stays running throughout the entire commissioning
  flow. No CAN bounce needed; the daemon already owns the open socket.

**Why the daemon does not need to pause its 200 Hz control loop:**
Production joints are configured at IDs 1–N. The commissioning ESC starts at ID 127,
which is not a configured joint — the control loop sends nothing to it. After the
DEVICE_ID write the ESC moves to its target ID; the loop then sends harmless PDO2 frames
to it while it is in IDLE mode (ignored by the ESC).

**ACK semantics for commissioning ELF:** The v3.0.0 commissioning firmware does not send
SDO write ACKs. `GENERIC_SDO_WRITE` will return `{status:"NO_ACK"}` for all writes to
ID 127. The wizard logs `"WARNING: no ACK (commissioning ELF may not ACK writes)"` and
continues — the commissioning ELF reads parameters from the config page written during
`_make_commissioning_config_page()`, so SDO writes to it may be advisory only. If DEVICE_ID
writes are also not processed, the symptom (ESC stays at 127) will still appear; the fix
at that point is to encode DEVICE_ID into the config page before flashing.
