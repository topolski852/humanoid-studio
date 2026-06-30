# Plan: boot-DISABLED firmware + per-motor disconnect

Status: **firmware DONE (v3.2.0, awaiting HIL confirm); app additions still pending.**
Decided approach (2026-06-29): fix the connect/commissioning problem primarily by a
**firmware function change** (boot into DISABLED), plus a per-motor Disconnect button
in the app.

## Done (2026-06-29)
- `motor_controller.c:116` `MODE_IDLE` → `MODE_DISABLED` (boot silent).
- `FIRMWARE_VERSION` `0x03010200` → `0x03020000` (v3.2.0).
- Rebuilt both prebuilt ELFs (`production_MAD_M6C12_150KV.elf`,
  `production_MAD_5010_200KV.elf`) — v3.2.0 word confirmed baked in.
- `firmware/esc/VERSION` → `v3.2.0`.
- **TODO tomorrow:** reflash all ESCs, then HIL-verify (candump silent on boot →
  Connect wakes to IDLE). Then do the **App additions** below.

## Problem

On app start / robot power-cycle, every ESC comes up **active and broadcasting**, so
the dashboard shows them "connected" before the user connects, and the **Flash Wizard
fails** because the daemon keeps talking to the *other* active motors on the bus
during commissioning (collisions).

## Firmware facts (verified)

- ESCs **boot into `MODE_IDLE`** — `MotorController_init()` ends with
  `MotorController_setMode(controller, MODE_IDLE)` (`motor_controller.c:116`,
  commented "change mode to idle"). The `MODE_DISABLED` at line 27 is only the
  pre-init struct default. So every ESC powers up active/broadcasting regardless of
  its mode before power-off. (Confirmed empirically: connect→IDLE, power-cycle,
  boots back IDLE.)
- A **DISABLED motor is fully silent and inert**: no heartbeat
  (`motor_controller.c:474` `if (mode == MODE_DISABLED) return;`), no PDO4 telemetry
  (`app.c:53`), watchdog off (`app.c:45`). The daemon can't even see a DISABLED motor.

## Primary fix — firmware boots DISABLED (function change)

Change the end of `MotorController_init()` (`motor_controller.c:116`) from
`MotorController_setMode(controller, MODE_IDLE);` to **`MODE_DISABLED`** (or drop the
line so it keeps the init default). Net effect:

- ESCs power up **silent**. Bus is quiet at startup → commissioning's other motors
  don't interfere; dashboard shows everything disconnected until the user connects.
- **Connect** already sends NMT IDLE (+ apply config), which wakes a DISABLED motor →
  active. **No app code change is required** for the core fix.
- A motor that browns out/power-cycles reboots DISABLED and stays disconnected until
  the user clicks Connect — matches "the buttons are the only trigger."

This is a **behavior change, not a bug fix**: bump the firmware version
(v3.1.2 → **v3.2.0**), rebuild the prebuilt ELFs (`firmware/esc/build_all.sh`), and
**reflash all ESCs**. Safe: boot mode IDLE vs DISABLED doesn't change pose-holding
(neither commands torque); it only decides chatty vs silent on boot.

Verify nothing in `init()` after line 116 relies on the motor being IDLE (it's the
last step — `PowerStage_start`/`calibratePhaseCurrentOffset`/`clearError` all run
before it, so DISABLED at the very end is fine).

## App additions (still wanted, independent of the firmware fix)

1. **Per-motor Disconnect button** — each motor tab gets Disconnect to complement the
   existing per-motor Connect.
   - `daemon_client.py`: add `disconnect_single(joint_name)` → `set_mode(joint,
     "DISABLED")` + `disable_slow_poll(joint)`, remove from `_directly_connected`.
   - `routes_motors.py`: `POST /motors/{joint_name}/disconnect`.
   - `api.js`: `disconnectMotor(j)`. `MotorTab.jsx`: the `ConnectMotorButton` becomes a
     Connect/Disconnect toggle keyed off mode (Connect when DISABLED/OFFLINE,
     Disconnect when IDLE/ENABLED).
2. **Global Disconnect → DISABLED** (consistency): `disconnect()` currently sets
   `SET_ALL_MODE IDLE`; change to `DISABLED` + `disable_all_slow_poll()` so the
   sidebar Disconnect also truly silences the bus (not just relies on boot state).
3. **Sidebar version label** `v0.1.0` → `v1.0.0` (and ideally source from
   `package.json` so it can't go stale).

No daemon C++ change is needed: a DISABLED motor is silent, so the existing auto-wake
(`needs_idle_wakeup_`) can't re-wake it; it only ever re-establishes a motor that is
*already* broadcasting (a transient comms dropout while still IDLE) — keep as-is.

## Verification (HIL, left leg on can_left_leg)
1. Flash v3.2.0 to a joint → power-cycle → `candump` shows **no frames** from it
   until Connect (boots DISABLED/silent); dashboard shows it disconnected.
2. Click Connect → it wakes to IDLE, telemetry live.
3. Disconnect → back to DISABLED, silent.
4. Flash Wizard with everything disconnected → commission a joint; the others stay
   silent (they booted DISABLED) → commissioning succeeds.
5. Per-motor Connect/Disconnect on one joint while others stay silent.

## Risks / watch-outs
- Reflashing **all** ESCs is required for the boot-DISABLED change to take effect
  everywhere; until a given ESC is reflashed it still boots IDLE.
- The release's bundled firmware moves to v3.2.0 — update `firmware/esc/VERSION` and
  the prebuilt ELFs; consider a follow-up app release once reflashed + retested.
- Confirm the daemon/Connect path reliably wakes a DISABLED motor (NMT IDLE → IDLE)
  — it does today via `apply_all_configs`/`connect_single`, but retest after the
  firmware change.
