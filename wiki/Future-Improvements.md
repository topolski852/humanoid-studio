# Future Improvements

This page tracks non-critical improvement ideas across the codebase. Items here are not bugs — the system works correctly without them — but they would improve robustness, debuggability, or maintainability. Review this list when planning a new firmware or software version.

---

## ESC Firmware (v3.0.0 baseline)

### Magic number guard in `loadConfig`

**Priority:** Medium  
**Effort:** Small (< 20 lines)  
**Target version:** Next firmware release

**Current behavior:** `MotorController_loadConfig` reads all float fields from the flash config page and validates them with `isnan()` checks. If any float is NaN (including when the page is freshly erased to all `0xFF`), `loadConfig` returns `HAL_ERROR` and `MotorController_init` enters an infinite UART error loop — no CAN response, no heartbeats, nothing.

**Proposed improvement:** Check the magic number `0xDEAD6431` at config page offset 0 before loading any fields. If the magic is missing (page erased or never written), skip loading and keep the compiled-in defaults, returning `HAL_OK`. This would mean:

```c
// Proposed addition at the top of MotorController_loadConfig:
if (config->magic != 0xDEAD6431) {
    // Flash config page was never written — boot with compiled-in defaults.
    // device_id is already set to DEVICE_CAN_ID by MotorController_init.
    return HAL_OK;
}
// ... existing NaN checks follow ...
```

**Why it matters:** With this change, flashing a commissioning ELF followed by a plain config page erase would work correctly — no pre-written config binary needed. The Flash Wizard currently works around the absence of this guard by writing a full valid config page binary after the ELF. That workaround is correct but adds complexity. The firmware guard is the cleaner long-term fix.

**Config page address:** `0x0801F800` (STM32G431CB Bank1, Page 63, 2 KB)

---

## Flash Wizard (Python)

*(No pending items — current implementation with pre-written commissioning config page is the correct workaround until the firmware magic guard is added.)*

---

## Daemon / CAN Protocol

### Position limits coordinate frame bug in apply_config

**Priority:** Medium  
**Effort:** Small (2 lines in `actuator.cpp`)

**Current behavior:** `configs/humanoid_lite.json` stores `position_limits` in the display frame (degrees converted to radians, zero-referenced to the calibrated joint zero). `apply_config` writes `position_limit_lower` and `position_limit_upper` directly to the firmware via SDO. The firmware's position controller operates in its internal frame (`display_position + position_offset`). For joints with a non-zero `position_offset`, the applied limits are `position_offset` radians too strict: a joint with `position_offset = 0.375 rad` and `max = 1.57 rad` has its firmware upper limit set to 1.57 rad instead of the correct 1.945 rad.

**Proposed fix:** In `actuator.cpp` `apply_config`, add `position_offset` to each limit before the SDO write:
```cpp
sdo_write_f32(bus, PARAM_POSITION_LIMIT_LOWER, cfg_.position_limits.min + cfg_.position_offset, ...);
sdo_write_f32(bus, PARAM_POSITION_LIMIT_UPPER, cfg_.position_limits.max + cfg_.position_offset, ...);
```

---

## Flash Wizard / Frontend

### FlashWizard errors silently swallowed

**Priority:** Low  
**Effort:** Small

**Current behavior:** `FlashWizard.jsx` `handleCanConnected` and `handleConfirm` catch errors with `console.error` only. If the underlying daemon call fails (e.g., CAN socket timeout during the post-flash parameter write), the user sees no error message in the UI — the wizard appears to stall silently.

**Proposed fix:** Route these errors through the wizard's existing log panel (`addLogEntry`) so the user sees the failure reason and can retry or abort.

---

*Last updated: 2026-06-16*
