// Canonical CAN bus definitions — single source of truth used across
// Dashboard, CanMonitor, CanSetup, and TelemetryContext.
export const BUSES = [
  { name: 'can_left_leg',  limb: 'left_leg',  label: 'Left Leg' },
  { name: 'can_right_leg', limb: 'right_leg', label: 'Right Leg' },
  { name: 'can_left_arm',  limb: 'left_arm',  label: 'Left Arm' },
  { name: 'can_right_arm', limb: 'right_arm', label: 'Right Arm' },
]
