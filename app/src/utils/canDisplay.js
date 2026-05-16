/**
 * Human-readable labels for CAN bus state strings from the backend.
 * Backend values are never changed — only the display strings here.
 */

const BUS_ERROR_LABELS = {
  'ERROR-ACTIVE':  'Healthy',
  'ERROR-WARNING': 'Degraded',
  'ERROR-PASSIVE': 'High Errors',
  'BUS-OFF':       'Bus Off — Reset',
  'DOWN':          'Down',
  'UNCONFIGURED':  'Not Configured',
  'UNKNOWN':       'Unknown',
}

/** Map a CAN bus error state string to a human-readable label. */
export function busErrorLabel(s) {
  return BUS_ERROR_LABELS[s] ?? s ?? 'Unknown'
}

/**
 * Derive a colour key (green/yellow/red/grey) from interface state + error state.
 * Used consistently across Dashboard, CanMonitor, and CanSetup.
 */
export function busColor(state, busErrorState) {
  if (!state || state === 'UNCONFIGURED') return 'yellow'
  if (state !== 'UP') return 'grey'
  if (busErrorState === 'ERROR-ACTIVE')  return 'green'
  if (busErrorState === 'ERROR-WARNING') return 'yellow'
  if (busErrorState === 'ERROR-PASSIVE' || busErrorState === 'BUS-OFF') return 'red'
  return 'grey'
}

/** Convert radians to degrees, formatted to 2 decimal places. */
export function radToDeg(rad) {
  if (rad == null) return null
  return rad * (180 / Math.PI)
}
