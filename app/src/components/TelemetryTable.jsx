// ── Metric row ────────────────────────────────────────────────────────────────
function Metric({ label, value, unit, large = false, danger = false }) {
  return (
    <div className="flex flex-col gap-0.5">
      <span className="data-label">{label}</span>
      <div className="flex items-baseline gap-1.5">
        <span
          className={`font-mono tabular-nums ${large ? 'text-2xl' : 'text-lg'} ${
            danger ? 'text-danger' : 'text-white'
          }`}
        >
          {value}
        </span>
        {unit && <span className="text-xs text-gray-500">{unit}</span>}
      </div>
    </div>
  )
}

function Divider() {
  return <div className="h-px bg-surface-3" />
}

// ── Main component ────────────────────────────────────────────────────────────
/**
 * passiveState: { status, position_rad, velocity_rads, last_seen_ms } | null
 * Shown as a fallback when active state is not available (robot not connected).
 */
export default function TelemetryTable({ state, passiveState = null }) {
  const fmt = (v, d = 3) => (v != null && typeof v === 'number' ? v.toFixed(d) : '—')
  const hasError = state?.error != null && state.error !== 0

  const R2D = 180 / Math.PI

  // Active state takes priority; passive state is fallback
  const posRad  = state?.position ?? passiveState?.position_rad
  const velRads = state?.velocity ?? passiveState?.velocity_rads
  const isPassive = state == null && passiveState != null

  const posDeg = posRad  != null ? (posRad  * R2D).toFixed(2) : '—'
  const velDeg = velRads != null ? (velRads * R2D).toFixed(2) : '—'

  return (
    <div className="w-72 flex-shrink-0 border-r border-surface-3 p-6 overflow-y-auto">
      <div className="flex items-center gap-2 mb-5">
        <h3 className="data-label flex-1">Live Telemetry</h3>
        {isPassive && (
          <span className="text-[9px] px-1.5 py-0.5 rounded bg-surface-2 border border-surface-3 text-gray-500 font-mono">
            PASSIVE CAN
          </span>
        )}
      </div>

      <div className="space-y-5">
        <Metric label="POSITION" value={posDeg} unit="°" large />

        <Metric label="VELOCITY" value={velDeg} unit="°/s" />

        <Divider />

        <Metric label="CURRENT (Iq)" value={fmt(state?.current, 3)} unit="A" />

        <Metric label="TORQUE (est.)" value={fmt(state?.torque, 3)} unit="Nm" />

        {/* Temperature — not available from firmware SDO map */}
        <div className="flex flex-col gap-0.5">
          <span className="data-label">TEMPERATURE</span>
          <span className="font-mono text-lg text-gray-600">N/A</span>
          <span className="text-[9px] text-gray-700">not exposed by firmware</span>
        </div>

        <Metric label="BUS VOLTAGE" value={fmt(state?.bus_voltage, 2)} unit="V" />

        <Divider />

        {/* Mode */}
        <div className="flex flex-col gap-0.5">
          <span className="data-label">CONTROL MODE</span>
          <span
            className={`font-mono text-sm ${
              state?.mode_name === 'POSITION' || state?.mode_name === 'VELOCITY'
                ? 'text-accent'
                : state?.mode_name === 'DISABLED' || state?.mode_name == null
                ? 'text-gray-500'
                : 'text-white'
            }`}
          >
            {state?.mode_name ?? '—'}
          </span>
        </div>

        {/* Error */}
        <div className="flex flex-col gap-0.5">
          <span className="data-label">ERROR FLAGS</span>
          {hasError ? (
            <div>
              <span className="font-mono text-sm text-danger">
                0x{state.error.toString(16).padStart(4, '0').toUpperCase()}
              </span>
              {state.error_names?.length > 0 && (
                <div className="mt-1 text-xs text-danger/70">
                  {state.error_names.join(', ')}
                </div>
              )}
            </div>
          ) : (
            <span className="font-mono text-sm text-gray-500">0x0000</span>
          )}
        </div>

        {/* Timestamp */}
        {state?.timestamp != null && (
          <div className="flex flex-col gap-0.5">
            <span className="data-label">LAST UPDATE</span>
            <span className="font-mono text-xs text-gray-600">
              {new Date(state.timestamp * 1000).toLocaleTimeString()}
            </span>
          </div>
        )}
      </div>

      {/* Disconnected overlay */}
      {state == null && passiveState == null && (
        <div className="mt-6 px-3 py-2 rounded-lg bg-surface-3 border border-surface-3">
          <p className="text-xs text-gray-500 text-center">No data — motor offline</p>
        </div>
      )}
    </div>
  )
}
