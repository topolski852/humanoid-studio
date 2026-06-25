import { useRef } from 'react'
import { radToDeg } from '../utils/canDisplay'
import StatusDot from './StatusDot'

function DataRow({ label, value, highlight = false, dim = false }) {
  return (
    <div className="flex items-baseline justify-between gap-2">
      <span className="data-label">{label}</span>
      <span className={`font-mono text-xs tabular-nums ${
        highlight ? 'text-accent' : dim ? 'text-gray-600' : 'text-gray-200'
      }`}>
        {value}
      </span>
    </div>
  )
}

/**
 * passiveState: { status, position_rad, velocity_rads, last_seen_ms } | null
 * Comes from passive CAN traffic when the robot is not actively connected.
 */
export default function MotorCard({ joint, state, passiveState, onClick }) {
  // Latch last known firmware version — brief OFFLINE transitions reset firmware_version to null
  // in the daemon but we don't want the card to flicker back to '—'
  const lastFwRef = useRef(null)
  if (state?.firmware_version) lastFwRef.current = state.firmware_version
  const fwVersion = lastFwRef.current

  // A motor is online if actively polled OR seen in passive traffic within 2s
  const activeOnline  = state != null
  const passiveOnline = passiveState?.status === 'ONLINE'
  const online = activeOnline || passiveOnline

  // Prefer active state; fall back to passive
  const posDeg = activeOnline && state.position != null
    ? radToDeg(state.position)?.toFixed(2)
    : passiveState?.position_rad != null
      ? radToDeg(passiveState.position_rad)?.toFixed(2)
      : null

  const mode     = state?.mode_name ?? '—'
  const hasError = state?.error != null && state.error !== 0

  return (
    <button
      onClick={onClick}
      className="card text-left p-4 hover:border-accent/50 hover:bg-surface-2 transition-all group w-full"
    >
      {/* Header */}
      <div className="flex items-start gap-2 mb-3">
        <StatusDot online={online} size="sm" />
        <div className="flex-1 min-w-0">
          <div className="text-[10px] text-gray-500 font-mono mb-0.5">
            {joint.can_channel} · ID {joint.can_id}
          </div>
          <div className="text-sm font-medium text-white group-hover:text-accent transition-colors truncate">
            {joint.joint_name}
          </div>
        </div>
        {hasError && (
          <span className="flex-shrink-0 text-[10px] bg-danger/20 text-danger px-1.5 py-0.5 rounded font-mono">
            ERR
          </span>
        )}
        {/* Passive-only indicator */}
        {!activeOnline && passiveOnline && (
          <span className="flex-shrink-0 text-[9px] bg-surface-2 text-gray-500 border border-surface-3 px-1.5 py-0.5 rounded font-mono">
            CAN
          </span>
        )}
      </div>

      {/* Data rows */}
      <div className="space-y-1.5 pt-2 border-t border-surface-3">
        <DataRow
          label="POSITION"
          value={posDeg != null ? `${posDeg}°` : '—'}
          dim={!activeOnline && passiveOnline}
        />
        <DataRow label="MODE" value={mode} highlight={activeOnline} />
        <DataRow label="FW" value={fwVersion ?? '—'} dim />
      </div>

      {/* Bottom accent bar */}
      <div className="mt-3 h-0.5 bg-surface-3 rounded-full overflow-hidden">
        <div
          className={`h-full rounded-full transition-all duration-300 ${
            activeOnline ? 'bg-accent w-full' :
            passiveOnline ? 'bg-online/60 w-full' : 'w-0'
          }`}
        />
      </div>
    </button>
  )
}
