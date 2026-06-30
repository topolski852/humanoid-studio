// Shared building blocks for the motor tuning panels (Auto / Diagnose / Gravity Tune).
// Extracted from AutoTunePanel.jsx so the three panels don't triplicate the same
// constants, formatting helpers, power controls, and telemetry chart.
import { useState } from 'react'
import {
  ComposedChart,
  Line,
  XAxis,
  YAxis,
  Tooltip,
  ResponsiveContainer,
} from 'recharts'
import { api } from '../api'

export const DEG = Math.PI / 180
export const R2D = 180 / Math.PI
export const round1 = (v) => Math.round(v * 10) / 10
export const round2 = (v) => Math.round(v * 100) / 100

export const ACTIVE_MODES = new Set(['POSITION', 'VELOCITY', 'TORQUE', 'CURRENT'])

export const SIGNAL_CONFIG = [
  { key: 'commanded',   label: 'Position Cmd (°)',  color: '#3b82f6', yAxisId: 'pos',   strokeDasharray: '4 2' },
  { key: 'position',    label: 'Position Meas (°)', color: '#22c55e', yAxisId: 'pos' },
  { key: 'velocity',    label: 'Velocity (°/s)',    color: '#eab308', yAxisId: 'other' },
  { key: 'torque',      label: 'Torque (Nm)',       color: '#f97316', yAxisId: 'other' },
  { key: 'current',     label: 'Current (A)',       color: '#ef4444', yAxisId: 'other' },
  { key: 'bus_voltage', label: 'Bus Voltage (V)',   color: '#94a3b8', yAxisId: 'other' },
]

export const DEFAULT_VISIBLE = new Set(['commanded', 'position', 'velocity', 'current'])

// Convert raw samples (rad) to display units (deg) for the chart.
export function toDisplaySamples(samples) {
  return samples.map((s) => ({
    ...s,
    commanded: s.commanded != null ? s.commanded * R2D : null,
    position:  s.position  != null ? s.position  * R2D : null,
    velocity:  s.velocity  != null ? s.velocity  * R2D : null,
  }))
}

export function SectionLabel({ children }) {
  return (
    <p className="text-[10px] font-semibold text-gray-500 uppercase tracking-wider">
      {children}
    </p>
  )
}

export function Field({ label, value, onChange }) {
  return (
    <div className="space-y-0.5">
      <p className="text-[9px] text-gray-600">{label}</p>
      <input
        type="text"
        inputMode="decimal"
        value={value}
        onChange={(e) => onChange(e.target.value)}
        onBlur={(e) => {
          const n = parseFloat(e.target.value)
          if (!isNaN(n)) onChange(String(n))
        }}
        className="w-full px-2 py-1 rounded border border-surface-3 bg-surface-2
          text-xs font-mono text-gray-200 outline-none focus:border-accent/50 transition-colors"
      />
    </div>
  )
}

export function MetricRow({ label, value, warn = false }) {
  return (
    <>
      <span className="text-gray-500">{label}</span>
      <span className={`font-mono text-right ${warn ? 'text-amber-400' : 'text-gray-200'}`}>
        {value}
      </span>
    </>
  )
}

// ── Power controls: not-connected notice, ESC error banner, Enable/Idle/E-Stop ──
// Self-contained: drives the motor over the API and reports errors via onLogError.
export function PowerControls({ jointName, state, onLogError }) {
  const [busy, setBusy] = useState(false)
  const [clearing, setClearing] = useState(false)

  const isConnected = state != null
  const isEnabled = ACTIVE_MODES.has(state?.mode_name)

  async function enable() {
    setBusy(true)
    try { await api.enableMotor(jointName, 'POSITION') } catch (e) { onLogError?.(e.message, 'Enable') }
    setBusy(false)
  }
  async function toIdle() {
    setBusy(true)
    try { await api.disableMotor(jointName) } catch (e) { onLogError?.(e.message, 'Disable') }
    setBusy(false)
  }
  async function doEstop() {
    setBusy(true)
    try { await api.estopMotor(jointName) } catch (e) { onLogError?.(e.message, 'E-Stop') }
    setBusy(false)
  }
  async function clearError() {
    setClearing(true)
    try { await api.clearMotorError(jointName) } catch (e) { onLogError?.(e.message, 'ClearError') }
    setClearing(false)
  }

  return (
    <>
      {!isConnected && (
        <div className="px-3 py-2.5 rounded-lg bg-surface-2 border border-surface-3">
          <p className="text-xs text-gray-400">Motor visible via passive CAN.</p>
          <p className="text-[10px] text-gray-600 mt-0.5">
            Connect the robot (top-right) to send commands.
          </p>
        </div>
      )}

      {isConnected && state?.error !== 0 && state?.error != null && (
        <div className="px-3 py-2.5 rounded-lg bg-danger/10 border border-danger/30">
          <div className="flex items-center justify-between gap-3">
            <div>
              <p className="text-xs font-medium text-danger">ESC Error: {state.error}</p>
              {state.error_names?.length > 0 && (
                <p className="text-[10px] text-danger/70 mt-0.5 font-mono">
                  {state.error_names.join(', ')}
                </p>
              )}
            </div>
            <button
              onClick={clearError}
              disabled={clearing}
              className="shrink-0 px-3 py-1 rounded text-xs font-medium bg-danger/20 text-danger
                border border-danger/30 hover:bg-danger/30 disabled:opacity-40 transition-colors"
            >
              {clearing ? 'Clearing…' : 'Clear ESC Error'}
            </button>
          </div>
        </div>
      )}

      <section className="space-y-1.5">
        <SectionLabel>Power</SectionLabel>
        {isEnabled ? (
          <div className="flex gap-2">
            <button
              onClick={toIdle}
              disabled={busy || !isConnected}
              className="flex-1 py-2 rounded-lg text-sm font-medium transition-colors
                bg-surface-2 text-gray-300 border border-surface-3
                hover:border-accent/40 hover:text-accent disabled:opacity-40"
            >
              {busy ? '…' : 'To Idle'}
            </button>
            <button
              onClick={doEstop}
              disabled={busy || !isConnected}
              className="flex-[2] py-2 rounded-lg text-sm font-medium transition-colors
                bg-danger/20 text-danger border border-danger/30
                hover:bg-danger/30 disabled:opacity-40"
            >
              {busy ? '…' : '⛔ E-STOP'}
            </button>
          </div>
        ) : (
          <button
            onClick={enable}
            disabled={busy || !isConnected}
            className="w-full py-2 rounded-lg text-sm font-medium transition-colors
              bg-accent/20 text-accent border border-accent/30
              hover:bg-accent/30 disabled:opacity-40"
          >
            {busy ? '…' : '▶ Enable (Position)'}
          </button>
        )}
        {isConnected && (
          <p className="text-[10px] text-gray-600 text-center">
            Mode: <span className="font-mono text-gray-400">{state?.mode_name ?? '—'}</span>
          </p>
        )}
      </section>
    </>
  )
}

// ── Telemetry chart with signal toggles. `samples` are raw (rad) sample dicts. ──
export function TuneChart({ samples }) {
  const [signals, setSignals] = useState(
    Object.fromEntries(SIGNAL_CONFIG.map((s) => [s.key, DEFAULT_VISIBLE.has(s.key)]))
  )
  const display = toDisplaySamples(samples ?? [])

  return (
    <>
      <section className="space-y-2">
        <SectionLabel>Signals</SectionLabel>
        <div className="flex flex-wrap gap-1.5">
          {SIGNAL_CONFIG.map(({ key, label, color }) => (
            <button
              key={key}
              onClick={() => setSignals((s) => ({ ...s, [key]: !s[key] }))}
              className={`flex items-center gap-1.5 px-2 py-0.5 rounded border text-[10px] transition-colors
                ${signals[key] ? 'border-current bg-surface-2' : 'border-surface-3 text-gray-600'}`}
              style={signals[key] ? { color } : {}}
            >
              <span
                className="w-2 h-2 rounded-full flex-shrink-0"
                style={{ background: signals[key] ? color : '#4b5563' }}
              />
              {label}
            </button>
          ))}
        </div>
      </section>

      <section className="w-full overflow-hidden">
        <ResponsiveContainer width="100%" height={240}>
          <ComposedChart data={display} margin={{ top: 4, right: 48, bottom: 4, left: 0 }}>
            <XAxis
              dataKey="t_ms"
              tickFormatter={(v) => `${(v / 1000).toFixed(1)}s`}
              tick={{ fontSize: 9, fill: '#6b7280' }}
              stroke="#374151"
            />
            <YAxis
              yAxisId="pos"
              tick={{ fontSize: 9, fill: '#6b7280' }}
              tickFormatter={(v) => v.toFixed(1)}
              stroke="#374151"
              width={42}
              unit="°"
            />
            <YAxis
              yAxisId="other"
              orientation="right"
              tick={{ fontSize: 9, fill: '#6b7280' }}
              tickFormatter={(v) => v.toFixed(1)}
              stroke="#374151"
              width={36}
            />
            <Tooltip
              content={({ active, payload, label }) =>
                active && payload?.length ? (
                  <div className="bg-surface-2 border border-surface-3 rounded px-2 py-1.5 text-[9px] font-mono space-y-0.5">
                    <p className="text-gray-400">{(label / 1000).toFixed(3)}s</p>
                    {payload.map((p) => (
                      <p key={p.dataKey} style={{ color: p.stroke }}>
                        {p.name}: {typeof p.value === 'number' ? p.value.toFixed(2) : '—'}
                      </p>
                    ))}
                  </div>
                ) : null
              }
            />
            {SIGNAL_CONFIG.map(({ key, label, color, yAxisId, strokeDasharray }) =>
              signals[key] ? (
                <Line
                  key={key}
                  yAxisId={yAxisId}
                  dataKey={key}
                  name={label}
                  stroke={color}
                  strokeWidth={1.5}
                  strokeDasharray={strokeDasharray}
                  dot={false}
                  isAnimationActive={false}
                  connectNulls={false}
                />
              ) : null
            )}
          </ComposedChart>
        </ResponsiveContainer>
      </section>
    </>
  )
}

// Small confirm modal used to gate destructive actions (phase flip, flash store).
export function ConfirmModal({ open, title, body, confirmLabel, danger, busy, onCancel, onConfirm }) {
  if (!open) return null
  return (
    <div className="fixed inset-0 z-50 flex items-center justify-center bg-black/60 p-4" onClick={onCancel}>
      <div
        className="w-full max-w-sm rounded-xl border border-surface-3 bg-surface-1 p-4 space-y-3"
        onClick={(e) => e.stopPropagation()}
      >
        <p className="text-sm font-semibold text-gray-200">{title}</p>
        <p className="text-[11px] text-gray-400 leading-relaxed">{body}</p>
        <div className="flex gap-2 pt-1">
          <button
            onClick={onCancel}
            disabled={busy}
            className="flex-1 py-1.5 rounded-lg text-xs font-medium bg-surface-2 text-gray-300
              border border-surface-3 hover:text-gray-100 disabled:opacity-40"
          >
            Cancel
          </button>
          <button
            onClick={onConfirm}
            disabled={busy}
            className={`flex-1 py-1.5 rounded-lg text-xs font-medium border disabled:opacity-40 ${
              danger
                ? 'bg-danger/20 text-danger border-danger/30 hover:bg-danger/30'
                : 'bg-accent/20 text-accent border-accent/30 hover:bg-accent/30'
            }`}
          >
            {busy ? 'Working…' : confirmLabel}
          </button>
        </div>
      </div>
    </div>
  )
}
