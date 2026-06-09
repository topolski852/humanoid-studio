import { useEffect, useRef, useState } from 'react'
import {
  ComposedChart,
  Line,
  XAxis,
  YAxis,
  Tooltip,
  ResponsiveContainer,
} from 'recharts'
import { api } from '../api'

// Firmware POSITION mode (0x13 = 19)
const MODE_POSITION = 0x13

const SIGNAL_CONFIG = [
  { key: 'commanded',   label: 'Position Cmd',  color: '#3b82f6', yAxisId: 'pos',   strokeDasharray: '4 2' },
  { key: 'position',    label: 'Position Meas', color: '#22c55e', yAxisId: 'pos' },
  { key: 'velocity',    label: 'Velocity',      color: '#eab308', yAxisId: 'other' },
  { key: 'torque',      label: 'Torque',        color: '#f97316', yAxisId: 'other' },
  { key: 'current',     label: 'Current',       color: '#ef4444', yAxisId: 'other' },
  { key: 'bus_voltage', label: 'Bus Voltage',   color: '#94a3b8', yAxisId: 'other' },
]

const DEFAULT_VISIBLE = new Set(['commanded', 'position', 'velocity', 'torque'])

export default function AutoTunePanel({ jointName, state, config, onLogError }) {
  const [testKp,          setTestKp]          = useState(20.0)
  const [testKi,          setTestKi]          = useState(0.0)
  const [testTorqueLimit, setTestTorqueLimit] = useState(2.0)
  const [centerRad,       setCenterRad]       = useState(0.0)
  const [offsetRad,       setOffsetRad]       = useState(0.45)
  const [stepHoldS,       setStepHoldS]       = useState(1.5)
  const [numSteps,        setNumSteps]        = useState(4)
  const [signals, setSignals] = useState(
    Object.fromEntries(SIGNAL_CONFIG.map((s) => [s.key, DEFAULT_VISIBLE.has(s.key)]))
  )
  const [running,  setRunning]  = useState(false)
  const [result,   setResult]   = useState(null)
  const [runError, setRunError] = useState(null)

  const cfgInit = useRef(false)
  const posInit = useRef(false)

  // Seed gains from robot config once on first load.
  useEffect(() => {
    if (cfgInit.current || !config) return
    cfgInit.current = true
    if (config.position_kp != null) setTestKp(Number(config.position_kp))
    if (config.position_ki != null) setTestKi(Number(config.position_ki))
    if (config.torque_limit != null) setTestTorqueLimit(Number(config.torque_limit))
  }, [config])

  // Seed center_rad from current motor position once telemetry arrives.
  useEffect(() => {
    if (posInit.current || state?.position == null) return
    posInit.current = true
    setCenterRad(parseFloat(state.position.toFixed(3)))
  }, [state])

  const motorEnabled = state?.mode === MODE_POSITION

  async function runTest() {
    setRunning(true)
    setRunError(null)
    setResult(null)
    try {
      const res = await api.runStepTest(jointName, {
        position_kp: testKp,
        position_ki: testKi,
        torque_limit: testTorqueLimit,
        center_rad: centerRad,
        offset_rad: offsetRad,
        step_hold_s: stepHoldS,
        num_steps: numSteps,
      })
      setResult(res)
    } catch (e) {
      const msg = `Step test failed: ${e.message}`
      setRunError(msg)
      onLogError?.(msg, 'Auto-Tune')
    }
    setRunning(false)
  }

  async function applyGains() {
    try {
      await api.applyMotorConfig(jointName, {
        position_kp: testKp,
        position_ki: testKi,
        torque_limit: testTorqueLimit,
      })
    } catch (e) {
      const msg = `Apply gains failed: ${e.message}`
      setRunError(msg)
      onLogError?.(msg, 'Auto-Tune')
    }
  }

  const metrics = result?.metrics ?? null
  const samples = result?.samples ?? []

  return (
    <div className="flex-1 overflow-y-auto p-3 space-y-4">
      {/* ── Gains ───────────────────────────────────────────────────── */}
      <section className="space-y-2">
        <SectionLabel>Gains for Test</SectionLabel>
        <div className="grid grid-cols-3 gap-2">
          <Field label="Position Kp" value={testKp}          onChange={(v) => setTestKp(Number(v))} />
          <Field label="Position Ki" value={testKi}          onChange={(v) => setTestKi(Number(v))} />
          <Field label="Torque Limit (Nm)" value={testTorqueLimit} onChange={(v) => setTestTorqueLimit(Number(v))} />
        </div>
      </section>

      {/* ── Step parameters ─────────────────────────────────────────── */}
      <section className="space-y-2">
        <SectionLabel>Step Parameters</SectionLabel>
        <div className="grid grid-cols-2 gap-2">
          <Field label="Center (rad)" value={centerRad} onChange={(v) => setCenterRad(Number(v))} />
          <Field label="Offset (rad)" value={offsetRad} onChange={(v) => setOffsetRad(Number(v))} />
          <Field label="Hold time (s)" value={stepHoldS} onChange={(v) => setStepHoldS(Number(v))} />
          <Field label="Steps" value={numSteps} onChange={(v) => setNumSteps(Math.max(1, Math.round(Number(v))))} />
        </div>
        <p className="text-[10px] text-gray-600">
          Moves between{' '}
          <span className="font-mono text-gray-400">{(centerRad - offsetRad).toFixed(3)}</span>
          {' '}and{' '}
          <span className="font-mono text-gray-400">{(centerRad + offsetRad).toFixed(3)}</span>
          {' '}rad
        </p>
      </section>

      {/* ── Run button ──────────────────────────────────────────────── */}
      <div className="space-y-1.5">
        <button
          onClick={runTest}
          disabled={running || !motorEnabled}
          className="w-full py-2 rounded-lg text-sm font-medium bg-accent/20 text-accent
            border border-accent/30 hover:bg-accent/30 disabled:opacity-40 transition-colors"
        >
          {running
            ? `Running… (${numSteps} steps × ${stepHoldS}s)`
            : 'Run Step Test'}
        </button>
        {!motorEnabled && !running && (
          <p className="text-[10px] text-warn text-center">
            Motor must be in POSITION mode (Enable in Move tab)
          </p>
        )}
        {runError && (
          <p className="text-[10px] text-danger font-mono break-words">{runError}</p>
        )}
      </div>

      {/* ── Results ─────────────────────────────────────────────────── */}
      {result && (
        <>
          {/* Signal toggles */}
          <section className="space-y-2">
            <SectionLabel>Signals</SectionLabel>
            <div className="flex flex-wrap gap-1.5">
              {SIGNAL_CONFIG.map(({ key, label, color }) => (
                <button
                  key={key}
                  onClick={() => setSignals((s) => ({ ...s, [key]: !s[key] }))}
                  className={`flex items-center gap-1.5 px-2 py-0.5 rounded border text-[10px] transition-colors
                    ${signals[key]
                      ? 'border-current bg-surface-2'
                      : 'border-surface-3 text-gray-600'
                    }`}
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

          {/* Chart */}
          <section>
            <ResponsiveContainer width="100%" height={240}>
              <ComposedChart
                data={samples}
                margin={{ top: 4, right: 48, bottom: 4, left: 0 }}
              >
                <XAxis
                  dataKey="t_ms"
                  tickFormatter={(v) => `${(v / 1000).toFixed(1)}s`}
                  tick={{ fontSize: 9, fill: '#6b7280' }}
                  stroke="#374151"
                />
                <YAxis
                  yAxisId="pos"
                  tick={{ fontSize: 9, fill: '#6b7280' }}
                  tickFormatter={(v) => v.toFixed(2)}
                  stroke="#374151"
                  width={42}
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
                            {p.name}:{' '}
                            {typeof p.value === 'number' ? p.value.toFixed(4) : '—'}
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

          {/* Metrics */}
          {metrics && (
            <section className="space-y-2">
              <SectionLabel>Metrics</SectionLabel>
              {metrics.torque_saturated && (
                <div className="text-[10px] text-amber-400 px-2.5 py-1.5 bg-amber-400/10 rounded border border-amber-400/30">
                  Torque saturated — lower Kp or raise Torque Limit
                </div>
              )}
              <div className="grid grid-cols-[1fr_auto] gap-x-3 gap-y-0.5 text-[10px]">
                <MetricRow
                  label="Max overshoot"
                  value={`${metrics.max_overshoot_pct?.toFixed(1)}%`}
                />
                <MetricRow
                  label="Settling time"
                  value={metrics.settling_time_ms != null ? `${metrics.settling_time_ms} ms` : 'N/A'}
                />
                <MetricRow
                  label="Steady-state error"
                  value={
                    metrics.steady_state_error_rad != null
                      ? `${metrics.steady_state_error_rad.toFixed(4)} rad`
                      : 'N/A'
                  }
                />
                <MetricRow
                  label="Max torque"
                  value={`${metrics.max_torque_nm?.toFixed(2)} Nm`}
                  warn={metrics.torque_saturated}
                />
                <MetricRow
                  label="Max current"
                  value={`${metrics.max_current_a?.toFixed(2)} A`}
                />
              </div>
            </section>
          )}

          {/* Apply */}
          <button
            onClick={applyGains}
            className="w-full py-1.5 rounded-lg text-xs font-medium bg-surface-2 text-gray-300
              border border-surface-3 hover:border-accent/40 hover:text-accent transition-colors"
          >
            Apply These Gains to ESC
          </button>
          <p className="text-[9px] text-gray-600 text-center">
            Use "Store to Flash" in the Tune tab to persist gains after reboot
          </p>
        </>
      )}
    </div>
  )
}

function SectionLabel({ children }) {
  return (
    <p className="text-[10px] font-semibold text-gray-500 uppercase tracking-wider">
      {children}
    </p>
  )
}

function Field({ label, value, onChange }) {
  return (
    <div className="space-y-0.5">
      <p className="text-[9px] text-gray-600">{label}</p>
      <input
        type="number"
        step="any"
        value={value}
        onChange={(e) => onChange(e.target.value)}
        className="w-full px-2 py-1 rounded border border-surface-3 bg-surface-2
          text-xs font-mono text-gray-200 outline-none focus:border-accent/50 transition-colors"
      />
    </div>
  )
}

function MetricRow({ label, value, warn = false }) {
  return (
    <>
      <span className="text-gray-500">{label}</span>
      <span className={`font-mono text-right ${warn ? 'text-amber-400' : 'text-gray-200'}`}>
        {value}
      </span>
    </>
  )
}
