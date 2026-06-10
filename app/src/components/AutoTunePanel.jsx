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

const DEG = Math.PI / 180
const R2D = 180 / Math.PI

const SIGNAL_CONFIG = [
  { key: 'commanded',   label: 'Position Cmd (°)',  color: '#3b82f6', yAxisId: 'pos',   strokeDasharray: '4 2' },
  { key: 'position',    label: 'Position Meas (°)', color: '#22c55e', yAxisId: 'pos' },
  { key: 'velocity',    label: 'Velocity (°/s)',    color: '#eab308', yAxisId: 'other' },
  { key: 'torque',      label: 'Torque (Nm)',       color: '#f97316', yAxisId: 'other' },
  { key: 'current',     label: 'Current (A)',       color: '#ef4444', yAxisId: 'other' },
  { key: 'bus_voltage', label: 'Bus Voltage (V)',   color: '#94a3b8', yAxisId: 'other' },
]

const DEFAULT_VISIBLE = new Set(['commanded', 'position', 'velocity', 'torque'])

// Convert raw samples (rad) to display units (deg) for chart
function toDisplaySamples(samples) {
  return samples.map((s) => ({
    ...s,
    commanded:   s.commanded   != null ? s.commanded   * R2D : null,
    position:    s.position    != null ? s.position    * R2D : null,
    velocity:    s.velocity    != null ? s.velocity    * R2D : null,
  }))
}

const ACTIVE_MODES = new Set(['POSITION', 'VELOCITY', 'TORQUE', 'CURRENT'])

// ── Suggestion algorithm ───────────────────────────────────────────────────────

const round1 = (v) => Math.round(v * 10) / 10
const round2 = (v) => Math.round(v * 100) / 100

/**
 * Compute suggested gains from step-test metrics.
 * params: { position_kp, position_ki, torque_limit, offset_rad }
 * Returns: { kp, ki, torqueLimit, rationale: string[], quality: 'good'|'marginal'|'poor' }
 */
function computeSuggestion(metrics, params) {
  const {
    max_overshoot_pct: overshoot,
    settling_time_ms:  settling,
    steady_state_error_rad: sseRad,
    max_torque_nm,
    torque_saturated,
  } = metrics
  const { position_kp: kp, position_ki: ki, torque_limit, offset_rad } = params

  const rationale = []
  let newKp          = kp
  let newKi          = ki
  let newTorqueLimit = torque_limit
  let quality        = 'good'

  // ── Kp / torque-limit decision (priority order) ───────────────────────────
  // All adjustments are intentionally small and incremental.  Multiple test
  // iterations are needed to converge — that is safer than large single jumps.
  // Hard cap: never suggest more than −12% or +8% in one step regardless of
  // what the heuristic computes.
  const kpFloor = round1(kp * 0.88)   // max reduction per step: −12%
  const kpCeil  = round1(kp * 1.08)   // max increase per step:  +8%

  if (torque_saturated) {
    rationale.push('Torque saturated — response shape unreliable; raise limit or reduce Kp')
    newKp          = round1(kp * 0.90)   // −10%
    newTorqueLimit = round1(max_torque_nm * 1.5)
    quality        = 'poor'
  } else if (overshoot > 20) {
    rationale.push(`High overshoot (${overshoot.toFixed(1)}%) — reduce Kp`)
    newKp    = round1(kp * 0.88)         // −12%
    quality  = 'poor'
  } else if (overshoot > 10) {
    rationale.push(`Overshoot ${overshoot.toFixed(1)}% — reduce Kp`)
    newKp   = round1(kp * 0.92)          // −8%
    quality = 'marginal'
  } else if (overshoot > 8) {
    rationale.push(`Overshoot ${overshoot.toFixed(1)}% — small Kp reduction`)
    newKp   = round1(kp * 0.96)          // −4%
    quality = 'marginal'
  } else if (overshoot >= 2 && overshoot <= 8) {
    if (settling != null && settling < 500) {
      rationale.push(`Overshoot ${overshoot.toFixed(1)}%, settling ${settling} ms — response is good`)
      quality = 'good'
    } else if (settling != null && settling > 1000) {
      rationale.push(`Low overshoot but slow settling (${settling} ms) — increase Kp slightly`)
      newKp   = round1(kp * 1.05)        // +5%
      quality = 'marginal'
    } else {
      rationale.push(`Overshoot ${overshoot.toFixed(1)}% — within target range`)
      quality = 'good'
    }
  } else {
    // overshoot < 2% — overdamped or slow
    if (settling == null || settling > 800) {
      rationale.push(settling == null
        ? 'Never settled within 2% band — increase Kp'
        : `Low overshoot, slow settling (${settling} ms) — increase Kp`)
      newKp   = round1(kp * 1.08)        // +8%
      quality = 'marginal'
    } else {
      rationale.push(`Low overshoot (${overshoot.toFixed(1)}%), fast settle — critically damped`)
      quality = 'good'
    }
  }

  // Hard safety cap — clamp to ±12%/+8% regardless of heuristic above
  newKp = Math.max(kpFloor, Math.min(kpCeil, newKp))

  // Ki is held constant — integral wind-up risk is high without knowing firmware integrator limits.
  // Tune Ki manually in the Tune tab after Kp is satisfactory.
  if (sseRad != null) {
    const sseDeg = sseRad * R2D
    if (sseDeg > 2.0 && ki === 0) {
      rationale.push(`Steady-state error ${sseDeg.toFixed(1)}° — consider adding a small Ki in the Tune tab once Kp is set`)
    }
  }

  return { kp: newKp, ki: newKi, torqueLimit: newTorqueLimit, rationale, quality }
}

export default function AutoTunePanel({ jointName, state, config, onLogError }) {
  const [testKp,          setTestKp]          = useState(20.0)
  const [testKi,          setTestKi]          = useState(0.0)
  const [testTorqueLimit, setTestTorqueLimit] = useState(2.0)
  const [centerDeg,       setCenterDeg]       = useState(0.0)
  const [offsetDeg,       setOffsetDeg]       = useState(25.0)
  const [stepHoldS,       setStepHoldS]       = useState(1.5)
  const [numSteps,        setNumSteps]        = useState(4)
  const [signals, setSignals] = useState(
    Object.fromEntries(SIGNAL_CONFIG.map((s) => [s.key, DEFAULT_VISIBLE.has(s.key)]))
  )
  const [running,   setRunning]   = useState(false)
  const [result,    setResult]    = useState(null)
  const [runError,  setRunError]  = useState(null)
  const [busy,              setBusy]              = useState(false)
  const [clearing,          setClearing]          = useState(false)
  const [applyingSuggestion, setApplyingSuggestion] = useState(false)
  const [triedSuggestion,    setTriedSuggestion]    = useState(false)
  // Gains and offset that were ACTUALLY USED for the last test run.
  // Suggestion is computed from these — not from the current form values —
  // so clicking "Try These Gains" cannot cascade the suggestion.
  const [lastTestParams, setLastTestParams] = useState(null)

  const cfgInit    = useRef(false)
  const posInit    = useRef(false)
  // AbortController for the in-flight step test fetch.
  // Aborted on component unmount (tab switch / navigate away) so the fetch
  // cannot keep running in the background and leave running=true stuck true.
  const abortRef   = useRef(null)

  useEffect(() => {
    return () => { abortRef.current?.abort() }
  }, [])

  const isConnected = state != null
  const isEnabled   = ACTIVE_MODES.has(state?.mode_name)
  const motorEnabled = state?.mode_name === 'POSITION'

  // Seed gains from robot config once on first load
  useEffect(() => {
    if (cfgInit.current || !config) return
    cfgInit.current = true
    if (config.position_kp != null) setTestKp(Number(config.position_kp))
    if (config.position_ki != null) setTestKi(Number(config.position_ki))
    if (config.torque_limit != null) setTestTorqueLimit(Number(config.torque_limit))
  }, [config])

  // Seed center from current motor position (in degrees) once telemetry arrives
  useEffect(() => {
    if (posInit.current || state?.position == null) return
    posInit.current = true
    setCenterDeg(parseFloat((state.position * R2D).toFixed(1)))
  }, [state])

  // ── Motor control ───────────────────────────────────────────────────────────
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
    // If a step test is running, interrupt it immediately
    setRunning(false)
    try { await api.estopMotor(jointName) } catch (e) { onLogError?.(e.message, 'E-Stop') }
    setBusy(false)
  }

  async function clearError() {
    setClearing(true)
    try { await api.clearMotorError(jointName) } catch (e) { onLogError?.(e.message, 'ClearError') }
    setClearing(false)
  }

  // ── Step test ───────────────────────────────────────────────────────────────
  async function runTest() {
    abortRef.current?.abort()
    abortRef.current = new AbortController()

    setRunning(true)
    setRunError(null)
    setResult(null)
    setTriedSuggestion(false)
    // Snapshot the gains and offset at the moment this test starts.
    // The suggestion card uses this snapshot, not the live form values,
    // so "Try These Gains" cannot cascade the suggestion without a new test.
    const params = {
      position_kp:  testKp,
      position_ki:  testKi,
      torque_limit: testTorqueLimit,
      offset_rad:   offsetDeg * DEG,
    }
    setLastTestParams(params)
    try {
      const res = await api.runStepTest(jointName, {
        position_kp:   testKp,
        position_ki:   testKi,
        torque_limit:  testTorqueLimit,
        center_rad:    centerDeg * DEG,
        offset_rad:    offsetDeg * DEG,
        step_hold_s:   stepHoldS,
        num_steps:     numSteps,
      }, abortRef.current.signal)
      setResult(res)
    } catch (e) {
      if (e.name === 'AbortError') return  // tab switch or navigate — don't set error state
      const msg = `Step test failed: ${e.message}`
      setRunError(msg)
      onLogError?.(msg, 'Auto-Tune')
    }
    setRunning(false)
  }

  async function applyGains() {
    try {
      await api.writeMotorGains(jointName, testKp, testKi, testTorqueLimit)
    } catch (e) {
      const msg = `Apply gains failed: ${e.message}`
      setRunError(msg)
      onLogError?.(msg, 'Auto-Tune')
    }
  }

  // ── Suggestion actions ──────────────────────────────────────────────────────
  function trySuggestion(s) {
    setTestKp(s.kp)
    setTestKi(s.ki)
    setTestTorqueLimit(s.torqueLimit)
    setTriedSuggestion(true)
  }

  async function applySuggestion(s) {
    setApplyingSuggestion(true)
    try {
      await api.writeMotorGains(jointName, s.kp, s.ki, s.torqueLimit)
    } catch (e) {
      const msg = `Apply suggestion failed: ${e.message}`
      setRunError(msg)
      onLogError?.(msg, 'Auto-Tune')
    }
    setApplyingSuggestion(false)
  }

  const metrics        = result?.metrics ?? null
  const displaySamples = toDisplaySamples(result?.samples ?? [])
  // Use the frozen snapshot from the last test run — never the live form values.
  const suggestion     = (metrics && lastTestParams)
    ? computeSuggestion(metrics, lastTestParams)
    : null

  return (
    <div className="flex-1 min-w-0 overflow-y-auto p-3 space-y-4">

      {/* ── Motor not connected notice ───────────────────────────────────── */}
      {!isConnected && (
        <div className="px-3 py-2.5 rounded-lg bg-surface-2 border border-surface-3">
          <p className="text-xs text-gray-400">Motor visible via passive CAN.</p>
          <p className="text-[10px] text-gray-600 mt-0.5">
            Connect the robot (top-right) to send commands.
          </p>
        </div>
      )}

      {/* ── ESC error banner ─────────────────────────────────────────────── */}
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

      {/* ── Power (Enable / Idle / E-Stop) ───────────────────────────────── */}
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
          <p className="text-[10px] text-gray-600 text-center font-mono">
            {state?.mode_name ?? '—'}
          </p>
        )}
      </section>

      {/* ── Gains ────────────────────────────────────────────────────────── */}
      <section className="space-y-2">
        <SectionLabel>Gains for Test</SectionLabel>
        <div className="grid grid-cols-3 gap-2">
          <Field label="Position Kp"      value={testKp}          onChange={(v) => setTestKp(Number(v))} />
          <Field label="Position Ki"      value={testKi}          onChange={(v) => setTestKi(Number(v))} />
          <Field label="Torque Limit (Nm)" value={testTorqueLimit} onChange={(v) => setTestTorqueLimit(Number(v))} />
        </div>
      </section>

      {/* ── Step parameters ──────────────────────────────────────────────── */}
      <section className="space-y-2">
        <SectionLabel>Step Parameters</SectionLabel>
        <div className="grid grid-cols-2 gap-2">
          <Field label="Center (°)"    value={centerDeg} onChange={(v) => setCenterDeg(Number(v))} />
          <Field label="Offset (°)"    value={offsetDeg} onChange={(v) => setOffsetDeg(Number(v))} />
          <Field label="Hold time (s)" value={stepHoldS} onChange={(v) => setStepHoldS(Number(v))} />
          <Field label="Steps"         value={numSteps}  onChange={(v) => setNumSteps(Math.max(1, Math.round(Number(v))))} />
        </div>
        <p className="text-[10px] text-gray-600">
          Moves between{' '}
          <span className="font-mono text-gray-400">{(centerDeg - offsetDeg).toFixed(1)}°</span>
          {' '}and{' '}
          <span className="font-mono text-gray-400">{(centerDeg + offsetDeg).toFixed(1)}°</span>
        </p>
      </section>

      {/* ── Run button ───────────────────────────────────────────────────── */}
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
        {!motorEnabled && !running && isConnected && (
          <p className="text-[10px] text-warn text-center">
            Enable motor in Position mode above to run test
          </p>
        )}
        {runError && (
          <p className="text-[10px] text-danger font-mono break-words">{runError}</p>
        )}
      </div>

      {/* ── Results ──────────────────────────────────────────────────────── */}
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

          {/* Chart — overflow-hidden prevents the SVG event-capture rect from
               escaping its container and blocking clicks on the sidebar/tab bar */}
          <section className="w-full overflow-hidden">
            <ResponsiveContainer width="100%" height={240}>
              <ComposedChart
                data={displaySamples}
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
                            {p.name}:{' '}
                            {typeof p.value === 'number' ? p.value.toFixed(2) : '—'}
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
                      ? `${(metrics.steady_state_error_rad * R2D).toFixed(2)}°`
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

          {/* Suggestion card */}
          {suggestion && lastTestParams && (
            <SuggestionCard
              suggestion={suggestion}
              testedKp={lastTestParams.position_kp}
              tried={triedSuggestion}
              applying={applyingSuggestion}
              canApply={isConnected}
              onTry={() => trySuggestion(suggestion)}
              onApply={() => applySuggestion(suggestion)}
            />
          )}

          {/* Apply current test gains */}
          <button
            onClick={applyGains}
            className="w-full py-1.5 rounded-lg text-xs font-medium bg-surface-2 text-gray-300
              border border-surface-3 hover:border-accent/40 hover:text-accent transition-colors"
          >
            Apply Current Test Gains to ESC
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

function SuggestionCard({ suggestion, testedKp, tried, applying, canApply, onTry, onApply }) {
  const { kp, ki, torqueLimit, rationale, quality } = suggestion

  const borderCls = quality === 'good'     ? 'border-online/40'
                  : quality === 'marginal' ? 'border-warn/40'
                  :                          'border-danger/40'
  const badgeCls  = quality === 'good'     ? 'bg-online/10 text-online'
                  : quality === 'marginal' ? 'bg-warn/10 text-warn'
                  :                          'bg-danger/10 text-danger'
  const dotCls    = quality === 'good'     ? 'bg-online'
                  : quality === 'marginal' ? 'bg-warn'
                  :                          'bg-danger'

  return (
    <div className={`rounded-xl border p-3 space-y-3 bg-surface-2 ${borderCls}`}>
      {/* Header */}
      <div className="flex items-center justify-between">
        <div>
          <SectionLabel>Suggested Gains</SectionLabel>
          {testedKp != null && (
            <p className="text-[9px] text-gray-600 mt-0.5 font-mono">
              based on test at Kp {testedKp}
            </p>
          )}
        </div>
        <span className={`flex items-center gap-1.5 text-[9px] font-medium px-2 py-0.5 rounded ${badgeCls}`}>
          <span className={`w-1.5 h-1.5 rounded-full ${dotCls}`} />
          {quality.toUpperCase()}
        </span>
      </div>

      {/* Rationale */}
      <ul className="space-y-0.5">
        {rationale.map((r, i) => (
          <li key={i} className="text-[10px] text-gray-400 flex gap-1.5">
            <span className="text-gray-600 flex-shrink-0">•</span>
            {r}
          </li>
        ))}
      </ul>

      {/* Suggested values */}
      <div className="grid grid-cols-3 gap-2 text-center">
        {[
          { label: 'Kp',    value: kp },
          { label: 'Ki',    value: ki },
          { label: 'Limit', value: `${torqueLimit} Nm` },
        ].map(({ label, value }) => (
          <div key={label} className="bg-surface-1 rounded-lg py-1.5 px-2">
            <p className="text-[9px] text-gray-600">{label}</p>
            <p className="font-mono text-sm text-gray-200">{value}</p>
          </div>
        ))}
      </div>

      {/* Actions */}
      <div className="flex gap-2">
        <button
          onClick={onTry}
          className="flex-1 py-1.5 rounded-lg text-xs font-medium transition-colors
            bg-surface-1 text-gray-300 border border-surface-3
            hover:border-accent/40 hover:text-accent"
        >
          Try These Gains
        </button>
        <button
          onClick={onApply}
          disabled={applying || !canApply}
          title={!canApply ? 'Motor must be connected (not E-stopped) to apply gains' : undefined}
          className="flex-1 py-1.5 rounded-lg text-xs font-medium transition-colors
            bg-accent/20 text-accent border border-accent/30
            hover:bg-accent/30 disabled:opacity-40"
        >
          {applying ? 'Applying…' : 'Apply to ESC'}
        </button>
      </div>
      {!canApply && (
        <p className="text-[9px] text-gray-600 text-center">
          Motor must be connected (not E-stopped) to apply gains directly
        </p>
      )}

      {tried && (
        <p className="text-[9px] text-accent text-center">
          Gains loaded above — click Run Step Test to validate
        </p>
      )}
    </div>
  )
}
