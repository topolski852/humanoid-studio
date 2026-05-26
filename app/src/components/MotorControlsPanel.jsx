import { useState, useRef, useEffect } from 'react'
import { api } from '../api'
import { useControlWebSocket } from '../hooks/useControlWebSocket'

const DEG = Math.PI / 180
const JOG_STEP_DEG = 1

// Modes where the motor actively accepts position/torque/current commands
const ACTIVE_MODES = new Set(['POSITION', 'VELOCITY', 'TORQUE', 'CURRENT'])

function SectionLabel({ children }) {
  return <p className="data-label mb-2">{children}</p>
}

export default function MotorControlsPanel({ jointName, state, config, onLogError }) {
  const isConnected = state != null
  const isEnabled   = ACTIVE_MODES.has(state?.mode_name)

  const [busy,    setBusy]    = useState(false)
  const [clearing, setClearing] = useState(false)

  const { sendPositionCommand, latencyMs, connected: wsConnected } = useControlWebSocket()

  // Position jog
  const [jogTargetDeg, setJogTargetDeg] = useState(0)
  const [jogError,     setJogError]     = useState(null)
  const [isRunning,    setIsRunning]    = useState(false)
  const [runTarget,    setRunTarget]    = useState(null)  // rad

  // Sine wave
  const [sineRunning,   setSineRunning]   = useState(false)
  const [sineFreq,      setSineFreq]      = useState(0.5)
  const [sineAmpDeg,    setSineAmpDeg]    = useState(30)
  const [sineOffsetDeg, setSineOffsetDeg] = useState(0.0)
  const sineRef = useRef(null)

  const minRad = config?.position_limits?.min ?? null
  const maxRad = config?.position_limits?.max ?? null
  const minDeg = minRad != null ? minRad / DEG : -180
  const maxDeg = maxRad != null ? maxRad / DEG : 180

  const clampDeg = (v) => Math.min(maxDeg, Math.max(minDeg, v))

  // ── Position run completion detection ──────────────────────────────────────
  useEffect(() => {
    if (!isRunning || runTarget === null || state == null) return
    const posErr = Math.abs((state.position ?? 0) - runTarget)
    const velAbs = Math.abs(state.velocity ?? 0)
    if (posErr < 0.02 && velAbs < 0.05) {  // ~1.1° tolerance, 0.05 rad/s
      setIsRunning(false)
      setRunTarget(null)
    }
  }, [state, isRunning, runTarget])

  // ── Run timeout (8 s) ─────────────────────────────────────────────────────
  useEffect(() => {
    if (!isRunning) return
    const t = setTimeout(() => { setIsRunning(false); setRunTarget(null) }, 8000)
    return () => clearTimeout(t)
  }, [isRunning])

  // ── Power controls ─────────────────────────────────────────────────────────
  async function enable() {
    setBusy(true)
    try { await api.enableMotor(jointName, 'POSITION') } catch (e) { console.error(e) }
    setBusy(false)
  }

  async function toIdle() {
    setBusy(true)
    if (sineRunning) stopSine()
    if (isRunning)   { setIsRunning(false); setRunTarget(null) }
    try { await api.disableMotor(jointName) } catch (e) { console.error(e) }
    setBusy(false)
  }

  async function doEstop() {
    setBusy(true)
    if (sineRunning) stopSine()
    if (isRunning)   { setIsRunning(false); setRunTarget(null) }
    try { await api.estopMotor(jointName) } catch (e) { console.error(e) }
    setBusy(false)
  }

  async function clearError() {
    setClearing(true)
    try { await api.clearMotorError(jointName) } catch (e) { onLogError?.(e.message, 'ClearError') }
    setClearing(false)
  }

  // ── Position jog ───────────────────────────────────────────────────────────
  async function runPosition() {
    const targetRad = jogTargetDeg * DEG
    setIsRunning(true)
    setRunTarget(targetRad)
    setJogError(null)
    try {
      const sent = sendPositionCommand(jointName, targetRad)
      if (!sent) await api.setPosition(jointName, targetRad)
    } catch (e) {
      setJogError(e.message)
      setIsRunning(false)
      setRunTarget(null)
      onLogError?.(e.message, 'Move')
    }
  }

  // ── Sine wave ──────────────────────────────────────────────────────────────
  const sineMaxAmpDeg = minRad != null && maxRad != null
    ? Math.min(maxDeg - sineOffsetDeg, sineOffsetDeg - minDeg)
    : 180

  function startSine() {
    if (sineRef.current) return
    setSineRunning(true)
    const offsetRad = sineOffsetDeg * DEG
    sineRef.current = setInterval(() => {
      const clampedAmp = Math.min(sineAmpDeg, Math.max(0, sineMaxAmpDeg - 0.1))
      const target = offsetRad + Math.sin(2 * Math.PI * sineFreq * (Date.now() / 1000)) * (clampedAmp * DEG)
      const sent = sendPositionCommand(jointName, target)
      if (!sent) api.setPosition(jointName, target).catch(() => {})
    }, 100)
  }

  function stopSine() {
    clearInterval(sineRef.current)
    sineRef.current = null
    setSineRunning(false)
    const offsetRad = sineOffsetDeg * DEG
    const sent = sendPositionCommand(jointName, offsetRad)
    if (!sent) api.setPosition(jointName, offsetRad).catch(() => {})
  }

  useEffect(() => () => { if (sineRef.current) clearInterval(sineRef.current) }, [])

  // ── Render ─────────────────────────────────────────────────────────────────
  return (
    <div className="flex-1 p-6 overflow-y-auto">
      {/* Not-connected notice */}
      {!isConnected && (
        <div className="mb-5 px-3 py-2.5 rounded-lg bg-surface-2 border border-surface-3">
          <p className="text-xs text-gray-400">Motor visible via passive CAN.</p>
          <p className="text-[10px] text-gray-600 mt-0.5">
            Connect the robot (top-right) to send commands.
          </p>
        </div>
      )}

      {/* ── ESC Error banner ── */}
      {isConnected && state?.error !== 0 && state?.error != null && (
        <div className="mb-6 px-3 py-2.5 rounded-lg bg-danger/10 border border-danger/30">
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

      {/* WS control channel status */}
      <div className="mb-4 flex items-center gap-2 text-[10px] font-mono text-gray-500">
        <span className={`w-1.5 h-1.5 rounded-full flex-shrink-0 ${wsConnected ? 'bg-online' : 'bg-offline'}`} />
        {wsConnected
          ? <span>WS control {latencyMs != null ? <span className="text-gray-400">{latencyMs} ms</span> : null}</span>
          : <span>WS offline — using HTTP (10 Hz)</span>
        }
      </div>

      {/* ── Power ── */}
      <div className="mb-6">
        <SectionLabel>POWER</SectionLabel>
        {isEnabled ? (
          <div className="flex gap-2">
            <button
              onClick={toIdle}
              disabled={busy || !isConnected}
              className="flex-1 py-2.5 rounded-lg text-sm font-medium transition-colors
                bg-surface-2 text-gray-300 border border-surface-3
                hover:border-accent/40 hover:text-accent disabled:opacity-40"
            >
              {busy ? '…' : 'To Idle'}
            </button>
            <button
              onClick={doEstop}
              disabled={busy || !isConnected}
              className="flex-[2] py-2.5 rounded-lg text-sm font-medium transition-colors
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
            className="w-full py-2.5 rounded-lg text-sm font-medium transition-colors
              bg-accent/20 text-accent border border-accent/30
              hover:bg-accent/30 disabled:opacity-40"
          >
            {busy ? '…' : '▶ Enable (Position)'}
          </button>
        )}
        {isConnected && (
          <p className="mt-1 text-[10px] text-gray-600 text-center font-mono">
            {state?.mode_name ?? '—'}
          </p>
        )}
      </div>

      {/* ── Position Jog ── */}
      <div className="mb-6">
        <SectionLabel>POSITION JOG</SectionLabel>

        {/* Target display */}
        <div className="flex items-center gap-2 mb-3">
          <button
            onClick={() => setJogTargetDeg(clampDeg(jogTargetDeg - JOG_STEP_DEG))}
            disabled={!isEnabled || isRunning}
            className="w-10 h-10 rounded-lg bg-surface-2 border border-surface-3 font-mono text-lg
              hover:border-accent/50 hover:bg-accent/10 hover:text-accent transition-colors
              disabled:opacity-40 disabled:cursor-not-allowed"
          >−</button>
          <div className="flex-1 text-center">
            <span className="font-mono text-xl tabular-nums">{jogTargetDeg.toFixed(1)}</span>
            <span className="text-xs text-gray-500 ml-1">°</span>
          </div>
          <button
            onClick={() => setJogTargetDeg(clampDeg(jogTargetDeg + JOG_STEP_DEG))}
            disabled={!isEnabled || isRunning}
            className="w-10 h-10 rounded-lg bg-surface-2 border border-surface-3 font-mono text-lg
              hover:border-accent/50 hover:bg-accent/10 hover:text-accent transition-colors
              disabled:opacity-40 disabled:cursor-not-allowed"
          >+</button>
        </div>

        <input
          type="range"
          min={minDeg}
          max={maxDeg}
          step={1}
          value={jogTargetDeg}
          onChange={(e) => setJogTargetDeg(parseFloat(e.target.value))}
          disabled={!isEnabled || isRunning}
          className="w-full disabled:opacity-40"
        />

        <div className="flex justify-between font-mono text-[10px] text-gray-600 mt-1 mb-3">
          <span>{minDeg.toFixed(0)}°</span>
          <span>{maxDeg.toFixed(0)}°</span>
        </div>

        {/* Preset buttons */}
        <div className="flex gap-2 mb-3">
          {[-90, -45, 0, 45, 90].map((preset) => (
            <button
              key={preset}
              onClick={() => setJogTargetDeg(clampDeg(preset))}
              disabled={!isEnabled || isRunning}
              className="flex-1 py-1 rounded text-[10px] font-mono bg-surface-2 border border-surface-3
                hover:border-accent/40 hover:text-accent transition-colors disabled:opacity-30"
            >
              {preset}°
            </button>
          ))}
        </div>

        {/* Run button */}
        <button
          onClick={runPosition}
          disabled={!isEnabled || isRunning}
          className="w-full py-2 rounded-lg text-xs font-medium transition-colors border
            bg-accent/10 text-accent border-accent/30 hover:bg-accent/20
            disabled:opacity-40"
        >
          {isRunning ? (
            <span className="flex items-center gap-2 justify-center">
              <span className="w-3 h-3 rounded-full border-2 border-accent border-t-transparent animate-spin" />
              Moving to {jogTargetDeg.toFixed(1)}°…
            </span>
          ) : 'Run to Position'}
        </button>

        {jogError && (
          <p className="mt-2 text-xs text-danger">{jogError}</p>
        )}
      </div>

      {/* ── Sine Wave Test ── */}
      <div className="mb-6">
        <SectionLabel>SINE WAVE TEST</SectionLabel>
        <div className="mb-3 space-y-2">
          <div className="flex items-center gap-3">
            <span className="text-[10px] text-gray-500 w-16 shrink-0">Center</span>
            <input
              type="number"
              step={1}
              value={sineOffsetDeg}
              onChange={(e) => setSineOffsetDeg(parseFloat(e.target.value) || 0)}
              disabled={sineRunning}
              className="flex-1 bg-surface-2 border border-surface-3 rounded px-2 py-1 text-xs font-mono text-center disabled:opacity-40"
            />
            <span className="font-mono text-xs text-gray-300 w-12 text-right">° offset</span>
          </div>
          <div className="flex items-center gap-3">
            <span className="text-[10px] text-gray-500 w-16 shrink-0">Freq</span>
            <input
              type="range"
              min={0.1} max={2.0} step={0.1}
              value={sineFreq}
              onChange={(e) => setSineFreq(parseFloat(e.target.value))}
              disabled={sineRunning}
              className="flex-1 disabled:opacity-40"
            />
            <span className="font-mono text-xs text-gray-300 w-12 text-right">{sineFreq.toFixed(1)} Hz</span>
          </div>
          <div className="flex items-center gap-3">
            <span className="text-[10px] text-gray-500 w-16 shrink-0">Amplitude</span>
            <input
              type="number"
              min={1} max={180} step={1}
              value={sineAmpDeg}
              onChange={(e) => setSineAmpDeg(Math.max(1, Math.min(180, parseInt(e.target.value) || 1)))}
              disabled={sineRunning}
              className="flex-1 bg-surface-2 border border-surface-3 rounded px-2 py-1 text-xs font-mono text-center disabled:opacity-40"
            />
            <span className="font-mono text-xs text-gray-300 w-12 text-right">° pk</span>
          </div>
        </div>
        {sineAmpDeg > sineMaxAmpDeg && (
          <p className="text-[10px] text-amber-400 mb-2">
            Exceeds joint limits at this center — max safe amplitude: {Math.max(0, sineMaxAmpDeg).toFixed(0)}°
          </p>
        )}
        {!isEnabled && (
          <p className="text-[10px] text-gray-500 mb-2">Enable motor first</p>
        )}
        <button
          onClick={sineRunning ? stopSine : startSine}
          disabled={!isEnabled}
          className={`w-full py-2 rounded-lg text-xs font-medium transition-colors border
            ${sineRunning
              ? 'bg-danger/20 text-danger border-danger/30 hover:bg-danger/30'
              : 'bg-surface-2 text-gray-300 border-surface-3 hover:border-accent/40 hover:text-accent'
            } disabled:opacity-40`}
        >
          {sineRunning ? '⏹ Stop Sine' : '▶ Start Sine'}
        </button>
      </div>
    </div>
  )
}
