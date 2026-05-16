import { useEffect, useRef, useState } from 'react'
import { api } from '../api'

// Steps matching FlashState.step_index (0-7)
const STEP_LABELS = [
  'Configure',    // 0 — pre-start
  'Init Flash',   // 1 — INIT_FLASH
  'Power Cycle',  // 2 — WAITING_POWER_CYCLE
  'Program',      // 3 — PROGRAM_FLASH / REFLASHING
  'Connect',      // 4 — WAITING_CAN_CONNECT
  'Calibrate',    // 5 — CALIBRATING / AWAITING_CONFIRMATION
  'Finalize',     // 6 — FINALIZE_FLASH
  'Done',         // 7 — COMPLETE
]

const ACTIVE_STATES = new Set([
  'INIT_FLASH', 'WAITING_POWER_CYCLE', 'PROGRAM_FLASH',
  'WAITING_CAN_CONNECT', 'CALIBRATING', 'AWAITING_CONFIRMATION',
  'REFLASHING', 'FINALIZE_FLASH',
])

// States where closing would leave the ESC in a broken intermediate state
const LOCKED_STATES = new Set([
  'INIT_FLASH', 'PROGRAM_FLASH', 'CALIBRATING', 'REFLASHING', 'FINALIZE_FLASH',
])


// ── Motor profile spec table ───────────────────────────────────────────────────
function ProfileTable({ profile }) {
  if (!profile) return null
  const fmt = (v) => (v != null ? v.toFixed ? v.toFixed(5) : v : '—')
  return (
    <div className="mt-2 bg-surface rounded border border-surface-3 text-[10px] font-mono">
      <div className="grid grid-cols-3 gap-x-4 px-3 py-2 text-gray-500 border-b border-surface-3">
        <span>Kt</span><span>Cal A</span><span>i_kp / i_ki</span>
      </div>
      <div className="grid grid-cols-3 gap-x-4 px-3 py-2 text-gray-300">
        <span>{profile.torque_constant != null ? `${profile.torque_constant} Nm/A` : <span className="text-warn">undef</span>}</span>
        <span>{profile.max_calibration_current} A</span>
        <span>{fmt(profile.i_kp)} / {Math.round(profile.i_ki)}</span>
      </div>
    </div>
  )
}


// ── Step progress strip ────────────────────────────────────────────────────────
function StepStrip({ stepIndex, isFailed, isComplete }) {
  // Show steps 1-8 (skip step 0 = Configure which is pre-start)
  const visibleSteps = STEP_LABELS.slice(1)
  return (
    <div className="flex items-center gap-0 overflow-x-auto pb-1">
      {visibleSteps.map((label, i) => {
        const idx = i + 1   // actual step_index
        const done = isComplete || idx < stepIndex
        const active = idx === stepIndex && !isComplete && !isFailed
        const failed = isFailed && idx === stepIndex

        return (
          <div key={label} className="flex items-center flex-shrink-0">
            <div className={`flex items-center gap-1 px-2 py-1 rounded-full text-[10px] whitespace-nowrap
              ${done ? 'bg-online/20 text-online' :
                active ? 'bg-accent/20 text-accent ring-1 ring-accent/40' :
                failed ? 'bg-danger/20 text-danger' :
                'text-gray-600'}`}
            >
              <span className="font-mono w-3 text-center">
                {done ? '✓' : failed ? '✗' : `${idx}`}
              </span>
              <span className="hidden sm:inline">{label}</span>
            </div>
            {i < visibleSteps.length - 1 && (
              <div className={`w-4 h-px mx-0.5 flex-shrink-0 ${done ? 'bg-online/40' : 'bg-surface-3'}`} />
            )}
          </div>
        )
      })}
    </div>
  )
}


// ── Power cycle waiting indicator ──────────────────────────────────────────────
function PowerCycleWaiting({ onManualConfirm }) {
  return (
    <div className="flex items-center gap-3">
      <div className="flex-1">
        <div className="flex items-center gap-2 mb-1">
          <span className="w-3 h-3 rounded-full bg-warn animate-pulse inline-block" />
          <p className="text-sm font-medium">Waiting for ESC to come back online…</p>
        </div>
        <p className="text-xs text-gray-500">
          Power cycle the ESC (disconnect / reconnect motor power).
          The wizard detects when it comes back — or click the button if you cycled it manually.
        </p>
      </div>
      <button
        onClick={onManualConfirm}
        className="btn-primary px-4 py-2 whitespace-nowrap flex-shrink-0"
      >
        ↻ Cycled
      </button>
    </div>
  )
}


// ── Calibration progress ───────────────────────────────────────────────────────
function CalibrationProgress({ startedAt }) {
  const [elapsed, setElapsed] = useState(0)
  useEffect(() => {
    const t = setInterval(() => setElapsed(Math.floor((Date.now() - startedAt) / 1000)), 500)
    return () => clearInterval(t)
  }, [startedAt])
  const est = 15
  const pct = Math.min(100, Math.round((elapsed / est) * 100))
  return (
    <div className="flex items-center gap-3">
      <span className="w-3.5 h-3.5 rounded-full border-2 border-accent border-t-transparent animate-spin flex-shrink-0" />
      <div className="flex-1">
        <p className="text-sm text-gray-300">Calibrating encoder flux offset…</p>
        <div className="mt-1.5 h-1.5 bg-surface-3 rounded-full overflow-hidden">
          <div
            className="h-full bg-accent rounded-full transition-all duration-500"
            style={{ width: `${pct}%` }}
          />
        </div>
        <p className="text-[10px] text-gray-600 mt-0.5 font-mono">{elapsed}s elapsed (est. ~15s)</p>
      </div>
    </div>
  )
}


// ── Log output ────────────────────────────────────────────────────────────────
function LogPane({ messages }) {
  const bottomRef = useRef(null)
  useEffect(() => {
    bottomRef.current?.scrollIntoView({ behavior: 'smooth' })
  }, [messages])
  return (
    <div className="flex-1 bg-surface rounded-lg border border-surface-3 p-3 overflow-y-auto font-mono text-xs leading-relaxed">
      {messages?.length === 0 && (
        <p className="text-gray-600">Waiting for log output…</p>
      )}
      {messages?.map((msg, i) => (
        <div key={i} className={`whitespace-pre-wrap break-all ${
          msg.startsWith('FAILED') || msg.includes('failed') || msg.includes('Error')
            ? 'text-danger'
            : msg.includes('complete') || msg.includes('OK') || msg.includes('done')
            ? 'text-online'
            : 'text-gray-400'
        }`}>
          <span className="text-gray-600 mr-2 select-none">&gt;</span>{msg}
        </div>
      ))}
      <div ref={bottomRef} />
    </div>
  )
}


// ── Main component ────────────────────────────────────────────────────────────
export default function FlashWizard({ canId, canChannel, jointName, onClose }) {
  const [status, setStatus]           = useState(null)
  const [stepInfo, setStepInfo]       = useState({ step_index: 0, total_steps: 8 })
  const [profiles, setProfiles]       = useState([])
  const [motorProfile, setMotorProfile] = useState('MAD_5010_200KV')
  const [invertPhase, setInvertPhase] = useState(false)
  const [started, setStarted]         = useState(false)
  const [startError, setStartError]   = useState(null)
  const [calStartedAt, setCalStartedAt] = useState(null)
  const [configSynced, setConfigSynced] = useState(false)
  const pollRef = useRef(null)

  // Reset backend state on mount so a fresh wizard never shows a previous session's data
  useEffect(() => {
    api.flashReset().catch(() => {})
    setStarted(false)
    setStartError(null)
    setConfigSynced(false)
  }, [])

  // Load motor profiles on mount
  useEffect(() => {
    api.flashProfiles()
      .then((d) => setProfiles(d.profiles ?? []))
      .catch(() => {})
  }, [])

  // Poll flash status while open
  useEffect(() => {
    async function poll() {
      try {
        const [s, step] = await Promise.all([api.flashStatus(), api.flashStep()])
        setStatus(s)
        setStepInfo(step)
        if (s.state === 'CALIBRATING' && !calStartedAt) {
          setCalStartedAt(Date.now())
        }
        if (s.state !== 'CALIBRATING') {
          setCalStartedAt(null)
        }
      } catch {}
    }
    poll()
    pollRef.current = setInterval(poll, 500)
    return () => clearInterval(pollRef.current)
  }, [calStartedAt])

  // Sync updated_config to humanoid_lite.json when wizard completes
  useEffect(() => {
    if (status?.state !== 'COMPLETE' || !status.updated_config || configSynced) return
    async function syncConfig() {
      try {
        const rc = await api.getRobotConfig()
        if (!rc?.joints?.[jointName]) return
        const updatedJoints = {
          ...rc.joints,
          [jointName]: { ...rc.joints[jointName], ...status.updated_config },
        }
        await api.putRobotConfig({ ...rc, joints: updatedJoints })
        setConfigSynced(true)
      } catch (e) {
        console.warn('Config sync after flash failed:', e)
      }
    }
    syncConfig()
  }, [status?.state, status?.updated_config, jointName, configSynced])

  async function handleStart() {
    setStartError(null)
    setStarted(true)
    try {
      await api.flashStart(canId, invertPhase, motorProfile, 'SWD', canChannel ?? 'can0')
    } catch (e) {
      setStartError(e.message)
      setStarted(false)
    }
  }

  async function handleReset() {
    try {
      await api.flashReset()
      setStartError(null)
      setStarted(false)
    } catch (e) {
      setStartError(e.message)
    }
  }

  async function handlePowerCycled() {
    try { await api.flashPowerCycled() } catch (e) { console.error(e) }
  }

  async function handleCanConnected() {
    try { await api.flashCanConnected() } catch (e) { console.error(e) }
  }

  async function handleDone() {
    await api.flashReset().catch(() => {})
    onClose()
  }

  async function handleConfirm(correct) {
    try { await api.flashConfirm(correct) } catch (e) { console.error(e) }
  }

  const currentState     = status?.state ?? 'IDLE'
  const stepIndex        = stepInfo.step_index ?? 0
  const isActive         = started && ACTIVE_STATES.has(currentState)
  const isComplete       = currentState === 'COMPLETE'
  const isFailed         = currentState === 'FAILED'
  const isLocked         = started && LOCKED_STATES.has(currentState)
  const awaitingPowerCycle  = currentState === 'WAITING_POWER_CYCLE'
  const awaitingCanConnect  = currentState === 'WAITING_CAN_CONNECT'
  const awaitingConfirm     = currentState === 'AWAITING_CONFIRMATION'
  const isCalibrating       = currentState === 'CALIBRATING'

  const selectedProfile  = profiles.find((p) => p.key === motorProfile) ?? null

  return (
    <div className="fixed inset-0 bg-black/70 backdrop-blur-sm flex items-center justify-center z-50 p-4">
      <div className="bg-surface-1 border border-surface-3 rounded-2xl w-full max-w-2xl flex flex-col shadow-2xl max-h-[90vh]">

        {/* ── Header ── */}
        <div className="flex items-center justify-between px-6 py-4 border-b border-surface-3 flex-shrink-0">
          <div>
            <h2 className="font-semibold text-base">ESC Flash Wizard</h2>
            <p className="text-xs text-gray-500 font-mono">
              {jointName} · {canChannel} · CAN ID {canId}
            </p>
          </div>
          <button
            onClick={onClose}
            disabled={isLocked}
            title={isLocked ? 'Cannot close while flashing' : 'Close'}
            className={`w-8 h-8 flex items-center justify-center rounded-lg text-xl transition-colors ${
              isLocked
                ? 'opacity-30 cursor-not-allowed text-gray-600'
                : 'hover:bg-white/10 text-gray-500 hover:text-white'
            }`}
          >
            ×
          </button>
        </div>

        {/* ── Step strip + progress bar ── */}
        <div className="px-6 py-3 border-b border-surface-3 space-y-2 flex-shrink-0">
          <StepStrip stepIndex={started ? stepIndex : 0} isFailed={isFailed} isComplete={isComplete} />
          {status && (
            <div>
              <div className="flex justify-between text-[10px] text-gray-600 mb-1 font-mono">
                <span>{currentState.toLowerCase().replace(/_/g, ' ')}</span>
                <span>{status.progress}%</span>
              </div>
              <div className="h-1 bg-surface-3 rounded-full overflow-hidden">
                <div
                  className={`h-full rounded-full transition-all duration-500 ${
                    isFailed ? 'bg-danger' : isComplete ? 'bg-online' : 'bg-accent'
                  }`}
                  style={{ width: `${status.progress}%` }}
                />
              </div>
            </div>
          )}
        </div>

        {/* ── Log pane ── */}
        <div className="flex-1 p-4 flex flex-col min-h-0 overflow-hidden">
          <div className="flex items-center justify-between mb-2 flex-shrink-0">
            <p className="data-label">Log Output</p>
            <button
              onClick={() => {
                const text = (status?.messages ?? []).join('\n')
                navigator.clipboard.writeText(text).catch(() => {})
              }}
              className="text-[10px] text-gray-600 hover:text-gray-300 px-2 py-0.5 rounded border border-surface-3 hover:border-gray-600 transition-colors"
            >
              Copy log
            </button>
          </div>
          <LogPane messages={status?.messages ?? []} />
        </div>

        {/* ── Footer ── */}
        <div className="px-6 py-4 border-t border-surface-3 flex-shrink-0">

          {/* Not started: configuration */}
          {!started && (
            <div className="space-y-3">
              {/* Motor profile selector */}
              <div className="space-y-1.5">
                <label className="data-label">Motor Profile</label>
                <select
                  value={motorProfile}
                  onChange={(e) => setMotorProfile(e.target.value)}
                  className="w-full bg-surface-2 border border-surface-3 rounded px-3 py-1.5 text-sm font-mono focus:outline-none focus:border-accent"
                >
                  {profiles.map((p) => (
                    <option key={p.key} value={p.key} disabled={!p.available}>
                      {p.key}{!p.available ? ' (incomplete)' : ''}
                    </option>
                  ))}
                  {profiles.length === 0 && (
                    <option value="MAD_5010_200KV">MAD_5010_200KV</option>
                  )}
                </select>
                <ProfileTable profile={selectedProfile} />
              </div>

              {/* Phase + programmer info + start */}
              <div className="flex gap-4 items-center flex-wrap">
                <label className="flex items-center gap-2 text-sm text-gray-400 cursor-pointer select-none">
                  <input
                    type="checkbox"
                    checked={invertPhase}
                    onChange={(e) => setInvertPhase(e.target.checked)}
                    className="accent-accent"
                  />
                  Invert Phase
                </label>
                <span className="text-xs text-gray-600 font-mono px-2 py-1 rounded bg-surface-2 border border-surface-3">
                  ST-LINK · SWD
                </span>
                <button onClick={handleStart} className="btn-primary ml-auto px-5">
                  Start Flash
                </button>
              </div>

              {/* Error display */}
              {startError && (
                startError.includes('apt install') ? (
                  <div className="rounded-lg bg-warn/10 border border-warn/20 px-4 py-3 space-y-1">
                    <p className="text-xs text-warn font-medium">Missing tools — run in terminal:</p>
                    <code className="block text-xs font-mono text-warn/80 select-all">
                      sudo apt install openocd gcc-arm-none-eabi
                    </code>
                  </div>
                ) : startError.includes('already in progress') ? (
                  <div className="flex items-center gap-3 rounded-lg bg-danger/10 border border-danger/20 px-4 py-2.5">
                    <p className="text-xs text-danger flex-1">Flash session already in progress.</p>
                    <button onClick={handleReset} className="btn-ghost text-xs px-3 py-1">
                      Reset session
                    </button>
                  </div>
                ) : (
                  <p className="text-xs text-danger">{startError}</p>
                )
              )}

              {/* 3-pass explanation */}
              <p className="text-[10px] text-gray-600">
                3-pass procedure: (1) init Flash option bytes → power cycle → (2) write CAN ID + calibrate → (3) operational firmware.
                Estimated time: ~3 min not counting compile time.
              </p>
            </div>
          )}

          {/* Awaiting power cycle */}
          {awaitingPowerCycle && (
            <PowerCycleWaiting onManualConfirm={handlePowerCycled} />
          )}

          {/* Awaiting CAN connect */}
          {awaitingCanConnect && (
            <div className="space-y-3">
              <div>
                <p className="text-sm font-medium">Connect motor, CAN bus, and encoder</p>
                <ol className="mt-2 space-y-1 text-xs text-gray-400 list-decimal list-inside">
                  <li>Connect the CAN bus cable to the ESC</li>
                  <li>Connect the motor phase wires to the ESC</li>
                  <li>Connect the encoder cable to the ESC</li>
                  <li>Power on the ESC</li>
                </ol>
              </div>
              <button onClick={handleCanConnected} className="btn-primary w-full">
                Motor connected — Start Calibration
              </button>
            </div>
          )}

          {/* Calibrating */}
          {isCalibrating && calStartedAt && (
            <CalibrationProgress startedAt={calStartedAt} />
          )}

          {/* Awaiting direction confirmation */}
          {awaitingConfirm && (
            <div className="flex items-center gap-3">
              <div className="flex-1">
                <p className="text-sm font-medium">Did the motor shaft move in the correct direction?</p>
                <p className="text-xs text-gray-500 mt-0.5">
                  Positive command = shaft moved as expected for joint convention.
                </p>
              </div>
              <button onClick={() => handleConfirm(false)} className="btn-danger px-4 py-2 whitespace-nowrap">
                No, invert ↻
              </button>
              <button onClick={() => handleConfirm(true)} className="btn-success px-4 py-2 whitespace-nowrap">
                Yes, correct ✓
              </button>
            </div>
          )}

          {/* Complete */}
          {isComplete && (
            <div className="flex items-center justify-between gap-4">
              <div>
                <p className="text-sm text-online font-medium">✓ ESC commissioned successfully</p>
                {status?.flux_offset != null && (
                  <p className="text-xs font-mono text-gray-400 mt-0.5">
                    flux_offset = {status.flux_offset.toFixed(4)} rad
                  </p>
                )}
                {configSynced && (
                  <p className="text-[10px] text-gray-500 mt-0.5">
                    Joint config updated in humanoid_lite.json
                  </p>
                )}
              </div>
              <button onClick={handleDone} className="btn-primary px-5 flex-shrink-0">
                Done
              </button>
            </div>
          )}

          {/* Failed */}
          {isFailed && (
            <div className="flex items-center justify-between gap-3">
              <p className="text-sm text-danger flex-1">
                {(status?.error ?? 'Flash failed — check log').split('\n')[0]}
              </p>
              <button onClick={onClose} className="btn-ghost px-4 py-2 flex-shrink-0">
                Close
              </button>
            </div>
          )}

          {/* Generic in-progress spinner */}
          {isActive && !awaitingPowerCycle && !awaitingCanConnect && !awaitingConfirm && !isCalibrating && (
            <div className="flex items-center gap-2">
              <span className="w-3.5 h-3.5 rounded-full border-2 border-accent border-t-transparent animate-spin" />
              <span className="text-sm text-gray-400">
                {currentState.toLowerCase().replace(/_/g, ' ')}…
              </span>
            </div>
          )}
        </div>
      </div>
    </div>
  )
}
