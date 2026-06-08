import { useEffect, useRef, useState } from 'react'
import { api } from '../api'

// Steps matching FlashState.step_index (0-5)
const STEP_LABELS = [
  'Configure',    // 0 — pre-start
  'Flash',        // 1 — FLASHING
  'Connect',      // 2 — WAITING_CAN_CONNECT
  'Commission',   // 3 — COMMISSIONING
  'Calibrate',    // 4 — CALIBRATING / AWAITING_CONFIRMATION
  'Done',         // 5 — COMPLETE
]

const ACTIVE_STATES = new Set([
  'FLASHING', 'COMMISSIONING',
  'WAITING_CAN_CONNECT', 'CALIBRATING', 'AWAITING_CONFIRMATION',
])

// States where closing would leave the ESC in a broken intermediate state
const LOCKED_STATES = new Set([
  'FLASHING', 'COMMISSIONING', 'CALIBRATING',
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


// ── Calibration progress ───────────────────────────────────────────────────────
function CalibrationProgress({ startedAt }) {
  const [elapsed, setElapsed] = useState(0)
  useEffect(() => {
    const t = setInterval(() => setElapsed(Math.floor((Date.now() - startedAt) / 1000)), 500)
    return () => clearInterval(t)
  }, [startedAt])
  // Estimated total time matches the 90s Python timeout; cap visual at 95% so
  // the bar never falsely shows "complete" while the firmware is still sweeping.
  const est = 90
  const pct = Math.min(95, Math.round((elapsed / est) * 100))
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
        <p className="text-[10px] text-gray-600 mt-0.5 font-mono">{elapsed}s elapsed (est. ~15–90s depending on motor)</p>
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
export default function FlashWizard({ canId, canChannel, jointName, onClose, commissionOnly = false }) {
  const [status, setStatus]           = useState(null)
  const [stepInfo, setStepInfo]       = useState({ step_index: 0, total_steps: 6 })
  const [profiles, setProfiles]       = useState([])
  const [motorProfile, setMotorProfile] = useState('MAD_5010_200KV')
  const [invertPhase, setInvertPhase] = useState(false)
  const [started, setStarted]         = useState(false)
  const [startError, setStartError]   = useState(null)
  const [calStartedAt, setCalStartedAt] = useState(null)
  const [configSynced, setConfigSynced] = useState(false)
  const [pingState, setPingState] = useState(null)   // null | 'pending' | {reachable, detail}
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

  // Reset ping result whenever we leave the WAITING_CAN_CONNECT step
  useEffect(() => {
    if (status?.state !== 'WAITING_CAN_CONNECT') {
      setPingState(null)
    }
  }, [status?.state])

  async function handleStart() {
    setStartError(null)
    setStarted(true)
    try {
      await api.flashStart(canId, invertPhase, motorProfile, 'SWD', canChannel ?? 'can0', commissionOnly)
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

  async function handleCanConnected() {
    try { await api.flashCanConnected() } catch (e) { console.error(e) }
  }

  async function handleCanPing() {
    setPingState('pending')
    try {
      const result = await api.flashCanPing()
      setPingState(result)
    } catch (e) {
      setPingState({ reachable: false, detail: e.message })
    }
  }

  async function handleDone() {
    // Stop polling before flashReset so the IDLE state update never reaches the UI.
    clearInterval(pollRef.current)
    await api.flashReset().catch(() => {})
    onClose()

    // After closing: restart daemon (it was stopped for SWD), apply config (firmware
    // RAM), then persist this joint's params to ESC flash. Non-fatal if any step fails.
    if (jointName) {
      Promise.resolve(window.electron?.ensureDaemon?.())
        .then(() => api.connectRobot())
        .then(() => api.storeMotorToFlash(jointName))
        .catch(() => {})
    }
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
            <h2 className="font-semibold text-base">
              {commissionOnly ? 'ESC Commission Wizard' : 'ESC Flash Wizard'}
            </h2>
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
                    <option key={p.key} value={p.key}>
                      {p.key}
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
                {!commissionOnly && (
                  <span className="text-xs text-gray-600 font-mono px-2 py-1 rounded bg-surface-2 border border-surface-3">
                    ST-LINK · SWD
                  </span>
                )}
                <button onClick={handleStart} className="btn-primary ml-auto px-5">
                  {commissionOnly ? 'Commission Motor' : 'Flash + Commission Motor'}
                </button>
              </div>

              {/* Error display */}
              {startError && (
                startError.includes('apt install') ? (
                  <div className="rounded-lg bg-warn/10 border border-warn/20 px-4 py-3 space-y-1">
                    <p className="text-xs text-warn font-medium">Missing tools — run in terminal:</p>
                    <code className="block text-xs font-mono text-warn/80 select-all">
                      sudo apt install openocd
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

              <p className="text-[10px] text-gray-600">
                {commissionOnly
                  ? 'Commissions the ESC over CAN (sets ID + gains), then runs encoder calibration. Firmware must already be on the ESC at commissioning ID 127. Estimated time: ~1 min.'
                  : 'Flashes pre-compiled firmware via ST-LINK, commissions the ESC over CAN (sets ID + gains), then runs encoder calibration. No compile step — estimated time: ~2 min.'}
              </p>
            </div>
          )}

          {/* Awaiting CAN connect */}
          {awaitingCanConnect && (
            <div className="space-y-3">
              <div>
                {commissionOnly ? (
                  <>
                    <p className="text-sm font-medium">Power cycle, then connect hardware</p>
                    <ol className="mt-2 space-y-1 text-xs text-gray-400 list-decimal list-inside">
                      <li>Power cycle the ESC — disconnect then reconnect its power supply</li>
                      <li>Connect the CAN bus cable to the ESC ({canChannel})</li>
                      <li>Connect the motor phase wires to the ESC</li>
                      <li>Connect the encoder cable to the ESC</li>
                    </ol>
                    <p className="mt-2 text-[11px] text-gray-500">
                      Power cycling resets the CAN controller to a clean state.
                      The ESC must have commissioning firmware and will boot at ID 127.
                      Commissioning (setting CAN ID {canId}) runs automatically after you click below.
                    </p>
                  </>
                ) : (
                  <>
                    <p className="text-sm font-medium">Power cycle, then connect hardware</p>
                    <ol className="mt-2 space-y-1 text-xs text-gray-400 list-decimal list-inside">
                      <li>Power cycle the ESC — disconnect then reconnect its power supply</li>
                      <li>Connect the CAN bus cable to the ESC ({canChannel})</li>
                      <li>Connect the motor phase wires to the ESC</li>
                      <li>Connect the encoder cable to the ESC</li>
                    </ol>
                    <p className="mt-2 text-[11px] text-gray-500">
                      A power cycle is required after SWD flash so the CAN peripheral initialises
                      from a clean hardware reset. Commissioning (setting CAN ID {canId}) runs
                      automatically after you click below.
                    </p>
                  </>
                )}
              </div>

              {/* CAN connectivity test — pings at commissioning ID 127 */}
              <div className="flex items-center gap-3 flex-wrap">
                <button
                  onClick={handleCanPing}
                  disabled={pingState === 'pending'}
                  className="btn-ghost px-3 py-1.5 text-xs flex-shrink-0 disabled:opacity-50"
                >
                  {pingState === 'pending' ? (
                    <span className="flex items-center gap-1.5">
                      <span className="w-3 h-3 rounded-full border-2 border-accent border-t-transparent animate-spin inline-block" />
                      Testing…
                    </span>
                  ) : 'Test CAN'}
                </button>
                {pingState && pingState !== 'pending' && (
                  <span className={`text-xs font-mono truncate ${pingState.reachable ? 'text-online' : 'text-danger'}`}>
                    {pingState.reachable ? '✓ ' : '✗ '}{pingState.detail}
                  </span>
                )}
              </div>
              {pingState && pingState !== 'pending' && !pingState.reachable && (
                <p className="text-xs text-warn">
                  ESC not responding. Check: {commissionOnly ? 'ESC was power-cycled and has commissioning firmware (boots at ID 127)' : 'ESC was power-cycled after flashing'}, CAN cable is connected to {canChannel}, 120 Ω termination on both ends of the CAN bus.
                </p>
              )}

              <button onClick={handleCanConnected} className="btn-primary w-full">
                Hardware connected — Start Commissioning
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
              <button
                onClick={handleReset}
                className="btn-ghost px-4 py-2 flex-shrink-0 text-warn border-warn/30 hover:border-warn/60"
              >
                ↻ Reset
              </button>
              <button onClick={onClose} className="btn-ghost px-4 py-2 flex-shrink-0">
                Close
              </button>
            </div>
          )}

          {/* Generic in-progress spinner */}
          {isActive && !awaitingCanConnect && !awaitingConfirm && !isCalibrating && (
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
