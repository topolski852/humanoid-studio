import { useState, useEffect, useRef } from 'react'
import { api } from '../api'

const DEG = Math.PI / 180
const d = (rad) => (rad == null ? '—' : `${(rad / DEG).toFixed(1)}°`)

// Hardstop range calibration step: zero the offset, capture both hardstops, then
// the backend derives gear_ratio sign (backwards-encoder detection) + offset +
// limits. A final verification jog confirms the joint moves the correct way.
export default function CalibrateStep({ jointName, onComplete, onLogError }) {
  const [phase, setPhase]       = useState('idle')   // idle | capturing | applied | verified
  const [livePos, setLivePos]   = useState(null)
  const [lower, setLower]       = useState(null)
  const [upper, setUpper]       = useState(null)
  const [busy, setBusy]         = useState(false)
  const [error, setError]       = useState(null)
  const [applyResult, setApply] = useState(null)
  const [verifyRes, setVerify]  = useState(null)
  const [storeFlash, setStore]  = useState(false)
  const [log, setLog]           = useState([])
  const pollRef = useRef(null)

  const addLog = (msg) => setLog((l) => [...l, msg])

  // Poll live position while capturing hardstops.
  useEffect(() => {
    if (phase !== 'capturing') return
    let cancelled = false
    async function poll() {
      try {
        const m = await api.getMotor(jointName)
        if (!cancelled) setLivePos(m?.state?.position ?? null)
      } catch { /* offline / transient */ }
    }
    poll()
    pollRef.current = setInterval(poll, 250)
    return () => { cancelled = true; clearInterval(pollRef.current) }
  }, [phase, jointName])

  async function run(fn, after) {
    setBusy(true); setError(null)
    try { const r = await fn(); after?.(r) }
    catch (e) { setError(e.message); addLog(`Error: ${e.message}`); onLogError?.(e.message, 'Cal') }
    setBusy(false)
  }

  const handleStart = () => run(
    () => api.rangeCalStart(jointName),
    () => {
      setLower(null); setUpper(null); setApply(null); setVerify(null); setPhase('capturing')
      addLog('Offset zeroed on ESC, motor idled. Move the joint to each hardstop and record it.')
    },
  )
  const recordStop = (which) => {
    if (livePos == null) return
    which === 'lower' ? setLower(livePos) : setUpper(livePos)
    addLog(`${which === 'lower' ? 'Lower' : 'Upper'} hardstop recorded at ${d(livePos)}.`)
  }
  const handleApply = () => run(
    () => { addLog('Computing gear sign, offset, and limits from the two hardstops...'); return api.rangeCalApply(jointName, lower, upper, { storeToFlash: storeFlash }) },
    (r) => {
      setApply(r); setPhase('applied')
      addLog(`Applied: gear_ratio ${r.gear_ratio}${r.flipped ? ' (encoder backward — auto-flipped)' : ''}, ` +
             `offset ${r.position_offset?.toFixed(4)} rad, limits ${d(r.limits?.min)}…${d(r.limits?.max)}, ` +
             `measured span ${(r.measured_range_rad / DEG).toFixed(1)}° ${r.range_ok ? '(OK)' : '(range mismatch!)'}.`)
    },
  )
  const handleVerify = () => run(
    () => { addLog('Verifying direction — jogging toward +...'); return api.jogDirection(jointName, 0.25, 1.5) },
    (r) => {
      setVerify(r); setPhase('verified')
      addLog(`Jog moved ${(r.signed_motion_rad / DEG).toFixed(1)}° ` +
             `(${r.signed_motion_rad >= 0 ? 'toward upper/+' : 'toward lower/−'}). Confirm direction visually.`)
    },
  )

  const backwards = lower != null && upper != null && upper < lower
  const measuredDeg = lower != null && upper != null ? Math.abs(upper - lower) / DEG : null

  return (
    <div className="space-y-3">
      <div>
        <p className="text-sm font-medium">Hardstop range calibration</p>
        <p className="text-xs text-gray-500 mt-0.5">
          Zeroes the offset, then you hand-move the joint to each hardstop. The two
          stops set the zero, the limits, and the direction (auto-flips a backwards encoder).
        </p>
      </div>

      {phase === 'idle' && (
        <button onClick={handleStart} disabled={busy} className="btn-primary w-full disabled:opacity-50">
          {busy ? 'Starting…' : 'Start Calibration (zero offset + idle)'}
        </button>
      )}

      {(phase === 'capturing' || phase === 'applied' || phase === 'verified') && (
        <>
          {/* Live position */}
          <div className="flex items-center justify-between px-3 py-2 rounded-lg bg-surface-2 border border-surface-3">
            <span className="text-[10px] text-gray-500">LIVE POSITION (raw)</span>
            <span className="font-mono text-sm text-gray-200">{d(livePos)}</span>
          </div>

          {/* Capture buttons */}
          <div className="flex gap-2">
            <button
              onClick={() => recordStop('lower')}
              disabled={busy || phase !== 'capturing'}
              className="flex-1 py-2 rounded-lg text-[11px] font-medium bg-surface-2 border border-surface-3
                hover:border-accent/40 hover:text-accent transition-colors disabled:opacity-40"
            >
              Record Lower hardstop<br /><span className="font-mono text-gray-400">{d(lower)}</span>
            </button>
            <button
              onClick={() => recordStop('upper')}
              disabled={busy || phase !== 'capturing'}
              className="flex-1 py-2 rounded-lg text-[11px] font-medium bg-surface-2 border border-surface-3
                hover:border-accent/40 hover:text-accent transition-colors disabled:opacity-40"
            >
              Record Upper hardstop<br /><span className="font-mono text-gray-400">{d(upper)}</span>
            </button>
          </div>

          {lower != null && upper != null && (
            <p className={`text-[11px] ${backwards ? 'text-warn' : 'text-online'}`}>
              {backwards
                ? `Encoder reads backward — gear_ratio will auto-flip. Measured span ${measuredDeg.toFixed(1)}°.`
                : `Direction forward. Measured span ${measuredDeg.toFixed(1)}°.`}
            </p>
          )}

          {phase === 'capturing' && (
            <button
              onClick={handleApply}
              disabled={busy || lower == null || upper == null}
              className="btn-ghost w-full py-2 disabled:opacity-40"
            >
              {busy ? 'Applying…' : 'Apply — compute gear / offset / limits'}
            </button>
          )}
        </>
      )}

      {/* Apply result */}
      {applyResult && (
        <div className="rounded-lg bg-surface-2 border border-surface-3 px-3 py-2.5 space-y-1 text-[11px] font-mono">
          <div className="flex justify-between"><span className="text-gray-500">gear_ratio</span>
            <span className={applyResult.flipped ? 'text-warn' : 'text-gray-200'}>
              {applyResult.gear_ratio}{applyResult.flipped ? '  (auto-flipped ↻)' : ''}
            </span></div>
          <div className="flex justify-between"><span className="text-gray-500">position_offset</span>
            <span className="text-gray-200">{applyResult.position_offset?.toFixed(4)} rad</span></div>
          <div className="flex justify-between"><span className="text-gray-500">limits</span>
            <span className="text-gray-200">{d(applyResult.limits?.min)} … {d(applyResult.limits?.max)}</span></div>
          <div className="flex justify-between"><span className="text-gray-500">measured range</span>
            <span className={applyResult.range_ok ? 'text-online' : 'text-danger'}>
              {(applyResult.measured_range_rad / DEG).toFixed(1)}° {applyResult.range_ok ? '✓' : '(range mismatch)'}
            </span></div>
        </div>
      )}

      {/* Verify */}
      {(phase === 'applied' || phase === 'verified') && (
        <button onClick={handleVerify} disabled={busy} className="btn-ghost w-full py-2 disabled:opacity-40">
          {busy ? 'Jogging…' : 'Verify direction (jog toward +)'}
        </button>
      )}
      {verifyRes && (
        <p className="text-[11px] text-gray-400">
          Jog moved <span className="font-mono">{(verifyRes.signed_motion_rad / DEG).toFixed(1)}°</span>
          {verifyRes.signed_motion_rad >= 0
            ? ' — position increased (toward upper / +). Confirm it physically moved the right way.'
            : ' — position decreased (toward lower / −, no room up). Confirm direction visually.'}
        </p>
      )}

      {error && <p className="text-xs text-danger font-mono">{error}</p>}

      {/* Finish */}
      {phase === 'verified' && (
        <div className="flex items-center justify-between gap-3 pt-1">
          <label className="flex items-center gap-2 text-xs text-gray-400 cursor-pointer select-none">
            <input type="checkbox" checked={storeFlash} onChange={(e) => setStore(e.target.checked)} className="accent-accent" />
            Store to ESC flash (persist across power-cycle)
          </label>
          <button
            onClick={() => run(
              async () => { if (storeFlash) await api.storeMotorToFlash(jointName) },
              () => onComplete?.(),
            )}
            disabled={busy}
            className="btn-primary px-5 flex-shrink-0 disabled:opacity-50"
          >
            {busy ? 'Saving…' : 'Done'}
          </button>
        </div>
      )}

      {/* Activity log */}
      {log.length > 0 && (
        <div className="mt-1">
          <p className="data-label mb-1">Calibration Log</p>
          <div className="bg-surface rounded-lg border border-surface-3 p-2.5 font-mono text-[11px] leading-relaxed max-h-40 overflow-y-auto">
            {log.map((m, i) => (
              <div key={i} className={`whitespace-pre-wrap break-words ${
                m.startsWith('Error') ? 'text-danger'
                  : m.startsWith('Applied') || m.startsWith('Jog') ? 'text-online'
                  : 'text-gray-400'}`}>
                <span className="text-gray-600 mr-2 select-none">&gt;</span>{m}
              </div>
            ))}
          </div>
        </div>
      )}
    </div>
  )
}
