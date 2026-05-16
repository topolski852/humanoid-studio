import { useState } from 'react'
import { api } from '../api'

const DEG = Math.PI / 180

function SectionLabel({ children }) {
  return <p className="data-label mb-2">{children}</p>
}

export default function MotorCalibrationPanel({ jointName, state, config, onOpenFlash, onLogError }) {
  const isConnected = state != null

  const [calibrating, setCalibrating]   = useState(false)
  const [calibResult, setCalibResult]   = useState(null)

  const [limitsMinDeg, setLimitsMinDeg] = useState(
    () => config?.position_limits?.min != null
      ? String((config.position_limits.min / DEG).toFixed(1))
      : ''
  )
  const [limitsMaxDeg, setLimitsMaxDeg] = useState(
    () => config?.position_limits?.max != null
      ? String((config.position_limits.max / DEG).toFixed(1))
      : ''
  )
  const [calLower, setCalLower]         = useState(null)
  const [calUpper, setCalUpper]         = useState(null)
  const [calApplying, setCalApplying]   = useState(false)
  const [calResult, setCalResult]       = useState(null)

  async function runCalibration() {
    setCalibrating(true)
    setCalibResult(null)
    try {
      const result = await api.calibrateMotor(jointName)
      setCalibResult(`flux_offset = ${result?.flux_offset?.toFixed(4) ?? '?'} rad`)
    } catch (e) {
      const msg = `Error: ${e.message}`
      setCalibResult(msg)
      onLogError?.(msg, 'Cal')
    }
    setCalibrating(false)
  }

  function recordHardstop(which) {
    const pos = state?.position
    if (pos == null) return
    if (which === 'lower') setCalLower(pos)
    else setCalUpper(pos)
    setCalResult(null)
  }

  async function applyCalibration() {
    if (calLower == null) {
      setCalResult('Error: Record lower hardstop first')
      return
    }
    // Use entered limits if provided; fall back to raw recorded encoder values
    const minRad = limitsMinDeg !== '' ? parseFloat(limitsMinDeg) * DEG : calLower
    const maxRad = limitsMaxDeg !== '' ? parseFloat(limitsMaxDeg) * DEG : calUpper
    if (maxRad == null || isNaN(minRad) || isNaN(maxRad) || minRad >= maxRad) {
      setCalResult('Error: Record upper hardstop or enter valid min < max limits')
      return
    }
    if (calUpper != null && calUpper < calLower) {
      const directionMsg =
        'Direction inverted — encoder reads backward. ' +
        'Fix: negate gear_ratio in Tune tab (e.g. −15 → 15). ' +
        'If jogging also moves the wrong way, reflash with opposite phase.'
      setCalResult(directionMsg)
      onLogError?.(directionMsg, 'Cal')
      return
    }
    if (calUpper != null) {
      const measuredRange = calUpper - calLower
      const expectedRange = maxRad - minRad
      const rangeErrorDeg = Math.abs(measuredRange - expectedRange) / DEG
      if (rangeErrorDeg > 20) {
        const rangeMsg =
          `Error: measured range ${(measuredRange / DEG).toFixed(1)}° vs ` +
          `expected ${(expectedRange / DEG).toFixed(1)}° — check gear ratio or limits`
        setCalResult(rangeMsg)
        onLogError?.(rangeMsg, 'Cal')
        return
      }
    }
    setCalApplying(true)
    try {
      const result = await api.positionCalibrateMotor(jointName, calLower, minRad, maxRad)
      const minDegStr = (minRad / DEG).toFixed(1)
      const maxDegStr = (maxRad / DEG).toFixed(1)
      setCalResult(
        `Done — offset=${result?.position_offset?.toFixed(4) ?? '?'} rad, limits=[${minDegStr}°, ${maxDegStr}°]`
      )
      setCalLower(null)
      setCalUpper(null)
    } catch (e) {
      const msg = `Error: ${e.message}`
      setCalResult(msg)
      onLogError?.(msg, 'Cal')
    }
    setCalApplying(false)
  }

  const canApply =
    calLower != null &&
    !calApplying &&
    isConnected &&
    (
      // explicit limits entered
      (!isNaN(parseFloat(limitsMinDeg)) && !isNaN(parseFloat(limitsMaxDeg)) && parseFloat(limitsMinDeg) < parseFloat(limitsMaxDeg)) ||
      // or both hardstops recorded in correct order
      (calUpper != null && calLower < calUpper)
    )

  return (
    <div className="flex-1 p-6 overflow-y-auto">
      {!isConnected && (
        <div className="mb-5 px-3 py-2.5 rounded-lg bg-surface-2 border border-surface-3">
          <p className="text-xs text-gray-400">Motor visible via passive CAN.</p>
          <p className="text-[10px] text-gray-600 mt-0.5">
            Connect the robot (top-right) to run calibration.
          </p>
        </div>
      )}

      {/* ── Flux Calibration ── */}
      <div className="mb-8">
        <SectionLabel>CALIBRATION</SectionLabel>
        <p className="text-[10px] text-gray-500 mb-3">
          Runs flux-offset calibration. Motor must be free-spinning (no load). Takes up to 90 s.
        </p>
        <button
          onClick={runCalibration}
          disabled={calibrating || !isConnected}
          className="btn-ghost w-full py-2 mb-2 disabled:opacity-40"
        >
          {calibrating ? (
            <span className="flex items-center gap-2 justify-center">
              <span className="w-3.5 h-3.5 rounded-full border-2 border-accent border-t-transparent animate-spin" />
              Calibrating… (up to 90 s)
            </span>
          ) : (
            'Run Flux Calibration'
          )}
        </button>
        {calibResult && (
          <p className={`text-xs font-mono ${calibResult.startsWith('Error') ? 'text-danger' : 'text-online'}`}>
            {calibResult}
          </p>
        )}
      </div>

      {/* ── Position Calibration ── */}
      <div className="mb-6">
        <SectionLabel>POSITION CALIBRATION</SectionLabel>
        <p className="text-[10px] text-gray-500 mb-3">
          Enable Damping, push joint to each hardstop, record, then Apply.
          Enter the expected joint limits for this joint.
        </p>

        {/* Limit inputs */}
        <div className="flex gap-3 mb-3">
          <div className="flex-1">
            <p className="text-[10px] text-gray-500 mb-1">Lower limit (°)</p>
            <input
              type="number"
              value={limitsMinDeg}
              onChange={(e) => { setLimitsMinDeg(e.target.value); setCalResult(null) }}
              placeholder="-30"
              className="w-full font-mono text-xs px-2 py-1.5 rounded border border-surface-3
                bg-surface-2 text-gray-300 outline-none focus:border-accent/50 transition-colors"
            />
          </div>
          <div className="flex-1">
            <p className="text-[10px] text-gray-500 mb-1">Upper limit (°)</p>
            <input
              type="number"
              value={limitsMaxDeg}
              onChange={(e) => { setLimitsMaxDeg(e.target.value); setCalResult(null) }}
              placeholder="30"
              className="w-full font-mono text-xs px-2 py-1.5 rounded border border-surface-3
                bg-surface-2 text-gray-300 outline-none focus:border-accent/50 transition-colors"
            />
          </div>
        </div>

        <button
          onClick={() => api.enableMotor(jointName, 'IDLE').catch(() => {})}
          disabled={!isConnected}
          className="btn-ghost w-full py-2 mb-3 text-xs disabled:opacity-40"
        >
          Enable Idle
        </button>

        <div className="flex gap-2 mb-3">
          <button
            onClick={() => recordHardstop('lower')}
            disabled={!isConnected}
            className="flex-1 py-2 rounded-lg text-[10px] font-medium bg-surface-2 border border-surface-3
              hover:border-accent/40 hover:text-accent transition-colors disabled:opacity-40"
          >
            Record Lower<br />
            <span className="font-mono text-gray-400">
              {calLower != null
                ? `${(calLower / DEG).toFixed(1)}°`
                : limitsMinDeg !== '' ? `(target ${limitsMinDeg}°)` : '—'}
            </span>
          </button>
          <button
            onClick={() => recordHardstop('upper')}
            disabled={!isConnected}
            className="flex-1 py-2 rounded-lg text-[10px] font-medium bg-surface-2 border border-surface-3
              hover:border-accent/40 hover:text-accent transition-colors disabled:opacity-40"
          >
            Record Upper<br />
            <span className="font-mono text-gray-400">
              {calUpper != null
                ? `${(calUpper / DEG).toFixed(1)}°`
                : limitsMaxDeg !== '' ? `(target ${limitsMaxDeg}°)` : '—'}
            </span>
          </button>
        </div>

        {calLower != null && calUpper != null && (
          <p className={`text-[10px] mb-2 ${calUpper > calLower ? 'text-online' : 'text-danger'}`}>
            {calUpper > calLower
              ? `Direction OK — measured ${((calUpper - calLower) / DEG).toFixed(1)}°` +
                (!isNaN(parseFloat(limitsMinDeg)) && !isNaN(parseFloat(limitsMaxDeg))
                  ? ` / expected ${(parseFloat(limitsMaxDeg) - parseFloat(limitsMinDeg)).toFixed(1)}°`
                  : '')
              : 'Direction inverted — encoder reads backward (see Apply for fix)'}
          </p>
        )}

        <button
          onClick={applyCalibration}
          disabled={!canApply}
          className="btn-ghost w-full py-2 mb-2 text-xs disabled:opacity-40"
        >
          {calApplying ? 'Applying…' : 'Apply Calibration'}
        </button>
        {calResult && (
          <p className={`text-xs font-mono ${
            calResult.startsWith('Error') || calResult.startsWith('Direction')
              ? 'text-danger'
              : 'text-online'
          }`}>
            {calResult}
          </p>
        )}
      </div>

      {/* ── Firmware ── */}
      <div className="mb-6">
        <SectionLabel>FIRMWARE</SectionLabel>
        <button onClick={onOpenFlash} className="btn-ghost w-full py-2">
          Flash Wizard
        </button>
      </div>
    </div>
  )
}
