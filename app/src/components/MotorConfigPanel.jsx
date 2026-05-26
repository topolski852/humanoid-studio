import { useState, useMemo } from 'react'
import { api } from '../api'

// Parameter metadata: key → { label, unit, group, readOnly }
const PARAMS = [
  // Position Controller
  { key: 'gear_ratio',              label: 'Gear Ratio',              unit: '',      group: 'Position Controller' },
  { key: 'position_kp',             label: 'Position Kp',             unit: '',      group: 'Position Controller' },
  { key: 'position_ki',             label: 'Position Ki',             unit: '',      group: 'Position Controller' },
  { key: 'velocity_kp',             label: 'Velocity Kp',             unit: '',      group: 'Position Controller' },
  { key: 'velocity_ki',             label: 'Velocity Ki',             unit: '',      group: 'Position Controller' },
  { key: 'torque_limit',            label: 'Torque Limit',            unit: 'Nm',    group: 'Position Controller' },
  { key: 'velocity_limit',          label: 'Velocity Limit',          unit: 'rad/s', group: 'Position Controller' },
  { key: 'position_limit_lower',    label: 'Pos Limit Lower',         unit: 'rad',   group: 'Position Controller' },
  { key: 'position_limit_upper',    label: 'Pos Limit Upper',         unit: 'rad',   group: 'Position Controller' },
  { key: 'position_offset',         label: 'Position Offset',         unit: 'rad',   group: 'Position Controller' },
  { key: 'torque_filter_alpha',     label: 'Torque Filter α',         unit: '',      group: 'Position Controller' },
  // Current Controller
  { key: 'current_limit',           label: 'Current Limit',           unit: 'A',     group: 'Current Controller' },
  { key: 'current_kp',              label: 'Current Kp',              unit: 'V/A',   group: 'Current Controller' },
  { key: 'current_ki',              label: 'Current Ki',              unit: '1/s',   group: 'Current Controller' },
  // Powerstage
  { key: 'undervoltage_threshold',  label: 'Undervoltage',            unit: 'V',     group: 'Powerstage' },
  { key: 'overvoltage_threshold',   label: 'Overvoltage',             unit: 'V',     group: 'Powerstage' },
  { key: 'bus_voltage_filter_alpha',label: 'Bus Voltage Filter α',    unit: '',      group: 'Powerstage' },
  // Motor
  { key: 'pole_pairs',              label: 'Pole Pairs',              unit: '',      group: 'Motor' },
  { key: 'torque_constant',         label: 'Torque Constant',         unit: 'Nm/A',  group: 'Motor' },
  { key: 'phase_order',             label: 'Phase Order',             unit: '',      group: 'Motor' },
  { key: 'max_calibration_current', label: 'Max Cal Current',         unit: 'A',     group: 'Motor' },
  // Encoder
  { key: 'cpr',                     label: 'CPR',                     unit: '',      group: 'Encoder' },
  { key: 'encoder_position_offset', label: 'Encoder Pos Offset',      unit: 'rad',   group: 'Encoder' },
  { key: 'velocity_filter_alpha',   label: 'Velocity Filter α',       unit: '',      group: 'Encoder' },
  { key: 'electrical_offset',       label: 'Flux Offset',             unit: 'rad',   group: 'Encoder' },
  // System
  { key: 'device_id',               label: 'Device ID',               unit: '',      group: 'System', readOnly: true },
  { key: 'firmware_version',        label: 'Firmware Version',        unit: '',      group: 'System', readOnly: true },
  { key: 'watchdog_timeout',        label: 'Watchdog Timeout',        unit: 'ms',    group: 'System' },
  { key: 'fast_frame_frequency',    label: 'PDO4 Frequency',          unit: 'Hz',    group: 'System' },
]

const GROUPS = ['Position Controller', 'Current Controller', 'Powerstage', 'Motor', 'Encoder', 'System']

const PID_KEYS    = ['position_kp', 'position_ki', 'velocity_kp', 'velocity_ki', 'current_kp', 'current_ki', 'current_limit']
const CAL_KEYS    = ['electrical_offset', 'encoder_position_offset', 'position_offset']
const DESIGN_KEYS = ['gear_ratio', 'position_kp', 'position_ki', 'velocity_kp', 'velocity_ki',
                     'torque_limit', 'velocity_limit', 'position_limit_lower', 'position_limit_upper',
                     'torque_filter_alpha', 'current_limit', 'current_kp', 'current_ki',
                     'fast_frame_frequency', 'watchdog_timeout']

function fmtVal(v) {
  if (v == null) return '—'
  if (typeof v === 'number') return Number.isInteger(v) ? String(v) : v.toFixed(6).replace(/\.?0+$/, '')
  return String(v)
}

function differs(a, b) {
  if (a == null || b == null) return false
  if (typeof a === 'number' && typeof b === 'number') return Math.abs(a - b) > 1e-6
  return String(a) !== String(b)
}

export default function MotorConfigPanel({ jointName, onLogError }) {
  const [appConfig,    setAppConfig]    = useState(null)
  const [deviceConfig, setDeviceConfig] = useState(null)
  const [selections,   setSelections]   = useState({})  // { key: 'app' | 'esc' | 'manual' }
  const [manualValues, setManualValues] = useState({})  // { key: string } raw user input
  const [pulling,      setPulling]      = useState(false)
  const [applying,     setApplying]     = useState(false)
  const [statusMsg,    setStatusMsg]    = useState(null)

  const mergedConfig = useMemo(() => {
    if (!appConfig || !deviceConfig) return null
    const out = {}
    for (const key of Object.keys(deviceConfig)) {
      const sel = selections[key] ?? 'app'
      if (sel === 'esc') {
        out[key] = deviceConfig[key]
      } else if (sel === 'manual') {
        const raw = manualValues[key] ?? ''
        const num = parseFloat(raw)
        out[key] = isNaN(num) ? (appConfig[key] ?? deviceConfig[key]) : num
      } else {
        out[key] = appConfig[key] ?? deviceConfig[key]
      }
    }
    return out
  }, [appConfig, deviceConfig, selections, manualValues])

  async function pullBoth(silent = false) {
    setPulling(true)
    if (!silent) setStatusMsg(null)
    try {
      const [robotCfg, escCfg] = await Promise.all([
        api.getRobotConfig(),
        api.getMotorConfigFromDevice(jointName),
      ])
      const jointCfg = robotCfg?.joints?.[jointName] ?? {}
      setAppConfig(jointCfg)
      setDeviceConfig(escCfg)
      // default all to 'app'; init manual values from app (fallback to ESC)
      const sels = {}
      const manual = {}
      for (const key of Object.keys(escCfg)) {
        sels[key] = 'app'
        const v = jointCfg[key] ?? escCfg[key]
        manual[key] = v == null ? '' : String(v)
      }
      setSelections(sels)
      setManualValues(manual)
      if (!silent) setStatusMsg({ type: 'ok', text: 'Pulled from ESC and app config.' })
    } catch (e) {
      const msg = `Pull failed: ${e.message}`
      setStatusMsg({ type: 'err', text: msg })
      onLogError?.(msg, 'Tune')
    }
    setPulling(false)
  }

  function setGroup(keys, source) {
    setSelections((s) => {
      const next = { ...s }
      for (const k of keys) if (k in next) next[k] = source
      return next
    })
  }

  function setAll(source) {
    setSelections((s) => Object.fromEntries(Object.keys(s).map((k) => [k, source])))
  }

  function toggleParam(key, source) {
    setSelections((s) => ({ ...s, [key]: source }))
  }

  function setManual(key, value) {
    setManualValues((m) => ({ ...m, [key]: value }))
  }

  async function apply() {
    if (!mergedConfig) return
    setApplying(true)
    setStatusMsg(null)
    try {
      await api.applyMotorConfig(jointName, mergedConfig)
      setStatusMsg({ type: 'ok', text: 'Applied — pulling updated values…' })
      await pullBoth(true)
      setStatusMsg({ type: 'ok', text: 'Applied to ESC RAM and saved config.' })
    } catch (e) {
      const msg = `Apply failed: ${e.message}`
      setStatusMsg({ type: 'err', text: msg })
      onLogError?.(msg, 'Tune')
    }
    setApplying(false)
  }

  async function storeToFlash() {
    if (!mergedConfig) return
    setApplying(true)
    setStatusMsg(null)
    try {
      // 1. Write merged config to ESC RAM and save to app JSON config.
      await api.applyMotorConfig(jointName, mergedConfig)
      setStatusMsg({ type: 'ok', text: 'Applied to ESC RAM — writing flash…' })
      // 2. Persist ESC RAM → flash.
      await api.storeMotorToFlash(jointName)
      setStatusMsg({ type: 'ok', text: 'Flash written — pulling to confirm…' })
      // 3. Auto-pull to confirm all three columns now match.
      await pullBoth(true)
      setStatusMsg({ type: 'ok', text: 'Stored to flash. ESC, App, and Manual all in sync.' })
    } catch (e) {
      const msg = `Store failed: ${e.message}`
      setStatusMsg({ type: 'err', text: msg })
      onLogError?.(msg, 'Tune')
    }
    setApplying(false)
  }

  const hasDiff = deviceConfig && appConfig && PARAMS.some(
    (p) => !p.readOnly && differs(appConfig[p.key], deviceConfig?.[p.key])
  )

  return (
    <div className="flex-1 flex flex-col overflow-hidden">
      {/* Toolbar */}
      <div className="flex-shrink-0 px-3 py-2 border-b border-surface-3 space-y-2">
        <button
          onClick={pullBoth}
          disabled={pulling}
          className="btn-ghost w-full py-1.5 text-xs"
        >
          {pulling ? 'Pulling…' : 'Pull from ESC'}
        </button>

        {deviceConfig && (
          <>
            {/* Group quick-selects */}
            <div className="flex flex-wrap gap-1">
              {[
                { label: 'All → App',    fn: () => setAll('app') },
                { label: 'All → ESC',    fn: () => setAll('esc') },
                { label: 'PID → App',    fn: () => setGroup(PID_KEYS, 'app') },
                { label: 'Cal → ESC',    fn: () => setGroup(CAL_KEYS, 'esc') },
                { label: 'Design → App', fn: () => setGroup(DESIGN_KEYS, 'app') },
              ].map(({ label, fn }) => (
                <button
                  key={label}
                  onClick={fn}
                  className="px-2 py-0.5 rounded text-[10px] bg-surface-2 border border-surface-3
                    hover:border-accent/40 hover:text-accent transition-colors"
                >
                  {label}
                </button>
              ))}
            </div>

            <button
              onClick={apply}
              disabled={applying}
              className="w-full py-1.5 rounded-lg text-xs font-medium bg-accent/20 text-accent
                border border-accent/30 hover:bg-accent/30 disabled:opacity-40 transition-colors"
            >
              {applying ? 'Applying…' : 'Apply'}
            </button>
            <button
              onClick={storeToFlash}
              disabled={applying}
              className="w-full py-1.5 rounded-lg text-xs font-medium bg-surface-2 text-gray-300
                border border-surface-3 hover:border-accent/40 hover:text-accent disabled:opacity-40 transition-colors"
            >
              Store to Flash
            </button>
          </>
        )}

        {statusMsg && (
          <p className={`text-[10px] font-mono ${statusMsg.type === 'err' ? 'text-danger' : 'text-online'}`}>
            {statusMsg.text}
          </p>
        )}
      </div>

      {/* Parameter table */}
      {!deviceConfig ? (
        <div className="flex-1 flex items-center justify-center">
          <p className="text-xs text-gray-600">Press "Pull from ESC" to load values</p>
        </div>
      ) : (
        <div className="flex-1 overflow-y-auto">
          {/* Column header */}
          <div className="sticky top-0 flex items-center px-2 py-1 bg-surface border-b border-surface-3 text-[9px] text-gray-600 font-medium uppercase tracking-wider z-10">
            <span className="w-5 text-center">A</span>
            <span className="w-5 text-center">E</span>
            <span className="w-5 text-center mr-1">M</span>
            <span className="flex-1 pl-1">Parameter</span>
            <span className="w-20 text-right pr-1">App</span>
            <span className="w-20 text-right pr-1">ESC</span>
            <span className="w-24 text-right">Manual</span>
          </div>

          {GROUPS.map((group) => {
            const groupParams = PARAMS.filter((p) => p.group === group && p.key in deviceConfig)
            if (groupParams.length === 0) return null
            return (
              <div key={group}>
                <div className="px-2 py-0.5 text-[9px] font-semibold text-gray-600 uppercase tracking-wider bg-surface-2 border-b border-surface-3">
                  {group}
                </div>
                {groupParams.map((p) => {
                  const appVal = appConfig?.[p.key]
                  const escVal = deviceConfig[p.key]
                  const mismatch = !p.readOnly && differs(appVal, escVal)
                  const sel = selections[p.key] ?? 'app'
                  return (
                    <div
                      key={p.key}
                      className={`flex items-center px-2 py-0.5 border-b border-surface-3/40 text-[10px]
                        ${mismatch ? 'bg-amber-500/5' : ''}`}
                    >
                      {/* App radio */}
                      <button
                        onClick={() => !p.readOnly && toggleParam(p.key, 'app')}
                        disabled={p.readOnly}
                        className={`w-5 h-5 flex items-center justify-center rounded-full border transition-colors flex-shrink-0
                          ${p.readOnly ? 'opacity-0 cursor-default' : 'cursor-pointer'}
                          ${sel === 'app' && !p.readOnly
                            ? 'border-accent bg-accent/20'
                            : 'border-surface-3 hover:border-gray-500'
                          }`}
                        title="Use app value"
                      >
                        {sel === 'app' && !p.readOnly && (
                          <span className="w-2 h-2 rounded-full bg-accent" />
                        )}
                      </button>

                      {/* ESC radio */}
                      <button
                        onClick={() => !p.readOnly && toggleParam(p.key, 'esc')}
                        disabled={p.readOnly}
                        className={`w-5 h-5 flex items-center justify-center rounded-full border transition-colors flex-shrink-0
                          ${p.readOnly ? 'opacity-0 cursor-default' : 'cursor-pointer'}
                          ${sel === 'esc' && !p.readOnly
                            ? 'border-online bg-online/20'
                            : 'border-surface-3 hover:border-gray-500'
                          }`}
                        title="Use ESC value"
                      >
                        {sel === 'esc' && !p.readOnly && (
                          <span className="w-2 h-2 rounded-full bg-online" />
                        )}
                      </button>

                      {/* Manual radio */}
                      <button
                        onClick={() => !p.readOnly && toggleParam(p.key, 'manual')}
                        disabled={p.readOnly}
                        className={`w-5 h-5 flex items-center justify-center rounded-full border transition-colors flex-shrink-0 mr-1
                          ${p.readOnly ? 'opacity-0 cursor-default' : 'cursor-pointer'}
                          ${sel === 'manual' && !p.readOnly
                            ? 'border-amber-400 bg-amber-400/20'
                            : 'border-surface-3 hover:border-gray-500'
                          }`}
                        title="Use manual value"
                      >
                        {sel === 'manual' && !p.readOnly && (
                          <span className="w-2 h-2 rounded-full bg-amber-400" />
                        )}
                      </button>

                      {/* Name */}
                      <span className={`flex-1 truncate ${mismatch ? 'text-amber-400' : 'text-gray-400'}`}>
                        {p.label}
                      </span>

                      {/* App value */}
                      <span className={`w-20 text-right font-mono pr-1 truncate flex-shrink-0
                        ${sel === 'app' && !p.readOnly ? 'text-accent' : 'text-gray-500'}`}>
                        {fmtVal(appVal)}
                      </span>

                      {/* ESC value */}
                      <span className={`w-20 text-right font-mono pr-1 truncate flex-shrink-0
                        ${sel === 'esc' && !p.readOnly ? 'text-online' : 'text-gray-500'}`}>
                        {fmtVal(escVal)}
                      </span>

                      {/* Manual input */}
                      {p.readOnly ? (
                        <span className="w-24 flex-shrink-0" />
                      ) : (
                        <input
                          type="text"
                          value={manualValues[p.key] ?? ''}
                          onChange={(e) => setManual(p.key, e.target.value)}
                          onClick={() => toggleParam(p.key, 'manual')}
                          className={`w-24 flex-shrink-0 text-right font-mono text-[10px] px-1 py-0 rounded border
                            bg-transparent outline-none transition-colors
                            ${sel === 'manual'
                              ? 'border-amber-400/50 text-amber-300'
                              : 'border-surface-3/50 text-gray-600 cursor-pointer'
                            }`}
                        />
                      )}
                    </div>
                  )
                })}
              </div>
            )
          })}

          {hasDiff && (
            <p className="px-3 py-2 text-[9px] text-amber-400">
              ● Amber rows: App ≠ ESC value
            </p>
          )}
        </div>
      )}
    </div>
  )
}
