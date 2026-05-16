import { useEffect, useRef, useState } from 'react'
import { useParams } from 'react-router-dom'
import { useTelemetry } from '../context/TelemetryContext'
import { api } from '../api'
import StatusDot from './StatusDot'
import TelemetryTable from './TelemetryTable'
import MotorControlsPanel    from './MotorControlsPanel'
import MotorConfigPanel      from './MotorConfigPanel'
import MotorCalibrationPanel from './MotorCalibrationPanel'
import FlashWizard from './FlashWizard'
import ErrorLogPanel from './ErrorLogPanel'

const TABS = [
  { id: 'drive', label: 'Move' },
  { id: 'tune',  label: 'Tune' },
  { id: 'cal',   label: 'Cal'  },
  { id: 'log',   label: 'Log'  },
]

export default function MotorTab() {
  const { jointName } = useParams()
  const decodedName = decodeURIComponent(jointName)
  const { states, passiveTelemetry } = useTelemetry()
  const [config, setConfig] = useState(null)
  const [showFlash, setShowFlash] = useState(false)
  const [view, setView] = useState('drive')

  const MAX_LOG = 50
  const [errorLog, setErrorLog] = useState([])

  function addLogEntry(message, source = 'System') {
    setErrorLog(prev => [
      ...prev.slice(-(MAX_LOG - 1)),
      { id: Date.now(), time: new Date(), source, message },
    ])
  }

  useEffect(() => {
    api
      .getRobotConfig()
      .then((rc) => {
        const joint = rc?.joints?.[decodedName] ?? null
        setConfig(joint)
      })
      .catch(() => {})
  }, [decodedName])

  const displayName = config?.joint_name ?? decodedName
  const state = states[decodedName] ?? null
  const passiveState = passiveTelemetry[decodedName] ?? null
  const online = state != null || passiveState?.status === 'ONLINE'

  const histRef = useRef({ position: [], current: [] })
  const [history, setHistory] = useState({ position: [], current: [] })

  useEffect(() => {
    if (state == null) return
    const pos = [...histRef.current.position.slice(-49), state.position ?? 0]
    const cur = [...histRef.current.current.slice(-49), state.current ?? 0]
    histRef.current = { position: pos, current: cur }
    setHistory({ position: pos, current: cur })
  }, [state])

  // Dwell filter: require N consecutive non-zero (or zero) reads before logging.
  // Filters out transient 1-2 frame noise from the ESC error register.
  const ERR_DWELL = 4   // ~200 ms at 20 Hz — must stay non-zero this long to log
  const CLR_DWELL = 4   // ~200 ms at 20 Hz — must stay zero this long to log "cleared"
  const errDwellRef   = useRef(0)
  const clearDwellRef = useRef(0)
  const loggedErrRef  = useRef(0)  // non-zero while a logged error is "active"

  useEffect(() => {
    const cur = state?.error ?? 0

    if (cur !== 0) {
      clearDwellRef.current = 0
      errDwellRef.current += 1
      if (errDwellRef.current === ERR_DWELL && loggedErrRef.current === 0) {
        const names = state?.error_names ?? []
        const msg = names.length > 0
          ? names.join(', ')
          : `0x${cur.toString(16).padStart(8, '0').toUpperCase()}`
        addLogEntry(msg, 'Firmware')
        loggedErrRef.current = cur
      }
    } else {
      errDwellRef.current = 0
      if (loggedErrRef.current !== 0) {
        clearDwellRef.current += 1
        if (clearDwellRef.current === CLR_DWELL) {
          addLogEntry('Error cleared', 'Firmware')
          loggedErrRef.current = 0
        }
      }
    }
  }, [state])

  return (
    <div className="h-full flex flex-col">
      {/* Header */}
      <div className="flex-shrink-0 px-6 py-3.5 border-b border-surface-3 flex items-center gap-3">
        <StatusDot online={online} size="lg" />
        <div>
          <h2 className="font-semibold text-sm leading-none">
            {displayName.replace(/_joint$/, '').replace(/_/g, ' ')
              .replace(/\b\w/g, (c) => c.toUpperCase())}
          </h2>
          <p className="text-xs text-gray-500 font-mono mt-0.5">
            {config?.can_channel ?? '—'} · CAN ID {config?.can_id ?? '—'}
          </p>
        </div>
        <div className="ml-auto flex items-center gap-3 text-xs text-gray-500">
          {config?.gear_ratio != null && (
            <span>Gear ratio: <span className="font-mono text-gray-300">{config.gear_ratio}</span></span>
          )}
          {config?.phase_inverted != null && (
            <span className={`px-2 py-0.5 rounded font-mono ${config.phase_inverted ? 'bg-warn/20 text-warn' : 'bg-surface-2 text-gray-400'}`}>
              {config.phase_inverted ? 'PHASE INV' : 'PHASE NOM'}
            </span>
          )}
          {/* Tab strip */}
          <div className="flex rounded-lg border border-surface-3 overflow-hidden ml-2">
            {TABS.map(({ id, label }) => (
              <button
                key={id}
                onClick={() => setView(id)}
                className={`px-3 py-1 text-[11px] font-medium transition-colors
                  ${view === id
                    ? 'bg-accent/20 text-accent'
                    : 'bg-surface-2 text-gray-500 hover:text-gray-300'
                  }`}
              >
                {label}
              </button>
            ))}
          </div>
        </div>
      </div>

      {/* Two-column layout: TelemetryTable always left, active panel fills right */}
      <div className="flex-1 flex overflow-hidden">
        <TelemetryTable
          state={state}
          passiveState={passiveState}
          posHistory={history.position}
          currentHistory={history.current}
        />

        {view === 'drive' && (
          <MotorControlsPanel
            jointName={decodedName}
            state={state}
            config={config}
            onLogError={addLogEntry}
          />
        )}
        {view === 'tune' && (
          <MotorConfigPanel jointName={decodedName} onLogError={addLogEntry} />
        )}
        {view === 'cal' && (
          <MotorCalibrationPanel
            jointName={decodedName}
            state={state}
            config={config}
            onOpenFlash={() => setShowFlash(true)}
            onLogError={addLogEntry}
          />
        )}
        {view === 'log' && (
          <ErrorLogPanel logs={errorLog} onClear={() => setErrorLog([])} />
        )}
      </div>

      {/* Flash wizard modal */}
      {showFlash && (
        <FlashWizard
          canId={config?.can_id}
          canChannel={config?.can_channel}
          jointName={displayName}
          onClose={() => setShowFlash(false)}
        />
      )}
    </div>
  )
}
