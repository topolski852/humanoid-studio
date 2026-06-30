import { useEffect, useState } from 'react'
import { useNavigate } from 'react-router-dom'
import { useTelemetry } from '../context/TelemetryContext'
import { busColor } from '../utils/canDisplay'
import { api } from '../api'
import MotorCard from '../components/MotorCard'
import RobotDiagram from '../components/RobotDiagram'
import { BUSES } from '../constants'

function CanHealthBar({ canHealth, onNavigate }) {
  const byName = Object.fromEntries((canHealth ?? []).map((i) => [i.name, i]))
  const unconfiguredCount = BUSES.filter(b => {
    const s = byName[b.name]?.state
    return !s || s === 'UNCONFIGURED'
  }).length

  return (
    <div className="space-y-2">
      <div className="flex items-center gap-2 flex-wrap">
        {BUSES.map((bus) => {
          const iface    = byName[bus.name]
          const color    = busColor(iface?.state, iface?.bus_error_state)
          const isUnconfigured = !iface || iface.state === 'UNCONFIGURED'
          const isDown   = !iface || iface.state !== 'UP'
          const dotCls = {
            green:  'bg-online animate-pulse',
            yellow: 'bg-warn',
            red:    'bg-danger animate-pulse',
            grey:   'bg-offline',
          }[color]
          const pillCls = {
            green:  'border-online/20 bg-online/5 text-online hover:bg-online/10',
            yellow: 'border-warn/20 bg-warn/5 text-warn hover:bg-warn/10',
            red:    'border-danger/30 bg-danger/10 text-danger hover:bg-danger/20',
            grey:   'border-surface-3 bg-surface-2 text-gray-500 hover:bg-surface-3',
          }[color]

          return (
            <button
              key={bus.name}
              onClick={() => onNavigate(isUnconfigured ? '/can-setup' : `/can-monitor?bus=${bus.name}`)}
              className={`flex items-center gap-1.5 px-2.5 py-1 rounded-full text-[10px] font-mono border transition-colors ${pillCls}`}
            >
              <span className={`w-1.5 h-1.5 rounded-full flex-shrink-0 ${dotCls}`} />
              {bus.label}
              {isUnconfigured
                ? <span className="ml-0.5 font-sans uppercase tracking-wide">SETUP →</span>
                : isDown
                  ? <span className="ml-0.5 font-sans uppercase tracking-wide">DOWN</span>
                  : <span className="ml-0.5 text-[9px] opacity-70">{iface?.message_rate?.toFixed(0)} msg/s · {iface?.joints_online ?? 0}/{iface?.joints_total ?? 0}</span>
              }
            </button>
          )
        })}
      </div>
      {unconfiguredCount > 0 && (
        <p className="text-[10px] text-warn">
          {unconfiguredCount} bus{unconfiguredCount !== 1 ? 'es' : ''} need adapter assignment —{' '}
          <button
            onClick={() => onNavigate('/can-setup')}
            className="underline hover:text-warn/80"
          >
            open CAN Setup
          </button>
        </p>
      )}
    </div>
  )
}

function EmptyState() {
  return (
    <div className="flex flex-col items-center justify-center h-64 text-center">
      <div className="w-16 h-16 rounded-2xl bg-surface-2 border border-surface-3 flex items-center justify-center mb-4">
        <svg className="w-8 h-8 text-gray-600" fill="none" viewBox="0 0 24 24" stroke="currentColor" strokeWidth={1}>
          <path strokeLinecap="round" strokeLinejoin="round" d="M5.25 14.25h13.5m-13.5 0a3 3 0 01-3-3m3 3a3 3 0 100 6h13.5a3 3 0 100-6m-16.5-3a3 3 0 013-3h13.5a3 3 0 013 3m-19.5 0a4.5 4.5 0 01.9-2.7L5.737 5.1a3.375 3.375 0 012.7-1.35h7.126c1.062 0 2.062.5 2.7 1.35l2.587 3.45a4.5 4.5 0 01.9 2.7m0 0a3 3 0 01-3 3m0 3h.008v.008h-.008v-.008zm0-6h.008v.008h-.008v-.008zm-3 6h.008v.008h-.008v-.008zm0-6h.008v.008h-.008v-.008z" />
        </svg>
      </div>
      <p className="text-gray-400 font-medium mb-1">No robot config loaded</p>
      <p className="text-xs text-gray-600 max-w-xs">
        Go to Robot Config to load a configuration file, or the backend may not be running.
      </p>
    </div>
  )
}

function DeviceBadge({ iface }) {
  return (
    <span
      className={`inline-flex items-center gap-1.5 px-2 py-0.5 rounded text-[10px] font-mono border ${
        iface.up
          ? 'border-online/40 text-online bg-online/10'
          : 'border-surface-3 text-gray-500 bg-surface-2'
      }`}
    >
      <span className={`w-1.5 h-1.5 rounded-full ${iface.up ? 'bg-online' : 'bg-offline'}`} />
      {iface.name}
    </span>
  )
}

export default function Dashboard({ onOpenMotor }) {
  const navigate = useNavigate()
  const { states, robotConnected, canHealth, passiveTelemetry } = useTelemetry()
  const [joints, setJoints] = useState([])
  const [devices, setDevices] = useState([])
  const [loading, setLoading] = useState(true)
  const [error, setError] = useState(null)

  useEffect(() => {
    setLoading(true)
    Promise.all([
      api.getRobotConfig().then((config) => {
        if (config?.joints) setJoints(Object.values(config.joints))
        else setJoints([])
      }),
      api.getDevices().then((data) => {
        setDevices(data?.interfaces ?? [])
      }),
    ])
      .catch((e) => setError(e.message))
      .finally(() => setLoading(false))
  }, [])

  // A motor is online if it has active state OR is seen in passive traffic within 2s.
  // Motors that aren't on the bus (never detected) report no state and are hidden
  // from the card grid entirely — only show what's actually connected.
  const isOnline = (j) =>
    states[j.joint_name] != null || passiveTelemetry[j.joint_name]?.status === 'ONLINE'
  const detectedJoints = joints.filter(isOnline)
  const onlineCount = detectedJoints.length

  // Handler for diagram node clicks: find the matching joint and open its tab
  function handleDiagramClick(jointName) {
    const joint = joints.find((j) => j.joint_name === jointName)
    if (joint) onOpenMotor(joint.can_id, joint.joint_name)
  }

  return (
    <div className="h-full flex flex-col overflow-hidden">
      {/* Header */}
      <div className="flex-shrink-0 px-6 py-4 border-b border-surface-3 flex items-center justify-between">
        <div>
          <h1 className="font-semibold text-base">Dashboard</h1>
          {joints.length > 0 && (
            <p className="text-xs text-gray-500 mt-0.5">
              {onlineCount} of {joints.length} motors connected
            </p>
          )}
        </div>

        <div className="flex items-center gap-3">
          {/* CAN interface badges */}
          {devices.length > 0 && (
            <div className="flex items-center gap-1.5">
              {devices.map((iface) => (
                <DeviceBadge key={iface.name} iface={iface} />
              ))}
            </div>
          )}
          {devices.length === 0 && !loading && (
            <span className="text-[10px] text-gray-600 font-mono">no CAN interfaces</span>
          )}

          {joints.length > 0 && (
            <div className="flex gap-3 text-xs text-gray-500">
              <span className="flex items-center gap-1.5">
                <span className="w-2 h-2 rounded-full bg-online" style={{ boxShadow: '0 0 6px #22c55e' }} />
                {onlineCount} connected
              </span>
              <span className="flex items-center gap-1.5">
                <span className="w-2 h-2 rounded-full bg-offline" />
                {joints.length - onlineCount} not present
              </span>
            </div>
          )}
        </div>
      </div>

      {/* CAN health bar */}
      <div className="flex-shrink-0 px-6 py-2 border-b border-surface-3">
        <CanHealthBar canHealth={canHealth} onNavigate={navigate} />
      </div>

      {/* Body */}
      <div className="flex-1 overflow-hidden flex">
        {/* ── Left panel: robot diagram ───────────────────────────────── */}
        {joints.length > 0 && (
          <div className="flex-shrink-0 w-72 border-r border-surface-3 overflow-y-auto p-4 flex flex-col items-center justify-start">
            <RobotDiagram
              states={states}
              passiveTelemetry={passiveTelemetry}
              onOpenMotor={handleDiagramClick}
            />
          </div>
        )}

        {/* ── Right panel: motor cards ────────────────────────────────── */}
        <div className="flex-1 overflow-y-auto p-6">
          {loading ? (
            <div className="flex items-center justify-center h-32">
              <div className="w-6 h-6 rounded-full border-2 border-accent border-t-transparent animate-spin" />
            </div>
          ) : error ? (
            <div className="flex flex-col items-center justify-center h-32">
              <p className="text-sm text-danger">Failed to load config: {error}</p>
              <p className="text-xs text-gray-600 mt-1">Is the backend running on localhost:8765?</p>
            </div>
          ) : joints.length === 0 ? (
            <EmptyState />
          ) : detectedJoints.length === 0 ? (
            <div className="flex flex-col items-center justify-center h-32 text-center">
              <p className="text-sm text-gray-400">No motors connected</p>
              <p className="text-xs text-gray-600 mt-1">
                Click <span className="text-gray-400">Connect</span> to wake the robot — only
                detected motors appear here.
              </p>
            </div>
          ) : (
            <div className="grid grid-cols-2 lg:grid-cols-3 gap-4">
              {detectedJoints.map((joint) => (
                <MotorCard
                  key={joint.joint_name}
                  joint={joint}
                  state={states[joint.joint_name]}
                  passiveState={passiveTelemetry[joint.joint_name] ?? null}
                  onClick={() => onOpenMotor(joint.can_id, joint.joint_name)}
                />
              ))}
            </div>
          )}
        </div>
      </div>
    </div>
  )
}
