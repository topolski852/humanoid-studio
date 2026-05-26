import { useEffect, useRef, useState } from 'react'
import { useNavigate, useSearchParams } from 'react-router-dom'
import { AreaChart, Area, ResponsiveContainer, Tooltip } from 'recharts'
import { useTelemetry } from '../context/TelemetryContext'
import { busErrorLabel, busColor } from '../utils/canDisplay'
import { api } from '../api'

const BUS_TO_LIMB = {
  can_left_leg:  'left_leg',
  can_right_leg: 'right_leg',
  can_left_arm:  'left_arm',
  can_right_arm: 'right_arm',
}

const ALL_BUSES = [
  { name: 'can_left_leg',  label: 'Left Leg' },
  { name: 'can_right_leg', label: 'Right Leg' },
  { name: 'can_left_arm',  label: 'Left Arm' },
  { name: 'can_right_arm', label: 'Right Arm' },
]

const COLOR_DOT = {
  green:  'bg-online',
  yellow: 'bg-warn',
  red:    'bg-danger',
  grey:   'bg-offline',
}
const COLOR_RING = {
  green:  'ring-online/30',
  yellow: 'ring-warn/30',
  red:    'ring-danger/30',
  grey:   'ring-surface-3',
}
const COLOR_TEXT = {
  green:  'text-online',
  yellow: 'text-warn',
  red:    'text-danger',
  grey:   'text-gray-500',
}
const COLOR_HEX = {
  green:  '#22c55e',
  yellow: '#f59e0b',
  red:    '#ef4444',
  grey:   '#4b5563',
}

// ── Recharts rate sparkline ───────────────────────────────────────────────────
function RateChart({ history, color }) {
  if (!history || history.length < 2) {
    return <div className="h-10 rounded bg-surface-3 opacity-20 flex-1" />
  }
  const data = history.map((v, i) => ({ i, v }))
  return (
    <div className="flex-1 h-10">
      <ResponsiveContainer width="100%" height="100%">
        <AreaChart data={data} margin={{ top: 2, right: 0, bottom: 0, left: 0 }}>
          <defs>
            <linearGradient id={`grad-${color.replace('#', '')}`} x1="0" y1="0" x2="0" y2="1">
              <stop offset="5%" stopColor={color} stopOpacity={0.25} />
              <stop offset="95%" stopColor={color} stopOpacity={0} />
            </linearGradient>
          </defs>
          <Area
            type="monotone"
            dataKey="v"
            stroke={color}
            strokeWidth={1.5}
            fill={`url(#grad-${color.replace('#', '')})`}
            dot={false}
            isAnimationActive={false}
          />
          <Tooltip
            content={({ active, payload }) =>
              active && payload?.[0] ? (
                <div className="bg-surface-2 border border-surface-3 rounded px-2 py-1 text-[10px] font-mono text-gray-300">
                  {payload[0].value.toFixed(1)} msg/s
                </div>
              ) : null
            }
          />
        </AreaChart>
      </ResponsiveContainer>
    </div>
  )
}

// ── Diagnostic hint for silent UP buses ──────────────────────────────────────
function SilentBusHint({ busName }) {
  return (
    <div className="rounded-lg bg-warn/5 border border-warn/20 px-3 py-2.5 space-y-1.5">
      <p className="text-[10px] text-warn font-medium">
        No CAN traffic detected on {busName}
      </p>
      <ul className="text-[10px] text-gray-400 space-y-0.5 list-disc list-inside">
        <li>Motors are not powered — check power connection</li>
        <li>Missing CAN termination — 120Ω resistor required at each end of the bus chain</li>
        <li>Loose cable between CAN adapter and motor chain</li>
      </ul>
      <p className="text-[10px] text-gray-500">Try: power on the motors, then click Refresh</p>
    </div>
  )
}

// ── Adapter card ──────────────────────────────────────────────────────────────
function AdapterCard({ meta, iface, hasAdapter, silentSince, onBringUp, onNavigateSetup }) {
  const [bringing, setBringing]   = useState(false)
  const [bringError, setBringError] = useState(null)
  const [resetting, setResetting] = useState(false)
  const color   = busColor(iface?.state, iface?.bus_error_state)
  const isDown  = !iface || iface.state !== 'UP'
  const isUnconfigured = !iface || iface.state === 'UNCONFIGURED'
  const totalJoints  = iface?.joints_total ?? 0
  const onlineJoints = iface?.joints_online ?? 0
  const jointPct     = totalJoints > 0 ? (onlineJoints / totalJoints) * 100 : 0
  const showSilent   = !isDown && (iface?.message_rate ?? 0) === 0 && silentSince != null

  async function handleBringUp() {
    setBringing(true)
    setBringError(null)
    try {
      await api.bringUpInterface(meta.name)
      onBringUp?.()
    } catch (e) {
      setBringError(e.message)
    }
    setBringing(false)
  }

  async function handleReset() {
    setResetting(true)
    setBringError(null)
    try {
      await api.bringUpInterface(meta.name)
    } catch (e) {
      setBringError(e.message)
    }
    setResetting(false)
  }

  return (
    <div className={`bg-surface-1 border rounded-xl p-4 flex flex-col gap-3 ${
      isDown ? 'border-warn/30' : 'border-surface-3'
    }`}>
      {/* Header */}
      <div className="flex items-center gap-2.5">
        <div className={`w-2.5 h-2.5 rounded-full ring-4 ${COLOR_DOT[color]} ${COLOR_RING[color]} ${
          color === 'green' ? 'animate-pulse' : ''
        }`} />
        <div className="flex-1 min-w-0">
          <p className="font-medium text-sm leading-none">{meta.label}</p>
          <p className="font-mono text-[10px] text-gray-500 mt-0.5">{meta.name}</p>
        </div>
        {!isDown && (
          <button
            onClick={handleReset}
            disabled={resetting}
            title="Cycle ip link down → up (daemon restarts socket on next reboot)"
            className="text-[10px] px-2 py-0.5 rounded border border-surface-3 text-gray-500 hover:text-gray-300 hover:border-gray-500 transition-colors disabled:opacity-40 whitespace-nowrap mr-1"
          >
            {resetting ? '…' : 'Reset'}
          </button>
        )}
        <span className={`text-xs font-mono font-medium ${COLOR_TEXT[color]}`}>
          {iface?.state ?? 'UNKNOWN'}
        </span>
      </div>

      {/* Error state label */}
      {iface?.state === 'UP' && (
        <div className={`text-[10px] font-medium px-2 py-0.5 rounded self-start ${
          color === 'green' ? 'bg-online/10 text-online' :
          color === 'yellow' ? 'bg-warn/10 text-warn' :
          'bg-danger/10 text-danger'
        }`}>
          {busErrorLabel(iface.bus_error_state)}
        </div>
      )}

      {/* Counters */}
      <div className="grid grid-cols-2 gap-x-4 gap-y-1 text-[10px] font-mono">
        <span className="text-gray-500">RX <span className="text-gray-300">{iface?.rx_packets?.toLocaleString() ?? '—'}</span></span>
        <span className="text-gray-500">TX <span className="text-gray-300">{iface?.tx_packets?.toLocaleString() ?? '—'}</span></span>
        <span className="text-gray-500">Err <span className={iface?.rx_errors || iface?.tx_errors ? 'text-danger' : 'text-gray-300'}>
          {((iface?.rx_errors ?? 0) + (iface?.tx_errors ?? 0)).toLocaleString()}
        </span></span>
        <span className="text-gray-500">Drop <span className="text-gray-300">
          {((iface?.rx_dropped ?? 0) + (iface?.tx_dropped ?? 0)).toLocaleString()}
        </span></span>
      </div>

      {/* Message rate + recharts sparkline */}
      <div className="flex items-center gap-2">
        <span className="font-mono text-xs text-gray-300 whitespace-nowrap">
          {iface?.message_rate?.toFixed(1) ?? '0.0'} msg/s
        </span>
        <RateChart history={iface?.rate_history} color={COLOR_HEX[color]} />
      </div>

      {/* Joints online progress bar */}
      {totalJoints > 0 && (
        <div className="space-y-1">
          <div className="flex justify-between text-[10px] text-gray-500">
            <span>Joints</span>
            <span className="font-mono">{onlineJoints}/{totalJoints} online</span>
          </div>
          <div className="h-1 bg-surface-3 rounded-full overflow-hidden">
            <div
              className="h-full bg-online rounded-full transition-all duration-500"
              style={{ width: `${jointPct}%` }}
            />
          </div>
          {/* Per-joint status dots */}
          {(iface?.joints ?? []).length > 0 && (
            <div className="flex flex-wrap gap-1 mt-1">
              {iface.joints.map(j => (
                <span
                  key={j.can_id}
                  title={`${j.name} (ID ${j.can_id}) — ${j.status}`}
                  className={`w-2 h-2 rounded-full ${
                    j.status === 'ONLINE' ? 'bg-online' :
                    j.status === 'STALE'  ? 'bg-warn'   : 'bg-offline'
                  }`}
                />
              ))}
            </div>
          )}
        </div>
      )}

      {/* USB path */}
      {iface?.usb_path && (
        <p className="text-[9px] text-gray-700 font-mono truncate" title={iface.usb_path}>
          {iface.usb_path.replace('/sys/devices', '...')}
        </p>
      )}

      {/* Silent bus diagnostic hint */}
      {showSilent && <SilentBusHint busName={meta.name} />}

      {/* No adapter assigned */}
      {!hasAdapter && (
        <div className="rounded-lg bg-surface-2 border border-surface-3 px-3 py-2 flex items-center gap-2">
          <span className="text-[10px] text-gray-500 flex-1">No adapter assigned</span>
          <button
            onClick={onNavigateSetup}
            className="text-[10px] px-2 py-1 rounded bg-accent/10 text-accent hover:bg-accent/20 transition-colors whitespace-nowrap"
          >
            Configure →
          </button>
        </div>
      )}

      {/* DOWN + bring-up */}
      {hasAdapter && isDown && !isUnconfigured && (
        <div className="rounded-lg bg-warn/10 border border-warn/20 px-3 py-2 flex items-center gap-2">
          <span className="text-[10px] text-warn flex-1">Interface down — check USB cable</span>
          <button
            onClick={handleBringUp}
            disabled={bringing}
            className="text-[10px] px-2 py-1 rounded bg-warn/20 text-warn hover:bg-warn/30 transition-colors disabled:opacity-50 whitespace-nowrap"
          >
            {bringing ? '…' : 'Bring Up'}
          </button>
        </div>
      )}
      {bringError && <p className="text-[9px] text-danger leading-tight">{bringError}</p>}
    </div>
  )
}

// ── Traffic table ─────────────────────────────────────────────────────────────
function TrafficTable({ busName, traffic }) {
  const rows = traffic[busName] ?? []

  if (rows.length === 0) {
    return (
      <div className="flex items-center justify-center h-24 text-xs text-gray-600">
        No frames seen on {busName}
      </div>
    )
  }

  function staleness(lastSeenAgo) {
    if (lastSeenAgo < 1)  return { cls: 'text-online', dot: 'bg-online' }
    if (lastSeenAgo < 5)  return { cls: 'text-warn',   dot: 'bg-warn'   }
    return                       { cls: 'text-gray-600', dot: 'bg-offline' }
  }

  const fmtVal = (v, unit) =>
    v != null ? `${v > 0 ? '+' : ''}${v}${unit}` : '—'

  return (
    <div className="overflow-x-auto">
      <table className="w-full text-xs font-mono border-collapse">
        <thead>
          <tr className="text-gray-600 border-b border-surface-3">
            <th className="text-left py-2 px-3 font-normal data-label">Node</th>
            <th className="text-left py-2 px-3 font-normal data-label">Joint</th>
            <th className="text-left py-2 px-3 font-normal data-label">Func</th>
            <th className="text-right py-2 px-3 font-normal data-label">Position</th>
            <th className="text-right py-2 px-3 font-normal data-label">Velocity</th>
            <th className="text-right py-2 px-3 font-normal data-label">Msg/s</th>
            <th className="text-right py-2 px-3 font-normal data-label">Age</th>
            <th className="py-2 px-3" />
          </tr>
        </thead>
        <tbody>
          {rows.map((row) => {
            const s = staleness(row.last_seen_ago)
            const opacity = row.last_seen_ago > 5 ? 'opacity-50' : ''
            const jointLabel = row.joint_name
              ? row.joint_name.replace(/_joint$/, '').replace(/_/g, ' ')
              : <span className="text-gray-600">—</span>
            return (
              <tr
                key={row.arb_id_hex}
                className={`border-b border-surface-3/40 hover:bg-white/5 transition-opacity ${opacity}`}
                title={`raw arb_id: ${row.arb_id_hex}`}
              >
                {/* Node ID — decimal, with raw compound ID as tooltip */}
                <td className="py-1.5 px-3 text-accent tabular-nums">{row.node_id}</td>
                <td className="py-1.5 px-3 text-gray-300 capitalize">{jointLabel}</td>
                <td className="py-1.5 px-3 text-gray-500 text-[10px]">{row.func_name}</td>
                <td className="py-1.5 px-3 text-right tabular-nums text-gray-300">
                  {row.position_deg != null ? `${row.position_deg}°` : '—'}
                </td>
                <td className="py-1.5 px-3 text-right tabular-nums text-gray-400">
                  {row.velocity_dps != null ? `${row.velocity_dps}°/s` : '—'}
                </td>
                <td className="py-1.5 px-3 text-right tabular-nums">{row.message_rate}</td>
                <td className={`py-1.5 px-3 text-right tabular-nums ${s.cls}`}>
                  {row.last_seen_ago.toFixed(1)}s
                </td>
                <td className="py-1.5 px-3">
                  <span className={`w-1.5 h-1.5 rounded-full inline-block ${s.dot}`} />
                </td>
              </tr>
            )
          })}
        </tbody>
      </table>
    </div>
  )
}

// ── Drop log ──────────────────────────────────────────────────────────────────
function DropLog({ events }) {
  if (events.length === 0) {
    return <p className="text-xs text-gray-600 px-4 py-3">No drop events recorded.</p>
  }
  return (
    <div className="space-y-0.5 max-h-40 overflow-y-auto">
      {events.map((ev, i) => {
        const ts = new Date(ev.timestamp).toLocaleTimeString()
        const isDown = ev.event === 'down'
        return (
          <div key={i} className={`flex items-start gap-2 px-4 py-1.5 text-xs font-mono ${
            isDown ? 'text-danger/80' : 'text-online/80'
          }`}>
            <span className="text-gray-600 flex-shrink-0">[{ts}]</span>
            <span>
              {ev.interface} {isDown
                ? `went DOWN — RX errors: ${ev.rx_errors}, TX errors: ${ev.tx_errors}`
                : 'came back UP'
              }
            </span>
          </div>
        )
      })}
    </div>
  )
}

// ── Main page ─────────────────────────────────────────────────────────────────
export default function CanMonitor() {
  const { canHealth, dropLog } = useTelemetry()
  const navigate = useNavigate()
  const [searchParams] = useSearchParams()
  const [selectedBus, setSelectedBus] = useState(
    () => searchParams.get('bus') ?? 'can_left_leg'
  )
  const [traffic, setTraffic]   = useState({})
  const [limbSlots, setLimbSlots] = useState({})
  const pollRef   = useRef(null)
  // Track when each UP interface first went silent (0 msg/s)
  const silentSinceRef = useRef({})

  // Poll traffic every 500ms
  useEffect(() => {
    async function poll() {
      try {
        const data = await api.getCanTraffic()
        setTraffic(data ?? {})
      } catch {}
    }
    poll()
    pollRef.current = setInterval(poll, 500)
    return () => clearInterval(pollRef.current)
  }, [])

  // Fetch adapter assignments once on mount
  useEffect(() => {
    api.getAdapters()
      .then(data => setLimbSlots(data?.limb_slots ?? {}))
      .catch(() => {})
  }, [])

  // Track when UP interfaces first went silent (for diagnostic hint after 10s)
  const [silentSince, setSilentSince] = useState({})
  useEffect(() => {
    const now = Date.now()
    const next = { ...silentSinceRef.current }
    for (const iface of canHealth) {
      if (iface.state === 'UP' && (iface.message_rate ?? 0) === 0) {
        if (!next[iface.name]) next[iface.name] = now
      } else {
        delete next[iface.name]
      }
    }
    silentSinceRef.current = next
    setSilentSince({ ...next })
  }, [canHealth])

  const healthByName = Object.fromEntries((canHealth ?? []).map((i) => [i.name, i]))

  return (
    <div className="h-full flex flex-col overflow-hidden">
      {/* Header */}
      <div className="flex-shrink-0 px-6 py-4 border-b border-surface-3">
        <h1 className="font-semibold text-base">CAN Bus Monitor</h1>
        <p className="text-xs text-gray-500 mt-0.5">Live adapter health and bus traffic</p>
      </div>

      <div className="flex-1 overflow-y-auto">
        {/* ── Adapter cards ── */}
        <div className="px-6 pt-6 pb-4 grid grid-cols-2 xl:grid-cols-4 gap-4">
          {ALL_BUSES.map((meta) => {
            const limb = BUS_TO_LIMB[meta.name]
            const hasAdapter = limb ? limbSlots[limb] != null : false
            const isSilentLong = silentSince[meta.name] != null &&
              Date.now() - silentSince[meta.name] > 10_000
            return (
              <AdapterCard
                key={meta.name}
                meta={meta}
                iface={healthByName[meta.name]}
                hasAdapter={hasAdapter}
                silentSince={isSilentLong ? silentSince[meta.name] : null}
                onBringUp={async () => {
                  try { await api.bringUpInterface(meta.name) } catch {}
                }}
                onNavigateSetup={() => navigate('/can-setup')}
              />
            )
          })}
        </div>

        {/* ── Traffic section ── */}
        <div className="px-6 pb-6">
          <div className="bg-surface-1 border border-surface-3 rounded-xl overflow-hidden">
            {/* Tab bar */}
            <div className="flex border-b border-surface-3">
              {ALL_BUSES.map((meta) => {
                const iface = healthByName[meta.name]
                const color = busColor(iface?.state, iface?.bus_error_state)
                return (
                  <button
                    key={meta.name}
                    onClick={() => setSelectedBus(meta.name)}
                    className={`flex items-center gap-2 px-4 py-2.5 text-xs transition-colors border-b-2 ${
                      selectedBus === meta.name
                        ? 'border-accent text-accent bg-accent/5'
                        : 'border-transparent text-gray-500 hover:text-gray-300 hover:bg-white/5'
                    }`}
                  >
                    <span className={`w-1.5 h-1.5 rounded-full ${COLOR_DOT[color]}`} />
                    {meta.label}
                    <span className="ml-1 font-mono text-[9px] text-gray-600">
                      {(traffic[meta.name] ?? []).length}
                    </span>
                  </button>
                )
              })}
            </div>

            {/* Traffic table */}
            <div className="min-h-[160px]">
              <TrafficTable busName={selectedBus} traffic={traffic} />
            </div>

            {/* Drop log */}
            <div className="border-t border-surface-3">
              <div className="px-4 py-2 flex items-center justify-between">
                <span className="data-label">CAN Drop Log</span>
                <span className="text-[10px] text-gray-600">{dropLog.length} events</span>
              </div>
              <DropLog events={dropLog} />
            </div>
          </div>
        </div>
      </div>
    </div>
  )
}
