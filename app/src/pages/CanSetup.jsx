import { useEffect, useRef, useState } from 'react'
import { useNavigate } from 'react-router-dom'
import { useTelemetry } from '../context/TelemetryContext'
import { busErrorLabel } from '../utils/canDisplay'
import { api } from '../api'

const LIMB_META = [
  { limb: 'left_arm',  label: 'Left Arm',  busName: 'can_left_arm' },
  { limb: 'right_arm', label: 'Right Arm', busName: 'can_right_arm' },
  { limb: 'left_leg',  label: 'Left Leg',  busName: 'can_left_leg' },
  { limb: 'right_leg', label: 'Right Leg', busName: 'can_right_leg' },
]

// ── Status bar ────────────────────────────────────────────────────────────────
function StatusBar({ adapters, canHealth }) {
  const totalDetected  = adapters.length
  const unassigned     = adapters.filter(a => !a.assigned_limb).length
  const assigned       = adapters.filter(a =>  a.assigned_limb).length
  const healthMap      = Object.fromEntries((canHealth ?? []).map(i => [i.name, i]))

  const assignedAdapters = adapters.filter(a => a.assigned_limb)
  const up           = assignedAdapters.filter(a => healthMap[`can_${a.assigned_limb}`]?.state === 'UP').length
  const down         = assignedAdapters.filter(a => healthMap[`can_${a.assigned_limb}`]?.state === 'DOWN').length
  const unconfigured = assigned - up - down

  let color, label
  if (totalDetected === 0) {
    color = 'bg-danger/10 border-danger/20 text-danger'
    label = 'No USB-CAN adapters detected — plug in adapters to configure'
  } else if (assigned === 0) {
    color = 'bg-warn/10 border-warn/20 text-warn'
    label = `${unassigned} adapter${unassigned !== 1 ? 's' : ''} detected — assign ${unassigned !== 1 ? 'them' : 'it'} to a limb below`
  } else if (unassigned > 0) {
    color = 'bg-warn/10 border-warn/20 text-warn'
    label = `${assigned} assigned · ${unassigned} unassigned — assign remaining adapters below`
  } else if (unconfigured > 0) {
    color = 'bg-warn/10 border-warn/20 text-warn'
    label = `${unconfigured} adapter${unconfigured !== 1 ? 's' : ''} not found — replug or check udev rules`
  } else if (down > 0) {
    color = 'bg-warn/10 border-warn/20 text-warn'
    label = `${down} adapter${down !== 1 ? 's' : ''} down · ${up} online`
  } else if (up > 0) {
    color = 'bg-online/10 border-online/20 text-online'
    label = `${up} adapter${up !== 1 ? 's' : ''} online`
  } else {
    color = 'bg-surface-2/10 border-surface-3 text-gray-400'
    label = `${assigned} adapter${assigned !== 1 ? 's' : ''} assigned — waiting for interface`
  }

  return (
    <div className={`flex-shrink-0 mx-6 mt-4 rounded-lg border px-4 py-2 text-xs ${color}`}>
      {label}
    </div>
  )
}

// ── Limb slot card (always-visible 2×2 grid) ─────────────────────────────────
function LimbSlotCard({ meta, adapter, iface, jointCount, unassigned, onAssign, onUnassign, onBringUp, onRefresh }) {
  const [selectedSerial, setSelectedSerial] = useState('')
  const [assigning, setAssigning]           = useState(false)
  const [bringingUp, setBringingUp]         = useState(false)
  const [error, setError]                   = useState(null)
  const [pingResult, setPingResult]         = useState(null)  // {count, device_ids} | null
  const [pinging, setPinging]               = useState(false)
  const [scanResult, setScanResult]         = useState(null)  // scan response | null
  const [scanning, setScanning]             = useState(false)
  const recheckTimerRef                     = useRef(null)

  useEffect(() => () => { if (recheckTimerRef.current) clearTimeout(recheckTimerRef.current) }, [])

  // When the unassigned list changes, reset the selection if it disappeared
  useEffect(() => {
    if (selectedSerial && !unassigned.find(a => a.usb_serial === selectedSerial)) {
      setSelectedSerial('')
    }
  }, [unassigned, selectedSerial])

  async function handleAssign() {
    if (!selectedSerial) return
    setAssigning(true)
    setError(null)
    try {
      await onAssign(selectedSerial, meta.limb)
      // Auto-recheck after 3 s to let the interface come up
      recheckTimerRef.current = setTimeout(onRefresh, 3000)
    } catch (e) {
      setError(e.message)
    } finally {
      setAssigning(false)
    }
  }

  async function handleUnassign() {
    if (!adapter) return
    setError(null)
    try {
      await onUnassign(adapter.usb_serial)
    } catch (e) {
      setError(e.message)
    }
  }

  async function handleBringUp() {
    setBringingUp(true)
    setError(null)
    try {
      await onBringUp(meta.busName)
      recheckTimerRef.current = setTimeout(onRefresh, 2000)
    } catch (e) {
      setError(e.message)
    } finally {
      setBringingUp(false)
    }
  }

  async function handlePing() {
    setPinging(true)
    setPingResult(null)
    try {
      const result = await api.pingBus(meta.busName)
      setPingResult(result)
    } catch (e) {
      setPingResult({ error: e.message })
    } finally {
      setPinging(false)
    }
  }

  async function handleScan() {
    setScanning(true)
    setScanResult(null)
    try {
      const result = await api.scanBus(meta.busName)
      setScanResult(result)
    } catch (e) {
      setScanResult({ error: e.message })
    } finally {
      setScanning(false)
    }
  }

  const isAssigned     = !!adapter
  const state          = iface?.state ?? (adapter ? 'UNCONFIGURED' : null)
  const isUp           = state === 'UP'
  const isDown         = state === 'DOWN'
  const isUnconfigured = state === 'UNCONFIGURED' || state === 'UNKNOWN' || (isAssigned && !iface)
  const msgRate    = iface?.message_rate
  const jOnline    = iface?.joints_online ?? 0
  const jTotal     = jointCount ?? 0
  const serialTail = adapter?.usb_serial?.slice(-6) ?? ''

  // Border style based on state
  let borderClass = 'border-2 border-dashed border-surface-3/60'
  if (isAssigned) {
    if (isUp)           borderClass = 'border border-online/40'
    else if (isUnconfigured) borderClass = 'border border-warn/40'
    else                borderClass = 'border border-surface-3'
  }

  // Dot color
  let dotClass = 'bg-surface-3'
  if (isAssigned) {
    if (isUp)           dotClass = 'bg-online animate-pulse'
    else if (isUnconfigured) dotClass = 'bg-warn'
    else                dotClass = 'bg-offline'
  }

  return (
    <div className={`bg-surface-1 rounded-xl p-4 flex flex-col gap-3 ${borderClass}`}>
      {/* Header */}
      <div className="flex items-center gap-2">
        <span className={`w-2 h-2 rounded-full flex-shrink-0 ${dotClass}`} />
        <div className="flex-1 min-w-0">
          <p className="text-sm font-medium leading-none">{meta.label}</p>
          <p className="text-[10px] font-mono text-gray-600 mt-0.5">{meta.busName}</p>
        </div>
        {isAssigned && (
          <button
            onClick={handleUnassign}
            title="Unassign adapter"
            className="text-gray-600 hover:text-danger text-xs px-1 transition-colors"
          >
            ×
          </button>
        )}
      </div>

      {/* Assigned: show state + live stats */}
      {isAssigned && (
        <div className="space-y-1">
          <div className="flex items-center gap-2 text-[10px] font-mono">
            <span className={
              isUp ? 'text-online' :
              isUnconfigured ? 'text-warn' :
              'text-gray-500'
            }>
              {busErrorLabel(state ?? 'UNKNOWN')}
            </span>
            <span className="text-gray-600">·</span>
            <span className="text-gray-400">{serialTail}</span>
          </div>

          {isDown && (
            <div className="flex items-center gap-2 rounded-lg bg-warn/5 border border-warn/20 px-2 py-1.5 mt-1">
              <span className="text-[9px] text-warn flex-1">Interface down</span>
              <button
                onClick={handleBringUp}
                disabled={bringingUp}
                className="text-[9px] px-2 py-1 rounded bg-warn/20 text-warn hover:bg-warn/30 disabled:opacity-50 whitespace-nowrap transition-colors"
              >
                {bringingUp ? '…' : 'Bring Up'}
              </button>
            </div>
          )}

          {isUnconfigured && (
            <p className="text-[9px] text-warn/80 leading-tight">
              Interface not found — replug adapter or check udev rules
            </p>
          )}

          {isUp && msgRate != null && (
            <p className="text-[10px] font-mono text-gray-400">
              {msgRate.toFixed(1)} msg/s
              {jTotal > 0 && (
                <span className={`ml-2 ${jOnline === jTotal ? 'text-online' : 'text-warn'}`}>
                  {jOnline}/{jTotal} joints {jOnline === jTotal ? '✓' : ''}
                </span>
              )}
            </p>
          )}

          {/* Action buttons — only when UP */}
          {isUp && (
            <div className="pt-1 space-y-2">
              <div className="flex gap-1.5">
                <button
                  onClick={handlePing}
                  disabled={pinging || scanning}
                  className="text-[10px] px-2 py-1 rounded bg-surface-2 border border-surface-3 text-gray-400 hover:text-gray-200 hover:border-accent/40 disabled:opacity-50 transition-colors"
                >
                  {pinging ? '…listening' : 'Passive Listen'}
                </button>
                <button
                  onClick={handleScan}
                  disabled={scanning || pinging}
                  className="text-[10px] px-2 py-1 rounded bg-surface-2 border border-surface-3 text-gray-400 hover:text-gray-200 hover:border-accent/40 disabled:opacity-50 transition-colors"
                >
                  {scanning ? '…scanning IDs 1-20' : 'Scan Bus'}
                </button>
              </div>

              {pingResult && !pingResult.error && (
                <p className="text-[9px] font-mono text-gray-400">
                  {pingResult.count === 0
                    ? 'No traffic seen in 2s'
                    : `Traffic: ${pingResult.count} device ID${pingResult.count !== 1 ? 's' : ''}: ${pingResult.device_ids.join(', ')}`
                  }
                </p>
              )}
              {pingResult?.error && (
                <p className="text-[9px] text-danger">{pingResult.error}</p>
              )}

              {/* Scan results table */}
              {scanResult && !scanResult.error && (
                <div className="mt-1 space-y-1">
                  <p className="text-[9px] text-gray-500 font-mono">
                    Scan complete · {scanResult.scan_duration_ms}ms
                  </p>
                  <div className="space-y-0.5">
                    {scanResult.results
                      .filter(r => r.responded || r.expected)
                      .map(r => (
                        <div key={r.can_id} className={`flex items-center gap-1.5 text-[9px] font-mono ${
                          !r.expected && r.responded ? 'text-warn' :
                          r.responded ? 'text-online' : 'text-gray-600'
                        }`}>
                          <span className="w-3 text-center flex-shrink-0">
                            {r.responded ? '✓' : '✗'}
                          </span>
                          <span className="w-5 text-right flex-shrink-0">{r.can_id}</span>
                          <span className="flex-1 truncate">
                            {r.joint_name
                              ? r.joint_name.replace(/_joint$/, '').replace(/_/g, ' ')
                              : '(unexpected)'}
                          </span>
                          <span className="flex-shrink-0">
                            {r.response_time_ms != null ? `${r.response_time_ms}ms` : '—'}
                          </span>
                        </div>
                      ))
                    }
                  </div>
                  <div className="pt-1 border-t border-surface-3 text-[9px] font-mono text-gray-500">
                    {scanResult.summary.expected_found}/{scanResult.summary.expected_found + scanResult.summary.expected_missing} expected responding
                    {scanResult.summary.unexpected_found > 0 && (
                      <span className="ml-2 text-warn">
                        · {scanResult.summary.unexpected_found} unexpected ID{scanResult.summary.unexpected_found !== 1 ? 's' : ''} — may need CAN ID reconfiguration
                      </span>
                    )}
                  </div>
                </div>
              )}
              {scanResult?.error && (
                <p className="text-[9px] text-danger">{scanResult.error}</p>
              )}
            </div>
          )}
        </div>
      )}

      {/* Empty: show assign controls */}
      {!isAssigned && (
        <>
          <p className="text-[10px] text-gray-600">
            {jTotal > 0 ? `Controls ${jTotal} joints` : 'No joints assigned in config'}
          </p>
          {unassigned.length > 0 ? (
            <div className="flex gap-1.5">
              <select
                value={selectedSerial}
                onChange={e => setSelectedSerial(e.target.value)}
                className="flex-1 bg-surface-2 border border-surface-3 rounded text-[10px] font-mono text-gray-300 px-2 py-1 focus:outline-none focus:border-accent"
              >
                <option value="">Select adapter…</option>
                {unassigned.map(a => (
                  <option key={a.usb_serial ?? a.current_name} value={a.usb_serial}>
                    {a.current_name} · {a.usb_serial?.slice(-6) ?? 'no serial'}
                  </option>
                ))}
              </select>
              <button
                onClick={handleAssign}
                disabled={!selectedSerial || assigning}
                className="px-2 py-1 rounded bg-accent/20 text-accent text-[10px] hover:bg-accent/30 disabled:opacity-40 transition-colors whitespace-nowrap"
              >
                {assigning ? '…' : 'Assign'}
              </button>
            </div>
          ) : (
            <p className="text-[10px] text-gray-600 italic">No unassigned adapters</p>
          )}
        </>
      )}

      {error && <p className="text-[9px] text-danger leading-tight">{error}</p>}
    </div>
  )
}

// ── Unassigned adapter card ───────────────────────────────────────────────────
function UnassignedAdapterCard({ adapter, availableLimbs, onAssign, onBringUp }) {
  const [selectedLimb, setSelectedLimb] = useState(availableLimbs[0] ?? '')
  const [assigning, setAssigning]       = useState(false)
  const [bringingUp, setBringingUp]     = useState(false)
  const [error, setError]               = useState(null)
  const [success, setSuccess]           = useState(null)

  useEffect(() => {
    if (selectedLimb && !availableLimbs.includes(selectedLimb)) {
      setSelectedLimb(availableLimbs[0] ?? '')
    }
  }, [availableLimbs, selectedLimb])

  async function handleAssign() {
    if (!selectedLimb) return
    setAssigning(true)
    setError(null)
    setSuccess(null)
    try {
      const result = await onAssign(adapter.usb_serial, selectedLimb)
      setSuccess(`Renamed to ${result?.new_name ?? `can_${selectedLimb}`} — interface is UP`)
    } catch (e) {
      setError(e.message)
    } finally {
      setAssigning(false)
    }
  }

  async function handleBringUp() {
    setBringingUp(true)
    setError(null)
    try {
      await onBringUp(adapter.current_name)
    } catch (e) {
      setError(e.message)
    } finally {
      setBringingUp(false)
    }
  }

  const isDown = adapter.state !== 'UP'
  const serialTail = adapter.usb_serial?.slice(-6) ?? 'no serial'

  return (
    <div className="bg-surface-1 border border-warn/30 rounded-xl p-4 flex flex-col gap-3">
      <div className="flex items-center gap-2.5">
        <div className="w-7 h-7 rounded-lg bg-warn/10 flex items-center justify-center flex-shrink-0">
          <svg className="w-4 h-4 text-warn" fill="none" viewBox="0 0 24 24" stroke="currentColor" strokeWidth={2}>
            <path strokeLinecap="round" strokeLinejoin="round" d="M13 10V3L4 14h7v7l9-11h-7z" />
          </svg>
        </div>
        <div className="flex-1 min-w-0">
          <p className="text-sm font-medium leading-none font-mono">{adapter.current_name}</p>
          <p className="text-[10px] text-gray-500 mt-0.5">
            Serial: {adapter.usb_serial ?? 'unknown'}
            {adapter.usb_path && (
              <span className="ml-2 text-gray-600">· {adapter.usb_path}</span>
            )}
          </p>
        </div>
        <span className={`text-[10px] font-mono font-medium ${isDown ? 'text-gray-500' : 'text-online'}`}>
          {adapter.state}
        </span>
      </div>

      {isDown && !success && (
        <div className="flex items-center gap-2 rounded-lg bg-warn/5 border border-warn/20 px-3 py-2">
          <span className="text-[10px] text-warn flex-1">Interface down — bring up before assigning</span>
          <button
            onClick={handleBringUp}
            disabled={bringingUp}
            className="text-[10px] px-2 py-1 rounded bg-warn/20 text-warn hover:bg-warn/30 disabled:opacity-50 whitespace-nowrap transition-colors"
          >
            {bringingUp ? '…' : 'Bring Up'}
          </button>
        </div>
      )}

      {!success && (
        <div className="space-y-1.5">
          <p className="text-[10px] text-gray-500">Assign to limb:</p>
          <div className="flex gap-2">
            <select
              value={selectedLimb}
              onChange={e => setSelectedLimb(e.target.value)}
              disabled={assigning || availableLimbs.length === 0}
              className="flex-1 bg-surface-2 border border-surface-3 rounded text-xs font-mono text-gray-300 px-2 py-1.5 focus:outline-none focus:border-accent"
            >
              {availableLimbs.length === 0
                ? <option value="">All limbs assigned</option>
                : availableLimbs.map(limb => (
                    <option key={limb} value={limb}>
                      {limb.replace('_', ' ').replace(/\b\w/g, c => c.toUpperCase())}
                    </option>
                  ))
              }
            </select>
            <button
              onClick={handleAssign}
              disabled={assigning || availableLimbs.length === 0}
              className="px-3 py-1.5 rounded bg-accent/20 text-accent text-xs font-medium hover:bg-accent/30 disabled:opacity-40 transition-colors whitespace-nowrap"
            >
              {assigning ? '…' : 'Assign →'}
            </button>
          </div>
        </div>
      )}

      {success && (
        <div className="rounded-lg bg-online/10 border border-online/20 px-3 py-2 text-[10px] text-online">
          {success}
        </div>
      )}
      {error && <p className="text-[9px] text-danger leading-tight">{error}</p>}
    </div>
  )
}

// ── UDEV rules status (collapsible) ──────────────────────────────────────────
function UdevStatus({ exists, content }) {
  const [open, setOpen] = useState(false)
  return (
    <div className="bg-surface-1 border border-surface-3 rounded-xl overflow-hidden">
      <button
        onClick={() => setOpen(v => !v)}
        className="w-full flex items-center gap-2 px-4 py-3 text-left hover:bg-white/5 transition-colors"
      >
        <span className={`w-2 h-2 rounded-full flex-shrink-0 ${exists ? 'bg-online' : 'bg-warn'}`} />
        <span className="text-xs font-medium flex-1">Persistent Name Rules</span>
        <span className="text-gray-600 text-[10px]">{open ? '▲' : '▼'}</span>
      </button>
      {open && (
        <div className="px-4 pb-3 space-y-2">
          {exists ? (
            <>
              <p className="text-[10px] text-online">
                Adapters will keep their names after replug (udev rule installed)
              </p>
              <pre className="text-[9px] font-mono text-gray-500 bg-surface-2 rounded p-2 overflow-x-auto whitespace-pre-wrap break-all">
                {content || '(empty)'}
              </pre>
            </>
          ) : (
            <p className="text-[10px] text-warn">
              Names will reset after replug — assign adapters above to create persistent udev rules.
            </p>
          )}
        </div>
      )}
    </div>
  )
}

// ── Main page ─────────────────────────────────────────────────────────────────
export default function CanSetup() {
  const { canHealth } = useTelemetry()
  const navigate      = useNavigate()
  const [data, setData]       = useState(null)
  const [loading, setLoading] = useState(true)
  const [error, setError]     = useState(null)

  async function refresh() {
    setLoading(true)
    setError(null)
    try {
      const result = await api.getAdapters()
      setData(result)
    } catch (e) {
      setError(e.message)
    } finally {
      setLoading(false)
    }
  }

  useEffect(() => { refresh() }, [])

  const adapters      = data?.adapters ?? []
  const limbSlots     = data?.limb_slots ?? {}
  const jointCount    = data?.joints_per_limb ?? {}
  const udevExists    = data?.udev_rules_exist ?? false
  const udevContent   = data?.udev_rules_content ?? ''

  const unassigned    = adapters.filter(a => !a.assigned_limb)
  const adapterByLimb = Object.fromEntries(
    adapters
      .filter(a => a.assigned_limb)
      .map(a => [a.assigned_limb, a])
  )

  const healthByName  = Object.fromEntries((canHealth ?? []).map(i => [i.name, i]))

  const takenLimbs     = new Set(Object.keys(adapterByLimb))
  const availableLimbs = ['left_leg', 'right_leg', 'left_arm', 'right_arm'].filter(
    l => !takenLimbs.has(l)
  )

  async function handleAssign(usb_serial, limb) {
    await api.assignAdapter(usb_serial, limb)
    await refresh()
  }

  async function handleUnassign(usb_serial) {
    await api.unassignAdapter(usb_serial)
    await refresh()
  }

  async function handleBringUp(name) {
    await api.bringUpInterface(name)
    await refresh()
  }

  return (
    <div className="h-full flex flex-col overflow-hidden">
      {/* Header */}
      <div className="flex-shrink-0 px-6 py-4 border-b border-surface-3 flex items-center justify-between">
        <div>
          <h1 className="font-semibold text-base">CAN Adapter Setup</h1>
          <p className="text-xs text-gray-500 mt-0.5">Assign USB-CAN adapters to robot limbs</p>
        </div>
        <button
          onClick={refresh}
          disabled={loading}
          className="flex items-center gap-1.5 px-3 py-1.5 rounded-lg text-xs text-gray-400 hover:text-gray-200 hover:bg-white/5 transition-colors disabled:opacity-50"
        >
          <svg className={`w-3.5 h-3.5 ${loading ? 'animate-spin' : ''}`} fill="none" viewBox="0 0 24 24" stroke="currentColor" strokeWidth={2}>
            <path strokeLinecap="round" strokeLinejoin="round" d="M4 4v5h.582m15.356 2A8.001 8.001 0 004.582 9m0 0H9m11 11v-5h-.581m0 0a8.003 8.003 0 01-15.357-2m15.357 2H15" />
          </svg>
          Refresh
        </button>
      </div>

      <div className="flex-1 overflow-y-auto">
        {/* Status bar */}
        {data && (
          <StatusBar
            adapters={adapters}
            canHealth={canHealth}
          />
        )}

        {error && (
          <div className="mx-6 mt-4 rounded-lg bg-danger/10 border border-danger/20 px-4 py-2 text-sm text-danger">
            {error}
          </div>
        )}

        {/* ── Section 1: Limb Slots ── */}
        <div className="px-6 pt-4 pb-4">
          <p className="text-xs text-gray-500 font-medium mb-3 uppercase tracking-wide">
            Limb Assignment Slots
          </p>
          <div className="grid grid-cols-2 gap-4">
            {LIMB_META.map(meta => (
              <LimbSlotCard
                key={meta.limb}
                meta={meta}
                adapter={adapterByLimb[meta.limb] ?? null}
                iface={healthByName[meta.busName] ?? null}
                jointCount={jointCount[meta.limb] ?? 0}
                unassigned={unassigned}
                onAssign={handleAssign}
                onUnassign={handleUnassign}
                onBringUp={handleBringUp}
                onRefresh={refresh}
              />
            ))}
          </div>
        </div>

        {/* ── Section 2: Unassigned adapters ── */}
        {unassigned.length > 0 && (
          <div className="px-6 pb-4">
            <div className="flex items-center gap-2 mb-3">
              <svg className="w-4 h-4 text-warn" fill="none" viewBox="0 0 24 24" stroke="currentColor" strokeWidth={2}>
                <path strokeLinecap="round" strokeLinejoin="round" d="M12 9v2m0 4h.01m-6.938 4h13.856c1.54 0 2.502-1.667 1.732-3L13.732 4c-.77-1.333-2.694-1.333-3.464 0L3.34 16c-.77 1.333.192 3 1.732 3z" />
              </svg>
              <p className="text-xs text-warn font-medium uppercase tracking-wide">
                Unassigned Adapters ({unassigned.length})
              </p>
            </div>
            <div className="space-y-3">
              {unassigned.map(adapter => (
                <UnassignedAdapterCard
                  key={adapter.usb_serial ?? adapter.current_name}
                  adapter={adapter}
                  availableLimbs={availableLimbs}
                  onAssign={handleAssign}
                  onBringUp={handleBringUp}
                />
              ))}
            </div>
          </div>
        )}

        {/* ── Section 3: UDEV rules status ── */}
        <div className="px-6 pb-6">
          <UdevStatus exists={udevExists} content={udevContent} />
        </div>
      </div>
    </div>
  )
}
