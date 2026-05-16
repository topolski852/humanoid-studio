import { useEffect, useState } from 'react'
import { api } from '../api'
import FlashWizard from '../components/FlashWizard'

// ── USB device status bar ─────────────────────────────────────────────────────
function UsbBar({ devices }) {
  if (devices.length === 0) {
    return (
      <div className="flex items-center gap-2 px-4 py-2.5 rounded-lg bg-surface-2 border border-surface-3 text-xs text-gray-500">
        <span className="w-2 h-2 rounded-full bg-gray-600 flex-shrink-0" />
        No flash devices detected — connect ST-LINK programmer via USB
      </div>
    )
  }
  return (
    <div className="space-y-1.5">
      {devices.map((d) => (
        <div
          key={`${d.bus}-${d.device}`}
          className="flex items-center gap-3 px-4 py-2.5 rounded-lg bg-surface-2 border border-surface-3"
        >
          <span
            className={`w-2 h-2 rounded-full flex-shrink-0 ${
              d.type === 'stlink' ? 'bg-online' : 'bg-warn animate-pulse'
            }`}
          />
          <span className="text-sm text-gray-200 flex-1">{d.label}</span>
          <span className="text-[10px] font-mono text-gray-600">{d.vid_pid}</span>
        </div>
      ))}
    </div>
  )
}

// ── Pill toggle buttons ───────────────────────────────────────────────────────
function PillGroup({ options, value, onChange }) {
  return (
    <div className="flex gap-2">
      {options.map((opt) => (
        <button
          key={opt.value}
          onClick={() => onChange(opt.value === value ? null : opt.value)}
          className={`px-5 py-2 rounded-lg text-sm font-medium transition-colors border ${
            value === opt.value
              ? 'bg-accent/20 text-accent border-accent/40'
              : 'bg-surface-2 text-gray-400 border-surface-3 hover:text-gray-200 hover:border-accent/20'
          }`}
        >
          {opt.label}
        </button>
      ))}
    </div>
  )
}

// ── Motor list for selected limb ──────────────────────────────────────────────
function MotorList({ joints, onSelect }) {
  if (joints.length === 0) {
    return <p className="text-xs text-gray-600">No motors found for this limb.</p>
  }
  return (
    <div className="space-y-1.5">
      {joints.map((j) => {
        const label = j.joint_name
          .replace(/_joint$/, '')
          .replace(/_/g, ' ')
          .replace(/\b\w/g, (c) => c.toUpperCase())
        return (
          <button
            key={j.joint_name}
            onClick={() => onSelect(j.joint_name)}
            className="w-full flex items-center gap-4 px-4 py-3 rounded-lg bg-surface-2 border border-surface-3 hover:border-accent/40 hover:bg-accent/5 transition-colors text-left group"
          >
            <div className="flex-1">
              <div className="text-sm text-gray-200 group-hover:text-white">{label}</div>
              <div className="text-[10px] text-gray-600 font-mono mt-0.5">{j.joint_name}</div>
            </div>
            <div className="text-right flex-shrink-0">
              <div className="text-xs font-mono text-gray-400">CAN ID {j.can_id}</div>
              <div className="text-[10px] text-gray-600">{j.can_channel ?? 'can0'}</div>
            </div>
            <svg className="w-4 h-4 text-gray-600 group-hover:text-accent flex-shrink-0" fill="none" viewBox="0 0 24 24" stroke="currentColor" strokeWidth={2}>
              <path strokeLinecap="round" strokeLinejoin="round" d="M9 5l7 7-7 7" />
            </svg>
          </button>
        )
      })}
    </div>
  )
}

// Derive side and limb type from can_channel, e.g. "can_right_leg" → {side:'right', type:'leg'}
function parseLimb(can_channel) {
  if (!can_channel) return { side: null, type: null }
  const m = can_channel.match(/can_(left|right)_(leg|arm)/)
  return m ? { side: m[1], type: m[2] } : { side: null, type: null }
}

// ── Main page ─────────────────────────────────────────────────────────────────
export default function EscSetup() {
  const [usbDevices, setUsbDevices]     = useState([])
  const [joints, setJoints]             = useState([])
  const [side, setSide]                 = useState(null)   // 'left' | 'right'
  const [limbType, setLimbType]         = useState(null)   // 'leg' | 'arm'
  const [selectedName, setSelectedName] = useState(null)

  // Poll USB devices every 3 s
  useEffect(() => {
    async function refreshUsb() {
      try {
        const result = await api.getUsbDevices()
        setUsbDevices(result?.devices ?? [])
      } catch { /* non-fatal */ }
    }
    refreshUsb()
    const id = setInterval(refreshUsb, 3000)
    return () => clearInterval(id)
  }, [])

  // Load joint list once
  useEffect(() => {
    api.getRobotConfig()
      .then((config) => {
        if (!config?.joints) return
        const sorted = Object.values(config.joints).sort((a, b) => a.can_id - b.can_id)
        setJoints(sorted)
      })
      .catch(() => {})
  }, [])

  function handleSide(s) {
    setSide(s)
    setLimbType(null)  // reset downstream
  }

  function handleLimbType(t) {
    setLimbType(t)
  }

  const filteredJoints = joints.filter((j) => {
    if (!side || !limbType) return false
    const { side: s, type: t } = parseLimb(j.can_channel)
    return s === side && t === limbType
  })

  const selectedJoint = joints.find((j) => j.joint_name === selectedName) ?? null

  return (
    <div className="h-full flex flex-col overflow-hidden">
      {/* Header */}
      <div className="flex-shrink-0 px-6 py-4 border-b border-surface-3">
        <h1 className="font-semibold text-base">ESC Setup</h1>
        <p className="text-xs text-gray-500 mt-0.5">Flash and commission ESC controllers via ST-LINK</p>
      </div>

      <div className="flex-1 overflow-y-auto">
        {selectedJoint ? (
          /* ── Flash Wizard ── */
          <FlashWizard
            canId={selectedJoint.can_id}
            canChannel={selectedJoint.can_channel ?? 'can0'}
            jointName={selectedJoint.joint_name}
            onClose={() => setSelectedName(null)}
          />
        ) : (
          /* ── Motor picker ── */
          <div className="px-6 pt-6 space-y-6 max-w-2xl">
            {/* USB Devices */}
            <div className="space-y-2">
              <p className="data-label">USB Flash Devices</p>
              <UsbBar devices={usbDevices} />
            </div>

            {/* Step 1: Side */}
            <div className="space-y-2">
              <p className="data-label">Step 1 — Select side</p>
              <PillGroup
                options={[
                  { label: 'Left',  value: 'left'  },
                  { label: 'Right', value: 'right' },
                ]}
                value={side}
                onChange={handleSide}
              />
            </div>

            {/* Step 2: Limb type — only shown after side is picked */}
            {side && (
              <div className="space-y-2">
                <p className="data-label">Step 2 — Select limb</p>
                <PillGroup
                  options={[
                    { label: 'Leg', value: 'leg' },
                    { label: 'Arm', value: 'arm' },
                  ]}
                  value={limbType}
                  onChange={handleLimbType}
                />
              </div>
            )}

            {/* Step 3: Motor list — only shown after both are picked */}
            {side && limbType && (
              <div className="space-y-2">
                <p className="data-label">
                  Step 3 — Select motor
                  <span className="ml-2 normal-case text-gray-600 font-normal">
                    {side} {limbType}
                  </span>
                </p>
                <MotorList joints={filteredJoints} onSelect={setSelectedName} />
              </div>
            )}

            {joints.length === 0 && (
              <p className="text-xs text-gray-600">
                No joints loaded — check that the backend is running and a robot config is selected.
              </p>
            )}
          </div>
        )}
      </div>
    </div>
  )
}
