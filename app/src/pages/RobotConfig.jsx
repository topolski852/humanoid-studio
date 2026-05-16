import { useEffect, useState } from 'react'
import { api } from '../api'

function Field({ label, value, mono = false }) {
  return (
    <div className="flex items-center justify-between py-2 border-b border-surface-3 last:border-0">
      <span className="text-xs text-gray-500">{label}</span>
      <span className={`text-xs ${mono ? 'font-mono text-gray-200' : 'text-gray-300'}`}>{String(value)}</span>
    </div>
  )
}

function JointRow({ joint, state }) {
  const [expanded, setExpanded] = useState(false)
  const online = state != null

  return (
    <div className="card mb-2 overflow-hidden">
      <button
        className="w-full flex items-center gap-3 px-4 py-3 hover:bg-surface-2 transition-colors"
        onClick={() => setExpanded((v) => !v)}
      >
        <span className={`w-2 h-2 rounded-full flex-shrink-0 ${online ? 'bg-online' : 'bg-offline'}`} />
        <span className="flex-1 text-sm text-left font-medium">{joint.joint_name}</span>
        <span className="font-mono text-[10px] text-gray-500 mr-2">ID {joint.can_id}</span>
        <span className="font-mono text-[10px] text-gray-600">{joint.can_channel}</span>
        <svg
          className={`w-4 h-4 text-gray-500 ml-2 transition-transform ${expanded ? 'rotate-180' : ''}`}
          fill="none" viewBox="0 0 24 24" stroke="currentColor" strokeWidth={2}
        >
          <path strokeLinecap="round" strokeLinejoin="round" d="M19 9l-7 7-7-7" />
        </svg>
      </button>

      {expanded && (
        <div className="px-4 pb-4 grid grid-cols-2 gap-x-8 bg-surface-2">
          <div>
            <Field label="Phase Inverted" value={joint.phase_inverted ? 'Yes' : 'No'} mono />
            <Field label="Gear Ratio" value={joint.gear_ratio} mono />
            <Field label="Electrical Offset" value={joint.electrical_offset?.toFixed(4)} mono />
            <Field label="Position Min" value={joint.position_limits?.min ?? '−∞'} mono />
            <Field label="Position Max" value={joint.position_limits?.max ?? '+∞'} mono />
            <Field label="Torque Limit" value={`${joint.torque_limit} Nm`} mono />
            <Field label="Velocity Limit" value={`${joint.velocity_limit} rad/s`} mono />
          </div>
          <div>
            <Field label="Position Kp" value={joint.position_kp} mono />
            <Field label="Velocity Kp" value={joint.velocity_kp} mono />
            <Field label="Current Limit" value={`${joint.current_limit} A`} mono />
            <Field label="Current Kp" value={joint.current_kp} mono />
            <Field label="Current Ki" value={joint.current_ki} mono />
            <Field label="Pole Pairs" value={joint.pole_pairs} mono />
            <Field label="Torque Constant" value={`${joint.torque_constant} Nm/A`} mono />
          </div>
        </div>
      )}
    </div>
  )
}

export default function RobotConfig() {
  const [config, setConfig] = useState(null)
  const [loading, setLoading] = useState(true)
  const [error, setError] = useState(null)
  const [saving, setSaving] = useState(false)
  const [saveResult, setSaveResult] = useState(null)
  const [editJson, setEditJson] = useState(false)
  const [jsonText, setJsonText] = useState('')
  const [jsonError, setJsonError] = useState(null)

  function load() {
    setLoading(true)
    api
      .getRobotConfig()
      .then((c) => {
        setConfig(c)
        setJsonText(JSON.stringify(c, null, 2))
        setError(null)
      })
      .catch((e) => setError(e.message))
      .finally(() => setLoading(false))
  }

  useEffect(load, [])

  async function handleSave() {
    setJsonError(null)
    let parsed
    try {
      parsed = JSON.parse(jsonText)
    } catch (e) {
      setJsonError(`Invalid JSON: ${e.message}`)
      return
    }
    setSaving(true)
    setSaveResult(null)
    try {
      await api.putRobotConfig(parsed)
      setSaveResult({ ok: true, msg: 'Config saved and reloaded' })
      load()
    } catch (e) {
      setSaveResult({ ok: false, msg: e.message })
    }
    setSaving(false)
  }

  const joints = config?.joints ? Object.values(config.joints) : []

  return (
    <div className="h-full flex flex-col overflow-hidden">
      {/* Header */}
      <div className="flex-shrink-0 px-6 py-4 border-b border-surface-3 flex items-center justify-between">
        <div>
          <h1 className="font-semibold text-base">Robot Config</h1>
          {config && (
            <p className="text-xs text-gray-500 mt-0.5">
              {config.robot_name} · {joints.length} joints
            </p>
          )}
        </div>
        <div className="flex gap-2">
          <button
            onClick={() => setEditJson((v) => !v)}
            className="btn-ghost px-3 py-1.5 text-xs"
          >
            {editJson ? 'Table View' : 'JSON Editor'}
          </button>
          {editJson && (
            <button onClick={handleSave} disabled={saving} className="btn-primary px-3 py-1.5 text-xs">
              {saving ? 'Saving…' : 'Save'}
            </button>
          )}
        </div>
      </div>

      {/* Content */}
      <div className="flex-1 overflow-y-auto p-6">
        {loading ? (
          <div className="flex justify-center items-center h-32">
            <div className="w-6 h-6 rounded-full border-2 border-accent border-t-transparent animate-spin" />
          </div>
        ) : error ? (
          <p className="text-sm text-danger">{error}</p>
        ) : editJson ? (
          <div className="space-y-2">
            {jsonError && <p className="text-xs text-danger font-mono">{jsonError}</p>}
            {saveResult && (
              <p className={`text-xs ${saveResult.ok ? 'text-online' : 'text-danger'}`}>
                {saveResult.msg}
              </p>
            )}
            <textarea
              value={jsonText}
              onChange={(e) => setJsonText(e.target.value)}
              className="w-full h-[calc(100vh-16rem)] font-mono text-xs bg-surface border border-surface-3 rounded-lg p-4 text-gray-300 resize-none focus:outline-none focus:border-accent"
              spellCheck={false}
            />
          </div>
        ) : (
          joints.map((joint) => (
            <JointRow key={joint.joint_name} joint={joint} state={null} />
          ))
        )}
      </div>
    </div>
  )
}
