import { useState, useEffect } from 'react'
import { useTelemetry } from '../context/TelemetryContext'
import { api } from '../api'

function SettingRow({ label, description, children }) {
  return (
    <div className="flex items-center justify-between py-4 border-b border-surface-3 last:border-0">
      <div>
        <p className="text-sm font-medium">{label}</p>
        {description && <p className="text-xs text-gray-500 mt-0.5">{description}</p>}
      </div>
      <div className="ml-8">{children}</div>
    </div>
  )
}

function Toggle({ checked, onChange, disabled }) {
  return (
    <button
      role="switch"
      aria-checked={checked}
      disabled={disabled}
      onClick={() => onChange(!checked)}
      className={`relative inline-flex h-5 w-9 flex-shrink-0 items-center rounded-full transition-colors
        disabled:opacity-40 ${checked ? 'bg-accent/70' : 'bg-surface-3'}`}
    >
      <span className={`inline-block h-4 w-4 transform rounded-full bg-white transition-transform
        ${checked ? 'translate-x-4' : 'translate-x-0.5'}`} />
    </button>
  )
}

export default function Settings() {
  const { wsConnected, robotConnected, states } = useTelemetry()
  const [apiUrl] = useState('http://localhost:8765')
  const [wsUrl]  = useState('ws://localhost:8765/ws/telemetry')

  const [configPath,      setConfigPath]      = useState('')
  const [configPathDraft, setConfigPathDraft] = useState('')
  const [savingPath,      setSavingPath]      = useState(false)
  const [pathMsg,         setPathMsg]         = useState(null)

  // Flash Wizard defaults
  const [commutationCheck, setCommutationCheck]       = useState(false)
  const [defaultPhaseInverted, setDefaultPhaseInverted] = useState(true)

  useEffect(() => {
    api.getSettings()
      .then((s) => {
        setConfigPath(s.config_path ?? '')
        setConfigPathDraft(s.config_path ?? '')
        setCommutationCheck(s.commutation_check ?? false)
        setDefaultPhaseInverted(s.default_phase_inverted ?? true)
      })
      .catch(() => {})
  }, [])

  async function saveFlashDefault(patch, revert) {
    try {
      await api.putSettings(patch)
    } catch {
      revert()   // roll the toggle back if the write failed
    }
  }

  async function saveConfigPath() {
    setSavingPath(true)
    setPathMsg(null)
    try {
      await api.putSettings({ config_path: configPathDraft })
      setConfigPath(configPathDraft)
      setPathMsg({ type: 'ok', text: 'Saved. Config writes go to new path immediately. Restart app to load robot config from new path.' })
    } catch (e) {
      setPathMsg({ type: 'err', text: `Save failed: ${e.message}` })
    }
    setSavingPath(false)
  }

  return (
    <div className="h-full overflow-y-auto">
      <div className="px-6 py-4 border-b border-surface-3">
        <h1 className="font-semibold text-base">Settings</h1>
      </div>

      <div className="p-6 max-w-2xl space-y-6">
        {/* Robot Config File */}
        <section>
          <h2 className="text-xs font-medium text-gray-500 uppercase tracking-widest mb-4">Robot Config</h2>
          <div className="card p-4 space-y-3">
            <div>
              <p className="text-sm font-medium">Config File Path</p>
              <p className="text-xs text-gray-500 mt-0.5">
                Where robot joint configuration is loaded from and saved to.
                Default is <span className="font-mono">configs/humanoid_lite.json</span> inside the app.
                Point this to your own file to keep config outside the app folder.
                Restart the app after changing to load joints from the new file.
              </p>
            </div>
            <input
              type="text"
              value={configPathDraft}
              onChange={(e) => setConfigPathDraft(e.target.value)}
              className="w-full font-mono text-xs px-2 py-1.5 rounded border border-surface-3
                bg-surface-2 text-gray-300 outline-none focus:border-accent/50 transition-colors"
              placeholder="/path/to/my_robot_config.json"
            />
            <div className="flex items-center gap-2">
              <button
                onClick={saveConfigPath}
                disabled={savingPath || configPathDraft === configPath}
                className="px-3 py-1 rounded text-xs font-medium bg-accent/20 text-accent
                  border border-accent/30 hover:bg-accent/30 disabled:opacity-40 transition-colors"
              >
                {savingPath ? 'Saving…' : 'Save'}
              </button>
              {configPathDraft !== configPath && (
                <button
                  onClick={() => setConfigPathDraft(configPath)}
                  className="px-3 py-1 rounded text-xs text-gray-500 hover:text-gray-300 transition-colors"
                >
                  Cancel
                </button>
              )}
              {pathMsg && (
                <p className={`text-[10px] font-mono ${pathMsg.type === 'err' ? 'text-danger' : 'text-online'}`}>
                  {pathMsg.text}
                </p>
              )}
            </div>
          </div>
        </section>

        {/* Flash Wizard */}
        <section>
          <h2 className="text-xs font-medium text-gray-500 uppercase tracking-widest mb-4">Flash Wizard</h2>
          <div className="card p-4">
            <SettingRow
              label="Run commutation check by default"
              description="The auto commutation check can flip phase order. It only works when the motor is free to spin — on an assembled/loaded joint it can misfire and corrupt phase order. Off by default; still toggleable per-run in the wizard."
            >
              <Toggle
                checked={commutationCheck}
                onChange={(v) => { setCommutationCheck(v); saveFlashDefault({ commutation_check: v }, () => setCommutationCheck(!v)) }}
              />
            </SettingRow>
            <SettingRow
              label="Default phase order: Inverted"
              description="Phase order is a hardware wiring constant (True/inverted for the standard wiring). Used as the starting value when commissioning; override per-run in the wizard."
            >
              <Toggle
                checked={defaultPhaseInverted}
                onChange={(v) => { setDefaultPhaseInverted(v); saveFlashDefault({ default_phase_inverted: v }, () => setDefaultPhaseInverted(!v)) }}
              />
            </SettingRow>
          </div>
        </section>

        {/* Connection status */}
        <section>
          <h2 className="text-xs font-medium text-gray-500 uppercase tracking-widest mb-4">Connection</h2>
          <div className="card p-4">
            <SettingRow label="API Endpoint" description="FastAPI backend">
              <span className="font-mono text-xs text-gray-400">{apiUrl}</span>
            </SettingRow>
            <SettingRow label="WebSocket" description="Telemetry stream">
              <span className="font-mono text-xs text-gray-400">{wsUrl}</span>
            </SettingRow>
            <SettingRow label="WebSocket Status">
              <span className={`text-xs font-medium ${wsConnected ? 'text-online' : 'text-danger'}`}>
                {wsConnected ? 'Connected' : 'Disconnected'}
              </span>
            </SettingRow>
            <SettingRow label="Robot Status">
              <span className={`text-xs font-medium ${robotConnected ? 'text-online' : 'text-gray-500'}`}>
                {robotConnected ? 'Online' : 'Offline'}
              </span>
            </SettingRow>
            <SettingRow label="Active Joints">
              <span className="font-mono text-xs text-gray-400">
                {Object.keys(states).length}
              </span>
            </SettingRow>
          </div>
        </section>

        {/* About */}
        <section>
          <h2 className="text-xs font-medium text-gray-500 uppercase tracking-widest mb-4">About</h2>
          <div className="card p-4">
            <SettingRow label="Humanoid Studio" description="Version 1.1.0">
              <span className="text-xs text-gray-500">Berkeley Humanoid Lite</span>
            </SettingRow>
            <SettingRow label="Robot" description="Motor controller firmware">
              <span className="text-xs text-gray-500">B-G431B-ESC1 · Recoil</span>
            </SettingRow>
          </div>
        </section>
      </div>
    </div>
  )
}
