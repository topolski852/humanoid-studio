import { useEffect, useRef, useState } from 'react'
import { api } from '../api'
import {
  DEG, R2D, SectionLabel, Field, MetricRow, PowerControls, ConfirmModal,
} from './tunePanelKit'

export default function GravityTunePanel({ jointName, state, config, onLogError }) {
  const [kp, setKp]               = useState('20')
  const [ki, setKi]               = useState('0')
  const [kdList, setKdList]       = useState('0.5, 1, 2, 4, 8')
  const [liftDeg, setLiftDeg]     = useState('15')
  const [liftSign, setLiftSign]   = useState(1)        // +1 = lift Up, -1 = lift Down
  const [torqueLimit, setTorqueLimit] = useState('6')
  const [testKi, setTestKi]       = useState(false)
  const [running, setRunning]     = useState(false)
  const [result, setResult]       = useState(null)
  const [err, setErr]             = useState(null)
  const [confirmAction, setConfirmAction] = useState(null)  // 'apply' | 'persist'
  const [busy, setBusy]           = useState(false)
  const cfgInit = useRef(false)
  const abortRef = useRef(null)

  useEffect(() => () => abortRef.current?.abort(), [])

  useEffect(() => {
    if (cfgInit.current || !config) return
    cfgInit.current = true
    if (config.position_kp  != null) setKp(String(config.position_kp))
    if (config.position_ki  != null) setKi(String(config.position_ki))
    if (config.torque_limit != null) setTorqueLimit(String(config.torque_limit))
  }, [config])

  const motorEnabled = state?.mode_name === 'POSITION'
  const rec = result?.recommended

  async function runSweep() {
    abortRef.current?.abort()
    abortRef.current = new AbortController()
    setRunning(true); setErr(null); setResult(null)
    const kd_values = kdList.split(',').map((s) => parseFloat(s.trim())).filter((n) => !isNaN(n) && n > 0)
    try {
      const res = await api.runGravityTune(jointName, {
        kp: parseFloat(kp) || 0,
        ki: parseFloat(ki) || 0,
        kd_values,
        lift_rad: (parseFloat(liftDeg) || 0) * DEG,
        lift_sign: liftSign,
        torque_limit: parseFloat(torqueLimit) || 0,
        test_ki: testKi,
      }, abortRef.current.signal)
      setResult(res)
    } catch (e) {
      if (e.name === 'AbortError') return
      const msg = `Gravity sweep failed: ${e.message}`
      setErr(msg); onLogError?.(msg, 'GravityTune')
    }
    setRunning(false)
  }

  async function doApply() {
    setBusy(true); setErr(null)
    try {
      const tl = parseFloat(torqueLimit) || 0
      if (confirmAction === 'apply') {
        await api.writeMotorGains(jointName, rec.kp, rec.ki, rec.kd, tl)
      } else if (confirmAction === 'persist') {
        await api.applyMotorConfig(jointName, {
          position_kp: rec.kp, position_ki: rec.ki, velocity_kp: rec.kd, torque_limit: tl,
        })
        await api.storeMotorToFlash(jointName)
      }
    } catch (e) {
      const msg = `Apply failed: ${e.message}`
      setErr(msg); onLogError?.(msg, 'GravityTune')
    }
    setBusy(false); setConfirmAction(null)
  }

  const confirmCopy = {
    apply: {
      title: 'Apply recommended gains',
      body: `Write Kp=${rec?.kp}, Kd=${rec?.kd}, Ki=${rec?.ki} to the ESC (RAM only). Keep clear of the joint.`,
      label: 'Apply to ESC',
    },
    persist: {
      title: 'Apply + persist to flash',
      body: `Write Kp=${rec?.kp}, Kd=${rec?.kd}, Ki=${rec?.ki} to config + ESC and store to flash (survives power cycle).`,
      label: 'Apply & Store',
    },
  }[confirmAction] ?? {}

  return (
    <div className="flex-1 min-w-0 overflow-y-auto p-3 space-y-4">
      <div className="px-3 py-2.5 rounded-lg bg-warn/10 border border-warn/30">
        <p className="text-xs font-semibold text-warn">⚠ Gravity Tune lifts &amp; drops the joint</p>
        <p className="text-[10px] text-warn/80 mt-1 leading-relaxed">
          For each Kd it lifts the joint against gravity, holds, then drops it — measuring the descent
          "slam" and the hold droop. Set the lift direction so "Up" opposes gravity. Auto-aborts on runaway.
        </p>
      </div>

      <PowerControls jointName={jointName} state={state} onLogError={onLogError} />

      <section className="space-y-1.5">
        <SectionLabel>Sweep settings</SectionLabel>
        <div className="grid grid-cols-2 gap-2">
          <Field label="Position Kp" value={kp} onChange={setKp} />
          <Field label="Torque Limit (Nm)" value={torqueLimit} onChange={setTorqueLimit} />
          <Field label="Position Ki (held 0)" value={ki} onChange={setKi} />
          <Field label="Lift angle (°)" value={liftDeg} onChange={setLiftDeg} />
        </div>
        <Field label="Kd sweep values (comma-separated)" value={kdList} onChange={setKdList} />
        <div className="flex items-center justify-between pt-1">
          <span className="text-[10px] text-gray-500">Lift direction (against gravity)</span>
          <div className="flex gap-1">
            {[{ v: 1, l: '▲ Up' }, { v: -1, l: '▼ Down' }].map(({ v, l }) => (
              <button
                key={v}
                onClick={() => setLiftSign(v)}
                className={`px-2.5 py-1 rounded text-[11px] border transition-colors ${
                  liftSign === v ? 'bg-accent/20 text-accent border-accent/40' : 'bg-surface-2 text-gray-500 border-surface-3'
                }`}
              >
                {l}
              </button>
            ))}
          </div>
        </div>
        <label className="flex items-center gap-2 text-[10px] text-gray-500 pt-1">
          <input type="checkbox" checked={testKi} onChange={(e) => setTestKi(e.target.checked)} />
          Probe Ki windup (one extra cycle with Ki — usually worsens the slam)
        </label>
      </section>

      <button
        onClick={runSweep}
        disabled={running || !motorEnabled}
        className="w-full py-2.5 rounded-lg text-sm font-semibold transition-colors
          bg-accent/20 text-accent border border-accent/30 hover:bg-accent/30 disabled:opacity-40"
      >
        {running ? 'Sweeping Kd… (lift / hold / drop per value)' : 'Run Gravity Sweep'}
      </button>
      {!motorEnabled && (
        <p className="text-[10px] text-gray-600 text-center -mt-2">Enable the motor in POSITION mode to sweep.</p>
      )}
      {err && <p className="text-[10px] text-danger px-2">{err}</p>}

      {result && (
        <>
          <section className="space-y-2">
            <SectionLabel>Kd sweep</SectionLabel>
            <div className="grid grid-cols-5 gap-x-2 gap-y-1 text-[10px]">
              {['Kd', 'Droop°', 'Descent °/s', 'Overshoot°', 'Max A'].map((h) => (
                <span key={h} className="text-gray-600 font-medium">{h}</span>
              ))}
              {result.sweep.map((r) => {
                const sel = r.kd === result.selected_kd
                return [
                  <span key={`${r.kd}-kd`} className={`font-mono ${sel ? 'text-accent font-bold' : 'text-gray-300'}`}>{r.kd}{sel ? ' ◀' : ''}</span>,
                  <span key={`${r.kd}-d`} className="font-mono text-gray-300">{(r.droop_rad * R2D).toFixed(1)}</span>,
                  <span key={`${r.kd}-v`} className="font-mono text-gray-300">{(r.descent_peak_velocity * R2D).toFixed(0)}</span>,
                  <span key={`${r.kd}-o`} className="font-mono text-gray-300">{(r.descent_overshoot_rad * R2D).toFixed(1)}</span>,
                  <span key={`${r.kd}-a`} className={`font-mono ${r.torque_saturated ? 'text-amber-400' : 'text-gray-300'}`}>{r.max_current_a.toFixed(1)}</span>,
                ]
              })}
            </div>
          </section>

          {rec && (
            <div className="rounded-xl border border-online/40 bg-surface-2 p-3 space-y-3">
              <SectionLabel>Recommended gains</SectionLabel>
              <ul className="space-y-0.5">
                {(result.rationale ?? []).map((r, i) => (
                  <li key={i} className="text-[10px] text-gray-400 flex gap-1.5">
                    <span className="text-gray-600 flex-shrink-0">•</span>{r}
                  </li>
                ))}
              </ul>
              {result.windup?.detected && (
                <p className="text-[10px] text-warn px-2 py-1 bg-warn/10 rounded border border-warn/30">
                  ⚠ Ki windup detected (+{result.windup.delta_pct}% descent velocity) — keeping Ki = 0.
                </p>
              )}
              <div className="grid grid-cols-3 gap-2 text-center">
                {[{ l: 'Kp', v: rec.kp }, { l: 'Kd', v: rec.kd }, { l: 'Ki', v: rec.ki }].map(({ l, v }) => (
                  <div key={l} className="bg-surface-1 rounded-lg py-1.5 px-2">
                    <p className="text-[9px] text-gray-600">{l}</p>
                    <p className="font-mono text-sm text-gray-200">{v}</p>
                  </div>
                ))}
              </div>
              <p className="text-[9px] text-gray-600">Residual hold droop ≈ {result.residual_droop_deg}° (gains alone can't remove it).</p>
              <div className="flex gap-2">
                <button
                  onClick={() => setConfirmAction('apply')}
                  className="flex-1 py-1.5 rounded-lg text-xs font-medium bg-surface-1 text-gray-300
                    border border-surface-3 hover:border-accent/40 hover:text-accent"
                >
                  Apply (RAM)
                </button>
                <button
                  onClick={() => setConfirmAction('persist')}
                  className="flex-1 py-1.5 rounded-lg text-xs font-medium bg-accent/20 text-accent
                    border border-accent/30 hover:bg-accent/30"
                >
                  Apply &amp; Flash
                </button>
              </div>
            </div>
          )}
        </>
      )}

      <ConfirmModal
        open={confirmAction != null}
        title={confirmCopy.title}
        body={confirmCopy.body}
        confirmLabel={confirmCopy.label}
        danger={false}
        busy={busy}
        onCancel={() => setConfirmAction(null)}
        onConfirm={doApply}
      />
    </div>
  )
}
