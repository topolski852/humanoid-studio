import { useEffect, useRef, useState } from 'react'
import { api } from '../api'
import {
  R2D, ACTIVE_MODES, SectionLabel, MetricRow,
  PowerControls, TuneChart, ConfirmModal,
} from './tunePanelKit'

// Concatenate the diagnosis phases (move → ramp_up → ramp_down) into one
// continuous trace for the chart, offsetting each phase's t_ms.
function flattenPhases(byPhase) {
  if (!byPhase) return []
  let out = []
  let offset = 0
  for (const k of ['move', 'ramp_up', 'ramp_down']) {
    const ph = byPhase[k]
    if (!ph || !ph.length) continue
    for (const s of ph) out.push({ ...s, t_ms: offset + s.t_ms })
    offset += ph[ph.length - 1].t_ms + 200
  }
  return out
}

const SEVERITY = {
  MOVES_OK:             { cls: 'bg-online/10 text-online border-online/40',  dot: 'bg-online',  label: 'Moves OK' },
  GRAVITY_LOAD:         { cls: 'bg-accent/10 text-accent border-accent/40',  dot: 'bg-accent',  label: 'Gravity / Load' },
  KP_STARVED:           { cls: 'bg-warn/10 text-warn border-warn/40',        dot: 'bg-warn',    label: 'Kp Too Low' },
  STUCK_TORQUE_STARVED: { cls: 'bg-warn/10 text-warn border-warn/40',        dot: 'bg-warn',    label: 'Torque-Starved' },
  COMMUTATION_FAULT:    { cls: 'bg-danger/10 text-danger border-danger/40',  dot: 'bg-danger',  label: 'Commutation Fault' },
  RUNAWAY:              { cls: 'bg-danger/10 text-danger border-danger/40',  dot: 'bg-danger',  label: 'Runaway' },
  NO_ROOM:              { cls: 'bg-surface-2 text-gray-400 border-surface-3', dot: 'bg-gray-500', label: 'No Room' },
}

export default function DiagnosePanel({ jointName, state, config, onLogError }) {
  const [running, setRunning] = useState(false)
  const [result, setResult] = useState(null)
  const [err, setErr] = useState(null)
  const [confirmAction, setConfirmAction] = useState(null)  // 'phase' | 'torque' | 'kp' | 'flash'
  const [remediating, setRemediating] = useState(false)
  const [remediation, setRemediation] = useState(null)      // phase before/after result
  const [persisted, setPersisted] = useState(false)
  const abortRef = useRef(null)

  useEffect(() => () => abortRef.current?.abort(), [])

  const isConnected = state != null
  const motorEnabled = state?.mode_name === 'POSITION'
  const rec = result?.recommendation
  const sev = SEVERITY[result?.classification] ?? SEVERITY.NO_ROOM

  async function runDiagnosis() {
    abortRef.current?.abort()
    abortRef.current = new AbortController()
    setRunning(true); setErr(null); setResult(null); setRemediation(null); setPersisted(false)
    try {
      const res = await api.runDiagnosis(jointName, {}, abortRef.current.signal)
      setResult(res)
    } catch (e) {
      if (e.name === 'AbortError') return
      const msg = `Diagnosis failed: ${e.message}`
      setErr(msg); onLogError?.(msg, 'Diagnose')
    }
    setRunning(false)
  }

  async function doRemediation() {
    setRemediating(true); setErr(null)
    try {
      if (confirmAction === 'phase') {
        const r = await api.remediatePhase(jointName)
        setRemediation(r)
      } else if (confirmAction === 'torque') {
        await api.raiseTorqueLimit(jointName, rec.params.torque_limit)
      } else if (confirmAction === 'kp') {
        await api.writeMotorGains(
          jointName, rec.params.position_kp,
          config?.position_ki ?? 0, config?.velocity_kp ?? 1, config?.torque_limit ?? 2,
        )
      } else if (confirmAction === 'flash') {
        await api.storeMotorToFlash(jointName)
        setPersisted(true)
      }
    } catch (e) {
      const msg = `Remediation failed: ${e.message}`
      setErr(msg); onLogError?.(msg, 'Diagnose')
    }
    setRemediating(false)
    setConfirmAction(null)
  }

  const confirmCopy = {
    phase: {
      title: 'Flip phase order + recalibrate',
      body: 'This flips phase_inverted AND runs a ~90 s flux recalibration — the motor WILL spin through its range. Keep clear of the joint. The new offset is saved to config (not flash yet).',
      label: 'Flip & Recalibrate', danger: true,
    },
    torque: {
      title: 'Raise torque limit',
      body: `Write torque_limit = ${rec?.params?.torque_limit} Nm to the ESC (RAM). The joint may move more forcefully.`,
      label: 'Raise Torque', danger: false,
    },
    kp: {
      title: 'Raise Kp',
      body: `Write position_kp = ${rec?.params?.position_kp} to the ESC (RAM). Stiffer response — keep clear.`,
      label: 'Raise Kp', danger: false,
    },
    flash: {
      title: 'Persist to flash',
      body: 'Store the current ESC config (including the new phase + flux offset) to flash so it survives a power cycle.',
      label: 'Store to Flash', danger: false,
    },
  }[confirmAction] ?? {}

  return (
    <div className="flex-1 min-w-0 overflow-y-auto p-3 space-y-4">
      <div className="px-3 py-2.5 rounded-lg bg-warn/10 border border-warn/30">
        <p className="text-xs font-semibold text-warn">⚠ Diagnosis moves the joint</p>
        <p className="text-[10px] text-warn/80 mt-1 leading-relaxed">
          Runs a small guarded step and slow ramp under a low torque limit, with an auto-abort if
          the joint runs away. Keep clear of the range of motion. Enable the motor in POSITION mode first.
        </p>
      </div>

      <PowerControls jointName={jointName} state={state} onLogError={onLogError} />

      <button
        onClick={runDiagnosis}
        disabled={running || !motorEnabled}
        className="w-full py-2.5 rounded-lg text-sm font-semibold transition-colors
          bg-accent/20 text-accent border border-accent/30 hover:bg-accent/30 disabled:opacity-40"
      >
        {running ? 'Diagnosing… (move → torque → commutation ramp)' : 'Run Diagnosis'}
      </button>
      {!motorEnabled && (
        <p className="text-[10px] text-gray-600 text-center -mt-2">Enable the motor in POSITION mode to diagnose.</p>
      )}
      {err && <p className="text-[10px] text-danger px-2">{err}</p>}

      {result && (
        <>
          <div className={`rounded-xl border p-3 space-y-3 bg-surface-2 ${sev.cls.split(' ').slice(-1)}`}>
            <div className="flex items-center justify-between">
              <SectionLabel>Diagnosis</SectionLabel>
              <span className={`flex items-center gap-1.5 text-[10px] font-medium px-2 py-0.5 rounded border ${sev.cls}`}>
                <span className={`w-1.5 h-1.5 rounded-full ${sev.dot}`} />
                {sev.label}
              </span>
            </div>

            <ul className="space-y-1">
              {(result.rationale ?? []).map((r, i) => (
                <li key={i} className="text-[10px] text-gray-400 flex gap-1.5">
                  <span className="text-gray-600 flex-shrink-0">•</span>{r}
                </li>
              ))}
            </ul>

            {result.flags?.includes('STICK_SLIP') && (
              <p className="text-[10px] text-warn">⚠ Stick-slip / chatter detected (velocity reversals).</p>
            )}

            {/* Evidence */}
            <div className="grid grid-cols-[1fr_auto] gap-x-3 gap-y-0.5 text-[10px] pt-1 border-t border-surface-3">
              <MetricRow label="Move travel" value={`${((result.evidence?.move_motion_rad ?? 0) * R2D).toFixed(1)}°`} />
              <MetricRow label="Move current (max)" value={`${result.evidence?.move_current?.max_current_a?.toFixed(2) ?? '—'} A`} />
              <MetricRow label="Velocity reversals" value={`${result.evidence?.velocity_reversals ?? '—'}`} />
              <MetricRow label="Torque chatter (pp)" value={`${result.evidence?.torque_chatter_pp_nm?.toFixed(2) ?? '—'} Nm`} />
              <MetricRow label="Torque saturated" value={result.evidence?.torque_saturated ? 'yes' : 'no'} warn={result.evidence?.torque_saturated} />
              {result.evidence?.ramp_current && Object.entries(result.evidence.ramp_current).map(([dir, cm]) => (
                <MetricRow key={dir} label={`Ramp ${dir} current`} value={`${cm.max_current_a?.toFixed(2)} A / ${(cm.motion_range_rad * R2D).toFixed(1)}°`} />
              ))}
            </div>
          </div>

          <TuneChart samples={flattenPhases(result.samples_by_phase)} />

          {/* Remediation */}
          {rec && rec.action !== 'none' && !remediation && (
            <div className="rounded-xl border border-surface-3 bg-surface-2 p-3 space-y-2">
              <SectionLabel>Recommended fix</SectionLabel>
              <p className="text-[11px] text-gray-300">{rec.confirm_label}</p>
              {(rec.rationale ?? []).map((r, i) => (
                <p key={i} className="text-[10px] text-gray-500">{r}</p>
              ))}
              <button
                onClick={() => setConfirmAction(
                  rec.action === 'phase_flip_and_recal' ? 'phase'
                  : rec.action === 'raise_torque_limit' ? 'torque'
                  : rec.action === 'raise_kp' ? 'kp' : null,
                )}
                className={`w-full py-2 rounded-lg text-xs font-semibold border transition-colors ${
                  rec.destructive
                    ? 'bg-danger/20 text-danger border-danger/30 hover:bg-danger/30'
                    : 'bg-accent/20 text-accent border-accent/30 hover:bg-accent/30'
                }`}
              >
                {rec.confirm_label}
              </button>
            </div>
          )}

          {rec && rec.action === 'none' && (
            <p className="text-[10px] text-gray-500 px-2">{rec.rationale?.[0]}</p>
          )}

          {/* Phase remediation result + persist */}
          {remediation && (
            <div className="rounded-xl border border-online/40 bg-surface-2 p-3 space-y-2">
              <SectionLabel>Phase remediation result</SectionLabel>
              <div className="grid grid-cols-[1fr_auto] gap-x-3 gap-y-0.5 text-[10px]">
                <MetricRow label="phase_inverted" value={String(remediation.phase_inverted)} />
                <MetricRow label="flux before → after" value={`${remediation.flux_before?.toFixed(2)} → ${remediation.flux_after?.toFixed(2)}`} />
                <MetricRow label="recheck" value={remediation.recheck?.classification} warn={remediation.recheck?.classification !== 'MOVES_OK'} />
                <MetricRow label="recheck current" value={`${remediation.recheck?.evidence?.move_current?.max_current_a?.toFixed(2) ?? '—'} A`} />
              </div>
              {remediation.recheck?.classification === 'MOVES_OK'
                ? <p className="text-[10px] text-online">✓ Joint now commutates and tracks. Persist to flash to keep it.</p>
                : <p className="text-[10px] text-warn">Recheck still not clean — may need the opposite phase or a mechanical check.</p>}
              <button
                onClick={() => setConfirmAction('flash')}
                disabled={persisted}
                className="w-full py-2 rounded-lg text-xs font-semibold bg-accent/20 text-accent
                  border border-accent/30 hover:bg-accent/30 disabled:opacity-40"
              >
                {persisted ? '✓ Stored to flash' : 'Persist to Flash'}
              </button>
            </div>
          )}
        </>
      )}

      <ConfirmModal
        open={confirmAction != null}
        title={confirmCopy.title}
        body={confirmCopy.body}
        confirmLabel={confirmCopy.label}
        danger={confirmCopy.danger}
        busy={remediating}
        onCancel={() => setConfirmAction(null)}
        onConfirm={doRemediation}
      />
    </div>
  )
}
