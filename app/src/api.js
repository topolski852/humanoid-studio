const BASE = 'http://localhost:8765'

async function request(path, options = {}, signal = undefined) {
  const res = await fetch(`${BASE}${path}`, {
    headers: { 'Content-Type': 'application/json' },
    signal,
    ...options,
  })
  const ct = res.headers.get('content-type') ?? ''
  if (!ct.includes('application/json')) {
    throw new Error(`HTTP ${res.status}: unexpected response type "${ct}"`)
  }
  const json = await res.json()
  if (!json.success) throw new Error(json.error || `HTTP ${res.status}`)
  return json.data
}

export const api = {
  // ── Devices ────────────────────────────────────────────────────────────────
  getDevices: () => request('/devices'),

  // ── Motors ─────────────────────────────────────────────────────────────────
  getMotor: (jointName) => request(`/motors/${encodeURIComponent(jointName)}`),
  // mode: 'POSITION' | 'VELOCITY' | 'TORQUE' | 'CURRENT'
  enableMotor: (jointName, mode = 'POSITION') =>
    request(`/motors/${encodeURIComponent(jointName)}/enable`, {
      method: 'POST',
      body: JSON.stringify({ mode }),
    }),
  connectMotor: (jointName) =>
    request(`/motors/${encodeURIComponent(jointName)}/connect`, { method: 'POST' }),
  disableMotor: (jointName) =>
    request(`/motors/${encodeURIComponent(jointName)}/disable`, { method: 'POST' }),
  clearMotorError: (jointName) =>
    request(`/motors/${encodeURIComponent(jointName)}/clear_error`, { method: 'POST' }),
  estopMotor: (jointName) =>
    request(`/motors/${encodeURIComponent(jointName)}/estop`, { method: 'POST' }),
  calibrateMotor: (jointName) =>
    request(`/motors/${encodeURIComponent(jointName)}/calibrate`, { method: 'POST' }),
  setPosition: (jointName, position, velocity_ff = 0, torque_ff = 0) =>
    request(`/motors/${encodeURIComponent(jointName)}/position`, {
      method: 'POST',
      body: JSON.stringify({ position, velocity_ff, torque_ff }),
    }),

  // ── Robot config ───────────────────────────────────────────────────────────
  getRobotConfig: () => request('/robot/config'),
  putRobotConfig: (config) =>
    request('/robot/config', { method: 'PUT', body: JSON.stringify(config) }),
  connectRobot: () => request('/robot/connect', { method: 'POST' }),
  disconnectRobot: () => request('/robot/disconnect', { method: 'POST' }),

  // ── USB devices ────────────────────────────────────────────────────────────
  getUsbDevices: () => request('/devices/usb'),

  // ── CAN monitor ────────────────────────────────────────────────────────────
  getCanStatus:  () => request('/devices/can/status'),
  getCanTraffic: () => request('/devices/can/traffic'),
  bringUpInterface: (name) =>
    request(`/devices/can/${encodeURIComponent(name)}/up`, { method: 'POST' }),

  // ── CAN adapter setup ──────────────────────────────────────────────────────
  getAdapters: () => request('/devices/can/adapters'),
  assignAdapter: (usb_serial, limb) =>
    request('/devices/can/assign', {
      method: 'POST',
      body: JSON.stringify({ usb_serial, limb }),
    }),
  unassignAdapter: (usb_serial) =>
    request('/devices/can/unassign', {
      method: 'POST',
      body: JSON.stringify({ usb_serial }),
    }),
  dismissSetup: () => request('/ui/dismiss-setup', { method: 'POST' }),
  pingBus: (name) => request(`/devices/can/${encodeURIComponent(name)}/ping`, { method: 'POST' }),
  scanBus: (name) => request(`/devices/can/${encodeURIComponent(name)}/scan`, { method: 'POST' }),

  // ── Flash wizard ───────────────────────────────────────────────────────────
  flashProfiles: () => request('/flash/profiles'),
  flashStart: (can_id, invert_phase, motor_profile, port = 'SWD', can_channel = 'can0', skip_flash = false, skip_commutation_check = false) =>
    request('/flash/start', {
      method: 'POST',
      body: JSON.stringify({ can_id, invert_phase, motor_profile, port, can_channel, skip_flash, skip_commutation_check }),
    }),
  flashStatus: () => request('/flash/status'),
  flashStep:   () => request('/flash/step'),
  flashReset:  () => request('/flash/reset', { method: 'POST' }),
  flashPowerCycled: () => request('/flash/power_cycled', { method: 'POST' }),
  flashCanConnected: () => request('/flash/can_connected', { method: 'POST' }),
  flashCanPing: () => request('/flash/can_ping', { method: 'POST' }),
  flashCheckFirmwareVersion: () => request('/flash/firmware_version'),

  // ── Motor calibration ──────────────────────────────────────────────────────
  setMotorPositionOffset: (jointName, positionOffset) =>
    request(`/motors/${encodeURIComponent(jointName)}/position_offset`, {
      method: 'POST',
      body: JSON.stringify({ position_offset: positionOffset }),
    }),
  positionCalibrateMotor: (jointName, hardstopLowerRad, limitsMin, limitsMax) =>
    request(`/motors/${encodeURIComponent(jointName)}/position_calibrate`, {
      method: 'POST',
      body: JSON.stringify({ hardstop_lower_rad: hardstopLowerRad, limits_min: limitsMin, limits_max: limitsMax }),
    }),
  // Hardstop range calibration: zero offset, then derive gear sign + offset + limits.
  rangeCalStart: (jointName) =>
    request(`/motors/${encodeURIComponent(jointName)}/range_cal_start`, { method: 'POST' }),
  rangeCalApply: (jointName, lowerPosRad, upperPosRad, opts = {}) =>
    request(`/motors/${encodeURIComponent(jointName)}/range_cal_apply`, {
      method: 'POST',
      body: JSON.stringify({
        lower_pos_rad: lowerPosRad,
        upper_pos_rad: upperPosRad,
        min_rad: opts.minRad ?? null,
        max_rad: opts.maxRad ?? null,
        store_to_flash: opts.storeToFlash ?? false,
      }),
    }),
  jogDirection: (jointName, stepRad = 0.2, moveS = 1.5) =>
    request(`/motors/${encodeURIComponent(jointName)}/jog_direction`, {
      method: 'POST',
      body: JSON.stringify({ step_rad: stepRad, move_s: moveS }),
    }),

  // ── ESC config sync ────────────────────────────────────────────────────────
  getMotorConfigFromDevice: (jointName) =>
    request(`/motors/${encodeURIComponent(jointName)}/config_from_device`),
  applyMotorConfig: (jointName, config) =>
    request(`/motors/${encodeURIComponent(jointName)}/apply_config`, {
      method: 'POST',
      body: JSON.stringify({ config }),
    }),
  writeMotorGains: (jointName, position_kp, position_ki, velocity_kp, torque_limit) =>
    request(`/motors/${encodeURIComponent(jointName)}/write_gains`, {
      method: 'POST',
      body: JSON.stringify({ position_kp, position_ki, velocity_kp, torque_limit }),
    }),
  storeMotorToFlash: (jointName) =>
    request(`/motors/${encodeURIComponent(jointName)}/store_to_flash`, { method: 'POST' }),
  runStepTest: (jointName, params, signal = undefined) =>
    request(`/motors/${encodeURIComponent(jointName)}/step_test`, {
      method: 'POST',
      body: JSON.stringify(params),
    }, signal),

  // ── Diagnostic + gravity-aware auto-tuner ────────────────────────────────────
  runDiagnosis: (jointName, params, signal = undefined) =>
    request(`/motors/${encodeURIComponent(jointName)}/diagnose`, {
      method: 'POST',
      body: JSON.stringify(params),
    }, signal),
  runGravityTune: (jointName, params, signal = undefined) =>
    request(`/motors/${encodeURIComponent(jointName)}/gravity_tune`, {
      method: 'POST',
      body: JSON.stringify(params),
    }, signal),
  // Long-running (~90s): flips phase order + recalibrates flux, then re-diagnoses.
  remediatePhase: (jointName) =>
    request(`/motors/${encodeURIComponent(jointName)}/remediate_phase`, {
      method: 'POST',
      body: JSON.stringify({ confirm: true }),
    }),
  raiseTorqueLimit: (jointName, torque_limit) =>
    request(`/motors/${encodeURIComponent(jointName)}/raise_torque_limit`, {
      method: 'POST',
      body: JSON.stringify({ torque_limit, confirm: true }),
    }),

  // ── App settings ───────────────────────────────────────────────────────────
  getSettings: () => request('/settings'),
  putSettings: (data) =>
    request('/settings', { method: 'PUT', body: JSON.stringify(data) }),
}
