const BASE = 'http://localhost:8765'

async function request(path, options = {}) {
  const res = await fetch(`${BASE}${path}`, {
    headers: { 'Content-Type': 'application/json' },
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
  flashStart: (can_id, invert_phase, motor_profile, port = 'SWD', can_channel = 'can0', skip_flash = false) =>
    request('/flash/start', {
      method: 'POST',
      body: JSON.stringify({ can_id, invert_phase, motor_profile, port, can_channel, skip_flash }),
    }),
  flashStatus: () => request('/flash/status'),
  flashStep:   () => request('/flash/step'),
  flashReset:  () => request('/flash/reset', { method: 'POST' }),
  flashPowerCycled: () => request('/flash/power_cycled', { method: 'POST' }),
  flashCanConnected: () => request('/flash/can_connected', { method: 'POST' }),
  flashCanPing: () => request('/flash/can_ping', { method: 'POST' }),
  flashCheckFirmwareVersion: () => request('/flash/firmware_version'),
  flashConfirm: (correct) =>
    request('/flash/confirm_direction', {
      method: 'POST',
      body: JSON.stringify({ correct }),
    }),

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

  // ── ESC config sync ────────────────────────────────────────────────────────
  getMotorConfigFromDevice: (jointName) =>
    request(`/motors/${encodeURIComponent(jointName)}/config_from_device`),
  applyMotorConfig: (jointName, config) =>
    request(`/motors/${encodeURIComponent(jointName)}/apply_config`, {
      method: 'POST',
      body: JSON.stringify({ config }),
    }),
  storeMotorToFlash: (jointName) =>
    request(`/motors/${encodeURIComponent(jointName)}/store_to_flash`, { method: 'POST' }),
  runStepTest: (jointName, params) =>
    request(`/motors/${encodeURIComponent(jointName)}/step_test`, {
      method: 'POST',
      body: JSON.stringify(params),
    }),

  // ── App settings ───────────────────────────────────────────────────────────
  getSettings: () => request('/settings'),
  putSettings: (data) =>
    request('/settings', { method: 'PUT', body: JSON.stringify(data) }),
}
