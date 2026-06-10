import { createContext, useContext, useEffect, useRef, useState } from 'react'
import { BUSES } from '../constants'

const WS_URL = 'ws://localhost:8765/ws/telemetry'

function makeDefaultHealth() {
  return BUSES.map(({ name }) => ({
    name,
    state: 'UNKNOWN',
    bus_error_state: 'UNKNOWN',
    bitrate: 0,
    rx_packets: 0, tx_packets: 0,
    rx_errors: 0,  tx_errors: 0,
    rx_dropped: 0,
    message_rate: 0.0,
    rate_history: [],
    joints_online: 0,
    joints_total: 0,
    joints: [],
    usb_path: '',
  }))
}

/**
 * Build passiveTelemetry lookup from canHealth joints arrays.
 * Result: { joint_name: { status, position_rad, velocity_rads, last_seen_ms } }
 * Only joints with status ONLINE or STALE are included.
 */
function buildPassiveTelemetry(ifaces) {
  const pt = {}
  for (const iface of ifaces) {
    for (const j of (iface.joints ?? [])) {
      if (j.name && (j.status === 'ONLINE' || j.status === 'STALE')) {
        pt[j.name] = {
          status:        j.status,
          position_rad:  j.position_rad,
          velocity_rads: j.velocity_rads,
          last_seen_ms:  j.last_seen_ms,
        }
      }
    }
  }
  return pt
}

const TelemetryContext = createContext({
  states: {},
  robotConnected: false,
  wsConnected: false,
  canHealth: makeDefaultHealth(),
  dropLog: [],
  passiveTelemetry: {},  // { joint_name: { status, position_rad, velocity_rads, last_seen_ms } }
})

export function TelemetryProvider({ children }) {
  const [states, setStates]               = useState({})
  const [robotConnected, setRobotConnected] = useState(false)
  const [wsConnected, setWsConnected]     = useState(false)
  const [canHealth, setCanHealth]         = useState(makeDefaultHealth)
  const [dropLog, setDropLog]             = useState([])
  const [passiveTelemetry, setPassiveTelemetry] = useState({})
  const wsRef           = useRef(null)
  const reconnectTimer  = useRef(null)

  useEffect(() => {
    let cancelled = false

    function connect() {
      if (cancelled) return
      const ws = new WebSocket(WS_URL)
      wsRef.current = ws

      ws.onopen = () => setWsConnected(true)

      ws.onmessage = (ev) => {
        try {
          const data = JSON.parse(ev.data)

          if (data.type === 'can_health') {
            const ifaces = data.interfaces ?? makeDefaultHealth()
            setCanHealth(ifaces)
            setPassiveTelemetry(buildPassiveTelemetry(ifaces))
          } else if (data.type === 'can_drop_event') {
            setDropLog((prev) => [data, ...prev].slice(0, 50))
          } else {
            // Motor telemetry (no type field — backward-compat)
            setRobotConnected(data.connected ?? false)
            setStates(data.actuators ?? {})
          }
        } catch(e) { console.warn('WS parse error', e) }
      }

      ws.onclose = () => {
        setWsConnected(false)
        setRobotConnected(false)
        if (!cancelled) reconnectTimer.current = setTimeout(connect, 2000)
      }

      ws.onerror = () => ws.close()
    }

    connect()

    return () => {
      cancelled = true
      clearTimeout(reconnectTimer.current)
      const ws = wsRef.current
      if (!ws) return
      // Null handlers before closing to prevent the reconnect timer from firing
      // and to avoid the "closed before connection established" Chrome warning
      // when React StrictMode double-mounts this effect in development.
      ws.onclose = null
      ws.onerror = null
      if (ws.readyState === WebSocket.CONNECTING) {
        // Can't close a CONNECTING socket without the Chrome warning; let it
        // connect first, then close immediately.
        ws.onopen = () => ws.close()
      } else {
        ws.close()
      }
    }
  }, [])

  return (
    <TelemetryContext.Provider value={{ states, robotConnected, wsConnected, canHealth, dropLog, passiveTelemetry }}>
      {children}
    </TelemetryContext.Provider>
  )
}

export const useTelemetry = () => useContext(TelemetryContext)
