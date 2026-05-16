import { createContext, useContext, useEffect, useRef, useState } from 'react'

const WS_URL = 'ws://localhost:8765/ws/telemetry'

const ALL_BUSES = ['can_left_leg', 'can_right_leg', 'can_left_arm', 'can_right_arm']

function makeDefaultHealth() {
  return ALL_BUSES.map((name) => ({
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
    function connect() {
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
        reconnectTimer.current = setTimeout(connect, 2000)
      }

      ws.onerror = () => ws.close()
    }

    connect()

    return () => {
      clearTimeout(reconnectTimer.current)
      wsRef.current?.close()
    }
  }, [])

  return (
    <TelemetryContext.Provider value={{ states, robotConnected, wsConnected, canHealth, dropLog, passiveTelemetry }}>
      {children}
    </TelemetryContext.Provider>
  )
}

export const useTelemetry = () => useContext(TelemetryContext)
