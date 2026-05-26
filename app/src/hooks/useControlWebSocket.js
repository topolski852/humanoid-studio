import { useCallback, useEffect, useRef, useState } from 'react'

const WS_URL = 'ws://localhost:8765/ws/control'
const RECONNECT_DELAY_MS = 2000

/**
 * WebSocket hook for high-rate position commands (up to 200 Hz).
 *
 * Returns:
 *   sendPositionCommand(jointName, posRad, options) — sends a command if WS is open
 *   lastAck      — most recent command_ack from the server
 *   latencyMs    — round-trip latency of the last ack (ms)
 *   connected    — whether the WS is currently open
 */
export function useControlWebSocket() {
  const wsRef      = useRef(null)
  const retryTimer = useRef(null)

  const [connected, setConnected]  = useState(false)
  const [lastAck,   setLastAck]    = useState(null)
  const [latencyMs, setLatencyMs]  = useState(null)

  useEffect(() => {
    let cancelled = false

    function connect() {
      if (cancelled) return
      const ws = new WebSocket(WS_URL)
      wsRef.current = ws

      ws.onopen = () => setConnected(true)

      ws.onmessage = (evt) => {
        try {
          const msg = JSON.parse(evt.data)
          if (msg.command_ack) {
            setLastAck(msg)
            if (msg.latency_ms != null) setLatencyMs(msg.latency_ms)
          }
        } catch { /* ignore parse errors */ }
      }

      ws.onclose = () => {
        setConnected(false)
        wsRef.current = null
        if (!cancelled) retryTimer.current = setTimeout(connect, RECONNECT_DELAY_MS)
      }

      ws.onerror = () => ws.close()
    }

    connect()

    return () => {
      cancelled = true
      clearTimeout(retryTimer.current)
      const ws = wsRef.current
      if (!ws) return
      ws.onclose = null
      ws.onerror = null
      if (ws.readyState === WebSocket.CONNECTING) {
        ws.onopen = () => ws.close()
      } else {
        ws.close()
      }
    }
  }, [])

  const sendPositionCommand = useCallback((jointName, posRad, options = {}) => {
    const ws = wsRef.current
    if (!ws || ws.readyState !== WebSocket.OPEN) return false
    ws.send(JSON.stringify({
      joint_name:  jointName,
      position:    posRad,
      velocity_ff: options.velocityFf ?? 0.0,
      torque_ff:   options.torqueFf   ?? 0.0,
    }))
    return true
  }, [])

  return { sendPositionCommand, lastAck, latencyMs, connected }
}
