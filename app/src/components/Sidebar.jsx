import { useEffect, useState } from 'react'
import { NavLink } from 'react-router-dom'
import { useTelemetry } from '../context/TelemetryContext'
import StatusDot from './StatusDot'
import { api } from '../api'

// ── Icons ─────────────────────────────────────────────────────────────────────
function GridIcon() {
  return (
    <svg className="w-4 h-4" fill="none" viewBox="0 0 24 24" stroke="currentColor" strokeWidth={1.5}>
      <path strokeLinecap="round" strokeLinejoin="round" d="M3.75 6A2.25 2.25 0 016 3.75h2.25A2.25 2.25 0 0110.5 6v2.25a2.25 2.25 0 01-2.25 2.25H6a2.25 2.25 0 01-2.25-2.25V6zM3.75 15.75A2.25 2.25 0 016 13.5h2.25a2.25 2.25 0 012.25 2.25V18a2.25 2.25 0 01-2.25 2.25H6A2.25 2.25 0 013.75 18v-2.25zM13.5 6a2.25 2.25 0 012.25-2.25H18A2.25 2.25 0 0120.25 6v2.25A2.25 2.25 0 0118 10.5h-2.25a2.25 2.25 0 01-2.25-2.25V6zM13.5 15.75a2.25 2.25 0 012.25-2.25H18a2.25 2.25 0 012.25 2.25V18A2.25 2.25 0 0118 20.25h-2.25A2.25 2.25 0 0113.5 18v-2.25z" />
    </svg>
  )
}

function ConfigIcon() {
  return (
    <svg className="w-4 h-4" fill="none" viewBox="0 0 24 24" stroke="currentColor" strokeWidth={1.5}>
      <path strokeLinecap="round" strokeLinejoin="round" d="M10.5 6h9.75M10.5 6a1.5 1.5 0 11-3 0m3 0a1.5 1.5 0 10-3 0M3.75 6H7.5m3 12h9.75m-9.75 0a1.5 1.5 0 01-3 0m3 0a1.5 1.5 0 00-3 0m-3.75 0H7.5m9-6h3.75m-3.75 0a1.5 1.5 0 01-3 0m3 0a1.5 1.5 0 00-3 0m-9.75 0h9.75" />
    </svg>
  )
}

function BusIcon() {
  return (
    <svg className="w-4 h-4" fill="none" viewBox="0 0 24 24" stroke="currentColor" strokeWidth={1.5}>
      <path strokeLinecap="round" strokeLinejoin="round"
        d="M8.288 15.038a5.25 5.25 0 017.424 0M5.106 11.856c3.807-3.808 9.98-3.808 13.788 0M1.924 8.674c5.565-5.565 14.587-5.565 20.152 0M12.53 18.22l-.53.53-.53-.53a.75.75 0 011.06 0z" />
    </svg>
  )
}

function SetupIcon() {
  return (
    <svg className="w-4 h-4" fill="none" viewBox="0 0 24 24" stroke="currentColor" strokeWidth={1.5}>
      <path strokeLinecap="round" strokeLinejoin="round" d="M14.25 6.087c0-.355.186-.676.401-.959.221-.29.349-.634.349-1.003 0-1.036-1.007-1.875-2.25-1.875s-2.25.84-2.25 1.875c0 .369.128.713.349 1.003.215.283.401.604.401.959v0a.64.64 0 01-.657.643 48.39 48.39 0 01-4.163-.3c.186 1.613.293 3.25.315 4.907a.656.656 0 01-.658.663v0c-.355 0-.676-.186-.959-.401a1.647 1.647 0 00-1.003-.349c-1.036 0-1.875 1.007-1.875 2.25s.84 2.25 1.875 2.25c.369 0 .713-.128 1.003-.349.283-.215.604-.401.959-.401v0c.31 0 .555.26.532.57a48.039 48.039 0 01-.642 5.056c1.518.19 3.058.309 4.616.354a.64.64 0 00.657-.643v0c0-.355-.186-.676-.401-.959a1.647 1.647 0 01-.349-1.003c0-1.035 1.008-1.875 2.25-1.875 1.243 0 2.25.84 2.25 1.875 0 .369-.128.713-.349 1.003-.215.283-.4.604-.4.959v0c0 .333.277.599.61.58a48.1 48.1 0 005.427-.63 48.05 48.05 0 00.582-4.717.532.532 0 00-.533-.57v0c-.355 0-.676.186-.959.401-.29.221-.634.349-1.003.349-1.035 0-1.875-1.007-1.875-2.25s.84-2.25 1.875-2.25c.37 0 .713.128 1.003.349.283.215.604.401.959.401v0a.656.656 0 00.658-.663 48.422 48.422 0 00-.37-5.36c-1.886.342-3.81.574-5.766.689a.578.578 0 01-.61-.58v0z" />
    </svg>
  )
}

function FlashIcon() {
  return (
    <svg className="w-4 h-4" fill="none" viewBox="0 0 24 24" stroke="currentColor" strokeWidth={1.5}>
      <path strokeLinecap="round" strokeLinejoin="round" d="M3.75 13.5l10.5-11.25L12 10.5h8.25L9.75 21.75 12 13.5H3.75z" />
    </svg>
  )
}

function SettingsIcon() {
  return (
    <svg className="w-4 h-4" fill="none" viewBox="0 0 24 24" stroke="currentColor" strokeWidth={1.5}>
      <path strokeLinecap="round" strokeLinejoin="round" d="M9.594 3.94c.09-.542.56-.94 1.11-.94h2.593c.55 0 1.02.398 1.11.94l.213 1.281c.063.374.313.686.645.87.074.04.147.083.22.127.324.196.72.257 1.075.124l1.217-.456a1.125 1.125 0 011.37.49l1.296 2.247a1.125 1.125 0 01-.26 1.431l-1.003.827c-.293.24-.438.613-.431.992a6.759 6.759 0 010 .255c-.007.378.138.75.43.99l1.005.828c.424.35.534.954.26 1.43l-1.298 2.247a1.125 1.125 0 01-1.369.491l-1.217-.456c-.355-.133-.75-.072-1.076.124a6.57 6.57 0 01-.22.128c-.331.183-.581.495-.644.869l-.213 1.28c-.09.543-.56.941-1.11.941h-2.594c-.55 0-1.02-.398-1.11-.94l-.213-1.281c-.062-.374-.312-.686-.644-.87a6.52 6.52 0 01-.22-.127c-.325-.196-.72-.257-1.076-.124l-1.217.456a1.125 1.125 0 01-1.369-.49l-1.297-2.247a1.125 1.125 0 01.26-1.431l1.004-.827c.292-.24.437-.613.43-.992a6.932 6.932 0 010-.255c.007-.378-.138-.75-.43-.99l-1.004-.828a1.125 1.125 0 01-.26-1.43l1.297-2.247a1.125 1.125 0 011.37-.491l1.216.456c.356.133.751.072 1.076-.124.072-.044.146-.087.22-.128.332-.183.582-.495.644-.869l.214-1.281z" />
      <path strokeLinecap="round" strokeLinejoin="round" d="M15 12a3 3 0 11-6 0 3 3 0 016 0z" />
    </svg>
  )
}

// ── Nav link ──────────────────────────────────────────────────────────────────
function SidebarLink({ to, icon, label }) {
  return (
    <NavLink
      to={to}
      className={({ isActive }) =>
        `flex items-center gap-3 px-3 py-2 rounded-lg text-sm transition-colors ${
          isActive
            ? 'bg-accent/20 text-accent'
            : 'text-gray-400 hover:text-gray-200 hover:bg-white/5'
        }`
      }
    >
      {icon}
      {label}
    </NavLink>
  )
}

// ── Main component ────────────────────────────────────────────────────────────
export default function Sidebar() {
  const { robotConnected, wsConnected } = useTelemetry()
  const [connecting, setConnecting] = useState(false)
  const [unassignedCount, setUnassignedCount] = useState(0)

  // Poll adapter assignment count every 15 s so badge stays current after setup
  useEffect(() => {
    function fetchCount() {
      api.getAdapters()
        .then(data => setUnassignedCount(data?.unassigned_count ?? 0))
        .catch(() => {})
    }
    fetchCount()
    const id = setInterval(fetchCount, 15_000)
    return () => clearInterval(id)
  }, [])

  async function handleConnectToggle() {
    setConnecting(true)
    try {
      if (robotConnected) {
        await api.disconnectRobot()
      } else {
        await api.connectRobot()
      }
    } catch {
      // telemetry WS will reflect the new state
    } finally {
      setConnecting(false)
    }
  }

  return (
    <aside className="w-60 flex-shrink-0 bg-surface-1 border-r border-surface-3 flex flex-col overflow-hidden">
      {/* ── Logo ── */}
      <div className="px-4 py-4 border-b border-surface-3">
        <div className="flex items-center gap-2.5 mb-2">
          <div className="w-8 h-8 rounded-lg bg-accent flex items-center justify-center font-bold text-sm shadow-lg shadow-accent/30">
            H
          </div>
          <div>
            <div className="font-semibold text-sm leading-none">Humanoid Studio</div>
            <div className="text-[10px] text-gray-500 leading-none mt-0.5">v0.1.0</div>
          </div>
        </div>
        <div className="flex items-center gap-1.5">
          <StatusDot online={wsConnected && robotConnected} />
          <span className="text-[11px] text-gray-500 flex-1">
            {!wsConnected ? 'API offline' : robotConnected ? 'Robot connected' : 'Robot disconnected'}
          </span>
          {wsConnected && (
            <button
              onClick={handleConnectToggle}
              disabled={connecting}
              className={`text-[10px] px-1.5 py-0.5 rounded font-medium transition-colors disabled:opacity-50 ${
                robotConnected
                  ? 'text-gray-500 hover:text-gray-300 hover:bg-white/5'
                  : 'text-accent hover:text-white hover:bg-accent/20'
              }`}
            >
              {connecting ? '…' : robotConnected ? 'Disconnect' : 'Connect'}
            </button>
          )}
        </div>
      </div>

      {/* ── Navigation ── */}
      <nav className="flex-1 px-2 pt-3 pb-1 space-y-0.5">
        <SidebarLink to="/dashboard"    icon={<GridIcon />}   label="Dashboard" />
        <SidebarLink to="/can-monitor"  icon={<BusIcon />}    label="CAN Monitor" />
        {/* CAN Setup with unassigned-adapter badge */}
        <NavLink
          to="/can-setup"
          className={({ isActive }) =>
            `flex items-center gap-3 px-3 py-2 rounded-lg text-sm transition-colors ${
              isActive
                ? 'bg-accent/20 text-accent'
                : 'text-gray-400 hover:text-gray-200 hover:bg-white/5'
            }`
          }
        >
          <SetupIcon />
          <span className="flex-1">CAN Setup</span>
          {unassignedCount > 0 && (
            <span className="min-w-[1.1rem] h-[1.1rem] rounded-full bg-warn text-surface text-[9px] font-bold flex items-center justify-center px-0.5">
              {unassignedCount}
            </span>
          )}
        </NavLink>
        <SidebarLink to="/esc-setup"    icon={<FlashIcon />}  label="ESC Setup" />
        <SidebarLink to="/robot-config" icon={<ConfigIcon />} label="Robot Config" />
      </nav>

      {/* ── Settings ── */}
      <div className="px-2 py-3 border-t border-surface-3">
        <SidebarLink to="/settings" icon={<SettingsIcon />} label="Settings" />
      </div>
    </aside>
  )
}
