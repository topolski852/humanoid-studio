import { useState, useCallback, useEffect } from 'react'
import { HashRouter, Routes, Route, Navigate, useNavigate, useLocation } from 'react-router-dom'
import { TelemetryProvider } from './context/TelemetryContext'
import Sidebar from './components/Sidebar'
import TabBar from './components/TabBar'
import MotorTab from './components/MotorTab'
import Dashboard from './pages/Dashboard'
import RobotConfig from './pages/RobotConfig'
import Settings from './pages/Settings'
import CanMonitor from './pages/CanMonitor'
import CanSetup from './pages/CanSetup'
import EscSetup from './pages/EscSetup'
import { api } from './api'

// ── Tab definitions ───────────────────────────────────────────────────────────
const STATIC_TABS = [
  { id: 'dashboard', label: 'Dashboard', path: '/dashboard', closeable: false },
]

function routeToTabId(pathname) {
  if (pathname === '/dashboard' || pathname === '/') return 'dashboard'
  if (pathname === '/robot-config') return 'robot-config'
  if (pathname === '/settings') return 'settings'
  const m = pathname.match(/^\/motor\/(.+)$/)
  if (m) return `motor-${decodeURIComponent(m[1])}`
  return 'dashboard'
}

// ── Setup banner ─────────────────────────────────────────────────────────────
function SetupBanner({ count, onDismiss }) {
  const navigate = useNavigate()
  if (count === 0) return null
  return (
    <div className="flex-shrink-0 flex items-center gap-3 px-4 py-2 bg-warn/10 border-b border-warn/20 text-xs">
      <svg className="w-3.5 h-3.5 text-warn flex-shrink-0" fill="none" viewBox="0 0 24 24" stroke="currentColor" strokeWidth={2}>
        <path strokeLinecap="round" strokeLinejoin="round" d="M12 9v2m0 4h.01m-6.938 4h13.856c1.54 0 2.502-1.667 1.732-3L13.732 4c-.77-1.333-2.694-1.333-3.464 0L3.34 16c-.77 1.333.192 3 1.732 3z" />
      </svg>
      <span className="text-warn flex-1">
        CAN adapters need setup — {count} unassigned adapter{count !== 1 ? 's' : ''} found
      </span>
      <button
        onClick={() => navigate('/can-setup')}
        className="text-warn font-medium hover:underline whitespace-nowrap"
      >
        Go to Setup →
      </button>
      <button
        onClick={onDismiss}
        className="text-gray-600 hover:text-gray-400 ml-1"
        title="Dismiss"
      >
        ✕
      </button>
    </div>
  )
}

// ── Inner app (needs Router context) ─────────────────────────────────────────
function AppInner() {
  const navigate = useNavigate()
  const location = useLocation()
  const [motorTabs, setMotorTabs] = useState([])
  const [unassignedCount, setUnassignedCount] = useState(0)
  const [bannerDismissed, setBannerDismissed] = useState(false)

  // Fetch adapter state once on mount to drive the banner
  useEffect(() => {
    api.getAdapters()
      .then(data => {
        setUnassignedCount(data?.unassigned_count ?? 0)
        setBannerDismissed(data?.banner_dismissed ?? false)
      })
      .catch(() => {})
  }, [])

  async function handleDismissBanner() {
    setBannerDismissed(true)
    try { await api.dismissSetup() } catch {}
  }

  // All tabs = static + open motor tabs
  const allTabs = [...STATIC_TABS, ...motorTabs]

  // Current active tab derived from URL
  const activeTabId = routeToTabId(location.pathname)

  const openMotorTab = useCallback(
    (canId, jointName) => {
      const name = jointName ?? `motor_${canId}`
      const id = `motor-${name}`
      const label = name
        .replace(/_joint$/, '').replace(/_/g, ' ')
        .replace(/\b\w/g, (c) => c.toUpperCase())
      const path = `/motor/${encodeURIComponent(name)}`

      setMotorTabs((prev) => {
        if (prev.find((t) => t.id === id)) return prev
        return [...prev, { id, label, path, closeable: true }]
      })
      navigate(path)
    },
    [navigate]
  )

  const closeTab = useCallback(
    (tabId) => {
      setMotorTabs((prev) => prev.filter((t) => t.id !== tabId))
      if (activeTabId === tabId) navigate('/dashboard')
    },
    [activeTabId, navigate]
  )

  const handleSelectTab = useCallback(
    (tab) => {
      navigate(tab.path)
    },
    [navigate]
  )

  return (
    <div className="flex h-screen bg-surface text-gray-200 overflow-hidden">
      {/* Sidebar */}
      <Sidebar />

      {/* Main area */}
      <div className="flex-1 flex flex-col overflow-hidden">
        {/* Tab bar */}
        <TabBar
          tabs={allTabs}
          activeTabId={activeTabId}
          onSelect={handleSelectTab}
          onClose={closeTab}
        />

        {/* Setup banner (shown when unassigned adapters exist and not dismissed) */}
        {!bannerDismissed && (
          <SetupBanner count={unassignedCount} onDismiss={handleDismissBanner} />
        )}

        {/* Route content */}
        <div className="flex-1 overflow-hidden">
          <Routes>
            <Route path="/dashboard" element={<Dashboard onOpenMotor={openMotorTab} />} />
            <Route path="/motor/:jointName" element={<MotorTab />} />
            <Route path="/robot-config" element={<RobotConfig />} />
            <Route path="/can-monitor" element={<CanMonitor />} />
            <Route path="/can-setup" element={<CanSetup />} />
            <Route path="/esc-setup" element={<EscSetup />} />
            <Route path="/settings" element={<Settings />} />
            <Route path="*" element={<Navigate to="/dashboard" replace />} />
          </Routes>
        </div>
      </div>
    </div>
  )
}

// ── Root ──────────────────────────────────────────────────────────────────────
export default function App() {
  return (
    <TelemetryProvider>
      <HashRouter future={{ v7_startTransition: true, v7_relativeSplatPath: true }}>
        <AppInner />
      </HashRouter>
    </TelemetryProvider>
  )
}
