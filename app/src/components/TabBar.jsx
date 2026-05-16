export default function TabBar({ tabs, activeTabId, onSelect, onClose }) {
  return (
    <div className="flex-shrink-0 h-10 bg-surface-1 border-b border-surface-3 flex items-end overflow-x-auto">
      {tabs.map((tab) => {
        const active = tab.id === activeTabId
        return (
          <div
            key={tab.id}
            className={`
              relative flex items-center gap-2 px-4 h-9 text-sm cursor-pointer
              border-r border-surface-3 select-none whitespace-nowrap flex-shrink-0
              transition-colors
              ${active
                ? 'bg-surface text-white'
                : 'text-gray-500 hover:text-gray-300 hover:bg-surface/50'
              }
            `}
            onClick={() => onSelect(tab)}
          >
            {/* Active indicator bar */}
            {active && (
              <span className="absolute bottom-0 left-0 right-0 h-0.5 bg-accent" />
            )}
            <span>{tab.label}</span>
            {tab.closeable && (
              <button
                onClick={(e) => {
                  e.stopPropagation()
                  onClose(tab.id)
                }}
                className="w-4 h-4 flex items-center justify-center rounded hover:bg-white/15 text-gray-500 hover:text-gray-300 text-base leading-none"
              >
                ×
              </button>
            )}
          </div>
        )
      })}
    </div>
  )
}
