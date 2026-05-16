export default function ErrorLogPanel({ logs, onClear }) {
  function copyAll() {
    const text = [...logs].reverse()
      .map(e => `[${e.time.toLocaleTimeString()}] [${e.source}] ${e.message}`)
      .join('\n')
    navigator.clipboard.writeText(text).catch(() => {})
  }

  return (
    <div className="flex-1 flex flex-col p-6 overflow-hidden">
      <div className="flex items-center gap-3 mb-4 flex-shrink-0">
        <p className="data-label flex-1">
          ERROR LOG
          {logs.length > 0 && (
            <span className="ml-1 text-gray-600 normal-case font-normal">
              ({logs.length}/50)
            </span>
          )}
        </p>
        {logs.length > 0 && (
          <>
            <button
              onClick={copyAll}
              className="text-[10px] text-gray-500 hover:text-gray-300 px-2 py-1 rounded hover:bg-surface-3 transition-colors font-mono"
            >
              Copy All
            </button>
            <button
              onClick={onClear}
              className="text-[10px] text-gray-500 hover:text-gray-300 px-2 py-1 rounded hover:bg-surface-3 transition-colors font-mono"
            >
              Clear
            </button>
          </>
        )}
      </div>

      {logs.length === 0 ? (
        <p className="text-xs text-gray-600 font-mono">No errors recorded.</p>
      ) : (
        <div className="flex-1 overflow-y-auto space-y-1">
          {[...logs].reverse().map((entry) => (
            <div key={entry.id} className="flex gap-2 text-[10px] font-mono leading-relaxed">
              <span className="text-gray-600 shrink-0 tabular-nums w-20">
                {entry.time.toLocaleTimeString()}
              </span>
              <span className={`px-1 rounded shrink-0 uppercase text-[9px] leading-[1.6]
                ${entry.source === 'Firmware'
                  ? 'bg-danger/20 text-danger'
                  : 'bg-surface-3 text-gray-400'}`}>
                {entry.source}
              </span>
              <span className="text-gray-300 break-all">{entry.message}</span>
            </div>
          ))}
        </div>
      )}
    </div>
  )
}
