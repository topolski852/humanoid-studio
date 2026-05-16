export default function StatusDot({ online, size = 'sm' }) {
  const dim = size === 'lg' ? 'w-3 h-3' : 'w-2 h-2'
  return online ? (
    <span
      className={`${dim} rounded-full bg-online flex-shrink-0`}
      style={{ boxShadow: '0 0 6px #22c55e' }}
    />
  ) : (
    <span className={`${dim} rounded-full bg-offline flex-shrink-0`} />
  )
}
