/**
 * Front-view schematic of the Berkeley Humanoid Lite.
 * Proportions based on the robot's known geometry:
 *   - ~35cm torso height, ~45cm legs, wide hip stance
 *   - three-DOF hip cluster, single knee, two-DOF ankle
 *   - three-DOF shoulder cluster, single elbow, single wrist
 *
 * SVG viewBox: 300 × 480
 * Origin: top-left (y increases downward)
 */

// ---------------------------------------------------------------------------
// Joint layout — pixel positions within the 300×480 viewBox
// ---------------------------------------------------------------------------

const JOINTS = [
  // ── Left leg ────────────────────────────────────────────────────────────
  { name: 'left_hip_roll_joint',    x: 112, y: 210, label: 'HR' },
  { name: 'left_hip_yaw_joint',     x: 112, y: 228, label: 'HY' },
  { name: 'left_hip_pitch_joint',   x: 112, y: 246, label: 'HP' },
  { name: 'left_knee_pitch_joint',  x: 106, y: 326, label: 'KP' },
  { name: 'left_ankle_pitch_joint', x: 100, y: 400, label: 'AP' },
  { name: 'left_ankle_roll_joint',  x: 100, y: 418, label: 'AR' },

  // ── Right leg ───────────────────────────────────────────────────────────
  { name: 'right_hip_roll_joint',    x: 188, y: 210, label: 'HR' },
  { name: 'right_hip_yaw_joint',     x: 188, y: 228, label: 'HY' },
  { name: 'right_hip_pitch_joint',   x: 188, y: 246, label: 'HP' },
  { name: 'right_knee_pitch_joint',  x: 194, y: 326, label: 'KP' },
  { name: 'right_ankle_pitch_joint', x: 200, y: 400, label: 'AP' },
  { name: 'right_ankle_roll_joint',  x: 200, y: 418, label: 'AR' },

  // ── Left arm ────────────────────────────────────────────────────────────
  { name: 'left_shoulder_pitch_joint', x: 88,  y: 90,  label: 'SP' },
  { name: 'left_shoulder_roll_joint',  x: 78,  y: 114, label: 'SR' },
  { name: 'left_shoulder_yaw_joint',   x: 66,  y: 150, label: 'SY' },
  { name: 'left_elbow_pitch_joint',    x: 56,  y: 192, label: 'EP' },
  { name: 'left_wrist_yaw_joint',      x: 48,  y: 230, label: 'WY' },

  // ── Right arm ───────────────────────────────────────────────────────────
  { name: 'right_shoulder_pitch_joint', x: 212, y: 90,  label: 'SP' },
  { name: 'right_shoulder_roll_joint',  x: 222, y: 114, label: 'SR' },
  { name: 'right_shoulder_yaw_joint',   x: 234, y: 150, label: 'SY' },
  { name: 'right_elbow_pitch_joint',    x: 244, y: 192, label: 'EP' },
  { name: 'right_wrist_yaw_joint',      x: 252, y: 230, label: 'WY' },
]

// Lookup for fast access
const JOINT_MAP = Object.fromEntries(JOINTS.map((j) => [j.name, j]))

// ---------------------------------------------------------------------------
// Structural lines — drawn before joints so joints sit on top
// ---------------------------------------------------------------------------

const LINES = [
  // Head and neck
  [150, 50, 150, 70],   // neck stalk

  // Torso
  [150, 70,  150, 200], // spine

  // Shoulder girdle
  [88,  90,  212, 90],  // clavicle

  // Left arm chain
  [88,  90,  78,  114],
  [78,  114, 66,  150],
  [66,  150, 56,  192],
  [56,  192, 48,  230],

  // Right arm chain
  [212, 90,  222, 114],
  [222, 114, 234, 150],
  [234, 150, 244, 192],
  [244, 192, 252, 230],

  // Hip girdle
  [112, 200, 188, 200],

  // Left leg chain
  [112, 200, 112, 210],
  [112, 228, 112, 246],
  [112, 246, 106, 326],
  [106, 326, 100, 400],
  [100, 418, 82,  436], // left foot
  [82,  436, 122, 436],

  // Right leg chain
  [188, 200, 188, 210],
  [188, 228, 188, 246],
  [188, 246, 194, 326],
  [194, 326, 200, 400],
  [200, 418, 182, 436], // right foot
  [182, 436, 218, 436],
]

// ---------------------------------------------------------------------------
// Component
// ---------------------------------------------------------------------------

const R = 8   // joint circle radius
const COLORS = {
  online:   '#22c55e',
  passive:  '#16a34a',  // muted green for passive-only
  offline:  '#4b5563',
  body:     '#1e2230',
  stroke:   '#2d3148',
  line:     '#374151',
  text:     '#9ca3af',
  textOnline: '#fff',
}

export default function RobotDiagram({ states = {}, passiveTelemetry = {}, onOpenMotor }) {
  return (
    <div className="flex flex-col items-center">
      <span className="data-label mb-2">Robot schematic</span>
      <svg
        viewBox="0 0 300 460"
        width="280"
        height="431"
        style={{ display: 'block' }}
        aria-label="Berkeley Humanoid Lite joint diagram"
      >
        {/* Background */}
        <rect x="0" y="0" width="300" height="460" fill="transparent" />

        {/* Head */}
        <circle cx="150" cy="34" r="18" fill={COLORS.body} stroke={COLORS.stroke} strokeWidth="1.5" />

        {/* Torso body outline */}
        <rect
          x="124" y="68" width="52" height="134"
          rx="6" fill={COLORS.body} stroke={COLORS.stroke} strokeWidth="1.5"
        />

        {/* Structural lines */}
        {LINES.map(([x1, y1, x2, y2], i) => (
          <line
            key={i}
            x1={x1} y1={y1} x2={x2} y2={y2}
            stroke={COLORS.line} strokeWidth="2" strokeLinecap="round"
          />
        ))}

        {/* Joint nodes */}
        {JOINTS.map((j) => {
          const activeOnline  = states[j.name] != null
          const passiveOnline = passiveTelemetry[j.name]?.status === 'ONLINE'
          const online  = activeOnline || passiveOnline
          const color   = activeOnline ? COLORS.online : passiveOnline ? COLORS.passive : COLORS.offline
          const fill    = color
          const txtCol  = online ? COLORS.textOnline : COLORS.text
          return (
            <g
              key={j.name}
              style={{ cursor: onOpenMotor ? 'pointer' : 'default' }}
              onClick={() => onOpenMotor?.(j.name)}
            >
              <title>{j.name}{activeOnline ? ` — ${states[j.name]?.mode_name ?? ''}` : passiveOnline ? ' (passive CAN)' : ' (offline)'}</title>

              {/* Glow ring when online */}
              {online && (
                <circle
                  cx={j.x} cy={j.y} r={R + 3}
                  fill="none" stroke={color}
                  strokeWidth="1" opacity="0.35"
                />
              )}

              <circle
                cx={j.x} cy={j.y} r={R}
                fill={fill} stroke={online ? color : COLORS.stroke}
                strokeWidth="1.5"
              />
              <text
                x={j.x} y={j.y + 3}
                textAnchor="middle" dominantBaseline="middle"
                fontSize="5.5" fontFamily="JetBrains Mono, monospace"
                fill={txtCol} fontWeight="700"
                style={{ userSelect: 'none' }}
              >
                {j.label}
              </text>
            </g>
          )
        })}
      </svg>

      {/* Legend */}
      <div className="flex gap-4 mt-2 flex-wrap justify-center">
        <span className="flex items-center gap-1.5 text-[10px] text-gray-500">
          <span className="w-2.5 h-2.5 rounded-full bg-online inline-block" />
          active
        </span>
        <span className="flex items-center gap-1.5 text-[10px] text-gray-500">
          <span className="w-2.5 h-2.5 rounded-full inline-block" style={{ backgroundColor: COLORS.passive }} />
          passive
        </span>
        <span className="flex items-center gap-1.5 text-[10px] text-gray-500">
          <span className="w-2.5 h-2.5 rounded-full bg-offline inline-block" />
          offline
        </span>
      </div>
    </div>
  )
}
