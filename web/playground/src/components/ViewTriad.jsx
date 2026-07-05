import { projectTriad } from '../lib/triad.js';
import { axisView } from '../lib/views.js';

const AXIS_COLORS = { x: '#e5646c', y: '#67c587', z: '#4f9cf9' };
const SIZE = 104;
const CENTER = SIZE / 2;
const ARM = 32;
const TIP = 42;

/**
 * Orientation triad (bottom-left, SolidWorks style): live world-axis
 * indicator. Clicking an axis tip snaps the camera to the standard view
 * looking down that axis; the hollow tip is the negative direction.
 */
export default function ViewTriad({ quat, onSelectView }) {
  const axes = projectTriad(quat);
  return (
    <svg
      className="view-triad"
      width={SIZE}
      height={SIZE}
      viewBox={`0 0 ${SIZE} ${SIZE}`}
      aria-label="Orientation triad"
    >
      {axes.map(({ axis, x, y, depth }) => {
        const color = AXIS_COLORS[axis];
        const sx = CENTER + x * ARM;
        const sy = CENTER - y * ARM;
        const tx = CENTER + x * TIP;
        const ty = CENTER - y * TIP;
        const facing = depth >= 0;
        return (
          <g key={axis} opacity={facing ? 1 : 0.45}>
            <line x1={CENTER} y1={CENTER} x2={sx} y2={sy} stroke={color} strokeWidth="2" />
            <circle
              className="triad-tip"
              cx={tx}
              cy={ty}
              r="9"
              fill={facing ? color : 'transparent'}
              stroke={color}
              strokeWidth="1.5"
              onClick={() => onSelectView?.(axisView(axis, facing))}
            >
              <title>{`${facing ? '+' : '−'}${axis.toUpperCase()} — ${axisView(axis, facing)} view`}</title>
            </circle>
            <text
              className="triad-label"
              x={tx}
              y={ty}
              fill={facing ? '#0b1220' : color}
              onClick={() => onSelectView?.(axisView(axis, facing))}
            >
              {axis.toUpperCase()}
            </text>
          </g>
        );
      })}
    </svg>
  );
}
