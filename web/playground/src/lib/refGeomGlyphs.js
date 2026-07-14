// Reference-geometry glyph geometry (of-fsl.14): pure functions that turn a
// reference entity into the line/point data Viewport3D uploads to three.js.
// Kept free of three.js so the endpoint math is unit-testable; the component
// only wraps the returned arrays in buffers.

const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const scale = (a, s) => [a[0] * s, a[1] * s, a[2] * s];

/** Default drawn span for an axis/csys glyph relative to scene size. */
export const DEFAULT_GLYPH_SPAN = 5;

/**
 * The two world endpoints of a reference axis's drawn segment.
 *
 * A two-point axis is drawn as the actual segment between its points (origin
 * to origin + direction·length). Every other axis has no intrinsic extent, so
 * it is drawn centered on its origin spanning `fallbackLength` — a datum-line
 * look that reads as "infinite" without running off to the horizon.
 */
export function axisEndpoints(axis, fallbackLength = DEFAULT_GLYPH_SPAN) {
  const { origin, direction, method, length } = axis;
  if (method === 'two-points') {
    const len = length > 0 ? length : fallbackLength;
    return [[...origin], add(origin, scale(direction, len))];
  }
  const half = scale(direction, fallbackLength / 2);
  return [sub(origin, half), add(origin, half)];
}

/** Flat [x,y,z, x,y,z] positions for an axis segment (three.js buffer form). */
export function axisPositions(axis, fallbackLength = DEFAULT_GLYPH_SPAN) {
  const [a, b] = axisEndpoints(axis, fallbackLength);
  return new Float32Array([a[0], a[1], a[2], b[0], b[1], b[2]]);
}

/** Coordinate-system axis colors (X red, Y green, Z blue — the CAD triad). */
export const CSYS_AXIS_COLORS = { x: 0xff5555, y: 0x55ff55, z: 0x5599ff };

/**
 * The three drawn axis segments of a coordinate system, each `{ key, color,
 * from, to }` running from the origin out along that axis by `size`.
 */
export function csysSegments(csys, size = DEFAULT_GLYPH_SPAN) {
  return ['x', 'y', 'z'].map((key) => ({
    key,
    color: CSYS_AXIS_COLORS[key],
    from: [...csys.origin],
    to: add(csys.origin, scale(csys[key], size)),
  }));
}

/**
 * The four world corners of a reference plane's quad indicator, in the order
 * (-u-v, +u-v, +u+v, -u+v). `half` is the half-side; defaults to the entity's
 * indicator extent. Corners feed a quad outline / fill in the viewport.
 */
export function planeQuadCorners(plane, half = plane.extent ?? DEFAULT_GLYPH_SPAN) {
  const { origin, u, v } = plane;
  const du = scale(u, half);
  const dv = scale(v, half);
  return [
    sub(sub(origin, du), dv),
    sub(add(origin, du), dv),
    add(add(origin, du), dv),
    add(sub(origin, du), dv),
  ];
}
