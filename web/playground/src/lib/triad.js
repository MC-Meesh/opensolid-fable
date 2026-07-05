// Orientation-triad projection: where the world X/Y/Z axes point on screen
// for a given camera orientation.
//
// The camera's world quaternion rotates camera space into world space, so the
// inverse (conjugate, for a unit quaternion) maps world axes into camera
// space: x is screen-right, y is screen-up, z points from the scene toward
// the viewer (depth > 0 means the axis tip faces the camera).
//
// Pure math, no three.js — unit-tested directly.

function rotateByConjugate([qx, qy, qz, qw], [vx, vy, vz]) {
  // v' = q* v q for unit quaternion q = (qx, qy, qz, qw).
  const cx = -qx;
  const cy = -qy;
  const cz = -qz;
  // t = 2 * (c × v)
  const tx = 2 * (cy * vz - cz * vy);
  const ty = 2 * (cz * vx - cx * vz);
  const tz = 2 * (cx * vy - cy * vx);
  // v' = v + w*t + c × t
  return [
    vx + qw * tx + cy * tz - cz * ty,
    vy + qw * ty + cz * tx - cx * tz,
    vz + qw * tz + cx * ty - cy * tx,
  ];
}

const WORLD_AXES = [
  { axis: 'x', dir: [1, 0, 0] },
  { axis: 'y', dir: [0, 1, 0] },
  { axis: 'z', dir: [0, 0, 1] },
];

/**
 * Project the world axes through a camera orientation.
 *
 * `quat` is the camera's world quaternion as `[x, y, z, w]`. Returns one
 * entry per axis: `{ axis, x, y, depth }` where `(x, y)` is the unit screen
 * direction (y up-positive) and `depth > 0` means the +axis tip points
 * toward the viewer. Entries are ordered back-to-front so callers can paint
 * them in array order.
 */
export function projectTriad(quat) {
  return WORLD_AXES.map(({ axis, dir }) => {
    const [x, y, depth] = rotateByConjugate(quat, dir);
    return { axis, x, y, depth };
  }).sort((a, b) => a.depth - b.depth);
}
