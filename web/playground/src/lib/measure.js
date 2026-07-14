// Measure-tool computations (of-fsl.17): pure geometry over the entities that
// measureTopology.snapEntity / facePlane produce — no three.js, no React, so
// every readout is unit-tested on plain arrays.
//
// Entity shapes (all points are world-space [x, y, z]):
//   { kind: 'vertex', point }
//   { kind: 'edge',   point, a, b, dir, length }
//   { kind: 'circle', point, center, radius, normal }
//   { kind: 'face',   point, plane: { origin, normal }, area }
//   { kind: 'point',  point }               // raw hit fallback
//
// `measureSingle(entity)` reads one entity; `measurePair(a, b)` reads the
// relationship between two (distance always, plus angle / gap / radius when
// the kinds make it meaningful).

const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
const norm = (a) => Math.hypot(a[0], a[1], a[2]);
const RAD2DEG = 180 / Math.PI;

function normalize(a) {
  const n = norm(a);
  return n > 0 ? [a[0] / n, a[1] / n, a[2] / n] : null;
}

/** Angle in degrees between two directions, folded to [0, 90] for lines/faces
 * (a line and its reverse are the same line; a plane and its flip the same
 * plane), so the readout never reports an "obtuse" version of an acute angle. */
function acuteAngleDeg(u, v) {
  const nu = normalize(u);
  const nv = normalize(v);
  if (!nu || !nv) return null;
  const c = Math.min(1, Math.max(-1, Math.abs(dot(nu, nv))));
  return Math.acos(c) * RAD2DEG;
}

/**
 * Axis-aligned bounding box of a positions array (flat xyz triples).
 * Returns `{ min, max, size: [dx, dy, dz], diagonal }`, or null when empty.
 */
export function boundingBoxDims(positions) {
  if (!positions || positions.length < 3) return null;
  const min = [Infinity, Infinity, Infinity];
  const max = [-Infinity, -Infinity, -Infinity];
  for (let i = 0; i < positions.length; i += 3) {
    for (let k = 0; k < 3; k += 1) {
      const c = positions[i + k];
      if (c < min[k]) min[k] = c;
      if (c > max[k]) max[k] = c;
    }
  }
  const size = [max[0] - min[0], max[1] - min[1], max[2] - min[2]];
  return { min, max, size, diagonal: norm(size) };
}

/** Total area of the triangles `tris` (indices into `indices`) of a mesh. */
export function triListArea(positions, indices, tris) {
  let area = 0;
  for (const t of tris) {
    const ia = indices[3 * t];
    const ib = indices[3 * t + 1];
    const ic = indices[3 * t + 2];
    const a = [positions[3 * ia], positions[3 * ia + 1], positions[3 * ia + 2]];
    const b = [positions[3 * ib], positions[3 * ib + 1], positions[3 * ib + 2]];
    const c = [positions[3 * ic], positions[3 * ic + 1], positions[3 * ic + 2]];
    area += norm(cross(sub(b, a), sub(c, a))) / 2;
  }
  return area;
}

/** Representative world point of an entity (its snapped/clicked location). */
export function entityPoint(entity) {
  if (!entity) return null;
  if (entity.point) return entity.point;
  if (entity.kind === 'circle') return entity.center;
  if (entity.kind === 'face') return entity.plane?.origin ?? null;
  return null;
}

/** Readout for a single selected entity: kind-specific measurements. */
export function measureSingle(entity) {
  if (!entity) return null;
  switch (entity.kind) {
    case 'vertex':
      return { kind: 'vertex', coord: entity.point };
    case 'edge':
      return { kind: 'edge', length: entity.length, a: entity.a, b: entity.b };
    case 'circle':
      return {
        kind: 'circle',
        radius: entity.radius,
        diameter: entity.radius * 2,
        center: entity.center,
      };
    case 'face':
      return { kind: 'face', area: entity.area, coord: entity.point };
    case 'point':
    default:
      return { kind: 'point', coord: entity.point };
  }
}

/** Signed perpendicular distance from a point to a plane `{ origin, normal }`. */
function pointToPlane(point, plane) {
  const n = normalize(plane.normal);
  if (!n) return null;
  return Math.abs(dot(sub(point, plane.origin), n));
}

/**
 * Readout for two selected entities. Always reports the straight-line distance
 * and its per-axis deltas between the two representative points. Adds, when the
 * kinds make it meaningful: face–face angle (+ parallel-plane gap), edge–edge
 * angle, and point/vertex-to-face perpendicular distance.
 */
export function measurePair(a, b) {
  const pa = entityPoint(a);
  const pb = entityPoint(b);
  if (!pa || !pb) return null;

  const delta = sub(pb, pa);
  const result = {
    distance: norm(delta),
    delta: [Math.abs(delta[0]), Math.abs(delta[1]), Math.abs(delta[2])],
  };

  if (a.kind === 'face' && b.kind === 'face') {
    const angle = acuteAngleDeg(a.plane.normal, b.plane.normal);
    result.angle = angle;
    // Parallel faces: report the perpendicular gap between the planes.
    if (angle != null && angle < 0.5) {
      result.gap = pointToPlane(b.plane.origin, a.plane);
    }
  } else if (a.kind === 'edge' && b.kind === 'edge') {
    result.angle = acuteAngleDeg(a.dir, b.dir);
  } else {
    // Point/vertex-to-face perpendicular distance (either order).
    const face = a.kind === 'face' ? a : b.kind === 'face' ? b : null;
    const other = face === a ? b : a;
    if (face && other.kind !== 'face') {
      const op = entityPoint(other);
      if (op) result.planeDistance = pointToPlane(op, face.plane);
    }
  }

  return result;
}
