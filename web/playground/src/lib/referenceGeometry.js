// Reference geometry (of-fsl.14): user-created datum planes, axes, points,
// and coordinate systems — SolidWorks-style reference geometry the model can
// sketch and build off of, beyond the three fixed standard planes.
//
// Reference geometry is NOT a Shape: it produces no solid and so lives outside
// the script/traced-tree source of truth. This module is the pure geometry
// layer — plain-data constructors kept free of React and three.js so they
// unit-test in isolation (same pattern as facePlane.js / sketchView.js). App
// owns the collection (a parallel list, like featureNames/hiddenKeys) and
// assigns each entity a stable id.
//
// Entity shapes (all plain data):
//   plane: { kind:'plane', method, origin, normal, u, v, extent }
//          — a strict superset of a face plane (lib/facePlane.js), so a
//          reference plane threads through planeToWorld / sketchViewPose /
//          sweep / the Viewport3D indicator with no special-casing.
//   axis:  { kind:'axis',  method, origin, direction, length }
//   point: { kind:'point', method, position }
//   csys:  { kind:'csys',  method, origin, x, y, z }
//
// A named standard plane ('XY' | 'XZ' | 'YZ') or a face-plane object can be
// used anywhere a base plane is expected — resolvePlane() normalizes both.

import { planeNormal, planeToWorld } from './sketch/profile.js';

// --- tiny vector helpers (per-file, matching facePlane.js's convention) ----
const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const scale = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
const norm = (a) => Math.hypot(a[0], a[1], a[2]);

function normalize(a) {
  const n = norm(a);
  if (n <= 1e-12) throw new Error('cannot normalize a zero-length vector');
  return [a[0] / n, a[1] / n, a[2] / n];
}

/** Rodrigues rotation of `v` about unit `axis` by `angle` radians. */
export function rotateAboutAxis(v, axis, angle) {
  const k = normalize(axis);
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  const kv = cross(k, v);
  const kd = dot(k, v) * (1 - c);
  return [
    v[0] * c + kv[0] * s + k[0] * kd,
    v[1] * c + kv[1] * s + k[1] * kd,
    v[2] * c + kv[2] * s + k[2] * kd,
  ];
}

/** Default in-plane extent for a fresh reference plane's viewport indicator. */
const DEFAULT_PLANE_EXTENT = 5;

/**
 * Normalize any plane reference — a named standard plane string
 * ('XY' | 'XZ' | 'YZ'), a picked face plane, or a reference plane — into a
 * bare basis `{ origin, normal, u, v }` with unit `u × v = normal`. Named
 * planes reuse profile.js's (u, v) convention so the basis is identical to
 * what sketching and sweeping already assume.
 */
export function resolvePlane(plane) {
  if (plane && typeof plane === 'object') {
    const { origin, normal, u, v } = plane;
    if (!origin || !normal || !u || !v) {
      throw new Error('plane object must carry origin, normal, u, and v');
    }
    return { origin, normal, u, v };
  }
  const origin = planeToWorld(plane, 0, 0);
  const u = sub(planeToWorld(plane, 1, 0), origin);
  const v = sub(planeToWorld(plane, 0, 1), origin);
  return { origin, normal: planeNormal(plane), u, v };
}

/** Assemble a reference-plane entity, defaulting the indicator extent. */
function planeEntity(method, { origin, normal, u, v }, extent) {
  return {
    kind: 'plane',
    method,
    origin,
    normal,
    u,
    v,
    extent: extent ?? DEFAULT_PLANE_EXTENT,
  };
}

/**
 * A reference plane parallel to `base`, shifted `distance` along its normal
 * (SolidWorks "offset" plane). Positive distance moves along +normal. The
 * (u, v) basis is inherited so the offset plane sketches in the same frame.
 */
export function offsetPlane(base, distance) {
  const { origin, normal, u, v } = resolvePlane(base);
  const n = normalize(normal);
  return planeEntity('offset', {
    origin: add(origin, scale(n, distance)),
    normal: n,
    u,
    v,
  });
}

/**
 * A reference plane tilted `angleDeg` about one of `base`'s in-plane axes
 * through its origin (SolidWorks "at angle" plane). `hinge` picks the axis
 * held fixed: 'u' (default) rotates v and the normal about u; 'v' rotates u
 * and the normal about v. The hinge axis stays in the plane, so the result
 * shares an edge with the base.
 */
export function angledPlane(base, angleDeg, hinge = 'u') {
  const { origin, normal, u, v } = resolvePlane(base);
  const angle = (angleDeg * Math.PI) / 180;
  const n = normalize(normal);
  if (hinge === 'v') {
    return planeEntity('angled', {
      origin,
      normal: rotateAboutAxis(n, v, angle),
      u: rotateAboutAxis(u, v, angle),
      v,
    });
  }
  return planeEntity('angled', {
    origin,
    normal: rotateAboutAxis(n, u, angle),
    u,
    v: rotateAboutAxis(v, u, angle),
  });
}

/**
 * The mid-plane parallel to two parallel planes `a` and `b`, positioned
 * halfway between them along `a`'s normal. Throws when the planes are not
 * parallel (their normals must be collinear).
 */
export function midPlane(a, b) {
  const pa = resolvePlane(a);
  const pb = resolvePlane(b);
  const na = normalize(pa.normal);
  const nb = normalize(pb.normal);
  if (norm(cross(na, nb)) > 1e-9) {
    throw new Error('mid-plane needs two parallel planes');
  }
  const gap = dot(sub(pb.origin, pa.origin), na);
  return planeEntity('mid', {
    origin: add(pa.origin, scale(na, gap / 2)),
    normal: na,
    u: pa.u,
    v: pa.v,
  });
}

/** Assemble a reference-axis entity. */
function axisEntity(method, origin, direction, length) {
  return { kind: 'axis', method, origin, direction: normalize(direction), length };
}

/**
 * A reference axis through two distinct points, directed from `p1` to `p2`.
 * `length` records the point separation (used to size the drawn segment).
 */
export function axisFromTwoPoints(p1, p2) {
  const d = sub(p2, p1);
  const len = norm(d);
  if (len <= 1e-12) throw new Error('axis needs two distinct points');
  return axisEntity('two-points', [...p1], d, len);
}

/** A reference axis at `origin` along `direction`. */
export function axisFromPointDirection(origin, direction) {
  return axisEntity('point-direction', [...origin], direction, norm(direction) || 1);
}

/**
 * The line where two non-parallel planes meet, as a reference axis. The
 * direction is `nA × nB`; the origin is the point on that line nearest the
 * world origin. Throws when the planes are parallel.
 */
export function axisFromPlaneIntersection(a, b) {
  const pa = resolvePlane(a);
  const pb = resolvePlane(b);
  const nA = normalize(pa.normal);
  const nB = normalize(pb.normal);
  const dir = cross(nA, nB);
  const denom = dot(dir, dir);
  if (denom <= 1e-12) throw new Error('axis needs two non-parallel planes');
  // Plane offsets: nA·x = dA, nB·x = dB. The point on the intersection line
  // closest to the origin is (dA (nB×dir) + dB (dir×nA)) / |dir|^2.
  const dA = dot(nA, pa.origin);
  const dB = dot(nB, pb.origin);
  const origin = scale(
    add(scale(cross(nB, dir), dA), scale(cross(dir, nA), dB)),
    1 / denom
  );
  return axisEntity('plane-intersection', origin, dir, 1);
}

/** A reference point at explicit world coordinates. */
export function pointFromCoords(position) {
  return { kind: 'point', method: 'coords', position: [...position] };
}

/** A reference point at the midpoint of two points. */
export function pointFromMidpoint(p1, p2) {
  return {
    kind: 'point',
    method: 'midpoint',
    position: scale(add(p1, p2), 0.5),
  };
}

/**
 * The reference point where an axis pierces a plane. Throws when the axis is
 * parallel to the plane (no single intersection).
 */
export function pointAtAxisPlane(axis, plane) {
  const p = resolvePlane(plane);
  const n = normalize(p.normal);
  const denom = dot(n, axis.direction);
  if (Math.abs(denom) <= 1e-12) {
    throw new Error('axis is parallel to the plane');
  }
  const t = dot(n, sub(p.origin, axis.origin)) / denom;
  return {
    kind: 'point',
    method: 'axis-plane',
    position: add(axis.origin, scale(axis.direction, t)),
  };
}

/**
 * A right-handed coordinate system at `origin` with X along `xDir` and Y in
 * the half-plane of `yHint` (Gram-Schmidt: Z = X × Y_hint, Y = Z × X). Throws
 * when the two directions are parallel.
 */
export function csysFromPointAndAxes(origin, xDir, yHint) {
  const x = normalize(xDir);
  const z = cross(x, yHint);
  if (norm(z) <= 1e-12) {
    throw new Error('coordinate system needs non-parallel X and Y directions');
  }
  const zn = normalize(z);
  return {
    kind: 'csys',
    method: 'point-axes',
    origin: [...origin],
    x,
    y: cross(zn, x),
    z: zn,
  };
}

/** A coordinate system aligned to a plane: X = u, Y = v, Z = normal. */
export function csysFromPlane(plane) {
  const p = resolvePlane(plane);
  return {
    kind: 'csys',
    method: 'plane',
    origin: [...p.origin],
    x: normalize(p.u),
    y: normalize(p.v),
    z: normalize(p.normal),
  };
}

/** Display metadata per reference-geometry kind (feature tree + panel). */
export const REFERENCE_META = {
  plane: { type: 'Plane', label: 'Plane' },
  axis: { type: 'Axis', label: 'Axis' },
  point: { type: 'Point', label: 'Point' },
  csys: { type: 'CSys', label: 'Coordinate System' },
};

/**
 * Default SolidWorks-style name for a new entity of `kind` given the names
 * already taken (e.g. 'Plane1', 'Axis2'). Numbers from the highest existing
 * ordinal so deletes don't cause reuse collisions.
 */
export function defaultReferenceName(kind, existingNames = []) {
  const { type } = REFERENCE_META[kind] ?? { type: kind };
  const re = new RegExp(`^${type}(\\d+)$`);
  let max = 0;
  for (const name of existingNames) {
    const m = re.exec(name);
    if (m) max = Math.max(max, Number(m[1]));
  }
  return `${type}${max + 1}`;
}

/** Is `plane` a persistent reference plane (vs. an ephemeral face pick)? */
export function isReferencePlane(plane) {
  return Boolean(plane) && typeof plane === 'object' && plane.kind === 'plane';
}
