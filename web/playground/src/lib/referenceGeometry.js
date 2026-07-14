// Reference geometry (of-fsl.14): datum planes, axes, points, and coordinate
// systems — the construction scaffolding you build features against, the way
// SolidWorks reference geometry works. These are NOT Shapes: they carry no
// mesh and never enter the CSG tree. They live as parallel App-level state
// and thread into the sketch-plane picker, the feature tree, and the viewport
// as glyphs.
//
// Every constructor here is pure and returns plain, serializable data:
//
//   plane  { kind:'plane', reference:true, origin, normal, u, v, extent }
//   axis   { kind:'axis',  reference:true, origin, direction, extent }
//   point  { kind:'point', reference:true, position }
//   csys   { kind:'csys',  reference:true, origin, x, y, z }
//
// A reference PLANE is a strict superset of the face-plane shape that
// facePlane.js produces (`{ origin, normal, u, v, extent }` with unit
// u × v = normal), so it threads through planeToWorld / sketchView / sweep /
// Viewport3D unchanged. The `reference: true` flag lets callers tell a
// persistent datum plane apart from an ephemeral face pick (see
// isReferencePlane in lib/sketch/profile.js): a face pick is reset to XY when
// a new sketch opens, a reference plane is not.
//
// Kept free of three.js / React / WASM so it is unit-testable on plain arrays
// (same pattern as facePlane.js and persistentRef.js).

import { facePlaneBasis } from './facePlane.js';

/** Fallback half-size for a datum with no size to inherit (glyph sizing). */
export const DEFAULT_EXTENT = 5;

const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const scale = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
const norm = (a) => Math.hypot(a[0], a[1], a[2]);

function normalize(a, what = 'vector') {
  const n = norm(a);
  if (n < 1e-12) throw new Error(`${what} is zero-length`);
  return [a[0] / n, a[1] / n, a[2] / n];
}

/**
 * Build a reference plane from a point and a normal, deriving the (u, v)
 * in-plane basis with facePlane.js's convention so u × v = normal exactly —
 * the same basis a picked face carries, so the WYSIWYG sketch overlay lines
 * up. `extent` is the glyph half-size.
 */
export function planeFromPointNormal(origin, normal, extent = DEFAULT_EXTENT) {
  const n = normalize(normal, 'plane normal');
  const { u, v } = facePlaneBasis(n);
  return {
    kind: 'plane',
    reference: true,
    origin: [origin[0], origin[1], origin[2]],
    normal: n,
    u,
    v,
    extent,
  };
}

/** The three named world planes as reference-plane data, for use as bases. */
export const NAMED_PLANE_NORMALS = {
  XY: [0, 0, 1],
  XZ: [0, 1, 0],
  YZ: [1, 0, 0],
};

/**
 * Resolve a base-plane argument to a reference-plane object. Accepts either a
 * named world plane ('XY' | 'XZ' | 'YZ') or an existing plane object carrying
 * `{ origin, normal }` (a reference plane or a picked face plane).
 */
export function resolveBasePlane(base) {
  if (typeof base === 'string') {
    const normal = NAMED_PLANE_NORMALS[base];
    if (!normal) throw new Error(`unknown named plane: ${base}`);
    return planeFromPointNormal([0, 0, 0], normal);
  }
  if (base && Array.isArray(base.normal) && Array.isArray(base.origin)) {
    return planeFromPointNormal(base.origin, base.normal, base.extent ?? DEFAULT_EXTENT);
  }
  throw new Error('base plane must be a named plane or a plane object');
}

// --- Planes -----------------------------------------------------------------

/**
 * Plane parallel to `base`, shifted `distance` along its normal. Positive
 * distance moves along +normal; negative moves the other way. Reuses the
 * base's normal and extent.
 */
export function offsetPlane(base, distance) {
  const b = resolveBasePlane(base);
  const origin = add(b.origin, scale(b.normal, distance));
  return planeFromPointNormal(origin, b.normal, b.extent);
}

/**
 * Plane through `base` rotated by `angleRad` about an in-plane axis. The axis
 * defaults to the base plane's `u` direction (through its origin); pass
 * `{ point, direction }` to rotate about a specific in-plane line. The origin
 * stays put unless a rotation `point` is given, in which case the plane is
 * pivoted about that point.
 *
 * Angled datum planes hinge a plane off an edge — the rotation axis must lie
 * in the base plane, so `direction` is projected onto the plane and the
 * out-of-plane component dropped.
 */
export function angledPlane(base, angleRad, axis = null) {
  const b = resolveBasePlane(base);
  const axisDir = axis?.direction
    ? projectOntoPlane(normalize(axis.direction, 'rotation axis'), b.normal)
    : b.u;
  const k = normalize(axisDir, 'rotation axis (in-plane component)');
  const pivot = axis?.point ? [axis.point[0], axis.point[1], axis.point[2]] : b.origin;
  const normal = rotateAboutAxis(b.normal, k, angleRad);
  return planeFromPointNormal(pivot, normal, b.extent);
}

/**
 * Mid-plane halfway between two planes. When the planes are parallel (the
 * SolidWorks mid-plane case) the result is parallel to both, through the
 * midpoint of their origins. When they are not parallel the result bisects
 * the dihedral angle: normal is the normalized average of the two (flipped to
 * agree in orientation), through the midpoint of the origins.
 */
export function midPlane(planeA, planeB) {
  const a = resolveBasePlane(planeA);
  const b = resolveBasePlane(planeB);
  const origin = scale(add(a.origin, b.origin), 0.5);
  // Agree on orientation so parallel planes don't cancel to a zero normal.
  const bn = dot(a.normal, b.normal) < 0 ? scale(b.normal, -1) : b.normal;
  const avg = add(a.normal, bn);
  if (norm(avg) < 1e-9) {
    throw new Error('cannot build a mid-plane between opposed planes');
  }
  const extent = Math.max(a.extent, b.extent);
  return planeFromPointNormal(origin, avg, extent);
}

// --- Axes -------------------------------------------------------------------

function axisFrom(origin, direction, extent = DEFAULT_EXTENT) {
  const d = normalize(direction, 'axis direction');
  return {
    kind: 'axis',
    reference: true,
    origin: [origin[0], origin[1], origin[2]],
    direction: d,
    extent,
  };
}

/** Axis through two distinct points, directed from `p1` toward `p2`. */
export function axisFromTwoPoints(p1, p2) {
  const dir = sub(p2, p1);
  if (norm(dir) < 1e-12) throw new Error('axis endpoints are coincident');
  return axisFrom(p1, dir, norm(dir));
}

/** Axis from an anchor point and a direction vector. */
export function axisFromPointDirection(point, direction) {
  return axisFrom(point, direction);
}

/**
 * Axis along the intersection line of two planes. Direction is normal_a ×
 * normal_b; the anchor is the point on the line nearest each plane's origin
 * (the line's closest approach to the midpoint of the two origins). Throws
 * when the planes are parallel (no intersection line).
 */
export function axisFromPlaneIntersection(planeA, planeB) {
  const a = resolveBasePlane(planeA);
  const b = resolveBasePlane(planeB);
  const dir = cross(a.normal, b.normal);
  if (norm(dir) < 1e-9) throw new Error('planes are parallel — no intersection axis');
  const d = normalize(dir, 'intersection direction');
  // Plane constants: n·x = n·origin.
  const ca = dot(a.normal, a.origin);
  const cb = dot(b.normal, b.origin);
  // Point on the intersection line: solve in the plane spanned by the normals.
  // x = s·na + t·nb with na·x = ca, nb·x = cb.
  const naa = dot(a.normal, a.normal);
  const nab = dot(a.normal, b.normal);
  const nbb = dot(b.normal, b.normal);
  const det = naa * nbb - nab * nab;
  const s = (ca * nbb - cb * nab) / det;
  const t = (cb * naa - ca * nab) / det;
  const anchor = add(scale(a.normal, s), scale(b.normal, t));
  const extent = Math.max(a.extent, b.extent);
  return axisFrom(anchor, d, extent);
}

// --- Points -----------------------------------------------------------------

function pointAt(position) {
  return {
    kind: 'point',
    reference: true,
    position: [position[0], position[1], position[2]],
  };
}

/** Reference point at explicit world coordinates. */
export function pointFromCoords([x, y, z]) {
  return pointAt([x, y, z]);
}

/** Midpoint of two points. */
export function pointMidpoint(p1, p2) {
  return pointAt(scale(add(p1, p2), 0.5));
}

/**
 * Point where an axis pierces a plane. Throws when the axis is parallel to
 * the plane (no single pierce point).
 */
export function pointPlaneAxisPierce(plane, axis) {
  const p = resolveBasePlane(plane);
  const o = axis.origin;
  const d = normalize(axis.direction, 'axis direction');
  const denom = dot(p.normal, d);
  if (Math.abs(denom) < 1e-9) throw new Error('axis is parallel to the plane');
  const s = dot(p.normal, sub(p.origin, o)) / denom;
  return pointAt(add(o, scale(d, s)));
}

// --- Coordinate systems -----------------------------------------------------

/**
 * Coordinate system whose (x, y, z) are a plane's (u, v, normal). An optional
 * `origin` overrides the plane's origin (e.g. anchored at a reference point).
 */
export function csysFromPlane(plane, origin = null) {
  const p = resolveBasePlane(plane);
  return {
    kind: 'csys',
    reference: true,
    origin: origin ? [origin[0], origin[1], origin[2]] : [...p.origin],
    x: [...p.u],
    y: [...p.v],
    z: [...p.normal],
  };
}

/**
 * Coordinate system from an origin, a primary axis direction (x), and a
 * secondary hint that fixes the xy-plane. `y` is the hint made orthogonal to
 * `x` (Gram-Schmidt); `z = x × y`. Throws when the two directions are
 * parallel (the plane is undefined).
 */
export function csysFromPointAxes(origin, xDir, yHint) {
  const x = normalize(xDir, 'x direction');
  const yRaw = projectOntoPlane(yHint, x);
  if (norm(yRaw) < 1e-9) throw new Error('x and y directions are parallel');
  const y = normalize(yRaw, 'y direction');
  const z = cross(x, y);
  return {
    kind: 'csys',
    reference: true,
    origin: [origin[0], origin[1], origin[2]],
    x,
    y,
    z,
  };
}

// --- Shared vector helpers (exported for reuse/tests) -----------------------

/** Component of `v` orthogonal to unit `axis` (drops the axis-parallel part). */
export function projectOntoPlane(v, axis) {
  const a = norm(axis) > 1 + 1e-9 || norm(axis) < 1 - 1e-9 ? normalize(axis, 'axis') : axis;
  const d = dot(v, a);
  return sub(v, scale(a, d));
}

/** Rotate `v` about unit `axis` by `angleRad` (Rodrigues' rotation). */
export function rotateAboutAxis(v, axis, angleRad) {
  const k = normalize(axis, 'rotation axis');
  const c = Math.cos(angleRad);
  const s = Math.sin(angleRad);
  const term1 = scale(v, c);
  const term2 = scale(cross(k, v), s);
  const term3 = scale(k, dot(k, v) * (1 - c));
  return add(add(term1, term2), term3);
}

// --- Form dispatch ----------------------------------------------------------

/**
 * Build a reference-geometry entry `{ kind, geom }` from a creation method and
 * its already-resolved parameters. This is the single pure bridge between the
 * ReferencePanel form and the constructors above — the panel gathers numbers
 * and base selections, this validates and constructs. Angles arrive in
 * DEGREES (what the panel shows) and are converted here. Throws the
 * constructor's error (bad geometry: parallel planes, coincident points, …)
 * so the panel can surface it inline.
 */
export function buildReference(method, p = {}) {
  switch (method) {
    case 'plane-offset':
      return { kind: 'plane', geom: offsetPlane(p.base, p.distance) };
    case 'plane-angled':
      return {
        kind: 'plane',
        geom: angledPlane(p.base, (p.angleDeg * Math.PI) / 180, p.axisDir ? { direction: p.axisDir } : null),
      };
    case 'plane-mid':
      return { kind: 'plane', geom: midPlane(p.base, p.base2) };
    case 'axis-2pt':
      return { kind: 'axis', geom: axisFromTwoPoints(p.p1, p.p2) };
    case 'axis-ptdir':
      return { kind: 'axis', geom: axisFromPointDirection(p.point, p.direction) };
    case 'axis-intersect':
      return { kind: 'axis', geom: axisFromPlaneIntersection(p.base, p.base2) };
    case 'point-coords':
      return { kind: 'point', geom: pointFromCoords(p.coords) };
    case 'point-mid':
      return { kind: 'point', geom: pointMidpoint(p.p1, p.p2) };
    case 'csys-plane':
      return { kind: 'csys', geom: csysFromPlane(p.base) };
    case 'csys-ptaxes':
      return { kind: 'csys', geom: csysFromPointAxes(p.origin, p.xDir, p.yHint) };
    default:
      throw new Error(`unknown reference method: ${method}`);
  }
}
