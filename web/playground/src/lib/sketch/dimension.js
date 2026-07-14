/**
 * Smart-dimension inference: map a SolidWorks-style selection to a dimension.
 *
 * `inferDimension(sketch, picks)` is pure — it reads the sketch and a list of
 * picks (`{ kind: 'point' | 'entity', id }`) and returns one of:
 *   - `null`                      picks are valid but incomplete (keep picking)
 *   - `{ error }`                 the selection can't be dimensioned
 *   - `{ kind, proto, measured, anchor, needsPlacement }`
 *
 * `proto` is a driving constraint prototype at the measured value (drop it
 * straight into `addConstraint`, or set `driven: true` for a reference dim).
 * `needsPlacement` marks dimensions whose orientation depends on where the
 * cursor places the witness lines (point-to-point distances); the UI resolves
 * the orientation with `orientForPlacement` before committing.
 *
 * Selection → dimension map:
 *   line                     → length
 *   circle                   → diameter
 *   arc                      → radius
 *   two points               → distance (aligned/horizontal/vertical)
 *   point + line             → perpendicular distance
 *   two parallel lines       → distance between them
 *   two non-parallel lines   → angle
 */

import { entityRadius } from './model.js';

// Directions closer than this (normalized cross product) count as parallel.
const PARALLEL_TOL = 1e-6;

export function inferDimension(sketch, picks) {
  if (!picks || picks.length === 0) return null;
  const items = picks.map((pk) => resolve(sketch, pk));
  if (items.some((it) => it === null)) return { error: 'unknown selection' };

  if (items.length === 1) {
    const it = items[0];
    if (it.kind === 'point') return null; // a lone point needs a partner
    const e = it.entity;
    if (e.type === 'line') return lengthDim(sketch, e);
    if (e.type === 'circle') return diameterDim(sketch, e);
    if (e.type === 'arc') return radiusDim(sketch, e);
    return { error: 'cannot dimension this entity' };
  }

  if (items.length === 2) {
    const points = items.filter((it) => it.kind === 'point').map((it) => it.point);
    const lines = items
      .filter((it) => it.kind === 'entity' && it.entity.type === 'line')
      .map((it) => it.entity);
    if (points.length === 2) return distanceDim(sketch, points[0], points[1]);
    if (points.length === 1 && lines.length === 1) {
      return pdistanceDim(sketch, points[0], lines[0]);
    }
    if (lines.length === 2) return twoLineDim(sketch, lines[0], lines[1]);
    return { error: 'unsupported selection pair' };
  }

  return { error: 'too many selections' };
}

/**
 * Which distance a point-to-point dimension measures, given where the witness
 * lines are being placed (cursor at cx, cy):
 *   pull the text sideways   → vertical distance (Δy)
 *   pull it above/below      → horizontal distance (Δx)
 *   pull it diagonally       → aligned (straight-line) distance
 */
export function orientForPlacement(ax, ay, bx, by, cx, cy) {
  const mx = (ax + bx) / 2;
  const my = (ay + by) / 2;
  const vx = Math.abs(cx - mx);
  const vy = Math.abs(cy - my);
  if (vx < 1e-12 && vy < 1e-12) return 'aligned';
  const ratio = Math.min(vx, vy) / Math.max(vx, vy);
  if (ratio > 0.5) return 'aligned';
  return vx > vy ? 'vertical' : 'horizontal';
}

/** Rebuild a point-to-point distance dimension with an explicit orientation. */
export function distanceDim(sketch, a, b, orient = 'aligned') {
  const measured = distanceMeasure(a, b, orient);
  return {
    kind: 'distance',
    proto: { type: 'distance', a: a.id, b: b.id, value: measured, orient },
    measured,
    anchor: { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 },
    needsPlacement: true,
  };
}

/**
 * Live measured value of a dimension constraint (world units; radians for
 * angles). Used to show driven/reference dims and preview driving ones.
 */
export function measureConstraint(sketch, c) {
  switch (c.type) {
    case 'length': {
      const l = sketch.entities[c.line];
      if (!l) return null;
      const a = sketch.points[l.p1];
      const b = sketch.points[l.p2];
      return Math.hypot(b.x - a.x, b.y - a.y);
    }
    case 'radius': {
      const e = sketch.entities[c.entity];
      return e ? entityRadius(sketch, e) : null;
    }
    case 'diameter': {
      const e = sketch.entities[c.entity];
      return e ? entityRadius(sketch, e) * 2 : null;
    }
    case 'distance': {
      const a = sketch.points[c.a];
      const b = sketch.points[c.b];
      if (!a || !b) return null;
      return distanceMeasure(a, b, c.orient ?? 'aligned');
    }
    case 'pdistance': {
      const l = sketch.entities[c.line];
      const p = sketch.points[c.point];
      if (!l || !p) return null;
      return perpDistance(p, sketch.points[l.p1], sketch.points[l.p2]);
    }
    case 'angle': {
      const l1 = sketch.entities[c.a];
      const l2 = sketch.entities[c.b];
      if (!l1 || !l2) return null;
      return lineAngle(sketch, l1, l2);
    }
    default:
      return null;
  }
}

// ---- selection resolution -------------------------------------------------

function resolve(sketch, pk) {
  if (!pk) return null;
  if (pk.kind === 'point') {
    const point = sketch.points[pk.id];
    return point ? { kind: 'point', point } : null;
  }
  if (pk.kind === 'entity') {
    const entity = sketch.entities[pk.id];
    return entity ? { kind: 'entity', entity } : null;
  }
  return null;
}

// ---- per-kind builders ----------------------------------------------------

function lengthDim(sketch, e) {
  const a = sketch.points[e.p1];
  const b = sketch.points[e.p2];
  const measured = Math.hypot(b.x - a.x, b.y - a.y);
  return {
    kind: 'length',
    proto: { type: 'length', line: e.id, value: measured },
    measured,
    anchor: { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 },
    needsPlacement: false,
  };
}

function diameterDim(sketch, e) {
  const measured = entityRadius(sketch, e) * 2;
  const c = sketch.points[e.center];
  return {
    kind: 'diameter',
    proto: { type: 'diameter', entity: e.id, value: measured },
    measured,
    anchor: { x: c.x, y: c.y },
    needsPlacement: false,
  };
}

function radiusDim(sketch, e) {
  const measured = entityRadius(sketch, e);
  const c = sketch.points[e.center];
  return {
    kind: 'radius',
    proto: { type: 'radius', entity: e.id, value: measured },
    measured,
    anchor: { x: c.x, y: c.y },
    needsPlacement: false,
  };
}

function pdistanceDim(sketch, p, line) {
  const a = sketch.points[line.p1];
  const b = sketch.points[line.p2];
  const measured = perpDistance(p, a, b);
  const foot = footOnLine(p, a, b);
  return {
    kind: 'pdistance',
    proto: { type: 'pdistance', point: p.id, line: line.id, value: measured },
    measured,
    anchor: { x: (p.x + foot.x) / 2, y: (p.y + foot.y) / 2 },
    needsPlacement: false,
  };
}

function twoLineDim(sketch, l1, l2) {
  const d1 = dir(sketch, l1);
  const d2 = dir(sketch, l2);
  const cross = d1.x * d2.y - d1.y * d2.x;
  const n1 = Math.hypot(d1.x, d1.y);
  const n2 = Math.hypot(d2.x, d2.y);
  const parallel =
    n1 < 1e-12 || n2 < 1e-12 || Math.abs(cross) <= PARALLEL_TOL * n1 * n2;
  if (parallel) {
    // Distance between two parallel lines: perpendicular from an endpoint of
    // the first line to the (infinite) second line.
    const p = sketch.points[l1.p1];
    const a = sketch.points[l2.p1];
    const b = sketch.points[l2.p2];
    const measured = perpDistance(p, a, b);
    const foot = footOnLine(p, a, b);
    return {
      kind: 'pdistance',
      proto: { type: 'pdistance', point: l1.p1, line: l2.id, value: measured },
      measured,
      anchor: { x: (p.x + foot.x) / 2, y: (p.y + foot.y) / 2 },
      needsPlacement: false,
    };
  }
  const measured = lineAngle(sketch, l1, l2);
  const anchor = intersection(sketch, l1, l2) ?? midOfMidpoints(sketch, l1, l2);
  return {
    kind: 'angle',
    proto: { type: 'angle', a: l1.id, b: l2.id, value: measured },
    measured,
    anchor,
    needsPlacement: false,
  };
}

// ---- geometry helpers -----------------------------------------------------

function dir(sketch, line) {
  const a = sketch.points[line.p1];
  const b = sketch.points[line.p2];
  return { x: b.x - a.x, y: b.y - a.y };
}

function midpoint(sketch, line) {
  const a = sketch.points[line.p1];
  const b = sketch.points[line.p2];
  return { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 };
}

function midOfMidpoints(sketch, l1, l2) {
  const m1 = midpoint(sketch, l1);
  const m2 = midpoint(sketch, l2);
  return { x: (m1.x + m2.x) / 2, y: (m1.y + m2.y) / 2 };
}

/** Acute angle (0, π/2] between two undirected lines. */
function lineAngle(sketch, l1, l2) {
  const d1 = dir(sketch, l1);
  const d2 = dir(sketch, l2);
  let ang = Math.abs(
    Math.atan2(d1.x * d2.y - d1.y * d2.x, d1.x * d2.x + d1.y * d2.y)
  );
  if (ang > Math.PI / 2) ang = Math.PI - ang;
  return ang;
}

function distanceMeasure(a, b, orient) {
  if (orient === 'horizontal') return Math.abs(b.x - a.x);
  if (orient === 'vertical') return Math.abs(b.y - a.y);
  return Math.hypot(b.x - a.x, b.y - a.y);
}

function perpDistance(p, a, b) {
  let nx = -(b.y - a.y);
  let ny = b.x - a.x;
  const len = Math.hypot(nx, ny);
  if (len < 1e-12) return Math.hypot(p.x - a.x, p.y - a.y);
  return Math.abs(((p.x - a.x) * nx + (p.y - a.y) * ny) / len);
}

function footOnLine(p, a, b) {
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  const len2 = dx * dx + dy * dy;
  if (len2 < 1e-24) return { x: a.x, y: a.y };
  const t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2;
  return { x: a.x + dx * t, y: a.y + dy * t };
}

function intersection(sketch, l1, l2) {
  const a = sketch.points[l1.p1];
  const b = sketch.points[l1.p2];
  const c = sketch.points[l2.p1];
  const d = sketch.points[l2.p2];
  const rx = b.x - a.x;
  const ry = b.y - a.y;
  const sx = d.x - c.x;
  const sy = d.y - c.y;
  const denom = rx * sy - ry * sx;
  if (Math.abs(denom) < 1e-12) return null;
  const t = ((c.x - a.x) * sy - (c.y - a.y) * sx) / denom;
  return { x: a.x + rx * t, y: a.y + ry * t };
}
