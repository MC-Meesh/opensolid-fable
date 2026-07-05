/** Snapping and hit-testing over a sketch, all in world coordinates. */

import { entityRadius } from './model.js';
import {
  arcSweep,
  distToArc,
  distToCircle,
  distToSegment,
  normalizeAngle,
} from './geom.js';

/** Snap a coordinate pair to the nearest grid intersection. */
export function snapToGrid(x, y, spacing) {
  return {
    x: Math.round(x / spacing) * spacing,
    y: Math.round(y / spacing) * spacing,
  };
}

/**
 * Nearest sketch point within `maxDist`, or null.
 * `exclude` ids are skipped (e.g. the point being dragged).
 */
export function nearestPoint(sketch, x, y, maxDist, exclude = new Set()) {
  let best = null;
  for (const p of Object.values(sketch.points)) {
    if (exclude.has(p.id)) continue;
    const d = Math.hypot(p.x - x, p.y - y);
    if (d <= maxDist && (!best || d < best.dist)) {
      best = { id: p.id, x: p.x, y: p.y, dist: d };
    }
  }
  return best;
}

/**
 * Snap the segment end (x1,y1) to horizontal/vertical alignment with its
 * start when within `ratio` of the segment span.
 * Returns { x, y, axis: 'h' | 'v' | null }.
 */
export function axisAlign(x0, y0, x1, y1, ratio = 0.08) {
  const dx = x1 - x0;
  const dy = y1 - y0;
  const len = Math.hypot(dx, dy);
  if (len === 0) return { x: x1, y: y1, axis: null };
  if (Math.abs(dy) <= len * ratio) return { x: x1, y: y0, axis: 'h' };
  if (Math.abs(dx) <= len * ratio) return { x: x0, y: y1, axis: 'v' };
  return { x: x1, y: y1, axis: null };
}

/** Perpendicular distance from a world point to an entity's outline. */
export function distToEntity(sketch, entity, x, y) {
  const pts = sketch.points;
  switch (entity.type) {
    case 'line': {
      const a = pts[entity.p1];
      const b = pts[entity.p2];
      return distToSegment(x, y, a.x, a.y, b.x, b.y);
    }
    case 'circle': {
      const c = pts[entity.center];
      return distToCircle(x, y, c.x, c.y, entity.radius);
    }
    case 'arc': {
      const c = pts[entity.center];
      const p1 = pts[entity.p1];
      const p2 = pts[entity.p2];
      const r = entityRadius(sketch, entity);
      const start = normalizeAngle(Math.atan2(p1.y - c.y, p1.x - c.x));
      const end = normalizeAngle(Math.atan2(p2.y - c.y, p2.x - c.x));
      const sweep = arcSweep(start, end, entity.ccw);
      return distToArc(x, y, c.x, c.y, r, start, sweep, entity.ccw);
    }
    default:
      return Infinity;
  }
}

/**
 * Hit-test at a world position. Points win over entities within their
 * respective tolerances. Returns { kind: 'point' | 'entity', id } or null.
 */
export function hitTest(sketch, x, y, pointTol, entityTol = pointTol) {
  const point = nearestPoint(sketch, x, y, pointTol);
  if (point) return { kind: 'point', id: point.id };
  let best = null;
  for (const e of Object.values(sketch.entities)) {
    const d = distToEntity(sketch, e, x, y);
    if (d <= entityTol && (!best || d < best.dist)) {
      best = { kind: 'entity', id: e.id, dist: d };
    }
  }
  return best ? { kind: 'entity', id: best.id } : null;
}
