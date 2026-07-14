/**
 * Sketch edit tools: offset, trim, extend, and convert-entities.
 *
 * These are the SolidWorks "Sketch → Edit" verbs that reshape existing
 * geometry rather than draw new geometry from scratch. Everything here is a
 * pure mutation of the plain sketch model (see model.js) plus a small library
 * of analytic 2D intersection helpers, so it is fully unit-testable without
 * the canvas.
 *
 *   offsetEntity(sketch, id, dist)    → parallel copy at signed distance
 *   trimEntityAt(sketch, id, x, y)    → power-trim the piece under (x, y)
 *   extendEntityAt(sketch, id, x, y)  → extend the near end to the next crossing
 *   convertEntities(sketch, loops)    → project outside geometry onto the sketch
 *
 * Trim/extend need the crossings between the target entity and the rest of the
 * sketch; `entityIntersections` computes them analytically (line/circle/arc in
 * any pairing) and each verb filters those crossings by the target's own
 * extent (segment span for lines, arc sweep for arcs).
 */

import {
  addPoint,
  addLine,
  addArc,
  addCircle,
  deleteEntity,
  entityRadius,
  constraintRefs,
} from './model.js';
import { normalizeAngle, arcSweep, angleOnArc } from './geom.js';

const EPS = 1e-9;

// ---- geometric shape of an entity (extent-independent) --------------------

/** Underlying analytic shape of an entity, in world coordinates. */
function entityShape(sketch, entity) {
  const pts = sketch.points;
  switch (entity.type) {
    case 'line': {
      const a = pts[entity.p1];
      const b = pts[entity.p2];
      return { kind: 'line', a: [a.x, a.y], b: [b.x, b.y] };
    }
    case 'circle': {
      const c = pts[entity.center];
      return { kind: 'circle', c: [c.x, c.y], r: entity.radius };
    }
    case 'arc': {
      const c = pts[entity.center];
      return { kind: 'circle', c: [c.x, c.y], r: entityRadius(sketch, entity) };
    }
    default:
      return null;
  }
}

// ---- analytic curve/curve intersection (ignores extent) -------------------

/** Intersection of two infinite lines through (a→b) and (c→d), or []. */
function infLineInfLine([ax, ay], [bx, by], [cx, cy], [dx, dy]) {
  const r = [bx - ax, by - ay];
  const s = [dx - cx, dy - cy];
  const denom = r[0] * s[1] - r[1] * s[0];
  if (Math.abs(denom) < EPS) return []; // parallel or coincident
  const t = ((cx - ax) * s[1] - (cy - ay) * s[0]) / denom;
  return [[ax + t * r[0], ay + t * r[1]]];
}

/** Intersections of the infinite line (a→b) with a circle (center, r). */
function infLineCircle([ax, ay], [bx, by], [cx, cy], r) {
  const dx = bx - ax;
  const dy = by - ay;
  const len2 = dx * dx + dy * dy;
  if (len2 < EPS) return [];
  // Project circle center onto the line; foot at parameter t0.
  const t0 = ((cx - ax) * dx + (cy - ay) * dy) / len2;
  const fx = ax + t0 * dx;
  const fy = ay + t0 * dy;
  const d2 = (fx - cx) ** 2 + (fy - cy) ** 2;
  const rr = r * r;
  if (d2 > rr + EPS) return [];
  const h = Math.sqrt(Math.max(0, rr - d2));
  const len = Math.sqrt(len2);
  const ux = dx / len;
  const uy = dy / len;
  if (h < EPS) return [[fx, fy]];
  return [
    [fx - h * ux, fy - h * uy],
    [fx + h * ux, fy + h * uy],
  ];
}

/** Intersections of two circles, or []. */
function circleCircle([c0x, c0y], r0, [c1x, c1y], r1) {
  const dx = c1x - c0x;
  const dy = c1y - c0y;
  const d = Math.hypot(dx, dy);
  if (d < EPS) return []; // concentric
  if (d > r0 + r1 + EPS || d < Math.abs(r0 - r1) - EPS) return [];
  const a = (r0 * r0 - r1 * r1 + d * d) / (2 * d);
  const h2 = r0 * r0 - a * a;
  const px = c0x + (a * dx) / d;
  const py = c0y + (a * dy) / d;
  if (h2 <= EPS) return [[px, py]];
  const h = Math.sqrt(h2);
  const ox = (-dy * h) / d;
  const oy = (dx * h) / d;
  return [
    [px + ox, py + oy],
    [px - ox, py - oy],
  ];
}

/** Candidate crossing points of two entity shapes (extent ignored). */
function shapeIntersections(sa, sb) {
  if (!sa || !sb) return [];
  if (sa.kind === 'line' && sb.kind === 'line') {
    return infLineInfLine(sa.a, sa.b, sb.a, sb.b);
  }
  if (sa.kind === 'line' && sb.kind === 'circle') {
    return infLineCircle(sa.a, sa.b, sb.c, sb.r);
  }
  if (sa.kind === 'circle' && sb.kind === 'line') {
    return infLineCircle(sb.a, sb.b, sa.c, sa.r);
  }
  return circleCircle(sa.c, sa.r, sb.c, sb.r);
}

// ---- extent tests (is a point on the drawn part of an entity?) ------------

/** Parameter t of (x, y) projected onto a line's p1→p2 (0 = p1, 1 = p2). */
function lineParam(sketch, entity, x, y) {
  const a = sketch.points[entity.p1];
  const b = sketch.points[entity.p2];
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  const len2 = dx * dx + dy * dy;
  if (len2 < EPS) return 0;
  return ((x - a.x) * dx + (y - a.y) * dy) / len2;
}

/** Whether (x, y) lies on the drawn extent of `entity` (within eps). */
function onEntityExtent(sketch, entity, x, y, eps = 1e-6) {
  switch (entity.type) {
    case 'line': {
      const t = lineParam(sketch, entity, x, y);
      return t >= -eps && t <= 1 + eps;
    }
    case 'circle':
      return true;
    case 'arc': {
      const c = sketch.points[entity.center];
      const p1 = sketch.points[entity.p1];
      const p2 = sketch.points[entity.p2];
      const start = normalizeAngle(Math.atan2(p1.y - c.y, p1.x - c.x));
      const end = normalizeAngle(Math.atan2(p2.y - c.y, p2.x - c.x));
      const sweep = arcSweep(start, end, entity.ccw);
      const ang = Math.atan2(y - c.y, x - c.x);
      return angleOnArc(ang, start, sweep, entity.ccw);
    }
    default:
      return false;
  }
}

/**
 * World-space crossing points between `entity` and every other entity in the
 * sketch, filtered to lie on the drawn extent of both. De-duplicated.
 */
export function entityIntersections(sketch, id) {
  const entity = sketch.entities[id];
  if (!entity) return [];
  const shape = entityShape(sketch, entity);
  const out = [];
  for (const other of Object.values(sketch.entities)) {
    if (other.id === id) continue;
    const points = shapeIntersections(shape, entityShape(sketch, other));
    for (const [x, y] of points) {
      if (!onEntityExtent(sketch, entity, x, y)) continue;
      if (!onEntityExtent(sketch, other, x, y)) continue;
      if (out.some((p) => Math.hypot(p[0] - x, p[1] - y) < 1e-7)) continue;
      out.push([x, y]);
    }
  }
  return out;
}

// ---- offset ---------------------------------------------------------------

/**
 * Create a parallel copy of `id` at signed perpendicular distance `dist`
 * (positive = left of a line's p1→p2 direction; positive grows a circle/arc
 * radius). Returns the new entity id, or null if the offset is degenerate
 * (e.g. a circle radius driven to <= 0). Points are fresh copies, so the
 * offset is independent of the original.
 */
export function offsetEntity(sketch, id, dist) {
  const entity = sketch.entities[id];
  if (!entity || !Number.isFinite(dist) || dist === 0) return null;
  switch (entity.type) {
    case 'line': {
      const a = sketch.points[entity.p1];
      const b = sketch.points[entity.p2];
      const dx = b.x - a.x;
      const dy = b.y - a.y;
      const len = Math.hypot(dx, dy);
      if (len < EPS) return null;
      const nx = -dy / len; // left normal
      const ny = dx / len;
      const p1 = addPoint(sketch, a.x + nx * dist, a.y + ny * dist);
      const p2 = addPoint(sketch, b.x + nx * dist, b.y + ny * dist);
      return addLine(sketch, p1, p2);
    }
    case 'circle': {
      const c = sketch.points[entity.center];
      const r = entity.radius + dist;
      if (r <= EPS) return null;
      const center = addPoint(sketch, c.x, c.y);
      return addCircle(sketch, center, r);
    }
    case 'arc': {
      const c = sketch.points[entity.center];
      const r0 = entityRadius(sketch, entity);
      const r = r0 + dist;
      if (r <= EPS) return null;
      const p1 = sketch.points[entity.p1];
      const p2 = sketch.points[entity.p2];
      const a1 = Math.atan2(p1.y - c.y, p1.x - c.x);
      const a2 = Math.atan2(p2.y - c.y, p2.x - c.x);
      const center = addPoint(sketch, c.x, c.y);
      const s1 = addPoint(sketch, c.x + r * Math.cos(a1), c.y + r * Math.sin(a1));
      const s2 = addPoint(sketch, c.x + r * Math.cos(a2), c.y + r * Math.sin(a2));
      return addArc(sketch, center, s1, s2, entity.ccw);
    }
    default:
      return null;
  }
}

// ---- trim -----------------------------------------------------------------

/** Drop length/radius dimension constraints on an entity whose size changed. */
function dropSizingConstraints(sketch, id) {
  for (const c of Object.values(sketch.constraints)) {
    if (
      (c.type === 'length' || c.type === 'radius') &&
      constraintRefs(c).includes(id)
    ) {
      delete sketch.constraints[c.id];
    }
  }
}

/**
 * Power-trim: remove the portion of entity `id` under the world pick (x, y),
 * bounded by its nearest crossings with other geometry. A line keeps its outer
 * pieces (0–2 remaining segments); a circle becomes the complementary arc; an
 * arc keeps its outer sub-arcs. With no bracketing crossings the whole entity
 * is deleted. Returns the ids of the surviving entities (possibly empty).
 */
export function trimEntityAt(sketch, id, x, y) {
  const entity = sketch.entities[id];
  if (!entity) return [];
  const crossings = entityIntersections(sketch, id);
  if (entity.type === 'line') return trimLine(sketch, entity, crossings, x, y);
  if (entity.type === 'circle') {
    return trimCircle(sketch, entity, crossings, x, y);
  }
  if (entity.type === 'arc') return trimArc(sketch, entity, crossings, x, y);
  return [id];
}

function trimLine(sketch, entity, crossings, x, y) {
  const a = sketch.points[entity.p1];
  const b = sketch.points[entity.p2];
  const tPick = clamp01(lineParam(sketch, entity, x, y));
  const ts = crossings
    .map(([cx, cy]) => lineParam(sketch, entity, cx, cy))
    .filter((t) => t > EPS && t < 1 - EPS)
    .sort((m, n) => m - n);
  let lo = 0;
  let hi = 1;
  for (const t of ts) {
    if (t <= tPick && t > lo) lo = t;
    if (t >= tPick && t < hi) hi = t;
  }
  const at = (t) => [a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t];
  const keepLow = lo > EPS; // piece 0 → lo survives
  const keepHigh = hi < 1 - EPS; // piece hi → 1 survives
  dropSizingConstraints(sketch, entity.id);
  if (!keepLow && !keepHigh) {
    deleteEntity(sketch, entity.id);
    return [];
  }
  if (keepLow && keepHigh) {
    // Reuse the original entity for the low piece (p1 → cut@lo); add a fresh
    // line for the high piece (cut@hi → original p2).
    const originalP2 = entity.p2;
    const [lx, ly] = at(lo);
    entity.p2 = addPoint(sketch, lx, ly);
    const [hx, hy] = at(hi);
    const cutHigh = addPoint(sketch, hx, hy);
    return [entity.id, addLine(sketch, cutHigh, originalP2)];
  }
  if (keepLow) {
    const [lx, ly] = at(lo);
    entity.p2 = addPoint(sketch, lx, ly);
    return [entity.id];
  }
  // keepHigh only: shift p1 up to the hi cut.
  const [hx, hy] = at(hi);
  entity.p1 = addPoint(sketch, hx, hy);
  return [entity.id];
}

function trimCircle(sketch, entity, crossings, x, y) {
  const c = sketch.points[entity.center];
  const r = entity.radius;
  const angs = crossings
    .map(([cx, cy]) => normalizeAngle(Math.atan2(cy - c.y, cx - c.x)))
    .sort((m, n) => m - n);
  if (angs.length < 2) return [entity.id]; // nothing bounds a removal
  const pick = normalizeAngle(Math.atan2(y - c.y, x - c.x));
  // Find the adjacent pair of crossings bracketing the pick (wrapping).
  const [lo, hi] = bracketAngle(angs, pick);
  // Keep the complementary arc: from `hi` around to `lo` (ccw). Delete the
  // circle first (it prunes orphan points), then materialize the arc's points.
  dropSizingConstraints(sketch, entity.id);
  deleteEntity(sketch, entity.id);
  const center = addPoint(sketch, c.x, c.y);
  const s1 = addPoint(sketch, c.x + r * Math.cos(hi), c.y + r * Math.sin(hi));
  const s2 = addPoint(sketch, c.x + r * Math.cos(lo), c.y + r * Math.sin(lo));
  return [addArc(sketch, center, s1, s2, true)];
}

function trimArc(sketch, entity, crossings, x, y) {
  const c = sketch.points[entity.center];
  const r = entityRadius(sketch, entity);
  const p1 = sketch.points[entity.p1];
  const p2 = sketch.points[entity.p2];
  const start = normalizeAngle(Math.atan2(p1.y - c.y, p1.x - c.x));
  const end = normalizeAngle(Math.atan2(p2.y - c.y, p2.x - c.x));
  const sweep = arcSweep(start, end, entity.ccw);
  // Relative sweep offset (0..sweep) of an absolute angle along this arc.
  const rel = (ang) =>
    entity.ccw
      ? normalizeAngle(ang - start)
      : normalizeAngle(start - ang);
  const cuts = crossings
    .map(([cx, cy]) => rel(Math.atan2(cy - c.y, cx - c.x)))
    .filter((t) => t > EPS && t < sweep - EPS)
    .sort((m, n) => m - n);
  const pick = clampRange(rel(Math.atan2(y - c.y, x - c.x)), 0, sweep);
  let lo = 0;
  let hi = sweep;
  for (const t of cuts) {
    if (t <= pick && t > lo) lo = t;
    if (t >= pick && t < hi) hi = t;
  }
  const dir = entity.ccw ? 1 : -1;
  const absAt = (t) => start + dir * t;
  const ptAt = (t) => {
    const ang = absAt(t);
    return addPoint(sketch, c.x + r * Math.cos(ang), c.y + r * Math.sin(ang));
  };
  const keepLow = lo > EPS;
  const keepHigh = hi < sweep - EPS;
  dropSizingConstraints(sketch, entity.id);
  if (!keepLow && !keepHigh) {
    deleteEntity(sketch, entity.id);
    return [];
  }
  const survivors = [];
  if (keepLow) {
    survivors.push(
      addArc(sketch, addPoint(sketch, c.x, c.y), entity.p1, ptAt(lo), entity.ccw)
    );
  }
  if (keepHigh) {
    survivors.push(
      addArc(sketch, addPoint(sketch, c.x, c.y), ptAt(hi), entity.p2, entity.ccw)
    );
  }
  deleteEntity(sketch, entity.id);
  return survivors;
}

// ---- extend ---------------------------------------------------------------

/**
 * Extend the endpoint of line/arc `id` nearer the pick (x, y) outward until it
 * reaches the next crossing with another entity's drawn extent. Moves the
 * endpoint in place and returns true, or false if there is nothing to meet.
 */
export function extendEntityAt(sketch, id, x, y) {
  const entity = sketch.entities[id];
  if (!entity) return false;
  if (entity.type === 'line') return extendLine(sketch, entity, x, y);
  if (entity.type === 'arc') return extendArc(sketch, entity, x, y);
  return false;
}

function extendLine(sketch, entity, x, y) {
  const a = sketch.points[entity.p1];
  const b = sketch.points[entity.p2];
  // Which end is nearer the pick? Extend along the outward direction.
  const nearP1 = Math.hypot(x - a.x, y - a.y) < Math.hypot(x - b.x, y - b.y);
  const shape = entityShape(sketch, entity);
  let best = null; // { t, point }
  for (const other of Object.values(sketch.entities)) {
    if (other.id === entity.id) continue;
    const points = shapeIntersections(shape, entityShape(sketch, other));
    for (const p of points) {
      if (!onEntityExtent(sketch, other, p[0], p[1])) continue;
      const t = lineParam(sketch, entity, p[0], p[1]);
      // Extending p1 means t < 0; extending p2 means t > 1.
      const past = nearP1 ? -t : t - 1;
      if (past <= EPS) continue;
      if (!best || past < best.past) best = { past, point: p };
    }
  }
  if (!best) return false;
  const end = sketch.points[nearP1 ? entity.p1 : entity.p2];
  end.x = best.point[0];
  end.y = best.point[1];
  dropSizingConstraints(sketch, entity.id);
  return true;
}

function extendArc(sketch, entity, x, y) {
  const c = sketch.points[entity.center];
  const r = entityRadius(sketch, entity);
  const p1 = sketch.points[entity.p1];
  const p2 = sketch.points[entity.p2];
  const start = normalizeAngle(Math.atan2(p1.y - c.y, p1.x - c.x));
  const end = normalizeAngle(Math.atan2(p2.y - c.y, p2.x - c.x));
  const sweep = arcSweep(start, end, entity.ccw);
  const dir = entity.ccw ? 1 : -1;
  // Extend the end nearer the pick: negative (before start) or positive
  // (after end) relative sweep offset.
  const pickRel = entity.ccw
    ? normalizeAngle(Math.atan2(y - c.y, x - c.x) - start)
    : normalizeAngle(start - Math.atan2(y - c.y, x - c.x));
  const nearStart = pickRel > sweep / 2 ? false : true;
  const shape = entityShape(sketch, entity);
  let best = null; // { past, ang }
  for (const other of Object.values(sketch.entities)) {
    if (other.id === entity.id) continue;
    const points = shapeIntersections(shape, entityShape(sketch, other));
    for (const p of points) {
      if (!onEntityExtent(sketch, other, p[0], p[1])) continue;
      const ang = Math.atan2(p[1] - c.y, p[0] - c.x);
      const rel = entity.ccw
        ? normalizeAngle(ang - start)
        : normalizeAngle(start - ang);
      // Distance past the near endpoint, going outward around the circle.
      const past = nearStart
        ? 2 * Math.PI - rel // before start (wrapping backwards)
        : rel - sweep; // after end
      if (past <= EPS || past >= 2 * Math.PI - EPS) continue;
      if (!best || past < best.past) best = { past, x: p[0], y: p[1] };
    }
  }
  if (!best) return false;
  const endPt = sketch.points[nearStart ? entity.p1 : entity.p2];
  endPt.x = best.x;
  endPt.y = best.y;
  return true;
}

// ---- convert entities -----------------------------------------------------

/**
 * Project outside geometry onto the sketch as new entities ("Convert
 * Entities"). Each loop is an ordered list of [u, v] sketch-plane points;
 * consecutive points become shared-endpoint lines, and a loop whose ends
 * coincide is closed. Returns the ids of the created line entities.
 *
 * The 2D loops come from a face/edge projected into the sketch plane (see
 * lib/faceBoundary.js); keeping the ingest pure means it is testable on plain
 * coordinate arrays.
 */
export function convertEntities(sketch, loops) {
  const created = [];
  for (const loop of loops) {
    const raw = dedupeLoop(loop);
    if (raw.length < 2) continue;
    const closed =
      raw.length > 2 &&
      Math.hypot(raw[0][0] - raw[raw.length - 1][0], raw[0][1] - raw[raw.length - 1][1]) <
        1e-6;
    // A closed loop repeats its first vertex at the end; drop it so the wrap
    // edge is synthesized instead of collapsing to a zero-length line.
    const pts = closed ? raw.slice(0, -1) : raw;
    const verts = pts.map(([u, v]) => addPoint(sketch, u, v));
    const n = closed ? verts.length : verts.length - 1;
    for (let i = 0; i < n; i++) {
      const from = verts[i];
      const to = verts[(i + 1) % verts.length];
      if (from === to) continue;
      created.push(addLine(sketch, from, to));
    }
  }
  return created;
}

/** Collapse consecutive near-coincident vertices (keeps a trailing repeat). */
function dedupeLoop(loop) {
  const out = [];
  for (const p of loop) {
    const last = out[out.length - 1];
    if (last && Math.hypot(last[0] - p[0], last[1] - p[1]) < 1e-6) continue;
    out.push(p);
  }
  return out;
}

// ---- small helpers --------------------------------------------------------

function clamp01(t) {
  return t < 0 ? 0 : t > 1 ? 1 : t;
}

function clampRange(t, lo, hi) {
  return t < lo ? lo : t > hi ? hi : t;
}

/**
 * Adjacent pair of sorted angles (ascending, in [0, 2π)) that brackets
 * `pick`, wrapping around 2π. Returns [lo, hi] with the removal arc going ccw
 * from lo to hi through the pick.
 */
function bracketAngle(sortedAngs, pick) {
  for (let i = 0; i < sortedAngs.length; i++) {
    const lo = sortedAngs[i];
    const hi = sortedAngs[(i + 1) % sortedAngs.length];
    // ccw span lo → hi (wrapping) that contains pick?
    const span = normalizeAngle(hi - lo) || 2 * Math.PI;
    const rel = normalizeAngle(pick - lo);
    if (rel <= span + EPS) return [lo, hi];
  }
  return [sortedAngs[0], sortedAngs[1]];
}
