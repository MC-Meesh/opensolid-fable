/**
 * Sketch edit tools (of-fsl.21): offset, trim, extend, and convert-entities.
 *
 * These operate on the in-place sketch model (see model.js) using the pure
 * 2D intersection primitives in geom.js. They are kept free of React so the
 * geometry is unit-testable on plain sketch objects.
 *
 *  - offsetEntities: parallel copies of the selected geometry a signed
 *    distance away. Connected line/arc chains join at the offset corners
 *    (intersections of neighboring offset curves); circles/branch points are
 *    offset per-entity.
 *  - trimEntityAt / extendEntityAt: click-tools. Trim removes the clicked
 *    stretch back to its bounding intersections; extend lengthens the clicked
 *    end to the nearest intersection beyond it.
 *  - faceBoundaryLoopsUV: project a picked planar face's mesh-region boundary
 *    into the sketch's (u, v) plane for "convert entities".
 */

import {
  addArc,
  addCircle,
  addLine,
  addPoint,
  deleteEntity,
  entityRadius,
} from './model.js';
import {
  arcSweep,
  circleCircleIntersections,
  lineCircleIntersections,
  lineLineIntersection,
  normalizeAngle,
} from './geom.js';
import { worldToPlane } from './profile.js';

const EPS = 1e-9;

// ---- offset ---------------------------------------------------------------

/**
 * A directed offset "carrier" for an entity: a shifted infinite line, or a
 * concentric circle. `dist` is signed; positive offsets to the left of the
 * entity's traversal direction (p1 → p2, along the arc sweep). Circles offset
 * outward for positive `dist`.
 */
function offsetCarrier(sketch, e, dist) {
  const P = sketch.points;
  if (e.type === 'line') {
    const a = P[e.p1];
    const b = P[e.p2];
    const len = Math.hypot(b.x - a.x, b.y - a.y) || 1;
    const nx = -(b.y - a.y) / len; // left normal
    const ny = (b.x - a.x) / len;
    return {
      kind: 'line',
      off: [nx * dist, ny * dist],
      a: [a.x + nx * dist, a.y + ny * dist],
      b: [b.x + nx * dist, b.y + ny * dist],
    };
  }
  const c = P[e.center];
  const r = entityRadius(sketch, e);
  if (e.type === 'circle') {
    return { kind: 'circle', c: [c.x, c.y], r: r + dist };
  }
  // Arc: the left of the sweep is toward the center for a CCW arc, away for
  // CW — so the radius shrinks or grows accordingly.
  const a = P[e.p1];
  const rad = [(a.x - c.x) / r, (a.y - c.y) / r];
  const sign = e.ccw ? -1 : 1;
  return { kind: 'circle', c: [c.x, c.y], r: r + dist * sign, rad };
}

/** Nearest carrier-intersection point to `[jx, jy]`, or null. */
function carrierJoint(p, q, jx, jy) {
  let cands = [];
  if (p.kind === 'line' && q.kind === 'line') {
    const h = lineLineIntersection(p.a, p.b, q.a, q.b);
    if (h) cands = [[h.x, h.y]];
  } else if (p.kind === 'line') {
    cands = lineCircleIntersections(p.a, p.b, q.c, q.r).map((h) => [h.x, h.y]);
  } else if (q.kind === 'line') {
    cands = lineCircleIntersections(q.a, q.b, p.c, p.r).map((h) => [h.x, h.y]);
  } else {
    cands = circleCircleIntersections(p.c, p.r, q.c, q.r).map((h) => [h.x, h.y]);
  }
  let best = null;
  for (const c of cands) {
    const d = (c[0] - jx) ** 2 + (c[1] - jy) ** 2;
    if (!best || d < best.d) best = { c, d };
  }
  return best ? best.c : null;
}

/** Terminal (unjoined) offset position of endpoint `pid` under carrier. */
function terminalOffset(sketch, carrier, pid) {
  const p = sketch.points[pid];
  if (carrier.kind === 'line') {
    return [p.x + carrier.off[0], p.y + carrier.off[1]];
  }
  const ang = Math.atan2(p.y - carrier.c[1], p.x - carrier.c[0]);
  return [carrier.c[0] + carrier.r * Math.cos(ang), carrier.c[1] + carrier.r * Math.sin(ang)];
}

/**
 * Offset the entities `ids` by signed distance `dist`, adding parallel copies
 * to the sketch. Connected line/arc chains join at their offset corners.
 * Returns the new entity ids. Entities whose offset collapses (radius ≤ 0)
 * are skipped.
 */
export function offsetEntities(sketch, ids, dist, opts = {}) {
  if (!(Math.abs(dist) > EPS)) return [];
  const entities = ids.map((id) => sketch.entities[id]).filter(Boolean);
  const ends = (e) => (e.type === 'circle' ? [] : [e.p1, e.p2]);

  // Which selected entities meet at each endpoint id.
  const usage = new Map();
  for (const e of entities) {
    for (const pid of ends(e)) {
      if (!usage.has(pid)) usage.set(pid, []);
      usage.get(pid).push(e.id);
    }
  }
  const neighborOf = (eid, pid) => {
    const list = usage.get(pid);
    if (!list || list.length !== 2) return null; // free end or branch
    return sketch.entities[list.find((x) => x !== eid)];
  };

  const carriers = new Map(entities.map((e) => [e.id, offsetCarrier(sketch, e, dist)]));
  const jointPoint = new Map(); // shared original point id -> new point id
  const created = [];

  const resolveEnd = (e, pid) => {
    const n = neighborOf(e.id, pid);
    if (n) {
      if (jointPoint.has(pid)) return jointPoint.get(pid);
      const orig = sketch.points[pid];
      const xy =
        carrierJoint(carriers.get(e.id), carriers.get(n.id), orig.x, orig.y) ??
        terminalOffset(sketch, carriers.get(e.id), pid);
      const np = addPoint(sketch, xy[0], xy[1]);
      jointPoint.set(pid, np);
      return np;
    }
    const xy = terminalOffset(sketch, carriers.get(e.id), pid);
    return addPoint(sketch, xy[0], xy[1]);
  };

  for (const e of entities) {
    const carrier = carriers.get(e.id);
    const co = e.construction ? { construction: true } : opts;
    if (e.type === 'line') {
      const p1 = resolveEnd(e, e.p1);
      const p2 = resolveEnd(e, e.p2);
      const A = sketch.points[p1];
      const B = sketch.points[p2];
      if (Math.hypot(B.x - A.x, B.y - A.y) > EPS) {
        created.push(addLine(sketch, p1, p2, co));
      }
    } else if (e.type === 'circle') {
      if (carrier.r > EPS) {
        const c = sketch.points[e.center];
        created.push(addCircle(sketch, addPoint(sketch, c.x, c.y), carrier.r, co));
      }
    } else {
      if (carrier.r > EPS) {
        const p1 = resolveEnd(e, e.p1);
        const p2 = resolveEnd(e, e.p2);
        const c = sketch.points[e.center];
        created.push(addArc(sketch, addPoint(sketch, c.x, c.y), p1, p2, e.ccw, co));
      }
    }
  }
  return created;
}

// ---- shared parametric curve model (trim / extend) ------------------------

/**
 * Parametric view of an entity for trim/extend. `paramOf(x, y)` maps a point
 * to a scalar parameter; the drawn extent is `[0, max]` (a line uses [0, 1];
 * an arc/circle uses radians of sweep). `pointAt(t)` inverts it.
 */
function curveOf(sketch, e) {
  const P = sketch.points;
  if (e.type === 'line') {
    const a = P[e.p1];
    const b = P[e.p2];
    const dx = b.x - a.x;
    const dy = b.y - a.y;
    const lenSq = dx * dx + dy * dy || 1;
    return {
      type: 'line',
      cyclic: false,
      max: 1,
      a: [a.x, a.y],
      b: [b.x, b.y],
      paramOf: (x, y) => ((x - a.x) * dx + (y - a.y) * dy) / lenSq,
      pointAt: (t) => [a.x + t * dx, a.y + t * dy],
    };
  }
  const c = P[e.center];
  const r = entityRadius(sketch, e);
  if (e.type === 'circle') {
    return {
      type: 'circle',
      cyclic: true,
      max: 2 * Math.PI,
      c: [c.x, c.y],
      r,
      paramOf: (x, y) => normalizeAngle(Math.atan2(y - c.y, x - c.x)),
      pointAt: (d) => [c.x + r * Math.cos(d), c.y + r * Math.sin(d)],
    };
  }
  const startAng = Math.atan2(P[e.p1].y - c.y, P[e.p1].x - c.x);
  const endAng = Math.atan2(P[e.p2].y - c.y, P[e.p2].x - c.x);
  const sweep = arcSweep(normalizeAngle(startAng), normalizeAngle(endAng), e.ccw);
  return {
    type: 'arc',
    cyclic: false,
    max: sweep,
    c: [c.x, c.y],
    r,
    ccw: e.ccw,
    startAng,
    paramOf: (x, y) => {
      const th = Math.atan2(y - c.y, x - c.x);
      return e.ccw ? normalizeAngle(th - startAng) : normalizeAngle(startAng - th);
    },
    pointAt: (d) => {
      const ang = e.ccw ? startAng + d : startAng - d;
      return [c.x + r * Math.cos(ang), c.y + r * Math.sin(ang)];
    },
  };
}

/** Intersection points of two carriers (curves as unbounded line/circle). */
function carrierPoints(a, b) {
  if (a.type === 'line' && b.type === 'line') {
    const h = lineLineIntersection(a.a, a.b, b.a, b.b);
    return h ? [[h.x, h.y]] : [];
  }
  if (a.type === 'line') {
    return lineCircleIntersections(a.a, a.b, b.c, b.r).map((h) => [h.x, h.y]);
  }
  if (b.type === 'line') {
    return lineCircleIntersections(b.a, b.b, a.c, a.r).map((h) => [h.x, h.y]);
  }
  return circleCircleIntersections(a.c, a.r, b.c, b.r).map((h) => [h.x, h.y]);
}

/** Whether `[x, y]` lies on the drawn extent of curve `c`. */
function onDrawn(c, x, y) {
  if (c.type === 'line') {
    const t = c.paramOf(x, y);
    return t >= -1e-6 && t <= 1 + 1e-6;
  }
  if (c.type === 'circle') return true;
  return c.paramOf(x, y) <= c.max + 1e-6;
}

/** Parameters where entity `id`'s carrier meets other entities, on both extents. */
function crossingParams(sketch, id, curve) {
  const params = [];
  for (const o of Object.values(sketch.entities)) {
    if (o.id === id) continue;
    const co = curveOf(sketch, o);
    for (const [px, py] of carrierPoints(curve, co)) {
      if (!onDrawn(co, px, py)) continue;
      const t = curve.paramOf(px, py);
      if (curve.cyclic) params.push(normalizeAngle(t));
      else if (t >= -1e-6 && t <= curve.max + 1e-6) params.push(t);
    }
  }
  return params;
}

/**
 * Rebuild entity `e` (removed by the caller) over parameter interval
 * `[s, eP]`, reusing the original endpoint ids at the drawn extremes so
 * connectivity survives. Returns the new entity id.
 */
function rebuildInterval(sketch, e, curve, s, eP, co) {
  const startId =
    Math.abs(s) < 1e-7 && !curve.cyclic
      ? e.p1
      : addPoint(sketch, ...curve.pointAt(s));
  const endId =
    Math.abs(eP - curve.max) < 1e-7 && !curve.cyclic
      ? e.p2
      : addPoint(sketch, ...curve.pointAt(eP));
  if (e.type === 'line') return addLine(sketch, startId, endId, co);
  const c = sketch.points[e.center];
  const centerId = addPoint(sketch, c.x, c.y);
  return addArc(sketch, centerId, startId, endId, curve.type === 'circle' ? true : e.ccw, co);
}

/**
 * Trim entity `id` at world point `[x, y]`: remove the stretch containing the
 * click, bounded by its nearest intersections with other entities. An
 * unbounded stretch trims the whole entity away; an interior stretch splits
 * it. Returns true when the sketch changed.
 */
export function trimEntityAt(sketch, id, x, y) {
  const e = sketch.entities[id];
  if (!e) return false;
  const curve = curveOf(sketch, e);
  const co = e.construction ? { construction: true } : {};
  const params = crossingParams(sketch, id, curve);

  if (curve.cyclic) {
    // Circle: need two cuts to bound an arc; sort angles and remove the
    // sector containing the click, keeping the complementary arc.
    const uniq = dedupeSorted(params.map((p) => normalizeAngle(p)));
    if (uniq.length < 2) {
      deleteEntity(sketch, id);
      return true;
    }
    const click = normalizeAngle(curve.paramOf(x, y));
    let hi = uniq.find((p) => p > click);
    let lo;
    if (hi === undefined) {
      hi = uniq[0];
      lo = uniq[uniq.length - 1];
    } else {
      const idx = uniq.indexOf(hi);
      lo = uniq[(idx - 1 + uniq.length) % uniq.length];
    }
    deleteEntity(sketch, id);
    // Keep the complement: an arc from hi CCW round to lo.
    const c = sketch.points[e.center] ?? { x: curve.c[0], y: curve.c[1] };
    const centerId = addPoint(sketch, c.x, c.y);
    const start = curve.pointAt(hi);
    const end = curve.pointAt(lo);
    addArc(sketch, centerId, addPoint(sketch, ...start), addPoint(sketch, ...end), true, co);
    return true;
  }

  const cuts = dedupeSorted(
    params.filter((t) => t > 1e-6 && t < curve.max - 1e-6)
  );
  const click = Math.max(0, Math.min(curve.max, curve.paramOf(x, y)));
  let lo = 0;
  let hi = curve.max;
  for (const t of cuts) {
    if (t <= click) lo = t;
    else {
      hi = t;
      break;
    }
  }
  // Intervals to keep = drawn extent minus (lo, hi).
  const keep = [];
  if (lo > 1e-7) keep.push([0, lo]);
  if (hi < curve.max - 1e-7) keep.push([hi, curve.max]);

  // Rebuild the kept intervals before deleting so the reused endpoint ids
  // aren't pruned as momentary orphans; deleting last drops the old entity
  // (and any endpoint no surviving interval reused).
  for (const [s, eP] of keep) rebuildInterval(sketch, e, curve, s, eP, co);
  deleteEntity(sketch, id);
  return true;
}

/**
 * Extend entity `id`'s end nearest `[x, y]` to the closest intersection just
 * beyond it. Returns true when the sketch changed.
 */
export function extendEntityAt(sketch, id, x, y) {
  const e = sketch.entities[id];
  if (!e || e.type === 'circle') return false;
  const curve = curveOf(sketch, e);
  const clickParam = curve.paramOf(x, y);
  const atStart = clickParam < curve.max / 2;

  let bestPt = null;
  let bestGap = Infinity;
  for (const o of Object.values(sketch.entities)) {
    if (o.id === id) continue;
    const co = curveOf(sketch, o);
    for (const [px, py] of carrierPoints(curve, co)) {
      if (!onDrawn(co, px, py)) continue;
      const gap = beyondGap(curve, px, py, atStart);
      if (gap !== null && gap < bestGap) {
        bestGap = gap;
        bestPt = [px, py];
      }
    }
  }
  if (!bestPt) return false;

  const np = addPoint(sketch, bestPt[0], bestPt[1]);
  if (e.type === 'line') {
    if (atStart) e.p1 = np;
    else e.p2 = np;
  } else if (atStart) {
    e.p1 = np;
  } else {
    e.p2 = np;
  }
  return true;
}

/**
 * Signed gap from the clicked end to `[x, y]` if it lies beyond that end (in
 * the extension direction), else null. Smaller = nearer.
 */
function beyondGap(curve, x, y, atStart) {
  if (curve.type === 'line') {
    const t = curve.paramOf(x, y);
    if (atStart) return t < -1e-6 ? -t : null;
    return t > 1 + 1e-6 ? t - 1 : null;
  }
  // Arc: measure angular distance past the clicked end, without wrapping past
  // the far end (i.e. within the complementary gap 2π − sweep).
  const gapMax = 2 * Math.PI - curve.max;
  const th = Math.atan2(y - curve.c[1], x - curve.c[0]);
  const endAng = curve.ccw ? curve.startAng + curve.max : curve.startAng - curve.max;
  let past;
  if (atStart) {
    past = curve.ccw
      ? normalizeAngle(curve.startAng - th)
      : normalizeAngle(th - curve.startAng);
  } else {
    past = curve.ccw ? normalizeAngle(th - endAng) : normalizeAngle(endAng - th);
  }
  return past > 1e-6 && past < gapMax - 1e-6 ? past : null;
}

/** Sort ascending and drop near-duplicate values. */
function dedupeSorted(values, tol = 1e-6) {
  const sorted = values.slice().sort((a, b) => a - b);
  const out = [];
  for (const v of sorted) {
    if (out.length === 0 || Math.abs(v - out[out.length - 1]) > tol) out.push(v);
  }
  return out;
}

// ---- convert entities (project a face boundary into the sketch plane) -----

const encodeEdge = (p, q) => (p < q ? `${p}_${q}` : `${q}_${p}`);

/**
 * Boundary loops of a mesh face region, projected into a sketch plane's
 * (u, v). `tris` are triangle indices into `indices` (as returned by
 * facePlane region detection); an edge on the region boundary belongs to
 * exactly one region triangle. Loops are walked in vertex order and
 * collinear runs are collapsed. Returns an array of loops, each an array of
 * `[u, v]` points (open — first ≠ last).
 */
export function faceBoundaryLoopsUV(positions, indices, tris, plane, tol = 1e-6) {
  const triSet = new Set(tris);
  const edgeCount = new Map(); // undirected edge -> count within the region
  const dirEdge = new Map(); // encoded -> [a, b] a directed representative
  for (const t of triSet) {
    const v = [indices[3 * t], indices[3 * t + 1], indices[3 * t + 2]];
    for (const [a, b] of [[v[0], v[1]], [v[1], v[2]], [v[2], v[0]]]) {
      const key = encodeEdge(a, b);
      edgeCount.set(key, (edgeCount.get(key) ?? 0) + 1);
      if (!dirEdge.has(key)) dirEdge.set(key, [a, b]);
    }
  }
  // Boundary edges appear once; build vertex adjacency over them.
  const adj = new Map();
  for (const [key, count] of edgeCount) {
    if (count !== 1) continue;
    const [a, b] = dirEdge.get(key);
    if (!adj.has(a)) adj.set(a, []);
    if (!adj.has(b)) adj.set(b, []);
    adj.get(a).push(b);
    adj.get(b).push(a);
  }

  const visited = new Set();
  const loops = [];
  for (const startV of adj.keys()) {
    if (visited.has(startV)) continue;
    const order = [];
    let cur = startV;
    let prev = -1;
    while (cur !== undefined && !visited.has(cur)) {
      visited.add(cur);
      order.push(cur);
      const next = (adj.get(cur) ?? []).find((n) => n !== prev && !visited.has(n));
      prev = cur;
      cur = next;
    }
    if (order.length < 3) continue;
    const uv = order.map((i) =>
      worldToPlane(plane, [positions[3 * i], positions[3 * i + 1], positions[3 * i + 2]])
    );
    const simplified = simplifyLoop(uv, tol);
    if (simplified.length >= 3) loops.push(simplified);
  }
  return loops;
}

/** Drop points that lie on the segment between their neighbors (closed loop). */
function simplifyLoop(points, tol) {
  const n = points.length;
  if (n < 3) return points;
  const out = [];
  for (let i = 0; i < n; i++) {
    const a = points[(i - 1 + n) % n];
    const b = points[i];
    const c = points[(i + 1) % n];
    const abx = b[0] - a[0];
    const aby = b[1] - a[1];
    const acx = c[0] - a[0];
    const acy = c[1] - a[1];
    const cross = abx * acy - aby * acx;
    const base = Math.hypot(acx, acy) || 1;
    if (Math.abs(cross) / base > tol) out.push(b);
  }
  return out.length >= 3 ? out : points;
}
