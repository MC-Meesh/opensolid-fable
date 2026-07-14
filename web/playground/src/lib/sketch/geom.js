/** 2D geometry helpers shared by snapping, hit-testing, and profile export. */

const TWO_PI = Math.PI * 2;

/** Normalize an angle to [0, 2π). */
export function normalizeAngle(a) {
  const r = a % TWO_PI;
  return r < 0 ? r + TWO_PI : r;
}

/**
 * Positive sweep (0, 2π] traversing from `start` to `end` in the given
 * direction. Coincident angles yield 2π, not 0 — a zero-extent arc is
 * invalid upstream.
 */
export function arcSweep(start, end, ccw) {
  const diff = ccw ? end - start : start - end;
  const s = normalizeAngle(diff);
  return s === 0 ? TWO_PI : s;
}

/** Whether `angle` lies on the arc from `start` sweeping `sweep` in `ccw`. */
export function angleOnArc(angle, start, sweep, ccw) {
  const rel = ccw
    ? normalizeAngle(angle - start)
    : normalizeAngle(start - angle);
  return rel <= sweep + 1e-9;
}

/** Distance from point (px, py) to segment (ax, ay)-(bx, by). */
export function distToSegment(px, py, ax, ay, bx, by) {
  const dx = bx - ax;
  const dy = by - ay;
  const lenSq = dx * dx + dy * dy;
  let t = 0;
  if (lenSq > 0) {
    t = ((px - ax) * dx + (py - ay) * dy) / lenSq;
    t = Math.max(0, Math.min(1, t));
  }
  return Math.hypot(px - (ax + t * dx), py - (ay + t * dy));
}

/** Distance from a point to a circle outline. */
export function distToCircle(px, py, cx, cy, r) {
  return Math.abs(Math.hypot(px - cx, py - cy) - r);
}

/**
 * Distance from a point to an arc (center cx,cy radius r from startAngle
 * sweeping `sweep` in direction `ccw`). Off-span points measure to the
 * nearest arc endpoint.
 */
export function distToArc(px, py, cx, cy, r, startAngle, sweep, ccw) {
  const angle = Math.atan2(py - cy, px - cx);
  if (angleOnArc(angle, startAngle, sweep, ccw)) {
    return distToCircle(px, py, cx, cy, r);
  }
  const endAngle = ccw ? startAngle + sweep : startAngle - sweep;
  const d1 = Math.hypot(
    px - (cx + r * Math.cos(startAngle)),
    py - (cy + r * Math.sin(startAngle))
  );
  const d2 = Math.hypot(
    px - (cx + r * Math.cos(endAngle)),
    py - (cy + r * Math.sin(endAngle))
  );
  return Math.min(d1, d2);
}

/** Sample an arc into `n + 1` points including both endpoints. */
export function sampleArc(cx, cy, r, startAngle, sweep, ccw, n) {
  const pts = [];
  const step = (ccw ? sweep : -sweep) / n;
  for (let i = 0; i <= n; i++) {
    const a = startAngle + step * i;
    pts.push([cx + r * Math.cos(a), cy + r * Math.sin(a)]);
  }
  return pts;
}

/**
 * Reflect point (px, py) across the infinite line through (ax, ay)-(bx, by).
 * A degenerate axis (a == b) reflects through the point instead.
 */
export function reflectPoint(px, py, ax, ay, bx, by) {
  const dx = bx - ax;
  const dy = by - ay;
  const lenSq = dx * dx + dy * dy;
  if (lenSq === 0) return [2 * ax - px, 2 * ay - py];
  // Projection parameter of P onto the axis, then mirror across the foot.
  const t = ((px - ax) * dx + (py - ay) * dy) / lenSq;
  const fx = ax + t * dx;
  const fy = ay + t * dy;
  return [2 * fx - px, 2 * fy - py];
}

/**
 * Intersection of two infinite lines, each given by two points. Returns
 * `{ x, y, t, u }` where `t` is the parameter along A (0 at a1, 1 at a2) and
 * `u` the parameter along B, or `null` when the lines are parallel.
 */
export function lineLineIntersection(a1, a2, b1, b2) {
  const r = [a2[0] - a1[0], a2[1] - a1[1]];
  const s = [b2[0] - b1[0], b2[1] - b1[1]];
  const denom = r[0] * s[1] - r[1] * s[0];
  if (Math.abs(denom) < 1e-12) return null; // parallel or degenerate
  const qp = [b1[0] - a1[0], b1[1] - a1[1]];
  const t = (qp[0] * s[1] - qp[1] * s[0]) / denom;
  const u = (qp[0] * r[1] - qp[1] * r[0]) / denom;
  return { x: a1[0] + t * r[0], y: a1[1] + t * r[1], t, u };
}

/**
 * Intersections of the infinite line through (a1, a2) with the circle
 * (center c, radius r). Returns 0, 1, or 2 points, each `{ x, y, t }` with
 * `t` the parameter along the line (0 at a1, 1 at a2), ordered by `t`.
 */
export function lineCircleIntersections(a1, a2, c, r) {
  const dx = a2[0] - a1[0];
  const dy = a2[1] - a1[1];
  const lenSq = dx * dx + dy * dy;
  if (lenSq < 1e-24) return [];
  // Foot of perpendicular from center to the line, as a parameter.
  const tFoot = ((c[0] - a1[0]) * dx + (c[1] - a1[1]) * dy) / lenSq;
  const fx = a1[0] + tFoot * dx;
  const fy = a1[1] + tFoot * dy;
  const distSq = (fx - c[0]) ** 2 + (fy - c[1]) ** 2;
  const rSq = r * r;
  if (distSq > rSq + 1e-12) return [];
  const half = Math.sqrt(Math.max(0, rSq - distSq) / lenSq);
  if (half < 1e-9) {
    return [{ x: fx, y: fy, t: tFoot }];
  }
  const ts = [tFoot - half, tFoot + half];
  return ts.map((t) => ({ x: a1[0] + t * dx, y: a1[1] + t * dy, t }));
}

/**
 * Intersections of two circles (centers c1/c2, radii r1/r2). Returns 0 or 2
 * points `{ x, y }` (a single tangent contact is returned once). Coincident
 * circles yield `[]`.
 */
export function circleCircleIntersections(c1, r1, c2, r2) {
  const dx = c2[0] - c1[0];
  const dy = c2[1] - c1[1];
  const d = Math.hypot(dx, dy);
  if (d < 1e-12) return []; // concentric
  if (d > r1 + r2 + 1e-9 || d < Math.abs(r1 - r2) - 1e-9) return []; // apart / nested
  const a = (r1 * r1 - r2 * r2 + d * d) / (2 * d);
  const hSq = r1 * r1 - a * a;
  const mx = c1[0] + (a * dx) / d;
  const my = c1[1] + (a * dy) / d;
  if (hSq <= 1e-12) return [{ x: mx, y: my }];
  const h = Math.sqrt(hSq);
  const ox = (-dy * h) / d;
  const oy = (dx * h) / d;
  return [
    { x: mx + ox, y: my + oy },
    { x: mx - ox, y: my - oy },
  ];
}

/** Signed area of a closed polygon (positive = counterclockwise). */
export function signedArea(points) {
  let area = 0;
  for (let i = 0; i < points.length; i++) {
    const [x1, y1] = points[i];
    const [x2, y2] = points[(i + 1) % points.length];
    area += x1 * y2 - x2 * y1;
  }
  return area / 2;
}
