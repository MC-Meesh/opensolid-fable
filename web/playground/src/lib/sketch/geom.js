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
