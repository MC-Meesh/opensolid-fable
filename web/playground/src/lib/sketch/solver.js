/**
 * Iterative constraint solver (Gauss–Seidel position projection).
 *
 * Each constraint knows how to project the points it touches toward
 * satisfaction; sweeping all constraints repeatedly converges for the
 * well-posed sketches this canvas produces. Points in `pinned` never move
 * (used while dragging so the rest of the sketch follows the cursor).
 *
 * Arcs carry a built-in constraint: both endpoints stay equidistant from the
 * center, so the arc keeps a single radius as geometry moves.
 */

const DEFAULT_ITERATIONS = 200;

/** Absolute residual below which the sketch counts as solved. */
export const SOLVE_TOLERANCE = 1e-7;

function projectHorizontal(pts, c, sketch, pinned) {
  const line = sketch.entities[c.line];
  if (!line) return 0;
  const a = pts[line.p1];
  const b = pts[line.p2];
  const dy = b.y - a.y;
  const aPin = pinned.has(a.id);
  const bPin = pinned.has(b.id);
  if (aPin && bPin) return Math.abs(dy);
  if (aPin) b.y = a.y;
  else if (bPin) a.y = b.y;
  else {
    const mid = (a.y + b.y) / 2;
    a.y = mid;
    b.y = mid;
  }
  return Math.abs(dy);
}

function projectVertical(pts, c, sketch, pinned) {
  const line = sketch.entities[c.line];
  if (!line) return 0;
  const a = pts[line.p1];
  const b = pts[line.p2];
  const dx = b.x - a.x;
  const aPin = pinned.has(a.id);
  const bPin = pinned.has(b.id);
  if (aPin && bPin) return Math.abs(dx);
  if (aPin) b.x = a.x;
  else if (bPin) a.x = b.x;
  else {
    const mid = (a.x + b.x) / 2;
    a.x = mid;
    b.x = mid;
  }
  return Math.abs(dx);
}

function projectCoincident(pts, c, pinned) {
  const a = pts[c.a];
  const b = pts[c.b];
  if (!a || !b) return 0;
  const err = Math.hypot(b.x - a.x, b.y - a.y);
  const aPin = pinned.has(a.id);
  const bPin = pinned.has(b.id);
  if (aPin && bPin) return err;
  if (aPin) {
    b.x = a.x;
    b.y = a.y;
  } else if (bPin) {
    a.x = b.x;
    a.y = b.y;
  } else {
    const mx = (a.x + b.x) / 2;
    const my = (a.y + b.y) / 2;
    a.x = mx;
    a.y = my;
    b.x = mx;
    b.y = my;
  }
  return err;
}

/**
 * Drive a line segment's length to `value` along its current direction,
 * splitting the correction between its free (non-pinned) endpoints. Returns
 * the absolute length error before the correction.
 */
function setLineLength(pts, line, value, pinned) {
  const a = pts[line.p1];
  const b = pts[line.p2];
  let ux = b.x - a.x;
  let uy = b.y - a.y;
  const len = Math.hypot(ux, uy);
  if (len < 1e-12) {
    ux = 1;
    uy = 0;
  } else {
    ux /= len;
    uy /= len;
  }
  const err = len - value;
  const aPin = pinned.has(a.id);
  const bPin = pinned.has(b.id);
  if (aPin && bPin) return Math.abs(err);
  if (aPin) {
    b.x = a.x + ux * value;
    b.y = a.y + uy * value;
  } else if (bPin) {
    a.x = b.x - ux * value;
    a.y = b.y - uy * value;
  } else {
    a.x += (ux * err) / 2;
    a.y += (uy * err) / 2;
    b.x -= (ux * err) / 2;
    b.y -= (uy * err) / 2;
  }
  return Math.abs(err);
}

function projectLength(pts, c, sketch, pinned) {
  const line = sketch.entities[c.line];
  if (!line) return 0;
  return setLineLength(pts, line, c.value, pinned);
}

function projectArcEndpointsToRadius(pts, entity, radius, pinned) {
  let worst = 0;
  const cp = pts[entity.center];
  for (const pid of [entity.p1, entity.p2]) {
    const p = pts[pid];
    const dx = p.x - cp.x;
    const dy = p.y - cp.y;
    const d = Math.hypot(dx, dy);
    worst = Math.max(worst, Math.abs(d - radius));
    if (pinned.has(pid) || d < 1e-12) continue;
    const s = radius / d;
    p.x = cp.x + dx * s;
    p.y = cp.y + dy * s;
  }
  return worst;
}

function projectRadius(pts, c, sketch, pinned) {
  const entity = sketch.entities[c.entity];
  if (!entity) return 0;
  if (entity.type === 'circle') {
    const err = Math.abs(entity.radius - c.value);
    entity.radius = c.value;
    return err;
  }
  return projectArcEndpointsToRadius(pts, entity, c.value, pinned);
}

function projectTangent(pts, c, sketch, pinned) {
  const line = sketch.entities[c.line];
  const curve = sketch.entities[c.curve];
  if (!line || !curve) return 0;
  const a = pts[line.p1];
  const b = pts[line.p2];
  const center = pts[curve.center];
  let nx = -(b.y - a.y);
  let ny = b.x - a.x;
  const len = Math.hypot(nx, ny);
  if (len < 1e-12) return 0;
  nx /= len;
  ny /= len;
  const d = (center.x - a.x) * nx + (center.y - a.y) * ny;
  const side = d >= 0 ? 1 : -1;
  const radius =
    curve.type === 'circle'
      ? curve.radius
      : entityRadiusFrom(pts, curve);
  const err = Math.abs(d) - radius;

  const centerFree = !pinned.has(center.id);
  const lineFree = !pinned.has(a.id) && !pinned.has(b.id);
  if (!centerFree && !lineFree) return Math.abs(err);
  const wCenter = centerFree && lineFree ? 0.5 : centerFree ? 1 : 0;
  const wLine = centerFree && lineFree ? 0.5 : lineFree ? 1 : 0;
  // Shrink |d| toward radius: move the center toward the line and/or the
  // line toward the center, along the line normal.
  center.x -= nx * side * err * wCenter;
  center.y -= ny * side * err * wCenter;
  a.x += nx * side * err * wLine;
  a.y += ny * side * err * wLine;
  b.x += nx * side * err * wLine;
  b.y += ny * side * err * wLine;
  return Math.abs(err);
}

/** Arc radius from live solver positions (not the sketch's point table). */
function entityRadiusFrom(pts, arc) {
  const c = pts[arc.center];
  const p1 = pts[arc.p1];
  return Math.hypot(p1.x - c.x, p1.y - c.y);
}

function projectBuiltInArc(pts, entity, pinned) {
  const cp = pts[entity.center];
  const p1 = pts[entity.p1];
  const p2 = pts[entity.p2];
  const r1 = Math.hypot(p1.x - cp.x, p1.y - cp.y);
  const r2 = Math.hypot(p2.x - cp.x, p2.y - cp.y);
  // Pinned endpoints dictate the radius; otherwise meet in the middle.
  let target;
  const p1Pin = pinned.has(p1.id);
  const p2Pin = pinned.has(p2.id);
  if (p1Pin && !p2Pin) target = r1;
  else if (p2Pin && !p1Pin) target = r2;
  else target = (r1 + r2) / 2;
  return projectArcEndpointsToRadius(pts, entity, target, pinned);
}

/** A line can rotate/translate iff at least one endpoint is free. */
function lineFree(line, pinned) {
  return !pinned.has(line.p1) || !pinned.has(line.p2);
}

/** Correction weights that split work between two movable lines. */
function shareWeights(free1, free2) {
  if (free1 && free2) return [0.5, 0.5];
  if (free1) return [1, 0];
  if (free2) return [0, 1];
  return [0, 0];
}

/**
 * Rotate a line's free endpoints about a pivot by `theta` radians. The pivot
 * is the pinned endpoint if exactly one is pinned, otherwise the midpoint.
 */
function rotateLine(pts, line, theta, pinned) {
  if (theta === 0) return;
  const a = pts[line.p1];
  const b = pts[line.p2];
  const aPin = pinned.has(a.id);
  const bPin = pinned.has(b.id);
  if (aPin && bPin) return;
  let px;
  let py;
  if (aPin) {
    px = a.x;
    py = a.y;
  } else if (bPin) {
    px = b.x;
    py = b.y;
  } else {
    px = (a.x + b.x) / 2;
    py = (a.y + b.y) / 2;
  }
  const cos = Math.cos(theta);
  const sin = Math.sin(theta);
  for (const p of [a, b]) {
    if (pinned.has(p.id)) continue;
    const dx = p.x - px;
    const dy = p.y - py;
    p.x = px + dx * cos - dy * sin;
    p.y = py + dx * sin + dy * cos;
  }
}

/** Wrap an angle into (-π/2, π/2] — the parallel-error range (mod π). */
function wrapHalfPi(angle) {
  return (((angle % Math.PI) + Math.PI * 1.5) % Math.PI) - Math.PI / 2;
}

/**
 * Align the directions of two lines. `offset` is the desired signed angle
 * between them, taken mod π: 0 for parallel/collinear, π/2 for perpendicular.
 * Returns the absolute angular residual (radians).
 */
function projectAngle(pts, l1, l2, offset, pinned) {
  const a1 = pts[l1.p1];
  const b1 = pts[l1.p2];
  const a2 = pts[l2.p1];
  const b2 = pts[l2.p2];
  const ang1 = Math.atan2(b1.y - a1.y, b1.x - a1.x);
  const ang2 = Math.atan2(b2.y - a2.y, b2.x - a2.x);
  const diff = wrapHalfPi(ang1 - ang2 - offset);
  const free1 = lineFree(l1, pinned);
  const free2 = lineFree(l2, pinned);
  const [w1, w2] = shareWeights(free1, free2);
  // Rotating l1 by -diff (or l2 by +diff) drives ang1-ang2 toward `offset`.
  rotateLine(pts, l1, -diff * w1, pinned);
  rotateLine(pts, l2, diff * w2, pinned);
  return Math.abs(diff);
}

function projectParallel(pts, c, sketch, pinned) {
  const l1 = sketch.entities[c.a];
  const l2 = sketch.entities[c.b];
  if (!l1 || !l2) return 0;
  return projectAngle(pts, l1, l2, 0, pinned);
}

function projectPerpendicular(pts, c, sketch, pinned) {
  const l1 = sketch.entities[c.a];
  const l2 = sketch.entities[c.b];
  if (!l1 || !l2) return 0;
  return projectAngle(pts, l1, l2, Math.PI / 2, pinned);
}

/** Translate a line's free endpoints by `dist` along direction (nx, ny). */
function translateLine(pts, line, nx, ny, dist, pinned) {
  for (const pid of [line.p1, line.p2]) {
    if (pinned.has(pid)) continue;
    const p = pts[pid];
    p.x += nx * dist;
    p.y += ny * dist;
  }
}

function projectCollinear(pts, c, sketch, pinned) {
  const l1 = sketch.entities[c.a];
  const l2 = sketch.entities[c.b];
  if (!l1 || !l2) return 0;
  // First make them parallel, then slide them onto a shared axis.
  const angErr = projectAngle(pts, l1, l2, 0, pinned);
  const a = pts[l1.p1];
  const b = pts[l1.p2];
  let nx = -(b.y - a.y);
  let ny = b.x - a.x;
  const nl = Math.hypot(nx, ny);
  if (nl < 1e-12) return angErr;
  nx /= nl;
  ny /= nl;
  // Signed offset of l2's endpoints from l1's infinite line (l1's own points
  // are at offset 0 by construction of the normal).
  const c1 = pts[l2.p1];
  const c2 = pts[l2.p2];
  const off1 = (c1.x - a.x) * nx + (c1.y - a.y) * ny;
  const off2 = (c2.x - a.x) * nx + (c2.y - a.y) * ny;
  const mid = (off1 + off2) / 2;
  const [w1, w2] = shareWeights(lineFree(l1, pinned), lineFree(l2, pinned));
  translateLine(pts, l1, nx, ny, mid * w1, pinned);
  translateLine(pts, l2, nx, ny, -mid * w2, pinned);
  return Math.max(angErr, Math.abs(off1), Math.abs(off2));
}

/** Live radius of a circle or arc from the solver's point table. */
function radiusNow(pts, entity) {
  if (entity.type === 'circle') return entity.radius;
  const c = pts[entity.center];
  const p1 = pts[entity.p1];
  return Math.hypot(p1.x - c.x, p1.y - c.y);
}

/** Whether an entity's radius can move (circles always can). */
function radiusFree(entity, pinned) {
  if (entity.type === 'circle') return true;
  return !pinned.has(entity.p1) || !pinned.has(entity.p2);
}

function setRadius(pts, entity, value, pinned) {
  if (entity.type === 'circle') {
    entity.radius = value;
    return;
  }
  projectArcEndpointsToRadius(pts, entity, value, pinned);
}

function projectEqual(pts, c, sketch, pinned) {
  const ea = sketch.entities[c.a];
  const eb = sketch.entities[c.b];
  if (!ea || !eb) return 0;
  if (ea.type === 'line' && eb.type === 'line') {
    const la = Math.hypot(
      pts[ea.p2].x - pts[ea.p1].x,
      pts[ea.p2].y - pts[ea.p1].y
    );
    const lb = Math.hypot(
      pts[eb.p2].x - pts[eb.p1].x,
      pts[eb.p2].y - pts[eb.p1].y
    );
    const aFree = lineFree(ea, pinned);
    const bFree = lineFree(eb, pinned);
    const target = aFree && bFree ? (la + lb) / 2 : aFree ? lb : la;
    if (aFree) setLineLength(pts, ea, target, pinned);
    if (bFree) setLineLength(pts, eb, target, pinned);
    return Math.abs(la - lb);
  }
  // Equal radius for two circles/arcs.
  const ra = radiusNow(pts, ea);
  const rb = radiusNow(pts, eb);
  const aFree = radiusFree(ea, pinned);
  const bFree = radiusFree(eb, pinned);
  const target = aFree && bFree ? (ra + rb) / 2 : aFree ? rb : ra;
  if (aFree) setRadius(pts, ea, target, pinned);
  if (bFree) setRadius(pts, eb, target, pinned);
  return Math.abs(ra - rb);
}

function projectConcentric(pts, c, sketch, pinned) {
  const ea = sketch.entities[c.a];
  const eb = sketch.entities[c.b];
  if (!ea || !eb) return 0;
  return projectCoincident(pts, { a: ea.center, b: eb.center }, pinned);
}

function projectMidpoint(pts, c, sketch, pinned) {
  const line = sketch.entities[c.line];
  const p = pts[c.point];
  if (!line || !p) return 0;
  const a = pts[line.p1];
  const b = pts[line.p2];
  const mx = (a.x + b.x) / 2;
  const my = (a.y + b.y) / 2;
  const err = Math.hypot(p.x - mx, p.y - my);
  if (!pinned.has(p.id)) {
    p.x = mx;
    p.y = my;
    return err;
  }
  // Point is pinned: shift the line's midpoint to the point instead.
  const dx = p.x - mx;
  const dy = p.y - my;
  const aPin = pinned.has(a.id);
  const bPin = pinned.has(b.id);
  if (aPin && bPin) return err;
  if (aPin) {
    b.x += 2 * dx;
    b.y += 2 * dy;
  } else if (bPin) {
    a.x += 2 * dx;
    a.y += 2 * dy;
  } else {
    a.x += dx;
    a.y += dy;
    b.x += dx;
    b.y += dy;
  }
  return err;
}

/** Reflect point (px, py) across the infinite line through (ax, ay) dir (ux, uy). */
function reflectAcross(px, py, ax, ay, ux, uy) {
  const vx = px - ax;
  const vy = py - ay;
  const t = vx * ux + vy * uy;
  const projx = ax + ux * t;
  const projy = ay + uy * t;
  return { x: 2 * projx - px, y: 2 * projy - py };
}

function projectSymmetric(pts, c, sketch, pinned) {
  const line = sketch.entities[c.line];
  const A = pts[c.a];
  const B = pts[c.b];
  if (!line || !A || !B) return 0;
  const la = pts[line.p1];
  const lb = pts[line.p2];
  let ux = lb.x - la.x;
  let uy = lb.y - la.y;
  const ul = Math.hypot(ux, uy);
  if (ul < 1e-12) return 0;
  ux /= ul;
  uy /= ul;
  const mirrorB = reflectAcross(B.x, B.y, la.x, la.y, ux, uy);
  const mirrorA = reflectAcross(A.x, A.y, la.x, la.y, ux, uy);
  const err = Math.hypot(A.x - mirrorB.x, A.y - mirrorB.y);
  const aPin = pinned.has(A.id);
  const bPin = pinned.has(B.id);
  if (aPin && bPin) return err;
  if (!aPin && !bPin) {
    A.x = (A.x + mirrorB.x) / 2;
    A.y = (A.y + mirrorB.y) / 2;
    B.x = (B.x + mirrorA.x) / 2;
    B.y = (B.y + mirrorA.y) / 2;
  } else if (!aPin) {
    A.x = mirrorB.x;
    A.y = mirrorB.y;
  } else {
    B.x = mirrorA.x;
    B.y = mirrorA.y;
  }
  return err;
}

/**
 * Point-to-point distance dimension. `orient` selects what is measured:
 * 'aligned' (straight-line), 'horizontal' (|Δx|), or 'vertical' (|Δy|). The
 * correction is split between the two free (non-pinned) points.
 */
function projectDistance(pts, c, pinned) {
  const a = pts[c.a];
  const b = pts[c.b];
  if (!a || !b) return 0;
  const orient = c.orient ?? 'aligned';
  if (orient === 'horizontal') return projectAxisDistance(a, b, c.value, pinned, 'x');
  if (orient === 'vertical') return projectAxisDistance(a, b, c.value, pinned, 'y');
  let ux = b.x - a.x;
  let uy = b.y - a.y;
  const len = Math.hypot(ux, uy);
  if (len < 1e-12) {
    ux = 1;
    uy = 0;
  } else {
    ux /= len;
    uy /= len;
  }
  const err = len - c.value;
  const aPin = pinned.has(a.id);
  const bPin = pinned.has(b.id);
  if (aPin && bPin) return Math.abs(err);
  if (aPin) {
    b.x = a.x + ux * c.value;
    b.y = a.y + uy * c.value;
  } else if (bPin) {
    a.x = b.x - ux * c.value;
    a.y = b.y - uy * c.value;
  } else {
    a.x += (ux * err) / 2;
    a.y += (uy * err) / 2;
    b.x -= (ux * err) / 2;
    b.y -= (uy * err) / 2;
  }
  return Math.abs(err);
}

/** Drive |b[axis] - a[axis]| to `value`, keeping the current sign. */
function projectAxisDistance(a, b, value, pinned, axis) {
  const d = b[axis] - a[axis];
  const sign = d >= 0 ? 1 : -1;
  const diff = sign * value - d; // amount to add to (b - a) along the axis
  const err = Math.abs(Math.abs(d) - value);
  const aPin = pinned.has(a.id);
  const bPin = pinned.has(b.id);
  if (aPin && bPin) return err;
  if (aPin) b[axis] += diff;
  else if (bPin) a[axis] -= diff;
  else {
    a[axis] -= diff / 2;
    b[axis] += diff / 2;
  }
  return err;
}

/**
 * Perpendicular distance from a point to an infinite line, driven to `value`.
 * Splits the correction between the (free) point and the (free) line, moving
 * both along the line normal — the same geometry as tangent, with the point
 * playing the role of a circle center.
 */
function projectPdistance(pts, c, sketch, pinned) {
  const line = sketch.entities[c.line];
  const p = pts[c.point];
  if (!line || !p) return 0;
  const a = pts[line.p1];
  const b = pts[line.p2];
  let nx = -(b.y - a.y);
  let ny = b.x - a.x;
  const len = Math.hypot(nx, ny);
  if (len < 1e-12) return 0;
  nx /= len;
  ny /= len;
  const d = (p.x - a.x) * nx + (p.y - a.y) * ny;
  const side = d >= 0 ? 1 : -1;
  const err = Math.abs(d) - c.value;
  const pointFree = !pinned.has(p.id);
  const lineIsFree = !pinned.has(a.id) && !pinned.has(b.id);
  if (!pointFree && !lineIsFree) return Math.abs(err);
  const wPoint = pointFree && lineIsFree ? 0.5 : pointFree ? 1 : 0;
  const wLine = pointFree && lineIsFree ? 0.5 : lineIsFree ? 1 : 0;
  p.x -= nx * side * err * wPoint;
  p.y -= ny * side * err * wPoint;
  a.x += nx * side * err * wLine;
  a.y += ny * side * err * wLine;
  b.x += nx * side * err * wLine;
  b.y += ny * side * err * wLine;
  return Math.abs(err);
}

/** Angle dimension between two lines: drive their directions `value` apart. */
function projectAngleDim(pts, c, sketch, pinned) {
  const l1 = sketch.entities[c.a];
  const l2 = sketch.entities[c.b];
  if (!l1 || !l2) return 0;
  return projectAngle(pts, l1, l2, c.value, pinned);
}

/** Diameter dimension on a circle/arc: half the target drives the radius. */
function projectDiameter(pts, c, sketch, pinned) {
  const entity = sketch.entities[c.entity];
  if (!entity) return 0;
  const r = c.value / 2;
  if (entity.type === 'circle') {
    const err = Math.abs(entity.radius - r);
    entity.radius = r;
    return err;
  }
  return projectArcEndpointsToRadius(pts, entity, r, pinned);
}

function projectConstraint(pts, c, sketch, pinned) {
  // Driven (reference) dimensions are measured, never enforced — they must not
  // push geometry or contribute to the residual.
  if (c.driven) return 0;
  switch (c.type) {
    case 'horizontal':
      return projectHorizontal(pts, c, sketch, pinned);
    case 'vertical':
      return projectVertical(pts, c, sketch, pinned);
    case 'coincident':
      return projectCoincident(pts, c, pinned);
    case 'length':
      return projectLength(pts, c, sketch, pinned);
    case 'radius':
      return projectRadius(pts, c, sketch, pinned);
    case 'tangent':
      return projectTangent(pts, c, sketch, pinned);
    case 'parallel':
      return projectParallel(pts, c, sketch, pinned);
    case 'perpendicular':
      return projectPerpendicular(pts, c, sketch, pinned);
    case 'collinear':
      return projectCollinear(pts, c, sketch, pinned);
    case 'equal':
      return projectEqual(pts, c, sketch, pinned);
    case 'concentric':
      return projectConcentric(pts, c, sketch, pinned);
    case 'midpoint':
      return projectMidpoint(pts, c, sketch, pinned);
    case 'symmetric':
      return projectSymmetric(pts, c, sketch, pinned);
    case 'distance':
      return projectDistance(pts, c, pinned);
    case 'pdistance':
      return projectPdistance(pts, c, sketch, pinned);
    case 'angle':
      return projectAngleDim(pts, c, sketch, pinned);
    case 'diameter':
      return projectDiameter(pts, c, sketch, pinned);
    case 'fix':
      return 0; // enforced by pinning in solve()
    default:
      return 0;
  }
}

/**
 * Solve the sketch's constraints in place.
 *
 * Returns `{ converged, error, iterations }` where `error` is the largest
 * residual (world units) across constraints after the final sweep.
 */
export function solve(sketch, { iterations = DEFAULT_ITERATIONS, pinned } = {}) {
  const constraints = Object.values(sketch.constraints);
  // `fix` constraints anchor a point in place — treat them as pinned for the
  // duration of the solve, alongside any caller-supplied (e.g. dragged) pins.
  const fixed = constraints.filter((c) => c.type === 'fix').map((c) => c.point);
  const pin =
    fixed.length === 0 ? pinned ?? new Set() : new Set([...(pinned ?? []), ...fixed]);
  const pts = sketch.points;
  const arcs = Object.values(sketch.entities).filter((e) => e.type === 'arc');

  let error = 0;
  let used = 0;
  for (let i = 0; i < iterations; i++) {
    used = i + 1;
    error = 0;
    for (const arc of arcs) {
      error = Math.max(error, projectBuiltInArc(pts, arc, pin));
    }
    for (const c of constraints) {
      error = Math.max(error, projectConstraint(pts, c, sketch, pin));
    }
    if (error < SOLVE_TOLERANCE) break;
  }
  return { converged: error < SOLVE_TOLERANCE * 10, error, iterations: used };
}
