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

function projectLength(pts, c, sketch, pinned) {
  const line = sketch.entities[c.line];
  if (!line) return 0;
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

function projectConstraint(pts, c, sketch, pinned) {
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
  const pin = pinned ?? new Set();
  const pts = sketch.points;
  const constraints = Object.values(sketch.constraints);
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
