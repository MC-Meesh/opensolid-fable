/**
 * Sketch data model: plain serializable objects, mutated in place.
 *
 * A sketch owns three id-keyed tables:
 *   points:      { id, x, y }
 *   entities:    { id, type: 'line',   p1, p2 }
 *                { id, type: 'circle', center, radius }
 *                { id, type: 'arc',    center, p1, p2, ccw }   (start p1 → end p2)
 *                { id, type: 'ellipse', center, rx, ry, rotation }   (closed)
 *                { id, type: 'spline', p1, p2, c1, c2 }         (cubic Bézier)
 *   constraints: { id, type: 'horizontal' | 'vertical', line }
 *                { id, type: 'coincident', a, b }              (point ids)
 *                { id, type: 'tangent', line, curve }          (curve: circle/arc id)
 *                { id, type: 'length', line, value }
 *                { id, type: 'radius', entity, value }         (circle or arc id)
 *                { id, type: 'parallel' | 'perpendicular' | 'collinear', a, b } (line ids)
 *                { id, type: 'equal', a, b }                   (two lines, or two circles/arcs)
 *                { id, type: 'concentric', a, b }              (two circle/arc ids)
 *                { id, type: 'midpoint', point, line }         (point at line's midpoint)
 *                { id, type: 'symmetric', a, b, line }         (points a,b mirrored over axis line)
 *                { id, type: 'fix', point }                    (anchor a point in place)
 *
 * Entities reference points by id; chained lines share endpoint ids, which is
 * implicit coincidence. The explicit `coincident` constraint glues two
 * distinct points together via the solver.
 *
 * Any entity may carry `construction: true` — reference geometry (centerlines,
 * mirror axes, layout guides) that is drawn but excluded from profile
 * extraction. `addLine`/`addCircle`/`addArc` take a trailing options object to
 * set it; the flag is omitted entirely for normal geometry.
 */

import { reflectPoint } from './geom.js';

export function createSketch() {
  return { seq: 0, points: {}, entities: {}, constraints: {} };
}

function nextId(sketch, prefix) {
  sketch.seq += 1;
  return `${prefix}${sketch.seq}`;
}

export function addPoint(sketch, x, y) {
  const id = nextId(sketch, 'p');
  sketch.points[id] = { id, x, y };
  return id;
}

/** Attach `construction: true` to an entity spec when requested. */
function withConstruction(entity, { construction = false } = {}) {
  return construction ? { ...entity, construction: true } : entity;
}

export function addLine(sketch, p1, p2, opts = {}) {
  const id = nextId(sketch, 'e');
  sketch.entities[id] = withConstruction({ id, type: 'line', p1, p2 }, opts);
  return id;
}

export function addCircle(sketch, center, radius, opts = {}) {
  const id = nextId(sketch, 'e');
  sketch.entities[id] = withConstruction(
    { id, type: 'circle', center, radius },
    opts
  );
  return id;
}

export function addArc(sketch, center, p1, p2, ccw = true, opts = {}) {
  const id = nextId(sketch, 'e');
  sketch.entities[id] = withConstruction(
    { id, type: 'arc', center, p1, p2, ccw },
    opts
  );
  return id;
}

/**
 * Closed ellipse centered at `center` (a point id) with semi-axis `rx` along
 * the major axis and `ry` along the minor, the major axis rotated `rotation`
 * radians from +X. Like a circle, an ellipse is a self-closed loop — it forms
 * a whole profile on its own and does not chain with other segments.
 */
export function addEllipse(sketch, center, rx, ry, rotation = 0, opts = {}) {
  const id = nextId(sketch, 'e');
  sketch.entities[id] = withConstruction(
    { id, type: 'ellipse', center, rx, ry, rotation },
    opts
  );
  return id;
}

/**
 * Cubic-Bézier spline segment from endpoint `p1` to `p2` (point ids) with
 * control points `c1`, `c2` (also point ids, so the handles are draggable).
 * Splines chain into a profile loop through shared endpoint ids exactly like
 * lines and arcs.
 */
export function addSpline(sketch, p1, p2, c1, c2, opts = {}) {
  const id = nextId(sketch, 'e');
  sketch.entities[id] = withConstruction(
    { id, type: 'spline', p1, p2, c1, c2 },
    opts
  );
  return id;
}

export function addConstraint(sketch, constraint) {
  const id = nextId(sketch, 'c');
  sketch.constraints[id] = { ...constraint, id };
  return id;
}

/**
 * Axis-aligned rectangle as 4 shared-corner lines with horizontal/vertical
 * constraints baked in. Returns the four line ids.
 */
export function addRectangle(sketch, x1, y1, x2, y2) {
  const a = addPoint(sketch, x1, y1);
  const b = addPoint(sketch, x2, y1);
  const c = addPoint(sketch, x2, y2);
  const d = addPoint(sketch, x1, y2);
  const bottom = addLine(sketch, a, b);
  const right = addLine(sketch, b, c);
  const top = addLine(sketch, c, d);
  const left = addLine(sketch, d, a);
  addConstraint(sketch, { type: 'horizontal', line: bottom });
  addConstraint(sketch, { type: 'horizontal', line: top });
  addConstraint(sketch, { type: 'vertical', line: right });
  addConstraint(sketch, { type: 'vertical', line: left });
  return [bottom, right, top, left];
}

/**
 * Regular n-gon inscribed in a circle of `radius` about (cx, cy), first
 * vertex at `rotation` radians (default 0 = +X). Built as `sides` chained
 * lines sharing corner ids; returns the line ids in order.
 */
export function addPolygon(sketch, cx, cy, radius, sides, rotation = 0, opts = {}) {
  const n = Math.max(3, Math.round(sides));
  const corners = [];
  for (let i = 0; i < n; i++) {
    const a = rotation + (i * 2 * Math.PI) / n;
    corners.push(addPoint(sketch, cx + radius * Math.cos(a), cy + radius * Math.sin(a)));
  }
  const lines = [];
  for (let i = 0; i < n; i++) {
    lines.push(addLine(sketch, corners[i], corners[(i + 1) % n], opts));
  }
  return lines;
}

/**
 * Straight slot: a stadium (obround) of half-width `radius` around the
 * centerline segment (x1, y1)-(x2, y2) — two parallel lines capped by two
 * semicircular arcs, forming one closed loop. Returns the entity ids
 * [line, arc, line, arc] in loop order.
 */
export function addSlot(sketch, x1, y1, x2, y2, radius, opts = {}) {
  const dx = x2 - x1;
  const dy = y2 - y1;
  const len = Math.hypot(dx, dy);
  // Unit normal to the centerline; offset the two side rails by ±radius.
  const nx = len > 0 ? -dy / len : 0;
  const ny = len > 0 ? dx / len : 1;
  const ox = nx * radius;
  const oy = ny * radius;
  // Rail endpoints: left side runs start→end, right side runs end→start so
  // the four segments chain head-to-tail around the loop (CCW for len,r > 0).
  const a = addPoint(sketch, x1 + ox, y1 + oy); // start, left
  const b = addPoint(sketch, x2 + ox, y2 + oy); // end,   left
  const c = addPoint(sketch, x2 - ox, y2 - oy); // end,   right
  const d = addPoint(sketch, x1 - ox, y1 - oy); // start, right
  const c1 = addPoint(sketch, x2, y2); // end cap center
  const c0 = addPoint(sketch, x1, y1); // start cap center
  const left = addLine(sketch, a, b, opts);
  const endCap = addArc(sketch, c1, b, c, true, opts);
  const right = addLine(sketch, c, d, opts);
  const startCap = addArc(sketch, c0, d, a, true, opts);
  return [left, endCap, right, startCap];
}

/**
 * Mirror `ids` (entity ids) across the infinite line through points
 * (ax, ay)-(bx, by), adding reflected copies to the sketch. Points shared
 * between mirrored entities are reflected once and reused, so the copy keeps
 * the original's connectivity. Returns the new entity ids.
 */
export function mirrorEntities(sketch, ids, ax, ay, bx, by, opts = {}) {
  const pointMap = new Map(); // original point id -> reflected point id
  const reflect = (pid) => {
    if (pointMap.has(pid)) return pointMap.get(pid);
    const p = sketch.points[pid];
    const [rx, ry] = reflectPoint(p.x, p.y, ax, ay, bx, by);
    const np = addPoint(sketch, rx, ry);
    pointMap.set(pid, np);
    return np;
  };
  const created = [];
  for (const id of ids) {
    const e = sketch.entities[id];
    if (!e) continue;
    const construction = e.construction ? { construction: true } : opts;
    if (e.type === 'line') {
      created.push(addLine(sketch, reflect(e.p1), reflect(e.p2), construction));
    } else if (e.type === 'circle') {
      created.push(addCircle(sketch, reflect(e.center), e.radius, construction));
    } else if (e.type === 'arc') {
      // Reflection reverses orientation, so the arc's winding flips.
      created.push(
        addArc(sketch, reflect(e.center), reflect(e.p1), reflect(e.p2), !e.ccw, construction)
      );
    } else if (e.type === 'ellipse') {
      // Reflecting a direction θ across an axis at angle φ gives 2φ − θ, so the
      // major axis rotation mirrors; the semi-axes are unchanged.
      const axisAngle = Math.atan2(by - ay, bx - ax);
      created.push(
        addEllipse(sketch, reflect(e.center), e.rx, e.ry, 2 * axisAngle - e.rotation, construction)
      );
    } else if (e.type === 'spline') {
      created.push(
        addSpline(
          sketch,
          reflect(e.p1),
          reflect(e.p2),
          reflect(e.c1),
          reflect(e.c2),
          construction
        )
      );
    }
  }
  return created;
}

/** Point ids referenced by an entity (center first for circle/arc). */
export function entityPointIds(entity) {
  switch (entity.type) {
    case 'line':
      return [entity.p1, entity.p2];
    case 'circle':
      return [entity.center];
    case 'arc':
      return [entity.center, entity.p1, entity.p2];
    case 'ellipse':
      return [entity.center];
    case 'spline':
      return [entity.p1, entity.p2, entity.c1, entity.c2];
    default:
      return [];
  }
}

/** Translate a set of points rigidly by (dx, dy). */
export function translatePoints(sketch, ids, dx, dy) {
  for (const pid of ids) {
    const p = sketch.points[pid];
    if (!p) continue;
    p.x += dx;
    p.y += dy;
  }
}

/** Ids (entity/point) a constraint references. */
export function constraintRefs(constraint) {
  switch (constraint.type) {
    case 'horizontal':
    case 'vertical':
      return [constraint.line];
    case 'coincident':
      return [constraint.a, constraint.b];
    case 'tangent':
      return [constraint.line, constraint.curve];
    case 'length':
      return [constraint.line];
    case 'radius':
      return [constraint.entity];
    case 'parallel':
    case 'perpendicular':
    case 'collinear':
    case 'equal':
    case 'concentric':
      return [constraint.a, constraint.b];
    case 'midpoint':
      return [constraint.point, constraint.line];
    case 'symmetric':
      return [constraint.a, constraint.b, constraint.line];
    case 'fix':
      return [constraint.point];
    default:
      return [];
  }
}

function removeConstraintsReferencing(sketch, ids) {
  const gone = new Set(ids);
  for (const c of Object.values(sketch.constraints)) {
    if (constraintRefs(c).some((ref) => gone.has(ref))) {
      delete sketch.constraints[c.id];
    }
  }
}

/**
 * Add a closed loop of connected lines through `points` (`[x, y]` pairs),
 * sharing corner point ids so the chain reads as one profile loop. A trailing
 * point coincident with the first is dropped. Returns the new line ids.
 */
export function addLoop(sketch, points, opts = {}) {
  const pts = points.slice();
  if (pts.length >= 2) {
    const [fx, fy] = pts[0];
    const [lx, ly] = pts[pts.length - 1];
    if (Math.hypot(lx - fx, ly - fy) < 1e-9) pts.pop();
  }
  if (pts.length < 2) return [];
  const ids = pts.map(([x, y]) => addPoint(sketch, x, y));
  const lines = [];
  for (let i = 0; i < ids.length; i++) {
    lines.push(addLine(sketch, ids[i], ids[(i + 1) % ids.length], opts));
  }
  return lines;
}

/** Remove points no entity references (and constraints that used them). */
export function pruneOrphanPoints(sketch) {
  removeOrphanPoints(sketch);
}

function removeOrphanPoints(sketch) {
  const used = new Set();
  for (const e of Object.values(sketch.entities)) {
    for (const pid of entityPointIds(e)) used.add(pid);
  }
  const orphans = Object.keys(sketch.points).filter((pid) => !used.has(pid));
  for (const pid of orphans) delete sketch.points[pid];
  removeConstraintsReferencing(sketch, orphans);
}

/** Delete an entity plus constraints that reference it and orphaned points. */
export function deleteEntity(sketch, id) {
  if (!sketch.entities[id]) return;
  delete sketch.entities[id];
  removeConstraintsReferencing(sketch, [id]);
  removeOrphanPoints(sketch);
}

/** Delete a point by deleting every entity that uses it. */
export function deletePoint(sketch, pid) {
  const users = Object.values(sketch.entities).filter((e) =>
    entityPointIds(e).includes(pid)
  );
  for (const e of users) deleteEntity(sketch, e.id);
  // A never-referenced point (e.g. an abandoned draft start) has no users.
  if (sketch.points[pid]) {
    delete sketch.points[pid];
    removeConstraintsReferencing(sketch, [pid]);
  }
}

export function deleteConstraint(sketch, id) {
  delete sketch.constraints[id];
}

/** Current radius of a circle or arc entity. */
export function entityRadius(sketch, entity) {
  if (entity.type === 'circle') return entity.radius;
  const c = sketch.points[entity.center];
  const p1 = sketch.points[entity.p1];
  return Math.hypot(p1.x - c.x, p1.y - c.y);
}

/**
 * Validate a prospective constraint against the sketch.
 * Returns an error string, or null if applicable.
 */
export function validateConstraint(sketch, constraint) {
  const line = sketch.entities[constraint.line];
  switch (constraint.type) {
    case 'horizontal':
    case 'vertical':
    case 'length':
      if (!line || line.type !== 'line') return 'requires a line';
      if (constraint.type === 'length' && !(constraint.value > 0)) {
        return 'length must be positive';
      }
      return null;
    case 'coincident': {
      if (constraint.a === constraint.b) return 'requires two distinct points';
      if (!sketch.points[constraint.a] || !sketch.points[constraint.b]) {
        return 'requires two points';
      }
      return null;
    }
    case 'tangent': {
      const curve = sketch.entities[constraint.curve];
      if (!line || line.type !== 'line') return 'requires a line';
      if (!curve || (curve.type !== 'circle' && curve.type !== 'arc')) {
        return 'requires a circle or arc';
      }
      return null;
    }
    case 'radius': {
      const curve = sketch.entities[constraint.entity];
      if (!curve || (curve.type !== 'circle' && curve.type !== 'arc')) {
        return 'requires a circle or arc';
      }
      if (!(constraint.value > 0)) return 'radius must be positive';
      return null;
    }
    case 'parallel':
    case 'perpendicular':
    case 'collinear': {
      if (constraint.a === constraint.b) return 'requires two distinct lines';
      const la = sketch.entities[constraint.a];
      const lb = sketch.entities[constraint.b];
      if (!la || la.type !== 'line' || !lb || lb.type !== 'line') {
        return 'requires two lines';
      }
      return null;
    }
    case 'equal': {
      if (constraint.a === constraint.b) return 'requires two distinct entities';
      const ea = sketch.entities[constraint.a];
      const eb = sketch.entities[constraint.b];
      if (!ea || !eb) return 'requires two entities';
      const isCurve = (e) => e.type === 'circle' || e.type === 'arc';
      if (ea.type === 'line' && eb.type === 'line') return null;
      if (isCurve(ea) && isCurve(eb)) return null;
      return 'requires two lines or two circles/arcs';
    }
    case 'concentric': {
      if (constraint.a === constraint.b) return 'requires two distinct curves';
      const ea = sketch.entities[constraint.a];
      const eb = sketch.entities[constraint.b];
      const isCurve = (e) => e && (e.type === 'circle' || e.type === 'arc');
      if (!isCurve(ea) || !isCurve(eb)) return 'requires two circles or arcs';
      return null;
    }
    case 'midpoint': {
      const ml = sketch.entities[constraint.line];
      if (!ml || ml.type !== 'line') return 'requires a line';
      if (!sketch.points[constraint.point]) return 'requires a point';
      if (constraint.point === ml.p1 || constraint.point === ml.p2) {
        return 'point cannot be an endpoint of the line';
      }
      return null;
    }
    case 'symmetric': {
      const axis = sketch.entities[constraint.line];
      if (!axis || axis.type !== 'line') return 'requires an axis line';
      if (constraint.a === constraint.b) return 'requires two distinct points';
      if (!sketch.points[constraint.a] || !sketch.points[constraint.b]) {
        return 'requires two points';
      }
      return null;
    }
    case 'fix': {
      if (!sketch.points[constraint.point]) return 'requires a point';
      return null;
    }
    default:
      return `unknown constraint type: ${constraint.type}`;
  }
}
