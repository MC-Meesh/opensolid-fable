/**
 * Sketch data model: plain serializable objects, mutated in place.
 *
 * A sketch owns three id-keyed tables:
 *   points:      { id, x, y }
 *   entities:    { id, type: 'line',   p1, p2 }
 *                { id, type: 'circle', center, radius }
 *                { id, type: 'arc',    center, p1, p2, ccw }   (start p1 → end p2)
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
 */

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

export function addLine(sketch, p1, p2) {
  const id = nextId(sketch, 'e');
  sketch.entities[id] = { id, type: 'line', p1, p2 };
  return id;
}

export function addCircle(sketch, center, radius) {
  const id = nextId(sketch, 'e');
  sketch.entities[id] = { id, type: 'circle', center, radius };
  return id;
}

export function addArc(sketch, center, p1, p2, ccw = true) {
  const id = nextId(sketch, 'e');
  sketch.entities[id] = { id, type: 'arc', center, p1, p2, ccw };
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

/** Point ids referenced by an entity (center first for circle/arc). */
export function entityPointIds(entity) {
  switch (entity.type) {
    case 'line':
      return [entity.p1, entity.p2];
    case 'circle':
      return [entity.center];
    case 'arc':
      return [entity.center, entity.p1, entity.p2];
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
