import { describe, expect, it } from 'vitest';
import {
  addArc,
  addCircle,
  addConstraint,
  addEllipse,
  addLine,
  addLoop,
  addPoint,
  addPolygon,
  addRectangle,
  addSlot,
  addSpline,
  constraintRefs,
  createSketch,
  deleteConstraint,
  deleteEntity,
  deletePoint,
  entityPointIds,
  entityRadius,
  mirrorEntities,
  translatePoints,
  validateConstraint,
} from './model.js';
import { extractProfile } from './profile.js';

describe('sketch model', () => {
  it('creates points, lines, circles, and arcs with unique ids', () => {
    const s = createSketch();
    const a = addPoint(s, 0, 0);
    const b = addPoint(s, 1, 0);
    const c = addPoint(s, 0.5, 0.5);
    const line = addLine(s, a, b);
    const circle = addCircle(s, c, 2);
    const arc = addArc(s, c, a, b, true);
    expect(new Set([a, b, c, line, circle, arc]).size).toBe(6);
    expect(s.entities[line]).toMatchObject({ type: 'line', p1: a, p2: b });
    expect(s.entities[circle]).toMatchObject({ type: 'circle', radius: 2 });
    expect(s.entities[arc]).toMatchObject({ type: 'arc', ccw: true });
  });

  it('addLoop chains points into a closed loop of shared-corner lines', () => {
    const s = createSketch();
    const ids = addLoop(s, [
      [0, 0],
      [2, 0],
      [2, 1],
      [0, 1],
    ]);
    expect(ids).toHaveLength(4);
    // Consecutive lines share a corner point id (closed chain).
    for (let i = 0; i < 4; i++) {
      expect(s.entities[ids[i]].p2).toBe(s.entities[ids[(i + 1) % 4]].p1);
    }
    // A closed loop of 4 lines extracts as a closed profile.
    expect(extractProfile(s, 'XY').closed).toBe(true);
  });

  it('addLoop drops a trailing point coincident with the first', () => {
    const s = createSketch();
    const ids = addLoop(s, [
      [0, 0],
      [1, 0],
      [1, 1],
      [0, 0],
    ]);
    expect(ids).toHaveLength(3);
  });

  it('entityPointIds covers every entity type', () => {
    const s = createSketch();
    const a = addPoint(s, 0, 0);
    const b = addPoint(s, 1, 0);
    const line = addLine(s, a, b);
    const circle = addCircle(s, a, 1);
    const arc = addArc(s, a, b, b, true);
    expect(entityPointIds(s.entities[line])).toEqual([a, b]);
    expect(entityPointIds(s.entities[circle])).toEqual([a]);
    expect(entityPointIds(s.entities[arc])).toEqual([a, b, b]);
  });

  it('addRectangle builds 4 connected lines with h/v constraints', () => {
    const s = createSketch();
    const [bottom, right, top, left] = addRectangle(s, 0, 0, 3, 2);
    expect(Object.keys(s.entities)).toHaveLength(4);
    expect(Object.keys(s.points)).toHaveLength(4);
    const constraints = Object.values(s.constraints);
    expect(constraints.filter((c) => c.type === 'horizontal')).toHaveLength(2);
    expect(constraints.filter((c) => c.type === 'vertical')).toHaveLength(2);
    // Corners are shared: bottom end == right start, etc.
    expect(s.entities[bottom].p2).toBe(s.entities[right].p1);
    expect(s.entities[right].p2).toBe(s.entities[top].p1);
    expect(s.entities[top].p2).toBe(s.entities[left].p1);
    expect(s.entities[left].p2).toBe(s.entities[bottom].p1);
  });

  it('addPolygon builds a closed n-gon of chained lines', () => {
    const s = createSketch();
    const lines = addPolygon(s, 0, 0, 2, 6);
    expect(lines).toHaveLength(6);
    expect(Object.keys(s.points)).toHaveLength(6);
    // Each line's end is the next line's start (shared corners → closed loop).
    for (let i = 0; i < 6; i++) {
      expect(s.entities[lines[i]].p2).toBe(s.entities[lines[(i + 1) % 6]].p1);
    }
    // First vertex sits at (radius, 0) with the default rotation.
    const v0 = s.points[s.entities[lines[0]].p1];
    expect(v0.x).toBeCloseTo(2, 12);
    expect(v0.y).toBeCloseTo(0, 12);
    // A regular polygon extracts as a closed profile.
    expect(extractProfile(s, 'XY').closed).toBe(true);
  });

  it('addPolygon clamps sides to a minimum of 3 and rounds', () => {
    const s = createSketch();
    expect(addPolygon(s, 0, 0, 1, 2)).toHaveLength(3);
    const s2 = createSketch();
    expect(addPolygon(s2, 0, 0, 1, 5.4)).toHaveLength(5);
  });

  it('addSlot builds a closed obround of two lines and two arcs', () => {
    const s = createSketch();
    const ids = addSlot(s, 0, 0, 4, 0, 1);
    expect(ids).toHaveLength(4);
    const types = ids.map((id) => s.entities[id].type);
    expect(types).toEqual(['line', 'arc', 'line', 'arc']);
    // Rails sit at y = ±radius above/below the horizontal centerline.
    const left = s.entities[ids[0]];
    expect(s.points[left.p1]).toMatchObject({ x: 0, y: 1 });
    expect(s.points[left.p2]).toMatchObject({ x: 4, y: 1 });
    // The loop closes into one profile.
    expect(extractProfile(s, 'XY').closed).toBe(true);
  });

  it('mirrorEntities reflects a copy across an axis, flipping arc winding', () => {
    const s = createSketch();
    const a = addPoint(s, 1, 1);
    const b = addPoint(s, 3, 2);
    const line = addLine(s, a, b);
    const c = addPoint(s, 2, 2);
    const arc = addArc(s, c, a, b, true);
    // Mirror across the X axis (y = 0): (x, y) -> (x, -y).
    const copies = mirrorEntities(s, [line, arc], 0, 0, 1, 0);
    expect(copies).toHaveLength(2);
    const mline = s.entities[copies[0]];
    expect(s.points[mline.p1]).toMatchObject({ x: 1, y: -1 });
    expect(s.points[mline.p2]).toMatchObject({ x: 3, y: -2 });
    // Shared endpoints are reflected once and reused by both copies.
    const marc = s.entities[copies[1]];
    expect(marc.p1).toBe(mline.p1);
    expect(marc.p2).toBe(mline.p2);
    expect(marc.ccw).toBe(false); // winding flips under reflection
  });

  it('mirrorEntities preserves the construction flag of its sources', () => {
    const s = createSketch();
    const a = addPoint(s, 0, 1);
    const b = addPoint(s, 2, 1);
    const line = addLine(s, a, b, { construction: true });
    const [copy] = mirrorEntities(s, [line], 0, 0, 1, 0);
    expect(s.entities[copy].construction).toBe(true);
  });

  it('construction entities are excluded from profile extraction', () => {
    const s = createSketch();
    addPolygon(s, 0, 0, 2, 4); // a real closed square
    addLine(s, addPoint(s, -5, 0), addPoint(s, 5, 0), { construction: true });
    // The stray construction line would otherwise break the single loop.
    expect(extractProfile(s, 'XY').closed).toBe(true);
  });

  it('entityRadius reads circles directly and arcs from geometry', () => {
    const s = createSketch();
    const c = addPoint(s, 1, 1);
    const start = addPoint(s, 4, 1);
    const end = addPoint(s, 1, 4);
    const circle = addCircle(s, c, 2.5);
    const arc = addArc(s, c, start, end, true);
    expect(entityRadius(s, s.entities[circle])).toBe(2.5);
    expect(entityRadius(s, s.entities[arc])).toBeCloseTo(3, 12);
  });

  it('deleteEntity removes referencing constraints and orphan points', () => {
    const s = createSketch();
    const a = addPoint(s, 0, 0);
    const b = addPoint(s, 1, 0);
    const c = addPoint(s, 2, 0);
    const l1 = addLine(s, a, b);
    const l2 = addLine(s, b, c);
    addConstraint(s, { type: 'horizontal', line: l1 });
    addConstraint(s, { type: 'length', line: l2, value: 1 });
    deleteEntity(s, l1);
    expect(s.entities[l1]).toBeUndefined();
    expect(s.points[a]).toBeUndefined(); // orphaned
    expect(s.points[b]).toBeDefined(); // still used by l2
    expect(
      Object.values(s.constraints).some((k) => k.type === 'horizontal')
    ).toBe(false);
    expect(Object.values(s.constraints).some((k) => k.type === 'length')).toBe(
      true
    );
  });

  it('deletePoint removes entities using the point', () => {
    const s = createSketch();
    const a = addPoint(s, 0, 0);
    const b = addPoint(s, 1, 0);
    const c = addPoint(s, 2, 0);
    addLine(s, a, b);
    addLine(s, b, c);
    deletePoint(s, b);
    expect(Object.keys(s.entities)).toHaveLength(0);
    expect(Object.keys(s.points)).toHaveLength(0);
  });

  it('deletePoint removes a dangling unreferenced point', () => {
    const s = createSketch();
    const a = addPoint(s, 0, 0);
    deletePoint(s, a);
    expect(Object.keys(s.points)).toHaveLength(0);
  });

  it('translatePoints moves listed points rigidly and skips unknown ids', () => {
    const s = createSketch();
    const a = addPoint(s, 0, 0);
    const b = addPoint(s, 1, 2);
    const c = addPoint(s, 5, 5);
    translatePoints(s, [a, b, 'missing'], 10, -3);
    expect(s.points[a]).toMatchObject({ x: 10, y: -3 });
    expect(s.points[b]).toMatchObject({ x: 11, y: -1 });
    expect(s.points[c]).toMatchObject({ x: 5, y: 5 });
  });

  it('deleteConstraint removes only the constraint', () => {
    const s = createSketch();
    const a = addPoint(s, 0, 0);
    const b = addPoint(s, 1, 0);
    const line = addLine(s, a, b);
    const cid = addConstraint(s, { type: 'horizontal', line });
    deleteConstraint(s, cid);
    expect(Object.keys(s.constraints)).toHaveLength(0);
    expect(s.entities[line]).toBeDefined();
  });

  it('constraintRefs names referenced ids for each type', () => {
    expect(constraintRefs({ type: 'horizontal', line: 'e1' })).toEqual(['e1']);
    expect(constraintRefs({ type: 'coincident', a: 'p1', b: 'p2' })).toEqual([
      'p1',
      'p2',
    ]);
    expect(constraintRefs({ type: 'tangent', line: 'e1', curve: 'e2' })).toEqual(
      ['e1', 'e2']
    );
    expect(constraintRefs({ type: 'radius', entity: 'e3' })).toEqual(['e3']);
    expect(constraintRefs({ type: 'parallel', a: 'e1', b: 'e2' })).toEqual([
      'e1',
      'e2',
    ]);
    expect(
      constraintRefs({ type: 'perpendicular', a: 'e1', b: 'e2' })
    ).toEqual(['e1', 'e2']);
    expect(constraintRefs({ type: 'collinear', a: 'e1', b: 'e2' })).toEqual([
      'e1',
      'e2',
    ]);
    expect(constraintRefs({ type: 'equal', a: 'e1', b: 'e2' })).toEqual([
      'e1',
      'e2',
    ]);
    expect(constraintRefs({ type: 'concentric', a: 'e1', b: 'e2' })).toEqual([
      'e1',
      'e2',
    ]);
    expect(
      constraintRefs({ type: 'midpoint', point: 'p1', line: 'e1' })
    ).toEqual(['p1', 'e1']);
    expect(
      constraintRefs({ type: 'symmetric', a: 'p1', b: 'p2', line: 'e1' })
    ).toEqual(['p1', 'p2', 'e1']);
    expect(constraintRefs({ type: 'fix', point: 'p1' })).toEqual(['p1']);
    expect(
      constraintRefs({ type: 'distance', a: 'p1', b: 'p2' })
    ).toEqual(['p1', 'p2']);
    expect(
      constraintRefs({ type: 'pdistance', point: 'p1', line: 'e1' })
    ).toEqual(['p1', 'e1']);
    expect(constraintRefs({ type: 'angle', a: 'e1', b: 'e2' })).toEqual([
      'e1',
      'e2',
    ]);
    expect(constraintRefs({ type: 'diameter', entity: 'e3' })).toEqual(['e3']);
  });

  it('validateConstraint handles dimension constraints', () => {
    const s = createSketch();
    const a = addPoint(s, 0, 0);
    const b = addPoint(s, 4, 0);
    const c = addPoint(s, 0, 3);
    const line = addLine(s, a, b);
    const line2 = addLine(s, a, c);
    const circle = addCircle(s, a, 1);
    const off = addPoint(s, 2, 5);

    expect(validateConstraint(s, { type: 'distance', a, b, value: 5 })).toBeNull();
    expect(
      validateConstraint(s, { type: 'distance', a, b: a, value: 5 })
    ).toMatch(/distinct/);
    expect(
      validateConstraint(s, { type: 'distance', a, b, value: 0 })
    ).toMatch(/positive/);

    expect(
      validateConstraint(s, { type: 'pdistance', point: off, line, value: 2 })
    ).toBeNull();
    expect(
      validateConstraint(s, { type: 'pdistance', point: a, line, value: 2 })
    ).toMatch(/endpoint/);
    expect(
      validateConstraint(s, { type: 'pdistance', point: off, line: circle, value: 2 })
    ).toMatch(/line/);

    expect(
      validateConstraint(s, { type: 'angle', a: line, b: line2, value: 1 })
    ).toBeNull();
    expect(
      validateConstraint(s, { type: 'angle', a: line, b: line, value: 1 })
    ).toMatch(/distinct/);
    expect(
      validateConstraint(s, { type: 'angle', a: line, b: circle, value: 1 })
    ).toMatch(/lines/);

    expect(
      validateConstraint(s, { type: 'diameter', entity: circle, value: 3 })
    ).toBeNull();
    expect(
      validateConstraint(s, { type: 'diameter', entity: line, value: 3 })
    ).toMatch(/circle or arc/);
  });

  it('validateConstraint accepts valid and rejects invalid combos', () => {
    const s = createSketch();
    const a = addPoint(s, 0, 0);
    const b = addPoint(s, 1, 0);
    const line = addLine(s, a, b);
    const circle = addCircle(s, a, 1);
    expect(validateConstraint(s, { type: 'horizontal', line })).toBeNull();
    expect(validateConstraint(s, { type: 'horizontal', line: circle })).toMatch(
      /line/
    );
    expect(validateConstraint(s, { type: 'coincident', a, b })).toBeNull();
    expect(validateConstraint(s, { type: 'coincident', a, b: a })).toMatch(
      /distinct/
    );
    expect(
      validateConstraint(s, { type: 'tangent', line, curve: circle })
    ).toBeNull();
    expect(
      validateConstraint(s, { type: 'tangent', line, curve: line })
    ).toMatch(/circle or arc/);
    expect(
      validateConstraint(s, { type: 'length', line, value: 2 })
    ).toBeNull();
    expect(validateConstraint(s, { type: 'length', line, value: 0 })).toMatch(
      /positive/
    );
    expect(
      validateConstraint(s, { type: 'radius', entity: circle, value: 1 })
    ).toBeNull();
    expect(
      validateConstraint(s, { type: 'radius', entity: line, value: 1 })
    ).toMatch(/circle or arc/);
    expect(validateConstraint(s, { type: 'nope' })).toMatch(/unknown/);
  });

  it('validateConstraint handles the added relations', () => {
    const s = createSketch();
    const a = addPoint(s, 0, 0);
    const b = addPoint(s, 1, 0);
    const c = addPoint(s, 2, 2);
    const line1 = addLine(s, a, b);
    const line2 = addLine(s, b, c);
    const circle1 = addCircle(s, a, 1);
    const circle2 = addCircle(s, c, 2);

    // parallel / perpendicular / collinear — two lines.
    for (const type of ['parallel', 'perpendicular', 'collinear']) {
      expect(validateConstraint(s, { type, a: line1, b: line2 })).toBeNull();
      expect(
        validateConstraint(s, { type, a: line1, b: circle1 })
      ).toMatch(/lines/);
      expect(validateConstraint(s, { type, a: line1, b: line1 })).toMatch(
        /distinct/
      );
    }

    // equal — two lines, or two curves, but not a mix.
    expect(
      validateConstraint(s, { type: 'equal', a: line1, b: line2 })
    ).toBeNull();
    expect(
      validateConstraint(s, { type: 'equal', a: circle1, b: circle2 })
    ).toBeNull();
    expect(
      validateConstraint(s, { type: 'equal', a: line1, b: circle1 })
    ).toMatch(/lines or two circles/);

    // concentric — two curves.
    expect(
      validateConstraint(s, { type: 'concentric', a: circle1, b: circle2 })
    ).toBeNull();
    expect(
      validateConstraint(s, { type: 'concentric', a: line1, b: circle1 })
    ).toMatch(/circles or arcs/);

    // midpoint — a point not on the line's ends, plus a line.
    const mid = addPoint(s, 5, 5);
    expect(
      validateConstraint(s, { type: 'midpoint', point: mid, line: line1 })
    ).toBeNull();
    expect(
      validateConstraint(s, { type: 'midpoint', point: a, line: line1 })
    ).toMatch(/endpoint/);
    expect(
      validateConstraint(s, { type: 'midpoint', point: mid, line: circle1 })
    ).toMatch(/line/);

    // symmetric — two distinct points about an axis line.
    expect(
      validateConstraint(s, { type: 'symmetric', a, b: c, line: line1 })
    ).toBeNull();
    expect(
      validateConstraint(s, { type: 'symmetric', a, b: a, line: line1 })
    ).toMatch(/distinct/);
    expect(
      validateConstraint(s, { type: 'symmetric', a, b: c, line: circle1 })
    ).toMatch(/axis line/);

    // fix — a point.
    expect(validateConstraint(s, { type: 'fix', point: a })).toBeNull();
    expect(validateConstraint(s, { type: 'fix', point: 'nope' })).toMatch(
      /point/
    );
  });

  it('addEllipse stores center, radii, and rotation', () => {
    const s = createSketch();
    const c = addPoint(s, 1, 2);
    const id = addEllipse(s, c, 3, 1, Math.PI / 4);
    const e = s.entities[id];
    expect(e).toMatchObject({ type: 'ellipse', center: c, rx: 3, ry: 1 });
    expect(e.rotation).toBeCloseTo(Math.PI / 4, 12);
    // Only the center is a referenced point.
    expect(entityPointIds(e)).toEqual([c]);
  });

  it('addSpline references both endpoints and both control points', () => {
    const s = createSketch();
    const p1 = addPoint(s, 0, 0);
    const p2 = addPoint(s, 4, 0);
    const c1 = addPoint(s, 1, 2);
    const c2 = addPoint(s, 3, 2);
    const id = addSpline(s, p1, p2, c1, c2);
    const e = s.entities[id];
    expect(e).toMatchObject({ type: 'spline', p1, p2, c1, c2 });
    expect(entityPointIds(e)).toEqual([p1, p2, c1, c2]);
  });

  it('deleting a spline removes its orphaned control points', () => {
    const s = createSketch();
    const p1 = addPoint(s, 0, 0);
    const p2 = addPoint(s, 4, 0);
    const c1 = addPoint(s, 1, 2);
    const c2 = addPoint(s, 3, 2);
    const id = addSpline(s, p1, p2, c1, c2);
    deleteEntity(s, id);
    expect(s.entities[id]).toBeUndefined();
    // Every point was used only by the spline, so all are gone.
    expect(Object.keys(s.points)).toHaveLength(0);
  });

  it('construction flag is settable on ellipse and spline', () => {
    const s = createSketch();
    const c = addPoint(s, 0, 0);
    const el = addEllipse(s, c, 2, 1, 0, { construction: true });
    expect(s.entities[el].construction).toBe(true);
    const p1 = addPoint(s, 0, 0);
    const p2 = addPoint(s, 1, 0);
    const h1 = addPoint(s, 0, 1);
    const h2 = addPoint(s, 1, 1);
    const sp = addSpline(s, p1, p2, h1, h2, { construction: true });
    expect(s.entities[sp].construction).toBe(true);
  });

  it('mirrorEntities reflects an ellipse (rotation negates across the u axis)', () => {
    const s = createSketch();
    const c = addPoint(s, 2, 3);
    const id = addEllipse(s, c, 4, 1, 0.5);
    // Mirror across the x axis (u): center.y flips, rotation negates.
    const [copy] = mirrorEntities(s, [id], 0, 0, 1, 0);
    const e = s.entities[copy];
    expect(s.points[e.center]).toMatchObject({ x: 2, y: -3 });
    expect(e.rotation).toBeCloseTo(-0.5, 12);
    expect(e.rx).toBe(4);
    expect(e.ry).toBe(1);
  });

  it('mirrorEntities reflects a spline endpoints and handles', () => {
    const s = createSketch();
    const p1 = addPoint(s, 0, 1);
    const p2 = addPoint(s, 4, 1);
    const c1 = addPoint(s, 1, 3);
    const c2 = addPoint(s, 3, 3);
    const id = addSpline(s, p1, p2, c1, c2);
    const [copy] = mirrorEntities(s, [id], 0, 0, 1, 0); // across x axis
    const e = s.entities[copy];
    expect(s.points[e.p1]).toMatchObject({ x: 0, y: -1 });
    expect(s.points[e.c1]).toMatchObject({ x: 1, y: -3 });
    expect(s.points[e.c2]).toMatchObject({ x: 3, y: -3 });
  });
});
