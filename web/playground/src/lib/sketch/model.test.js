import { describe, expect, it } from 'vitest';
import {
  addArc,
  addCircle,
  addConstraint,
  addLine,
  addPoint,
  addRectangle,
  constraintRefs,
  createSketch,
  deleteConstraint,
  deleteEntity,
  deletePoint,
  entityPointIds,
  entityRadius,
  translatePoints,
  validateConstraint,
} from './model.js';

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
});
