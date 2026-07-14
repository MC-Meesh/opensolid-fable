import { describe, expect, it } from 'vitest';
import {
  addArc,
  addCircle,
  addLine,
  addPoint,
  createSketch,
} from './model.js';
import {
  distanceDim,
  inferDimension,
  measureConstraint,
  orientForPlacement,
} from './dimension.js';

const pk = (kind, id) => ({ kind, id });

describe('inferDimension', () => {
  it('returns null for an empty selection', () => {
    expect(inferDimension(createSketch(), [])).toBeNull();
  });

  it('infers length from a single line', () => {
    const s = createSketch();
    const line = addLine(s, addPoint(s, 0, 0), addPoint(s, 3, 4));
    const dim = inferDimension(s, [pk('entity', line)]);
    expect(dim.kind).toBe('length');
    expect(dim.proto).toMatchObject({ type: 'length', line });
    expect(dim.measured).toBeCloseTo(5, 9);
    expect(dim.anchor).toEqual({ x: 1.5, y: 2 });
  });

  it('infers diameter from a circle', () => {
    const s = createSketch();
    const circle = addCircle(s, addPoint(s, 1, 1), 2);
    const dim = inferDimension(s, [pk('entity', circle)]);
    expect(dim.kind).toBe('diameter');
    expect(dim.proto).toMatchObject({ type: 'diameter', entity: circle });
    expect(dim.measured).toBeCloseTo(4, 9);
    expect(dim.anchor).toEqual({ x: 1, y: 1 });
  });

  it('infers radius from an arc', () => {
    const s = createSketch();
    const arc = addArc(s, addPoint(s, 0, 0), addPoint(s, 2, 0), addPoint(s, 0, 2));
    const dim = inferDimension(s, [pk('entity', arc)]);
    expect(dim.kind).toBe('radius');
    expect(dim.proto).toMatchObject({ type: 'radius', entity: arc });
    expect(dim.measured).toBeCloseTo(2, 9);
  });

  it('waits (null) after a single point', () => {
    const s = createSketch();
    const p = addPoint(s, 0, 0);
    expect(inferDimension(s, [pk('point', p)])).toBeNull();
  });

  it('infers an aligned distance from two points', () => {
    const s = createSketch();
    const a = addPoint(s, 0, 0);
    const b = addPoint(s, 3, 4);
    const dim = inferDimension(s, [pk('point', a), pk('point', b)]);
    expect(dim.kind).toBe('distance');
    expect(dim.proto).toMatchObject({ type: 'distance', a, b, orient: 'aligned' });
    expect(dim.measured).toBeCloseTo(5, 9);
    expect(dim.needsPlacement).toBe(true);
  });

  it('infers perpendicular distance from a point and a line', () => {
    const s = createSketch();
    const a = addPoint(s, -5, 0);
    const b = addPoint(s, 5, 0);
    const line = addLine(s, a, b);
    const p = addPoint(s, 2, 3);
    const dim = inferDimension(s, [pk('point', p), pk('entity', line)]);
    expect(dim.kind).toBe('pdistance');
    expect(dim.proto).toMatchObject({ type: 'pdistance', point: p, line });
    expect(dim.measured).toBeCloseTo(3, 9);
    // Anchor sits between the point and its foot on the line.
    expect(dim.anchor).toEqual({ x: 2, y: 1.5 });
  });

  it('infers distance between two parallel lines', () => {
    const s = createSketch();
    const l1 = addLine(s, addPoint(s, 0, 0), addPoint(s, 4, 0));
    const l2 = addLine(s, addPoint(s, 0, 2), addPoint(s, 4, 2));
    const dim = inferDimension(s, [pk('entity', l1), pk('entity', l2)]);
    expect(dim.kind).toBe('pdistance');
    expect(dim.measured).toBeCloseTo(2, 9);
  });

  it('infers the acute angle between two crossing lines', () => {
    const s = createSketch();
    const l1 = addLine(s, addPoint(s, 0, 0), addPoint(s, 4, 0));
    const l2 = addLine(s, addPoint(s, 0, 0), addPoint(s, 4, 4));
    const dim = inferDimension(s, [pk('entity', l1), pk('entity', l2)]);
    expect(dim.kind).toBe('angle');
    expect(dim.proto).toMatchObject({ type: 'angle', a: l1, b: l2 });
    expect(dim.measured).toBeCloseTo(Math.PI / 4, 9);
    expect(dim.anchor.x).toBeCloseTo(0, 9);
    expect(dim.anchor.y).toBeCloseTo(0, 9);
  });

  it('folds obtuse crossings to the acute angle', () => {
    const s = createSketch();
    const l1 = addLine(s, addPoint(s, 0, 0), addPoint(s, 4, 0));
    // Direction 135° — the undirected angle to the x-axis is 45°.
    const l2 = addLine(s, addPoint(s, 0, 0), addPoint(s, -4, 4));
    const dim = inferDimension(s, [pk('entity', l1), pk('entity', l2)]);
    expect(dim.kind).toBe('angle');
    expect(dim.measured).toBeCloseTo(Math.PI / 4, 9);
  });

  it('errors on an unsupported pair', () => {
    const s = createSketch();
    const circle = addCircle(s, addPoint(s, 0, 0), 1);
    const line = addLine(s, addPoint(s, 5, 0), addPoint(s, 6, 0));
    const res = inferDimension(s, [pk('entity', circle), pk('entity', line)]);
    expect(res.error).toBeTruthy();
  });

  it('errors on too many picks', () => {
    const s = createSketch();
    const a = addPoint(s, 0, 0);
    const b = addPoint(s, 1, 0);
    const c = addPoint(s, 2, 0);
    const res = inferDimension(s, [pk('point', a), pk('point', b), pk('point', c)]);
    expect(res.error).toBeTruthy();
  });

  it('errors on a stale (deleted) pick', () => {
    const s = createSketch();
    const res = inferDimension(s, [pk('entity', 'gone')]);
    expect(res.error).toBeTruthy();
  });
});

describe('distanceDim orientation', () => {
  it('measures only Δx / Δy for horizontal / vertical orients', () => {
    const a = { id: 'p1', x: 0, y: 0 };
    const b = { id: 'p2', x: 3, y: 4 };
    expect(distanceDim({}, a, b, 'horizontal').measured).toBeCloseTo(3, 9);
    expect(distanceDim({}, a, b, 'vertical').measured).toBeCloseTo(4, 9);
    expect(distanceDim({}, a, b, 'aligned').measured).toBeCloseTo(5, 9);
  });
});

describe('orientForPlacement', () => {
  // Segment along the x-axis from (0,0) to (4,0); midpoint (2,0).
  it('pulling the text sideways gives a vertical distance', () => {
    expect(orientForPlacement(0, 0, 4, 0, 8, 0)).toBe('vertical');
  });

  it('pulling the text above gives a horizontal distance', () => {
    expect(orientForPlacement(0, 0, 4, 0, 2, 5)).toBe('horizontal');
  });

  it('pulling the text diagonally gives an aligned distance', () => {
    expect(orientForPlacement(0, 0, 4, 0, 5, 3)).toBe('aligned');
  });
});

describe('measureConstraint', () => {
  it('reads live values for each dimension type', () => {
    const s = createSketch();
    const a = addPoint(s, 0, 0);
    const b = addPoint(s, 6, 0);
    const line = addLine(s, a, b);
    const circle = addCircle(s, addPoint(s, 0, 0), 2.5);
    expect(measureConstraint(s, { type: 'length', line })).toBeCloseTo(6, 9);
    expect(
      measureConstraint(s, { type: 'diameter', entity: circle })
    ).toBeCloseTo(5, 9);
    expect(
      measureConstraint(s, { type: 'distance', a, b, orient: 'horizontal' })
    ).toBeCloseTo(6, 9);
  });

  it('returns null for stale references', () => {
    const s = createSketch();
    expect(measureConstraint(s, { type: 'length', line: 'gone' })).toBeNull();
  });
});
