import { describe, expect, it } from 'vitest';
import {
  addArc,
  addCircle,
  addConstraint,
  addLine,
  addPoint,
  addRectangle,
  createSketch,
  entityRadius,
} from './model.js';
import { solve } from './solver.js';

describe('constraint solver', () => {
  it('horizontal levels both endpoints', () => {
    const s = createSketch();
    const a = addPoint(s, 0, 1);
    const b = addPoint(s, 4, 3);
    const line = addLine(s, a, b);
    addConstraint(s, { type: 'horizontal', line });
    const result = solve(s);
    expect(result.converged).toBe(true);
    expect(s.points[a].y).toBeCloseTo(s.points[b].y, 9);
    expect(s.points[a].y).toBeCloseTo(2, 9); // met in the middle
  });

  it('horizontal respects a pinned endpoint', () => {
    const s = createSketch();
    const a = addPoint(s, 0, 1);
    const b = addPoint(s, 4, 3);
    const line = addLine(s, a, b);
    addConstraint(s, { type: 'horizontal', line });
    solve(s, { pinned: new Set([a]) });
    expect(s.points[a].y).toBe(1); // pinned untouched
    expect(s.points[b].y).toBeCloseTo(1, 9);
  });

  it('vertical aligns x coordinates', () => {
    const s = createSketch();
    const a = addPoint(s, 1, 0);
    const b = addPoint(s, 3, 5);
    const line = addLine(s, a, b);
    addConstraint(s, { type: 'vertical', line });
    solve(s);
    expect(s.points[a].x).toBeCloseTo(s.points[b].x, 9);
  });

  it('reports non-convergence when both endpoints are pinned', () => {
    const s = createSketch();
    const a = addPoint(s, 0, 0);
    const b = addPoint(s, 4, 3);
    const line = addLine(s, a, b);
    addConstraint(s, { type: 'horizontal', line });
    const result = solve(s, { pinned: new Set([a, b]), iterations: 10 });
    expect(result.converged).toBe(false);
    expect(result.error).toBeCloseTo(3, 9);
  });

  it('coincident merges two points', () => {
    const s = createSketch();
    const a = addPoint(s, 0, 0);
    const b = addPoint(s, 2, 2);
    const c = addPoint(s, 4, 0);
    const d = addPoint(s, 6, 2);
    addLine(s, a, b);
    addLine(s, c, d);
    addConstraint(s, { type: 'coincident', a: b, b: c });
    solve(s);
    expect(s.points[b].x).toBeCloseTo(s.points[c].x, 9);
    expect(s.points[b].y).toBeCloseTo(s.points[c].y, 9);
  });

  it('length drives a line to the target measure', () => {
    const s = createSketch();
    const a = addPoint(s, 0, 0);
    const b = addPoint(s, 3, 4);
    const line = addLine(s, a, b);
    addConstraint(s, { type: 'length', line, value: 10 });
    const result = solve(s);
    expect(result.converged).toBe(true);
    const len = Math.hypot(
      s.points[b].x - s.points[a].x,
      s.points[b].y - s.points[a].y
    );
    expect(len).toBeCloseTo(10, 7);
    // Direction preserved (3-4-5 triangle scaled to 6-8-10).
    expect(s.points[b].x - s.points[a].x).toBeCloseTo(6, 6);
  });

  it('radius sets a circle radius exactly', () => {
    const s = createSketch();
    const c = addPoint(s, 0, 0);
    const circle = addCircle(s, c, 1);
    addConstraint(s, { type: 'radius', entity: circle, value: 3.5 });
    solve(s);
    expect(s.entities[circle].radius).toBe(3.5);
  });

  it('radius moves arc endpoints onto the target circle', () => {
    const s = createSketch();
    const c = addPoint(s, 0, 0);
    const p1 = addPoint(s, 2, 0);
    const p2 = addPoint(s, 0, 3);
    const arc = addArc(s, c, p1, p2, true);
    addConstraint(s, { type: 'radius', entity: arc, value: 5 });
    const result = solve(s);
    expect(result.converged).toBe(true);
    expect(Math.hypot(s.points[p1].x, s.points[p1].y)).toBeCloseTo(5, 7);
    expect(Math.hypot(s.points[p2].x, s.points[p2].y)).toBeCloseTo(5, 7);
  });

  it('keeps arc endpoints equidistant from the center (built-in)', () => {
    const s = createSketch();
    const c = addPoint(s, 0, 0);
    const p1 = addPoint(s, 4, 0);
    const p2 = addPoint(s, 0, 2);
    addArc(s, c, p1, p2, true);
    solve(s);
    const arc = Object.values(s.entities)[0];
    expect(entityRadius(s, arc)).toBeCloseTo(
      Math.hypot(s.points[p2].x, s.points[p2].y),
      7
    );
  });

  it('built-in arc radius follows a pinned endpoint', () => {
    const s = createSketch();
    const c = addPoint(s, 0, 0);
    const p1 = addPoint(s, 4, 0);
    const p2 = addPoint(s, 0, 2);
    addArc(s, c, p1, p2, true);
    solve(s, { pinned: new Set([p2]) });
    expect(Math.hypot(s.points[p1].x, s.points[p1].y)).toBeCloseTo(2, 7);
  });

  it('tangent brings a line to touch a circle', () => {
    const s = createSketch();
    const a = addPoint(s, -5, 3);
    const b = addPoint(s, 5, 3);
    const line = addLine(s, a, b);
    const center = addPoint(s, 0, 0);
    const circle = addCircle(s, center, 1);
    addConstraint(s, { type: 'tangent', line, curve: circle });
    const result = solve(s);
    expect(result.converged).toBe(true);
    // Perpendicular distance from center to line equals radius.
    const A = s.points[a];
    const B = s.points[b];
    const C = s.points[center];
    const nx = -(B.y - A.y);
    const ny = B.x - A.x;
    const nl = Math.hypot(nx, ny);
    const d = Math.abs(((C.x - A.x) * nx + (C.y - A.y) * ny) / nl);
    expect(d).toBeCloseTo(1, 7);
  });

  it('tangent works against an arc using its live radius', () => {
    const s = createSketch();
    const a = addPoint(s, -5, 4);
    const b = addPoint(s, 5, 4);
    const line = addLine(s, a, b);
    const center = addPoint(s, 0, 0);
    const p1 = addPoint(s, 2, 0);
    const p2 = addPoint(s, 0, 2);
    const arc = addArc(s, center, p1, p2, true);
    addConstraint(s, { type: 'tangent', line, curve: arc });
    const result = solve(s, { pinned: new Set([center, p1, p2]) });
    expect(result.converged).toBe(true);
    const A = s.points[a];
    const B = s.points[b];
    const C = s.points[center];
    const nx = -(B.y - A.y);
    const ny = B.x - A.x;
    const nl = Math.hypot(nx, ny);
    const d = Math.abs(((C.x - A.x) * nx + (C.y - A.y) * ny) / nl);
    expect(d).toBeCloseTo(2, 6);
  });

  it('solves a rectangle with two dimensions to exact size', () => {
    const s = createSketch();
    // Sloppy near-rectangle; constraints should square it up at 4 x 2.
    const [bottom, right] = addRectangle(s, 0.1, -0.2, 3.7, 2.3);
    addConstraint(s, { type: 'length', line: bottom, value: 4 });
    addConstraint(s, { type: 'length', line: right, value: 2 });
    const result = solve(s);
    expect(result.converged).toBe(true);
    const b = s.entities[bottom];
    const r = s.entities[right];
    const width = Math.abs(s.points[b.p2].x - s.points[b.p1].x);
    const height = Math.abs(s.points[r.p2].y - s.points[r.p1].y);
    expect(width).toBeCloseTo(4, 6);
    expect(height).toBeCloseTo(2, 6);
    expect(s.points[b.p1].y).toBeCloseTo(s.points[b.p2].y, 7);
    expect(s.points[r.p1].x).toBeCloseTo(s.points[r.p2].x, 7);
  });
});
