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

  const lineAngle = (s, id) => {
    const l = s.entities[id];
    return Math.atan2(
      s.points[l.p2].y - s.points[l.p1].y,
      s.points[l.p2].x - s.points[l.p1].x
    );
  };
  const lineLen = (s, id) => {
    const l = s.entities[id];
    return Math.hypot(
      s.points[l.p2].x - s.points[l.p1].x,
      s.points[l.p2].y - s.points[l.p1].y
    );
  };
  // Deviation of two lines' directions from parallel, in (0, π/2].
  const parallelGap = (s, a, b) => {
    let d = Math.abs(lineAngle(s, a) - lineAngle(s, b)) % Math.PI;
    return Math.min(d, Math.PI - d);
  };

  it('parallel aligns two line directions', () => {
    const s = createSketch();
    const l1 = addLine(s, addPoint(s, 0, 0), addPoint(s, 4, 0));
    const l2 = addLine(s, addPoint(s, 0, 2), addPoint(s, 3, 4));
    addConstraint(s, { type: 'parallel', a: l1, b: l2 });
    const result = solve(s);
    expect(result.converged).toBe(true);
    expect(parallelGap(s, l1, l2)).toBeCloseTo(0, 7);
  });

  it('parallel rotates only the free line when the other is pinned', () => {
    const s = createSketch();
    const a = addPoint(s, 0, 0);
    const b = addPoint(s, 4, 0);
    const l1 = addLine(s, a, b);
    const l2 = addLine(s, addPoint(s, 0, 2), addPoint(s, 3, 5));
    addConstraint(s, { type: 'parallel', a: l1, b: l2 });
    solve(s, { pinned: new Set([a, b]) });
    // Pinned horizontal reference is untouched; l2 becomes horizontal too.
    expect(lineAngle(s, l1)).toBeCloseTo(0, 9);
    expect(parallelGap(s, l1, l2)).toBeCloseTo(0, 7);
  });

  it('perpendicular makes two lines meet at a right angle', () => {
    const s = createSketch();
    const l1 = addLine(s, addPoint(s, 0, 0), addPoint(s, 4, 0));
    const l2 = addLine(s, addPoint(s, 1, 1), addPoint(s, 4, 2));
    addConstraint(s, { type: 'perpendicular', a: l1, b: l2 });
    const result = solve(s);
    expect(result.converged).toBe(true);
    let gap = Math.abs(lineAngle(s, l1) - lineAngle(s, l2)) % Math.PI;
    gap = Math.min(gap, Math.PI - gap);
    expect(gap).toBeCloseTo(Math.PI / 2, 7);
  });

  it('equal drives two lines to the same length', () => {
    const s = createSketch();
    const l1 = addLine(s, addPoint(s, 0, 0), addPoint(s, 6, 0));
    const l2 = addLine(s, addPoint(s, 0, 5), addPoint(s, 0, 7));
    addConstraint(s, { type: 'equal', a: l1, b: l2 });
    const result = solve(s);
    expect(result.converged).toBe(true);
    expect(lineLen(s, l1)).toBeCloseTo(lineLen(s, l2), 7);
    expect(lineLen(s, l1)).toBeCloseTo(4, 7); // met in the middle (6 and 2)
  });

  it('equal matches the free line to a fully pinned one', () => {
    const s = createSketch();
    const a = addPoint(s, 0, 0);
    const b = addPoint(s, 6, 0);
    const l1 = addLine(s, a, b);
    const l2 = addLine(s, addPoint(s, 0, 5), addPoint(s, 0, 7));
    addConstraint(s, { type: 'equal', a: l1, b: l2 });
    solve(s, { pinned: new Set([a, b]) });
    expect(lineLen(s, l1)).toBeCloseTo(6, 9);
    expect(lineLen(s, l2)).toBeCloseTo(6, 7);
  });

  it('equal drives two circles to the same radius', () => {
    const s = createSketch();
    const c1 = addCircle(s, addPoint(s, 0, 0), 2);
    const c2 = addCircle(s, addPoint(s, 10, 0), 8);
    addConstraint(s, { type: 'equal', a: c1, b: c2 });
    solve(s);
    expect(s.entities[c1].radius).toBeCloseTo(5, 9);
    expect(s.entities[c2].radius).toBeCloseTo(5, 9);
  });

  it('concentric brings two circle centers together', () => {
    const s = createSketch();
    const c1 = addCircle(s, addPoint(s, 0, 0), 2);
    const c2 = addCircle(s, addPoint(s, 4, 3), 5);
    addConstraint(s, { type: 'concentric', a: c1, b: c2 });
    const result = solve(s);
    expect(result.converged).toBe(true);
    const center1 = s.points[s.entities[c1].center];
    const center2 = s.points[s.entities[c2].center];
    expect(center1.x).toBeCloseTo(center2.x, 7);
    expect(center1.y).toBeCloseTo(center2.y, 7);
  });

  it('midpoint moves a point to the center of a line', () => {
    const s = createSketch();
    const a = addPoint(s, 0, 0);
    const b = addPoint(s, 8, 4);
    const line = addLine(s, a, b);
    const p = addPoint(s, 1, 1);
    addConstraint(s, { type: 'midpoint', point: p, line });
    const result = solve(s);
    expect(result.converged).toBe(true);
    expect(s.points[p].x).toBeCloseTo(4, 7);
    expect(s.points[p].y).toBeCloseTo(2, 7);
  });

  it('collinear places two lines on one infinite line', () => {
    const s = createSketch();
    const l1 = addLine(s, addPoint(s, 0, 0), addPoint(s, 4, 0));
    const l2 = addLine(s, addPoint(s, 6, 1), addPoint(s, 10, 2));
    addConstraint(s, { type: 'collinear', a: l1, b: l2 });
    const result = solve(s);
    expect(result.converged).toBe(true);
    expect(parallelGap(s, l1, l2)).toBeCloseTo(0, 6);
    // Endpoints of l2 lie on l1's infinite line (through its two points).
    const a = s.points[s.entities[l1].p1];
    const b = s.points[s.entities[l1].p2];
    let nx = -(b.y - a.y);
    let ny = b.x - a.x;
    const nl = Math.hypot(nx, ny);
    nx /= nl;
    ny /= nl;
    for (const pid of [s.entities[l2].p1, s.entities[l2].p2]) {
      const q = s.points[pid];
      const off = (q.x - a.x) * nx + (q.y - a.y) * ny;
      expect(off).toBeCloseTo(0, 6);
    }
  });

  it('symmetric mirrors two points across an axis line', () => {
    const s = createSketch();
    // Vertical axis along x = 0.
    const axis = addLine(s, addPoint(s, 0, -1), addPoint(s, 0, 1));
    const a = addPoint(s, 2, 3);
    const b = addPoint(s, -5, -1);
    addLine(s, a, b);
    addConstraint(s, { type: 'symmetric', a, b, line: axis });
    const result = solve(s);
    expect(result.converged).toBe(true);
    // Reflection over x = 0: x flips, y matches.
    expect(s.points[a].x).toBeCloseTo(-s.points[b].x, 7);
    expect(s.points[a].y).toBeCloseTo(s.points[b].y, 7);
  });

  it('fix anchors a point so constraints move the other end', () => {
    const s = createSketch();
    const a = addPoint(s, 1, 2);
    const b = addPoint(s, 5, 2);
    const line = addLine(s, a, b);
    addConstraint(s, { type: 'fix', point: a });
    addConstraint(s, { type: 'length', line, value: 10 });
    const result = solve(s);
    expect(result.converged).toBe(true);
    // The fixed endpoint stays put; the free one absorbs the length change.
    expect(s.points[a].x).toBeCloseTo(1, 9);
    expect(s.points[a].y).toBeCloseTo(2, 9);
    expect(lineLen(s, line)).toBeCloseTo(10, 7);
  });

  it('distance drives two points to an aligned separation', () => {
    const s = createSketch();
    const a = addPoint(s, 0, 0);
    const b = addPoint(s, 3, 4);
    addConstraint(s, { type: 'distance', a, b, value: 10 });
    const result = solve(s);
    expect(result.converged).toBe(true);
    expect(
      Math.hypot(s.points[b].x - s.points[a].x, s.points[b].y - s.points[a].y)
    ).toBeCloseTo(10, 7);
    // Direction preserved (3-4-5 scaled to 6-8-10).
    expect(s.points[b].x - s.points[a].x).toBeCloseTo(6, 6);
  });

  it('distance with horizontal orient constrains only Δx', () => {
    const s = createSketch();
    const a = addPoint(s, 0, 0);
    const b = addPoint(s, 2, 5);
    addConstraint(s, { type: 'distance', a, b, value: 8, orient: 'horizontal' });
    const result = solve(s);
    expect(result.converged).toBe(true);
    expect(Math.abs(s.points[b].x - s.points[a].x)).toBeCloseTo(8, 7);
    // Vertical offset is untouched.
    expect(s.points[b].y - s.points[a].y).toBeCloseTo(5, 9);
  });

  it('distance with vertical orient constrains only Δy against a pin', () => {
    const s = createSketch();
    const a = addPoint(s, 0, 0);
    const b = addPoint(s, 2, 5);
    addConstraint(s, { type: 'distance', a, b, value: 3, orient: 'vertical' });
    solve(s, { pinned: new Set([a]) });
    expect(s.points[a].y).toBe(0);
    expect(s.points[b].y).toBeCloseTo(3, 7);
    expect(s.points[b].x).toBe(2); // x never touched by a vertical distance
  });

  it('pdistance drives a point to a perpendicular offset from a line', () => {
    const s = createSketch();
    const a = addPoint(s, -5, 0);
    const b = addPoint(s, 5, 0);
    const line = addLine(s, a, b);
    const p = addPoint(s, 0, 1);
    addConstraint(s, { type: 'pdistance', point: p, line, value: 4 });
    const result = solve(s, { pinned: new Set([a, b]) });
    expect(result.converged).toBe(true);
    // Line is on y = 0; the point sits 4 above it.
    expect(s.points[p].y).toBeCloseTo(4, 7);
  });

  it('angle drives two lines to a target angle', () => {
    const s = createSketch();
    const a = addPoint(s, 0, 0);
    const b = addPoint(s, 4, 0);
    const l1 = addLine(s, a, b);
    const l2 = addLine(s, addPoint(s, 0, 0), addPoint(s, 4, 0.5));
    addConstraint(s, { type: 'angle', a: l1, b: l2, value: Math.PI / 4 });
    const result = solve(s, { pinned: new Set([a, b]) });
    expect(result.converged).toBe(true);
    const ang1 = lineAngle(s, l1);
    const ang2 = lineAngle(s, l2);
    let gap = Math.abs(ang1 - ang2) % Math.PI;
    gap = Math.min(gap, Math.PI - gap);
    expect(gap).toBeCloseTo(Math.PI / 4, 6);
  });

  it('diameter drives a circle radius to half the value', () => {
    const s = createSketch();
    const circle = addCircle(s, addPoint(s, 0, 0), 1);
    addConstraint(s, { type: 'diameter', entity: circle, value: 7 });
    solve(s);
    expect(s.entities[circle].radius).toBeCloseTo(3.5, 9);
  });

  it('diameter moves arc endpoints onto the target circle', () => {
    const s = createSketch();
    const c = addPoint(s, 0, 0);
    const p1 = addPoint(s, 2, 0);
    const p2 = addPoint(s, 0, 3);
    const arc = addArc(s, c, p1, p2, true);
    addConstraint(s, { type: 'diameter', entity: arc, value: 10 });
    const result = solve(s);
    expect(result.converged).toBe(true);
    expect(Math.hypot(s.points[p1].x, s.points[p1].y)).toBeCloseTo(5, 7);
  });

  it('driven dimensions are measured, not enforced', () => {
    const s = createSketch();
    const a = addPoint(s, 0, 0);
    const b = addPoint(s, 3, 0);
    addConstraint(s, { type: 'distance', a, b, value: 100, driven: true });
    const result = solve(s);
    expect(result.converged).toBe(true);
    // The driven target is ignored — geometry stays put.
    expect(s.points[b].x).toBe(3);
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
