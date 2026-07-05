import { describe, expect, it } from 'vitest';
import {
  addArc,
  addCircle,
  addLine,
  addPoint,
  createSketch,
} from './model.js';
import {
  axisAlign,
  distToEntity,
  hitTest,
  nearestPoint,
  snapToGrid,
} from './snap.js';

describe('snap', () => {
  it('snapToGrid rounds to the nearest intersection', () => {
    expect(snapToGrid(1.3, -0.6, 0.5)).toEqual({ x: 1.5, y: -0.5 });
    expect(snapToGrid(0.24, 0.26, 0.5)).toEqual({ x: 0, y: 0.5 });
  });

  it('nearestPoint finds the closest point within range', () => {
    const s = createSketch();
    addPoint(s, 0, 0);
    const b = addPoint(s, 1, 1);
    const hit = nearestPoint(s, 1.1, 0.9, 0.5);
    expect(hit.id).toBe(b);
    expect(nearestPoint(s, 5, 5, 0.5)).toBeNull();
  });

  it('nearestPoint honors exclusions', () => {
    const s = createSketch();
    const a = addPoint(s, 0, 0);
    expect(nearestPoint(s, 0.1, 0, 1, new Set([a]))).toBeNull();
  });

  it('axisAlign snaps near-horizontal and near-vertical segments', () => {
    expect(axisAlign(0, 0, 10, 0.3)).toMatchObject({ y: 0, axis: 'h' });
    expect(axisAlign(0, 0, 0.3, 10)).toMatchObject({ x: 0, axis: 'v' });
    expect(axisAlign(0, 0, 5, 5).axis).toBeNull();
    expect(axisAlign(0, 0, 0, 0).axis).toBeNull(); // zero-length
  });

  it('distToEntity measures line, circle, and arc outlines', () => {
    const s = createSketch();
    const a = addPoint(s, 0, 0);
    const b = addPoint(s, 2, 0);
    const line = addLine(s, a, b);
    const c = addPoint(s, 10, 0);
    const circle = addCircle(s, c, 1);
    const ac = addPoint(s, -10, 0);
    const s1 = addPoint(s, -9, 0);
    const s2 = addPoint(s, -10, 1);
    const arc = addArc(s, ac, s1, s2, true);
    expect(distToEntity(s, s.entities[line], 1, 0.5)).toBeCloseTo(0.5, 12);
    expect(distToEntity(s, s.entities[circle], 13, 0)).toBeCloseTo(2, 12);
    const diag = Math.SQRT1_2;
    expect(
      distToEntity(s, s.entities[arc], -10 + 2 * diag, 2 * diag)
    ).toBeCloseTo(1, 9);
  });

  it('hitTest prefers points over entities', () => {
    const s = createSketch();
    const a = addPoint(s, 0, 0);
    const b = addPoint(s, 2, 0);
    const line = addLine(s, a, b);
    expect(hitTest(s, 0.05, 0.05, 0.2)).toEqual({ kind: 'point', id: a });
    expect(hitTest(s, 1, 0.1, 0.2)).toEqual({ kind: 'entity', id: line });
    expect(hitTest(s, 1, 5, 0.2)).toBeNull();
  });
});
