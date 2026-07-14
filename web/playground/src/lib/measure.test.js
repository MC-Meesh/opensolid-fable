import { describe, expect, it } from 'vitest';
import {
  boundingBoxDims,
  entityPoint,
  measurePair,
  measureSingle,
  triListArea,
} from './measure.js';

const close = (a, b, tol = 1e-6) => Math.abs(a - b) < tol;

describe('boundingBoxDims', () => {
  it('returns null for an empty array', () => {
    expect(boundingBoxDims(new Float32Array(0))).toBeNull();
  });

  it('measures a box from its vertices', () => {
    const positions = new Float32Array([
      -1, 0, 2, 3, 4, 2, -1, 4, -1, 3, 0, -1,
    ]);
    const b = boundingBoxDims(positions);
    expect(b.min).toEqual([-1, 0, -1]);
    expect(b.max).toEqual([3, 4, 2]);
    expect(b.size).toEqual([4, 4, 3]);
    expect(close(b.diagonal, Math.hypot(4, 4, 3))).toBe(true);
  });
});

describe('triListArea', () => {
  it('sums the areas of the listed triangles', () => {
    // Two unit right triangles forming a 1x1 square in z=0.
    const positions = new Float32Array([0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0]);
    const indices = new Uint32Array([0, 1, 2, 0, 2, 3]);
    expect(close(triListArea(positions, indices, [0, 1]), 1)).toBe(true);
    expect(close(triListArea(positions, indices, [0]), 0.5)).toBe(true);
  });
});

describe('measureSingle', () => {
  it('reads a vertex coordinate', () => {
    expect(measureSingle({ kind: 'vertex', point: [1, 2, 3] })).toEqual({
      kind: 'vertex',
      coord: [1, 2, 3],
    });
  });

  it('reads an edge length', () => {
    const r = measureSingle({ kind: 'edge', length: 4.2, a: [0, 0, 0], b: [4.2, 0, 0] });
    expect(r.kind).toBe('edge');
    expect(close(r.length, 4.2)).toBe(true);
  });

  it('reads a circle radius and diameter', () => {
    const r = measureSingle({ kind: 'circle', radius: 3, center: [0, 0, 0], normal: [0, 0, 1] });
    expect(close(r.radius, 3)).toBe(true);
    expect(close(r.diameter, 6)).toBe(true);
  });

  it('reads a face area', () => {
    const r = measureSingle({ kind: 'face', point: [1, 1, 0], area: 12, plane: {} });
    expect(close(r.area, 12)).toBe(true);
  });
});

describe('entityPoint', () => {
  it('falls back to center / origin when no snapped point', () => {
    expect(entityPoint({ kind: 'circle', center: [1, 2, 3] })).toEqual([1, 2, 3]);
    expect(entityPoint({ kind: 'face', plane: { origin: [4, 5, 6] } })).toEqual([4, 5, 6]);
  });
});

describe('measurePair', () => {
  it('reports distance and per-axis deltas between two points', () => {
    const r = measurePair(
      { kind: 'vertex', point: [0, 0, 0] },
      { kind: 'vertex', point: [3, 4, 0] }
    );
    expect(close(r.distance, 5)).toBe(true);
    expect(r.delta).toEqual([3, 4, 0]);
  });

  it('reports the angle and parallel gap between two faces', () => {
    const a = { kind: 'face', point: [0, 0, 0], plane: { origin: [0, 0, 0], normal: [0, 0, 1] } };
    const b = { kind: 'face', point: [0, 0, 5], plane: { origin: [0, 0, 5], normal: [0, 0, 1] } };
    const r = measurePair(a, b);
    expect(close(r.angle, 0, 1e-6)).toBe(true);
    expect(close(r.gap, 5)).toBe(true);
  });

  it('reports a 90-degree angle between perpendicular faces', () => {
    const a = { kind: 'face', point: [0, 0, 0], plane: { origin: [0, 0, 0], normal: [0, 0, 1] } };
    const b = { kind: 'face', point: [0, 0, 0], plane: { origin: [0, 0, 0], normal: [1, 0, 0] } };
    const r = measurePair(a, b);
    expect(close(r.angle, 90, 1e-6)).toBe(true);
    expect(r.gap).toBeUndefined();
  });

  it('reports the angle between two edges', () => {
    const a = { kind: 'edge', point: [0, 0, 0], dir: [1, 0, 0], a: [0, 0, 0], b: [1, 0, 0] };
    const b = { kind: 'edge', point: [0, 0, 0], dir: [0, 1, 0], a: [0, 0, 0], b: [0, 1, 0] };
    const r = measurePair(a, b);
    expect(close(r.angle, 90, 1e-6)).toBe(true);
  });

  it('reports vertex-to-face perpendicular distance', () => {
    const face = { kind: 'face', point: [0, 0, 0], plane: { origin: [0, 0, 0], normal: [0, 0, 1] } };
    const vert = { kind: 'vertex', point: [3, 7, 4] };
    const r = measurePair(vert, face);
    expect(close(r.planeDistance, 4)).toBe(true);
  });
});
