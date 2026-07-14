import { describe, expect, it } from 'vitest';
import {
  regionBoundaryVertexLoops,
  projectLoopToPlane,
  faceBoundaryLoops,
} from './faceBoundary.js';

// A unit square in the XY plane, two triangles sharing the 0-2 diagonal.
const SQUARE_POS = [
  0, 0, 0, // 0
  1, 0, 0, // 1
  1, 1, 0, // 2
  0, 1, 0, // 3
];
const SQUARE_IDX = [0, 1, 2, 0, 2, 3];
const XY_PLANE = { origin: [0, 0, 0], u: [1, 0, 0], v: [0, 1, 0] };

describe('regionBoundaryVertexLoops', () => {
  it('extracts the outline of a two-triangle square, dropping the diagonal', () => {
    const loops = regionBoundaryVertexLoops(SQUARE_IDX, [0, 1]);
    expect(loops).toHaveLength(1);
    const loop = loops[0];
    // Closed loop: first === last, four distinct corners.
    expect(loop[0]).toBe(loop[loop.length - 1]);
    expect(new Set(loop).size).toBe(4);
    expect(loop).toHaveLength(5);
  });

  it('walks a single triangle as its own boundary', () => {
    const loops = regionBoundaryVertexLoops([0, 1, 2], [0]);
    expect(loops).toHaveLength(1);
    expect(new Set(loops[0]).size).toBe(3);
  });

  it('returns nothing for an empty region', () => {
    expect(regionBoundaryVertexLoops(SQUARE_IDX, [])).toEqual([]);
  });

  it('finds the outer boundary of a larger fan region', () => {
    // Four triangles around a central strip; outer boundary is one loop.
    const positions = [
      0, 0, 0, 1, 0, 0, 2, 0, 0, 2, 1, 0, 1, 1, 0, 0, 1, 0,
    ];
    const indices = [0, 1, 5, 1, 4, 5, 1, 2, 4, 2, 3, 4];
    const loops = regionBoundaryVertexLoops(indices, [0, 1, 2, 3]);
    expect(loops).toHaveLength(1);
    expect(new Set(loops[0]).size).toBe(6);
  });
});

describe('projectLoopToPlane', () => {
  it('maps XY vertices straight through the XY basis', () => {
    const uv = projectLoopToPlane(SQUARE_POS, [0, 1, 2, 3, 0], XY_PLANE);
    expect(uv).toEqual([
      [0, 0],
      [1, 0],
      [1, 1],
      [0, 1],
      [0, 0],
    ]);
  });

  it('honors a shifted origin and a rotated basis', () => {
    const plane = { origin: [1, 0, 0], u: [0, 1, 0], v: [0, 0, 1] };
    const uv = projectLoopToPlane([1, 2, 3], [0], plane);
    expect(uv[0]).toEqual([2, 3]);
  });
});

describe('faceBoundaryLoops', () => {
  it('produces 2D outline polylines end to end', () => {
    const loops = faceBoundaryLoops(SQUARE_POS, SQUARE_IDX, [0, 1], XY_PLANE);
    expect(loops).toHaveLength(1);
    expect(loops[0][0]).toEqual([0, 0]);
    expect(loops[0]).toHaveLength(5);
  });
});
