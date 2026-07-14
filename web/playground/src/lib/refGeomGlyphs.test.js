import { describe, expect, it } from 'vitest';
import {
  axisEndpoints,
  axisPositions,
  csysSegments,
  CSYS_AXIS_COLORS,
  planeQuadCorners,
} from './refGeomGlyphs.js';
import {
  axisFromPointDirection,
  axisFromTwoPoints,
  csysFromPlane,
  offsetPlane,
} from './referenceGeometry.js';

const close = (a, b, tol = 1e-9) => {
  for (let i = 0; i < a.length; i++) expect(Math.abs(a[i] - b[i])).toBeLessThan(tol);
};

describe('axisEndpoints', () => {
  it('draws a two-point axis as its actual segment', () => {
    const a = axisFromTwoPoints([0, 0, 0], [0, 0, 4]);
    const [p0, p1] = axisEndpoints(a);
    close(p0, [0, 0, 0]);
    close(p1, [0, 0, 4]);
  });

  it('centers a point-direction axis on its origin', () => {
    const a = axisFromPointDirection([1, 0, 0], [0, 1, 0]);
    const [p0, p1] = axisEndpoints(a, 6);
    close(p0, [1, -3, 0]);
    close(p1, [1, 3, 0]);
  });

  it('falls back to span when a two-point length is degenerate', () => {
    const a = { method: 'two-points', origin: [0, 0, 0], direction: [1, 0, 0], length: 0 };
    const [, p1] = axisEndpoints(a, 5);
    close(p1, [5, 0, 0]);
  });
});

describe('axisPositions', () => {
  it('flattens the endpoints into a 6-float buffer', () => {
    const buf = axisPositions(axisFromTwoPoints([0, 0, 0], [1, 0, 0]));
    expect(buf).toBeInstanceOf(Float32Array);
    expect(Array.from(buf)).toEqual([0, 0, 0, 1, 0, 0]);
  });
});

describe('csysSegments', () => {
  it('emits three axis segments from the origin with triad colors', () => {
    const segs = csysSegments(csysFromPlane('XY'), 2);
    expect(segs.map((s) => s.key)).toEqual(['x', 'y', 'z']);
    expect(segs[0].color).toBe(CSYS_AXIS_COLORS.x);
    close(segs[0].from, [0, 0, 0]);
    close(segs[0].to, [2, 0, 0]);
    close(segs[2].to, [0, 0, 2]); // Z axis
  });
});

describe('planeQuadCorners', () => {
  it('returns four coplanar corners centered on the origin', () => {
    const p = offsetPlane('XY', 3); // origin (0,0,3), u=+X, v=+Y
    const c = planeQuadCorners(p, 1);
    close(c[0], [-1, -1, 3]);
    close(c[1], [1, -1, 3]);
    close(c[2], [1, 1, 3]);
    close(c[3], [-1, 1, 3]);
    // all share the plane's z
    for (const corner of c) expect(corner[2]).toBeCloseTo(3);
  });

  it('defaults the half-side to the entity extent', () => {
    const p = offsetPlane('XY', 0); // extent defaults to DEFAULT_PLANE_EXTENT (5)
    const c = planeQuadCorners(p);
    close(c[2], [p.extent, p.extent, 0]);
  });
});
