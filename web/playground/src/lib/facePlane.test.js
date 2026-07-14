import { describe, expect, it } from 'vitest';
import {
  axisAngleFromBasis,
  createFaceRegionIndex,
  detectFacePlane,
  facePlaneBasis,
  makeTangentPlane,
} from './facePlane.js';

const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
const closeTo3 = (a, b, tol = 1e-9) =>
  Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]) < tol;

/** Rodrigues rotation of `p` around unit axis by `angle`. */
function rotatePoint([x, y, z], [ux, uy, uz], angle) {
  const cos = Math.cos(angle);
  const sin = Math.sin(angle);
  const d = ux * x + uy * y + uz * z;
  return [
    x * cos + (uy * z - uz * y) * sin + ux * d * (1 - cos),
    y * cos + (uz * x - ux * z) * sin + uy * d * (1 - cos),
    z * cos + (ux * y - uy * x) * sin + uz * d * (1 - cos),
  ];
}

/**
 * Subdivided parallelogram patch lying in the plane `origin + s*du + t*dv`
 * (s ∈ [0, nx], t ∈ [0, ny]) appended to a shared mesh pool. Triangle
 * winding follows du × dv. Coincident vertices are welded (by exact
 * coordinates) so patches share boundary vertices like a real tessellation.
 */
function gridMesh(origin, du, dv, nx, ny, pool = { positions: [], indices: [], ids: new Map() }) {
  const { positions, indices, ids } = pool;
  const vertexId = (i, j) => {
    const p = [
      origin[0] + i * du[0] + j * dv[0],
      origin[1] + i * du[1] + j * dv[1],
      origin[2] + i * du[2] + j * dv[2],
    ];
    const key = p.join(',');
    if (!ids.has(key)) {
      ids.set(key, positions.length / 3);
      positions.push(...p);
    }
    return ids.get(key);
  };
  for (let j = 0; j < ny; j += 1) {
    for (let i = 0; i < nx; i += 1) {
      const a = vertexId(i, j);
      const b = vertexId(i + 1, j);
      const c = vertexId(i, j + 1);
      const d = vertexId(i + 1, j + 1);
      indices.push(a, b, d, a, d, c);
    }
  }
  return pool;
}

/** Cylinder-ish strip: `steps` quads sweeping `arc` radians around the Y
 * axis at radius `r`, height 1. Adjacent quads share vertices. */
function curvedStrip(r, arc, steps) {
  const positions = [];
  const indices = [];
  for (let i = 0; i <= steps; i += 1) {
    const a = (arc * i) / steps;
    positions.push(r * Math.cos(a), 0, r * Math.sin(a));
    positions.push(r * Math.cos(a), 1, r * Math.sin(a));
  }
  for (let i = 0; i < steps; i += 1) {
    const a = 2 * i;
    indices.push(a, a + 2, a + 3, a, a + 3, a + 1);
  }
  return { positions, indices };
}

describe('facePlaneBasis', () => {
  it('reproduces the named-plane conventions for axis normals', () => {
    expect(facePlaneBasis([0, 0, 1])).toEqual({ u: [1, 0, 0], v: [0, 1, 0] }); // XY
    const top = facePlaneBasis([0, 1, 0]); // XZ
    expect(closeTo3(top.u, [1, 0, 0])).toBe(true);
    expect(closeTo3(top.v, [0, 0, -1])).toBe(true);
    const right = facePlaneBasis([1, 0, 0]); // YZ
    expect(closeTo3(right.u, [0, 0, -1])).toBe(true);
    expect(closeTo3(right.v, [0, 1, 0])).toBe(true);
  });

  it('builds a right-handed orthonormal frame for slanted normals', () => {
    const n = [1 / 3, 2 / 3, 2 / 3];
    const { u, v } = facePlaneBasis(n);
    expect(dot(u, v)).toBeCloseTo(0, 12);
    expect(dot(u, n)).toBeCloseTo(0, 12);
    expect(dot(v, n)).toBeCloseTo(0, 12);
    expect(closeTo3(cross(u, v), n)).toBe(true);
    // v is the in-plane direction closest to world up.
    expect(v[1]).toBeGreaterThan(0);
  });

  it('handles downward-facing planes and normalizes the input', () => {
    const { u, v } = facePlaneBasis([0, -2, 0]);
    expect(closeTo3(cross(u, v), [0, -1, 0])).toBe(true);
    expect(() => facePlaneBasis([0, 0, 0])).toThrow(/zero/);
  });
});

describe('axisAngleFromBasis', () => {
  it('returns null for the identity', () => {
    expect(axisAngleFromBasis([1, 0, 0], [0, 1, 0], [0, 0, 1])).toBeNull();
  });

  it('recovers simple rotations', () => {
    // 90° about Z: X -> Y, Y -> -X.
    const r = axisAngleFromBasis([0, 1, 0], [-1, 0, 0], [0, 0, 1]);
    expect(closeTo3(r.axis, [0, 0, 1])).toBe(true);
    expect(r.angle).toBeCloseTo(Math.PI / 2, 12);
    // 180° about Y (trace-negative branch).
    const half = axisAngleFromBasis([-1, 0, 0], [0, 1, 0], [0, 0, -1]);
    expect(closeTo3(half.axis, [0, 1, 0])).toBe(true);
    expect(half.angle).toBeCloseTo(Math.PI, 12);
  });

  it('round-trips arbitrary bases through Rodrigues rotation', () => {
    const n = [2 / 7, 3 / 7, 6 / 7];
    const { u, v } = facePlaneBasis(n);
    const cols = [u, n, [-v[0], -v[1], -v[2]]]; // the extrude post-rotation
    const { axis, angle } = axisAngleFromBasis(...cols);
    expect(Math.hypot(...axis)).toBeCloseTo(1, 12);
    const basis = [
      [1, 0, 0],
      [0, 1, 0],
      [0, 0, 1],
    ];
    basis.forEach((e, i) => {
      expect(closeTo3(rotatePoint(e, axis, angle), cols[i])).toBe(true);
    });
  });
});

describe('detectFacePlane', () => {
  // An L-shaped shell: a 4x4 top face at y = 1 (+Y out) meeting a 4x4 front
  // face at z = 1 (+Z out) along a welded 90° edge, like a real mesh.
  function lShell() {
    const pool = gridMesh([0, 1, 0], [0, 0, 0.25], [0.25, 0, 0], 4, 4);
    return gridMesh([0, 1, 1], [0, -0.25, 0], [0.25, 0, 0], 4, 4, pool);
  }

  it('detects a planar face: normal, centroid origin, extent, basis', () => {
    const { positions, indices } = lShell();
    const result = detectFacePlane(positions, indices, 0); // a top-face tri
    expect(result.planar).toBe(true);
    const { plane } = result;
    expect(closeTo3(plane.normal, [0, 1, 0])).toBe(true);
    // Origin at the centroid of the 1x1 top face starting at (0, 1, 0).
    expect(closeTo3(plane.origin, [0.5, 1, 0.5])).toBe(true);
    // Extent reaches the face corners.
    expect(plane.extent).toBeCloseTo(Math.SQRT1_2, 6);
    expect(closeTo3(cross(plane.u, plane.v), plane.normal)).toBe(true);
  });

  it('stops the region at sharp edges instead of leaking around them', () => {
    const { positions, indices } = lShell();
    // A front-face triangle (the second grid's first cell).
    const front = detectFacePlane(positions, indices, 32);
    expect(front.planar).toBe(true);
    expect(closeTo3(front.plane.normal, [0, 0, 1])).toBe(true);
    expect(closeTo3(front.plane.origin, [0.5, 0.5, 1])).toBe(true);
  });

  it('flags curved faces as sketchable with a local normal and extent', () => {
    const { positions, indices } = curvedStrip(2, Math.PI / 2, 32);
    const result = detectFacePlane(positions, indices, 10);
    expect(result.planar).toBe(false);
    expect(result.curved).toBe(true);
    expect(result.reason).toBe('face is curved');
    // The seed triangle's normal is radial on the cylinder (unit length).
    expect(Math.hypot(...result.normal)).toBeCloseTo(1, 6);
    expect(result.normal[1]).toBeCloseTo(0, 6); // no Y component
    expect(result.extent).toBeGreaterThan(0);
    expect(result.tris.length).toBeGreaterThan(3);
  });

  it('rejects regions too small to classify (a lone coarse facet)', () => {
    const { positions, indices } = curvedStrip(2, Math.PI / 2, 4);
    const result = detectFacePlane(positions, indices, 0);
    expect(result.planar).toBe(false);
    expect(result.reason).toMatch(/too small/);
  });

  it('rejects out-of-range and degenerate seeds', () => {
    const { positions, indices } = lShell();
    expect(detectFacePlane(positions, indices, -1).planar).toBe(false);
    expect(detectFacePlane(positions, indices, 9999).planar).toBe(false);
    const degenerate = {
      positions: [0, 0, 0, 0, 0, 0, 1, 1, 1],
      indices: [0, 1, 2],
    };
    expect(
      detectFacePlane(degenerate.positions, degenerate.indices, 0).planar
    ).toBe(false);
  });

  it('reports the region triangles for highlighting', () => {
    const { positions, indices } = lShell();
    const result = detectFacePlane(positions, indices, 0);
    // The 4x4 top face is 32 triangles and stops at the welded edge.
    expect(result.tris).toHaveLength(32);
    expect(result.tris).toContain(0);
    expect(result.tris).not.toContain(32);
  });
});

describe('makeTangentPlane', () => {
  it('builds a face-plane frame at the pick point with a right-handed basis', () => {
    const plane = makeTangentPlane([1, 2, 3], [0, 0, 5], 0.7);
    expect(closeTo3(plane.origin, [1, 2, 3])).toBe(true);
    // Normal is normalized.
    expect(closeTo3(plane.normal, [0, 0, 1])).toBe(true);
    expect(plane.extent).toBe(0.7);
    // u × v = normal, and both are unit and orthogonal to the normal.
    expect(closeTo3(cross(plane.u, plane.v), plane.normal)).toBe(true);
    expect(Math.hypot(...plane.u)).toBeCloseTo(1, 9);
    expect(Math.hypot(...plane.v)).toBeCloseTo(1, 9);
    expect(dot(plane.u, plane.normal)).toBeCloseTo(0, 9);
    expect(dot(plane.v, plane.normal)).toBeCloseTo(0, 9);
  });

  it('copies the origin (no aliasing of the caller array)', () => {
    const origin = [4, 5, 6];
    const plane = makeTangentPlane(origin, [1, 0, 0], 1);
    origin[0] = 99;
    expect(plane.origin[0]).toBe(4);
  });

  it('throws on a zero normal', () => {
    expect(() => makeTangentPlane([0, 0, 0], [0, 0, 0], 1)).toThrow(/normal is zero/);
  });
});

describe('createFaceRegionIndex', () => {
  function lShell() {
    const pool = gridMesh([0, 1, 0], [0, 0, 0.25], [0.25, 0, 0], 4, 4);
    return gridMesh([0, 1, 1], [0, -0.25, 0], [0.25, 0, 0], 4, 4, pool);
  }

  it('classifies faces like detectFacePlane', () => {
    const { positions, indices } = lShell();
    const index = createFaceRegionIndex(positions, indices);
    const region = index.regionAt(0);
    expect(region.planar).toBe(true);
    expect(closeTo3(region.plane.normal, [0, 1, 0])).toBe(true);
    expect(region.tris).toHaveLength(32);
  });

  it('returns the same cached region object for any seed inside it', () => {
    const { positions, indices } = lShell();
    const index = createFaceRegionIndex(positions, indices);
    const fromFirst = index.regionAt(0);
    const fromLast = index.regionAt(31);
    expect(fromLast).toBe(fromFirst);
    // A different face is a different region.
    expect(index.regionAt(32)).not.toBe(fromFirst);
  });

  it('caches curved regions as non-planar', () => {
    const { positions, indices } = curvedStrip(2, Math.PI / 2, 32);
    const index = createFaceRegionIndex(positions, indices);
    const region = index.regionAt(10);
    expect(region.planar).toBe(false);
    expect(index.regionAt(11)).toBe(region);
  });

  it('rejects out-of-range seeds with an empty region', () => {
    const { positions, indices } = lShell();
    const index = createFaceRegionIndex(positions, indices);
    expect(index.regionAt(-1)).toEqual({
      planar: false,
      reason: 'no triangle under the cursor',
      tris: [],
    });
    expect(index.regionAt(null).planar).toBe(false);
  });
});
