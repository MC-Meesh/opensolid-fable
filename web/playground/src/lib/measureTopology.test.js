import { describe, expect, it } from 'vitest';
import { buildEdgeModel, fitCircle, snapEntity } from './measureTopology.js';

const close = (a, b, tol = 1e-6) => Math.abs(a - b) < tol;
const close3 = (a, b, tol = 1e-6) =>
  Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]) < tol;

/** Unit cube [0,1]^3 as 12 triangles, quads wound consistently per face. */
function box() {
  const positions = new Float32Array([
    0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, // z=0
    0, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, // z=1
  ]);
  const quads = [
    [0, 1, 2, 3],
    [4, 5, 6, 7],
    [0, 1, 5, 4],
    [1, 2, 6, 5],
    [2, 3, 7, 6],
    [3, 0, 4, 7],
  ];
  const indices = [];
  for (const [a, b, c, d] of quads) indices.push(a, b, c, a, c, d);
  return { positions, indices: new Uint32Array(indices) };
}

/** Flat disk (triangle fan) of radius `r` in the z=0 plane, `n` rim segments. */
function disk(r = 2, n = 24) {
  const positions = [0, 0, 0];
  for (let i = 0; i < n; i += 1) {
    const a = (2 * Math.PI * i) / n;
    positions.push(r * Math.cos(a), r * Math.sin(a), 0);
  }
  const indices = [];
  for (let i = 0; i < n; i += 1) {
    const rim = 1 + i;
    const next = 1 + ((i + 1) % n);
    indices.push(0, rim, next);
  }
  return { positions: new Float32Array(positions), indices: new Uint32Array(indices) };
}

describe('fitCircle', () => {
  it('recovers center, radius and normal of a coplanar ring', () => {
    const pts = [];
    for (let i = 0; i < 16; i += 1) {
      const a = (2 * Math.PI * i) / 16;
      pts.push([3 + 5 * Math.cos(a), 3 + 5 * Math.sin(a), 7]);
    }
    const c = fitCircle(pts);
    expect(c).not.toBeNull();
    expect(close(c.radius, 5, 1e-6)).toBe(true);
    expect(close3(c.center, [3, 3, 7], 1e-6)).toBe(true);
    expect(close(Math.abs(c.normal[2]), 1, 1e-9)).toBe(true);
  });

  it('rejects points that do not lie on a circle', () => {
    expect(fitCircle([[0, 0, 0], [1, 0, 0], [2, 0, 0], [3, 0, 0]])).toBeNull();
  });
});

describe('buildEdgeModel', () => {
  it('returns empty model for an empty mesh', () => {
    const m = buildEdgeModel(new Float32Array(0), new Uint32Array(0));
    expect(m).toEqual({ vertices: [], edges: [], circles: [] });
  });

  it('recovers a cube as 8 corner vertices and 12 unit edges', () => {
    const { positions, indices } = box();
    const m = buildEdgeModel(positions, indices);
    expect(m.vertices).toHaveLength(8);
    expect(m.edges).toHaveLength(12);
    expect(m.circles).toHaveLength(0);
    for (const e of m.edges) expect(close(e.length, 1, 1e-6)).toBe(true);
    // Every corner of the cube is present.
    for (const corner of [[0, 0, 0], [1, 1, 1], [1, 0, 1]]) {
      expect(m.vertices.some((v) => close3(v.point, corner))).toBe(true);
    }
  });

  it('fits a flat disk boundary to a circle (no stray straight edges)', () => {
    const { positions, indices } = disk(2, 24);
    const m = buildEdgeModel(positions, indices);
    expect(m.circles).toHaveLength(1);
    expect(close(m.circles[0].radius, 2, 0.02)).toBe(true);
    expect(close3(m.circles[0].center, [0, 0, 0], 1e-6)).toBe(true);
    expect(m.edges).toHaveLength(0);
    expect(m.vertices).toHaveLength(0);
  });
});

describe('snapEntity', () => {
  const model = buildEdgeModel(box().positions, box().indices);

  it('prefers a corner vertex when the hit is near one', () => {
    const e = snapEntity(model, [0.02, 0.02, 0.02], 0.2);
    expect(e.kind).toBe('vertex');
    expect(close3(e.point, [0, 0, 0], 1e-6)).toBe(true);
  });

  it('snaps to an edge when between corners', () => {
    const e = snapEntity(model, [0.5, 0.03, 0.0], 0.2);
    expect(e.kind).toBe('edge');
    expect(close(e.length, 1, 1e-6)).toBe(true);
    // Closest point lies on the y=0,z=0 edge near x=0.5.
    expect(close(e.point[0], 0.5, 0.05)).toBe(true);
  });

  it('returns null when nothing is within tolerance', () => {
    expect(snapEntity(model, [5, 5, 5], 0.2)).toBeNull();
  });

  it('snaps to a circular rim on a disk', () => {
    const dm = buildEdgeModel(disk(2, 24).positions, disk(2, 24).indices);
    const e = snapEntity(dm, [2.0, 0.05, 0], 0.3);
    expect(e.kind).toBe('circle');
    expect(close(e.radius, 2, 0.02)).toBe(true);
  });
});
