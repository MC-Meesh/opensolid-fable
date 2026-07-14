import { describe, it, expect } from 'vitest';
import { collectCreaseEdges, createEdgePickIndex } from './edgePick.js';

// --- test meshes -----------------------------------------------------------

// Unit cube [-1,1]^3, 8 vertices, 12 triangles (2 per face, consistent
// outward winding). Its 12 edges are 90° creases; the 6 face diagonals are
// flat. No open boundaries.
function cube() {
  const positions = [
    -1, -1, -1, // 0
    1, -1, -1, //  1
    1, 1, -1, //   2
    -1, 1, -1, //  3
    -1, -1, 1, //  4
    1, -1, 1, //   5
    1, 1, 1, //    6
    -1, 1, 1, //   7
  ];
  const indices = [
    0, 3, 2, 0, 2, 1, // -Z
    4, 5, 6, 4, 6, 7, // +Z
    0, 1, 5, 0, 5, 4, // -Y
    3, 7, 6, 3, 6, 2, // +Y
    0, 4, 7, 0, 7, 3, // -X
    1, 2, 6, 1, 6, 5, // +X
  ];
  return { positions, indices };
}

// Tent with a ridge along X subdivided at the middle: ridge vertices
// R0(-1,0,1) R1(0,0,1) R2(1,0,1); left skirt drops to y=-1,z=0 (plane z-y=1),
// right skirt to y=1,z=0 (plane z+y=1). The ridge is a sharp crease; its
// middle vertex R1 has exactly two crease edges (degree 2), so a pick should
// chain R0–R1–R2 through it. R0/R2 are degree-3 junctions (ridge + two open
// boundary edges), so the chain stops there.
function tent() {
  const V = {
    R0: [-1, 0, 1],
    R1: [0, 0, 1],
    R2: [1, 0, 1],
    L0: [-1, -1, 0],
    L1: [0, -1, 0],
    L2: [1, -1, 0],
    G0: [-1, 1, 0],
    G1: [0, 1, 0],
    G2: [1, 1, 0],
  };
  const order = ['R0', 'R1', 'R2', 'L0', 'L1', 'L2', 'G0', 'G1', 'G2'];
  const id = Object.fromEntries(order.map((k, i) => [k, i]));
  const positions = order.flatMap((k) => V[k]);
  const tri = (a, b, c) => [id[a], id[b], id[c]];
  const indices = [
    // left face (plane z - y = 1)
    ...tri('R0', 'L0', 'L1'), ...tri('R0', 'L1', 'R1'),
    ...tri('R1', 'L1', 'L2'), ...tri('R1', 'L2', 'R2'),
    // right face (plane z + y = 1)
    ...tri('R0', 'G1', 'G0'), ...tri('R0', 'R1', 'G1'),
    ...tri('R1', 'G2', 'G1'), ...tri('R1', 'R2', 'G2'),
  ];
  return { positions, indices, id, V };
}

// --- collectCreaseEdges ----------------------------------------------------

describe('collectCreaseEdges', () => {
  it('keeps the 12 sharp edges of a cube and drops the flat face diagonals', () => {
    const { positions, indices } = cube();
    const { edges } = collectCreaseEdges(positions, indices);
    expect(edges.length).toBe(12);
    // Every kept edge connects two cube vertices differing in exactly one axis
    // by 2 (a real cube edge, never a face diagonal or body diagonal).
    for (const { a, b } of edges) {
      const pa = [positions[3 * a], positions[3 * a + 1], positions[3 * a + 2]];
      const pb = [positions[3 * b], positions[3 * b + 1], positions[3 * b + 2]];
      const diff = pa.map((c, k) => Math.abs(c - pb[k]));
      const changed = diff.filter((d) => d > 1e-9);
      expect(changed).toEqual([2]);
    }
  });

  it('excludes a flat interior edge shared by two coplanar triangles', () => {
    // Two coplanar triangles sharing edge 1–2; the shared edge is flat, the
    // four outer edges are open boundaries (single triangle) and kept.
    const positions = [0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0];
    const indices = [0, 1, 2, 1, 3, 2];
    const { edges } = collectCreaseEdges(positions, indices);
    const key = (a, b) => (a < b ? `${a}-${b}` : `${b}-${a}`);
    const kept = new Set(edges.map((e) => key(e.a, e.b)));
    expect(kept.has(key(1, 2))).toBe(false); // flat diagonal excluded
    expect(edges.length).toBe(4); // the four boundary edges
  });

  it('builds vertex→edge adjacency covering every crease vertex', () => {
    const { positions, indices } = cube();
    const { edges, vertexEdges } = collectCreaseEdges(positions, indices);
    // Each cube corner touches exactly 3 cube edges.
    for (const [, list] of vertexEdges) expect(list.length).toBe(3);
    let incidences = 0;
    for (const [, list] of vertexEdges) incidences += list.length;
    expect(incidences).toBe(2 * edges.length);
  });
});

// --- createEdgePickIndex ---------------------------------------------------

describe('createEdgePickIndex', () => {
  it('returns null when the mesh has no crease edges', () => {
    const index = createEdgePickIndex([], []);
    expect(index.pickAt([0, 0, 0])).toBe(null);
  });

  it('picks the nearest cube edge as a single-segment polyline', () => {
    const { positions, indices } = cube();
    const index = createEdgePickIndex(positions, indices);
    // Just outside the +X/+Z edge (the segment from (1,-1,1) to (1,1,1)).
    const result = index.pickAt([1.1, 0, 1.1]);
    expect(result).not.toBe(null);
    expect(result.segments).toBe(1);
    expect(result.points.length).toBe(2);
    // Both endpoints sit on the picked cube edge: x≈1, z≈1, y = ±1.
    for (const [x, , z] of result.points) {
      expect(x).toBeCloseTo(1);
      expect(z).toBeCloseTo(1);
    }
    const ys = result.points.map((p) => p[1]).sort((m, n) => m - n);
    expect(ys[0]).toBeCloseTo(-1);
    expect(ys[1]).toBeCloseTo(1);
    // The flat polyline mirrors the points.
    expect(result.polyline.length).toBe(6);
    // Seed is the projection of the click onto the edge (y≈0, x≈1, z≈1).
    expect(result.seed[0]).toBeCloseTo(1);
    expect(result.seed[1]).toBeCloseTo(0);
    expect(result.seed[2]).toBeCloseTo(1);
    // Pick distance ≈ sqrt(0.1^2 + 0.1^2).
    expect(result.dist).toBeCloseTo(Math.hypot(0.1, 0.1));
  });

  it('does not chain across a cube corner (degree-3 junction)', () => {
    const { positions, indices } = cube();
    const index = createEdgePickIndex(positions, indices);
    const result = index.pickAt([0, 1.1, 1.1]); // near the +Y/+Z edge
    expect(result.segments).toBe(1); // stops at both corner junctions
  });

  it('chains a subdivided ridge through its degree-2 middle vertex', () => {
    const { positions, indices } = tent();
    const index = createEdgePickIndex(positions, indices);
    const result = index.pickAt([0.4, 0, 1]); // right on the ridge
    expect(result.segments).toBe(2);
    expect(result.points.length).toBe(3);
    // Ordered chain runs R0(-1)→R1(0)→R2(1) along X (or its reverse).
    const xs = result.points.map((p) => p[0]);
    const monotone =
      (xs[0] < xs[1] && xs[1] < xs[2]) || (xs[0] > xs[1] && xs[1] > xs[2]);
    expect(monotone).toBe(true);
    for (const [, y, z] of result.points) {
      expect(y).toBeCloseTo(0);
      expect(z).toBeCloseTo(1);
    }
  });

  it('reuses crease topology across picks (index is stable)', () => {
    const { positions, indices } = cube();
    const index = createEdgePickIndex(positions, indices);
    const a = index.pickAt([1.1, 0, 1.1]);
    const b = index.pickAt([1.1, 0, 1.1]);
    expect(a.polyline).toEqual(b.polyline);
  });
});
