import { describe, expect, it } from 'vitest';
import {
  buildEdgeTriMap,
  orderPolyline,
  pickEdge,
  pointSegmentDist2,
} from './edgePick.js';

describe('pointSegmentDist2', () => {
  it('measures perpendicular distance for a point beside the segment', () => {
    // Segment along X from origin to (2,0,0); point one unit above its middle.
    expect(pointSegmentDist2([1, 1, 0], [0, 0, 0], [2, 0, 0])).toBeCloseTo(1);
  });

  it('clamps to the nearest endpoint past the segment ends', () => {
    // Point beyond the far end: nearest point is the endpoint (2,0,0).
    expect(pointSegmentDist2([4, 0, 0], [0, 0, 0], [2, 0, 0])).toBeCloseTo(4);
  });

  it('handles a degenerate (zero-length) segment as point distance', () => {
    expect(pointSegmentDist2([3, 4, 0], [0, 0, 0], [0, 0, 0])).toBeCloseTo(25);
  });
});

describe('buildEdgeTriMap', () => {
  it('maps a shared interior edge to both triangles', () => {
    // Two triangles of a quad sharing the diagonal 0-2.
    const indices = [0, 1, 2, 0, 2, 3];
    const map = buildEdgeTriMap(indices);
    const base = 2 ** 26;
    const diag = 0 * base + 2; // undirected key for {0,2}
    expect(map.get(diag).sort()).toEqual([0, 1]);
    // A border edge belongs to just one triangle.
    expect(map.get(0 * base + 1)).toEqual([0]);
  });
});

describe('orderPolyline', () => {
  it('walks an open arc end to end', () => {
    // Path 0-1-2-3 as an unordered bag of edges.
    const edges = [
      [1, 2],
      [0, 1],
      [2, 3],
    ];
    const order = orderPolyline(edges, 0);
    expect(order).toEqual([0, 1, 2, 3]);
  });

  it('keeps only the component containing the seed', () => {
    // Two disjoint seams: 0-1-2 and 10-11. Seeding on 11 returns just that one.
    const edges = [
      [0, 1],
      [1, 2],
      [10, 11],
    ];
    expect(orderPolyline(edges, 11).sort()).toEqual([10, 11]);
  });

  it('traverses a closed loop from the seed', () => {
    const edges = [
      [0, 1],
      [1, 2],
      [2, 0],
    ];
    const order = orderPolyline(edges, 0);
    expect(order).toHaveLength(3);
    expect(new Set(order)).toEqual(new Set([0, 1, 2]));
    expect(order[0]).toBe(0);
  });

  it('returns [] for an empty edge bag', () => {
    expect(orderPolyline([], 0)).toEqual([]);
  });
});

// A minimal two-region mesh: a horizontal quad (region A, z=0) and a vertical
// quad (region B, x=1) meeting along the shared edge v1(1,0,0)-v2(1,1,0).
function twoRegionMesh() {
  const positions = [
    0, 0, 0, // v0
    1, 0, 0, // v1
    1, 1, 0, // v2
    0, 1, 0, // v3
    1, 0, 1, // v4
    1, 1, 1, // v5
  ];
  const indices = [
    0, 1, 2, // t0 (A)
    0, 2, 3, // t1 (A)
    1, 2, 5, // t2 (B)
    1, 5, 4, // t3 (B)
  ];
  const regionA = {
    planar: true,
    tris: [0, 1],
    plane: { origin: [0.5, 0.5, 0], normal: [0, 0, 1] },
  };
  const regionB = {
    planar: true,
    tris: [2, 3],
    plane: { origin: [1, 0.5, 0.5], normal: [1, 0, 0] },
  };
  const regions = {
    regionAt: (tri) => (tri <= 1 ? regionA : regionB),
  };
  return { positions, indices, regions, regionA, regionB };
}

describe('pickEdge', () => {
  it('traces the crease between two planar regions', () => {
    const { positions, indices, regions, regionA, regionB } = twoRegionMesh();
    const result = pickEdge(regions, positions, indices, [1, 0.5, 0.05], 0);
    expect(result.ok).toBe(true);
    expect(result.points).toHaveLength(2);
    // The crease is the shared edge (1,0,0)-(1,1,0).
    const xs = result.points.map((p) => p[0]);
    expect(xs).toEqual([1, 1]);
    expect(result.flat).toHaveLength(6);
    expect(result.regionA).toBe(regionA);
    expect(result.regionB).toBe(regionB);
  });

  it('reports the seed point from the pick ray', () => {
    const { positions, indices, regions } = twoRegionMesh();
    const result = pickEdge(regions, positions, indices, [1, 0.5, 0.05], 0);
    expect(result.seed).toEqual([1, 0.5, 0.05]);
  });

  it('refuses a click with no face under it', () => {
    const { positions, indices, regions } = twoRegionMesh();
    const result = pickEdge(regions, positions, indices, [0, 0, 0], null);
    expect(result.ok).toBe(false);
  });

  it('rejects a curved clicked face', () => {
    const { positions, indices } = twoRegionMesh();
    const regions = {
      regionAt: () => ({ planar: false, reason: 'face is curved', tris: [0] }),
    };
    const result = pickEdge(regions, positions, indices, [1, 0.5, 0.05], 0);
    expect(result.ok).toBe(false);
    expect(result.reason).toMatch(/curved/i);
  });

  it('rejects an edge that borders a curved face', () => {
    const { positions, indices, regionA } = twoRegionMesh();
    const curvedB = { planar: false, reason: 'face is curved', tris: [2, 3] };
    const regions = { regionAt: (tri) => (tri <= 1 ? regionA : curvedB) };
    const result = pickEdge(regions, positions, indices, [1, 0.5, 0.05], 0);
    expect(result.ok).toBe(false);
    expect(result.reason).toMatch(/curved|flat faces/i);
  });
});
