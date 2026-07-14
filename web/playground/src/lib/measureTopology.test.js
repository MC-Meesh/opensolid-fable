import { describe, expect, it } from 'vitest';
import { buildEdgeModel, fitCircle, snapEntity } from './measureTopology.js';

// A unit box over [-1, 1]^3: 8 corners, 12 triangles (2 per face, each face's
// pair wound consistently so their normals agree and the shared diagonal is
// coplanar, not a crease).
const BOX_POSITIONS = [
  -1, -1, -1, // 0
  1, -1, -1, //  1
  1, 1, -1, //   2
  -1, 1, -1, //  3
  -1, -1, 1, //  4
  1, -1, 1, //   5
  1, 1, 1, //    6
  -1, 1, 1, //   7
];
const BOX_INDICES = [
  0, 1, 2, 0, 2, 3, // z-
  4, 5, 6, 4, 6, 7, // z+
  0, 3, 7, 0, 7, 4, // x-
  1, 2, 6, 1, 6, 5, // x+
  0, 1, 5, 0, 5, 4, // y-
  3, 2, 6, 3, 6, 7, // y+
];

describe('buildEdgeModel', () => {
  const model = buildEdgeModel(BOX_POSITIONS, BOX_INDICES);

  it('finds the 8 corners of a box', () => {
    expect(model.corners).toHaveLength(8);
  });

  it('finds the 12 edges of a box', () => {
    expect(model.edges).toHaveLength(12);
  });

  it('reports each box edge as a straight line of length 2', () => {
    for (const e of model.edges) {
      expect(e.straight).toBe(true);
      expect(e.closed).toBe(false);
      expect(e.circle).toBeNull();
      expect(e.length).toBeCloseTo(2, 6);
    }
  });

  it('ignores coplanar interior (diagonal) edges', () => {
    // 12 cube edges only — no face diagonals leak in as features.
    expect(model.edges.every((e) => e.points.length === 2)).toBe(true);
  });

  it('reports the mesh diagonal', () => {
    expect(model.diagonal).toBeCloseTo(Math.sqrt(12), 6);
  });
});

describe('snapEntity', () => {
  const model = buildEdgeModel(BOX_POSITIONS, BOX_INDICES);

  it('snaps a hit near a corner to that vertex', () => {
    const e = snapEntity(model, [0.95, 0.95, 0.95], 0.2);
    expect(e.kind).toBe('vertex');
    expect(e.point[0]).toBeCloseTo(1, 6);
    expect(e.point[1]).toBeCloseTo(1, 6);
    expect(e.point[2]).toBeCloseTo(1, 6);
  });

  it('snaps a hit near an edge midpoint to that edge', () => {
    const e = snapEntity(model, [1, -1, 0.02], 0.15);
    expect(e.kind).toBe('edge');
    expect(e.length).toBeCloseTo(2, 6);
  });

  it('prefers a vertex over an edge when both are in range', () => {
    const e = snapEntity(model, [0.9, -0.9, -1], 0.4);
    expect(e.kind).toBe('vertex');
  });

  it('returns null when nothing is within tolerance', () => {
    expect(snapEntity(model, [0, 0, 0], 0.2)).toBeNull();
  });
});

describe('fitCircle', () => {
  it('fits a planar ring of points to a circle', () => {
    const r = 3;
    const points = [];
    for (let i = 0; i < 16; i += 1) {
      const a = (i / 16) * 2 * Math.PI;
      points.push([r * Math.cos(a), r * Math.sin(a), 5]);
    }
    const fit = fitCircle(points);
    expect(fit.radius).toBeCloseTo(3, 6);
    expect(fit.center[0]).toBeCloseTo(0, 6);
    expect(fit.center[1]).toBeCloseTo(0, 6);
    expect(fit.center[2]).toBeCloseTo(5, 6);
    expect(Math.abs(fit.normal[2])).toBeCloseTo(1, 6);
  });

  it('rejects a non-circular loop', () => {
    // An ellipse (a=4, b=1): radii swing far too much to read as a circle.
    const ellipse = [];
    for (let i = 0; i < 16; i += 1) {
      const a = (i / 16) * 2 * Math.PI;
      ellipse.push([4 * Math.cos(a), Math.sin(a), 0]);
    }
    expect(fitCircle(ellipse)).toBeNull();
  });
});
