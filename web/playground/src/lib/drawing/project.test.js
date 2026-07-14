import { describe, expect, it } from 'vitest';
import {
  bodyEdges3d,
  meshFeatureEdges,
  meshSilhouetteEdges,
  projectPoint,
  projectView,
  viewBasis,
} from './project.js';

// A unit cube [0,1]^3 as a closed triangle mesh (12 tris, shared vertices).
const CUBE_POS = new Float32Array([
  0, 0, 0, // 0
  1, 0, 0, // 1
  1, 1, 0, // 2
  0, 1, 0, // 3
  0, 0, 1, // 4
  1, 0, 1, // 5
  1, 1, 1, // 6
  0, 1, 1, // 7
]);
// Outward-wound (CCW seen from outside) faces.
const CUBE_IDX = new Uint32Array([
  0, 3, 2, 0, 2, 1, // z=0 (normal -Z)
  4, 5, 6, 4, 6, 7, // z=1 (normal +Z)
  0, 1, 5, 0, 5, 4, // y=0 (normal -Y)
  3, 7, 6, 3, 6, 2, // y=1 (normal +Y)
  0, 4, 7, 0, 7, 3, // x=0 (normal -X)
  1, 2, 6, 1, 6, 5, // x=1 (normal +X)
]);
const CUBE = { positions: CUBE_POS, indices: CUBE_IDX };

describe('viewBasis', () => {
  it('front looks down -Z: u = +X, v = +Y, w = +Z', () => {
    const b = viewBasis('front');
    expect(b.u).toEqual([1, 0, 0]);
    expect(b.v).toEqual([0, 1, 0]);
    expect(b.w).toEqual([0, 0, 1]);
  });

  it('right looks down +X: depth axis is world X, Y stays up', () => {
    const b = viewBasis('right');
    expect(b.w).toEqual([1, 0, 0]);
    expect(b.v).toEqual([0, 1, 0]); // up preserved
    expect(b.u).toEqual([0, 0, -1]); // screen-right = -Z
  });

  it('top looks down +Y with X to the right', () => {
    const b = viewBasis('top');
    expect(b.u[0]).toBeCloseTo(1, 6);
    expect(b.w[1]).toBeCloseTo(1, 3); // ~+Y (tiny pole tilt)
  });

  it('is orthonormal for every standard view', () => {
    for (const name of ['front', 'back', 'left', 'right', 'top', 'bottom', 'iso']) {
      const { u, v, w } = viewBasis(name);
      const d = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
      expect(d(u, u)).toBeCloseTo(1, 6);
      expect(d(v, v)).toBeCloseTo(1, 6);
      expect(d(w, w)).toBeCloseTo(1, 6);
      expect(d(u, v)).toBeCloseTo(0, 6);
      expect(d(u, w)).toBeCloseTo(0, 6);
      expect(d(v, w)).toBeCloseTo(0, 6);
    }
  });

  it('returns null for an unknown view', () => {
    expect(viewBasis('nope')).toBeNull();
  });
});

describe('projectPoint', () => {
  it('front projection keeps (x, y) and reports z as depth', () => {
    const b = viewBasis('front');
    expect(projectPoint(b, [2, 3, 5])).toEqual([2, 3, 5]);
  });
});

describe('meshFeatureEdges', () => {
  it('finds the 12 sharp edges of a cube', () => {
    const segs = meshFeatureEdges(CUBE_POS, CUBE_IDX);
    expect(segs.length).toBe(12);
  });

  it('drops edges below the dihedral threshold', () => {
    // A single flat quad (two coplanar triangles): the shared diagonal is not
    // a feature edge, but the four boundary edges are.
    const pos = new Float32Array([0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0]);
    const idx = new Uint32Array([0, 1, 2, 0, 2, 3]);
    const segs = meshFeatureEdges(pos, idx);
    expect(segs.length).toBe(4); // 4 boundary edges, diagonal excluded
  });

  it('returns nothing for an empty mesh', () => {
    expect(meshFeatureEdges(new Float32Array(0), new Uint32Array(0))).toEqual([]);
  });

  it('welds duplicated vertices so adjacency is recovered', () => {
    // Same flat quad but each triangle has its own copy of the shared edge
    // vertices; welding must still find 4 boundary edges (not 6).
    const pos = new Float32Array([
      0, 0, 0, 1, 0, 0, 1, 1, 0, // tri A
      0, 0, 0, 1, 1, 0, 0, 1, 0, // tri B (shares edge 0-2)
    ]);
    const idx = new Uint32Array([0, 1, 2, 3, 4, 5]);
    expect(meshFeatureEdges(pos, idx).length).toBe(4);
  });
});

describe('meshSilhouetteEdges', () => {
  it('finds a ridge where adjacent faces flip sign across the view', () => {
    // Two triangles sharing edge (0,0,0)-(1,0,0): one tilts toward +Z
    // (n·w > 0), the other toward -Z (n·w < 0). Viewed along +Z the shared
    // ridge is a silhouette; the 4 outer edges are boundary edges (always on
    // the outline). So 5 segments total.
    const pos = new Float32Array([0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 1, -1]);
    const idx = new Uint32Array([0, 1, 2, 1, 0, 3]);
    const segs = meshSilhouetteEdges(pos, idx, [0, 0, 1]);
    expect(segs.length).toBe(5);
    const hasRidge = segs.some(
      ([a, b]) =>
        (a[0] === 0 && a[1] === 0 && b[0] === 1 && b[1] === 0) ||
        (b[0] === 0 && b[1] === 0 && a[0] === 1 && a[1] === 0)
    );
    expect(hasRidge).toBe(true);
  });

  it('a cube viewed face-on has no interior silhouette (feature edges cover it)', () => {
    // Every side face is edge-on (n·w = 0); grazing faces are not a strict
    // sign flip, so the outline comes from feature edges, not silhouettes.
    const segs = meshSilhouetteEdges(CUBE_POS, CUBE_IDX, [0, 0, 1]);
    expect(segs.length).toBe(0);
  });

  it('returns nothing for an empty mesh', () => {
    expect(meshSilhouetteEdges(new Float32Array(0), new Uint32Array(0), [0, 0, 1])).toEqual([]);
  });
});

describe('bodyEdges3d', () => {
  it('unions feature and silhouette edges', () => {
    const edges = bodyEdges3d(CUBE, 'front');
    expect(edges.length).toBeGreaterThan(0);
    // Every entry is a pair of 3D points.
    for (const [a, b] of edges) {
      expect(a).toHaveLength(3);
      expect(b).toHaveLength(3);
    }
  });

  it('prefers a supplied featureEdges buffer over the dihedral walk', () => {
    const featureEdges = new Float32Array([0, 0, 0, 1, 0, 0]); // one segment
    const mesh = { ...CUBE, featureEdges };
    const edges = bodyEdges3d(mesh, 'front');
    // First segment comes from the buffer; silhouettes append after.
    expect(edges[0]).toEqual([
      [0, 0, 0],
      [1, 0, 0],
    ]);
  });

  it('is empty for an unknown view', () => {
    expect(bodyEdges3d(CUBE, 'nope')).toEqual([]);
  });
});

describe('projectView', () => {
  it('projects the cube front view into a unit square of segments', () => {
    const { view, segments, bounds } = projectView(CUBE, 'front');
    expect(view).toBe('front');
    expect(segments.length).toBeGreaterThan(0);
    expect(bounds.minX).toBeCloseTo(0, 6);
    expect(bounds.minY).toBeCloseTo(0, 6);
    expect(bounds.maxX).toBeCloseTo(1, 6);
    expect(bounds.maxY).toBeCloseTo(1, 6);
    for (const seg of segments) {
      expect(seg.style).toBe('visible');
      expect(seg.pts).toHaveLength(2);
      expect(typeof seg.depth).toBe('number');
    }
  });

  it('returns empty geometry with null bounds for an empty mesh', () => {
    const empty = { positions: new Float32Array(0), indices: new Uint32Array(0) };
    const p = projectView(empty, 'front');
    expect(p.segments).toEqual([]);
    expect(p.bounds).toBeNull();
  });

  it('returns empty geometry for an unknown view', () => {
    const p = projectView(CUBE, 'nope');
    expect(p.segments).toEqual([]);
    expect(p.bounds).toBeNull();
  });
});
