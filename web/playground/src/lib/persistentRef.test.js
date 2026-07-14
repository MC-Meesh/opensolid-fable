import { describe, expect, it } from 'vitest';
import {
  ANCHOR_TOL_FACTOR,
  faceRefFromPlane,
  planarRegionsOf,
  resolveFaceRef,
  resolveRefs,
} from './persistentRef.js';

/** A fake planar region as facePlane.js would produce. */
function region(normal, origin, { extent = 1, planar = true } = {}) {
  return { planar, plane: { origin, normal, u: [1, 0, 0], v: [0, 1, 0], extent }, tris: [] };
}

/** A region index over a fixed list, addressed by triangle -> region. */
function fakeIndex(regionsByTri) {
  return { regionAt: (t) => regionsByTri[t] ?? { planar: false, tris: [] } };
}

describe('faceRefFromPlane', () => {
  it('captures owner, orientation, and anchor as copies', () => {
    const plane = { origin: [1, 2, 3], normal: [0, 0, 1], extent: 0.5 };
    const ref = faceRefFromPlane('extrude:1', plane);
    expect(ref).toEqual({
      owner: 'extrude:1',
      normal: [0, 0, 1],
      anchor: [1, 2, 3],
      extent: 0.5,
    });
    // Copies, not aliases: mutating the source plane must not leak in.
    plane.origin[0] = 99;
    plane.normal[2] = 99;
    expect(ref.anchor[0]).toBe(1);
    expect(ref.normal[2]).toBe(1);
  });

  it('defaults a missing extent to 0', () => {
    expect(faceRefFromPlane('f', { origin: [0, 0, 0], normal: [1, 0, 0] }).extent).toBe(0);
  });
});

describe('planarRegionsOf', () => {
  it('collects distinct planar regions, deduped by identity', () => {
    const top = region([0, 0, 1], [0, 0, 1]);
    const side = region([1, 0, 0], [1, 0, 0]);
    // top spans tris 0,1; side spans tri 2; tri 3 is non-planar (curved facet).
    const index = fakeIndex([top, top, side, { planar: false, tris: [] }]);
    const regions = planarRegionsOf(index, 4);
    expect(regions).toHaveLength(2);
    expect(regions).toContain(top);
    expect(regions).toContain(side);
  });

  it('returns nothing for a mesh with no planar faces', () => {
    const index = fakeIndex([{ planar: false }, { planar: false }]);
    expect(planarRegionsOf(index, 2)).toEqual([]);
  });
});

describe('resolveFaceRef', () => {
  const ref = faceRefFromPlane('extrude:1', { origin: [0, 0, 1], normal: [0, 0, 1] });

  it('re-resolves to the same face after it slides along its normal', () => {
    // Upstream edit grew the box: the top face moved from z=1 to z=1.5.
    const moved = region([0, 0, 1], [0, 0, 1.5]);
    const other = region([1, 0, 0], [1, 0, 0]);
    const result = resolveFaceRef(ref, [moved, other], 2);
    expect(result.resolved).toBe(true);
    expect(result.plane).toBe(moved.plane);
    expect(result.anchor).toEqual([0, 0, 1.5]);
    expect(result.distance).toBeCloseTo(0.5);
  });

  it('gates on orientation before distance: a nearer opposite face is rejected', () => {
    // A downward face right at the anchor must NOT match an upward reference.
    const down = region([0, 0, -1], [0, 0, 1]);
    const up = region([0, 0, 1], [0, 0, 1.2]);
    const result = resolveFaceRef(ref, [down, up], 4);
    expect(result.resolved).toBe(true);
    expect(result.plane).toBe(up.plane); // the oriented one, though farther
  });

  it('picks the nearest among several equally-oriented faces', () => {
    const near = region([0, 0, 1], [0, 0, 1.1]);
    const far = region([0, 0, 1], [0, 0, 3]);
    const result = resolveFaceRef(ref, [far, near], 5);
    expect(result.plane).toBe(near.plane);
  });

  it('reports dangling when no face shares the orientation', () => {
    const side = region([1, 0, 0], [0, 0, 1]);
    const result = resolveFaceRef(ref, [side], 2);
    expect(result.resolved).toBe(false);
    expect(result.reason).toBe('no matching orientation');
  });

  it('reports dangling when the nearest oriented face is beyond tolerance', () => {
    // Oriented face exists but 10 units away; tolerance is 0.35 * radius(2).
    const faraway = region([0, 0, 1], [0, 0, 11]);
    const result = resolveFaceRef(ref, [faraway], 2);
    expect(result.resolved).toBe(false);
    expect(result.reason).toBe('nearest face too far');
    expect(ANCHOR_TOL_FACTOR * 2).toBeLessThan(11);
  });

  it('admits a small normal wobble within the angular tolerance', () => {
    // ~2° off the reference normal — inside NORMAL_TOL_DEG (3°).
    const rad = (2 * Math.PI) / 180;
    const wobbled = region([0, Math.sin(rad), Math.cos(rad)], [0, 0, 1.05]);
    expect(resolveFaceRef(ref, [wobbled], 2).resolved).toBe(true);
  });
});

describe('resolveRefs', () => {
  it('re-anchors live refs and passes dangling refs through unchanged', () => {
    const live = faceRefFromPlane('extrude:1', { origin: [0, 0, 1], normal: [0, 0, 1] });
    const gone = faceRefFromPlane('extrude:2', { origin: [5, 5, 5], normal: [0, 1, 0] });
    const refMap = new Map([
      ['extrude:1', live],
      ['extrude:2', gone],
    ]);
    const moved = region([0, 0, 1], [0, 0, 1.4]);
    const { statuses, refs } = resolveRefs(refMap, [moved], 3);

    expect(statuses.get('extrude:1')).toEqual({ status: 'ok' });
    expect(statuses.get('extrude:2').status).toBe('dangling');

    // Live ref re-anchored to the face's new centroid...
    expect(refs.get('extrude:1').anchor).toEqual([0, 0, 1.4]);
    // ...dangling ref preserved so a later edit can bring the face back.
    expect(refs.get('extrude:2')).toBe(gone);
  });

  it('is a no-op on an empty ref map', () => {
    const { statuses, refs } = resolveRefs(new Map(), [], 1);
    expect(statuses.size).toBe(0);
    expect(refs.size).toBe(0);
  });
});
