import { describe, expect, it } from 'vitest';
import {
  BASE_RGB,
  HOVER_RGB,
  SELECTED_RGB,
  expandToNonIndexed,
  paintHighlights,
} from './faceHighlight.js';

// Two triangles sharing the edge (1, 2): a unit quad in the XY plane.
const quad = {
  positions: new Float32Array([0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0]),
  normals: new Float32Array([0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1]),
  indices: new Uint32Array([0, 1, 2, 1, 3, 2]),
};

/** The rgb triple of vertex `v` of triangle `tri` in a non-indexed array. */
function triColor(colors, tri, v) {
  const base = tri * 9 + v * 3;
  return [colors[base], colors[base + 1], colors[base + 2]];
}

/** Expected value of an rgb triple after a round-trip through Float32Array. */
const f32 = (rgb) => rgb.map(Math.fround);

describe('expandToNonIndexed', () => {
  it('unshares vertices so each triangle owns its attribute range', () => {
    const { positions, normals, colors } = expandToNonIndexed(quad);
    expect(positions.length).toBe(quad.indices.length * 3);
    expect(normals.length).toBe(quad.indices.length * 3);
    expect(colors.length).toBe(quad.indices.length * 3);
    // Vertex 1 appears in both triangles: same coordinates, distinct slots.
    expect([...positions.slice(3, 6)]).toEqual([1, 0, 0]); // tri 0, vertex 1
    expect([...positions.slice(9, 12)]).toEqual([1, 0, 0]); // tri 1, vertex 0
    expect([...normals.slice(0, 3)]).toEqual([0, 0, 1]);
    expect(colors.every((c) => c === 1)).toBe(true);
  });

  it('handles an empty mesh', () => {
    const empty = expandToNonIndexed({
      positions: new Float32Array(0),
      normals: new Float32Array(0),
      indices: new Uint32Array(0),
    });
    expect(empty.positions.length).toBe(0);
    expect(empty.colors.length).toBe(0);
  });
});

describe('paintHighlights', () => {
  it('paints region triangles and reports them for the next restore', () => {
    const { colors } = expandToNonIndexed(quad);
    const painted = paintHighlights(colors, [], [{ tris: [1], rgb: HOVER_RGB }]);
    expect(painted).toEqual([1]);
    expect(triColor(colors, 0, 0)).toEqual(BASE_RGB);
    for (let v = 0; v < 3; v += 1) {
      expect(triColor(colors, 1, v)).toEqual(f32(HOVER_RGB));
    }
  });

  it('restores previously painted triangles before painting anew', () => {
    const { colors } = expandToNonIndexed(quad);
    const first = paintHighlights(colors, [], [{ tris: [0], rgb: HOVER_RGB }]);
    const second = paintHighlights(colors, first, [{ tris: [1], rgb: HOVER_RGB }]);
    expect(second).toEqual([1]);
    expect(triColor(colors, 0, 0)).toEqual(BASE_RGB);
    expect(triColor(colors, 1, 0)).toEqual(f32(HOVER_RGB));
  });

  it('lets later regions win on overlap (selected over hover)', () => {
    const { colors } = expandToNonIndexed(quad);
    paintHighlights(colors, [], [
      { tris: [0, 1], rgb: HOVER_RGB },
      { tris: [1], rgb: SELECTED_RGB },
    ]);
    expect(triColor(colors, 0, 0)).toEqual(f32(HOVER_RGB));
    expect(triColor(colors, 1, 0)).toEqual(f32(SELECTED_RGB));
  });

  it('skips out-of-range triangles from a stale region', () => {
    const { colors } = expandToNonIndexed(quad);
    const painted = paintHighlights(colors, [5], [{ tris: [1, 7, -1], rgb: HOVER_RGB }]);
    expect(painted).toEqual([1]);
    expect(triColor(colors, 1, 0)).toEqual(f32(HOVER_RGB));
  });
});
