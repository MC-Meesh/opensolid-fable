import { describe, expect, it } from 'vitest';
import { buildBinaryStl } from './stl.js';

// One right triangle in the xy plane, CCW as seen from +z: normal is +z.
const TRIANGLE_POSITIONS = new Float32Array([
  0, 0, 0,
  1, 0, 0,
  0, 1, 0,
]);
const TRIANGLE_INDICES = new Uint32Array([0, 1, 2]);

describe('buildBinaryStl', () => {
  it('sizes the buffer as 84-byte header plus 50 bytes per triangle', () => {
    const buffer = buildBinaryStl(TRIANGLE_POSITIONS, TRIANGLE_INDICES);
    expect(buffer.byteLength).toBe(84 + 50);
  });

  it('writes the triangle count little-endian at offset 80', () => {
    const positions = new Float32Array([
      0, 0, 0, 1, 0, 0, 0, 1, 0,
      0, 0, 1, 1, 0, 1, 0, 1, 1,
    ]);
    const indices = new Uint32Array([0, 1, 2, 3, 4, 5]);
    const view = new DataView(buildBinaryStl(positions, indices));
    expect(view.getUint32(80, true)).toBe(2);
  });

  it('recomputes the facet normal from CCW winding', () => {
    const view = new DataView(buildBinaryStl(TRIANGLE_POSITIONS, TRIANGLE_INDICES));
    expect(view.getFloat32(84, true)).toBeCloseTo(0);
    expect(view.getFloat32(88, true)).toBeCloseTo(0);
    expect(view.getFloat32(92, true)).toBeCloseTo(1);
  });

  it('round-trips vertex coordinates', () => {
    const view = new DataView(buildBinaryStl(TRIANGLE_POSITIONS, TRIANGLE_INDICES));
    const vertices = [];
    for (let i = 0; i < 9; i++) vertices.push(view.getFloat32(96 + i * 4, true));
    expect(vertices).toEqual([0, 0, 0, 1, 0, 0, 0, 1, 0]);
  });

  it('zeroes the normal for degenerate triangles', () => {
    const positions = new Float32Array([0, 0, 0, 0, 0, 0, 0, 0, 0]);
    const view = new DataView(buildBinaryStl(positions, new Uint32Array([0, 1, 2])));
    expect(view.getFloat32(84, true)).toBe(0);
    expect(view.getFloat32(88, true)).toBe(0);
    expect(view.getFloat32(92, true)).toBe(0);
  });

  it('ignores trailing indices that do not form a full triangle', () => {
    const buffer = buildBinaryStl(TRIANGLE_POSITIONS, new Uint32Array([0, 1, 2, 0, 1]));
    expect(new DataView(buffer).getUint32(80, true)).toBe(1);
  });

  it('produces an empty STL for an empty mesh', () => {
    const buffer = buildBinaryStl(new Float32Array(0), new Uint32Array(0));
    expect(buffer.byteLength).toBe(84);
    expect(new DataView(buffer).getUint32(80, true)).toBe(0);
  });
});
