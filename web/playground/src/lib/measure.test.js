import { describe, expect, it } from 'vitest';
import { boundingBoxDims, measurePair, measureSingle, triListArea } from './measure.js';

const BOX_POSITIONS = [
  -1, -1, -1, 1, -1, -1, 1, 1, -1, -1, 1, -1,
  -1, -1, 1, 1, -1, 1, 1, 1, 1, -1, 1, 1,
];
const BOX_INDICES = [
  0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7,
  0, 3, 7, 0, 7, 4, 1, 2, 6, 1, 6, 5,
  0, 1, 5, 0, 5, 4, 3, 2, 6, 3, 6, 7,
];

describe('boundingBoxDims', () => {
  it('measures a box', () => {
    const b = boundingBoxDims(BOX_POSITIONS);
    expect(b.size).toEqual([2, 2, 2]);
    expect(b.diagonal).toBeCloseTo(Math.sqrt(12), 6);
    expect(b.center).toEqual([0, 0, 0]);
  });

  it('returns null for an empty mesh', () => {
    expect(boundingBoxDims([])).toBeNull();
  });
});

describe('triListArea', () => {
  it('sums the box surface area', () => {
    const tris = Array.from({ length: BOX_INDICES.length / 3 }, (_, i) => i);
    expect(triListArea(BOX_POSITIONS, BOX_INDICES, tris)).toBeCloseTo(24, 6);
  });

  it('sums a single face (two triangles)', () => {
    expect(triListArea(BOX_POSITIONS, BOX_INDICES, [0, 1])).toBeCloseTo(4, 6);
  });
});

describe('measureSingle', () => {
  it('reports an edge length', () => {
    const m = measureSingle({ kind: 'edge', length: 2.5, a: [0, 0, 0], b: [2.5, 0, 0] });
    expect(m.length).toBeCloseTo(2.5, 6);
  });

  it('reports a circle radius and diameter', () => {
    const m = measureSingle({ kind: 'circle', radius: 3, center: [0, 0, 0] });
    expect(m.diameter).toBeCloseTo(6, 6);
    expect(m.circumference).toBeCloseTo(2 * Math.PI * 3, 6);
  });

  it('reports a face area', () => {
    const m = measureSingle({ kind: 'face', area: 12, origin: [0, 0, 0], normal: [0, 0, 1] });
    expect(m.area).toBe(12);
  });

  it('reports a vertex coordinate', () => {
    const m = measureSingle({ kind: 'vertex', point: [1, 2, 3] });
    expect(m.coord).toEqual([1, 2, 3]);
  });
});

describe('measurePair', () => {
  const vertex = (p) => ({ kind: 'vertex', point: p });
  const face = (origin, normal) => ({ kind: 'face', origin, normal, area: 1 });
  const edge = (a, b) => ({ kind: 'edge', a, b, length: 1 });

  it('measures distance and per-axis delta between two vertices', () => {
    const m = measurePair(vertex([0, 0, 0]), vertex([3, 4, 0]));
    expect(m.distance).toBeCloseTo(5, 6);
    expect(m.delta).toEqual([3, 4, 0]);
  });

  it('reports the angle and gap between two parallel faces', () => {
    const m = measurePair(face([0, 0, 0], [0, 0, 1]), face([0, 0, 5], [0, 0, -1]));
    expect(m.angle).toBeCloseTo(180, 6);
    expect(m.planeDistance).toBeCloseTo(5, 6);
  });

  it('reports the angle between two perpendicular faces without a gap', () => {
    const m = measurePair(face([0, 0, 0], [0, 0, 1]), face([0, 0, 0], [1, 0, 0]));
    expect(m.angle).toBeCloseTo(90, 6);
    expect(m.planeDistance).toBeUndefined();
  });

  it('measures the perpendicular distance from a point to a face', () => {
    const m = measurePair(face([0, 0, 0], [0, 1, 0]), vertex([1, 3, 2]));
    expect(m.planeDistance).toBeCloseTo(3, 6);
    expect(m.distance).toBeCloseTo(Math.hypot(1, 3, 2), 6);
  });

  it('measures the angle between two edges', () => {
    const m = measurePair(edge([0, 0, 0], [1, 0, 0]), edge([0, 0, 0], [0, 1, 0]));
    expect(m.angle).toBeCloseTo(90, 6);
  });
});
