import { describe, expect, it } from 'vitest';
import {
  defaultMetrics,
  dimensionGeometry,
  dimensionText,
  dimensionValue,
  sheetDistance,
} from './dimensions.js';

const M = defaultMetrics(100);

describe('sheetDistance', () => {
  it('divides raw sheet distance by the placement scale', () => {
    // Sheet points 6 apart, placed at scale 2 → 3 model units.
    expect(sheetDistance([0, 0], [6, 0], 2)).toBe(3);
  });

  it('treats a missing/zero scale as unscaled', () => {
    expect(sheetDistance([0, 0], [3, 4], 0)).toBe(5);
    expect(sheetDistance([0, 0], [3, 4], undefined)).toBe(5);
  });
});

describe('dimensionValue / dimensionText', () => {
  it('measures a linear dimension in model units', () => {
    const dim = { kind: 'linear', a: [0, 0], b: [10, 0] };
    expect(dimensionValue(dim, 2)).toBe(5);
    expect(dimensionText(dim, 2)).toBe('5');
  });

  it('measures a radius and prefixes R', () => {
    const dim = { kind: 'radius', center: [0, 0], rim: [0, 8] };
    expect(dimensionValue(dim, 4)).toBe(2);
    expect(dimensionText(dim, 4)).toBe('R2');
  });

  it('returns 0 for an unknown kind', () => {
    expect(dimensionValue({ kind: 'weird' }, 1)).toBe(0);
    expect(dimensionValue(null, 1)).toBe(0);
  });

  it('formats fractional values with trailing zeros stripped', () => {
    const dim = { kind: 'linear', a: [0, 0], b: [2.5, 0] };
    expect(dimensionText(dim, 1)).toBe('2.5');
  });
});

describe('defaultMetrics', () => {
  it('scales metrics with the sheet diagonal', () => {
    const small = defaultMetrics(10);
    const big = defaultMetrics(100);
    expect(big.arrow).toBeGreaterThan(small.arrow);
    expect(big.offset).toBeGreaterThan(big.arrow);
  });

  it('falls back to a unit base for a degenerate diagonal', () => {
    const m = defaultMetrics(0);
    expect(m.arrow).toBeGreaterThan(0);
  });
});

describe('dimensionGeometry — linear', () => {
  const dim = { kind: 'linear', a: [0, 0], b: [10, 0], offset: 5 };
  const g = dimensionGeometry(dim, 1, M);

  it('reports the value and text', () => {
    expect(g.value).toBe(10);
    expect(g.text).toBe('10');
  });

  it('emits two witness lines and one dimension line', () => {
    expect(g.lines).toHaveLength(3);
    // The dimension line is the last, offset perpendicular by `offset`.
    const [oa, ob] = g.lines[2];
    expect(oa[1]).toBe(5);
    expect(ob[1]).toBe(5);
    expect(oa[0]).toBe(0);
    expect(ob[0]).toBe(10);
  });

  it('emits an arrowhead triangle at each dimension-line end', () => {
    expect(g.arrows).toHaveLength(2);
    for (const tri of g.arrows) expect(tri).toHaveLength(3);
    // Tips sit on the dimension line ends.
    expect(g.arrows[0][0]).toEqual([0, 5]);
    expect(g.arrows[1][0]).toEqual([10, 5]);
  });

  it('centers the label on the dimension line, nudged off it', () => {
    expect(g.label.text).toBe('10');
    expect(g.label.pos[0]).toBe(5); // midpoint x
    expect(g.label.pos[1]).toBeGreaterThan(5); // nudged past the offset
  });

  it('flips the stand-off side for a negative offset', () => {
    const below = dimensionGeometry(
      { kind: 'linear', a: [0, 0], b: [10, 0], offset: -5 },
      1,
      M
    );
    expect(below.lines[2][0][1]).toBe(-5);
    expect(below.label.pos[1]).toBeLessThan(-5);
  });

  it('handles coincident anchors without NaN', () => {
    const deg = dimensionGeometry(
      { kind: 'linear', a: [2, 2], b: [2, 2] },
      1,
      M
    );
    expect(deg.lines).toHaveLength(0);
    expect(deg.arrows).toHaveLength(0);
  });
});

describe('dimensionGeometry — radius', () => {
  const dim = { kind: 'radius', center: [0, 0], rim: [6, 0] };
  const g = dimensionGeometry(dim, 2, M);

  it('measures radius in model units and labels it R', () => {
    expect(g.value).toBe(3);
    expect(g.text).toBe('R3');
    expect(g.label.text).toBe('R3');
  });

  it('draws a single leader center→rim with one arrowhead at the rim', () => {
    expect(g.lines).toHaveLength(1);
    expect(g.lines[0]).toEqual([
      [0, 0],
      [6, 0],
    ]);
    expect(g.arrows).toHaveLength(1);
    expect(g.arrows[0][0]).toEqual([6, 0]); // tip at the rim
  });
});

describe('dimensionGeometry — fallbacks', () => {
  it('returns empty geometry for an unknown kind', () => {
    const g = dimensionGeometry({ kind: 'nope' }, 1, M);
    expect(g.lines).toHaveLength(0);
    expect(g.arrows).toHaveLength(0);
    expect(g.label).toBeNull();
  });

  it('tolerates missing metrics', () => {
    const g = dimensionGeometry({ kind: 'linear', a: [0, 0], b: [4, 0] }, 1);
    expect(g.value).toBe(4);
    expect(g.lines.length).toBeGreaterThan(0);
  });
});
