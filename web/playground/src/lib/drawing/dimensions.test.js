import { describe, expect, it } from 'vitest';
import {
  measure,
  formatDimLabel,
  createLinearDim,
  createRadiusDim,
  dimensionGeometry,
  dimensionBounds,
  nextDimId,
} from './dimensions.js';

describe('measure', () => {
  it('returns sheet distance divided by scale', () => {
    expect(measure([0, 0], [3, 4], 1)).toBeCloseTo(5);
    expect(measure([0, 0], [3, 4], 2)).toBeCloseTo(2.5);
  });

  it('treats a non-positive scale as 1', () => {
    expect(measure([0, 0], [6, 8], 0)).toBeCloseTo(10);
    expect(measure([0, 0], [6, 8], -2)).toBeCloseTo(10);
  });
});

describe('formatDimLabel', () => {
  it('prefixes radius with R and leaves linear bare', () => {
    expect(formatDimLabel('linear', 12.5)).toBe('12.5');
    expect(formatDimLabel('radius', 4)).toBe('R4');
  });

  it('reuses the sketch compact formatting (2 decimals, stripped)', () => {
    expect(formatDimLabel('linear', 3.14159)).toBe('3.14');
    expect(formatDimLabel('linear', 10)).toBe('10');
  });
});

describe('createLinearDim', () => {
  it('freezes the model value at creation and stores anchors', () => {
    const dim = createLinearDim([0, 0], [10, 0], 2, 2);
    expect(dim.kind).toBe('linear');
    expect(dim.a).toEqual([0, 0]);
    expect(dim.b).toEqual([10, 0]);
    expect(dim.offset).toBe(2);
    expect(dim.value).toBeCloseTo(5); // 10 sheet units / scale 2
    expect(dim.id).toMatch(/^dim\d+$/);
  });
});

describe('createRadiusDim', () => {
  it('measures center→rim as the model radius', () => {
    const dim = createRadiusDim([0, 0], [0, 6], 1, 3);
    expect(dim.kind).toBe('radius');
    expect(dim.value).toBeCloseTo(2); // 6 / 3
  });
});

describe('nextDimId', () => {
  it('is unique and monotonic', () => {
    const a = nextDimId();
    const b = nextDimId();
    expect(a).not.toBe(b);
  });
});

describe('dimensionGeometry — linear', () => {
  const dim = createLinearDim([0, 0], [10, 0], 3, 1);
  const g = dimensionGeometry(dim, { arrow: 1, ext: 0.5, gap: 0, textGap: 1 });

  it('offsets the dimension line by the perpendicular offset', () => {
    // Horizontal segment offset +3 in +Y → dimension line at y=3.
    const dimLine = g.lines[0];
    expect(dimLine[0][1]).toBeCloseTo(3);
    expect(dimLine[1][1]).toBeCloseTo(3);
    expect(dimLine[0][0]).toBeCloseTo(0);
    expect(dimLine[1][0]).toBeCloseTo(10);
  });

  it('emits two witness lines and two arrowheads', () => {
    expect(g.lines).toHaveLength(3); // dim line + 2 witness
    expect(g.arrowheads).toHaveLength(2);
    for (const head of g.arrowheads) expect(head).toHaveLength(3);
  });

  it('places the label centered above the dimension line', () => {
    expect(g.text.label).toBe('10');
    expect(g.text.pos[0]).toBeCloseTo(5);
    expect(g.text.pos[1]).toBeCloseTo(4); // offset 3 + textGap 1
  });

  it('keeps the label upright for a leftward segment', () => {
    const flipped = createLinearDim([10, 0], [0, 0], 1, 1);
    const gf = dimensionGeometry(flipped, { arrow: 1 });
    expect(Math.abs(gf.text.angle)).toBeLessThanOrEqual(Math.PI / 2 + 1e-9);
  });

  it('draws nothing for coincident anchors', () => {
    const degenerate = createLinearDim([2, 2], [2, 2], 1, 1);
    const gd = dimensionGeometry(degenerate);
    expect(gd.lines).toHaveLength(0);
    expect(gd.arrowheads).toHaveLength(0);
    expect(gd.text).toBeNull();
  });
});

describe('dimensionGeometry — radius', () => {
  const dim = createRadiusDim([0, 0], [5, 0], 1, 1);
  const g = dimensionGeometry(dim, { arrow: 1, textGap: 1 });

  it('draws one leader from center past the rim with an arrowhead at the rim', () => {
    expect(g.lines).toHaveLength(1);
    expect(g.lines[0][0]).toEqual([0, 0]); // center
    expect(g.lines[0][1][0]).toBeCloseTo(6); // rim 5 + offset 1
    expect(g.arrowheads).toHaveLength(1);
    // Arrowhead tip seats on the arc (the rim point).
    expect(g.arrowheads[0][0]).toEqual([5, 0]);
  });

  it('labels the radius with an R prefix', () => {
    expect(g.text.label).toBe('R5');
  });
});

describe('dimensionBounds', () => {
  it('covers the whole dimension geometry', () => {
    const dim = createLinearDim([0, 0], [10, 0], 3, 1);
    const b = dimensionBounds(dim, { arrow: 1, ext: 0.5, gap: 0, textGap: 1 });
    expect(b.minX).toBeLessThanOrEqual(0);
    expect(b.maxX).toBeGreaterThanOrEqual(10);
    expect(b.maxY).toBeGreaterThanOrEqual(3);
  });

  it('returns null for empty geometry', () => {
    const dim = createLinearDim([1, 1], [1, 1], 0, 1);
    expect(dimensionBounds(dim)).toBeNull();
  });
});
