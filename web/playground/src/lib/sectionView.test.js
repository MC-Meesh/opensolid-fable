import { describe, expect, it } from 'vitest';
import {
  SECTION_AXES,
  axisComponent,
  clipPlaneParams,
  defaultSection,
  handlePosition,
  offsetRange,
  reseatOffset,
  sectionBounds,
} from './sectionView.js';

// A 2x4x6 box centered at (1, 2, 3): min (0,0,0), max (2,4,6).
const BOX = new Float32Array([
  0, 0, 0,
  2, 4, 6,
  1, 2, 3,
]);

describe('axisComponent', () => {
  it('maps axis letters to vector indices', () => {
    expect(axisComponent('X')).toBe(0);
    expect(axisComponent('Y')).toBe(1);
    expect(axisComponent('Z')).toBe(2);
  });
});

describe('sectionBounds', () => {
  it('computes min/max/center/radius from a positions array', () => {
    const b = sectionBounds(BOX);
    expect(b.min).toEqual([0, 0, 0]);
    expect(b.max).toEqual([2, 4, 6]);
    expect(b.center).toEqual([1, 2, 3]);
    expect(b.radius).toBeCloseTo(Math.hypot(2, 4, 6) / 2, 9);
  });

  it('falls back to a unit box for empty input', () => {
    const b = sectionBounds(new Float32Array(0));
    expect(b.center).toEqual([0, 0, 0]);
    expect(b.radius).toBe(0.5);
    // Same fallback for null / too-short arrays.
    expect(sectionBounds(null).radius).toBe(0.5);
  });
});

describe('offsetRange', () => {
  it('spans the axis extent with a little padding on each end', () => {
    const b = sectionBounds(BOX);
    const r = offsetRange(b, 'Y');
    expect(r.min).toBeLessThan(0);
    expect(r.max).toBeGreaterThan(4);
    // Symmetric padding around the [0, 4] extent.
    expect(r.min + r.max).toBeCloseTo(4, 9);
  });
});

describe('defaultSection', () => {
  it('cuts across X through the model center, unflipped', () => {
    const b = sectionBounds(BOX);
    expect(defaultSection(b)).toEqual({ axis: 'X', offset: 1, flip: false });
  });
});

describe('reseatOffset', () => {
  it('recenters the offset on the new axis', () => {
    const b = sectionBounds(BOX);
    const next = reseatOffset({ axis: 'Z', offset: 0, flip: true }, b);
    expect(next).toEqual({ axis: 'Z', offset: 3, flip: true });
  });
});

describe('clipPlaneParams', () => {
  // three.js keeps the half-space where normal·p + constant >= 0.
  const keeps = ({ normal, constant }, p) =>
    normal[0] * p[0] + normal[1] * p[1] + normal[2] * p[2] + constant >= 0;

  it('keeps the coord <= offset half by default', () => {
    const plane = clipPlaneParams({ axis: 'X', offset: 1, flip: false });
    expect(keeps(plane, [0.5, 9, 9])).toBe(true); // x < 1 kept
    expect(keeps(plane, [1.5, 9, 9])).toBe(false); // x > 1 clipped
  });

  it('flip keeps the opposite half', () => {
    const plane = clipPlaneParams({ axis: 'X', offset: 1, flip: true });
    expect(keeps(plane, [0.5, 9, 9])).toBe(false);
    expect(keeps(plane, [1.5, 9, 9])).toBe(true);
  });

  it('produces a unit normal along the chosen axis', () => {
    for (const axis of SECTION_AXES) {
      const { normal } = clipPlaneParams({ axis, offset: 0, flip: false });
      expect(Math.hypot(...normal)).toBeCloseTo(1, 9);
    }
    // (add 0 to normalize -0 -> 0 so the signed-zero doesn't trip toEqual)
    const norm = (v) => v.map((c) => c + 0);
    expect(norm(clipPlaneParams({ axis: 'Z', offset: 0, flip: false }).normal)).toEqual([0, 0, -1]);
    expect(norm(clipPlaneParams({ axis: 'Z', offset: 0, flip: true }).normal)).toEqual([0, 0, 1]);
  });

  it('the offset lies exactly on the plane (boundary point kept)', () => {
    const plane = clipPlaneParams({ axis: 'Y', offset: 2, flip: false });
    expect(keeps(plane, [9, 2, 9])).toBe(true);
  });
});

describe('handlePosition', () => {
  it('sits at the model center in-plane and at the offset along the axis', () => {
    const b = sectionBounds(BOX);
    expect(handlePosition(b, { axis: 'X', offset: 1.5 })).toEqual([1.5, 2, 3]);
    expect(handlePosition(b, { axis: 'Z', offset: 5 })).toEqual([1, 2, 5]);
  });
});
