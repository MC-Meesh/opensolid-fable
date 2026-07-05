import { describe, expect, it } from 'vitest';
import { projectTriad } from './triad.js';

const IDENTITY = [0, 0, 0, 1];

function axisOf(result, name) {
  return result.find((e) => e.axis === name);
}

describe('projectTriad', () => {
  it('identity camera: X right, Y up, Z toward the viewer', () => {
    const result = projectTriad(IDENTITY);
    const x = axisOf(result, 'x');
    expect(x.x).toBeCloseTo(1, 9);
    expect(x.y).toBeCloseTo(0, 9);
    expect(x.depth).toBeCloseTo(0, 9);
    const y = axisOf(result, 'y');
    expect(y.x).toBeCloseTo(0, 9);
    expect(y.y).toBeCloseTo(1, 9);
    const z = axisOf(result, 'z');
    expect(z.depth).toBeCloseTo(1, 9);
    expect(z.x).toBeCloseTo(0, 9);
    expect(z.y).toBeCloseTo(0, 9);
  });

  it('right view (camera yawed +90° about Y): +X faces the viewer, +Z points screen-left', () => {
    const s = Math.SQRT1_2;
    const result = projectTriad([0, s, 0, s]);
    const x = axisOf(result, 'x');
    expect(x.depth).toBeCloseTo(1, 9);
    const z = axisOf(result, 'z');
    expect(z.x).toBeCloseTo(-1, 9);
    expect(z.depth).toBeCloseTo(0, 9);
    const y = axisOf(result, 'y');
    expect(y.y).toBeCloseTo(1, 9);
  });

  it('camera rolled 90° about Z: world X points screen-up', () => {
    const s = Math.SQRT1_2;
    const result = projectTriad([0, 0, s, s]);
    const x = axisOf(result, 'x');
    expect(x.x).toBeCloseTo(0, 9);
    expect(x.y).toBeCloseTo(-1, 9);
    const y = axisOf(result, 'y');
    expect(y.x).toBeCloseTo(1, 9);
    expect(y.y).toBeCloseTo(0, 9);
  });

  it('orders entries back-to-front by depth', () => {
    const s = Math.SQRT1_2;
    const result = projectTriad([0, s, 0, s]);
    for (let i = 1; i < result.length; i++) {
      expect(result[i].depth).toBeGreaterThanOrEqual(result[i - 1].depth);
    }
    expect(result[result.length - 1].axis).toBe('x');
  });

  it('projected directions stay unit length', () => {
    const raw = [0.183, 0.354, 0.067, 0.915]; // arbitrary rotation
    const len = Math.hypot(...raw);
    const q = raw.map((c) => c / len);
    for (const e of projectTriad(q)) {
      expect(Math.hypot(e.x, e.y, e.depth)).toBeCloseTo(1, 5);
    }
  });
});
