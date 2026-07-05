import { describe, expect, it } from 'vitest';
import {
  angleOnArc,
  arcSweep,
  distToArc,
  distToCircle,
  distToSegment,
  normalizeAngle,
  sampleArc,
  signedArea,
} from './geom.js';

const PI = Math.PI;

describe('geom', () => {
  it('normalizeAngle wraps into [0, 2π)', () => {
    expect(normalizeAngle(0)).toBe(0);
    expect(normalizeAngle(3 * PI)).toBeCloseTo(PI, 12);
    expect(normalizeAngle(-PI / 2)).toBeCloseTo((3 * PI) / 2, 12);
    expect(normalizeAngle(2 * PI)).toBeCloseTo(0, 12);
  });

  it('arcSweep measures traversal in either direction', () => {
    expect(arcSweep(0, PI / 2, true)).toBeCloseTo(PI / 2, 12);
    expect(arcSweep(0, PI / 2, false)).toBeCloseTo((3 * PI) / 2, 12);
    expect(arcSweep(PI, 0, true)).toBeCloseTo(PI, 12);
    // Coincident endpoints mean a full turn, not zero.
    expect(arcSweep(1, 1, true)).toBeCloseTo(2 * PI, 12);
  });

  it('angleOnArc detects containment respecting direction', () => {
    expect(angleOnArc(PI / 4, 0, PI / 2, true)).toBe(true);
    expect(angleOnArc(-PI / 4, 0, PI / 2, true)).toBe(false);
    expect(angleOnArc(-PI / 4, 0, PI / 2, false)).toBe(true);
  });

  it('distToSegment clamps to endpoints', () => {
    expect(distToSegment(1, 1, 0, 0, 2, 0)).toBeCloseTo(1, 12);
    expect(distToSegment(-3, 4, 0, 0, 2, 0)).toBeCloseTo(5, 12);
    expect(distToSegment(5, 4, 2, 0, 2, 0)).toBeCloseTo(5, 12); // degenerate
  });

  it('distToCircle measures to the outline', () => {
    expect(distToCircle(3, 0, 0, 0, 1)).toBeCloseTo(2, 12);
    expect(distToCircle(0, 0, 0, 0, 1)).toBeCloseTo(1, 12);
  });

  it('distToArc uses outline on-span and endpoints off-span', () => {
    // Quarter arc from 0 to π/2, radius 1.
    const d = (px, py) => distToArc(px, py, 0, 0, 1, 0, PI / 2, true);
    expect(d(2 * Math.SQRT1_2, 2 * Math.SQRT1_2)).toBeCloseTo(1, 9);
    // Point near angle -π/2 is off-span: distance to endpoint (1, 0).
    expect(d(0, -1)).toBeCloseTo(Math.SQRT2, 9);
  });

  it('sampleArc includes both endpoints and respects direction', () => {
    const pts = sampleArc(0, 0, 1, 0, PI / 2, true, 4);
    expect(pts).toHaveLength(5);
    expect(pts[0][0]).toBeCloseTo(1, 12);
    expect(pts[4][1]).toBeCloseTo(1, 12);
    const cw = sampleArc(0, 0, 1, 0, PI / 2, false, 4);
    expect(cw[4][1]).toBeCloseTo(-1, 12);
  });

  it('signedArea is positive for counterclockwise polygons', () => {
    const ccw = [
      [0, 0],
      [2, 0],
      [2, 1],
      [0, 1],
    ];
    expect(signedArea(ccw)).toBeCloseTo(2, 12);
    expect(signedArea(ccw.slice().reverse())).toBeCloseTo(-2, 12);
  });
});
