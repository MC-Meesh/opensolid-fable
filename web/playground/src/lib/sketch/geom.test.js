import { describe, expect, it } from 'vitest';
import {
  angleOnArc,
  arcSweep,
  circleCircleIntersections,
  distToArc,
  distToCircle,
  distToSegment,
  lineCircleIntersections,
  lineLineIntersection,
  normalizeAngle,
  reflectPoint,
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

  it('reflectPoint mirrors across an axis and through a degenerate point', () => {
    // Across the X axis (y = 0): (x, y) -> (x, -y).
    expect(reflectPoint(3, 2, 0, 0, 1, 0)).toEqual([3, -2]);
    // Across the Y axis (x = 0): (x, y) -> (-x, y).
    expect(reflectPoint(3, 2, 0, 0, 0, 1)).toEqual([-3, 2]);
    // Across the 45° line y = x: (x, y) -> (y, x).
    const [rx, ry] = reflectPoint(3, 1, 0, 0, 1, 1);
    expect(rx).toBeCloseTo(1, 12);
    expect(ry).toBeCloseTo(3, 12);
    // Degenerate axis reflects through the point.
    expect(reflectPoint(5, 4, 1, 1, 1, 1)).toEqual([-3, -2]);
  });

  it('lineLineIntersection crosses and reports parameters', () => {
    // x-axis segment (0,0)-(2,0) vs vertical (1,-1)-(1,1): cross at (1,0).
    const hit = lineLineIntersection([0, 0], [2, 0], [1, -1], [1, 1]);
    expect(hit.x).toBeCloseTo(1, 12);
    expect(hit.y).toBeCloseTo(0, 12);
    expect(hit.t).toBeCloseTo(0.5, 12); // halfway along A
    expect(hit.u).toBeCloseTo(0.5, 12); // halfway along B
  });

  it('lineLineIntersection returns null for parallel lines', () => {
    expect(lineLineIntersection([0, 0], [1, 0], [0, 1], [1, 1])).toBeNull();
    expect(lineLineIntersection([0, 0], [0, 0], [0, 1], [1, 1])).toBeNull();
  });

  it('lineCircleIntersections finds two, one (tangent), or none', () => {
    // Horizontal line y=0 through unit circle: (-1,0) and (1,0), t-ordered.
    const two = lineCircleIntersections([-2, 0], [2, 0], [0, 0], 1);
    expect(two).toHaveLength(2);
    expect(two[0].x).toBeCloseTo(-1, 12);
    expect(two[1].x).toBeCloseTo(1, 12);
    // Tangent line y=1.
    const one = lineCircleIntersections([-2, 1], [2, 1], [0, 0], 1);
    expect(one).toHaveLength(1);
    expect(one[0].y).toBeCloseTo(1, 9);
    // Miss.
    expect(lineCircleIntersections([-2, 2], [2, 2], [0, 0], 1)).toHaveLength(0);
  });

  it('circleCircleIntersections handles crossing, tangent, and disjoint', () => {
    // Unit circles at (0,0) and (1,0) cross at x=0.5, y=±√3/2.
    const pts = circleCircleIntersections([0, 0], 1, [1, 0], 1);
    expect(pts).toHaveLength(2);
    expect(pts[0].x).toBeCloseTo(0.5, 12);
    expect(Math.abs(pts[0].y)).toBeCloseTo(Math.sqrt(3) / 2, 12);
    // Externally tangent: (0,0) r1 and (2,0) r1 touch at (1,0).
    const tan = circleCircleIntersections([0, 0], 1, [2, 0], 1);
    expect(tan).toHaveLength(1);
    expect(tan[0].x).toBeCloseTo(1, 9);
    // Disjoint and concentric.
    expect(circleCircleIntersections([0, 0], 1, [5, 0], 1)).toHaveLength(0);
    expect(circleCircleIntersections([0, 0], 1, [0, 0], 2)).toHaveLength(0);
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
