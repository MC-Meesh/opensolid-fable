import { describe, expect, it } from 'vitest';
import {
  angleOnArc,
  arcSweep,
  catmullRomHandles,
  circleCircleIntersections,
  cubicPoint,
  distToArc,
  distToCircle,
  distToCubic,
  distToEllipse,
  distToSegment,
  ellipseParam,
  ellipsePoint,
  lineCircleIntersections,
  lineLineIntersection,
  normalizeAngle,
  reflectPoint,
  sampleArc,
  sampleCubic,
  sampleEllipse,
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

  it('ellipsePoint places axis points and honors rotation', () => {
    // Axis-aligned ellipse rx=3, ry=1 at origin.
    expect(ellipsePoint(0, 0, 3, 1, 0, 0)).toEqual([3, 0]);
    const [x, y] = ellipsePoint(0, 0, 3, 1, 0, PI / 2);
    expect(x).toBeCloseTo(0, 12);
    expect(y).toBeCloseTo(1, 12);
    // Rotated 90°: the major axis (rx) now points up.
    const [rx, ry] = ellipsePoint(0, 0, 3, 1, PI / 2, 0);
    expect(rx).toBeCloseTo(0, 12);
    expect(ry).toBeCloseTo(3, 12);
  });

  it('ellipseParam inverts ellipsePoint', () => {
    const [cx, cy, a, b, rot] = [2, -1, 4, 1.5, 0.7];
    for (const t of [0.1, 1.2, 2.6, 4.0, 5.5]) {
      const [px, py] = ellipsePoint(cx, cy, a, b, rot, t);
      const back = ellipseParam(cx, cy, a, b, rot, px, py);
      expect(normalizeAngle(back)).toBeCloseTo(normalizeAngle(t), 9);
    }
  });

  it('sampleEllipse returns n+1 points on the outline', () => {
    const pts = sampleEllipse(0, 0, 2, 1, 0, 0, PI, 8);
    expect(pts).toHaveLength(9);
    expect(pts[0][0]).toBeCloseTo(2, 12);
    expect(pts[8][0]).toBeCloseTo(-2, 12);
    // Every sample lies on the ellipse: (x/2)^2 + y^2 = 1.
    for (const [x, y] of pts) {
      expect((x / 2) ** 2 + y ** 2).toBeCloseTo(1, 9);
    }
  });

  it('distToEllipse is ~0 on the outline and grows off it', () => {
    // Axis-aligned rx=3 ry=1; the point (3,0) sits on the outline.
    expect(distToEllipse(3, 0, 0, 0, 3, 1, 0)).toBeCloseTo(0, 2);
    expect(distToEllipse(0, 1, 0, 0, 3, 1, 0)).toBeCloseTo(0, 2);
    // The center is one minor radius from the nearest outline point.
    expect(distToEllipse(0, 0, 0, 0, 3, 1, 0)).toBeCloseTo(1, 2);
  });

  it('cubicPoint hits the endpoints and midpoint', () => {
    const p0 = [0, 0];
    const c1 = [0, 1];
    const c2 = [1, 1];
    const p1 = [1, 0];
    expect(cubicPoint(p0, c1, c2, p1, 0)).toEqual([0, 0]);
    expect(cubicPoint(p0, c1, c2, p1, 1)).toEqual([1, 0]);
    const mid = cubicPoint(p0, c1, c2, p1, 0.5);
    expect(mid[0]).toBeCloseTo(0.5, 12);
    expect(mid[1]).toBeCloseTo(0.75, 12);
  });

  it('sampleCubic returns n+1 points spanning the curve', () => {
    const pts = sampleCubic([0, 0], [1, 1], [2, 1], [3, 0], 6);
    expect(pts).toHaveLength(7);
    expect(pts[0]).toEqual([0, 0]);
    expect(pts[6]).toEqual([3, 0]);
  });

  it('distToCubic is ~0 on the curve', () => {
    const p0 = [0, 0];
    const c1 = [1, 2];
    const c2 = [3, 2];
    const p1 = [4, 0];
    const mid = cubicPoint(p0, c1, c2, p1, 0.5);
    expect(distToCubic(mid[0], mid[1], p0, c1, c2, p1)).toBeCloseTo(0, 3);
    expect(distToCubic(2, -5, p0, c1, c2, p1)).toBeGreaterThan(4);
  });

  it('catmullRomHandles yields a smooth interpolating segment', () => {
    // Collinear, evenly spaced points: handles land on the straight line.
    const { c1, c2 } = catmullRomHandles([0, 0], [1, 0], [2, 0], [3, 0]);
    expect(c1[0]).toBeCloseTo(4 / 3, 12);
    expect(c1[1]).toBeCloseTo(0, 12);
    expect(c2[0]).toBeCloseTo(5 / 3, 12);
    expect(c2[1]).toBeCloseTo(0, 12);
    // The Bézier through p1->p2 with these handles interpolates its endpoints.
    expect(cubicPoint([1, 0], c1, c2, [2, 0], 0)).toEqual([1, 0]);
    expect(cubicPoint([1, 0], c1, c2, [2, 0], 1)).toEqual([2, 0]);
  });
});
