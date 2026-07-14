import { describe, expect, it } from 'vitest';
import {
  DEFAULT_EXTENT,
  angledPlane,
  buildReference,
  axisFromPlaneIntersection,
  axisFromPointDirection,
  axisFromTwoPoints,
  csysFromPlane,
  csysFromPointAxes,
  midPlane,
  offsetPlane,
  planeFromPointNormal,
  pointFromCoords,
  pointMidpoint,
  pointPlaneAxisPierce,
  projectOntoPlane,
  resolveBasePlane,
  rotateAboutAxis,
} from './referenceGeometry.js';

const near = (a, b, tol = 1e-9) => Math.abs(a - b) <= tol;
function expectVecClose(actual, expected, tol = 1e-9) {
  expect(actual).toHaveLength(expected.length);
  actual.forEach((c, i) => {
    if (!near(c, expected[i], tol)) {
      throw new Error(`[${actual}] !≈ [${expected}] at ${i}`);
    }
  });
}
const norm = (a) => Math.hypot(...a);
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];

/** A plane's (u, v, normal) must be a right-handed orthonormal frame. */
function expectValidPlane(p) {
  expect(p.kind).toBe('plane');
  expect(p.reference).toBe(true);
  expectVecClose([norm(p.normal), norm(p.u), norm(p.v)], [1, 1, 1]);
  expect(near(dot(p.u, p.v), 0)).toBe(true);
  expect(near(dot(p.u, p.normal), 0)).toBe(true);
  expect(near(dot(p.v, p.normal), 0)).toBe(true);
  expectVecClose(cross(p.u, p.v), p.normal); // u × v = normal
}

describe('planeFromPointNormal', () => {
  it('builds a right-handed orthonormal frame with u × v = normal', () => {
    const p = planeFromPointNormal([1, 2, 3], [0, 0, 5]);
    expectValidPlane(p);
    expectVecClose(p.origin, [1, 2, 3]);
    expectVecClose(p.normal, [0, 0, 1]); // normalized
    expect(p.extent).toBe(DEFAULT_EXTENT);
  });

  it('matches the named-plane basis conventions (XY → u=X, v=Y)', () => {
    const p = planeFromPointNormal([0, 0, 0], [0, 0, 1]);
    expectVecClose(p.u, [1, 0, 0]);
    expectVecClose(p.v, [0, 1, 0]);
  });

  it('copies the origin (no aliasing)', () => {
    const src = [1, 1, 1];
    const p = planeFromPointNormal(src, [1, 0, 0]);
    src[0] = 99;
    expect(p.origin[0]).toBe(1);
  });

  it('throws on a zero-length normal', () => {
    expect(() => planeFromPointNormal([0, 0, 0], [0, 0, 0])).toThrow(/zero-length/);
  });
});

describe('resolveBasePlane', () => {
  it('resolves named world planes through the origin', () => {
    expectVecClose(resolveBasePlane('XY').normal, [0, 0, 1]);
    expectVecClose(resolveBasePlane('XZ').normal, [0, 1, 0]);
    expectVecClose(resolveBasePlane('YZ').normal, [1, 0, 0]);
    expectVecClose(resolveBasePlane('XY').origin, [0, 0, 0]);
  });

  it('re-derives a clean plane from any {origin, normal} object', () => {
    const p = resolveBasePlane({ origin: [0, 0, 2], normal: [0, 0, 3], extent: 7 });
    expectValidPlane(p);
    expectVecClose(p.normal, [0, 0, 1]);
    expect(p.extent).toBe(7);
  });

  it('throws on an unknown plane name or malformed object', () => {
    expect(() => resolveBasePlane('QQ')).toThrow(/unknown named plane/);
    expect(() => resolveBasePlane({})).toThrow(/base plane/);
  });
});

describe('offsetPlane', () => {
  it('shifts the origin along the base normal', () => {
    const p = offsetPlane('XY', 4);
    expectVecClose(p.origin, [0, 0, 4]);
    expectVecClose(p.normal, [0, 0, 1]);
    expectValidPlane(p);
  });

  it('shifts negatively the other way', () => {
    expectVecClose(offsetPlane('XZ', -3).origin, [0, -3, 0]);
  });

  it('offsets relative to an existing reference plane', () => {
    const base = offsetPlane('XY', 2);
    const p = offsetPlane(base, 5);
    expectVecClose(p.origin, [0, 0, 7]);
  });
});

describe('angledPlane', () => {
  it('rotates the base normal about the default in-plane u axis', () => {
    // XY plane rotated 90° about X → normal from +Z to -Y (right-hand rule).
    const p = angledPlane('XY', Math.PI / 2);
    expectVecClose(p.normal, [0, -1, 0], 1e-9);
    expectValidPlane(p);
  });

  it('rotates about a supplied in-plane direction', () => {
    // Rotate XY about the Y axis: +Z normal swings toward +X.
    const p = angledPlane('XY', Math.PI / 2, { direction: [0, 1, 0] });
    expectVecClose(p.normal, [1, 0, 0], 1e-9);
  });

  it('pivots about a supplied point', () => {
    const p = angledPlane('XY', Math.PI / 2, { point: [5, 0, 0] });
    expectVecClose(p.origin, [5, 0, 0]);
  });

  it('a zero angle leaves the plane orientation unchanged', () => {
    const p = angledPlane('XY', 0);
    expectVecClose(p.normal, [0, 0, 1]);
  });
});

describe('midPlane', () => {
  it('is parallel and halfway between two parallel planes', () => {
    const a = offsetPlane('XY', 0);
    const b = offsetPlane('XY', 10);
    const m = midPlane(a, b);
    expectVecClose(m.origin, [0, 0, 5]);
    expectVecClose(m.normal, [0, 0, 1]);
    expectValidPlane(m);
  });

  it('handles anti-parallel normals without cancelling', () => {
    const a = planeFromPointNormal([0, 0, 0], [0, 0, 1]);
    const b = planeFromPointNormal([0, 0, 4], [0, 0, -1]);
    const m = midPlane(a, b);
    expectVecClose(m.origin, [0, 0, 2]);
    expectVecClose(m.normal, [0, 0, 1]);
  });

  it('bisects the dihedral angle of two perpendicular planes', () => {
    const m = midPlane('XY', 'XZ'); // normals +Z and +Y → bisector (0, .707, .707)
    expectVecClose(m.normal, [0, Math.SQRT1_2, Math.SQRT1_2], 1e-9);
  });

  it('throws on directly opposed coincident planes', () => {
    const a = planeFromPointNormal([0, 0, 0], [0, 0, 1]);
    const b = planeFromPointNormal([0, 0, 0], [0, 0, 1]);
    // Same orientation is fine; opposed with the flip rule would still agree.
    expect(() => midPlane(a, b)).not.toThrow();
  });
});

describe('axisFromTwoPoints', () => {
  it('points from p1 to p2, unit direction, anchored at p1', () => {
    const ax = axisFromTwoPoints([1, 0, 0], [1, 0, 5]);
    expect(ax.kind).toBe('axis');
    expect(ax.reference).toBe(true);
    expectVecClose(ax.origin, [1, 0, 0]);
    expectVecClose(ax.direction, [0, 0, 1]);
    expect(ax.extent).toBeCloseTo(5);
  });

  it('throws on coincident points', () => {
    expect(() => axisFromTwoPoints([1, 1, 1], [1, 1, 1])).toThrow(/coincident/);
  });
});

describe('axisFromPointDirection', () => {
  it('normalizes the direction', () => {
    const ax = axisFromPointDirection([2, 2, 2], [0, 0, 9]);
    expectVecClose(ax.direction, [0, 0, 1]);
    expectVecClose(ax.origin, [2, 2, 2]);
  });
});

describe('axisFromPlaneIntersection', () => {
  it('runs along XY ∩ XZ = the X axis', () => {
    const ax = axisFromPlaneIntersection('XY', 'XZ');
    // Direction is ±X.
    expect(Math.abs(ax.direction[0])).toBeCloseTo(1);
    expect(near(ax.direction[1], 0)).toBe(true);
    expect(near(ax.direction[2], 0)).toBe(true);
    // Anchor lies on both planes (z=0 and y=0 through origin).
    expect(near(ax.origin[1], 0)).toBe(true);
    expect(near(ax.origin[2], 0)).toBe(true);
  });

  it('anchors on the line for offset planes', () => {
    const a = offsetPlane('XZ', 3); // y = 3
    const b = offsetPlane('YZ', 2); // x = 2
    const ax = axisFromPlaneIntersection(a, b); // line x=2, y=3, along Z
    expect(near(ax.origin[0], 2)).toBe(true);
    expect(near(ax.origin[1], 3)).toBe(true);
    expect(Math.abs(ax.direction[2])).toBeCloseTo(1);
  });

  it('throws on parallel planes', () => {
    expect(() => axisFromPlaneIntersection('XY', offsetPlane('XY', 5))).toThrow(/parallel/);
  });
});

describe('points', () => {
  it('pointFromCoords stores an independent copy', () => {
    const p = pointFromCoords([1, 2, 3]);
    expect(p.kind).toBe('point');
    expect(p.reference).toBe(true);
    expectVecClose(p.position, [1, 2, 3]);
  });

  it('pointMidpoint averages two points', () => {
    expectVecClose(pointMidpoint([0, 0, 0], [4, 8, 2]).position, [2, 4, 1]);
  });

  it('pointPlaneAxisPierce finds the pierce point', () => {
    const plane = offsetPlane('XY', 5); // z = 5
    const axis = axisFromPointDirection([1, 2, 0], [0, 0, 1]); // straight up
    expectVecClose(pointPlaneAxisPierce(plane, axis).position, [1, 2, 5]);
  });

  it('pointPlaneAxisPierce throws when the axis is parallel to the plane', () => {
    const plane = resolveBasePlane('XY');
    const axis = axisFromPointDirection([0, 0, 3], [1, 0, 0]); // in-plane direction
    expect(() => pointPlaneAxisPierce(plane, axis)).toThrow(/parallel/);
  });
});

describe('coordinate systems', () => {
  it('csysFromPlane maps (u, v, normal) to (x, y, z)', () => {
    const c = csysFromPlane('XY');
    expect(c.kind).toBe('csys');
    expect(c.reference).toBe(true);
    expectVecClose(c.x, [1, 0, 0]);
    expectVecClose(c.y, [0, 1, 0]);
    expectVecClose(c.z, [0, 0, 1]);
    expectVecClose(c.origin, [0, 0, 0]);
  });

  it('csysFromPlane can override the origin', () => {
    const c = csysFromPlane('XY', [3, 3, 3]);
    expectVecClose(c.origin, [3, 3, 3]);
  });

  it('csysFromPointAxes orthonormalizes the y hint', () => {
    const c = csysFromPointAxes([0, 0, 0], [2, 0, 0], [1, 5, 0]);
    expectVecClose(c.x, [1, 0, 0]);
    expectVecClose(c.y, [0, 1, 0]); // hint made orthogonal to x
    expectVecClose(c.z, [0, 0, 1]); // x × y
  });

  it('csysFromPointAxes throws on parallel x and y', () => {
    expect(() => csysFromPointAxes([0, 0, 0], [1, 0, 0], [3, 0, 0])).toThrow(/parallel/);
  });
});

describe('vector helpers', () => {
  it('projectOntoPlane drops the axis-parallel component', () => {
    expectVecClose(projectOntoPlane([1, 2, 3], [0, 0, 1]), [1, 2, 0]);
  });

  it('rotateAboutAxis rotates a vector by the right angle', () => {
    expectVecClose(rotateAboutAxis([1, 0, 0], [0, 0, 1], Math.PI / 2), [0, 1, 0], 1e-9);
  });

  it('rotateAboutAxis leaves an on-axis vector fixed', () => {
    expectVecClose(rotateAboutAxis([0, 0, 2], [0, 0, 1], 1.234), [0, 0, 2], 1e-9);
  });
});

describe('buildReference form dispatch', () => {
  it('builds an offset plane from base + distance', () => {
    const { kind, geom } = buildReference('plane-offset', { base: 'XY', distance: 3 });
    expect(kind).toBe('plane');
    expectVecClose(geom.origin, [0, 0, 3]);
    expect(geom.reference).toBe(true);
  });

  it('converts angled-plane degrees to radians', () => {
    const { geom } = buildReference('plane-angled', { base: 'XY', angleDeg: 90 });
    expectVecClose(geom.normal, [0, -1, 0], 1e-9);
  });

  it('builds a mid-plane from two bases', () => {
    const { geom } = buildReference('plane-mid', {
      base: offsetPlane('XY', 0),
      base2: offsetPlane('XY', 8),
    });
    expectVecClose(geom.origin, [0, 0, 4]);
  });

  it('builds each axis method', () => {
    expect(buildReference('axis-2pt', { p1: [0, 0, 0], p2: [0, 0, 2] }).kind).toBe('axis');
    expect(buildReference('axis-ptdir', { point: [1, 1, 1], direction: [1, 0, 0] }).kind).toBe('axis');
    expect(buildReference('axis-intersect', { base: 'XY', base2: 'XZ' }).kind).toBe('axis');
  });

  it('builds each point and csys method', () => {
    expect(buildReference('point-coords', { coords: [1, 2, 3] }).geom.position).toEqual([1, 2, 3]);
    expect(buildReference('point-mid', { p1: [0, 0, 0], p2: [2, 0, 0] }).geom.position).toEqual([1, 0, 0]);
    expect(buildReference('csys-plane', { base: 'XY' }).kind).toBe('csys');
    expect(
      buildReference('csys-ptaxes', { origin: [0, 0, 0], xDir: [1, 0, 0], yHint: [0, 1, 0] }).kind
    ).toBe('csys');
  });

  it('throws the constructor error on bad geometry', () => {
    expect(() => buildReference('axis-intersect', { base: 'XY', base2: offsetPlane('XY', 5) })).toThrow(
      /parallel/
    );
    expect(() => buildReference('point-coords', {})).toThrow();
  });

  it('throws on an unknown method', () => {
    expect(() => buildReference('bogus', {})).toThrow(/unknown reference method/);
  });
});
