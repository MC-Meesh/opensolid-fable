import { describe, expect, it } from 'vitest';
import {
  angledPlane,
  axisFromPlaneIntersection,
  axisFromPointDirection,
  axisFromTwoPoints,
  csysFromPlane,
  csysFromPointAndAxes,
  defaultReferenceName,
  isReferencePlane,
  midPlane,
  offsetPlane,
  pointAtAxisPlane,
  pointFromCoords,
  pointFromMidpoint,
  REFERENCE_META,
  resolvePlane,
  rotateAboutAxis,
} from './referenceGeometry.js';

const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const norm = (a) => Math.hypot(a[0], a[1], a[2]);
const unit = (a) => a.map((c) => c / norm(a));

function closeTo3(a, b, tol = 1e-9) {
  expect(a.length).toBe(3);
  for (let i = 0; i < 3; i++) expect(Math.abs(a[i] - b[i])).toBeLessThan(tol);
}

// A face-plane-shaped basis for the reference plane object path.
function faceLike(origin, u, v) {
  return { origin, u, v, normal: cross(u, v), extent: 2 };
}

describe('rotateAboutAxis', () => {
  it('rotates 90° about +Z: +X -> +Y', () => {
    closeTo3(rotateAboutAxis([1, 0, 0], [0, 0, 1], Math.PI / 2), [0, 1, 0]);
  });

  it('leaves a vector parallel to the axis unchanged', () => {
    closeTo3(rotateAboutAxis([0, 0, 3], [0, 0, 5], 1.234), [0, 0, 3]);
  });

  it('normalizes the axis (magnitude does not matter)', () => {
    const a = rotateAboutAxis([1, 0, 0], [0, 0, 1], Math.PI / 2);
    const b = rotateAboutAxis([1, 0, 0], [0, 0, 100], Math.PI / 2);
    closeTo3(a, b);
  });

  it('preserves vector length', () => {
    const r = rotateAboutAxis([1, 2, 3], [1, 1, 0], 0.7);
    expect(Math.abs(norm(r) - norm([1, 2, 3]))).toBeLessThan(1e-9);
  });
});

describe('resolvePlane', () => {
  it('resolves a named plane into the profile.js basis', () => {
    const xy = resolvePlane('XY');
    closeTo3(xy.origin, [0, 0, 0]);
    closeTo3(xy.normal, [0, 0, 1]);
    // u × v = normal for all named planes.
    closeTo3(cross(unit(xy.u), unit(xy.v)), [0, 0, 1]);
  });

  it('YZ keeps the profile.js convention (u = -z, v = y)', () => {
    const yz = resolvePlane('YZ');
    closeTo3(yz.u, [0, 0, -1]);
    closeTo3(yz.v, [0, 1, 0]);
    closeTo3(yz.normal, [1, 0, 0]);
  });

  it('passes a full plane object through untouched', () => {
    const p = faceLike([1, 2, 3], [1, 0, 0], [0, 1, 0]);
    const r = resolvePlane(p);
    closeTo3(r.origin, [1, 2, 3]);
    closeTo3(r.u, [1, 0, 0]);
    closeTo3(r.v, [0, 1, 0]);
    closeTo3(r.normal, [0, 0, 1]);
  });

  it('throws on a malformed plane object', () => {
    expect(() => resolvePlane({ origin: [0, 0, 0] })).toThrow(/origin, normal/);
  });
});

describe('offsetPlane', () => {
  it('shifts along +normal by the distance', () => {
    const p = offsetPlane('XY', 3);
    expect(p.kind).toBe('plane');
    expect(p.method).toBe('offset');
    closeTo3(p.origin, [0, 0, 3]);
    closeTo3(p.normal, [0, 0, 1]);
  });

  it('negative distance shifts along -normal', () => {
    closeTo3(offsetPlane('XY', -2).origin, [0, 0, -2]);
  });

  it('inherits the base (u, v) basis so the frame is preserved', () => {
    const p = offsetPlane('XZ', 5);
    const base = resolvePlane('XZ');
    closeTo3(p.u, base.u);
    closeTo3(p.v, base.v);
  });

  it('offsets from a face-plane object along its own normal', () => {
    const base = faceLike([0, 0, 1], [1, 0, 0], [0, 1, 0]); // normal +Z
    closeTo3(offsetPlane(base, 2).origin, [0, 0, 3]);
  });

  it('gets a default indicator extent', () => {
    expect(offsetPlane('XY', 1).extent).toBeGreaterThan(0);
  });
});

describe('angledPlane', () => {
  it('tilts about u by default, keeping u fixed and origin shared', () => {
    const p = angledPlane('XY', 90); // hinge u = +X
    closeTo3(p.origin, [0, 0, 0]);
    closeTo3(p.u, [1, 0, 0]);
    // normal rotates 90° about +X: +Z -> -Y
    closeTo3(p.normal, [0, -1, 0]);
  });

  it('tilts about v when hinge = v', () => {
    const p = angledPlane('XY', 90, 'v'); // hinge v = +Y
    closeTo3(p.v, [0, 1, 0]);
    // normal rotates 90° about +Y: +Z -> +X
    closeTo3(p.normal, [1, 0, 0]);
  });

  it('0° is a no-op orientation', () => {
    const p = angledPlane('XY', 0);
    const base = resolvePlane('XY');
    closeTo3(p.normal, unit(base.normal));
  });

  it('keeps u × v aligned with the normal (right-handed)', () => {
    const p = angledPlane('XY', 37);
    closeTo3(unit(cross(p.u, p.v)), unit(p.normal), 1e-9);
  });
});

describe('midPlane', () => {
  it('sits halfway between two parallel planes', () => {
    const p = midPlane('XY', offsetPlane('XY', 4));
    closeTo3(p.origin, [0, 0, 2]);
    closeTo3(p.normal, [0, 0, 1]);
  });

  it('works when the planes are given in the other order', () => {
    const p = midPlane(offsetPlane('XY', 4), 'XY');
    closeTo3(p.origin, [0, 0, 2]);
  });

  it('throws when the planes are not parallel', () => {
    expect(() => midPlane('XY', 'XZ')).toThrow(/parallel/);
  });
});

describe('axis constructors', () => {
  it('axisFromTwoPoints is directed p1 -> p2 with the separation length', () => {
    const a = axisFromTwoPoints([0, 0, 0], [0, 0, 5]);
    expect(a.kind).toBe('axis');
    expect(a.method).toBe('two-points');
    closeTo3(a.origin, [0, 0, 0]);
    closeTo3(a.direction, [0, 0, 1]);
    expect(a.length).toBeCloseTo(5);
  });

  it('axisFromTwoPoints throws on coincident points', () => {
    expect(() => axisFromTwoPoints([1, 1, 1], [1, 1, 1])).toThrow(/distinct/);
  });

  it('axisFromPointDirection normalizes the direction', () => {
    const a = axisFromPointDirection([1, 2, 3], [0, 0, 9]);
    closeTo3(a.origin, [1, 2, 3]);
    closeTo3(a.direction, [0, 0, 1]);
  });

  it('axisFromPlaneIntersection of XY and XZ is the X axis through origin', () => {
    const a = axisFromPlaneIntersection('XY', 'XZ');
    // The line is the world X axis (orientation is sign-arbitrary).
    expect(Math.abs(dot(unit(a.direction), [1, 0, 0]))).toBeCloseTo(1);
    // Closest point on the line to the origin is the origin itself.
    closeTo3(a.origin, [0, 0, 0]);
  });

  it('axisFromPlaneIntersection lands on both planes', () => {
    const a = axisFromPlaneIntersection(offsetPlane('XY', 2), 'YZ');
    const pa = resolvePlane(offsetPlane('XY', 2));
    const pb = resolvePlane('YZ');
    // origin satisfies both plane equations n·x = n·o.
    expect(dot(unit(pa.normal), sub(a.origin, pa.origin))).toBeCloseTo(0);
    expect(dot(unit(pb.normal), sub(a.origin, pb.origin))).toBeCloseTo(0);
  });

  it('axisFromPlaneIntersection throws on parallel planes', () => {
    expect(() => axisFromPlaneIntersection('XY', offsetPlane('XY', 1))).toThrow(
      /non-parallel/
    );
  });
});

describe('point constructors', () => {
  it('pointFromCoords copies the position', () => {
    const src = [1, 2, 3];
    const p = pointFromCoords(src);
    closeTo3(p.position, [1, 2, 3]);
    src[0] = 9;
    expect(p.position[0]).toBe(1); // defensive copy
  });

  it('pointFromMidpoint averages the two points', () => {
    closeTo3(pointFromMidpoint([0, 0, 0], [2, 4, 6]).position, [1, 2, 3]);
  });

  it('pointAtAxisPlane finds the pierce point', () => {
    const axis = axisFromPointDirection([1, 1, -3], [0, 0, 1]);
    const p = pointAtAxisPlane(axis, 'XY');
    closeTo3(p.position, [1, 1, 0]);
  });

  it('pointAtAxisPlane throws when the axis is parallel to the plane', () => {
    const axis = axisFromPointDirection([0, 0, 1], [1, 0, 0]); // in XY direction
    expect(() => pointAtAxisPlane(axis, 'XY')).toThrow(/parallel/);
  });
});

describe('coordinate systems', () => {
  it('csysFromPointAndAxes is orthonormal and right-handed', () => {
    const c = csysFromPointAndAxes([1, 0, 0], [2, 0, 0], [0, 3, 0]);
    closeTo3(c.origin, [1, 0, 0]);
    closeTo3(c.x, [1, 0, 0]);
    closeTo3(c.y, [0, 1, 0]);
    closeTo3(c.z, [0, 0, 1]);
    closeTo3(cross(c.x, c.y), c.z);
  });

  it('csysFromPointAndAxes re-orthogonalizes a skewed Y hint', () => {
    const c = csysFromPointAndAxes([0, 0, 0], [1, 0, 0], [1, 1, 0]);
    expect(dot(c.x, c.y)).toBeCloseTo(0);
    expect(norm(c.y)).toBeCloseTo(1);
  });

  it('csysFromPointAndAxes throws on parallel X and Y', () => {
    expect(() => csysFromPointAndAxes([0, 0, 0], [1, 0, 0], [2, 0, 0])).toThrow(
      /non-parallel/
    );
  });

  it('csysFromPlane maps X=u, Y=v, Z=normal', () => {
    const c = csysFromPlane('XY');
    closeTo3(c.origin, [0, 0, 0]);
    closeTo3(c.z, [0, 0, 1]);
    closeTo3(cross(c.x, c.y), c.z);
  });
});

describe('defaultReferenceName', () => {
  it('numbers from 1 for an empty collection', () => {
    expect(defaultReferenceName('plane', [])).toBe('Plane1');
    expect(defaultReferenceName('axis', [])).toBe('Axis1');
    expect(defaultReferenceName('point', [])).toBe('Point1');
    expect(defaultReferenceName('csys', [])).toBe('CSys1');
  });

  it('continues from the highest existing ordinal (no reuse after delete)', () => {
    expect(defaultReferenceName('plane', ['Plane1', 'Plane3'])).toBe('Plane4');
  });

  it('ignores names of other kinds and custom renames', () => {
    expect(defaultReferenceName('plane', ['Axis5', 'Datum', 'Plane2'])).toBe(
      'Plane3'
    );
  });
});

describe('REFERENCE_META & isReferencePlane', () => {
  it('has display metadata for every kind', () => {
    for (const kind of ['plane', 'axis', 'point', 'csys']) {
      expect(REFERENCE_META[kind]).toBeTruthy();
      expect(typeof REFERENCE_META[kind].type).toBe('string');
    }
  });

  it('isReferencePlane recognizes reference-plane entities only', () => {
    expect(isReferencePlane(offsetPlane('XY', 1))).toBe(true);
    expect(isReferencePlane('XY')).toBe(false);
    expect(isReferencePlane(faceLike([0, 0, 0], [1, 0, 0], [0, 1, 0]))).toBe(
      false
    );
    expect(isReferencePlane(null)).toBe(false);
    expect(isReferencePlane(axisFromTwoPoints([0, 0, 0], [1, 0, 0]))).toBe(false);
  });
});
