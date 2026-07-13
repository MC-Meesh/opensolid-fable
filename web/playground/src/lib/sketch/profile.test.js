import { describe, expect, it } from 'vitest';
import {
  addArc,
  addCircle,
  addConstraint,
  addLine,
  addPoint,
  addRectangle,
  createSketch,
} from './model.js';
import {
  extractProfile,
  isFacePlane,
  planeAxisLabels,
  planeLabel,
  planeNormal,
  planeToWorld,
  profileTo3D,
  segmentEnd2D,
  segmentStart2D,
  worldToPlane,
} from './profile.js';

// A picked-face plane: orthonormal right-handed (u, v, normal) basis.
const FACE = {
  origin: [3, -4, 5],
  normal: [0, 0, 1],
  u: [0, 1, 0],
  v: [-1, 0, 0],
  extent: 2,
};

function triangle() {
  const s = createSketch();
  const a = addPoint(s, 0, 0);
  const b = addPoint(s, 2, 0);
  const c = addPoint(s, 0, 2);
  addLine(s, a, b);
  addLine(s, b, c);
  addLine(s, c, a);
  return s;
}

/** Max endpoint gap between consecutive segments. */
function maxGap(profile) {
  let worst = 0;
  const segs = profile.segments;
  for (let i = 0; i < segs.length; i++) {
    const end = segmentEnd2D(segs[i]);
    const start = segmentStart2D(segs[(i + 1) % segs.length]);
    worst = Math.max(worst, Math.hypot(end[0] - start[0], end[1] - start[1]));
  }
  return worst;
}

describe('extractProfile', () => {
  it('rejects an empty sketch', () => {
    expect(extractProfile(createSketch()).closed).toBe(false);
  });

  it('extracts a CCW triangle as three connected lines', () => {
    const profile = extractProfile(triangle(), 'XY');
    expect(profile.closed).toBe(true);
    expect(profile.plane).toBe('XY');
    expect(profile.segments).toHaveLength(3);
    expect(profile.segments.every((seg) => seg.kind === 'line')).toBe(true);
    expect(maxGap(profile)).toBeLessThan(1e-9);
  });

  it('reverses a clockwise loop to counterclockwise output', () => {
    const s = createSketch();
    const a = addPoint(s, 0, 0);
    const b = addPoint(s, 0, 2); // drawn CW: up, right-down, back
    const c = addPoint(s, 2, 0);
    addLine(s, a, b);
    addLine(s, b, c);
    addLine(s, c, a);
    const profile = extractProfile(s);
    expect(profile.closed).toBe(true);
    // Signed area of output vertices must be positive (CCW).
    const verts = profile.segments.map((seg) => segmentStart2D(seg));
    let area = 0;
    for (let i = 0; i < verts.length; i++) {
      const [x1, y1] = verts[i];
      const [x2, y2] = verts[(i + 1) % verts.length];
      area += x1 * y2 - x2 * y1;
    }
    expect(area).toBeGreaterThan(0);
    expect(maxGap(profile)).toBeLessThan(1e-9);
  });

  it('extracts a rectangle from the rectangle helper', () => {
    const s = createSketch();
    addRectangle(s, 0, 0, 3, 2);
    const profile = extractProfile(s);
    expect(profile.closed).toBe(true);
    expect(profile.segments).toHaveLength(4);
  });

  it('handles arcs: semicircle + diameter line', () => {
    const s = createSketch();
    const center = addPoint(s, 0, 0);
    const a = addPoint(s, 1, 0);
    const b = addPoint(s, -1, 0);
    addArc(s, center, a, b, true); // upper semicircle from (1,0) to (-1,0)
    addLine(s, b, a);
    const profile = extractProfile(s);
    expect(profile.closed).toBe(true);
    const arc = profile.segments.find((seg) => seg.kind === 'arc');
    expect(arc.radius).toBeCloseTo(1, 12);
    expect(maxGap(profile)).toBeLessThan(1e-9);
  });

  it('emits a lone circle as two counterclockwise semicircles', () => {
    const s = createSketch();
    const c = addPoint(s, 1, 2);
    addCircle(s, c, 1.5);
    const profile = extractProfile(s);
    expect(profile.closed).toBe(true);
    expect(profile.segments).toHaveLength(2);
    expect(profile.segments.every((seg) => seg.kind === 'arc')).toBe(true);
    expect(profile.segments[0].radius).toBe(1.5);
    expect(maxGap(profile)).toBeLessThan(1e-9);
  });

  it('rejects a circle mixed with other entities', () => {
    const s = triangle();
    const c = addPoint(s, 10, 10);
    addCircle(s, c, 1);
    const profile = extractProfile(s);
    expect(profile.closed).toBe(false);
    expect(profile.reason).toMatch(/circle/);
  });

  it('rejects an open chain', () => {
    const s = createSketch();
    const a = addPoint(s, 0, 0);
    const b = addPoint(s, 1, 0);
    const c = addPoint(s, 1, 1);
    addLine(s, a, b);
    addLine(s, b, c);
    const profile = extractProfile(s);
    expect(profile.closed).toBe(false);
    expect(profile.reason).toMatch(/open/);
  });

  it('rejects branching sketches', () => {
    const s = triangle();
    const first = Object.values(s.entities)[0];
    const d = addPoint(s, 5, 5);
    addLine(s, first.p1, d);
    addLine(s, d, first.p2); // second path between the same corners
    const profile = extractProfile(s);
    expect(profile.closed).toBe(false);
    expect(profile.reason).toMatch(/branch/);
  });

  it('rejects two disjoint loops', () => {
    const s = triangle();
    const a = addPoint(s, 10, 10);
    const b = addPoint(s, 12, 10);
    const c = addPoint(s, 10, 12);
    addLine(s, a, b);
    addLine(s, b, c);
    addLine(s, c, a);
    const profile = extractProfile(s);
    expect(profile.closed).toBe(false);
  });

  it('joins separate points through coincident constraints', () => {
    const s = createSketch();
    // Triangle drawn as three disconnected lines glued by constraints.
    const a1 = addPoint(s, 0, 0);
    const b1 = addPoint(s, 2, 0);
    const b2 = addPoint(s, 2, 0);
    const c1 = addPoint(s, 0, 2);
    const c2 = addPoint(s, 0, 2);
    const a2 = addPoint(s, 0, 0);
    addLine(s, a1, b1);
    addLine(s, b2, c1);
    addLine(s, c2, a2);
    addConstraint(s, { type: 'coincident', a: b1, b: b2 });
    addConstraint(s, { type: 'coincident', a: c1, b: c2 });
    addConstraint(s, { type: 'coincident', a: a2, b: a1 });
    const profile = extractProfile(s);
    expect(profile.closed).toBe(true);
    expect(profile.segments).toHaveLength(3);
  });

  it('rejects coincident-constrained points that are not touching', () => {
    const s = createSketch();
    const a1 = addPoint(s, 0, 0);
    const b1 = addPoint(s, 2, 0);
    const b2 = addPoint(s, 2.5, 0.5); // constrained but unsolved
    const c1 = addPoint(s, 0, 2);
    addLine(s, a1, b1);
    addLine(s, b2, c1);
    addLine(s, c1, a1);
    addConstraint(s, { type: 'coincident', a: b1, b: b2 });
    const profile = extractProfile(s);
    expect(profile.closed).toBe(false);
    expect(profile.reason).toMatch(/not touching/);
  });

  it('rejects a zero-area loop', () => {
    const s = createSketch();
    const a = addPoint(s, 0, 0);
    const b = addPoint(s, 2, 0);
    addLine(s, a, b);
    addLine(s, b, a);
    const profile = extractProfile(s);
    expect(profile.closed).toBe(false);
  });
});

describe('plane mapping', () => {
  it('planeToWorld maps sketch axes onto world planes', () => {
    expect(planeToWorld('XY', 1, 2)).toEqual([1, 2, 0]);
    // Top view: +Z runs down-screen, so sketch v maps to -z.
    expect(planeToWorld('XZ', 1, 2)).toEqual([1, 0, -2]);
    // Right view: +Z runs left on screen, so sketch u maps to -z, v to y.
    expect(planeToWorld('YZ', 1, 2)).toEqual([0, 2, -1]);
    expect(() => planeToWorld('AB', 0, 0)).toThrow(/unknown/);
  });

  it('worldToPlane inverts planeToWorld on every plane', () => {
    for (const plane of ['XY', 'XZ', 'YZ']) {
      expect(worldToPlane(plane, planeToWorld(plane, 1.5, -2.5))).toEqual([
        1.5, -2.5,
      ]);
    }
    expect(() => worldToPlane('AB', [0, 0, 0])).toThrow(/unknown/);
  });

  it('sketch bases are right-handed: e_u x e_v equals the plane normal', () => {
    for (const plane of ['XY', 'XZ', 'YZ']) {
      const eu = planeToWorld(plane, 1, 0);
      const ev = planeToWorld(plane, 0, 1);
      const cross = [
        eu[1] * ev[2] - eu[2] * ev[1],
        eu[2] * ev[0] - eu[0] * ev[2],
        eu[0] * ev[1] - eu[1] * ev[0],
      ];
      expect(cross).toEqual(planeNormal(plane));
    }
  });

  it('planeNormal matches each plane', () => {
    expect(planeNormal('XY')).toEqual([0, 0, 1]);
    expect(planeNormal('XZ')).toEqual([0, 1, 0]);
    expect(planeNormal('YZ')).toEqual([1, 0, 0]);
    expect(() => planeNormal('AB')).toThrow(/unknown/);
  });

  it('profileTo3D lifts segments onto the sketch plane', () => {
    const s = triangle();
    const profile = extractProfile(s, 'XZ');
    const lifted = profileTo3D(profile);
    expect(lifted.normal).toEqual([0, 1, 0]);
    for (const seg of lifted.segments) {
      expect(seg.start3[1]).toBe(0); // world Y is zero on the XZ plane
      expect(seg.end3[1]).toBe(0);
    }
  });

  it('profileTo3D passes through non-closed profiles', () => {
    const open = { closed: false, reason: 'nope' };
    expect(profileTo3D(open)).toBe(open);
  });

  it('face planes map through their own origin and basis', () => {
    expect(planeToWorld(FACE, 1, 2)).toEqual([3 - 2, -4 + 1, 5]);
    expect(worldToPlane(FACE, planeToWorld(FACE, 1.5, -2.5))).toEqual([1.5, -2.5]);
    expect(planeNormal(FACE)).toEqual([0, 0, 1]);
    // A defensive copy, not the plane's own array.
    expect(planeNormal(FACE)).not.toBe(FACE.normal);
  });

  it('classifies and labels face planes', () => {
    expect(isFacePlane(FACE)).toBe(true);
    expect(isFacePlane('XY')).toBe(false);
    expect(isFacePlane(null)).toBe(false);
    expect(planeLabel(FACE)).toBe('Face');
    expect(planeLabel('XZ')).toBe('XZ');
    expect(planeAxisLabels(FACE)).toEqual(['U', 'V']);
    expect(planeAxisLabels('YZ')).toEqual(['Z', 'Y']);
  });
});
