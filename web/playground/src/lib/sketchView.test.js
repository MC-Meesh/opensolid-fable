import { describe, expect, it } from 'vitest';
import {
  SKETCH_VIEW_POSES,
  cameraFromSketchView,
  easeInOutCubic,
  gridLevels,
  orthoHalfExtents,
  planeIndicatorSize,
  sketchDistForPxPerUnit,
  sketchPxPerUnit,
  sketchScreenToWorld,
  sketchViewFromCamera,
  sketchViewPose,
  sketchWorldToScreen,
} from './sketchView.js';
import { addRectangle, createSketch } from './sketch/model.js';
import {
  extractProfile,
  planeToWorld,
  segmentEnd2D,
  segmentStart2D,
} from './sketch/profile.js';

describe('sketchViewPose', () => {
  it('places the camera along +Z for the XY plane (front view)', () => {
    const pose = sketchViewPose('XY', [0, 0, 0], 5);
    expect(pose.position).toEqual([0, 0, 5]);
    expect(pose.target).toEqual([0, 0, 0]);
    expect(pose.up).toEqual([0, 1, 0]);
  });

  it('places the camera along +Y for the XZ plane (top view)', () => {
    const pose = sketchViewPose('XZ', [0, 0, 0], 8);
    expect(pose.position).toEqual([0, 8, 0]);
    expect(pose.up).toEqual([0, 0, -1]);
  });

  it('places the camera along +X for the YZ plane (right view)', () => {
    const pose = sketchViewPose('YZ', [0, 0, 0], 3);
    expect(pose.position).toEqual([3, 0, 0]);
    expect(pose.up).toEqual([0, 1, 0]);
  });

  it('projects the orbit target onto the sketch plane', () => {
    const pose = sketchViewPose('XY', [2, 3, 7], 5);
    expect(pose.target).toEqual([2, 3, 0]);
    expect(pose.position).toEqual([2, 3, 5]);
  });

  it('keeps in-plane target components for the XZ plane', () => {
    const pose = sketchViewPose('XZ', [2, 3, 7], 5);
    expect(pose.target).toEqual([2, 0, 7]);
    expect(pose.position).toEqual([2, 5, 7]);
  });

  it('returns null for an unknown plane', () => {
    expect(sketchViewPose('AB', [0, 0, 0], 5)).toBeNull();
  });

  it('returns a fresh up array (not a shared reference)', () => {
    const a = sketchViewPose('XY', [0, 0, 0], 1);
    a.up[0] = 99;
    expect(SKETCH_VIEW_POSES.XY.up).toEqual([0, 1, 0]);
  });

  it('camera direction is orthogonal to the plane for every plane', () => {
    for (const plane of Object.keys(SKETCH_VIEW_POSES)) {
      const pose = sketchViewPose(plane, [1.5, -2, 0.5], 4);
      const dir = [
        pose.position[0] - pose.target[0],
        pose.position[1] - pose.target[1],
        pose.position[2] - pose.target[2],
      ];
      const n = SKETCH_VIEW_POSES[plane].normal;
      // Direction is parallel to the normal with length 4.
      expect(dir[0]).toBeCloseTo(n[0] * 4);
      expect(dir[1]).toBeCloseTo(n[1] * 4);
      expect(dir[2]).toBeCloseTo(n[2] * 4);
      // Up is perpendicular to the view direction.
      const up = pose.up;
      expect(dir[0] * up[0] + dir[1] * up[1] + dir[2] * up[2]).toBeCloseTo(0);
    }
  });
});

describe('orthoHalfExtents', () => {
  it('matches the perspective apparent size at the target', () => {
    const { halfW, halfH } = orthoHalfExtents(10, 45, 2);
    expect(halfH).toBeCloseTo(10 * Math.tan(Math.PI / 8));
    expect(halfW).toBeCloseTo(2 * halfH);
  });

  it('scales linearly with distance', () => {
    const near = orthoHalfExtents(5, 45, 1);
    const far = orthoHalfExtents(10, 45, 1);
    expect(far.halfH).toBeCloseTo(near.halfH * 2);
  });
});

describe('px <-> world mapping (of-4eh.14)', () => {
  const FOV = 45;
  const HEIGHT = 800;

  it('pxPerUnit is the viewport height over the ortho frustum height', () => {
    for (const dist of [0.5, 3, 42]) {
      const { halfH } = orthoHalfExtents(dist, FOV, 1.7);
      expect(sketchPxPerUnit(dist, FOV, HEIGHT) * 2 * halfH).toBeCloseTo(HEIGHT);
    }
  });

  it('sketchDistForPxPerUnit inverts sketchPxPerUnit', () => {
    for (const dist of [0.25, 1, 12, 300]) {
      const scale = sketchPxPerUnit(dist, FOV, HEIGHT);
      expect(sketchDistForPxPerUnit(scale, FOV, HEIGHT)).toBeCloseTo(dist);
    }
  });

  it('world<->screen round-trips and scales uniformly in x and y', () => {
    const view = { cx: 1.5, cy: -2, scale: 37 };
    const size = { w: 1200, h: 800 };
    const [sx, sy] = sketchWorldToScreen(view, size, 3, 4);
    const back = sketchScreenToWorld(view, size, sx, sy);
    expect(back.x).toBeCloseTo(3);
    expect(back.y).toBeCloseTo(4);
    // One world unit is `scale` px along both axes; screen y grows downward.
    const [sx1, sy1] = sketchWorldToScreen(view, size, 4, 5);
    expect(sx1 - sx).toBeCloseTo(view.scale);
    expect(sy1 - sy).toBeCloseTo(-view.scale);
    // The view center maps to the viewport center.
    expect(sketchWorldToScreen(view, size, 1.5, -2)).toEqual([600, 400]);
  });

  it('camera <-> overlay view round-trips on every plane', () => {
    for (const plane of Object.keys(SKETCH_VIEW_POSES)) {
      const view = { cx: 2.5, cy: -1.25, scale: 90 };
      const pose = cameraFromSketchView(plane, view, FOV, HEIGHT);
      const back = sketchViewFromCamera(plane, pose.target, pose.dist, FOV, HEIGHT);
      expect(back.cx).toBeCloseTo(view.cx);
      expect(back.cy).toBeCloseTo(view.cy);
      expect(back.scale).toBeCloseTo(view.scale);
      // The camera sits `dist` off the plane, over the view center.
      expect(pose.target).toEqual(planeToWorld(plane, view.cx, view.cy));
      const n = SKETCH_VIEW_POSES[plane].normal;
      for (let i = 0; i < 3; i += 1) {
        expect(pose.position[i]).toBeCloseTo(pose.target[i] + n[i] * pose.dist);
      }
    }
    expect(cameraFromSketchView('AB', { cx: 0, cy: 0, scale: 1 }, FOV, HEIGHT)).toBeNull();
  });

  it('sketch (u, v) axes match the normal-to camera screen axes', () => {
    // The overlay draws u right and v up; the camera must show
    // planeToWorld(u, v) at that same spot, so e_u == camera-right and
    // e_v == camera-up for every plane. This pins overlay orientation to
    // the 3D render (regression: XZ/YZ used to be mirrored).
    for (const plane of Object.keys(SKETCH_VIEW_POSES)) {
      const { normal: n, up } = SKETCH_VIEW_POSES[plane];
      const right = [
        up[1] * n[2] - up[2] * n[1],
        up[2] * n[0] - up[0] * n[2],
        up[0] * n[1] - up[1] * n[0],
      ];
      const eu = planeToWorld(plane, 1, 0);
      const ev = planeToWorld(plane, 0, 1);
      for (let i = 0; i < 3; i += 1) {
        expect(eu[i]).toBeCloseTo(right[i]);
        expect(ev[i]).toBeCloseTo(up[i]);
      }
    }
  });

  it('regression: profile bbox in model units == drawn px bbox / pxPerUnit', () => {
    // Simulate drawing a rectangle on screen with the camera-matched view,
    // exactly as SketchCanvas does: screen px -> world via the shared
    // mapping, then extract the profile the extrude consumes.
    const dist = 7.3;
    const scale = sketchPxPerUnit(dist, FOV, HEIGHT);
    const view = { cx: 0.4, cy: -0.7, scale };
    const size = { w: 1280, h: HEIGHT };
    const pxA = [300, 620];
    const pxB = [640, 180]; // 340 x 440 px on screen
    const a = sketchScreenToWorld(view, size, ...pxA);
    const b = sketchScreenToWorld(view, size, ...pxB);
    const sketch = createSketch();
    addRectangle(sketch, a.x, a.y, b.x, b.y);
    const profile = extractProfile(sketch, 'XY');
    expect(profile.closed).toBe(true);
    const us = [];
    const vs = [];
    for (const seg of profile.segments) {
      for (const [u, v] of [segmentStart2D(seg), segmentEnd2D(seg)]) {
        us.push(u);
        vs.push(v);
      }
    }
    const bboxW = Math.max(...us) - Math.min(...us);
    const bboxH = Math.max(...vs) - Math.min(...vs);
    expect(bboxW).toBeCloseTo(Math.abs(pxB[0] - pxA[0]) / scale, 9);
    expect(bboxH).toBeCloseTo(Math.abs(pxB[1] - pxA[1]) / scale, 9);
  });
});

describe('gridLevels', () => {
  it('reproduces the legacy grid density at the default framing distance', () => {
    const g = gridLevels(10);
    expect(g.minor).toBeCloseTo(0.5);
    expect(g.major).toBeCloseTo(5);
    expect(g.minorAlpha).toBeCloseTo(1);
  });

  it('steps up one decade per 10x of camera distance', () => {
    expect(gridLevels(100).minor).toBeCloseTo(5);
    expect(gridLevels(1000).minor).toBeCloseTo(50);
  });

  it('fades minor lines continuously between decades', () => {
    const mid = gridLevels(10 * 10 ** 0.5); // halfway between decades
    expect(mid.minorAlpha).toBeCloseTo(0.5);
    const nearNext = gridLevels(10 * 10 ** 0.99);
    expect(nearNext.minorAlpha).toBeLessThan(0.05);
  });

  it('keeps major spacing at 10x minor and fade distance proportional', () => {
    for (const d of [0.3, 7, 42, 5000]) {
      const g = gridLevels(d);
      expect(g.major).toBeCloseTo(g.minor * 10);
      expect(g.fadeDist).toBeCloseTo(d * 4);
    }
  });

  it('handles zero/negative distance without NaN', () => {
    const g = gridLevels(0);
    expect(Number.isFinite(g.minor)).toBe(true);
    expect(g.minor).toBeGreaterThan(0);
  });
});

describe('easeInOutCubic', () => {
  it('hits the endpoints and midpoint', () => {
    expect(easeInOutCubic(0)).toBe(0);
    expect(easeInOutCubic(1)).toBe(1);
    expect(easeInOutCubic(0.5)).toBeCloseTo(0.5);
  });

  it('clamps outside [0, 1]', () => {
    expect(easeInOutCubic(-2)).toBe(0);
    expect(easeInOutCubic(3)).toBe(1);
  });

  it('is monotonic', () => {
    let prev = -1;
    for (let i = 0; i <= 20; i += 1) {
      const v = easeInOutCubic(i / 20);
      expect(v).toBeGreaterThanOrEqual(prev);
      prev = v;
    }
  });
});

describe('planeIndicatorSize', () => {
  it('has a floor for empty scenes', () => {
    expect(planeIndicatorSize([0, 0, 0], 0)).toBe(20);
  });

  it('is several times the scene extent', () => {
    expect(planeIndicatorSize([0, 0, 0], 10)).toBeCloseTo(60);
  });

  it('accounts for off-origin scenes', () => {
    expect(planeIndicatorSize([3, 0, 4], 5)).toBeCloseTo((5 + 5) * 6);
  });
});
