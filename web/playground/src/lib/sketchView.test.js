import { describe, expect, it } from 'vitest';
import {
  SKETCH_VIEW_POSES,
  easeInOutCubic,
  gridLevels,
  orthoHalfExtents,
  planeIndicatorSize,
  sketchViewPose,
} from './sketchView.js';

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
