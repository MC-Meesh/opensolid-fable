import { describe, expect, it } from 'vitest';
import {
  FIT_DISTANCE_FACTOR,
  MIN_FIT_RADIUS,
  VIEW_NAMES,
  VIEW_SHORTCUTS,
  axisView,
  cameraStateFor,
  viewDirection,
} from './views.js';

const length = (v) => Math.hypot(...v);

describe('viewDirection', () => {
  it('gives unit directions for every standard view', () => {
    for (const name of VIEW_NAMES) {
      const dir = viewDirection(name);
      expect(dir, name).not.toBeNull();
      expect(length(dir)).toBeCloseTo(1, 9);
    }
  });

  it('front looks along -Z (camera at +Z), Y-up convention', () => {
    expect(viewDirection('front')).toEqual([0, 0, 1]);
    expect(viewDirection('back')).toEqual([0, 0, -1]);
    expect(viewDirection('right')).toEqual([1, 0, 0]);
    expect(viewDirection('left')).toEqual([-1, 0, 0]);
  });

  it('tilts top/bottom slightly off the pole so Y-up look-at never degenerates', () => {
    const top = viewDirection('top');
    expect(top[1]).toBeGreaterThan(0.999);
    expect(top[2]).toBeGreaterThan(0);
    const bottom = viewDirection('bottom');
    expect(bottom[1]).toBeLessThan(-0.999);
    expect(bottom[2]).toBeGreaterThan(0);
  });

  it('returns null for unknown views', () => {
    expect(viewDirection('trimetric')).toBeNull();
  });

  it('returns a copy, not the internal table entry', () => {
    const a = viewDirection('front');
    a[0] = 99;
    expect(viewDirection('front')).toEqual([0, 0, 1]);
  });
});

describe('VIEW_SHORTCUTS', () => {
  it('maps digits 1-7 to the SolidWorks standard view order', () => {
    expect(Object.keys(VIEW_SHORTCUTS)).toHaveLength(7);
    expect(VIEW_SHORTCUTS[1]).toBe('front');
    expect(VIEW_SHORTCUTS[5]).toBe('top');
    expect(VIEW_SHORTCUTS[7]).toBe('iso');
    for (const name of Object.values(VIEW_SHORTCUTS)) {
      expect(VIEW_NAMES).toContain(name);
    }
  });
});

describe('axisView', () => {
  it('maps triad axes to the view looking down that axis', () => {
    expect(axisView('x')).toBe('right');
    expect(axisView('x', false)).toBe('left');
    expect(axisView('y')).toBe('top');
    expect(axisView('y', false)).toBe('bottom');
    expect(axisView('z')).toBe('front');
    expect(axisView('z', false)).toBe('back');
    expect(axisView('w')).toBeNull();
  });
});

describe('cameraStateFor', () => {
  it('places the camera along the view direction at the fit distance', () => {
    const { position, target } = cameraStateFor('right', [1, 2, 3], 2);
    expect(target).toEqual([1, 2, 3]);
    expect(position[0]).toBeCloseTo(1 + 2 * FIT_DISTANCE_FACTOR, 9);
    expect(position[1]).toBeCloseTo(2, 9);
    expect(position[2]).toBeCloseTo(3, 9);
  });

  it('clamps degenerate radii to the minimum fit radius', () => {
    const { position } = cameraStateFor('front', [0, 0, 0], 0);
    expect(position[2]).toBeCloseTo(MIN_FIT_RADIUS * FIT_DISTANCE_FACTOR, 9);
  });

  it('returns null for unknown views', () => {
    expect(cameraStateFor('nope', [0, 0, 0], 1)).toBeNull();
  });
});
