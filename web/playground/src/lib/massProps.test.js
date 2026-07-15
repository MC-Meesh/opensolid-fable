import { describe, expect, it } from 'vitest';
import { formatMass, formatNumber, massProperties, parseMeasure } from './massProps.js';

/**
 * A unit-density 10-unit cube centred on the origin, in the shape `measure()`
 * returns. Volume 1000, area 600, and the closed-form inertia of a solid cube
 * about its centre: I = m·a²/6 with m = 1000 (unit density), a = 10.
 */
const CUBE_I = (1000 * 10 ** 2) / 6;
const cubeMeasure = {
  volume: 1000,
  surfaceArea: 600,
  centroid: [0, 0, 0],
  inertia: [
    [CUBE_I, 0, 0],
    [0, CUBE_I, 0],
    [0, 0, CUBE_I],
  ],
  boundingBox: { min: [-5, -5, -5], max: [5, 5, 5], size: [10, 10, 10] },
  triangles: 12,
  vertices: 8,
  exact: true,
};

describe('parseMeasure', () => {
  it('parses a measure payload', () => {
    expect(parseMeasure('{"volume":8}')).toEqual({ volume: 8 });
  });

  it('returns null for unparseable or non-object payloads', () => {
    // A mangled payload must surface as a readout error, never a throw.
    expect(parseMeasure('{oops')).toBeNull();
    expect(parseMeasure('null')).toBeNull();
    expect(parseMeasure('42')).toBeNull();
    expect(parseMeasure(undefined)).toBeNull();
  });
});

describe('formatNumber', () => {
  it('trims trailing zeros left by toPrecision', () => {
    expect(formatNumber(2.5)).toBe('2.5');
    expect(formatNumber(1000)).toBe('1000');
  });

  it('uses exponential notation outside the readable fixed range', () => {
    expect(formatNumber(4.5e-8)).toContain('e-8');
    expect(formatNumber(1.2e9)).toContain('e+9');
  });

  it('renders non-finite values as an em dash, never NaN', () => {
    expect(formatNumber(NaN)).toBe('—');
    expect(formatNumber(Infinity)).toBe('—');
    expect(formatNumber(0)).toBe('0');
  });
});

describe('formatMass', () => {
  it('scales to kg, g, and mg so small parts stay readable', () => {
    expect(formatMass(2.5)).toBe('2.5 kg');
    expect(formatMass(0.0027)).toBe('2.7 g');
    expect(formatMass(1e-7)).toBe('0.1 mg');
  });

  it('renders non-finite mass as an em dash', () => {
    expect(formatMass(NaN)).toBe('—');
  });
});

describe('massProperties', () => {
  it('reports geometry in document units, unscaled', () => {
    const mp = massProperties({ measure: cubeMeasure, density: 2700, unit: 'mm' });
    expect(mp.ok).toBe(true);
    expect(mp.volume).toBe(1000);
    expect(mp.surfaceArea).toBe(600);
    expect(mp.centroid).toEqual([0, 0, 0]);
  });

  it('converts volume to m3 by the cube of the unit scale', () => {
    // 1000 mm3 = 1e-6 m3.
    const mm = massProperties({ measure: cubeMeasure, density: 1000, unit: 'mm' });
    expect(mm.volumeM3).toBeCloseTo(1e-6, 15);
    // The same 1000 authored in metres is a thousand cubic metres.
    const m = massProperties({ measure: cubeMeasure, density: 1000, unit: 'm' });
    expect(m.volumeM3).toBeCloseTo(1000, 9);
  });

  it('computes mass as density x volume in SI', () => {
    // A 10 mm aluminium cube weighs 2.7 g.
    const mp = massProperties({ measure: cubeMeasure, density: 2700, unit: 'mm' });
    expect(mp.massKg).toBeCloseTo(2.7e-3, 12);
    expect(formatMass(mp.massKg)).toBe('2.7 g');
  });

  it('scales mass with the document unit, since the kernel is unitless', () => {
    // Identical numbers authored in cm are a 10 cm cube: 1000x the volume of
    // the mm cube, so 1000x the mass.
    const mm = massProperties({ measure: cubeMeasure, density: 2700, unit: 'mm' });
    const cm = massProperties({ measure: cubeMeasure, density: 2700, unit: 'cm' });
    expect(cm.massKg / mm.massKg).toBeCloseTo(1000, 6);
  });

  it('scales inertia by density and the fifth power of the unit scale', () => {
    // Cross-check the L^5 exponent against the closed form computed directly
    // in SI: a 10 mm aluminium cube has m = 2.7e-3 kg, a = 0.01 m, so
    // I = m*a^2/6 = 4.5e-8 kg*m^2. If the exponent were wrong this would miss
    // by orders of magnitude.
    const mp = massProperties({ measure: cubeMeasure, density: 2700, unit: 'mm' });
    const expected = (2.7e-3 * 0.01 ** 2) / 6;
    expect(expected).toBeCloseTo(4.5e-8, 15);
    expect(mp.inertia[0][0]).toBeCloseTo(expected, 15);
    expect(mp.inertia[1][1]).toBeCloseTo(expected, 15);
    expect(mp.inertia[0][1]).toBe(0);
  });

  it('scales inertia linearly with density', () => {
    const a = massProperties({ measure: cubeMeasure, density: 1000, unit: 'mm' });
    const b = massProperties({ measure: cubeMeasure, density: 2000, unit: 'mm' });
    expect(b.inertia[0][0] / a.inertia[0][0]).toBeCloseTo(2, 9);
  });

  it('handles inches, where the scale is not a power of ten', () => {
    const mp = massProperties({ measure: cubeMeasure, density: 1000, unit: 'in' });
    expect(mp.volumeM3).toBeCloseTo(1000 * 0.0254 ** 3, 12);
    expect(mp.massKg).toBeCloseTo(1000 * 1000 * 0.0254 ** 3, 9);
  });

  it('passes through mesh counts and exactness', () => {
    const mp = massProperties({ measure: cubeMeasure, density: 2700, unit: 'mm' });
    expect(mp.triangles).toBe(12);
    expect(mp.vertices).toBe(8);
    expect(mp.exact).toBe(true);
    expect(mp.boundingBox.size).toEqual([10, 10, 10]);
  });

  it('surfaces the kernel massError but keeps the bounding box', () => {
    // The kernel always reports bounds, even for a shape with no volume.
    const open = {
      volume: null,
      surfaceArea: null,
      centroid: null,
      inertia: null,
      boundingBox: { min: [0, 0, 0], max: [1, 1, 1], size: [1, 1, 1] },
      triangles: 2,
      vertices: 4,
      exact: false,
      massError: 'mesh is not a closed, consistently oriented manifold',
    };
    const mp = massProperties({ measure: open, density: 2700, unit: 'mm' });
    expect(mp.ok).toBe(false);
    expect(mp.error).toContain('manifold');
    expect(mp.boundingBox.size).toEqual([1, 1, 1]);
    expect(mp.triangles).toBe(2);
  });

  it('rejects a missing measurement', () => {
    const mp = massProperties({ measure: null, density: 2700, unit: 'mm' });
    expect(mp.ok).toBe(false);
    expect(mp.error).toBeTruthy();
  });

  it('rejects a non-positive density', () => {
    const mp = massProperties({ measure: cubeMeasure, density: 0, unit: 'mm' });
    expect(mp.ok).toBe(false);
    expect(mp.error).toContain('positive');
  });

  it('reports no solid when volume is absent without an explicit massError', () => {
    const mp = massProperties({
      measure: { ...cubeMeasure, volume: null, centroid: null },
      density: 2700,
      unit: 'mm',
    });
    expect(mp.ok).toBe(false);
    expect(mp.error).toContain('does not enclose a solid');
  });

  it('nulls a malformed inertia tensor instead of rendering NaNs', () => {
    const mp = massProperties({
      measure: { ...cubeMeasure, inertia: [[1, 2], [3, 4]] },
      density: 2700,
      unit: 'mm',
    });
    expect(mp.ok).toBe(true);
    expect(mp.inertia).toBeNull();
  });
});
