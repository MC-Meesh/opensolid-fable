import { describe, expect, it } from 'vitest';
import {
  CUSTOM_MATERIAL,
  DEFAULT_MATERIAL,
  MATERIALS,
  densityForSelection,
  materialDensity,
  materialName,
  normalizeDensity,
  normalizeMaterial,
} from './materials.js';

describe('material library', () => {
  it('defaults to aluminium and lists Custom first', () => {
    expect(DEFAULT_MATERIAL).toBe('aluminium-6061');
    expect(MATERIALS[0].key).toBe(CUSTOM_MATERIAL);
  });

  it('gives every material a unique key and a positive density', () => {
    const keys = MATERIALS.map((m) => m.key);
    expect(new Set(keys).size).toBe(keys.length);
    for (const m of MATERIALS) {
      expect(m.density).toBeGreaterThan(0);
      expect(m.name).toBeTruthy();
    }
  });

  it('normalizes unknown/undefined keys to the default', () => {
    expect(normalizeMaterial('titanium')).toBe('titanium');
    expect(normalizeMaterial('unobtainium')).toBe(DEFAULT_MATERIAL);
    expect(normalizeMaterial(undefined)).toBe(DEFAULT_MATERIAL);
    expect(normalizeMaterial(null)).toBe(DEFAULT_MATERIAL);
  });

  it('reports tabulated densities in kg/m3', () => {
    expect(materialDensity('steel-1020')).toBe(7870);
    expect(materialDensity('water')).toBe(1000);
    expect(materialDensity('bogus')).toBe(materialDensity(DEFAULT_MATERIAL));
    expect(materialName('titanium')).toBe('Titanium');
  });

  it('accepts positive densities as numbers or typed strings', () => {
    expect(normalizeDensity(7870)).toBe(7870);
    expect(normalizeDensity('7870')).toBe(7870);
    expect(normalizeDensity('  2700.5 ')).toBe(2700.5);
  });

  it('rejects non-positive and non-numeric densities rather than clamping', () => {
    // A zero/negative density would silently report a massless or
    // negative-mass solid, so these must be refused, not defaulted.
    expect(normalizeDensity(0)).toBeNull();
    expect(normalizeDensity(-5)).toBeNull();
    expect(normalizeDensity('abc')).toBeNull();
    expect(normalizeDensity('')).toBeNull();
    expect(normalizeDensity(NaN)).toBeNull();
    expect(normalizeDensity(Infinity)).toBeNull();
    expect(normalizeDensity(undefined)).toBeNull();
  });

  it('adopts a listed material density on selection', () => {
    expect(densityForSelection('steel-1020', 2700)).toBe(7870);
    expect(densityForSelection('water', 99)).toBe(1000);
  });

  it('keeps the current density when switching to Custom', () => {
    // Switching to Custom to tweak a value must not first destroy it.
    expect(densityForSelection(CUSTOM_MATERIAL, 7870)).toBe(7870);
    expect(densityForSelection(CUSTOM_MATERIAL, '1234')).toBe(1234);
  });

  it('seeds Custom with a usable density when the current one is invalid', () => {
    expect(densityForSelection(CUSTOM_MATERIAL, 0)).toBe(1000);
    expect(densityForSelection(CUSTOM_MATERIAL, 'abc')).toBe(1000);
  });
});
