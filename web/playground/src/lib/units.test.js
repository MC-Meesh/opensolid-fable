import { describe, expect, it } from 'vitest';
import {
  DEFAULT_LENGTH_UNIT,
  LENGTH_UNITS,
  normalizeUnit,
  unitLabel,
  withUnit,
} from './units.js';

describe('document units', () => {
  it('defaults to millimetres and lists it first', () => {
    expect(DEFAULT_LENGTH_UNIT).toBe('mm');
    expect(LENGTH_UNITS[0].key).toBe('mm');
  });

  it('covers the four exchange units the STEP writer supports', () => {
    expect(LENGTH_UNITS.map((u) => u.key)).toEqual(['mm', 'cm', 'm', 'in']);
  });

  it('normalizes unknown/undefined keys to the default', () => {
    expect(normalizeUnit('mm')).toBe('mm');
    expect(normalizeUnit('in')).toBe('in');
    expect(normalizeUnit('furlong')).toBe('mm');
    expect(normalizeUnit(undefined)).toBe('mm');
    expect(normalizeUnit(null)).toBe('mm');
  });

  it('labels known units and falls back for unknown ones', () => {
    expect(unitLabel('cm')).toBe('cm');
    expect(unitLabel('in')).toBe('in');
    expect(unitLabel('bogus')).toBe('mm');
  });

  it('appends the unit suffix, leaving blanks blank', () => {
    expect(withUnit('12.5', 'mm')).toBe('12.5 mm');
    expect(withUnit('3', 'in')).toBe('3 in');
    expect(withUnit('', 'mm')).toBe('');
    expect(withUnit(null, 'mm')).toBe('');
  });
});
