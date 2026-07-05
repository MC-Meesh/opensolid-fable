import { describe, expect, it } from 'vitest';
import { formatAngle, formatNumber, parseDimension } from './format.js';

describe('formatNumber', () => {
  it('trims to at most 2 decimals and strips trailing zeros', () => {
    expect(formatNumber(12.3456)).toBe('12.35');
    expect(formatNumber(12.3)).toBe('12.3');
    expect(formatNumber(12)).toBe('12');
    expect(formatNumber(0.5)).toBe('0.5');
  });

  it('never emits -0 and handles non-finite input', () => {
    expect(formatNumber(-0.001)).toBe('0');
    expect(formatNumber(0)).toBe('0');
    expect(formatNumber(NaN)).toBe('');
    expect(formatNumber(Infinity)).toBe('');
  });
});

describe('formatAngle', () => {
  it('converts radians to a degree readout', () => {
    expect(formatAngle(Math.PI / 4)).toBe('45°');
    expect(formatAngle(Math.PI)).toBe('180°');
    expect(formatAngle(0)).toBe('0°');
  });
});

describe('parseDimension', () => {
  it('accepts strictly positive finite numbers', () => {
    expect(parseDimension('12.5')).toBe(12.5);
    expect(parseDimension('0.001')).toBe(0.001);
  });

  it('rejects zero, negatives, garbage, and empty', () => {
    expect(parseDimension('0')).toBeNull();
    expect(parseDimension('-3')).toBeNull();
    expect(parseDimension('abc')).toBeNull();
    expect(parseDimension('')).toBeNull();
    expect(parseDimension('  ')).toBeNull();
    expect(parseDimension(undefined)).toBeNull();
  });
});
