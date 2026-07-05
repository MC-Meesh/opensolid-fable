/** Formatting/parsing helpers for on-canvas dimension readouts and entry. */

/**
 * Compact number for dimension readouts: at most 2 decimals, trailing zeros
 * stripped, never "-0".
 */
export function formatNumber(v) {
  if (!Number.isFinite(v)) return '';
  let s = v.toFixed(2).replace(/\.?0+$/, '');
  if (s === '-0' || s === '') s = '0';
  return s;
}

/** Angle readout in degrees, e.g. "45°". */
export function formatAngle(rad) {
  const deg = (rad * 180) / Math.PI;
  return `${formatNumber(deg)}°`;
}

/** Parse a typed dimension: a finite, strictly positive number or null. */
export function parseDimension(text) {
  if (typeof text !== 'string' || text.trim() === '') return null;
  const v = Number(text);
  return Number.isFinite(v) && v > 0 ? v : null;
}
