// Mass properties: turning the kernel's geometric measure into physical
// quantities.
//
// `WasmShape.measure()` reports pure geometry as a JSON string — volume,
// surface area, centroid, and an inertia tensor about the centroid — computed
// as exact polyhedral integrals over the measured mesh. Two things are missing
// before that can answer "what does this weigh?":
//
//   1. **Density.** The tensor and volume are at *unit density*, so mass is
//      `density · volume` and inertia scales linearly with density.
//   2. **Units.** The kernel is unitless (see units.js): a volume is in
//      document-unit³, not m³. Densities are physical (kg/m³), so raw
//      quantities must be scaled into SI before the two can be combined.
//      Getting this wrong is silent — the numbers still look plausible — so
//      the exponents below are the load-bearing part of this module:
//
//        length   L¹ → ·s      centroid, bounding box
//        area     L² → ·s²     surface area
//        volume   L³ → ·s³     volume
//        inertia  L⁵ → ·s⁵     ∫r² dV, then ·density for kg·m²
//
//      where `s = metresPerUnit(documentUnit)`.
//
// Reported centroid, surface area, and volume stay in *document* units, which
// is what the user authored in and expects to read back; only mass and inertia
// cross into SI, where a kilogram is a kilogram regardless of document unit.
//
// Kept free of React and WASM imports so it can be unit-tested directly.

import { metresPerUnit } from './units.js';

/**
 * Parse the JSON string from `WasmShape.measure()`. Returns `null` for
 * anything unparseable rather than throwing, so a mangled or truncated
 * payload surfaces as a readout error instead of unmounting the app.
 */
export function parseMeasure(json) {
  if (typeof json !== 'string') return null;
  try {
    const parsed = JSON.parse(json);
    return parsed && typeof parsed === 'object' ? parsed : null;
  } catch {
    return null;
  }
}

/**
 * Format a number for display at `digits` significant figures, trimming the
 * trailing zeros `toPrecision` leaves behind (`Number(...)` does the trimming)
 * and falling back to exponential for magnitudes where fixed notation would be
 * unreadable. Non-finite input renders as an em dash, never `NaN`.
 */
export function formatNumber(value, digits = 6) {
  if (!Number.isFinite(value)) return '—';
  if (value === 0) return '0';
  const magnitude = Math.abs(value);
  if (magnitude >= 1e7 || magnitude < 1e-4) {
    return value.toExponential(Math.max(0, digits - 1)).replace('e', 'e');
  }
  return String(Number(value.toPrecision(digits)));
}

/**
 * Pick a human-scaled mass unit and format the value: kilograms down to 1 kg,
 * grams below that, milligrams below a gram. A 10 mm cube of aluminium weighs
 * 2.7 g — reading that as `0.0027 kg` is technically right and practically
 * useless, which is why this switches rather than fixing one unit.
 */
export function formatMass(kilograms) {
  if (!Number.isFinite(kilograms)) return '—';
  const magnitude = Math.abs(kilograms);
  if (magnitude >= 1) return `${formatNumber(kilograms, 6)} kg`;
  if (magnitude >= 1e-3) return `${formatNumber(kilograms * 1e3, 6)} g`;
  return `${formatNumber(kilograms * 1e6, 6)} mg`;
}

/** Scale a 3×3 tensor (array of rows) by `factor`, preserving shape. */
function scaleTensor(rows, factor) {
  return rows.map((row) => row.map((v) => v * factor));
}

function isVector3(v) {
  return Array.isArray(v) && v.length === 3 && v.every(Number.isFinite);
}

function isTensor3(m) {
  return Array.isArray(m) && m.length === 3 && m.every(isVector3);
}

/**
 * Combine a parsed `measure()` payload with a density and document unit into
 * the physical quantities the Mass Properties panel reports.
 *
 * Returns `{ ok: false, error }` when the mesh doesn't bound a solid — the
 * kernel says so via `massError` and null fields, which happens for an open or
 * non-manifold shape. The bounding box survives that case (the kernel always
 * reports it), so it is passed through either way and the panel can still show
 * something useful about a shape that has no volume.
 *
 * @param measure parsed `measure()` JSON (see `parseMeasure`)
 * @param density kg/m³, already validated positive (see `normalizeDensity`)
 * @param unit    document unit key, e.g. `'mm'`
 */
export function massProperties({ measure, density, unit }) {
  if (!measure) {
    return { ok: false, error: 'Measurement unavailable.', boundingBox: null };
  }
  const boundingBox = measure.boundingBox ?? null;
  const counts = {
    triangles: measure.triangles ?? null,
    vertices: measure.vertices ?? null,
    exact: Boolean(measure.exact),
  };
  const base = { boundingBox, ...counts };

  if (measure.massError) {
    return { ok: false, error: String(measure.massError), ...base };
  }
  if (!Number.isFinite(measure.volume) || !isVector3(measure.centroid)) {
    return { ok: false, error: 'This shape does not enclose a solid.', ...base };
  }
  const rho = Number(density);
  if (!Number.isFinite(rho) || rho <= 0) {
    return { ok: false, error: 'Density must be a positive number.', ...base };
  }

  const s = metresPerUnit(unit);
  const volumeM3 = measure.volume * s ** 3;
  const massKg = rho * volumeM3;

  // Unit-density L⁵ tensor → physical kg·m². Guard the shape of the tensor:
  // the kernel nulls it alongside volume, and a partial payload shouldn't
  // render `NaN`s into the table.
  const inertia = isTensor3(measure.inertia)
    ? scaleTensor(measure.inertia, rho * s ** 5)
    : null;

  return {
    ok: true,
    error: null,
    ...base,
    density: rho,
    volume: measure.volume,
    surfaceArea: Number.isFinite(measure.surfaceArea) ? measure.surfaceArea : null,
    centroid: measure.centroid,
    volumeM3,
    massKg,
    inertia,
  };
}
