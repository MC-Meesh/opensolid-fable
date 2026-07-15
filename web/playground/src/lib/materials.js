// Material library for the playground.
//
// A material exists for exactly one reason: it carries a density, which turns
// the kernel's geometric volume into a mass. Nothing here affects geometry or
// rendering — the viewport's appearance is independent of the assigned
// material (see Viewport3D's shader material, which is a *render* material and
// unrelated to this table).
//
// Densities are SI (kg/m³), the convention every published material table and
// CAD system uses. The document unit never rescales them: a document authored
// in millimetres still assigns steel 7870 kg/m³, and the volume is converted
// to m³ at mass time via `metresPerUnit` (see units.js and massProps.js).
// Values are nominal room-temperature figures for the common alloy/grade, good
// enough for the "what does this weigh?" question the readout answers.
//
// Kept free of React and WASM imports so it can be unit-tested directly.

/**
 * The selectable materials, in menu order. `key` is the stable identifier
 * persisted to storage; `name` is the menu label; `density` is kg/m³.
 *
 * `custom` is the escape hatch: it holds no authoritative density of its own
 * (the user types one), and selecting it must not clobber the density already
 * in the field. Its listed density is only the seed for a fresh document.
 */
export const MATERIALS = [
  { key: 'custom', name: 'Custom', density: 1000 },
  { key: 'aluminium-6061', name: 'Aluminium 6061', density: 2700 },
  { key: 'steel-1020', name: 'Steel AISI 1020', density: 7870 },
  { key: 'stainless-304', name: 'Stainless Steel 304', density: 8000 },
  { key: 'titanium', name: 'Titanium', density: 4510 },
  { key: 'brass', name: 'Brass', density: 8500 },
  { key: 'copper', name: 'Copper', density: 8940 },
  { key: 'abs', name: 'ABS', density: 1020 },
  { key: 'nylon-66', name: 'Nylon 6/6', density: 1140 },
  { key: 'pla', name: 'PLA', density: 1240 },
  { key: 'polycarbonate', name: 'Polycarbonate', density: 1200 },
  { key: 'oak', name: 'Oak', density: 700 },
  { key: 'water', name: 'Water', density: 1000 },
];

/**
 * Default material: aluminium, the most common machined stock and a mid-range
 * density that makes an unconfigured mass readout plausible rather than 1.
 */
export const DEFAULT_MATERIAL = 'aluminium-6061';

/** The key of the material whose density the user supplies. */
export const CUSTOM_MATERIAL = 'custom';

const BY_KEY = new Map(MATERIALS.map((m) => [m.key, m]));

/** localStorage key the playground persists the assigned material under. */
export const MATERIAL_STORAGE_KEY = 'opensolid.material';

/** localStorage key the playground persists a custom density under. */
export const DENSITY_STORAGE_KEY = 'opensolid.density';

/**
 * Coerce an arbitrary value to a known material key, falling back to the
 * default for anything unrecognised (stale storage, undefined).
 */
export function normalizeMaterial(key) {
  return BY_KEY.has(key) ? key : DEFAULT_MATERIAL;
}

/** Menu label for a material key, e.g. `'Titanium'`. */
export function materialName(key) {
  return (BY_KEY.get(key) ?? BY_KEY.get(DEFAULT_MATERIAL)).name;
}

/**
 * The tabulated density (kg/m³) for a material key. `custom` has no
 * authoritative density — callers hold the user's value and must not consult
 * this for it — so this returns the seed value.
 */
export function materialDensity(key) {
  return (BY_KEY.get(key) ?? BY_KEY.get(DEFAULT_MATERIAL)).density;
}

/**
 * Coerce a typed density to a usable positive number of kg/m³, or `null` if
 * it isn't one. Zero and negatives are rejected rather than clamped: a
 * massless or negative-mass solid is not a thing, and silently substituting a
 * default would misreport a mass the user believes they specified.
 */
export function normalizeDensity(value) {
  const n = typeof value === 'string' ? Number(value.trim()) : Number(value);
  if (!Number.isFinite(n) || n <= 0) return null;
  return n;
}

/**
 * The density to apply when `key` is selected, given the density currently in
 * the field. Picking a listed material adopts its tabulated density; picking
 * `custom` keeps what's there, so switching to Custom to tweak a value doesn't
 * first destroy it.
 */
export function densityForSelection(key, currentDensity) {
  if (key === CUSTOM_MATERIAL) {
    return normalizeDensity(currentDensity) ?? materialDensity(CUSTOM_MATERIAL);
  }
  return materialDensity(key);
}
