// Document unit system for the playground.
//
// The kernel and every geometry op are unitless: a coordinate is just a
// number. A *document unit* attaches meaning to those numbers for display,
// dimension entry, and — critically — the STEP export's unit declaration, so
// a file opened in another CAD system resolves to the intended scale.
//
// Numbers are NOT rescaled when the document unit changes: box3(1,…) stays 1
// whether the document is in millimetres or inches. The unit is metadata plus
// a display label; switching it relabels fields and changes the STEP
// SI_UNIT/CONVERSION_BASED_UNIT declaration, nothing more. This mirrors the
// kernel writer, which emits coordinates verbatim interpreted in the declared
// unit (see crates/opensolid-kernel/src/io/step/write.rs, `LengthUnit`).
//
// Kept free of React and WASM imports so it can be unit-tested directly and
// shared by the property panel, sketch canvas, status bar, and export path.

/**
 * The length units a document can be authored in. `key` is the stable
 * identifier persisted and passed to the WASM `exportStep`; `label` is the
 * suffix shown next to dimensions; `name` is the long form for menus;
 * `metres` is how many metres one unit spans, which converts the kernel's
 * unitless numbers into SI for mass properties. Order is the order shown in
 * the unit picker.
 */
export const LENGTH_UNITS = [
  { key: 'mm', label: 'mm', name: 'Millimetres', metres: 0.001 },
  { key: 'cm', label: 'cm', name: 'Centimetres', metres: 0.01 },
  { key: 'm', label: 'm', name: 'Metres', metres: 1 },
  { key: 'in', label: 'in', name: 'Inches', metres: 0.0254 },
];

/** Default document unit: millimetres, the conventional CAD exchange unit. */
export const DEFAULT_LENGTH_UNIT = 'mm';

const BY_KEY = new Map(LENGTH_UNITS.map((u) => [u.key, u]));

/** localStorage key the playground persists the document unit under. */
export const UNIT_STORAGE_KEY = 'opensolid.documentUnit';

/**
 * Coerce an arbitrary value to a known unit key, falling back to the default
 * for anything unrecognised (stale storage, a bad query param, undefined).
 */
export function normalizeUnit(key) {
  return BY_KEY.has(key) ? key : DEFAULT_LENGTH_UNIT;
}

/** Short display suffix for a unit key, e.g. `'mm'`. */
export function unitLabel(key) {
  return (BY_KEY.get(key) ?? BY_KEY.get(DEFAULT_LENGTH_UNIT)).label;
}

/**
 * How many metres one document unit spans, e.g. `metresPerUnit('mm') ===
 * 0.001`. Mass properties are the one place the playground must leave the
 * kernel's unitless world: a density is physical (kg/m³), so a raw volume in
 * document-unit³ has to be scaled by this factor cubed to reach m³. Unknown
 * keys fall back to the default unit, matching [`normalizeUnit`].
 */
export function metresPerUnit(key) {
  return (BY_KEY.get(key) ?? BY_KEY.get(DEFAULT_LENGTH_UNIT)).metres;
}

/**
 * Append a unit suffix to an already-formatted number string, e.g.
 * `withUnit('12.5', 'mm') === '12.5 mm'`. A blank number stays blank so
 * empty readouts don't render a lone unit.
 */
export function withUnit(numberText, key) {
  if (numberText === '' || numberText == null) return '';
  return `${numberText} ${unitLabel(key)}`;
}
