// Property panel model + the traced-tree mutations behind it.
//
// OP_SPECS describes, for every scene-tree op, which numeric arguments the
// property panel exposes: display label, unit, valid range, drag step, and
// (for angles) display-unit conversion. setNodeArg / setBooleanOp apply a
// panel edit to the traced tree and return a new tree that App serializes
// back into the script — the same flow the viewport gizmo uses.
//
// Kept free of React and WASM imports so it can be unit-tested directly.

import { BINARY_OPS } from './sceneTree.js';
import { replaceById } from './transformEdit.js';

/** Unit label shown next to length-dimensioned fields (scene units). */
export const LENGTH_UNIT = 'mm';

/** Blend radius given to smoothUnion when a boolean is switched onto it. */
export const DEFAULT_BLEND = 0.1;

const DEG_PER_RAD = 180 / Math.PI;
const LIMIT = 10000;
const MIN_SIZE = 0.001;

function field(arg, label, opts = {}) {
  return {
    arg,
    label,
    unit: opts.unit ?? LENGTH_UNIT,
    min: opts.min ?? -LIMIT,
    max: opts.max ?? LIMIT,
    step: opts.step ?? 0.05,
    toDisplay: opts.toDisplay ?? null,
    fromDisplay: opts.fromDisplay ?? null,
  };
}

// Strictly-positive dimension (radius, half-extent, half-height).
const size = (arg, label) => field(arg, label, { min: MIN_SIZE });
// Non-negative radius (fillet, blend) — zero disables the rounding.
const radius0 = (arg, label) => field(arg, label, { min: 0 });
// Unbounded coordinate / offset.
const coord = (arg, label) => field(arg, label);
// Unitless axis direction component.
const axis = (arg, label) => field(arg, label, { unit: '' });
// Strictly-positive scale factor.
const factor = (arg, label) => field(arg, label, { unit: '×', min: MIN_SIZE });
// Angle stored in radians, displayed in degrees.
const angle = (arg, label) =>
  field(arg, label, {
    unit: '°',
    min: -3600,
    max: 3600,
    step: 1,
    toDisplay: (rad) => rad * DEG_PER_RAD,
    fromDisplay: (deg) => deg / DEG_PER_RAD,
  });

const xyz = (make, base = 0) => [make(base, 'x'), make(base + 1, 'y'), make(base + 2, 'z')];

/**
 * Panel description per op: `kind` picks the section layout
 * ('primitive' | 'transform' | 'boolean'), `groups` cluster related fields
 * (position / rotation / scale render as grouped XYZ rows).
 */
export const OP_SPECS = {
  sphere: {
    kind: 'primitive',
    title: 'Sphere',
    groups: [{ label: 'Size', fields: [size(0, 'r')] }],
  },
  box3: {
    kind: 'primitive',
    title: 'Box',
    groups: [{ label: 'Half extents', fields: xyz(size) }],
  },
  roundedBox: {
    kind: 'primitive',
    title: 'Rounded box',
    groups: [
      { label: 'Half extents', fields: xyz(size) },
      { label: 'Fillet', fields: [radius0(3, 'r')] },
    ],
  },
  cylinder: {
    kind: 'primitive',
    title: 'Cylinder',
    groups: [{ label: 'Size', fields: [size(0, 'r'), size(1, 'half h')] }],
  },
  cone: {
    kind: 'primitive',
    title: 'Cone',
    // Either radius may be zero (a pointed apex), so the radii use the
    // non-negative field; the half-height must stay strictly positive.
    groups: [
      { label: 'Size', fields: [radius0(0, 'r bottom'), radius0(1, 'r top'), size(2, 'half h')] },
    ],
  },
  torus: {
    kind: 'primitive',
    title: 'Torus',
    groups: [{ label: 'Size', fields: [size(0, 'R'), size(1, 'r')] }],
  },
  capsule: {
    kind: 'primitive',
    title: 'Capsule',
    groups: [
      { label: 'Start', fields: xyz(coord) },
      { label: 'End', fields: xyz(coord, 3) },
      { label: 'Size', fields: [size(6, 'r')] },
    ],
  },
  halfSpace: {
    kind: 'primitive',
    title: 'Half space',
    groups: [
      { label: 'Point', fields: xyz(coord) },
      { label: 'Normal', fields: xyz(axis, 3) },
    ],
  },
  translate: {
    kind: 'transform',
    title: 'Translate',
    groups: [{ label: 'Position', fields: xyz(coord) }],
  },
  rotate: {
    kind: 'transform',
    title: 'Rotate',
    groups: [
      { label: 'Axis', fields: xyz(axis) },
      { label: 'Rotation', fields: [angle(3, 'θ')] },
    ],
  },
  scale: {
    kind: 'transform',
    title: 'Scale',
    groups: [{ label: 'Scale', fields: xyz(factor) }],
  },
  uniformScale: {
    kind: 'transform',
    title: 'Uniform scale',
    groups: [{ label: 'Scale', fields: [factor(0, 'f')] }],
  },
  taper: {
    kind: 'transform',
    title: 'Draft',
    // Args: pull axis (0..2), neutral point (3..5), draft angle in degrees
    // (6) — the kernel binding takes degrees for this feature.
    groups: [
      { label: 'Pull direction', fields: xyz(axis) },
      { label: 'Neutral point', fields: xyz(coord, 3) },
      {
        label: 'Draft',
        fields: [field(6, 'θ', { unit: '°', min: -89, max: 89, step: 0.5 })],
      },
    ],
  },
  extrude: {
    kind: 'sweep',
    title: 'Extrude',
    groups: [{ label: 'Depth', fields: [size(0, 'h')] }],
  },
  revolve: {
    kind: 'sweep',
    title: 'Revolve',
    // The revolve angle is stored in degrees already (kernel contract).
    groups: [
      {
        label: 'Rotation',
        fields: [field(0, 'θ', { unit: '°', min: 0.1, max: 360, step: 1 })],
      },
    ],
  },
  union: { kind: 'boolean', title: 'Boolean', groups: [] },
  intersect: { kind: 'boolean', title: 'Boolean', groups: [] },
  subtract: { kind: 'boolean', title: 'Boolean', groups: [] },
  smoothUnion: {
    kind: 'boolean',
    title: 'Boolean',
    groups: [{ label: 'Blend', fields: [radius0(0, 'r')] }],
  },
};

/** Dropdown choices for the boolean operation selector. */
export const BOOLEAN_CHOICES = [
  { op: 'union', label: 'Union' },
  { op: 'subtract', label: 'Subtract' },
  { op: 'intersect', label: 'Intersect' },
  { op: 'smoothUnion', label: 'Smooth union' },
];

export function opSpec(op) {
  return OP_SPECS[op] ?? null;
}

/** Current value of a field in display units (e.g. degrees for angles). */
export function displayValue(fieldSpec, node) {
  const raw = node.args[fieldSpec.arg];
  return fieldSpec.toDisplay ? fieldSpec.toDisplay(raw) : raw;
}

/** Clamp a display-unit value into the field's valid range. */
export function clampDisplay(fieldSpec, value) {
  return Math.min(fieldSpec.max, Math.max(fieldSpec.min, value));
}

function findById(node, id) {
  if (node.id === id) return node;
  for (const child of node.children) {
    const found = findById(child, id);
    if (found) return found;
  }
  return null;
}

function round4(v) {
  return Math.round(v * 10000) / 10000;
}

/**
 * Set one argument of a node to `value` (given in the field's display units).
 * The value is validated against the field's range and converted to the
 * stored unit. Returns `{ root }` with a new tree — or the same tree object
 * when the edit is a no-op — or `{ error }`.
 */
export function setNodeArg(root, id, argIndex, value) {
  if (!Number.isFinite(value)) return { error: 'value must be a finite number' };
  const target = findById(root, id);
  if (!target) return { error: `no scene node #${id}` };
  const spec = OP_SPECS[target.op];
  const f = spec?.groups.flatMap((g) => g.fields).find((g) => g.arg === argIndex);
  if (!f) return { error: `${target.op} has no editable argument ${argIndex}` };
  const display = clampDisplay(f, value);
  const raw = round4(f.fromDisplay ? f.fromDisplay(display) : display);
  if (raw === target.args[argIndex]) return { root };
  const args = target.args.slice();
  args[argIndex] = raw;
  return { root: replaceById(root, id, { ...target, args, shape: null }) };
}

/**
 * Switch a boolean node to a different operation. Switching onto smoothUnion
 * gains a default blend radius; switching off it drops the radius argument.
 */
export function setBooleanOp(root, id, op) {
  const target = findById(root, id);
  if (!target) return { error: `no scene node #${id}` };
  if (!BINARY_OPS.includes(target.op)) {
    return { error: `${target.op} is not a boolean operation` };
  }
  if (!BINARY_OPS.includes(op)) return { error: `unknown boolean operation "${op}"` };
  if (op === target.op) return { root };
  const args = op === 'smoothUnion' ? [DEFAULT_BLEND] : [];
  return { root: replaceById(root, id, { ...target, op, args, shape: null }) };
}
