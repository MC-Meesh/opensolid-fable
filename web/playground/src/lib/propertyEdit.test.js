import { describe, expect, it } from 'vitest';
import { BINARY_OPS, PRIMITIVE_OPS, UNARY_OPS, serializeTree } from './sceneTree.js';
import {
  BOOLEAN_CHOICES,
  DEFAULT_BLEND,
  OP_SPECS,
  clampDisplay,
  displayValue,
  opSpec,
  setBooleanOp,
  setNodeArg,
} from './propertyEdit.js';

// union(sphere, translate(box)) — ids 1..4, root is 4.
function sampleTree() {
  const sphere = { id: 1, op: 'sphere', args: [0.5], children: [], shape: null };
  const box = { id: 2, op: 'box3', args: [0.4, 0.3, 0.2], children: [], shape: null };
  const move = { id: 3, op: 'translate', args: [1, 0, 0], children: [box], shape: null };
  const root = { id: 4, op: 'union', args: [], children: [sphere, move], shape: null };
  return { sphere, box, move, root };
}

describe('OP_SPECS', () => {
  it('covers every op the tracer records', () => {
    for (const op of [...PRIMITIVE_OPS, ...UNARY_OPS, ...BINARY_OPS]) {
      expect(opSpec(op), `missing spec for ${op}`).toBeTruthy();
    }
  });

  it('field arg indices are unique within each op', () => {
    for (const [op, spec] of Object.entries(OP_SPECS)) {
      const indices = spec.groups.flatMap((g) => g.fields.map((f) => f.arg));
      expect(new Set(indices).size, `duplicate arg index in ${op}`).toBe(indices.length);
    }
  });

  it('boolean dropdown choices are all binary ops', () => {
    for (const choice of BOOLEAN_CHOICES) {
      expect(BINARY_OPS).toContain(choice.op);
    }
  });
});

describe('setNodeArg', () => {
  it('updates the argument and leaves unrelated branches untouched', () => {
    const { sphere, root } = sampleTree();
    const result = setNodeArg(root, 2, 1, 0.75);
    expect(result.error).toBeUndefined();
    const newBox = result.root.children[1].children[0];
    expect(newBox.args).toEqual([0.4, 0.75, 0.2]);
    // Structural sharing: the sphere branch is the same object.
    expect(result.root.children[0]).toBe(sphere);
  });

  it('clamps below the minimum for strictly-positive dimensions', () => {
    const { root } = sampleTree();
    const result = setNodeArg(root, 1, 0, -5);
    expect(result.root.children[0].args[0]).toBe(0.001);
  });

  it('returns the same tree when the value does not change', () => {
    const { root } = sampleTree();
    const result = setNodeArg(root, 1, 0, 0.5);
    expect(result.root).toBe(root);
  });

  it('rejects non-finite values', () => {
    const { root } = sampleTree();
    expect(setNodeArg(root, 1, 0, NaN).error).toMatch(/finite/);
    expect(setNodeArg(root, 1, 0, Infinity).error).toMatch(/finite/);
  });

  it('rejects unknown nodes and non-editable arguments', () => {
    const { root } = sampleTree();
    expect(setNodeArg(root, 99, 0, 1).error).toMatch(/no scene node/);
    expect(setNodeArg(root, 1, 1, 1).error).toMatch(/no editable argument/);
    expect(setNodeArg(root, 4, 0, 1).error).toMatch(/no editable argument/);
  });

  it('converts angle edits from degrees to stored radians', () => {
    const box = { id: 1, op: 'box3', args: [0.5, 0.5, 0.5], children: [], shape: null };
    const rot = { id: 2, op: 'rotate', args: [0, 0, 1, 0], children: [box], shape: null };
    const result = setNodeArg(rot, 2, 3, 90);
    expect(result.root.args[3]).toBeCloseTo(Math.PI / 2, 3);
  });

  it('serializes the edited tree back into script text', () => {
    const { root } = sampleTree();
    const result = setNodeArg(root, 1, 0, 0.75);
    expect(serializeTree(result.root)).toBe(
      'return Shape.sphere(0.75).union(Shape.box3(0.4, 0.3, 0.2).translate(1, 0, 0));\n'
    );
  });
});

describe('setBooleanOp', () => {
  it('switches the operation and keeps both children', () => {
    const { sphere, move, root } = sampleTree();
    const result = setBooleanOp(root, 4, 'subtract');
    expect(result.root.op).toBe('subtract');
    expect(result.root.args).toEqual([]);
    expect(result.root.children[0]).toBe(sphere);
    expect(result.root.children[1]).toBe(move);
  });

  it('adds a default blend radius when switching onto smoothUnion', () => {
    const { root } = sampleTree();
    const result = setBooleanOp(root, 4, 'smoothUnion');
    expect(result.root.op).toBe('smoothUnion');
    expect(result.root.args).toEqual([DEFAULT_BLEND]);
  });

  it('drops the blend radius when switching off smoothUnion', () => {
    const { root } = sampleTree();
    const smooth = setBooleanOp(root, 4, 'smoothUnion').root;
    const back = setBooleanOp(smooth, 4, 'intersect');
    expect(back.root.op).toBe('intersect');
    expect(back.root.args).toEqual([]);
  });

  it('is a no-op when the operation is unchanged', () => {
    const { root } = sampleTree();
    expect(setBooleanOp(root, 4, 'union').root).toBe(root);
  });

  it('rejects non-boolean nodes and unknown operations', () => {
    const { root } = sampleTree();
    expect(setBooleanOp(root, 1, 'subtract').error).toMatch(/not a boolean/);
    expect(setBooleanOp(root, 4, 'xor').error).toMatch(/unknown boolean/);
    expect(setBooleanOp(root, 99, 'union').error).toMatch(/no scene node/);
  });
});

describe('sweep specs', () => {
  it('covers extrude and revolve so feature-tree clicks open parameters', () => {
    expect(opSpec('extrude').kind).toBe('sweep');
    expect(opSpec('extrude').groups[0].fields[0].arg).toBe(0);
    expect(opSpec('revolve').kind).toBe('sweep');
    // Revolve angle is stored in degrees — no display conversion.
    expect(opSpec('revolve').groups[0].fields[0].toDisplay).toBeNull();
    expect(opSpec('revolve').groups[0].fields[0].max).toBe(360);
  });

  it('setNodeArg edits a sweep parameter and keeps the profile snapshot', () => {
    const profile = { start: [0, 0], segs: [{ x: 1, y: 0, bulge: 0 }] };
    const ext = { id: 1, op: 'extrude', args: [2], children: [], shape: null, profile };
    const result = setNodeArg(ext, 1, 0, 3.5);
    expect(result.error).toBeUndefined();
    expect(result.root.args).toEqual([3.5]);
    expect(result.root.profile).toBe(profile);
  });
});

describe('display helpers', () => {
  it('displayValue converts stored radians to degrees for angle fields', () => {
    const rot = { id: 1, op: 'rotate', args: [0, 0, 1, Math.PI], children: [], shape: null };
    const angleField = OP_SPECS.rotate.groups[1].fields[0];
    expect(displayValue(angleField, rot)).toBeCloseTo(180, 6);
  });

  it('clampDisplay enforces the field range', () => {
    const rField = OP_SPECS.sphere.groups[0].fields[0];
    expect(clampDisplay(rField, -1)).toBe(0.001);
    expect(clampDisplay(rField, 99999)).toBe(10000);
    expect(clampDisplay(rField, 0.5)).toBe(0.5);
  });
});
