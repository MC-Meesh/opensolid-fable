import { describe, expect, it } from 'vitest';
import { featureArgs, featureTreeNode, findNodeById } from './pattern.js';
import { serializeTree } from './sceneTree.js';

// Plain-data traced nodes (no WASM), same shape the tracer produces.
const leaf = (id, op, args = []) => ({ id, op, args, children: [], shape: null });

describe('featureArgs', () => {
  it('orders linear-pattern args as [dx, dy, dz, count]', () => {
    expect(
      featureArgs({ kind: 'linearPattern', dx: 2, dy: 0, dz: 0, count: 3 })
    ).toEqual([2, 0, 0, 3]);
  });

  it('orders circular-pattern args as axis, center, count, angle', () => {
    expect(
      featureArgs({
        kind: 'circularPattern',
        ax: 0,
        ay: 1,
        az: 0,
        cx: 0,
        cy: 0,
        cz: 0,
        count: 6,
        angleDeg: 360,
      })
    ).toEqual([0, 1, 0, 0, 0, 0, 6, 360]);
  });

  it('orders mirror args as normal then point', () => {
    expect(
      featureArgs({ kind: 'mirror', nx: 1, ny: 0, nz: 0, px: 0.5, py: 0, pz: 0 })
    ).toEqual([1, 0, 0, 0.5, 0, 0]);
  });

  it('rounds noisy floats and rejects unknown kinds', () => {
    expect(featureArgs({ kind: 'mirror', nx: 0.1 + 0.2, ny: 0, nz: 0, px: 0, py: 0, pz: 0 })[0]).toBe(0.3);
    expect(featureArgs({ kind: 'bogus' })).toBeNull();
  });
});

describe('findNodeById', () => {
  it('finds a nested node and returns null for a miss', () => {
    const root = { id: 3, op: 'union', args: [], children: [leaf(1, 'sphere', [1]), leaf(2, 'box3', [1, 1, 1])], shape: null };
    expect(findNodeById(root, 2).op).toBe('box3');
    expect(findNodeById(root, 99)).toBeNull();
    expect(findNodeById(null, 1)).toBeNull();
  });
});

describe('featureTreeNode', () => {
  it('wraps the target in a unary pattern node with a fresh id', () => {
    const target = leaf(1, 'box3', [0.5, 0.5, 0.5]);
    const root = featureTreeNode(target, {
      kind: 'linearPattern',
      targetId: 1,
      dx: 2,
      dy: 0,
      dz: 0,
      count: 3,
    });
    expect(root.op).toBe('linearPattern');
    expect(root.args).toEqual([2, 0, 0, 3]);
    expect(root.id).toBe(2); // maxId(1) + 1
    expect(root.children[0]).toBe(target);
  });

  it('wraps a node nested inside a larger tree in place', () => {
    const box = leaf(2, 'box3', [1, 1, 1]);
    const root = { id: 3, op: 'union', args: [], children: [leaf(1, 'sphere', [1]), box], shape: null };
    const next = featureTreeNode(root, {
      kind: 'mirror',
      targetId: 2,
      nx: 1,
      ny: 0,
      nz: 0,
      px: 0,
      py: 0,
      pz: 0,
    });
    // Root stays a union; the box child is now wrapped by a mirror node.
    expect(next.op).toBe('union');
    expect(next.children[1].op).toBe('mirror');
    expect(next.children[1].children[0]).toBe(box);
    expect(next.children[0].op).toBe('sphere'); // sibling untouched
  });

  it('serializes to a canonical method call that round-trips', () => {
    const target = leaf(1, 'sphere', [1]);
    const root = featureTreeNode(target, {
      kind: 'circularPattern',
      targetId: 1,
      ax: 0,
      ay: 1,
      az: 0,
      cx: 0,
      cy: 0,
      cz: 0,
      count: 6,
      angleDeg: 360,
    });
    const script = serializeTree(root);
    expect(script).toContain('Shape.sphere(1).circularPattern(0, 1, 0, 0, 0, 0, 6, 360)');
  });

  it('returns the tree unchanged when the target is missing', () => {
    const root = leaf(1, 'sphere', [1]);
    expect(featureTreeNode(root, { kind: 'mirror', targetId: 99, nx: 1, ny: 0, nz: 0, px: 0, py: 0, pz: 0 })).toBe(root);
  });
});
