import { describe, expect, it } from 'vitest';
import { applyTranslate, applyRotate, applyScale, pathTo, nodeAt } from './transformEdit.js';

function node(id, op, args = [], children = []) {
  return { id, op, args, children, shape: null };
}

describe('applyTranslate', () => {
  it('wraps a primitive in a new translate node', () => {
    const root = node(1, 'union', [], [
      node(2, 'sphere', [1]),
      node(3, 'box3', [1, 1, 1]),
    ]);
    const result = applyTranslate(root, 2, [1, 0, 0]);
    const wrapped = result.children[0];
    expect(wrapped.op).toBe('translate');
    expect(wrapped.args).toEqual([1, 0, 0]);
    expect(wrapped.children[0].op).toBe('sphere');
    expect(result.children[1]).toBe(root.children[1]);
  });

  it('folds into an existing translate node', () => {
    const root = node(1, 'union', [], [
      node(2, 'translate', [1, 0, 0], [node(3, 'sphere', [1])]),
      node(4, 'box3', [1, 1, 1]),
    ]);
    const result = applyTranslate(root, 2, [0.5, -1, 0]);
    const updated = result.children[0];
    expect(updated.op).toBe('translate');
    expect(updated.args).toEqual([1.5, -1, 0]);
    expect(updated.children[0].op).toBe('sphere');
  });

  it('rounds args to 4 decimals', () => {
    const root = node(1, 'sphere', [1]);
    const result = applyTranslate(root, 1, [0.1 + 0.2, 0, 0]);
    expect(result.args[0]).toBe(0.3);
  });

  it('returns root unchanged for missing id', () => {
    const root = node(1, 'sphere', [1]);
    expect(applyTranslate(root, 99, [1, 0, 0])).toBe(root);
  });

  it('is DAG-safe: shared node replaced via memo', () => {
    const shared = node(1, 'sphere', [1]);
    const root = node(3, 'union', [], [
      shared,
      node(2, 'translate', [1, 0, 0], [shared]),
    ]);
    const result = applyTranslate(root, 1, [0, 1, 0]);
    const left = result.children[0];
    const innerRight = result.children[1].children[0];
    expect(left).toBe(innerRight);
    expect(left.op).toBe('translate');
  });
});

describe('applyRotate', () => {
  it('wraps in a rotate node at the origin', () => {
    const root = node(1, 'sphere', [1]);
    const result = applyRotate(root, 1, [0, 1, 0], Math.PI / 2, [0, 0, 0]);
    expect(result.op).toBe('rotate');
    expect(result.args).toEqual([0, 1, 0, 1.5708]);
    expect(result.children[0].op).toBe('sphere');
  });

  it('adds compensating translate for non-origin pivot', () => {
    const root = node(1, 'sphere', [1]);
    const result = applyRotate(root, 1, [0, 1, 0], Math.PI / 2, [1, 0, 0]);
    expect(result.op).toBe('translate');
    expect(result.children[0].op).toBe('rotate');
    expect(result.children[0].children[0].op).toBe('sphere');
  });

  it('returns root unchanged for missing id', () => {
    const root = node(1, 'sphere', [1]);
    expect(applyRotate(root, 99, [0, 1, 0], 0.5, [0, 0, 0])).toBe(root);
  });
});

describe('applyScale', () => {
  it('uses uniformScale for equal factors', () => {
    const root = node(1, 'sphere', [1]);
    const result = applyScale(root, 1, [2, 2, 2], [0, 0, 0]);
    expect(result.op).toBe('uniformScale');
    expect(result.args).toEqual([2]);
    expect(result.children[0].op).toBe('sphere');
  });

  it('uses scale for unequal factors', () => {
    const root = node(1, 'sphere', [1]);
    const result = applyScale(root, 1, [1, 2, 1], [0, 0, 0]);
    expect(result.op).toBe('scale');
    expect(result.args).toEqual([1, 2, 1]);
  });

  it('adds compensating translate for non-origin pivot', () => {
    const root = node(1, 'sphere', [1]);
    const result = applyScale(root, 1, [2, 2, 2], [1, 0, 0]);
    expect(result.op).toBe('translate');
    expect(result.args[0]).toBeCloseTo(-1, 4);
    expect(result.children[0].op).toBe('uniformScale');
  });

  it('returns root unchanged for missing id', () => {
    const root = node(1, 'sphere', [1]);
    expect(applyScale(root, 99, [2, 2, 2], [0, 0, 0])).toBe(root);
  });
});

describe('pathTo', () => {
  it('returns [] for the root', () => {
    const root = node(1, 'sphere', [1]);
    expect(pathTo(root, 1)).toEqual([]);
  });

  it('returns correct path through a tree', () => {
    const target = node(4, 'cylinder', [0.3, 2]);
    const root = node(1, 'subtract', [], [
      node(2, 'union', [], [
        node(3, 'sphere', [1]),
        target,
      ]),
      node(5, 'box3', [1, 1, 1]),
    ]);
    expect(pathTo(root, 4)).toEqual([0, 1]);
  });

  it('returns null for missing id', () => {
    const root = node(1, 'sphere', [1]);
    expect(pathTo(root, 99)).toBeNull();
  });
});

describe('nodeAt', () => {
  it('returns root for empty path', () => {
    const root = node(1, 'sphere', [1]);
    expect(nodeAt(root, [])).toBe(root);
  });

  it('navigates to the correct node', () => {
    const target = node(4, 'cylinder', [0.3, 2]);
    const root = node(1, 'subtract', [], [
      node(2, 'union', [], [
        node(3, 'sphere', [1]),
        target,
      ]),
      node(5, 'box3', [1, 1, 1]),
    ]);
    expect(nodeAt(root, [0, 1])).toBe(target);
  });

  it('returns null for invalid path', () => {
    const root = node(1, 'sphere', [1]);
    expect(nodeAt(root, [0])).toBeNull();
  });

  it('round-trips with pathTo', () => {
    const target = node(3, 'sphere', [1]);
    const root = node(1, 'union', [], [
      node(2, 'translate', [1, 0, 0], [target]),
      node(4, 'box3', [1, 1, 1]),
    ]);
    const path = pathTo(root, 3);
    expect(nodeAt(root, path)).toBe(target);
  });
});
