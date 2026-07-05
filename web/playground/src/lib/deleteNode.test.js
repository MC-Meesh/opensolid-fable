import { describe, expect, it } from 'vitest';
import { deleteNode } from './deleteNode.js';
import { serializeTree } from './sceneTree.js';

let nextId;
function n(op, args, children = []) {
  return { id: nextId++, op, args, children, shape: null };
}

function build() {
  nextId = 1;
  const sphere = n('sphere', [1]);
  const box = n('box3', [1, 0.5, 0.8]);
  const movedBox = n('translate', [2, 0, 0], [box]);
  const union = n('union', [], [sphere, movedBox]);
  const cyl = n('cylinder', [0.4, 2]);
  const root = n('subtract', [], [union, cyl]);
  return { sphere, box, movedBox, union, cyl, root };
}

describe('deleteNode', () => {
  it('deleting one boolean operand promotes the sibling', () => {
    const { sphere, movedBox, root } = build();
    const result = deleteNode(root, sphere.id);
    expect(result.error).toBeUndefined();
    expect(result.root.op).toBe('subtract');
    expect(result.root.children[0]).toBe(movedBox);
  });

  it('deleting a node inside unary wrappers removes the whole wrapped branch', () => {
    const { box, sphere, root } = build();
    const result = deleteNode(root, box.id);
    // The translate wrapper goes with the box; the union collapses to the sphere.
    expect(result.root.children[0]).toBe(sphere);
  });

  it('deleting the subtracted tool restores the receiver', () => {
    const { union, cyl, root } = build();
    const result = deleteNode(root, cyl.id);
    expect(result.root).toBe(union);
  });

  it('deleting a boolean node removes it and its subtree', () => {
    const { union, cyl, root } = build();
    const result = deleteNode(root, union.id);
    expect(result.root).toBe(cyl);
  });

  it('refuses to delete the only body', () => {
    nextId = 1;
    const lone = n('translate', [1, 0, 0], [n('sphere', [1])]);
    expect(deleteNode(lone, lone.children[0].id).error).toMatch(/only body/);
    expect(deleteNode(lone, lone.id).error).toMatch(/only body/);
  });

  it('errors on unknown ids', () => {
    const { root } = build();
    expect(deleteNode(root, 999).error).toMatch(/no scene node/);
  });

  it('does not mutate the original tree and serializes cleanly', () => {
    const { root, sphere } = build();
    const before = serializeTree(root);
    const result = deleteNode(root, sphere.id);
    expect(serializeTree(root)).toBe(before);
    expect(serializeTree(result.root)).toContain('subtract');
    expect(serializeTree(result.root)).not.toContain('sphere');
  });

  it('handles shared subtrees (DAG): deleting one operand of a self-union', () => {
    nextId = 1;
    const shared = n('sphere', [1]);
    const moved = n('translate', [1, 0, 0], [shared]);
    const union = n('union', [], [shared, moved]);
    const result = deleteNode(union, moved.id);
    expect(result.root).toBe(shared);
  });
});
