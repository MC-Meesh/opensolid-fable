// Delete a branch from the traced scene tree (the SolidWorks "select body,
// press Delete" gesture).
//
// The deleted branch is the target node plus any unary transform wrappers
// directly above it; the nearest binary boolean ancestor is then replaced by
// its other operand. Deleting the last remaining body is refused — an empty
// tree has no script serialization.
//
// Kept free of React and WASM imports so it can be unit-tested directly.

import { BINARY_OPS } from './sceneTree.js';
import { nodeAt, pathTo, replaceById } from './transformEdit.js';

const binarySet = new Set(BINARY_OPS);

/**
 * Remove the branch containing node `id`. Returns `{ root }` with a new tree
 * or `{ error }` when the node is missing or is the only body.
 */
export function deleteNode(root, id) {
  const path = pathTo(root, id);
  if (path === null) return { error: `no scene node #${id}` };

  // Climb through unary wrappers to the top of the branch being removed.
  let branchPath = path;
  while (branchPath.length > 0) {
    const parent = nodeAt(root, branchPath.slice(0, -1));
    if (binarySet.has(parent.op)) break;
    branchPath = branchPath.slice(0, -1);
  }

  if (branchPath.length === 0) {
    return { error: 'Cannot delete the only body — edit the script instead.' };
  }

  const parent = nodeAt(root, branchPath.slice(0, -1));
  const sibling = parent.children[1 - branchPath[branchPath.length - 1]];
  return { root: replaceById(root, parent.id, sibling) };
}
