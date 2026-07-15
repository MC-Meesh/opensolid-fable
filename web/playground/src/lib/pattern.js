// Pattern / mirror features: graft a unary LinearPattern, CircularPattern, or
// Mirror node onto the traced tree, wrapping the selected body.
//
// A "pending feature" is a plain object the GUI panel edits live; when applied
// it becomes a one-child scene-tree node (op + numeric args, the selected node
// as its child), committed through the normal script-sync path exactly like a
// transform (see applyTranslate / applyRotate in transformEdit.js). Kept free
// of React and WASM imports so it can be unit-tested directly.

import { maxId, replaceById } from './transformEdit.js';

function round4(v) {
  return Math.round(v * 10000) / 10000;
}

/** The node with the given id, or null. */
export function findNodeById(node, id) {
  if (!node) return null;
  if (node.id === id) return node;
  for (const child of node.children) {
    const found = findNodeById(child, id);
    if (found) return found;
  }
  return null;
}

/**
 * Numeric args for a pending feature, in the order the WASM method (and the
 * canonical script) expect. Returns null for an unknown kind.
 *
 * - linearPattern:   [dx, dy, dz, count]
 * - circularPattern: [ax, ay, az, cx, cy, cz, count, angleDeg]
 * - mirror:          [nx, ny, nz, px, py, pz]
 */
export function featureArgs(feature) {
  switch (feature.kind) {
    case 'linearPattern':
      return [feature.dx, feature.dy, feature.dz, feature.count].map(round4);
    case 'circularPattern':
      return [
        feature.ax,
        feature.ay,
        feature.az,
        feature.cx,
        feature.cy,
        feature.cz,
        feature.count,
        feature.angleDeg,
      ].map(round4);
    case 'mirror':
      return [feature.nx, feature.ny, feature.nz, feature.px, feature.py, feature.pz].map(round4);
    default:
      return null;
  }
}

/**
 * Wrap the feature's target node in a new unary pattern/mirror node and return
 * the new root. When `targetId` is absent from the tree the tree is returned
 * unchanged. `feature.kind` is used as the op name, matching the WASM method
 * and the UNARY_OPS registry.
 */
export function featureTreeNode(root, feature) {
  if (!root) return root;
  const target = findNodeById(root, feature.targetId);
  const args = featureArgs(feature);
  if (!target || !args) return root;
  const wrapper = {
    id: maxId(root) + 1,
    op: feature.kind,
    args,
    children: [target],
    shape: null,
  };
  return replaceById(root, feature.targetId, wrapper);
}
