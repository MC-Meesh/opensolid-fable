// Feature tree: the CAD feature-history view over the traced construction
// tree (SolidWorks FeatureManager convention).
//
// buildFeatures() flattens the tree reachable from the root into a
// chronological list of features — creation order, not CSG nesting — with
// SolidWorks-style default names (Box1, Extrude1, Union1, ...). A sweep
// feature owning a profile snapshot gets a nested Sketch child feature.
//
// pruneTree() removes features from a tree with bypass semantics, backing
// the eye/suppress toggles (view-only recompute) and feature deletion
// (committed through the normal script sync path).
//
// This module is a presentation layer: the script and the traced tree stay
// the source of truth. Kept free of React and WASM imports so it can be
// unit-tested directly (same pattern as sceneTree.js).

import { UNARY_OPS } from './sceneTree.js';

/** Display metadata per op: feature kind and type name used for numbering. */
export const FEATURE_META = {
  sphere: { kind: 'primitive', type: 'Sphere' },
  box3: { kind: 'primitive', type: 'Box' },
  roundedBox: { kind: 'primitive', type: 'RoundedBox' },
  cylinder: { kind: 'primitive', type: 'Cylinder' },
  torus: { kind: 'primitive', type: 'Torus' },
  capsule: { kind: 'primitive', type: 'Capsule' },
  extrude: { kind: 'sweep', type: 'Extrude' },
  revolve: { kind: 'sweep', type: 'Revolve' },
  union: { kind: 'boolean', type: 'Union' },
  intersect: { kind: 'boolean', type: 'Intersect' },
  subtract: { kind: 'boolean', type: 'Subtract' },
  smoothUnion: { kind: 'boolean', type: 'SmoothUnion' },
  translate: { kind: 'transform', type: 'Translate' },
  rotate: { kind: 'transform', type: 'Rotate' },
  scale: { kind: 'transform', type: 'Scale' },
  uniformScale: { kind: 'transform', type: 'Scale' },
};

function metaFor(op) {
  return FEATURE_META[op] ?? { kind: 'unknown', type: op };
}

/** Nodes reachable from `root`, deduped, in creation (id) order. */
function reachableNodes(root) {
  const seen = new Map();
  const walk = (node) => {
    if (seen.has(node.id)) return;
    seen.set(node.id, node);
    node.children.forEach(walk);
  };
  walk(root);
  return [...seen.values()].sort((a, b) => a.id - b.id);
}

/**
 * Build the chronological feature list for a traced tree.
 *
 * Returns `[{ key, id, node, op, kind, type, name, defaultName, depth,
 * sketch? }]` in creation order. Sweep features carrying a profile snapshot
 * are followed by a nested sketch feature (`kind: 'sketch'`, `depth: 1`,
 * `node` = the owning sweep node, `parentKey` = the sweep's key).
 *
 * `key` is `type:ordinal` (e.g. `extrude:1`) — deterministic for a given
 * script, so rename / visibility maps keyed by it survive re-evaluation.
 * `names` maps keys to user renames; unmapped features get `defaultName`.
 */
export function buildFeatures(root, names = {}) {
  if (!root) return [];
  const counters = {};
  const nextOrdinal = (type) => (counters[type] = (counters[type] ?? 0) + 1);

  const features = [];
  for (const node of reachableNodes(root)) {
    const { kind, type } = metaFor(node.op);
    const key = `${type.toLowerCase()}:${nextOrdinal(type)}`;
    const defaultName = `${type}${counters[type]}`;
    features.push({
      key,
      id: node.id,
      node,
      op: node.op,
      kind,
      type,
      name: names[key] ?? defaultName,
      defaultName,
      depth: 0,
    });
    if (node.profile) {
      const sketchKey = `sketch:${nextOrdinal('Sketch')}`;
      const sketchDefault = `Sketch${counters.Sketch}`;
      features.push({
        key: sketchKey,
        id: node.id,
        node,
        op: node.op,
        kind: 'sketch',
        type: 'Sketch',
        name: names[sketchKey] ?? sketchDefault,
        defaultName: sketchDefault,
        depth: 1,
        parentKey: key,
      });
    }
  }
  return features;
}

/** Node ids for the feature keys that resolve in the current feature list.
 * Sketch keys resolve to their owning sweep node. */
export function resolveKeys(features, keys) {
  const wanted = new Set(keys);
  const ids = new Set();
  for (const f of features) {
    if (wanted.has(f.key)) ids.add(f.id);
  }
  return ids;
}

/**
 * Remove the features in `ids` from a tree with bypass semantics:
 *  - a removed unary or binary op is skipped — its receiver (first child)
 *    takes its place, so downstream history survives;
 *  - a removed leaf collapses any boolean referencing it onto the surviving
 *    operand, and a transform left with nothing to act on vanishes.
 *
 * Returns the new root (plain node data, `shape` stripped on copies), the
 * original root unchanged when nothing resolved, or `null` when nothing
 * remains. Shared (DAG) subtrees are rewritten once.
 */
export function pruneTree(root, ids) {
  if (!root || ids.size === 0) return root;
  const memo = new Map();

  const walk = (node) => {
    if (memo.has(node.id)) return memo.get(node.id);
    let result;
    if (ids.has(node.id)) {
      result = node.children.length > 0 ? walk(node.children[0]) : null;
    } else if (node.children.length === 0) {
      result = node;
    } else {
      const children = node.children.map(walk);
      if (UNARY_OPS.includes(node.op) || node.children.length === 1) {
        result = children[0] === null ? null : rebuilt(node, children);
      } else {
        const [receiver, other] = children;
        if (receiver === null) result = other;
        else if (other === null) result = receiver;
        else result = rebuilt(node, [receiver, other]);
      }
    }
    memo.set(node.id, result);
    return result;
  };

  const rebuilt = (node, children) =>
    children.every((c, i) => c === node.children[i])
      ? node
      : { ...node, children, shape: null };

  return walk(root);
}
