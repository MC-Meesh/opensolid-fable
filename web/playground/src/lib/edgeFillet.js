// Map a picked boolean edge back onto the construction tree and rewrite the
// combining union into an edge-selective fillet/chamfer (of-rpo).
//
// pickEdge() (edgePick.js) gives the two operand faces bordering a crease.
// This module resolves each face to its operand body, finds the union node in
// the tree that combines them, and replaces that node with a `filletEdge` /
// `chamferEdge` node carrying the radius and the crease polyline. Because the
// kernel's blend is windowed to the polyline (BooleanKind::Union — the MVP
// only blends union edges), the rewrite is local: `union(L, R)` becomes
// `L.filletEdge(R, radius, edge)`, and every untouched edge stays sharp.
//
// Kept free of React and WASM imports so it can be unit-tested with plain
// tree data (same pattern as transformEdit.js / featureTree.js).

import { pickCandidates, pickNodeAt } from './picking.js';
import { replaceById } from './transformEdit.js';

/** Nearest pickable body node to a world point, or null (see picking.js). */
export function nearestBody(root, point) {
  return pickNodeAt(pickCandidates(root), point);
}

function contains(node, id) {
  if (node.id === id) return true;
  return node.children.some((c) => contains(c, id));
}

/**
 * The deepest `union` node whose two subtrees separate bodies `aId` and `bId`
 * (one under each child) — the boolean that produced the picked edge. Returns
 * that node, or null when no union combines the two bodies (e.g. they meet
 * across a subtract/intersect, which the F-Rep fillet MVP does not blend, or
 * they are the same body — a primitive-intrinsic edge, also out of scope).
 */
export function findUnionSeparating(root, aId, bId) {
  if (aId === bId) return null;
  let found = null;
  const walk = (node) => {
    node.children.forEach(walk);
    if (found) return;
    if (node.op === 'union' && node.children.length === 2) {
      const [l, r] = node.children;
      const split =
        (contains(l, aId) && contains(r, bId)) || (contains(l, bId) && contains(r, aId));
      if (split) found = node;
    }
  };
  walk(root);
  return found;
}

/**
 * Replace the union node `unionId` with an edge blend. `mode` is `'fillet'`
 * or `'chamfer'`, `radius` the blend radius/setback, `edge` the flat
 * `[x, y, z, …]` crease polyline. Keeps the node's id (feature-key stable) and
 * its two children; returns the new root. The `shape` fields are stripped on
 * rewritten nodes so a re-evaluation rebuilds them.
 */
export function replaceWithBlend(root, unionId, mode, radius, edge) {
  const op = mode === 'chamfer' ? 'chamferEdge' : 'filletEdge';
  const findNode = (node) => {
    if (node.id === unionId) return node;
    for (const c of node.children) {
      const hit = findNode(c);
      if (hit) return hit;
    }
    return null;
  };
  const target = findNode(root);
  if (!target) return root;
  const replacement = {
    ...target,
    op,
    args: [radius],
    edge: [...edge],
    shape: null,
  };
  return replaceById(root, unionId, replacement);
}

/**
 * Resolve a picked edge (from pickEdge) to a rewrite target on the tree.
 *
 * Returns `{ ok: true, unionId, bodyA, bodyB }` naming the union node to
 * rewrite, or `{ ok: false, reason }` when the edge is not a union edge
 * between two distinct bodies.
 */
export function resolveEdgeTarget(root, pick) {
  const bodyA = nearestBody(root, pick.regionA.plane.origin);
  const bodyB = nearestBody(root, pick.regionB.plane.origin);
  if (!bodyA || !bodyB) {
    return { ok: false, reason: 'could not resolve the faces to bodies' };
  }
  if (bodyA.id === bodyB.id) {
    return {
      ok: false,
      reason: 'This is an edge of a single body — F-Rep fillet needs an edge between two bodies (MVP)',
    };
  }
  const union = findUnionSeparating(root, bodyA.id, bodyB.id);
  if (!union) {
    return {
      ok: false,
      reason: 'Only edges where two bodies are unioned can be filleted yet (MVP)',
    };
  }
  return { ok: true, unionId: union.id, bodyA, bodyB };
}
