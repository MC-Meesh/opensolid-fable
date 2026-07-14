// Edge-selective fillet/chamfer: turning a picked feature edge into a tree
// rewrite (of-rpo). The picked edge is a crease on the union of two operand
// bodies; filleting/chamfering it replaces that `union` node with a
// `filletEdge` / `chamferEdge` node over the same two operands. The kernel
// blend is always a *union* blend (see crates/opensolid-frep + the WASM
// bindings), so we only retarget union-family booleans.
//
// Pure JS (no React, no three.js, no WASM) so it is unit-testable with a
// stand-in Shape whose `.distance()` is controlled (same pattern as picking.js
// / sweep.js).

/** Boolean ops whose surface is a *union* of its operands, so the crease the
 * user picked can be re-expressed as an edge blend of those two operands. */
const UNION_LIKE = new Set(['union', 'smoothUnion']);

/**
 * Find the boolean node whose crease the seed point lies on: a union-family
 * node both of whose operand shapes pass through `seed` (|distance| within
 * `tol`). Walks the whole tree and returns the best-fitting match
 * `{ node, a, b }` (the node to retarget and its two operand child nodes), or
 * `null` when no union edge is near the seed.
 *
 * `tol` is an absolute distance; seeds come from Newton-snapped mesh crease
 * vertices, so they sit within a mesh-cell of both surfaces.
 */
export function findBlendTarget(root, seed, tol = 1e-2) {
  let best = null;
  let bestScore = Infinity;
  const seen = new Set();

  const walk = (node) => {
    if (!node || seen.has(node.id)) return;
    seen.add(node.id);
    if (UNION_LIKE.has(node.op) && node.children.length === 2) {
      const [a, b] = node.children;
      const da = Math.abs(a.shape.distance(...seed));
      const db = Math.abs(b.shape.distance(...seed));
      if (da <= tol && db <= tol && da + db < bestScore) {
        bestScore = da + db;
        best = { node, a, b };
      }
    }
    node.children.forEach(walk);
  };

  walk(root);
  return best;
}

const opFor = (mode) => (mode === 'chamfer' ? 'chamferEdge' : 'filletEdge');

/**
 * Build a live preview shape for a fillet/chamfer of the picked edge:
 * `aShape.filletEdge(bShape, radius, edge)` (or `chamferEdge`). `aShape` /
 * `bShape` are the operand bodies' retained shapes; the caller owns and frees
 * the returned shape.
 */
export function buildFilletShape(aShape, bShape, { mode, radius, edge }) {
  return aShape[opFor(mode)](bShape, radius, Array.from(edge));
}

/**
 * Clone `root`, replacing the node with id `targetId` by a `filletEdge` /
 * `chamferEdge` node over that node's two children (the operands), with args
 * `[radius, edge]`. Ancestors on the path are shallow-cloned with fresh
 * negative ids (so they can't collide with traced ids); untouched subtrees are
 * shared by reference. Returns the new root — plain node data ready to
 * serialize and re-evaluate. Throws if `targetId` isn't found or isn't a
 * two-operand boolean.
 */
export function filletTreeNode(root, { targetId, mode, radius, edge }) {
  let nextId = -1;
  let replaced = false;
  const polyline = Array.from(edge);

  const rewrite = (node) => {
    if (node.id === targetId) {
      if (node.children.length !== 2) {
        throw new Error('edge blend target must have two operands');
      }
      replaced = true;
      return {
        id: nextId--,
        op: opFor(mode),
        args: [radius, polyline],
        children: node.children,
      };
    }
    if (node.children.length === 0) return node;
    const children = node.children.map(rewrite);
    if (children.every((c, i) => c === node.children[i])) return node;
    return { id: nextId--, op: node.op, args: node.args, children, profile: node.profile };
  };

  const out = rewrite(root);
  if (!replaced) throw new Error(`no blend target node with id ${targetId}`);
  return out;
}
