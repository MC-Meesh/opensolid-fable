import { BINARY_OPS } from './sceneTree.js';

const binarySet = new Set(BINARY_OPS);

function hasBinaryOp(node) {
  if (binarySet.has(node.op)) return true;
  return node.children.some(hasBinaryOp);
}

export function pickCandidates(root) {
  const candidates = [];
  const seen = new Set();

  function walk(node) {
    if (seen.has(node.id)) return;
    seen.add(node.id);

    if (binarySet.has(node.op)) {
      node.children.forEach(walk);
    } else if (!hasBinaryOp(node)) {
      candidates.push(node);
    } else {
      node.children.forEach(walk);
    }
  }

  walk(root);
  return candidates;
}

export function pickNodeAt(candidates, point) {
  let best = null;
  let bestDist = Infinity;
  for (const c of candidates) {
    const d = Math.abs(c.shape.distance(...point));
    if (d < bestDist) {
      bestDist = d;
      best = c;
    }
  }
  return best;
}
