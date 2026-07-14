// Store-sync hardening (of-4eh.19): the traced construction tree is the
// single source of truth; the script is a view regenerated in canonical form
// on any GUI edit, and script edits reparse into the tree. This module holds
// the WASM-free machinery behind that contract:
//
//   reparseTree()            script -> plain-data tree via a stub Shape API,
//                            so consistency checks and tests never need WASM.
//   checkConsistency()       do a script and a rendered tree describe the
//                            same model? (canonical serializations compared)
//   assertStoreConsistency() dev-mode wrapper that console.errors divergence.
//   addPrimitiveNode()       the palette "add shape" as a store mutation.
//   hashTree()               stable hash of a tree's canonical form.
//
// Kept free of React and WASM imports so it can be unit-tested directly
// (same pattern as sceneTree.js).

import {
  BINARY_OPS,
  PRIMITIVE_OPS,
  SWEEP_OPS,
  UNARY_OPS,
  freeNodes,
  runTracedScript,
  serializeTree,
} from './sceneTree.js';

/**
 * A stand-in Shape/Profile API implementing the full scripting surface with
 * inert objects. `live` tracks every instance not yet freed, so tests can
 * prove that evaluation leaves no orphaned scene nodes behind.
 */
export function createStubApi() {
  const live = new Set();

  class StubShape {
    constructor() {
      live.add(this);
    }
    free() {
      live.delete(this);
    }
  }
  for (const op of [...PRIMITIVE_OPS, ...SWEEP_OPS]) {
    StubShape[op] = () => new StubShape();
  }
  StubShape.sweep = () => new StubShape();
  StubShape.loft = () => new StubShape();
  for (const op of [...UNARY_OPS, ...BINARY_OPS]) {
    StubShape.prototype[op] = () => new StubShape();
  }

  class StubProfile {
    constructor() {
      live.add(this);
    }
    arcTo() {}
    close() {}
    free() {
      live.delete(this);
    }
  }

  class StubPath {
    constructor() {
      live.add(this);
    }
    lineTo() {}
    free() {
      live.delete(this);
    }
  }

  return { Shape: StubShape, Profile: StubProfile, Path: StubPath, live };
}

/**
 * Parse a script back into a plain-data construction tree by evaluating it
 * against the stub API. Stub shapes are freed immediately (the returned nodes
 * carry `shape: null`), so the result is pure data.
 *
 * Returns `{ root, nodes, leaked }` — `leaked` is the number of stub
 * instances still alive after freeing, which must be 0 unless the script
 * itself smuggled shapes out of scope. Throws when the script fails to run.
 */
export function reparseTree(source) {
  const api = createStubApi();
  const traced = runTracedScript(source, api.Shape, api.Profile, api.Path);
  freeNodes(traced.nodes);
  return { root: traced.root, nodes: traced.nodes, leaked: api.live.size };
}

/** FNV-1a hash of a string, hex-encoded. */
function fnv1a(text) {
  let h = 0x811c9dc5;
  for (let i = 0; i < text.length; i += 1) {
    h ^= text.charCodeAt(i);
    h = Math.imul(h, 0x01000193);
  }
  return (h >>> 0).toString(16).padStart(8, '0');
}

/** Stable hash of a tree's canonical serialization. */
export function hashTree(root) {
  return fnv1a(serializeTree(root));
}

/**
 * Check that `script` (the view) and `root` (the store driving the rendered
 * scene) describe the same model: the script is reparsed with the stub API
 * and both trees are compared in canonical serialized form, so formatting,
 * variable names, and hand-written non-canonical code never false-positive.
 *
 * Returns `{ ok, expected, actual }` (canonical texts) or `{ ok: false,
 * error }` when the script no longer evaluates.
 */
export function checkConsistency(script, root) {
  let reparsed;
  try {
    reparsed = reparseTree(script);
  } catch (err) {
    return { ok: false, error: String(err) };
  }
  const expected = serializeTree(reparsed.root);
  const actual = serializeTree(root);
  return { ok: expected === actual, expected, actual };
}

/**
 * Dev-mode divergence tripwire: console.errors (and returns false) when the
 * script view and the rendered tree disagree — which means some mutation
 * path bypassed the single store commit. Scripts with non-deterministic
 * values (e.g. Math.random()) can trip this legitimately; it never throws.
 */
export function assertStoreConsistency(script, root, log = console) {
  const result = checkConsistency(script, root);
  if (!result.ok) {
    log.error(
      'OpenSolid store divergence: the script view and the rendered scene ' +
        'no longer describe the same model. A mutation path bypassed the ' +
        'store commit.',
      result.error ?? { script: result.expected, scene: result.actual }
    );
  }
  return result.ok;
}

/**
 * The palette "add primitive" as a store mutation: the new shape unioned
 * onto the existing tree (or the whole tree when there is none). Synthetic
 * ids are negative so they can't collide with traced ids; like
 * sweepTreeNode, the result is only valid to serialize once and re-evaluate.
 */
export function addPrimitiveNode(root, ctor, args) {
  const prim = { id: -1, op: ctor, args: [...args], children: [], shape: null };
  if (!root) return prim;
  return { id: -2, op: 'union', args: [], children: [root, prim], shape: null };
}

/**
 * Graft a prebuilt feature node (e.g. `defaultSweepNode`/`defaultLoftNode`,
 * which carry `profile`/`path`/`profile2` snapshots) onto the tree, unioned
 * onto the existing root. Like `addPrimitiveNode`, synthetic ids are negative
 * and the result is only valid to serialize once and re-evaluate.
 */
export function addFeatureNode(root, node) {
  const feature = { ...node, id: -1, children: [], shape: null };
  if (!root) return feature;
  return { id: -2, op: 'union', args: [], children: [root, feature], shape: null };
}
