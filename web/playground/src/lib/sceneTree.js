// Scene tree: the shared model of a script's shape construction graph.
//
// This module is the single source of truth that both the script and the GUI
// read/write:
//   script -> model : runTracedScript() executes the script with a tracing
//                     Shape wrapper that records every operation as a node
//                     while delegating to the real Shape class.
//   model -> script : serializeTree() emits a canonical script for any tree,
//                     hoisting shared subtrees into `const` bindings.
//
// Kept free of React and WASM imports so it can be unit-tested with a
// stand-in Shape class (same pattern as runScript.js).

import { runScript } from './runScript.js';

// Static constructors on Shape. Nodes for these have no children.
export const PRIMITIVE_OPS = [
  'sphere',
  'box3',
  'roundedBox',
  'cylinder',
  'torus',
  'capsule',
];

// Instance methods taking only numeric args. One child: the receiver.
export const UNARY_OPS = ['translate', 'rotate', 'scale', 'uniformScale'];

// Instance methods taking another shape (plus optional numeric args).
// Two children: the receiver, then the other shape.
export const BINARY_OPS = ['union', 'intersect', 'subtract', 'smoothUnion'];

const OP_LABELS = {
  sphere: 'Sphere',
  box3: 'Box3',
  roundedBox: 'Rounded Box',
  cylinder: 'Cylinder',
  torus: 'Torus',
  capsule: 'Capsule',
  translate: 'Translate',
  rotate: 'Rotate',
  scale: 'Scale',
  uniformScale: 'Uniform Scale',
  union: 'Union',
  intersect: 'Intersect',
  subtract: 'Subtract',
  smoothUnion: 'Smooth Union',
};

function formatArg(value) {
  if (typeof value === 'number' && Number.isFinite(value)) {
    // Trim float noise for display without changing round values.
    return String(Number(value.toPrecision(6)));
  }
  return String(value);
}

/** Display label for a node, e.g. 'Box3 [1, 0.55, 0.8]' or 'Subtract'. */
export function nodeLabel(node) {
  const name = OP_LABELS[node.op] ?? node.op;
  if (node.args.length === 0) return name;
  return `${name} [${node.args.map(formatArg).join(', ')}]`;
}

/**
 * Create a Shape-compatible tracing class that records a construction node
 * for every operation while delegating to `ShapeClass`.
 *
 * Returns `{ TracingShape, nodes }` where `nodes` accumulates every node
 * created (including ones a script builds but never uses), so callers can
 * free the underlying shapes.
 *
 * Node shape: `{ id, op, args, children, shape }` — `args` are the numeric
 * arguments, `children` reference other nodes, and `shape` is the retained
 * `ShapeClass` instance for that intermediate result.
 */
export function createTracer(ShapeClass) {
  let nextId = 1;
  const nodes = [];

  class TracingShape {
    constructor(node) {
      this.node = node;
    }
    get shape() {
      return this.node.shape;
    }
  }

  const record = (op, args, children, shape) => {
    const node = { id: nextId++, op, args, children, shape };
    nodes.push(node);
    return new TracingShape(node);
  };

  for (const op of PRIMITIVE_OPS) {
    TracingShape[op] = (...args) => record(op, args, [], ShapeClass[op](...args));
  }

  for (const op of UNARY_OPS) {
    TracingShape.prototype[op] = function (...args) {
      return record(op, args, [this.node], this.node.shape[op](...args));
    };
  }

  for (const op of BINARY_OPS) {
    TracingShape.prototype[op] = function (other, ...args) {
      if (!(other instanceof TracingShape)) {
        throw new Error(`.${op}(...) expects a Shape as its first argument`);
      }
      // Drop a trailing explicit `undefined` so optional args (e.g. the
      // smoothUnion radius) serialize back without it.
      while (args.length > 0 && args[args.length - 1] === undefined) args.pop();
      return record(
        op,
        args,
        [this.node, other.node],
        this.node.shape[op](other.node.shape, ...args)
      );
    };
  }

  return { TracingShape, nodes };
}

/**
 * Evaluate a script with construction tracing.
 *
 * Returns `{ root, nodes }`: `root` is the node of the returned shape (its
 * `.shape` is the real `ShapeClass` instance), `nodes` is every node created.
 * On error, any shapes created before the failure are freed, then the error
 * is rethrown.
 */
export function runTracedScript(source, ShapeClass) {
  const { TracingShape, nodes } = createTracer(ShapeClass);
  let result;
  try {
    result = runScript(source, TracingShape);
  } catch (err) {
    freeNodes(nodes);
    throw err;
  }
  return { root: result.node, nodes };
}

/** Free the retained shape of every node (safe to call more than once). */
export function freeNodes(nodes) {
  for (const node of nodes) {
    if (node.shape && typeof node.shape.free === 'function') {
      node.shape.free();
    }
    node.shape = null;
  }
}

/**
 * Emit a canonical script for the tree rooted at `root`.
 *
 * Single-use nodes are inlined into their parent expression; nodes referenced
 * more than once are hoisted into `const s<N> = ...;` bindings so the DAG
 * structure survives a round trip through the script.
 */
export function serializeTree(root) {
  const refs = new Map();
  const countRefs = (node) => {
    const seen = (refs.get(node.id) ?? 0) + 1;
    refs.set(node.id, seen);
    if (seen === 1) node.children.forEach(countRefs);
  };
  countRefs(root);

  const names = new Map();
  const lines = [];

  const exprOf = (node) => {
    if (names.has(node.id)) return names.get(node.id);
    const args = node.args.map(String);
    let text;
    if (node.children.length === 0) {
      text = `Shape.${node.op}(${args.join(', ')})`;
    } else if (node.children.length === 1) {
      text = `${exprOf(node.children[0])}.${node.op}(${args.join(', ')})`;
    } else {
      const [receiver, other] = node.children;
      const rest = args.length > 0 ? `, ${args.join(', ')}` : '';
      text = `${exprOf(receiver)}.${node.op}(${exprOf(other)}${rest})`;
    }
    if (node !== root && refs.get(node.id) > 1) {
      const name = `s${names.size + 1}`;
      names.set(node.id, name);
      lines.push(`const ${name} = ${text};`);
      return name;
    }
    return text;
  };

  lines.push(`return ${exprOf(root)};`);
  return `${lines.join('\n')}\n`;
}
