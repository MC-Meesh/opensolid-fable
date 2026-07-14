// Scene tree: the shared model of a script's shape construction graph.
//
// This module is the single source of truth that both the script and the GUI
// read/write:
//   script -> model : runTracedScript() executes the script with a tracing
//                     Shape wrapper that records every operation as a node
//                     while delegating to the real Shape class.
//   model -> script : serializeTree() emits a canonical script for any tree,
//                     one readable statement per feature (shared subtrees and
//                     boolean steps become `const` bindings), optionally
//                     keeping the script's leading comment header.
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
  // The terminating half-space of an "up to face" extrude (Shape.halfSpace).
  'halfSpace',
];

// Instance methods taking only numeric args. One child: the receiver.
export const UNARY_OPS = ['translate', 'rotate', 'scale', 'uniformScale'];

// Instance methods taking another shape (plus optional numeric args).
// Two children: the receiver, then the other shape.
export const BINARY_OPS = ['union', 'intersect', 'subtract', 'smoothUnion'];

// Static constructors sweeping a 2D profile (a `Profile` instance, then
// numeric args). No children; the node carries a `profile` snapshot:
// `{ start: [x, y], segs: [{ x, y, bulge }] }`.
export const SWEEP_OPS = ['extrude', 'revolve'];

const OP_LABELS = {
  sphere: 'Sphere',
  box3: 'Box3',
  roundedBox: 'Rounded Box',
  cylinder: 'Cylinder',
  torus: 'Torus',
  capsule: 'Capsule',
  halfSpace: 'Half Space',
  translate: 'Translate',
  rotate: 'Rotate',
  scale: 'Scale',
  uniformScale: 'Uniform Scale',
  union: 'Union',
  intersect: 'Intersect',
  subtract: 'Subtract',
  smoothUnion: 'Smooth Union',
  extrude: 'Extrude',
  revolve: 'Revolve',
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
 * Create Shape/Profile-compatible tracing classes that record a construction
 * node for every operation while delegating to `ShapeClass` / `ProfileClass`.
 *
 * Returns `{ TracingShape, TracingProfile, nodes, profiles }`: `nodes`
 * accumulates every node created (including ones a script builds but never
 * uses) so callers can free the underlying shapes; `profiles` accumulates
 * every profile so its real instance can be freed after the script runs.
 *
 * Node shape: `{ id, op, args, children, shape }` — `args` are the numeric
 * arguments, `children` reference other nodes, and `shape` is the retained
 * `ShapeClass` instance for that intermediate result. Sweep nodes
 * additionally carry `profile`, a plain-data snapshot of the profile at the
 * moment of the sweep call.
 */
export function createTracer(ShapeClass, ProfileClass) {
  let nextId = 1;
  const nodes = [];
  const profiles = [];

  class TracingShape {
    constructor(node) {
      this.node = node;
    }
    get shape() {
      return this.node.shape;
    }
  }

  class TracingProfile {
    constructor(x, y) {
      this.real = new ProfileClass(x, y);
      this.start = [x, y];
      this.segs = [];
      this.closed = false;
      profiles.push(this);
    }
    lineTo(x, y) {
      this.arcTo(x, y, 0);
    }
    arcTo(x, y, bulge) {
      if (!this.closed) this.segs.push({ x, y, bulge });
      this.real.arcTo(x, y, bulge);
    }
    close() {
      this.closed = true;
      this.real.close();
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

  for (const op of SWEEP_OPS) {
    TracingShape[op] = (profile, ...args) => {
      if (!(profile instanceof TracingProfile)) {
        throw new Error(
          `Shape.${op}(...) expects a Profile as its first argument`
        );
      }
      const traced = record(
        op,
        args,
        [],
        ShapeClass[op](profile.real, ...args)
      );
      // Snapshot so later mutation of the profile can't change this node.
      traced.node.profile = {
        start: [...profile.start],
        segs: profile.segs.map((s) => ({ ...s })),
      };
      return traced;
    };
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

  return { TracingShape, TracingProfile, nodes, profiles };
}

/**
 * Evaluate a script with construction tracing.
 *
 * Returns `{ root, nodes }`: `root` is the node of the returned shape (its
 * `.shape` is the real `ShapeClass` instance), `nodes` is every node created.
 * Real profile instances are freed once the script finishes (sweep nodes keep
 * only the plain-data snapshot). On error, any shapes created before the
 * failure are freed, then the error is rethrown.
 */
export function runTracedScript(source, ShapeClass, ProfileClass) {
  const { TracingShape, TracingProfile, nodes, profiles } = createTracer(
    ShapeClass,
    ProfileClass
  );
  let result;
  try {
    result = runScript(source, TracingShape, TracingProfile);
  } catch (err) {
    freeNodes(nodes);
    throw err;
  } finally {
    for (const profile of profiles) {
      if (typeof profile.real?.free === 'function') profile.real.free();
      profile.real = null;
    }
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
 * The leading comment block of a script — the run of `//` lines, `/* ... *​/`
 * blocks, and blank lines at the top, up to the first line of code — with
 * trailing blank lines trimmed, ending in a single newline. Returns '' when
 * the script does not start with a comment. Lets canonical re-serialization
 * carry the API-reference header (and any user preamble) across GUI edits.
 */
export function scriptHeader(source) {
  const lines = source.split('\n');
  let end = 0;
  let inBlock = false;
  for (const line of lines) {
    const text = line.trim();
    if (inBlock) {
      end += 1;
      if (text.includes('*/')) inBlock = false;
      continue;
    }
    if (text === '' || text.startsWith('//')) {
      end += 1;
    } else if (text.startsWith('/*')) {
      end += 1;
      if (!text.includes('*/')) inBlock = true;
    } else {
      break;
    }
  }
  while (end > 0 && lines[end - 1].trim() === '') end -= 1;
  if (end === 0) return '';
  return `${lines.slice(0, end).join('\n')}\n`;
}

/**
 * Emit a canonical script for the tree rooted at `root`.
 *
 * Every boolean (two-child) operation and every node referenced more than
 * once is hoisted into a `const s<N> = ...;` binding, so the emitted script
 * is a readable statement-per-feature program (and the DAG structure
 * survives a round trip); single-use primitives and transform chains stay
 * inline. Sweep nodes emit their profile as `const p<N> = new Profile(...)`
 * builder statements first. `header` (see `scriptHeader`) is prepended with
 * a blank separator line when given.
 */
export function serializeTree(root, { header = '' } = {}) {
  const refs = new Map();
  const countRefs = (node) => {
    const seen = (refs.get(node.id) ?? 0) + 1;
    refs.set(node.id, seen);
    if (seen === 1) node.children.forEach(countRefs);
  };
  countRefs(root);

  const names = new Map();
  const lines = [];
  let profileCount = 0;

  const emitProfile = (profile) => {
    const name = `p${++profileCount}`;
    lines.push(`const ${name} = new Profile(${profile.start.map(String).join(', ')});`);
    for (const seg of profile.segs) {
      lines.push(
        seg.bulge === 0
          ? `${name}.lineTo(${seg.x}, ${seg.y});`
          : `${name}.arcTo(${seg.x}, ${seg.y}, ${seg.bulge});`
      );
    }
    lines.push(`${name}.close();`);
    return name;
  };

  const exprOf = (node) => {
    if (names.has(node.id)) return names.get(node.id);
    const args = node.args.map(String);
    let text;
    if (node.profile) {
      const pname = emitProfile(node.profile);
      const rest = args.length > 0 ? `, ${args.join(', ')}` : '';
      text = `Shape.${node.op}(${pname}${rest})`;
    } else if (node.children.length === 0) {
      text = `Shape.${node.op}(${args.join(', ')})`;
    } else if (node.children.length === 1) {
      text = `${exprOf(node.children[0])}.${node.op}(${args.join(', ')})`;
    } else {
      const [receiver, other] = node.children;
      const rest = args.length > 0 ? `, ${args.join(', ')}` : '';
      text = `${exprOf(receiver)}.${node.op}(${exprOf(other)}${rest})`;
    }
    if (node !== root && (refs.get(node.id) > 1 || node.children.length === 2)) {
      const name = `s${names.size + 1}`;
      names.set(node.id, name);
      lines.push(`const ${name} = ${text};`);
      return name;
    }
    return text;
  };

  lines.push(`return ${exprOf(root)};`);
  const body = `${lines.join('\n')}\n`;
  return header ? `${header}\n${body}` : body;
}
