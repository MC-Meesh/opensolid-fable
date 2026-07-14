// Kernel bridge: loads the Node build of opensolid-wasm and runs playground
// scripts against it, so an agent's script produces the identical shape it
// would in the browser playground. Also owns the in-memory model store keyed
// by the ids handed back to callers.

import { createRequire } from 'node:module';
import { fileURLToPath } from 'node:url';
import { dirname, resolve } from 'node:path';

const require = createRequire(import.meta.url);
const here = dirname(fileURLToPath(import.meta.url));

// The wasm-pack `nodejs` target is CommonJS and initializes synchronously on
// require (it reads and instantiates the .wasm alongside it).
const wasm = require(resolve(here, '..', 'pkg', 'opensolid_wasm.js'));

/** The Shape class scripts build against (bound as `Shape`). */
export const Shape = wasm.WasmShape;
/** The 2D profile builder (bound as `Profile`) for extrude/revolve. */
export const Profile = wasm.WasmProfile2D;

/**
 * Evaluate a playground script that must `return` a Shape. Runs in strict
 * mode with `Shape` and `Profile` in scope — the same contract as the
 * playground's Code tab (see web/playground/src/lib/runScript.js). Throws a
 * message-only Error on syntax errors, runtime errors, or a non-Shape return.
 *
 * @param {string} source script body (a function body, not a module)
 * @returns {object} the returned WasmShape
 */
export function runScript(source) {
  if (typeof source !== 'string' || source.trim() === '') {
    throw new Error('script must be a non-empty string');
  }
  let build;
  try {
    build = new Function('Shape', 'Profile', `"use strict";\n${source}`);
  } catch (err) {
    throw new Error(`script has a syntax error: ${err.message}`);
  }
  const shape = build(Shape, Profile);
  if (!(shape instanceof Shape)) {
    throw new Error('script must return a Shape, e.g. end with:\n  return solid;');
  }
  return shape;
}

let counter = 0;

/** A short, human-readable, collision-resistant model id, e.g. `model-1-8f3a`. */
function newId() {
  counter += 1;
  const suffix = Math.random().toString(16).slice(2, 6);
  return `model-${counter}-${suffix}`;
}

/**
 * In-memory registry of built models. Each entry keeps the shape, the source
 * script (so exports/screenshots are reproducible), the requested name, the
 * exact-booleans flag, and the creation time.
 */
export class ModelStore {
  constructor() {
    /** @type {Map<string, {id:string, name:string, script:string, shape:object, exact:boolean, createdAt:string}>} */
    this._models = new Map();
  }

  /**
   * Compile a script and register the resulting model.
   * @param {{script:string, name?:string, exact?:boolean}} req
   */
  create({ script, name, exact = false }) {
    // The exact-booleans path is a process-global toggle in the kernel; set
    // it before building so booleans route correctly for this model.
    Shape.setExactBooleans(Boolean(exact));
    const shape = runScript(script);
    const id = newId();
    const entry = {
      id,
      name: name || id,
      script,
      shape,
      exact: Boolean(exact),
      createdAt: new Date().toISOString(),
    };
    this._models.set(id, entry);
    return entry;
  }

  /**
   * Look up a model, applying its exact-booleans flag to the global toggle
   * first so any subsequent mesh/measure/export reads the right pipeline.
   * @param {string} id
   */
  get(id) {
    const entry = this._models.get(id);
    if (!entry) {
      throw new Error(`unknown model_id: ${id}`);
    }
    Shape.setExactBooleans(entry.exact);
    return entry;
  }

  list() {
    return [...this._models.values()].map((m) => ({
      model_id: m.id,
      name: m.name,
      exact: m.exact,
      createdAt: m.createdAt,
    }));
  }

  has(id) {
    return this._models.has(id);
  }
}
