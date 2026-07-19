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
 * Build the `param()` helper a script uses to declare a design variable, plus
 * the array it records declarations into.
 *
 * `param(name, default, { min, max })` returns the value to use for this
 * evaluation — the `default`, or an `overrides[name]` when one is supplied —
 * and records `{ name, default, value, min, max }`. This is how a script both
 * *runs* (create_model: every param takes its default) and becomes
 * *optimizable* (the `optimize` tool re-runs it with overrides). Declaring the
 * bounds at the call site is deliberate: per DIFFERENTIABLE.md §7, guessing
 * which numbers are design variables optimizes the wrong thing, and a bound is
 * not optional in CAD — a wall thickness of −3 mm is not a design.
 *
 * @param {Record<string, number>} overrides parameter values to substitute
 * @returns {{ param: Function, declared: Array<object> }}
 */
function makeParam(overrides = {}) {
  const declared = [];
  const seen = new Set();
  const param = (name, defaultValue, opts = {}) => {
    if (typeof name !== 'string' || name === '') {
      throw new Error('param(name, default, {min, max}): name must be a non-empty string');
    }
    if (seen.has(name)) {
      throw new Error(`param '${name}' is declared more than once`);
    }
    if (!Number.isFinite(defaultValue)) {
      throw new Error(`param '${name}': default must be a finite number`);
    }
    const { min, max } = opts || {};
    const bounded = min !== undefined || max !== undefined;
    if (bounded && !(Number.isFinite(min) && Number.isFinite(max))) {
      throw new Error(`param '${name}': min and max must both be finite numbers, or both omitted`);
    }
    if (bounded && min > max) {
      throw new Error(`param '${name}': min ${min} exceeds max ${max}`);
    }
    const override = Object.prototype.hasOwnProperty.call(overrides, name)
      ? overrides[name]
      : undefined;
    if (override !== undefined && !Number.isFinite(override)) {
      throw new Error(`param '${name}': override value must be a finite number`);
    }
    seen.add(name);
    declared.push({
      name,
      default: defaultValue,
      value: override !== undefined ? override : defaultValue,
      min: bounded ? min : undefined,
      max: bounded ? max : undefined,
    });
    return override !== undefined ? override : defaultValue;
  };
  return { param, declared };
}

/**
 * Evaluate a playground script that must `return` a Shape. Runs in strict
 * mode with `Shape`, `Profile`, and `param` in scope — the same contract as
 * the playground's Code tab (see web/playground/src/lib/runScript.js), plus
 * `param()` for declaring design variables. Throws a message-only Error on
 * syntax errors, runtime errors, or a non-Shape return.
 *
 * @param {string} source script body (a function body, not a module)
 * @param {Record<string, number>} [overrides] param values to substitute
 * @returns {{ shape: object, params: Array<object> }} the Shape and the
 *   design parameters the script declared (in declaration order)
 */
export function runScript(source, overrides = {}) {
  if (typeof source !== 'string' || source.trim() === '') {
    throw new Error('script must be a non-empty string');
  }
  let build;
  try {
    build = new Function('Shape', 'Profile', 'param', `"use strict";\n${source}`);
  } catch (err) {
    throw new Error(`script has a syntax error: ${err.message}`);
  }
  const { param, declared } = makeParam(overrides);
  const shape = build(Shape, Profile, param);
  if (!(shape instanceof Shape)) {
    throw new Error('script must return a Shape, e.g. end with:\n  return solid;');
  }
  return { shape, params: declared };
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
    const { shape, params } = runScript(script);
    const id = newId();
    const entry = {
      id,
      name: name || id,
      script,
      shape,
      // The design variables the script declared via `param()`, kept so the
      // `optimize` tool knows what may move and within what bounds, and so
      // `create_model` can surface them to the agent. `values` tracks the
      // params currently baked into `shape` — the declared defaults until
      // `applyOptimized` writes an optimised point back.
      params,
      exact: Boolean(exact),
      createdAt: new Date().toISOString(),
    };
    this._models.set(id, entry);
    return entry;
  }

  /**
   * Write an optimised parameter point back into a model, so the next
   * `measure`/`export`/`get_screenshot` reflects the optimised part rather
   * than the part as first authored (DIFFERENTIABLE.md §10 — `optimize`
   * *moves* the model, it does not just report on it).
   *
   * @param {string} id
   * @param {object} shape the shape rebuilt at the optimised overrides
   * @param {Record<string, number>} overrides the winning param values
   */
  applyOptimized(id, shape, overrides) {
    const entry = this._models.get(id);
    if (!entry) {
      throw new Error(`unknown model_id: ${id}`);
    }
    entry.shape = shape;
    entry.params = entry.params.map((p) =>
      Object.prototype.hasOwnProperty.call(overrides, p.name)
        ? { ...p, value: overrides[p.name] }
        : p,
    );
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
