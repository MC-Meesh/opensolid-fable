// Shape-script evaluation, kept free of React and WASM imports so it can be
// unit-tested with a stand-in Shape class.

/**
 * Evaluate a user script that must `return` an instance of `ShapeClass`.
 *
 * The script body runs in strict mode with a single binding, `Shape`. Throws
 * on syntax errors, runtime errors, or a non-Shape return value; the thrown
 * error's message is suitable for direct display.
 *
 * @param {string} source script body (function body, not a full module)
 * @param {Function} ShapeClass constructor the script must return an instance of
 * @returns {object} the shape the script returned
 */
export function runScript(source, ShapeClass) {
  const build = new Function('Shape', `"use strict";\n${source}`);
  const shape = build(ShapeClass);
  if (!(shape instanceof ShapeClass)) {
    throw new Error('Script must return a Shape, e.g. end with:\n  return solid;');
  }
  return shape;
}
