// Shape-script evaluation, kept free of React and WASM imports so it can be
// unit-tested with a stand-in Shape class.

/**
 * Evaluate a user script that must `return` an instance of `ShapeClass`.
 *
 * The script body runs in strict mode with four bindings: `Shape`,
 * `Profile` (the 2D profile builder consumed by `Shape.extrude` /
 * `Shape.revolve` / `Shape.loft`), `Path` (the 3D polyline builder
 * consumed by `Shape.sweep`), and `OpenPath` (the open 2D polyline builder
 * consumed by `Shape.rib`). Throws on syntax errors, runtime errors, or a
 * non-Shape return value; the thrown error's message is suitable for direct
 * display.
 *
 * @param {string} source script body (function body, not a full module)
 * @param {Function} ShapeClass constructor the script must return an instance of
 * @param {Function} [ProfileClass] constructor bound as `Profile` in the script
 * @param {Function} [PathClass] constructor bound as `Path` in the script
 * @param {Function} [OpenPathClass] constructor bound as `OpenPath` in the script
 * @returns {object} the shape the script returned
 */
export function runScript(source, ShapeClass, ProfileClass, PathClass, OpenPathClass) {
  const build = new Function(
    'Shape',
    'Profile',
    'Path',
    'OpenPath',
    `"use strict";\n${source}`
  );
  const shape = build(ShapeClass, ProfileClass, PathClass, OpenPathClass);
  if (!(shape instanceof ShapeClass)) {
    throw new Error('Script must return a Shape, e.g. end with:\n  return solid;');
  }
  return shape;
}
