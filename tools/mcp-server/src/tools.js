// MCP tool definitions and handlers for the OpenSolid kernel. Transport-free
// so the tools can be unit-tested directly. Each handler returns an MCP
// content result: `{ content: [...], isError? }`.

import { writeFileSync, mkdirSync, statSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { resolve, isAbsolute, join } from 'node:path';
import { ModelStore } from './kernel.js';
import { getMesh, buildBinaryStl, buildObj } from './mesh.js';
import { renderPng, VIEW_NAMES } from './render.js';

const EXPORT_FORMATS = ['step', 'stl', 'obj'];
const MEASURE_QUERIES = ['all', 'volume', 'surface_area', 'bbox', 'centroid', 'mass'];

function text(obj) {
  const body = typeof obj === 'string' ? obj : JSON.stringify(obj, null, 2);
  return { content: [{ type: 'text', text: body }] };
}

function fail(message) {
  return { content: [{ type: 'text', text: `Error: ${message}` }], isError: true };
}

// Extract a human-readable message from a thrown value. wasm-bindgen rejects a
// Rust `Result::Err(String)` by throwing the *raw string* (not an Error), so
// `err.message` is `undefined` for kernel-side failures — the useful text lives
// in the value itself. Read `.message` when present, otherwise stringify.
function errMessage(err) {
  if (err && typeof err.message === 'string') return err.message;
  return String(err);
}

// A null volume is never self-explanatory: the kernel says why in `massError`,
// and on the SDF path the cause is often just a mesh too coarse to close, which
// a finer `accuracy` fixes. Carry both onto any payload that reports volume, so
// a null always arrives with its reason rather than looking like a broken model.
function withMassError(view, full) {
  if (!full.massError) return view;
  const annotated = { ...view, massError: full.massError };
  if (!full.exact) {
    annotated.hint =
      'Mass properties are integrated over the measured mesh; at this accuracy the mesh ' +
      'does not close. Retry with a smaller `accuracy` (e.g. half the current value) ' +
      'before concluding the model itself is bad.';
  }
  return annotated;
}

/** Resolve where an export should be written. */
function exportPath(requested, outputDir, model, format) {
  if (requested) {
    return isAbsolute(requested) ? requested : resolve(outputDir, requested);
  }
  mkdirSync(outputDir, { recursive: true });
  return join(outputDir, `${model.name}.${format}`);
}

/**
 * Build the tool registry bound to a fresh model store.
 * @param {{outputDir?:string}} [config]
 */
export function createTools(config = {}) {
  const store = new ModelStore();
  const outputDir = config.outputDir || join(tmpdir(), 'opensolid-mcp');

  /** @type {Record<string, {definition:object, handler:(args:object)=>object}>} */
  const tools = {
    create_model: {
      definition: {
        name: 'create_model',
        description:
          'Build a CAD model from a playground JS script and register it under a ' +
          'model_id. The script has `Shape` and `Profile` in scope and must `return` ' +
          'a Shape (identical semantics to the browser playground). Returns the ' +
          'model_id plus mesh statistics and a validation summary.',
        inputSchema: {
          type: 'object',
          properties: {
            script: {
              type: 'string',
              description:
                'JS body that returns a Shape, e.g. `return Shape.sphere(1).subtract(Shape.box3(1,1,1));`',
            },
            name: { type: 'string', description: 'Optional friendly name for the model.' },
            exact: {
              type: 'boolean',
              description:
                'Route sharp booleans through the exact B-Rep pipeline (crisp edges, ' +
                'analytic STEP). Default false (SDF path).',
            },
          },
          required: ['script'],
        },
      },
      handler(args) {
        let model;
        try {
          model = store.create({
            script: args.script,
            name: args.name,
            exact: args.exact,
          });
        } catch (err) {
          return fail(`script failed: ${errMessage(err)}`);
        }
        const measure = JSON.parse(model.shape.measure(undefined));
        const validation = JSON.parse(model.shape.validate(undefined));
        return text(
          withMassError(
            {
              model_id: model.id,
              name: model.name,
              exact: model.exact,
              mesh: { triangles: measure.triangles, vertices: measure.vertices },
              boundingBox: measure.boundingBox,
              volume: measure.volume,
              valid: validation.valid,
              issues: validation.issues,
            },
            measure,
          ),
        );
      },
    },

    get_screenshot: {
      definition: {
        name: 'get_screenshot',
        description:
          'Render a model to a PNG image from a named view. Returns the image inline. ' +
          `Views: ${VIEW_NAMES.join(', ')} (default iso).`,
        inputSchema: {
          type: 'object',
          properties: {
            model_id: { type: 'string' },
            view: { type: 'string', enum: VIEW_NAMES, description: 'Camera view (default iso).' },
            width: { type: 'number', description: 'Image width in px (default 800).' },
            height: { type: 'number', description: 'Image height in px (default 600).' },
          },
          required: ['model_id'],
        },
      },
      handler(args) {
        let model;
        try {
          model = store.get(args.model_id);
        } catch (err) {
          return fail(err.message);
        }
        const mesh = getMesh(model.shape);
        if (mesh.triangles === 0) {
          return fail('model produced an empty mesh; nothing to render');
        }
        const png = renderPng(mesh, model.shape.bounds(), {
          view: args.view,
          width: args.width,
          height: args.height,
        });
        return {
          content: [
            {
              type: 'image',
              data: png.toString('base64'),
              mimeType: 'image/png',
            },
          ],
        };
      },
    },

    export: {
      definition: {
        name: 'export',
        description:
          'Export a model to a file. STEP serializes analytic surfaces (exact chains) ' +
          'or a faceted B-Rep; STL and OBJ write the current mesh. Returns the file path ' +
          'and byte size.',
        inputSchema: {
          type: 'object',
          properties: {
            model_id: { type: 'string' },
            format: { type: 'string', enum: EXPORT_FORMATS, description: 'step | stl | obj.' },
            path: {
              type: 'string',
              description:
                'Optional output path (absolute, or relative to the server output dir). ' +
                'Defaults to <name>.<format> in the output dir.',
            },
          },
          required: ['model_id', 'format'],
        },
      },
      handler(args) {
        const format = String(args.format || '').toLowerCase();
        if (!EXPORT_FORMATS.includes(format)) {
          return fail(`unsupported format '${args.format}'; use one of ${EXPORT_FORMATS.join(', ')}`);
        }
        let model;
        try {
          model = store.get(args.model_id);
        } catch (err) {
          return fail(err.message);
        }
        const dest = exportPath(args.path, outputDir, model, format);
        try {
          mkdirSync(resolve(dest, '..'), { recursive: true });
          if (format === 'step') {
            writeFileSync(dest, model.shape.exportStep(undefined), 'utf8');
          } else if (format === 'stl') {
            const mesh = getMesh(model.shape);
            writeFileSync(dest, buildBinaryStl(mesh.positions, mesh.indices));
          } else {
            const mesh = getMesh(model.shape);
            writeFileSync(dest, buildObj(mesh.positions, mesh.normals, mesh.indices), 'utf8');
          }
        } catch (err) {
          return fail(`export failed: ${errMessage(err)}`);
        }
        return text({ model_id: model.id, format, path: dest, bytes: statSync(dest).size });
      },
    },

    measure: {
      definition: {
        name: 'measure',
        description:
          'Compute mass properties of a model: volume, surface area, centroid, inertia, ' +
          'and bounding box (exact polyhedral integrals over the mesh). `query` narrows ' +
          `the result. Queries: ${MEASURE_QUERIES.join(', ')} (default all). ` +
          'When the mesh does not bound a finite non-zero volume the mass fields are null ' +
          'and `massError` says why; the bounding box is still returned.',
        inputSchema: {
          type: 'object',
          properties: {
            model_id: { type: 'string' },
            query: { type: 'string', enum: MEASURE_QUERIES, description: 'Which properties (default all).' },
            accuracy: {
              type: 'number',
              description: 'Target chordal deviation for the measured mesh (model units).',
            },
          },
          required: ['model_id'],
        },
      },
      handler(args) {
        let model;
        try {
          model = store.get(args.model_id);
        } catch (err) {
          return fail(err.message);
        }
        const full = JSON.parse(model.shape.measure(accuracyArg(args.accuracy)));
        const query = args.query || 'all';
        const view = {
          all: full,
          volume: { volume: full.volume, exact: full.exact },
          surface_area: { surfaceArea: full.surfaceArea, exact: full.exact },
          bbox: { boundingBox: full.boundingBox },
          centroid: { centroid: full.centroid, exact: full.exact },
          mass: {
            volume: full.volume,
            surfaceArea: full.surfaceArea,
            centroid: full.centroid,
            inertia: full.inertia,
            exact: full.exact,
          },
        }[query];
        // `bbox` is the one view that never reports a mass property — it is
        // always present and correct, so a mass failure is not its business.
        if (query === 'bbox') return text(view);
        return text(withMassError(view ?? full, full));
      },
    },

    validate: {
      definition: {
        name: 'validate',
        description:
          'Check a model: whether its mesh is a closed, consistently oriented manifold ' +
          'enclosing a finite non-zero volume. Returns a report with any issues found.',
        inputSchema: {
          type: 'object',
          properties: {
            model_id: { type: 'string' },
            accuracy: { type: 'number', description: 'Target chordal deviation for the checked mesh.' },
          },
          required: ['model_id'],
        },
      },
      handler(args) {
        let model;
        try {
          model = store.get(args.model_id);
        } catch (err) {
          return fail(err.message);
        }
        return text(JSON.parse(model.shape.validate(accuracyArg(args.accuracy))));
      },
    },

    list_models: {
      definition: {
        name: 'list_models',
        description: 'List the models registered this session (id, name, exact flag, creation time).',
        inputSchema: { type: 'object', properties: {} },
      },
      handler() {
        return text({ models: store.list() });
      },
    },
  };

  return {
    store,
    outputDir,
    definitions: Object.values(tools).map((t) => t.definition),
    call(name, args) {
      const tool = tools[name];
      if (!tool) {
        return fail(`unknown tool: ${name}`);
      }
      try {
        return tool.handler(args || {});
      } catch (err) {
        return fail(errMessage(err));
      }
    },
  };
}

function accuracyArg(value) {
  return Number.isFinite(value) && value > 0 ? value : undefined;
}
