// MCP tool definitions and handlers for the OpenSolid kernel. Transport-free
// so the tools can be unit-tested directly. Each handler returns an MCP
// content result: `{ content: [...], isError? }`.

import { writeFileSync, mkdirSync, statSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { resolve, isAbsolute, join } from 'node:path';
import { ModelStore } from './kernel.js';
import { getMesh, buildBinaryStl, buildObj } from './mesh.js';
import { renderPng, VIEW_NAMES } from './render.js';
import { optimize } from './optimize.js';
import { buildManifest, SERVER_INFO, UNITS } from './capabilities.js';

const EXPORT_FORMATS = ['step', 'stl', 'obj'];
const MEASURE_QUERIES = ['all', 'volume', 'surface_area', 'bbox', 'centroid', 'mass'];
// Document units the STEP writer can declare (docs/units.md). The kernel
// defaults an unknown key to millimetres silently; the tool rejects it instead,
// because "I asked for inches and got millimetres" is exactly the interop bug
// the unit declaration exists to prevent.
const UNIT_KEYS = UNITS.map((u) => u.key);
const DEFAULT_UNIT = 'mm';

function text(obj) {
  const body = typeof obj === 'string' ? obj : JSON.stringify(obj, null, 2);
  return { content: [{ type: 'text', text: body }] };
}

// Every other payload is small enough that indentation is free. The capability
// manifest is not: pretty-printing the whole surface costs ~50 KB against ~29 KB
// compact, and its reader is a machine.
function compactText(obj) {
  return { content: [{ type: 'text', text: JSON.stringify(obj) }] };
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

// A null volume is never self-explanatory: the kernel says why in `massError`.
// Carry it onto any payload that reports volume, so a null always arrives with
// its reason rather than looking like a broken model.
//
// The hint deliberately does *not* just say "retry with a finer accuracy". That
// advice was measured against the failure agents actually hit here (of-9l3) and
// it is a dead end: on the gallery hinge leaf, 16x finer accuracy quadrupled the
// triangle count and the mesh still did not close, because the defect is a
// mesher pinch at a near-tangent feature (of-o0o), not coarseness. `massError`
// now names the defect kind, so key the advice off that instead of guessing.
function withMassError(view, full) {
  if (!full.massError) return view;
  const annotated = { ...view, massError: full.massError };
  if (!full.exact) {
    annotated.hint = /pinched edge/.test(full.massError)
      ? 'Mass properties are integrated over the measured mesh, and this mesh is pinched ' +
        'rather than under-resolved: a finer `accuracy` will not reliably close it, and ' +
        'resizing the feature only moves the pinch. Nudging the feature size or the ' +
        'overall proportions is the available workaround; the model itself may be fine.'
      : 'Mass properties are integrated over the measured mesh; at this accuracy the mesh ' +
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
          'model_id. The script has `Shape`, `Profile` (closed 2D profiles), `Path` ' +
          '(3D polyline for `Shape.sweep`), `OpenPath` (open 2D polyline for ' +
          '`Shape.rib`), and `param` in scope, and must `return` a Shape (playground ' +
          'semantics). `get_capabilities` lists every callable op. Declare a ' +
          "design variable with `param(name, default, {min, max})` — e.g. " +
          "`const t = param('thickness', 4, {min: 2, max: 12});` — to make it " +
          'optimizable by the `optimize` tool; the call returns the value to use and ' +
          'the model builds at the default. Returns the model_id, mesh statistics, a ' +
          'validation summary, and any declared params.',
        inputSchema: {
          type: 'object',
          properties: {
            script: {
              type: 'string',
              description:
                'JS body that returns a Shape, e.g. `return Shape.sphere(1).subtract(Shape.box3(1,1,1));`. ' +
                "Wrap tunable dimensions in `param('name', default, {min, max})` to expose them to `optimize`.",
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
              // The design variables the script declared via param(). Present so
              // an agent sees, from the create call alone, exactly what `optimize`
              // may move and within what bounds. Omitted when the script declares none.
              ...(model.params.length
                ? {
                    params: model.params.map((p) => ({
                      name: p.name,
                      value: p.value,
                      ...(p.min !== undefined ? { min: p.min } : {}),
                      ...(p.max !== undefined ? { max: p.max } : {}),
                    })),
                  }
                : {}),
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
          'and byte size. `unit` declares the document unit in the STEP header — the ' +
          'kernel is unitless, so this is what tells the importer whether a coordinate ' +
          'of 60 means 60 mm or 60 in.',
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
            accuracy: {
              type: 'number',
              description:
                'Target chordal deviation of the exported facets (model units); defaults ' +
                'to 0.5% of the extent. Coarser values mean fewer facets and smaller ' +
                'files, saturating once the octree hits its minimum depth (roughly ' +
                'accuracy = extent/16). Ignored for STEP when the model has an exact B-Rep.',
            },
            unit: {
              type: 'string',
              enum: UNIT_KEYS,
              description:
                'Document unit declared in the STEP header (default mm). Metadata only: ' +
                'coordinates are written verbatim and never rescaled, so switching to ' +
                '`in` makes a 60-unit part 60 inches wide, not 2.36. STL and OBJ carry no ' +
                'unit declaration, so it does not apply to them.',
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
        const requestedUnit = args.unit === undefined ? undefined : String(args.unit).toLowerCase();
        if (requestedUnit !== undefined && !UNIT_KEYS.includes(requestedUnit)) {
          return fail(
            `unsupported unit '${args.unit}'; use one of ${UNIT_KEYS.join(', ')} ` +
              '(the unit is a STEP header declaration, not a rescale)',
          );
        }
        let model;
        try {
          model = store.get(args.model_id);
        } catch (err) {
          return fail(err.message);
        }
        const dest = exportPath(args.path, outputDir, model, format);
        const accuracy = accuracyArg(args.accuracy);
        const unit = requestedUnit || DEFAULT_UNIT;
        try {
          mkdirSync(resolve(dest, '..'), { recursive: true });
          if (format === 'step') {
            writeFileSync(dest, model.shape.exportStep(accuracy, unit), 'utf8');
          } else if (format === 'stl') {
            const mesh = getMesh(model.shape, { accuracy });
            writeFileSync(dest, buildBinaryStl(mesh.positions, mesh.indices));
          } else {
            const mesh = getMesh(model.shape, { accuracy });
            writeFileSync(dest, buildObj(mesh.positions, mesh.normals, mesh.indices), 'utf8');
          }
        } catch (err) {
          return fail(`export failed: ${errMessage(err)}`);
        }
        return text({
          model_id: model.id,
          format,
          path: dest,
          bytes: statSync(dest).size,
          // Report the unit only where it means something. Echoing "mm" on an
          // STL would be a claim the file does not make.
          ...(format === 'step' ? { unit } : {}),
          ...(format !== 'step' && requestedUnit
            ? {
                note:
                  `${format.toUpperCase()} carries no unit declaration, so 'unit' was not ` +
                  'applied. Export STEP when the unit has to travel with the geometry.',
              }
            : {}),
        });
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

    get_capabilities: {
      definition: {
        name: 'get_capabilities',
        description:
          'The full machine-readable capability manifest: every tool with its input ' +
          'schema, and every script operation with its signature, argument names, and ' +
          'notes — primitives, sketch features (extrude/revolve/sweep/loft/rib), ' +
          'transforms, booleans, blends (smoothUnion/filletEdge/chamferEdge), shell, ' +
          'patterns, in-script queries (distance/normalAt/bounds), the Profile / Path / ' +
          'OpenPath builders, and `param`. Also the axis and half-extent conventions and ' +
          'the document units `export` accepts. Call this first to learn the whole ' +
          'surface without reading the prose docs. The full manifest is ~29 KB of ' +
          'compact JSON; `section` narrows it to one part.',
        inputSchema: {
          type: 'object',
          properties: {
            section: {
              type: 'string',
              enum: ['all', 'tools', 'script', 'conventions', 'units'],
              description: 'Which part of the manifest to return (default all).',
            },
          },
        },
      },
      handler(args) {
        const manifest = buildManifest(
          SERVER_INFO,
          Object.values(tools).map((t) => t.definition),
        );
        const section = args.section || 'all';
        if (section === 'all') return compactText(manifest);
        const view = {
          tools: { tools: manifest.tools },
          script: { script: manifest.script },
          conventions: { conventions: manifest.conventions },
          units: { units: manifest.units },
        }[section];
        if (!view) {
          return fail(
            `unknown section '${args.section}'; use one of all, tools, script, conventions, units`,
          );
        }
        return compactText(view);
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

    optimize: {
      definition: {
        name: 'optimize',
        description:
          "Drive a model's `param()` design variables onto an objective under keep-out / " +
          'mass / volume constraints, using gradient descent on the smooth F-Rep field ' +
          '(the active counterpart to `measure`: measure reports, optimize *moves*). The ' +
          'named params must have been declared in the model\'s script with ' +
          "`param(name, default, {min, max})`. Writes the converged values back into the " +
          'model, so a subsequent get_screenshot/export/measure shows the optimized part. ' +
          'Returns the converged params, the achieved objective and constraint values ' +
          'measured on the EXACT mesh, whether it converged or hit a bound/iteration/time ' +
          'cap, per-iteration loss history, and warnings (pinned or no-effect params). ' +
          'Topology is yours to choose: optimize only moves numbers — to change structure, ' +
          'edit the script and optimize again. Every op is supported, including rotate.',
        inputSchema: {
          type: 'object',
          properties: {
            model_id: { type: 'string' },
            params: {
              type: 'array',
              description:
                'Which declared params may move, and their bounds. Bounds are required ' +
                '(a wall thickness of −3 mm is not a design); they may be omitted here only ' +
                'if the param() declaration already carries them.',
              items: {
                type: 'object',
                properties: {
                  name: { type: 'string' },
                  min: { type: 'number' },
                  max: { type: 'number' },
                  start: { type: 'number', description: 'Optional starting value (default: the param\'s current value).' },
                },
                required: ['name'],
              },
            },
            objective: {
              type: 'object',
              description:
                'What to minimize toward. target_mass/target_volume drive a scalar to `value`; ' +
                'centroid_at drives the centre of mass to a point. target_mass needs a `density` ' +
                '(mass per model unit³, e.g. 0.0027 g/mm³ for aluminium 6061).',
              properties: {
                type: { type: 'string', enum: ['target_mass', 'target_volume', 'centroid_at'] },
                value: {
                  description: 'Target: a positive number for mass/volume, or [x,y,z] (null to skip an axis) for centroid_at.',
                },
                density: { type: 'number', description: 'Mass per model unit³, required for target_mass.' },
              },
              required: ['type', 'value'],
            },
            constraints: {
              type: 'array',
              description:
                'Optional penalties. clearance: solid stays `min` away from keep-out `probes` ' +
                '(point keep-outs — [[x,y,z],…] or flat [x,y,z,…]). mass/volume: hold the ' +
                'measured quantity within [min,max] (mass needs a density).',
              items: {
                type: 'object',
                properties: {
                  type: { type: 'string', enum: ['clearance', 'mass', 'volume'] },
                  probes: { description: 'Keep-out points for clearance: [[x,y,z],…] or a flat [x,y,z,…] array.' },
                  min: { type: 'number' },
                  max: { type: 'number' },
                  softness: { type: 'number', description: 'Clearance softmin blend (model units, default 0.02).' },
                  density: { type: 'number', description: 'For a mass bound; inherits the objective density if omitted.' },
                },
                required: ['type'],
              },
            },
            options: {
              type: 'object',
              description: 'Guardrails and tuning.',
              properties: {
                max_iters: { type: 'number', description: `Iteration cap (default 60, max ${300}).` },
                time_budget_ms: { type: 'number', description: 'Wall-clock cap in ms (default 30000, max 120000).' },
                resolution: { type: 'number', description: 'Field quadrature samples per axis (default 32, max 64; cost ~res³).' },
                penalty_weight: { type: 'number', description: 'Constraint penalty weight relative to the objective (default 10).' },
              },
            },
          },
          required: ['model_id', 'params', 'objective'],
        },
      },
      handler(args) {
        let model;
        try {
          model = store.get(args.model_id);
        } catch (err) {
          return fail(err.message);
        }
        let result;
        try {
          result = optimize(model, args);
        } catch (err) {
          return fail(`optimize failed: ${errMessage(err)}`);
        }
        // Commit the winning point back into the model so the next
        // measure/export/get_screenshot reflects the optimized part.
        store.applyOptimized(model.id, result.shape, result.overrides);
        return text({ model_id: model.id, ...result.report });
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
