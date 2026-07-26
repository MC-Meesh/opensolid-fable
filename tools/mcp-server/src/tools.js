// MCP tool definitions and handlers for the OpenSolid kernel. Transport-free
// so the tools can be unit-tested directly. Each handler returns an MCP
// content result: `{ content: [...], isError? }`.

import { writeFileSync, readFileSync, mkdirSync, statSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { resolve, isAbsolute, join, basename } from 'node:path';
import { ModelStore, importStep } from './kernel.js';
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

// How many per-entity diagnostics an import reports inline. A NIST test part
// can produce hundreds of `Info` lines about trimming decisions; the counts
// are always complete, and the items are the sample an agent reads. Errors and
// warnings are kept ahead of info so a truncation can never hide the finding
// that mattered.
const MAX_DIAGNOSTIC_ITEMS = 40;
const SEVERITY_RANK = { error: 0, warning: 1, info: 2 };

/** The diagnostics view: complete counts, a bounded, severity-first sample. */
function diagnosticsView(report) {
  const items = [...report.diagnostics]
    .map((d, i) => ({ d, i }))
    .sort((a, b) => SEVERITY_RANK[a.d.severity] - SEVERITY_RANK[b.d.severity] || a.i - b.i)
    .slice(0, MAX_DIAGNOSTIC_ITEMS)
    .map(({ d }) => d);
  return {
    ...report.diagnosticCounts,
    items,
    ...(items.length < report.diagnostics.length
      ? {
          truncated: {
            shown: items.length,
            total: report.diagnostics.length,
            note: 'Highest severity first; the counts above are complete.',
          },
        }
      : {}),
  };
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

    import_step: {
      definition: {
        name: 'import_step',
        description:
          'Read an existing STEP (Part 21) file and register it as a model, so an ' +
          'agent can start from a part it was given instead of only from a script it ' +
          'wrote. Pass `path` (a file) or `text` (the file contents). Every ' +
          'MANIFOLD_SOLID_BREP comes back on one of three outcomes: `brep` (exact ' +
          'B-Rep — analytic surfaces, re-exports as analytic STEP), `mesh` (valid ' +
          'STEP the kernel cannot represent exactly, imported as a closed ' +
          'tessellation wrapped as an SDF), or `failed`. Returns the whole file as ' +
          'one `model_id`, a per-solid `model_id` for each solid, the reader\'s ' +
          'per-entity diagnostics, how many repairs the healer applied, the file\'s ' +
          'declared units, and an immediate measure/validate summary of the imported ' +
          'part — the same oracle `create_model` returns, because an import is where ' +
          'the trust loop starts. Imported models work with every other tool ' +
          '(measure, validate, get_screenshot, export); they carry no `param()`s, so ' +
          '`optimize` has nothing to move.',
        inputSchema: {
          type: 'object',
          properties: {
            path: {
              type: 'string',
              description:
                'STEP file to read (absolute, or relative to the server output dir — ' +
                'so a file just written by `export` can be read back by bare name). ' +
                'Mutually exclusive with `text`.',
            },
            text: {
              type: 'string',
              description:
                'The STEP file contents inline, for a file the server cannot reach on ' +
                'disk. Mutually exclusive with `path`.',
            },
            name: { type: 'string', description: 'Optional friendly name for the model.' },
            circle_segments: {
              type: 'number',
              description:
                'Tessellation fidelity of imported bodies, as segments around a full ' +
                'circle (default 32, clamped to 3..512). This is the mesh every ' +
                'measurement and screenshot of the import reads, so raise it when a ' +
                'curved part measures short. It does not affect the analytic surfaces ' +
                'themselves — an `export` of a `brep` solid is unaffected.',
            },
          },
        },
      },
      handler(args) {
        const hasPath = typeof args.path === 'string' && args.path !== '';
        const hasText = typeof args.text === 'string' && args.text !== '';
        if (hasPath === hasText) {
          return fail(
            hasPath
              ? "pass either 'path' or 'text', not both"
              : "nothing to import: pass 'path' (a STEP file) or 'text' (its contents)",
          );
        }

        let bytes;
        let origin;
        if (hasPath) {
          const src = isAbsolute(args.path) ? args.path : resolve(outputDir, args.path);
          try {
            bytes = readFileSync(src);
          } catch (err) {
            return fail(`cannot read STEP file: ${errMessage(err)}`);
          }
          origin = { kind: 'step', path: src, bytes: bytes.length };
        } else {
          // STEP is a Latin-1 format, so encode the string as Latin-1 rather
          // than UTF-8: a degree sign in a product name is one byte in a
          // STEP file, and encoding it as two would corrupt the name the
          // reader hands back.
          bytes = Buffer.from(args.text, 'latin1');
          origin = { kind: 'step', path: null, bytes: bytes.length };
        }

        let imported;
        try {
          imported = importStep(bytes, positiveArg(args.circle_segments));
        } catch (err) {
          return fail(`import failed: ${errMessage(err)}`);
        }

        try {
          const report = JSON.parse(imported.report);
          const baseName =
            args.name || (origin.path ? basename(origin.path).replace(/\.(stp|step)$/i, '') : 'imported');

          // One model per solid, so an agent can address a single part of a
          // multi-solid file, and one for the file as a whole (placed by its
          // assembly occurrences — see `assembled`).
          const solids = report.solids.map((solid, index) => {
            const shape = imported.solid(index);
            const registered = shape
              ? store.registerImported({
                  shape,
                  name: solid.name || `${baseName}-solid-${index}`,
                  origin: { ...origin, solidIndex: index, stepId: solid.stepId, outcome: solid.outcome },
                  exact: solid.exact,
                })
              : null;
            return {
              index,
              step_id: solid.stepId,
              name: solid.name,
              outcome: solid.outcome,
              exact: solid.exact,
              triangles: solid.triangles,
              ...(registered ? { model_id: registered.id } : {}),
              ...(solid.shapeError ? { shapeError: solid.shapeError } : {}),
            };
          });

          const whole = imported.assembled();
          if (!whole) {
            // Valid Part 21, nothing usable in it. That is a real answer, not
            // a crash — but it is an error for the caller, who asked for a
            // part. Hand back the diagnostics that explain it.
            return {
              content: [
                {
                  type: 'text',
                  text: JSON.stringify(
                    {
                      error:
                        report.solids.length === 0
                          ? 'the file parsed but declares no solids (no MANIFOLD_SOLID_BREP)'
                          : report.counts.failed === report.solids.length
                            ? 'the file parsed but no solid could be imported; see diagnostics'
                            : 'the file\'s solids imported but none could be turned into a usable ' +
                              'shape; see shapeError on each solid',
                      solids,
                      counts: report.counts,
                      diagnostics: diagnosticsView(report),
                    },
                    null,
                    2,
                  ),
                },
              ],
              isError: true,
            };
          }

          const model = store.registerImported({
            shape: whole,
            name: baseName,
            origin,
            exact: report.assembledExact,
          });
          const measure = JSON.parse(model.shape.measure(undefined));
          const validation = JSON.parse(model.shape.validate(undefined));
          return text(
            withMassError(
              {
                model_id: model.id,
                name: model.name,
                exact: model.exact,
                source: origin.path
                  ? { path: origin.path, bytes: origin.bytes }
                  : { text: true, bytes: origin.bytes },
                solids,
                counts: report.counts,
                healing: {
                  operations: report.healOperations,
                  ...(report.healOperations
                    ? {
                        note:
                          'Repairs the importer applied to make bodies valid; each one is ' +
                          'also an info diagnostic naming the entity it touched.',
                      }
                    : {}),
                },
                units: {
                  lengthScale: report.lengthScale,
                  angleScale: report.angleScale,
                  note:
                    'Scale factors resolved from the file, already applied: the imported ' +
                    'geometry is in millimetres (and radians) whatever the file declared.',
                },
                assembly: {
                  isAssembly: report.isAssembly,
                  instances: report.instances.length,
                  ...(report.isAssembly
                    ? {
                        note:
                          'This model is the placed assembly (every occurrence transformed ' +
                          'into root coordinates). The per-solid model_ids above are ' +
                          'part-local, as the file stores them.',
                      }
                    : {}),
                },
                diagnostics: diagnosticsView(report),
                mesh: { triangles: measure.triangles, vertices: measure.vertices },
                boundingBox: measure.boundingBox,
                volume: measure.volume,
                valid: validation.valid,
                issues: validation.issues,
              },
              measure,
            ),
          );
        } finally {
          // The shapes handed out are independent objects; this releases the
          // import's own wasm handle rather than waiting on a finalizer.
          imported.free();
        }
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
        description:
          'List the models registered this session (id, name, exact flag, source kind, ' +
          'creation time). `source` is `script` or `step`; `get_model` returns the ' +
          'source itself.',
        inputSchema: { type: 'object', properties: {} },
      },
      handler() {
        return text({ models: store.list() });
      },
    },

    get_model: {
      definition: {
        name: 'get_model',
        description:
          "Return a model's own source: the script it was built from (verbatim, ready " +
          'to edit and re-submit to `create_model`), or the STEP file it was imported ' +
          'from. Also its params with their current values — after `optimize` those ' +
          'are the converged numbers, so this is how an optimized design is recovered ' +
          'as something reproducible rather than as a `model_id` that dies with the ' +
          'session.',
        inputSchema: {
          type: 'object',
          properties: { model_id: { type: 'string' } },
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
        return text({
          model_id: model.id,
          name: model.name,
          exact: model.exact,
          createdAt: model.createdAt,
          source: model.origin.kind,
          ...(model.origin.kind === 'script'
            ? { script: model.script }
            : {
                imported: {
                  ...(model.origin.path ? { path: model.origin.path } : { text: true }),
                  bytes: model.origin.bytes,
                  ...(model.origin.solidIndex !== undefined
                    ? {
                        solidIndex: model.origin.solidIndex,
                        stepId: model.origin.stepId,
                        outcome: model.origin.outcome,
                      }
                    : { note: 'The whole file: every solid, placed by its assembly occurrences.' }),
                },
              }),
          params: model.params.map((p) => ({
            name: p.name,
            value: p.value,
            default: p.default,
            ...(p.min !== undefined ? { min: p.min } : {}),
            ...(p.max !== undefined ? { max: p.max } : {}),
          })),
        });
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
        // An imported model has no script, so there is nothing to re-run at a
        // new parameter point. Say that, rather than letting the search fail
        // deeper in with a message about a script the model never had.
        if (model.origin.kind !== 'script') {
          return fail(
            `model ${model.id} was imported from a file, not built from a script, so it ` +
              'has no design variables to move. Optimization needs a `create_model` ' +
              "script that declares them with param('name', default, {min, max}); an " +
              'imported part can be measured, validated, rendered and exported, but not ' +
              'rebuilt at a new parameter point.',
          );
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

/** A positive finite number, or `undefined` to mean "use the default". */
function positiveArg(value) {
  return Number.isFinite(value) && value > 0 ? value : undefined;
}

function accuracyArg(value) {
  return positiveArg(value);
}
