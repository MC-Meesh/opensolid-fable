// MCP tool definitions and handlers for the OpenSolid kernel. Transport-free
// so the tools can be unit-tested directly. Each handler returns an MCP
// content result: `{ content: [...], isError? }`.

import { writeFileSync, readFileSync, mkdirSync, statSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { resolve, isAbsolute, join, basename } from 'node:path';
import { ModelStore, importStep, Assembly } from './kernel.js';
import { getMesh, buildBinaryStl, buildObj } from './mesh.js';
import {
  renderScene,
  viewDirection,
  VIEW_NAMES,
  RENDER_MODES,
  SECTION_AXES,
} from './render.js';
import { optimize } from './optimize.js';
import { buildManifest, SERVER_INFO, UNITS } from './capabilities.js';
import { inspect as inspectTopology, probeAxis } from './topology.js';

const EXPORT_FORMATS = ['step', 'stl', 'obj'];
const MEASURE_QUERIES = ['all', 'volume', 'surface_area', 'bbox', 'centroid', 'mass'];
/** Softmin blend `measure_clearance` uses when the caller names none. */
const DEFAULT_CLEARANCE_SOFTNESS = 0.02;
/**
 * Relative tolerance `assert_model` applies to a continuous quantity when the
 * caller gives none. 1% because these are integrals over a tessellation, not
 * analytic values — a tighter default would fail on correct parts, and a looser
 * one would pass the wrong-axis-hole bug the whole tool exists to catch (that
 * bracket's volume was ~4% light).
 */
const DEFAULT_RELATIVE_TOLERANCE = 0.01;
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

// Extract the raw thrown value as text. wasm-bindgen rejects a Rust
// `Result::Err(String)` by throwing the *raw string* (not an Error), so
// `err.message` is `undefined` for kernel-side failures — the useful text
// lives in the value itself. Read `.message` when present, otherwise
// stringify.
function errRaw(err) {
  if (err && typeof err.message === 'string') return err.message;
  return String(err);
}

// Structured kernel errors (of-2y4.9): every error the wasm boundary throws
// is one JSON object — {code, category, message, hint?} — so an agent can
// branch on `code`/`category` instead of pattern-matching prose. Returns the
// parsed object, or null for errors that are not structured (JS TypeErrors,
// fs errors, pre-upgrade kernels).
function errInfo(err) {
  const raw = errRaw(err);
  if (!raw.startsWith('{"code":"')) return null;
  try {
    const parsed = JSON.parse(raw);
    if (parsed && typeof parsed.code === 'string' && typeof parsed.message === 'string') {
      return parsed;
    }
  } catch {
    // Looked structured but was not valid JSON; treat as prose.
  }
  return null;
}

// Human-readable message from a thrown value, structured or not.
function errMessage(err) {
  const info = errInfo(err);
  return info ? info.message : errRaw(err);
}

// Failure result that keeps the kernel's structure. Structured errors are
// re-serialized as JSON (with `prefix` folded into the message) so the agent
// receives code/category/hint verbatim; unstructured errors fall back to the
// plain-text form.
function failErr(err, prefix) {
  const info = errInfo(err);
  if (!info) {
    const raw = errRaw(err);
    return fail(prefix ? `${prefix}: ${raw}` : raw);
  }
  const payload = {
    code: info.code,
    category: info.category,
    message: prefix ? `${prefix}: ${info.message}` : info.message,
  };
  if (info.hint) payload.hint = info.hint;
  return { content: [{ type: 'text', text: `Error: ${JSON.stringify(payload, null, 2)}` }], isError: true };
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

/**
 * Normalize a probe-point argument: `[[x,y,z],…]`, a flat `[x,y,z,…]`, or a
 * single `[x,y,z]`, into a `Float64Array` of flat coordinates.
 *
 * Both shapes are accepted because both are natural to write and the flat form
 * is what the kernel takes; rejecting one would be a papercut with no upside.
 * Throws with a message naming the accepted shapes.
 */
export function flattenProbes(value, label = 'probes') {
  if (!Array.isArray(value) || value.length === 0) {
    throw new Error(`${label} must be a non-empty array of [x,y,z] points or a flat [x,y,z,…] array`);
  }
  const flat = [];
  if (Array.isArray(value[0])) {
    for (const p of value) {
      if (!Array.isArray(p) || p.length !== 3) {
        throw new Error(`${label}: every point must be [x,y,z], got ${JSON.stringify(p)}`);
      }
      flat.push(...p);
    }
  } else {
    flat.push(...value);
  }
  if (flat.length % 3 !== 0) {
    throw new Error(`${label} has ${flat.length} coordinates, which is not a whole number of points`);
  }
  if (!flat.every((c) => Number.isFinite(c))) {
    throw new Error(`${label} contains a non-finite coordinate`);
  }
  return new Float64Array(flat);
}

/**
 * The tolerance an assertion allows: an explicit absolute `tolerance`, an
 * explicit `relative_tolerance` of the expected magnitude, or
 * `DEFAULT_RELATIVE_TOLERANCE` of it.
 *
 * Zero is a legitimate absolute tolerance (an exact integer count), so the
 * check is `!== undefined` rather than truthiness.
 */
function toleranceFor(spec, expected) {
  if (spec.tolerance !== undefined) {
    if (!(Number.isFinite(spec.tolerance) && spec.tolerance >= 0)) {
      throw new Error(`tolerance must be a non-negative number, got ${spec.tolerance}`);
    }
    return spec.tolerance;
  }
  const rel = spec.relative_tolerance ?? DEFAULT_RELATIVE_TOLERANCE;
  if (!(Number.isFinite(rel) && rel >= 0)) {
    throw new Error(`relative_tolerance must be a non-negative number, got ${rel}`);
  }
  return rel * Math.abs(expected);
}

/**
 * The line-work a screenshot's edge modes draw: the mesh's crease/boundary
 * feature edges (view-independent, already in hand from meshing) plus the
 * shape's silhouette for this camera (view-dependent, a separate kernel call).
 *
 * Both are asked at the *same* accuracy the render meshes at, or the outline
 * traces a different tessellation than the surface underneath it and lands a
 * pixel or two off.
 *
 * A shape whose silhouette the kernel declines (a mesh-fallback body, say) is
 * not a failed screenshot — the feature edges alone still draw a usable
 * drawing, so the silhouette is dropped rather than raised.
 */
function edgesFor(shape, mesh, args) {
  let silhouette;
  try {
    const [vx, vy, vz] = viewDirection({ view: args.view, direction: args.direction });
    silhouette = shape.silhouetteEdges(vx, vy, vz, mesh.accuracy);
  } catch {
    silhouette = undefined;
  }
  return { feature: mesh.featureEdges, silhouette };
}

/** One assertion result, in the shape every `assert_model` check returns. */
function checked(type, ok, expected, actual, message) {
  return { type, ok, expected, actual, message };
}

/**
 * Compare a scalar against an expectation, or report why it could not be
 * compared. A null `actual` is a *failure*, not a skip: "the tool could not
 * measure this" is exactly the case that used to pass silently.
 */
function checkScalar(type, spec, actual) {
  if (!Number.isFinite(spec.value)) {
    return checked(type, false, spec.value, actual, `${type}: 'value' must be a finite number`);
  }
  if (!Number.isFinite(actual)) {
    return checked(
      type,
      false,
      spec.value,
      actual,
      `${type} could not be measured (got ${actual}); a null measurement is not a passing check`,
    );
  }
  const tol = toleranceFor(spec, spec.value);
  const delta = actual - spec.value;
  const ok = Math.abs(delta) <= tol;
  return checked(
    type,
    ok,
    spec.value,
    actual,
    ok
      ? `${type} ${actual} is within ${tol} of ${spec.value}`
      : `${type} ${actual} differs from ${spec.value} by ${delta} (allowed ${tol})`,
  );
}

/**
 * Compare a 3-vector componentwise. A `null` component in the expectation skips
 * that axis, so "centred in x and z, don't care about y" is expressible.
 */
function checkVector(type, spec, actual, expectedName = 'value') {
  const expected = spec[expectedName];
  if (!Array.isArray(expected) || expected.length !== 3) {
    return checked(type, false, expected, actual, `${type}: '${expectedName}' must be [x,y,z]`);
  }
  if (!Array.isArray(actual) || actual.length !== 3 || !actual.every((c) => Number.isFinite(c))) {
    return checked(
      type,
      false,
      expected,
      actual,
      `${type} could not be measured (got ${JSON.stringify(actual)})`,
    );
  }
  const failures = [];
  for (let i = 0; i < 3; i += 1) {
    if (expected[i] === null || expected[i] === undefined) continue;
    if (!Number.isFinite(expected[i])) {
      failures.push(`component ${i} of '${expectedName}' is not a number`);
      continue;
    }
    const tol = toleranceFor(spec, expected[i]);
    const delta = actual[i] - expected[i];
    if (Math.abs(delta) > tol) {
      failures.push(`axis ${'xyz'[i]}: ${actual[i]} vs ${expected[i]} (off by ${delta}, allowed ${tol})`);
    }
  }
  return checked(
    type,
    failures.length === 0,
    expected,
    actual,
    failures.length === 0 ? `${type} matches` : `${type}: ${failures.join('; ')}`,
  );
}

/** Compare an integer count exactly. */
function checkCount(type, spec, actual, what) {
  if (!Number.isInteger(spec.value) || spec.value < 0) {
    return checked(type, false, spec.value, actual, `${type}: 'value' must be a non-negative integer`);
  }
  if (!Number.isInteger(actual)) {
    return checked(
      type,
      false,
      spec.value,
      actual,
      `${type} could not be counted (got ${actual}) — the mesh may not be a closed surface`,
    );
  }
  const ok = actual === spec.value;
  return checked(
    type,
    ok,
    spec.value,
    actual,
    ok ? `${actual} ${what}, as expected` : `expected ${spec.value} ${what}, found ${actual}`,
  );
}

/**
 * Lazy, memoized views of one model, so a batch of assertions meshes and
 * measures the part once however many of them need it. Meshing a bracket at a
 * fine accuracy is the expensive part of any of these calls.
 */
function modelViews(shape, accuracy) {
  let measure;
  let validation;
  let census;
  let brep;
  return {
    shape,
    measure: () => (measure ??= JSON.parse(shape.measure(accuracy))),
    validation: () => (validation ??= JSON.parse(shape.validate(accuracy, undefined))),
    census: () => (census ??= inspectTopology(shape, getMesh(shape, { accuracy }))),
    brep: () => (brep ??= JSON.parse(shape.brepCheck(false))),
  };
}

/** Every `assert_model` check type, for the schema and the error message. */
const ASSERTION_TYPES = [
  'volume',
  'surface_area',
  'bbox_size',
  'centroid',
  'closed_solid',
  'shells',
  'genus',
  'planar_faces',
  'through_holes',
  'hole_at',
  'material_at',
  'clearance',
  'brep_sound',
];

/**
 * Evaluate one assertion against a model. Returns `{ type, ok, expected,
 * actual, message }`.
 *
 * The design rule throughout: an assertion that cannot be evaluated **fails**.
 * A null volume, an unmeasurable genus, or an absent B-Rep are all reported as
 * `ok: false` with the reason, never as a pass. The friction log's core finding
 * is that a part with sideways holes satisfied every oracle it was offered, and
 * a check that quietly abstains reproduces exactly that.
 */
export function evaluateAssertion(spec, views) {
  const type = spec && spec.type;
  switch (type) {
    case 'volume':
      return checkScalar('volume', spec, views.measure().volume);
    case 'surface_area':
      return checkScalar('surface_area', spec, views.measure().surfaceArea);
    case 'bbox_size': {
      const box = views.measure().boundingBox;
      return checkVector('bbox_size', spec, box ? box.size : null);
    }
    case 'centroid':
      return checkVector('centroid', spec, views.measure().centroid);
    case 'closed_solid': {
      const v = views.validation();
      return checked(
        'closed_solid',
        v.valid === true,
        true,
        v.valid,
        v.valid ? 'closed, consistently oriented solid' : `not a valid solid: ${v.issues.join('; ')}`,
      );
    }
    case 'shells':
      return checkCount('shells', spec, views.census().counts.shells, 'disconnected shells');
    case 'genus':
      return checkCount(
        'genus',
        spec,
        views.census().counts.genus,
        'handles (through-holes) in the surface',
      );
    case 'planar_faces':
      return checkCount('planar_faces', spec, views.census().counts.planarFaces, 'planar faces');
    case 'through_holes': {
      // Optionally filtered by axis and diameter, which is the assertion that
      // catches a hole drilled on the wrong axis: the count is right and the
      // axis is not.
      const all = views.census().cylinders.filter((c) => c.kind === 'through-hole');
      let matching = all;
      const notes = [];
      if (spec.axis !== undefined) {
        const axis = spec.axis;
        if (!Array.isArray(axis) || axis.length !== 3 || !axis.every((c) => Number.isFinite(c))) {
          return checked('through_holes', false, spec.value, null, "through_holes: 'axis' must be [x,y,z]");
        }
        const len = Math.hypot(...axis);
        if (len === 0) {
          return checked('through_holes', false, spec.value, null, "through_holes: 'axis' must be non-zero");
        }
        const unit = axis.map((c) => c / len);
        const cosTol = Math.cos(((spec.axis_tolerance_deg ?? 5) * Math.PI) / 180);
        // Absolute dot: a hole has no preferred direction along its own axis.
        matching = matching.filter(
          (c) => Math.abs(c.axis[0] * unit[0] + c.axis[1] * unit[1] + c.axis[2] * unit[2]) >= cosTol,
        );
        notes.push(`axis ≈ [${unit.map((c) => c.toFixed(3))}]`);
      }
      if (spec.diameter !== undefined) {
        const tol = toleranceFor(spec, spec.diameter);
        matching = matching.filter((c) => Math.abs(c.diameter - spec.diameter) <= tol);
        notes.push(`Ø${spec.diameter} ±${tol}`);
      }
      const result = checkCount(
        'through_holes',
        spec,
        matching.length,
        `through-holes${notes.length ? ` matching ${notes.join(', ')}` : ''}`,
      );
      // When the filter is what failed, say what *was* found — "0 holes on +Z"
      // is far less useful than "0 on +Z; 4 on +Y".
      if (!result.ok && all.length !== matching.length) {
        result.message +=
          `. The part has ${all.length} through-hole(s) in total: ` +
          all
            .map(
              (c) =>
                `Ø${c.diameter.toFixed(3)} along [${c.axis.map((v) => v.toFixed(2))}] ` +
                `at [${c.center.map((v) => v.toFixed(2))}]`,
            )
            .join('; ');
      }
      return result;
    }
    case 'hole_at': {
      // "There is a bore through `at`, running along `axis`" is two facts, and
      // checking only one of them passes on nonsense. Along the axis the line
      // must be *clear* — that is what makes it a bore rather than material.
      // Across the axis it must go material → gap → material, which is what
      // makes the clear line a hole in something rather than a line in free
      // space beside the part. Both, or neither means anything.
      const expected = { at: spec.at, axis: spec.axis, diameter: spec.diameter };
      if (spec.at === undefined || spec.axis === undefined) {
        return checked('hole_at', false, expected, null, "hole_at needs both 'at' and 'axis'");
      }
      const along = probeAxis(views.shape, spec.axis, spec.at);
      if (along.materialLength > 0) {
        return checked(
          'hole_at',
          false,
          expected,
          { clearAlongAxis: false, materialLength: along.materialLength },
          `the line through [${spec.at}] along [${along.axis.map((v) => v.toFixed(2))}] ` +
            `crosses ${along.materialLength} of material, so no bore runs that way here. ` +
            'Name the bore\'s own axis and a point on its centreline.',
        );
      }
      // Two directions across the axis. A circular bore reads the same width
      // from any of them, so both agreeing is also a check that the void is a
      // bore and not a slot.
      const a = along.axis;
      const seed = Math.abs(a[0]) < 0.9 ? [1, 0, 0] : [0, 1, 0];
      const cross = (p, q) => [
        p[1] * q[2] - p[2] * q[1],
        p[2] * q[0] - p[0] * q[2],
        p[0] * q[1] - p[1] * q[0],
      ];
      const u = cross(a, seed);
      const across = [u, cross(a, u)].map((dir) => probeAxis(views.shape, dir, spec.at));
      const enclosed = across.filter((p) => p.throughHole);
      if (enclosed.length < 2) {
        return checked(
          'hole_at',
          false,
          expected,
          { clearAlongAxis: true, enclosedDirections: enclosed.length },
          `[${spec.at}] is clear along [${a.map((v) => v.toFixed(2))}], but the void around it ` +
            `is enclosed by material in only ${enclosed.length} of 2 directions across that ` +
            'axis — this reads as open space beside the part, not a bore through it',
        );
      }
      // The narrower of the two crossings: for a point on the centreline both
      // equal the diameter, and a point off-centre reads narrower in one.
      const width = Math.min(...across.map((p) => p.gapLength));
      if (spec.diameter === undefined) {
        return checked(
          'hole_at',
          true,
          { at: spec.at, axis: spec.axis },
          width,
          `a bore runs through [${spec.at}] along [${a.map((v) => v.toFixed(2))}], ${width} across`,
        );
      }
      const tol = toleranceFor(spec, spec.diameter);
      const ok = Math.abs(width - spec.diameter) <= tol;
      return checked(
        'hole_at',
        ok,
        spec.diameter,
        width,
        ok
          ? `bore present, ${width} across`
          : `bore present but ${width} across, not ${spec.diameter} (allowed ${tol}). ` +
            'A point off the bore centreline reads narrower than the diameter — check `at`.',
      );
    }
    case 'material_at': {
      if (!Array.isArray(spec.at) || spec.at.length !== 3) {
        return checked('material_at', false, spec.value, null, "material_at needs 'at' as [x,y,z]");
      }
      const want = spec.value === undefined ? true : Boolean(spec.value);
      const d = views.shape.distance(spec.at[0], spec.at[1], spec.at[2]);
      const isSolid = d < 0;
      return checked(
        'material_at',
        isSolid === want,
        want,
        isSolid,
        `[${spec.at}] is ${isSolid ? 'inside' : 'outside'} the solid (signed distance ${d})`,
      );
    }
    case 'clearance': {
      if (spec.min === undefined || !Number.isFinite(spec.min)) {
        return checked('clearance', false, spec.min, null, "clearance needs a finite 'min'");
      }
      let flat;
      try {
        flat = flattenProbes(spec.probes, 'clearance.probes');
      } catch (err) {
        return checked('clearance', false, spec.min, null, err.message);
      }
      let worst = Infinity;
      let worstAt = null;
      for (let i = 0; i < flat.length; i += 3) {
        const d = views.shape.distance(flat[i], flat[i + 1], flat[i + 2]);
        if (d < worst) {
          worst = d;
          worstAt = [flat[i], flat[i + 1], flat[i + 2]];
        }
      }
      const ok = worst >= spec.min;
      return checked(
        'clearance',
        ok,
        spec.min,
        worst,
        ok
          ? `every probe clears the solid by at least ${worst}`
          : `the nearest probe [${worstAt}] is ${worst} from the solid, under the required ${spec.min}` +
            (worst < 0 ? ' (negative means the probe is inside the material)' : ''),
      );
    }
    case 'brep_sound': {
      const b = views.brep();
      if (!b.available) {
        return checked('brep_sound', false, 'a checkable B-Rep', null, `no B-Rep to check: ${b.reason}`);
      }
      const ok = b.failures.length === 0;
      return checked(
        'brep_sound',
        ok,
        0,
        b.failures.length,
        ok
          ? 'the B-Rep body passes the kernel check'
          : `${b.failures.length} B-Rep defect(s): ${b.failures.map((f) => f.message).join('; ')}`,
      );
    }
    default:
      return checked(
        String(type),
        false,
        null,
        null,
        `unknown assertion type '${type}'; use one of ${ASSERTION_TYPES.join(', ')}`,
      );
  }
}

/** The mate kinds `assemble` accepts, for the schema and error messages. */
const MATE_KINDS = ['coincident', 'concentric', 'distance'];
/** The mate feature types, likewise. */
const FEATURE_TYPES = ['plane', 'axis', 'point'];

/**
 * Normalize one side of a mate (`{instance, feature}`) into the flat form the
 * kernel binding takes. Throws with the side's label (`a` / `b`) in the
 * message, because "which side was wrong" is the first thing a caller fixing
 * a mate needs to know.
 */
function mateSide(side, label) {
  if (!side || !Number.isInteger(side.instance) || side.instance < 0) {
    throw new Error(`mate side '${label}' needs a non-negative integer 'instance' index`);
  }
  const f = side.feature;
  if (!f || !FEATURE_TYPES.includes(f.type)) {
    throw new Error(
      `mate side '${label}' needs a feature with type one of ${FEATURE_TYPES.join(', ')}`,
    );
  }
  if (!Array.isArray(f.point) || f.point.length !== 3) {
    throw new Error(`mate side '${label}': feature.point must be [x, y, z] in the part's local frame`);
  }
  // Planes orient by `normal`, axes by `direction`; a point has no direction
  // (zeros are passed through and ignored kernel-side).
  const direction = f.type === 'plane' ? f.normal : f.type === 'axis' ? f.direction : [0, 0, 0];
  if (f.type !== 'point' && (!Array.isArray(direction) || direction.length !== 3)) {
    throw new Error(
      `mate side '${label}': a ${f.type} feature needs ${
        f.type === 'plane' ? "'normal'" : "'direction'"
      } as [x, y, z]`,
    );
  }
  return { instance: side.instance, type: f.type, point: f.point, direction };
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
          return failErr(err, 'script failed');
        }
        const measure = JSON.parse(model.shape.measure(undefined));
        const validation = JSON.parse(model.shape.validate(undefined, undefined));
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
              // Which mesh `valid`/`volume` are statements about, and whether an
              // exact B-Rep was checked at all — so a bare `valid: true` is not
              // mistaken for a full bill of health. `validate` carries the whole
              // report; `inspect_topology` is where structure gets checked.
              mesher: validation.mesher,
              brepChecked: validation.brep.available,
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
          return failErr(err, 'import failed');
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
          const validation = JSON.parse(model.shape.validate(undefined, undefined));
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
                // Imported topology is exactly what `TopologyStore::check` was
                // written for — a body of unknown provenance, where "the file
                // parsed" says nothing about whether the geometry holds
                // together. `validate` on this model_id gives the full report,
                // `deep: true` adds self-intersection.
                mesher: validation.mesher,
                brepChecked: validation.brep.available,
                ...(validation.brep.available && validation.brep.counts
                  ? { brepCounts: validation.brep.counts }
                  : {}),
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

    assemble: {
      definition: {
        name: 'assemble',
        description:
          'Compose registered models into a multi-part assembly: place instances, ' +
          'constrain them with mates, solve for the poses, and get back the resolved ' +
          'transforms, an interference (clash) report, and aggregate mass properties. ' +
          'Instances reference models by model_id (the same model may be instanced any ' +
          'number of times — geometry is shared, never copied). Mates constrain features ' +
          'given in each part\'s LOCAL frame: coincident (plane–plane flush, or ' +
          'point-on-plane), concentric (axis–axis, a shaft in a bore), and distance ' +
          '(plane–plane or point–point, with `value`). The solver holds `fixed` ' +
          'instances as ground and moves the floating ones; `solve.status` reports ' +
          '`converged` or `over_constrained` (conflicting mates — poses are the ' +
          'least-squares best fit), and `solve.freeDof` counts the degrees of freedom ' +
          'still unconstrained (a seated bolt free to spin reports 1; that is normal, ' +
          'not an error). The returned assembly_id is a model like any other: ' +
          'get_screenshot to see the assembly, measure it, export it (faceted geometry ' +
          '— the placed union has no analytic B-Rep), diff it. Interference is checked ' +
          'pairwise at the solved poses; `interference.pairs` lists each clashing pair ' +
          'with its estimated overlap volume. Mass properties compose per-part results ' +
          'with per-instance `density` (overlapping instances double-count the ' +
          'overlap, which a well-mated assembly keeps near zero).',
        inputSchema: {
          type: 'object',
          properties: {
            instances: {
              type: 'array',
              description:
                'The placed parts, in order — mates reference them by this index.',
              items: {
                type: 'object',
                properties: {
                  model_id: { type: 'string', description: 'A model registered by create_model / import_step.' },
                  transform: {
                    type: 'object',
                    description:
                      'Initial placement (default identity). For floating instances ' +
                      'this is the solver\'s starting guess; for fixed instances it is final.',
                    properties: {
                      translation: {
                        type: 'array',
                        items: { type: 'number' },
                        minItems: 3,
                        maxItems: 3,
                        description: '[x, y, z] offset (default [0,0,0]).',
                      },
                      rotation: {
                        type: 'object',
                        description: 'Rotation about an axis through the origin, applied before the translation.',
                        properties: {
                          axis: { type: 'array', items: { type: 'number' }, minItems: 3, maxItems: 3 },
                          angle_deg: { type: 'number' },
                        },
                        required: ['axis', 'angle_deg'],
                      },
                    },
                  },
                  fixed: {
                    type: 'boolean',
                    description:
                      'Ground this instance: the solver holds its pose constant. At ' +
                      'least one fixed instance keeps a mated assembly from floating freely.',
                  },
                  density: {
                    type: 'number',
                    description:
                      'Mass per model unit³ (default 1). Only affects the aggregate ' +
                      'mass properties, not the solve.',
                  },
                  name: { type: 'string', description: 'Display name (defaults to the model\'s name).' },
                },
                required: ['model_id'],
              },
            },
            mates: {
              type: 'array',
              description:
                'Constraints between instance features. Each side is `{instance, ' +
                'feature}` where `feature` is `{type: plane|axis|point, point: [x,y,z], ' +
                'normal|direction: [x,y,z]}` in that instance\'s local (part) frame — ' +
                'e.g. the bore axis a shaft mates into, as reported by ' +
                'inspect_topology on the part. Kinds: coincident (plane–plane or ' +
                'point–plane), concentric (axis–axis), distance (plane–plane or ' +
                'point–point, requires `value`).',
              items: {
                type: 'object',
                properties: {
                  kind: { type: 'string', enum: MATE_KINDS },
                  a: {
                    type: 'object',
                    description: 'First side: {instance, feature}.',
                    properties: {
                      instance: { type: 'number', description: 'Index into `instances`.' },
                      feature: {
                        type: 'object',
                        properties: {
                          type: { type: 'string', enum: FEATURE_TYPES },
                          point: { type: 'array', items: { type: 'number' }, minItems: 3, maxItems: 3 },
                          normal: { type: 'array', items: { type: 'number' }, minItems: 3, maxItems: 3, description: 'Plane normal (plane features).' },
                          direction: { type: 'array', items: { type: 'number' }, minItems: 3, maxItems: 3, description: 'Axis direction (axis features).' },
                        },
                        required: ['type', 'point'],
                      },
                    },
                    required: ['instance', 'feature'],
                  },
                  b: {
                    type: 'object',
                    description: 'Second side, same shape as `a`.',
                    properties: {
                      instance: { type: 'number' },
                      feature: {
                        type: 'object',
                        properties: {
                          type: { type: 'string', enum: FEATURE_TYPES },
                          point: { type: 'array', items: { type: 'number' }, minItems: 3, maxItems: 3 },
                          normal: { type: 'array', items: { type: 'number' }, minItems: 3, maxItems: 3 },
                          direction: { type: 'array', items: { type: 'number' }, minItems: 3, maxItems: 3 },
                        },
                        required: ['type', 'point'],
                      },
                    },
                    required: ['instance', 'feature'],
                  },
                  value: { type: 'number', description: 'Signed offset, for distance mates.' },
                },
                required: ['kind', 'a', 'b'],
              },
            },
            solve: {
              type: 'boolean',
              description:
                'Run the mate solver (default: true when mates are given, false ' +
                'otherwise). With false, instances stay exactly where their transforms ' +
                'put them — a purely positional assembly.',
            },
            name: { type: 'string', description: 'Optional friendly name for the assembly model.' },
          },
          required: ['instances'],
        },
      },
      handler(args) {
        if (!Array.isArray(args.instances) || args.instances.length === 0) {
          return fail('instances must be a non-empty array of {model_id, …}');
        }
        const mateSpecs = args.mates === undefined ? [] : args.mates;
        if (!Array.isArray(mateSpecs)) {
          return fail('mates must be an array of {kind, a, b, …} constraints');
        }
        const asm = new Assembly();
        try {
          const instanceViews = [];
          for (let i = 0; i < args.instances.length; i += 1) {
            const spec = args.instances[i];
            if (!spec || typeof spec.model_id !== 'string') {
              return fail(`instances[${i}] needs a model_id`);
            }
            let model;
            try {
              model = store.get(spec.model_id);
            } catch (err) {
              return fail(`instances[${i}]: ${err.message}`);
            }
            const t = spec.transform || {};
            const translation = t.translation === undefined ? [0, 0, 0] : t.translation;
            const rotation = t.rotation || {};
            const axis = rotation.axis === undefined ? [0, 0, 0] : rotation.axis;
            const angleDeg = rotation.angle_deg === undefined ? 0 : rotation.angle_deg;
            const fixed = Boolean(spec.fixed);
            const density = spec.density === undefined ? 1 : spec.density;
            const name = spec.name || model.name;
            try {
              asm.addInstance(model.shape, translation, axis, angleDeg, fixed, density, name);
            } catch (err) {
              return failErr(err, `instances[${i}]`);
            }
            instanceViews.push({ index: i, model_id: model.id, name, fixed, density });
          }

          for (let i = 0; i < mateSpecs.length; i += 1) {
            const spec = mateSpecs[i];
            if (!spec || !MATE_KINDS.includes(spec.kind)) {
              return fail(`mates[${i}]: 'kind' must be one of ${MATE_KINDS.join(', ')}`);
            }
            let a;
            let b;
            try {
              a = mateSide(spec.a, 'a');
              b = mateSide(spec.b, 'b');
            } catch (err) {
              return fail(`mates[${i}]: ${err.message}`);
            }
            try {
              asm.addMate(
                spec.kind,
                a.instance,
                a.type,
                a.point,
                a.direction,
                b.instance,
                b.type,
                b.point,
                b.direction,
                spec.value === undefined ? undefined : spec.value,
              );
            } catch (err) {
              return failErr(err, `mates[${i}]`);
            }
          }

          const solveRequested = args.solve === undefined ? mateSpecs.length > 0 : Boolean(args.solve);
          const solve = solveRequested
            ? JSON.parse(asm.solve())
            : {
                status: 'skipped',
                note: 'No solve was run; instances sit exactly where their transforms placed them.',
              };
          // Post-solve poses for every instance, solved or not — the answer
          // to "where did everything end up?".
          const transforms = JSON.parse(asm.transforms());
          const interference = JSON.parse(asm.interferences());
          const massProperties = JSON.parse(asm.massProperties());

          let shape;
          try {
            shape = asm.assembledShape();
          } catch (err) {
            return failErr(err);
          }
          const placedInstances = instanceViews.map((v, i) => ({ ...v, transform: transforms[i] }));
          const model = store.registerAssembly({
            shape,
            name: args.name,
            // The recipe, kept whole so `get_model` can reproduce the how —
            // which models went where, under which constraints, and what the
            // solver concluded.
            assembly: { instances: placedInstances, mates: mateSpecs, solve },
          });

          const measure = JSON.parse(model.shape.measure(undefined));
          const validation = JSON.parse(model.shape.validate(undefined, undefined));
          return text(
            withMassError(
              {
                assembly_id: model.id,
                name: model.name,
                instances: placedInstances,
                solve,
                interference,
                massProperties,
                mesh: { triangles: measure.triangles, vertices: measure.vertices },
                boundingBox: measure.boundingBox,
                volume: measure.volume,
                valid: validation.valid,
                issues: validation.issues,
                mesher: validation.mesher,
                brepChecked: validation.brep.available,
              },
              measure,
            ),
          );
        } finally {
          // The registered model owns the assembled shape; this releases the
          // builder's own wasm handle rather than waiting on a finalizer.
          asm.free();
        }
      },
    },

    get_screenshot: {
      definition: {
        name: 'get_screenshot',
        description:
          'Render a model to a PNG image and return it inline, followed by a text block ' +
          'naming the exact camera that produced it. This is the visual-inspection ' +
          'channel: a smoke test that catches geometry which is topologically valid but ' +
          'semantically wrong (a hole on the wrong axis, a pocket that broke through), ' +
          'which no measurement flags on its own. ' +
          `Views: ${VIEW_NAMES.join(', ')} (default iso), or pass an arbitrary ` +
          '`direction`. Framing: `region` frames a world-space box (paste the ' +
          '`boundingBox` from `measure`), `target` re-centres on a point (paste a hole or ' +
          "boss `center` from `inspect_topology`) and `zoom` magnifies. `section` cuts an " +
          'axis-aligned clip plane and shades the cut face ' +
          `so interior geometry is visible. \`mode\`: ${RENDER_MODES.join(', ')} — ` +
          'edge modes draw feature + silhouette line-work with hidden lines removed, ' +
          'which reads like a dimension drawing. Rendering is deterministic: the same ' +
          'arguments against the same model produce byte-identical PNGs.',
        inputSchema: {
          type: 'object',
          properties: {
            model_id: { type: 'string' },
            view: { type: 'string', enum: VIEW_NAMES, description: 'Named camera view (default iso).' },
            direction: {
              type: 'array',
              items: { type: 'number' },
              minItems: 3,
              maxItems: 3,
              description:
                'Arbitrary view direction [x,y,z] — the way the camera looks, so [0,-1,0] ' +
                'looks down. Overrides `view`. Use it for three-quarter shots a named view ' +
                'cannot reach.',
            },
            up: {
              type: 'array',
              items: { type: 'number' },
              minItems: 3,
              maxItems: 3,
              description:
                'Screen-up vector for a custom `direction` (default +Y, or +/-Z when the ' +
                'direction is vertical).',
            },
            region: {
              type: 'object',
              description:
                'World-space box to frame, {min:[x,y,z], max:[x,y,z]} — the same shape ' +
                '`measure` and `inspect_topology` report a bounding box in. Zooms to that ' +
                'box instead of the whole model, which is how you inspect one feature.',
              properties: {
                min: { type: 'array', items: { type: 'number' }, minItems: 3, maxItems: 3 },
                max: { type: 'array', items: { type: 'number' }, minItems: 3, maxItems: 3 },
              },
              required: ['min', 'max'],
            },
            target: {
              type: 'array',
              items: { type: 'number' },
              minItems: 3,
              maxItems: 3,
              description:
                'World point to place at the centre of the frame (default: the centre of ' +
                'whatever is being fitted).',
            },
            zoom: {
              type: 'number',
              description:
                'Magnification on top of the fit: 1 (default) fits, 2 is twice as close ' +
                'and shows a quarter of the area, 0.5 pulls back.',
            },
            mode: {
              type: 'string',
              enum: RENDER_MODES,
              description:
                'shaded (default) | shaded_edges (solid plus line-work) | edges ' +
                '(line-work on a light ground, hidden lines removed).',
            },
            section: {
              type: 'object',
              description:
                'Axis-aligned section cut. Keeps the half with the smaller coordinate on ' +
                '`axis`; `flip` keeps the other. The cut face is shaded amber so it can ' +
                'never be mistaken for material.',
              properties: {
                axis: { type: 'string', enum: SECTION_AXES, description: 'X | Y | Z (default X).' },
                offset: {
                  type: 'number',
                  description:
                    'Where the plane sits on that axis, in model units (default: the ' +
                    "model's midpoint on that axis, which is guaranteed to cut it).",
                },
                flip: { type: 'boolean', description: 'Keep the other half (default false).' },
              },
            },
            line_width: {
              type: 'number',
              description: 'Line-work thickness in px, 1..8 (default 2 in `edges`, 1 overlaid).',
            },
            accuracy: {
              type: 'number',
              description:
                'Chordal deviation of the rendered mesh in model units (default 0.5% of ' +
                'the extent). Pin it when a series of shots must be compared facet-for-facet.',
            },
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
        if (args.accuracy !== undefined && (!Number.isFinite(args.accuracy) || args.accuracy <= 0)) {
          return fail('accuracy must be a positive number of model units');
        }
        const mesh = getMesh(model.shape, { accuracy: args.accuracy });
        if (mesh.triangles === 0) {
          return fail('model produced an empty mesh; nothing to render');
        }
        const mode = args.mode === undefined ? 'shaded' : args.mode;
        let result;
        try {
          result = renderScene(mesh, model.shape.bounds(), {
            view: args.view,
            direction: args.direction,
            up: args.up,
            region: args.region,
            target: args.target,
            zoom: args.zoom,
            mode,
            section: args.section,
            lineWidth: args.line_width,
            edges: mode === 'shaded' ? undefined : edgesFor(model.shape, mesh, args),
            width: args.width,
            height: args.height,
          });
        } catch (err) {
          return failErr(err);
        }
        return {
          content: [
            {
              type: 'image',
              data: result.png.toString('base64'),
              mimeType: 'image/png',
            },
            // The resolved camera, not the requested one. A shot an agent
            // cannot reproduce is not evidence: this is what turns "looks
            // wrong" into a second, tighter shot of the same thing.
            {
              type: 'text',
              text: JSON.stringify({
                model_id: model.id,
                accuracy: mesh.accuracy,
                camera: result.camera,
              }),
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
          return failErr(err, 'export failed');
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
          'Check a model on two fronts: its mesh is a closed, consistently oriented ' +
          'manifold enclosing a finite non-zero volume, and — when the model carries an ' +
          'exact B-Rep (`exact: true`, or an un-booleaned primitive) — that body passes ' +
          "the kernel's own validation: referential integrity, loop bookkeeping, " +
          'edge/face closure and orientation, the Euler-Poincaré formula, edges lying on ' +
          'their faces, pcurve fidelity, face sense against loop winding, and (with ' +
          '`deep`) face-face self-intersection. `mesher` names which mesh answered; ' +
          '`brep` carries the body report, including a reason when there is no B-Rep to ' +
          'check — so a `valid: true` always says what it did *not* verify. Mesh closure ' +
          'alone cannot see a hole bored on the wrong axis; pair this with ' +
          '`inspect_topology` or `assert_model` for that.',
        inputSchema: {
          type: 'object',
          properties: {
            model_id: { type: 'string' },
            accuracy: { type: 'number', description: 'Target chordal deviation for the checked mesh.' },
            deep: {
              type: 'boolean',
              description:
                'Also run the B-Rep self-intersection pass (every face pair tested for ' +
                'clashes away from shared topology). Off by default: it is a pairwise ' +
                'search over faces, not a single walk of the body.',
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
        return text(
          JSON.parse(model.shape.validate(accuracyArg(args.accuracy), Boolean(args.deep))),
        );
      },
    },

    inspect_topology: {
      definition: {
        name: 'inspect_topology',
        description:
          'Interrogate a model\'s structure rather than its scalars: planar faces (with ' +
          'normals, plane offsets and areas), circular rims, cylindrical features ' +
          'classified as through-holes or bosses (each with its axis, diameter, centre ' +
          'and depth), shell count, and genus — the number of handles in the surface, ' +
          'i.e. how many holes go through the part. Optionally casts `probes`: lines ' +
          'through a point along an axis, reporting the solid/void spans they cross, ' +
          'which answers "is there a hole through here, along this direction?" directly. ' +
          'For models with an exact B-Rep the report also carries that body\'s authoritative ' +
          'entity counts (faces, edges, vertices, hole loops, and how many faces sit on ' +
          'planes vs cylinders vs spheres). This is the oracle `validate`, `measure`, ' +
          '`boundingBox` and a screenshot all miss: a hole drilled on the wrong axis ' +
          'removes nearly the right volume and renders plausibly, but it is a different ' +
          'shape, and the axis is right here.',
        inputSchema: {
          type: 'object',
          properties: {
            model_id: { type: 'string' },
            accuracy: {
              type: 'number',
              description:
                'Target chordal deviation of the inspected mesh. Finer resolves small ' +
                'features; the genus and shell counts are exact combinatorics and do not ' +
                'drift with it.',
            },
            probes: {
              type: 'array',
              description:
                'Lines to cast. Each is `{axis:[x,y,z], at:[x,y,z]}` — the line through ' +
                '`at` along `axis`, reported as the ordered solid/void spans it crosses ' +
                'plus a `throughHole` verdict (material, gap, material).',
              items: {
                type: 'object',
                properties: {
                  axis: { type: 'array', items: { type: 'number' }, description: '[x,y,z] direction.' },
                  at: { type: 'array', items: { type: 'number' }, description: '[x,y,z] point on the line.' },
                },
                required: ['axis', 'at'],
              },
            },
            include_faces: {
              type: 'boolean',
              description:
                'Include the per-face list (normal, offset, area). Default true; set false ' +
                'for just the counts on a part with many faces.',
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
        const accuracy = accuracyArg(args.accuracy);
        let census;
        try {
          census = inspectTopology(model.shape, getMesh(model.shape, { accuracy }));
        } catch (err) {
          return failErr(err, 'topology inspection failed');
        }
        let probes;
        if (args.probes !== undefined) {
          if (!Array.isArray(args.probes)) {
            return fail('probes must be an array of {axis, at} objects');
          }
          try {
            probes = args.probes.map((p) => {
              const r = probeAxis(model.shape, p && p.axis, p && p.at);
              return {
                axis: r.axis,
                at: r.at,
                throughHole: r.throughHole,
                solidSpans: r.solidSpans,
                voidSpans: r.voidSpans,
                materialLength: r.materialLength,
                gapLength: r.gapLength,
                spans: r.spans,
              };
            });
          } catch (err) {
            return failErr(err, 'probe failed');
          }
        }
        const brep = JSON.parse(model.shape.brepCheck(false));
        return text({
          model_id: model.id,
          ...(accuracy !== undefined ? { accuracy } : {}),
          counts: census.counts,
          mesh: census.mesh,
          cylinders: census.cylinders,
          planarFaces: {
            count: census.planarFaces.faces.length,
            totalArea: census.planarFaces.totalArea,
            // What fraction of the surface the planar faces account for. The
            // rest is genuine curvature plus the one-cell bevel the SDF mesher
            // leaves where the part has a sharp edge — stated, because face
            // areas run that much under their analytic values.
            planarAreaFraction: census.planarFaces.planarAreaFraction,
            remainderArea: census.planarFaces.remainderArea,
            ...(args.include_faces === false ? {} : { faces: census.planarFaces.faces }),
          },
          // The exact body's own counts when there is one: a mesh cannot supply
          // a true face or edge count, because the mesher bevels sharp edges.
          brep: brep.available
            ? { available: true, source: brep.source, counts: brep.counts }
            : { available: false, reason: brep.reason },
          ...(probes ? { probes } : {}),
        });
      },
    },

    assert_model: {
      definition: {
        name: 'assert_model',
        description:
          'Check a model against expected values and report each one pass/fail. This is ' +
          'the tool to reach for before believing a part is right: state what you intended ' +
          '— the volume you computed from the spec, the bounding box, four Ø5 holes along ' +
          '+Y, genus 4, nothing inside a keep-out — and get back which expectations the ' +
          'geometry actually meets. An expectation that cannot be evaluated (a null volume, ' +
          'an unmeasurable genus, an absent B-Rep) FAILS rather than abstaining, because a ' +
          'check that quietly skips is how a wrong part passes. Continuous quantities ' +
          'default to a 1% relative tolerance; counts must match exactly.',
        inputSchema: {
          type: 'object',
          properties: {
            model_id: { type: 'string' },
            accuracy: { type: 'number', description: 'Target chordal deviation for the measured mesh.' },
            expect: {
              type: 'array',
              description:
                'The expectations. Each is `{type, …}`:\n' +
                '• `volume` / `surface_area` — `value`, plus `tolerance` (absolute) or ' +
                '`relative_tolerance` (default 0.01).\n' +
                '• `bbox_size` / `centroid` — `value: [x,y,z]`; a null component skips that axis.\n' +
                '• `closed_solid` — the mesh is a watertight, consistently oriented solid.\n' +
                '• `shells` — `value`: expected number of disconnected shells (2 means the ' +
                'part fell apart).\n' +
                '• `genus` — `value`: handles in the surface, i.e. holes through the part.\n' +
                '• `planar_faces` — `value`: number of planar faces.\n' +
                '• `through_holes` — `value`: count, optionally filtered by `axis: [x,y,z]` ' +
                '(within `axis_tolerance_deg`, default 5) and `diameter`. The axis filter is ' +
                'what catches a hole drilled the wrong way; the failure message lists the ' +
                'holes that *are* there and their axes.\n' +
                '• `hole_at` — `at: [x,y,z]` on a bore\'s centreline, `axis: [x,y,z]` the ' +
                "bore's own direction, optional `diameter`: the line along the axis is clear " +
                'of material and the void around it is enclosed by material in both directions ' +
                'across it. Both, so that free space beside the part cannot pass.\n' +
                '• `material_at` — `at: [x,y,z]`, `value` true (default) or false: whether ' +
                'that point is inside the solid.\n' +
                '• `clearance` — `probes` and `min`: every probe point stands at least `min` ' +
                'clear of the solid.\n' +
                '• `brep_sound` — the exact B-Rep body passes the kernel check (fails when ' +
                'the model has no B-Rep, since then nothing was verified).',
              items: {
                type: 'object',
                properties: {
                  type: { type: 'string', enum: ASSERTION_TYPES },
                  value: { description: 'Expected value: a number, [x,y,z], or a boolean by type.' },
                  tolerance: { type: 'number', description: 'Absolute tolerance.' },
                  relative_tolerance: {
                    type: 'number',
                    description: 'Tolerance as a fraction of the expected magnitude (default 0.01).',
                  },
                  at: { type: 'array', items: { type: 'number' }, description: 'Point, for hole_at / material_at.' },
                  axis: { type: 'array', items: { type: 'number' }, description: 'Direction, for hole_at / through_holes.' },
                  axis_tolerance_deg: { type: 'number', description: 'Angular slack for an axis filter (default 5).' },
                  diameter: { type: 'number', description: 'Expected hole diameter.' },
                  probes: { description: 'Keep-out points for clearance: [[x,y,z],…] or flat [x,y,z,…].' },
                  min: { type: 'number', description: 'Minimum clearance.' },
                },
                required: ['type'],
              },
            },
          },
          required: ['model_id', 'expect'],
        },
      },
      handler(args) {
        let model;
        try {
          model = store.get(args.model_id);
        } catch (err) {
          return fail(err.message);
        }
        if (!Array.isArray(args.expect) || args.expect.length === 0) {
          return fail('expect must be a non-empty array of {type, …} expectations');
        }
        const views = modelViews(model.shape, accuracyArg(args.accuracy));
        const results = [];
        for (const spec of args.expect) {
          try {
            results.push(evaluateAssertion(spec, views));
          } catch (err) {
            results.push(
              checked(String(spec && spec.type), false, null, null, `check failed: ${errMessage(err)}`),
            );
          }
        }
        const failed = results.filter((r) => !r.ok);
        return text({
          model_id: model.id,
          ok: failed.length === 0,
          passed: results.length - failed.length,
          failed: failed.length,
          checks: results,
        });
      },
    },

    diff_models: {
      definition: {
        name: 'diff_models',
        description:
          'Compare two models and report what changed: volume (absolute delta and ratio), ' +
          'surface area, bounding-box size, centroid, and the structural counts — shells, ' +
          'genus, planar faces, through-holes. Built for the before/after question a ' +
          'feature edit actually poses: "I subtracted four Ø5 holes from a 5 mm plate; did ' +
          'they remove the 392.7 mm³ they should have?" `expect_volume_delta` turns that ' +
          'into a pass/fail. A volume delta compared against a number computed from the ' +
          'spec is the only oracle that caught the wrong-axis-hole bug on the gallery ' +
          'bracket, and it needed a hand calculation to do it; this is that oracle, ' +
          'without the paper.',
        inputSchema: {
          type: 'object',
          properties: {
            model_id_a: { type: 'string', description: 'The baseline model (the "before").' },
            model_id_b: { type: 'string', description: 'The model to compare against it (the "after").' },
            accuracy: {
              type: 'number',
              description:
                'Target chordal deviation for BOTH meshes, so the two are measured at the ' +
                'same fidelity — comparing a fine mesh against a coarse one turns a meshing ' +
                'difference into an apparent design difference.',
            },
            expect_volume_delta: {
              type: 'object',
              description:
                'Optional assertion on `volume(b) - volume(a)`. Negative for material removed.',
              properties: {
                value: { type: 'number' },
                tolerance: { type: 'number', description: 'Absolute tolerance.' },
                relative_tolerance: {
                  type: 'number',
                  description: 'Fraction of the expected delta (default 0.01).',
                },
              },
              required: ['value'],
            },
          },
          required: ['model_id_a', 'model_id_b'],
        },
      },
      handler(args) {
        const accuracy = accuracyArg(args.accuracy);
        // Each model carries its own exact-booleans flag, and `store.get` applies
        // it to the process-global toggle — so measure each fully before touching
        // the other, or the second `get` re-routes the first one's meshing.
        const read = (id) => {
          const model = store.get(id);
          const views = modelViews(model.shape, accuracy);
          const m = views.measure();
          const c = views.census().counts;
          return {
            model_id: model.id,
            name: model.name,
            volume: m.volume,
            surfaceArea: m.surfaceArea,
            centroid: m.centroid,
            bboxSize: m.boundingBox ? m.boundingBox.size : null,
            counts: c,
            ...(m.massError ? { massError: m.massError } : {}),
          };
        };
        let a;
        let b;
        try {
          a = read(args.model_id_a);
          b = read(args.model_id_b);
        } catch (err) {
          return fail(err.message);
        }

        const scalarDelta = (x, y) =>
          Number.isFinite(x) && Number.isFinite(y) ? y - x : null;
        const vectorDelta = (x, y) =>
          Array.isArray(x) && Array.isArray(y) ? y.map((v, i) => v - x[i]) : null;
        const volumeDelta = scalarDelta(a.volume, b.volume);
        const countDelta = {};
        for (const key of Object.keys(a.counts)) {
          const av = a.counts[key];
          const bv = b.counts[key];
          countDelta[key] = Number.isFinite(av) && Number.isFinite(bv) ? bv - av : null;
        }

        const payload = {
          a,
          b,
          delta: {
            volume: volumeDelta,
            // Fraction of A's volume, the form a "3% lighter" claim wants.
            volumeRatio:
              Number.isFinite(volumeDelta) && Number.isFinite(a.volume) && a.volume !== 0
                ? volumeDelta / a.volume
                : null,
            surfaceArea: scalarDelta(a.surfaceArea, b.surfaceArea),
            centroid: vectorDelta(a.centroid, b.centroid),
            bboxSize: vectorDelta(a.bboxSize, b.bboxSize),
            counts: countDelta,
          },
        };
        if (args.expect_volume_delta !== undefined) {
          try {
            payload.volumeDeltaCheck = checkScalar(
              'volume_delta',
              args.expect_volume_delta,
              volumeDelta,
            );
          } catch (err) {
            return failErr(err, 'expect_volume_delta');
          }
        }
        return text(payload);
      },
    },

    measure_clearance: {
      definition: {
        name: 'measure_clearance',
        description:
          'Measure how close a model comes to points or to another model — a passive ' +
          'interference check, the read-only counterpart to the `clearance` constraint ' +
          '`optimize` descends on. With `probes`, reports each point\'s signed distance to ' +
          'the solid (negative means the point is inside the material), the nearest one, ' +
          'and the smooth softmin the optimiser uses. With `against_model_id`, samples that ' +
          "model's mesh vertices against this model's field and reports whether the two " +
          'overlap and by how much, with the deepest point of intersection. Nothing else ' +
          'in the toolset answers "do these two parts collide?".',
        inputSchema: {
          type: 'object',
          properties: {
            model_id: { type: 'string' },
            probes: {
              description:
                'Points to measure against the solid: [[x,y,z],…] or a flat [x,y,z,…] array.',
            },
            against_model_id: {
              type: 'string',
              description:
                "Another model to test for interference. Its mesh vertices are measured " +
                "against this model's field, so the resolution of the answer is that mesh's " +
                '`accuracy`: a shallow overlap thinner than the triangle spacing can be missed.',
            },
            accuracy: {
              type: 'number',
              description: 'Target chordal deviation for the other model\'s mesh (interference mode).',
            },
            softness: {
              type: 'number',
              description:
                `Softmin blend for the differentiable clearance value (model units, default ${DEFAULT_CLEARANCE_SOFTNESS}). ` +
                'Smaller approaches a hard min; this affects only the `softMin` field.',
            },
          },
          required: ['model_id'],
        },
      },
      handler(args) {
        if (args.probes === undefined && args.against_model_id === undefined) {
          return fail('give either `probes` (points) or `against_model_id` (another model)');
        }
        let model;
        try {
          model = store.get(args.model_id);
        } catch (err) {
          return fail(err.message);
        }
        const softness =
          Number.isFinite(args.softness) && args.softness > 0
            ? args.softness
            : DEFAULT_CLEARANCE_SOFTNESS;

        /** Measure a flat probe buffer against this model's field. */
        const against = (flat) => {
          const distances = [];
          let min = Infinity;
          let minAt = null;
          let inside = 0;
          for (let i = 0; i < flat.length; i += 3) {
            const d = model.shape.distance(flat[i], flat[i + 1], flat[i + 2]);
            distances.push(d);
            if (d < 0) inside += 1;
            if (d < min) {
              min = d;
              minAt = [flat[i], flat[i + 1], flat[i + 2]];
            }
          }
          return { distances, min, minAt, inside, count: distances.length };
        };

        const payload = { model_id: model.id };

        if (args.probes !== undefined) {
          let flat;
          try {
            flat = flattenProbes(args.probes);
          } catch (err) {
            return fail(err.message);
          }
          const r = against(flat);
          let softMin = null;
          try {
            softMin = model.shape.fieldClearance(flat, softness);
          } catch (err) {
            // A softmin failure must not sink the exact per-probe answer, which
            // is the number a caller actually wants here.
            payload.softMinError = errMessage(err);
          }
          payload.probes = {
            count: r.count,
            // Exact signed distances, one per probe, in input order.
            distances: r.distances,
            minDistance: r.min,
            nearestProbe: r.minAt,
            probesInsideMaterial: r.inside,
            clear: r.min > 0,
            softMin,
            softness,
          };
        }

        if (args.against_model_id !== undefined) {
          let other;
          try {
            other = store.get(args.against_model_id);
          } catch (err) {
            return fail(err.message);
          }
          // `store.get` just re-pointed the global exact toggle at `other`; mesh
          // it now, then put the toggle back before measuring against `model`.
          const mesh = getMesh(other.shape, { accuracy: accuracyArg(args.accuracy) });
          if (mesh.triangles === 0) {
            return fail(`model ${other.id} produced an empty mesh; nothing to test against`);
          }
          store.get(model.id);
          const r = against(mesh.positions);
          payload.interference = {
            against_model_id: other.id,
            sampledVertices: r.count,
            // Negative min = the other model's surface reaches inside this one.
            minDistance: r.min,
            deepestPoint: r.minAt,
            verticesInsideMaterial: r.inside,
            interferes: r.min < 0,
            // Positive: the gap between the two. Negative: how deep they overlap.
            clearance: r.min,
            note:
              'Sampled at the other model\'s mesh vertices, so an overlap narrower than its ' +
              'triangle spacing can be missed; pass a finer `accuracy` to tighten it. Only ' +
              "the other model's surface is sampled, so one solid entirely inside the other " +
              'with no surface contact reads as a deep interference, which it is.',
          };
        }

        return text(payload);
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
          'creation time). `source` is `script`, `step`, or `assembly`; `get_model` ' +
          'returns the source itself.',
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
          'to edit and re-submit to `create_model`), the STEP file it was imported ' +
          'from, or the assembly recipe (`assemble` instances, mates, and resolved ' +
          'transforms) that composed it. Also its params with their current values — after `optimize` those ' +
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
            : model.origin.kind === 'assembly'
            ? {
                // The recipe `assemble` was called with, plus the resolved
                // per-instance transforms — enough to re-assemble, the way a
                // script is enough to re-create.
                assembly: model.origin.assembly,
              }
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
          return failErr(err, 'optimize failed');
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
        return failErr(err);
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
