// Tests for the gradient-based `optimize` tool (of-2y4.2), exercising the real
// wasm kernel through the MCP tool handlers. Requires the built pkg
// (`npm run build`).

import { test } from 'node:test';
import assert from 'node:assert/strict';
import { mkdtempSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { createTools } from '../src/tools.js';

function freshTools() {
  return createTools({ outputDir: mkdtempSync(join(tmpdir(), 'osopt-')) });
}

function jsonOf(result) {
  assert.equal(result.isError, undefined, `unexpected error: ${result.content?.[0]?.text}`);
  return JSON.parse(result.content[0].text);
}

function makeModel(t, script, name) {
  return jsonOf(t.call('create_model', { script, name })).model_id;
}

// A radius-parameterised sphere: volume 4/3·π·r³ is a clean analytic target.
const SPHERE = "const r = param('r', 1.0, {min: 0.4, max: 3}); return Shape.sphere(r);";

test('create_model surfaces declared params, and none when there are none', () => {
  const t = freshTools();
  const withParam = jsonOf(t.call('create_model', { script: SPHERE }));
  assert.deepEqual(withParam.params, [{ name: 'r', value: 1, min: 0.4, max: 3 }]);
  const without = jsonOf(t.call('create_model', { script: 'return Shape.sphere(1);' }));
  assert.equal(without.params, undefined);
});

test('optimize drives an exact volume onto target and writes the params back', () => {
  const t = freshTools();
  const id = makeModel(t, SPHERE);
  const r = jsonOf(
    t.call('optimize', {
      model_id: id,
      params: [{ name: 'r' }],
      objective: { type: 'target_volume', value: 20 },
      options: { max_iters: 50, resolution: 28 },
    }),
  );
  assert.equal(r.converged, true, r.warnings.join('; '));
  assert.equal(r.feasible, true);
  // Exact mesh volume within 0.5% of the target — the calibration corrects the
  // field bias, so the reported (exact) volume actually lands on target.
  assert.ok(Math.abs(r.objective.relativeError) < 0.005, `relErr ${r.objective.relativeError}`);
  const analytic = Math.cbrt(20 / ((4 / 3) * Math.PI));
  assert.ok(Math.abs(r.params.r - analytic) < 0.02, `r ${r.params.r} vs ${analytic}`);

  // The winning radius is committed: a subsequent measure sees the optimized part.
  const measured = jsonOf(t.call('measure', { model_id: id, query: 'volume' }));
  assert.ok(Math.abs(measured.volume - 20) < 0.2, `measured ${measured.volume}`);
});

test('optimize hits a mass target given a density', () => {
  const t = freshTools();
  // box3 uses half-extents: box3(w,10,10) is 2w×20×20. At w=10, vol 8000; target
  // 15 g at 0.0027 g/mm³ needs vol 5555 → w ≈ 6.94, an interior optimum.
  const id = makeModel(t, "const w = param('w', 10, {min: 5, max: 40}); return Shape.box3(w, 10, 10);");
  const r = jsonOf(
    t.call('optimize', {
      model_id: id,
      params: [{ name: 'w' }],
      objective: { type: 'target_mass', value: 15, density: 0.0027 },
      options: { max_iters: 60, resolution: 32 },
    }),
  );
  assert.equal(r.converged, true, r.warnings.join('; '));
  assert.ok(Math.abs(r.objective.achieved - 15) < 0.15, `mass ${r.objective.achieved}`);
  assert.equal(r.objective.density, 0.0027);
});

test('optimize drives the centroid to a target point', () => {
  const t = freshTools();
  const id = makeModel(t, "const x = param('x', 0, {min: -3, max: 5}); return Shape.box3(1,1,1).translate(x, 0, 0);");
  const r = jsonOf(
    t.call('optimize', {
      model_id: id,
      params: [{ name: 'x' }],
      objective: { type: 'centroid_at', value: [2.0, null, null] },
      options: { max_iters: 50, resolution: 28 },
    }),
  );
  assert.equal(r.converged, true, r.warnings.join('; '));
  assert.ok(Math.abs(r.params.x - 2.0) < 0.02, `x ${r.params.x}`);
  assert.ok(Math.abs(r.objective.achieved[0] - 2.0) < 0.05, `cx ${r.objective.achieved[0]}`);
});

test('a satisfiable clearance constraint is met', () => {
  const t = freshTools();
  // Target volume 20 (r≈1.68) with a keep-out probe 2 units out, min clearance
  // 0.2 → needs r ≤ 1.8: compatible.
  const id = makeModel(t, "const r = param('r', 1.0, {min: 0.4, max: 3}); return Shape.sphere(r);");
  const r = jsonOf(
    t.call('optimize', {
      model_id: id,
      params: [{ name: 'r' }],
      objective: { type: 'target_volume', value: 20 },
      constraints: [{ type: 'clearance', probes: [[2, 0, 0]], min: 0.2 }],
      options: { max_iters: 70, resolution: 32 },
    }),
  );
  assert.equal(r.feasible, true, r.warnings.join('; '));
  assert.equal(r.constraints[0].satisfied, true);
  assert.ok(r.constraints[0].value >= 0.2 - 1e-6, `clearance ${r.constraints[0].value}`);
});

test('an unsatisfiable constraint reports feasible:false, not a converged solution', () => {
  const t = freshTools();
  // Volume 40 wants r≈2.12, but clearance to a probe 2 out at min 0.3 forbids
  // r > 1.7: a genuine conflict the soft penalty cannot resolve.
  const id = makeModel(t, "const r = param('r', 1.5, {min: 0.4, max: 3}); return Shape.sphere(r);");
  const r = jsonOf(
    t.call('optimize', {
      model_id: id,
      params: [{ name: 'r' }],
      objective: { type: 'target_volume', value: 40 },
      constraints: [{ type: 'clearance', probes: [[2, 0, 0]], min: 0.3 }],
      options: { max_iters: 80, resolution: 32 },
    }),
  );
  assert.equal(r.feasible, false);
  assert.equal(r.converged, false);
  assert.equal(r.constraints[0].satisfied, false);
  assert.ok(
    r.warnings.some((w) => /not satisfied/i.test(w)),
    `expected a constraint warning: ${JSON.stringify(r.warnings)}`,
  );
});

test('a parameter driven past its bound is pinned and reported', () => {
  const t = freshTools();
  // Target volume 500 needs r≈4.9, well past the max bound of 2.
  const id = makeModel(t, "const r = param('r', 1.0, {min: 0.4, max: 2}); return Shape.sphere(r);");
  const r = jsonOf(
    t.call('optimize', {
      model_id: id,
      params: [{ name: 'r' }],
      objective: { type: 'target_volume', value: 500 },
      options: { max_iters: 40, resolution: 24 },
    }),
  );
  assert.ok(Math.abs(r.params.r - 2) < 1e-3, `r pinned near 2, got ${r.params.r}`);
  assert.deepEqual(r.pinned, [{ name: 'r', value: r.params.r, at: 'max' }]);
  assert.equal(r.converged, false);
  assert.ok(r.warnings.some((w) => /pinned/i.test(w)));
});

test('optimize covers a script containing rotate (which the AD tower does not)', () => {
  const t = freshTools();
  // A rotated box: the whole point of the finite-difference-over-WASM path is
  // that it re-evaluates arbitrary scripts, rotate included.
  const id = makeModel(
    t,
    "const w = param('w', 6, {min: 3, max: 15}); return Shape.box3(w, 5, 4).rotate(0, 0, 1, 0.4);",
  );
  const r = jsonOf(
    t.call('optimize', {
      model_id: id,
      params: [{ name: 'w' }],
      objective: { type: 'target_volume', value: 3000 },
      options: { max_iters: 50, resolution: 30 },
    }),
  );
  // box3(w,5,4) is 2w×10×8 = 160w; target 3000 → w ≈ 18.75, capped at 15, so it
  // pins — the point is that it evaluated and descended a rotate script at all.
  assert.ok(r.iterations >= 1);
  assert.equal(typeof r.objective.achieved, 'number');
});

test('the trajectory logs monotone-ish loss and exact (not field) measures', () => {
  const t = freshTools();
  const id = makeModel(t, SPHERE);
  const r = jsonOf(
    t.call('optimize', {
      model_id: id,
      params: [{ name: 'r' }],
      objective: { type: 'target_volume', value: 20 },
      options: { max_iters: 50, resolution: 28 },
    }),
  );
  assert.ok(r.trajectory.length >= 2);
  assert.equal(r.trajectory[0].iter, 0);
  // The last trajectory volume matches the reported exact volume, confirming the
  // log is in exact units, not the biased field.
  const last = r.trajectory[r.trajectory.length - 1];
  assert.ok(Math.abs(last.volume - r.exactMeasure.volume) < 1e-6, `${last.volume} vs ${r.exactMeasure.volume}`);
  // Loss falls overall.
  assert.ok(last.loss < r.trajectory[0].loss);
  // Domain is a plain array, not a typed-array object.
  assert.ok(Array.isArray(r.field.domain.min) && r.field.domain.min.length === 3);
});

// ── Guardrails ─────────────────────────────────────────────────────────────

test('optimize rejects a param the script never declared', () => {
  const t = freshTools();
  const id = makeModel(t, SPHERE);
  const bad = t.call('optimize', {
    model_id: id,
    params: [{ name: 'nope' }],
    objective: { type: 'target_volume', value: 5 },
  });
  assert.equal(bad.isError, true);
  assert.match(bad.content[0].text, /not declared|param 'nope'/);
});

test('optimize requires bounds when neither request nor declaration carries them', () => {
  const t = freshTools();
  // Declared without bounds; request omits them too.
  const id = makeModel(t, "const r = param('r', 1.0); return Shape.sphere(r);");
  const bad = t.call('optimize', {
    model_id: id,
    params: [{ name: 'r' }],
    objective: { type: 'target_volume', value: 5 },
  });
  assert.equal(bad.isError, true);
  assert.match(bad.content[0].text, /finite min and max/);
});

test('target_mass without a density is rejected with a clear message', () => {
  const t = freshTools();
  const id = makeModel(t, SPHERE);
  const bad = t.call('optimize', {
    model_id: id,
    params: [{ name: 'r' }],
    objective: { type: 'target_mass', value: 5 },
  });
  assert.equal(bad.isError, true);
  assert.match(bad.content[0].text, /density/);
});

test('optimize rejects an empty params array', () => {
  const t = freshTools();
  const id = makeModel(t, SPHERE);
  const bad = t.call('optimize', { model_id: id, params: [], objective: { type: 'target_volume', value: 5 } });
  assert.equal(bad.isError, true);
  assert.match(bad.content[0].text, /non-empty `params`/);
});

test('optimize rejects an unknown objective type', () => {
  const t = freshTools();
  const id = makeModel(t, SPHERE);
  const bad = t.call('optimize', {
    model_id: id,
    params: [{ name: 'r' }],
    objective: { type: 'target_diameter', value: 5 },
  });
  assert.equal(bad.isError, true);
  assert.match(bad.content[0].text, /unknown objective/);
});

test('optimize honours the iteration cap as a guardrail', () => {
  const t = freshTools();
  const id = makeModel(t, SPHERE);
  const r = jsonOf(
    t.call('optimize', {
      model_id: id,
      params: [{ name: 'r' }],
      objective: { type: 'target_volume', value: 200 },
      options: { max_iters: 3, resolution: 20 },
    }),
  );
  assert.ok(r.iterations <= 3, `iterations ${r.iterations}`);
});
