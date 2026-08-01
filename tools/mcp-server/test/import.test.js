// STEP import, end to end through the real kernel (of-2y4.7).
//
// The reader has been mature for a while — exact B-Rep mapping, mesh
// fallback, healing, per-entity diagnostics — and none of it was reachable
// from an agent. These tests hold the whole path open: bytes in, a model_id
// out, and the report that tells an agent whether the part it just ingested
// is the part it was given.
//
// The corpus is the server's own writer: export a solid whose volume is known
// analytically, import it back, and assert on the number. That makes the test
// self-contained (no fixture files) *and* covers the round trip an agent
// actually performs — hand a part to a colleague, get it back.

import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import { mkdtempSync, writeFileSync, readFileSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join, resolve, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';
import { createTools } from '../src/tools.js';

const here = dirname(fileURLToPath(import.meta.url));
const repoRoot = resolve(here, '../../..');

function freshTools() {
  return createTools({ outputDir: mkdtempSync(join(tmpdir(), 'osmcp-import-')) });
}

function jsonOf(result) {
  assert.equal(result.isError, undefined, `unexpected error: ${result.content?.[0]?.text}`);
  return JSON.parse(result.content[0].text);
}

/** Build a model, export it to STEP, and return the file's text and path. */
function exportedStep(t, script, { exact = true, name = 'part' } = {}) {
  const model = jsonOf(t.call('create_model', { script, name, exact }));
  const written = jsonOf(t.call('export', { model_id: model.model_id, format: 'step' }));
  return { path: written.path, text: readFileSync(written.path, 'utf8'), model };
}

describe('import_step', () => {
  // A 20x20x20 box is 8000 mm³ exactly, and a planar B-Rep tessellates
  // exactly — so this is an equality assertion in all but name. If the
  // importer silently dropped a face or mis-scaled a coordinate, no
  // tolerance loose enough to hide it would still pass.
  test('reads back a file this server wrote, with the volume intact', () => {
    const t = freshTools();
    const { path } = exportedStep(t, 'return Shape.box3(10, 10, 10);');

    const out = jsonOf(t.call('import_step', { path }));
    assert.match(out.model_id, /^model-\d+-[0-9a-f]{4}$/);
    assert.deepEqual(out.counts, { brep: 1, mesh: 0, failed: 0 });
    assert.equal(out.solids.length, 1);
    assert.equal(out.solids[0].outcome, 'brep');
    assert.equal(out.solids[0].exact, true);
    assert.equal(out.exact, true, 'a single-solid B-Rep import keeps its analytic surfaces');
    assert.ok(out.solids[0].model_id, 'each solid gets its own model_id');
    assert.equal(out.valid, true);
    assert.deepEqual(out.issues, []);
    assert.ok(Math.abs(out.volume - 8000) < 1e-6, `imported volume ${out.volume}`);
    assert.deepEqual(out.boundingBox.size, [20, 20, 20]);
    assert.equal(out.diagnostics.error, 0);
  });

  test('accepts the file contents inline as well as a path', () => {
    const t = freshTools();
    const { text } = exportedStep(t, 'return Shape.box3(5, 5, 5);');
    const out = jsonOf(t.call('import_step', { text, name: 'inline' }));
    assert.equal(out.name, 'inline');
    assert.equal(out.source.text, true);
    assert.ok(out.source.bytes > 100);
    assert.ok(Math.abs(out.volume - 1000) < 1e-6, `imported volume ${out.volume}`);
  });

  test('resolves a bare path against the output dir, so an export reads back', () => {
    const t = freshTools();
    const model = jsonOf(t.call('create_model', { script: 'return Shape.box3(2, 3, 4);', exact: true }));
    jsonOf(t.call('export', { model_id: model.model_id, format: 'step', path: 'part.step' }));
    const out = jsonOf(t.call('import_step', { path: 'part.step' }));
    assert.ok(Math.abs(out.volume - 4 * 6 * 8) < 1e-6, `imported volume ${out.volume}`);
  });

  // The whole point of importing: the part is then a first-class model.
  test('an imported model works with measure, validate, export and screenshots', () => {
    const t = freshTools();
    const { path } = exportedStep(t, 'return Shape.box3(10, 10, 10);');
    const id = jsonOf(t.call('import_step', { path })).model_id;

    const measured = jsonOf(t.call('measure', { model_id: id }));
    assert.ok(Math.abs(measured.volume - 8000) < 1e-6);
    assert.ok(Math.abs(measured.surfaceArea - 2400) < 1e-6);
    assert.equal(jsonOf(t.call('validate', { model_id: id })).valid, true);

    const shot = t.call('get_screenshot', { model_id: id, width: 120, height: 90 });
    assert.equal(shot.isError, undefined);
    assert.equal(shot.content[0].type, 'image');

    const written = jsonOf(t.call('export', { model_id: id, format: 'step' }));
    assert.ok(written.bytes > 0);
  });

  // An imported B-Rep keeps its analytic surfaces, so exporting it again must
  // not degrade it to recovered facets. The cheap, robust signal for that is
  // the re-imported outcome: a faceted export of a curved part comes back as
  // hundreds of planar faces, an analytic one as the same handful.
  test('an imported B-Rep re-exports analytically, not as facets', () => {
    const t = freshTools();
    const { path } = exportedStep(t, 'return Shape.cylinder(5, 10);');
    const first = jsonOf(t.call('import_step', { path }));
    assert.equal(first.solids[0].outcome, 'brep');

    const again = jsonOf(t.call('export', { model_id: first.model_id, format: 'step' }));
    const round = jsonOf(t.call('import_step', { path: again.path }));
    assert.equal(round.solids[0].outcome, 'brep', 'the second lap must still be exact');
    // A cylinder meshed at 32 segments: the two volumes agree because both
    // come from the same analytic surfaces, not because both were faceted the
    // same way.
    assert.ok(
      Math.abs(round.volume - first.volume) < 1e-6,
      `${round.volume} vs ${first.volume}`,
    );
  });

  test('circle_segments drives the fidelity of the measured mesh', () => {
    const t = freshTools();
    const { path } = exportedStep(t, 'return Shape.cylinder(5, 10);');
    const coarse = jsonOf(t.call('import_step', { path, circle_segments: 8 }));
    const fine = jsonOf(t.call('import_step', { path, circle_segments: 128 }));
    assert.ok(fine.mesh.triangles > coarse.mesh.triangles);
    // π·25·20 = 1570.8; an inscribed prism under-reads, and less so as the
    // segment count rises.
    assert.ok(fine.volume > coarse.volume, `${fine.volume} vs ${coarse.volume}`);
    assert.ok(Math.abs(fine.volume - 1570.796) < 1, `fine volume ${fine.volume}`);
  });

  test('reports the units the file declared, without rescaling the geometry', () => {
    const t = freshTools();
    const model = jsonOf(t.call('create_model', { script: 'return Shape.box3(10, 10, 10);', exact: true }));
    const written = jsonOf(t.call('export', { model_id: model.model_id, format: 'step', unit: 'in' }));
    const out = jsonOf(t.call('import_step', { path: written.path }));
    // An inch file: 25.4 mm per unit, and the reader scales the geometry to
    // millimetres, so a 20-unit box arrives 508 mm across.
    assert.ok(Math.abs(out.units.lengthScale - 25.4) < 1e-9, `scale ${out.units.lengthScale}`);
    assert.deepEqual(out.boundingBox.size.map(Math.round), [508, 508, 508]);
  });

  describe('failure reporting', () => {
    test('a file that is not STEP at all is a clean error', () => {
      const t = freshTools();
      const bad = t.call('import_step', { text: 'this is not a STEP file' });
      assert.equal(bad.isError, true);
      assert.match(bad.content[0].text, /import failed/i);
    });

    test('a missing file names the path rather than throwing', () => {
      const t = freshTools();
      const bad = t.call('import_step', { path: '/nonexistent/nowhere.step' });
      assert.equal(bad.isError, true);
      assert.match(bad.content[0].text, /cannot read STEP file/);
    });

    test('neither path nor text is an error that says what to pass', () => {
      const t = freshTools();
      const bad = t.call('import_step', {});
      assert.equal(bad.isError, true);
      assert.match(bad.content[0].text, /pass 'path'.*or 'text'/);
    });

    test('both path and text is an error rather than a silent preference', () => {
      const t = freshTools();
      const bad = t.call('import_step', { path: '/tmp/x.step', text: 'ISO-10303-21;' });
      assert.equal(bad.isError, true);
      assert.match(bad.content[0].text, /not both/);
    });

    // Valid Part 21 carrying no solid is not a crash and not a success: the
    // caller asked for a part and there is none. It must come back as an
    // error that still carries the diagnostics.
    test('a solid-free file is an error that explains itself', () => {
      const t = freshTools();
      const bad = t.call('import_step', {
        text: [
          'ISO-10303-21;',
          'HEADER;',
          "FILE_SCHEMA(('CONFIG_CONTROL_DESIGN'));",
          'ENDSEC;',
          'DATA;',
          "#1 = CARTESIAN_POINT('', (0., 0., 0.));",
          'ENDSEC;',
          'END-ISO-10303-21;',
        ].join('\n'),
      });
      assert.equal(bad.isError, true);
      const payload = JSON.parse(bad.content[0].text);
      assert.match(payload.error, /no solids/);
      assert.deepEqual(payload.counts, { brep: 0, mesh: 0, failed: 0 });
    });
  });

  // A real file from the kernel's corpus, not one this server wrote: the
  // writer and reader share conventions, and a round trip alone could hide a
  // shared misreading of what other CAD systems emit.
  test('imports a foreign file from the kernel corpus', () => {
    const t = freshTools();
    const path = resolve(repoRoot, 'crates/opensolid-kernel/tests/data/step/sg1-c5-214.stp');
    const out = jsonOf(t.call('import_step', { path }));
    assert.ok(out.solids.length > 0, 'the corpus file declares solids');
    assert.ok(
      out.counts.brep + out.counts.mesh > 0,
      `nothing imported: ${JSON.stringify(out.counts)}`,
    );
    assert.ok(out.volume > 0, `imported volume ${out.volume}`);
    // Diagnostics are reported in full by count even when the sample is cut.
    assert.equal(typeof out.diagnostics.error, 'number');
    if (out.diagnostics.truncated) {
      assert.ok(out.diagnostics.truncated.total > out.diagnostics.truncated.shown);
      assert.equal(out.diagnostics.items.length, out.diagnostics.truncated.shown);
    }
  });

  // AS1 is the canonical STEP assembly test file: five distinct parts placed
  // eighteen times. It covers what a single-part round trip cannot — a model
  // per solid, and placement, which is the difference between an assembly and
  // five parts piled on the origin.
  test('a multi-solid assembly yields a model per part and a placed whole', () => {
    const t = freshTools();
    const path = resolve(repoRoot, 'crates/opensolid-kernel/tests/data/step/as1-oc-214.stp');
    const out = jsonOf(t.call('import_step', { path }));

    assert.equal(out.solids.length, 5, 'AS1 declares five distinct solids');
    assert.equal(out.counts.failed, 0);
    assert.equal(out.assembly.isAssembly, true);
    assert.ok(out.assembly.instances > out.solids.length, 'parts are placed more than once');
    // The parts kept their analytic surfaces; the placed whole is a
    // composition, so it did not.
    assert.equal(out.exact, false, 'a placed assembly is not an analytic body');
    assert.ok(
      out.solids.every((s) => s.exact),
      'every AS1 part imported as an exact B-Rep',
    );

    // Every part is separately addressable and measurable on its own.
    const parts = out.solids.map((solid) => {
      assert.ok(solid.model_id, `solid ${solid.index} has no model_id`);
      const each = jsonOf(t.call('measure', { model_id: solid.model_id, query: 'volume' }));
      assert.ok(each.volume > 0, `solid ${solid.index} measured ${each.volume}`);
      return each.volume;
    });

    // The placed whole holds more material than any single part, because the
    // occurrences are real copies in space rather than one part at the origin.
    const largestPart = Math.max(...parts);
    assert.ok(out.volume > largestPart, `assembly ${out.volume} vs largest part ${largestPart}`);
    assert.equal(out.valid, true, `assembled mesh is not a closed manifold: ${out.issues}`);
  });

  // The placed assembly is measured from the parts' own triangles rather
  // than by re-meshing a union of eighteen mesh SDFs. That is not a
  // micro-optimization: the field route measured 134 s on this file, which
  // is past the point where an agent's client gives up.
  test('an assembly imports in seconds, not minutes', { timeout: 30_000 }, () => {
    const t = freshTools();
    const path = resolve(repoRoot, 'crates/opensolid-kernel/tests/data/step/as1-oc-214.stp');
    const started = Date.now();
    const out = jsonOf(t.call('import_step', { path }));
    const elapsed = Date.now() - started;
    assert.ok(out.mesh.triangles > 1000, 'precondition: a real assembly, not a stub');
    assert.ok(elapsed < 20_000, `import_step took ${elapsed} ms`);
  });

  // A body the reader maps exactly can still hit a gap in the *tessellator*.
  // That is not a crash and not a silent empty model: the solid comes back
  // named, with the outcome it achieved and the reason it has no shape.
  //
  // This used to be fenced on io1-cm-214, whose torus fillets the
  // tessellator refused; of-6fcu closed that gap, so the contract is held
  // open here on a part that still hits one — a NIST file whose only solid
  // is bounded by a cylindrical face carrying no seam edge, so its boundary
  // wraps a full period and no parameter ring can bound it.
  test('a solid the tessellator cannot handle says so per solid', () => {
    const t = freshTools();
    const path = resolve(repoRoot, 'crates/opensolid-kernel/tests/data/step/nist/nist_ftc_11_asme1_rb.stp');
    const out = t.call('import_step', { path });
    assert.equal(out.isError, true);
    const payload = JSON.parse(out.content[0].text);
    assert.match(payload.error, /usable shape/);
    assert.equal(payload.counts.brep, 1, 'the reader did import it as an exact B-Rep');
    assert.match(payload.solids[0].shapeError, /tessellat/i);
  });

  // The other half of of-6fcu: the two corpus files that read as exact
  // B-Reps with no diagnostics at all and still could not be ingested,
  // because every solid in them failed to mesh. An agent handed either one
  // got `isError` and no model; both must now come back measurable.
  //
  // io1-cm is one solid of planes, cylinders and two torus fillets;
  // dm1-id is three solids of ruled spline patches closed in `u`. The
  // volume figures are gated against OpenCASCADE in `occ_reference.rs`; what
  // matters here is that the path an agent walks produces a valid model.
  test('parts that only the tessellator was blocking now import', () => {
    for (const [file, solids] of [
      ['io1-cm-214.stp', 1],
      ['dm1-id-214.stp', 3],
    ]) {
      const t = freshTools();
      const path = resolve(repoRoot, `crates/opensolid-kernel/tests/data/step/${file}`);
      const out = jsonOf(t.call('import_step', { path }));
      assert.equal(out.counts.brep, solids, `${file}: every solid must map exactly`);
      assert.equal(out.counts.failed, 0, `${file}: no solid may fail`);
      assert.ok(out.model_id, `${file}: the whole file must register as a model`);
      assert.equal(out.valid, true, `${file}: mesh is not a closed manifold: ${out.issues}`);
      assert.ok(out.volume > 0, `${file}: volume ${out.volume}`);
      for (const solid of out.solids) {
        assert.equal(solid.shapeError, undefined, `${file}: solid ${solid.index} has no shape`);
        assert.ok(solid.model_id, `${file}: solid ${solid.index} did not register`);
      }
    }
  });
});

describe('get_model', () => {
  test('hands back the script a model was built from, verbatim', () => {
    const t = freshTools();
    const script = "const t = param('thickness', 4, {min: 2, max: 8});\nreturn Shape.box3(t, 10, 10);";
    const id = jsonOf(t.call('create_model', { script, name: 'plate' })).model_id;

    const out = jsonOf(t.call('get_model', { model_id: id }));
    assert.equal(out.model_id, id);
    assert.equal(out.name, 'plate');
    assert.equal(out.source, 'script');
    assert.equal(out.script, script, 'the script must come back byte-for-byte');
    assert.deepEqual(out.params, [{ name: 'thickness', value: 4, default: 4, min: 2, max: 8 }]);
  });

  // The recovery this exists for: after `optimize` moves a param, the script
  // plus the reported values are the whole design. Without it the converged
  // numbers live only in the optimize response.
  test("reports a param's current value after optimize moved it", () => {
    const t = freshTools();
    const id = jsonOf(
      t.call('create_model', {
        script: "const r = param('radius', 5, {min: 2, max: 12});\nreturn Shape.sphere(r);",
      }),
    ).model_id;
    const run = jsonOf(
      t.call('optimize', {
        model_id: id,
        params: [{ name: 'radius' }],
        objective: { type: 'target_volume', value: 1000 },
        options: { max_iters: 8, resolution: 16 },
      }),
    );
    const out = jsonOf(t.call('get_model', { model_id: id }));
    assert.equal(out.params[0].default, 5, 'the declaration is unchanged');
    assert.equal(out.params[0].value, run.params.radius, 'the current value is the converged one');
  });

  test('reports where an imported model came from', () => {
    const t = freshTools();
    const { path } = exportedStep(t, 'return Shape.box3(10, 10, 10);');
    const imported = jsonOf(t.call('import_step', { path }));

    const whole = jsonOf(t.call('get_model', { model_id: imported.model_id }));
    assert.equal(whole.source, 'step');
    assert.equal(whole.script, undefined, 'an import has no script to hand back');
    assert.equal(whole.imported.path, path);
    assert.ok(whole.imported.bytes > 0);
    assert.deepEqual(whole.params, []);

    const solid = jsonOf(t.call('get_model', { model_id: imported.solids[0].model_id }));
    assert.equal(solid.imported.solidIndex, 0);
    assert.equal(solid.imported.outcome, 'brep');
    assert.equal(solid.imported.stepId, imported.solids[0].step_id);
  });

  test('optimize says why an imported model cannot be optimized', () => {
    const t = freshTools();
    const { path } = exportedStep(t, 'return Shape.box3(10, 10, 10);');
    const id = jsonOf(t.call('import_step', { path })).model_id;
    const bad = t.call('optimize', {
      model_id: id,
      params: [{ name: 'thickness', min: 1, max: 5 }],
      objective: { type: 'target_volume', value: 100 },
    });
    assert.equal(bad.isError, true);
    assert.match(bad.content[0].text, /imported from a file, not built from a script/);
  });

  test('an unknown model_id is a clean error', () => {
    const t = freshTools();
    const bad = t.call('get_model', { model_id: 'model-404-dead' });
    assert.equal(bad.isError, true);
    assert.match(bad.content[0].text, /unknown model_id/);
  });
});

describe('list_models', () => {
  test('says which models still have a source to recover', () => {
    const t = freshTools();
    jsonOf(t.call('create_model', { script: 'return Shape.sphere(1);', name: 'ball' }));
    const dir = mkdtempSync(join(tmpdir(), 'osmcp-step-'));
    const stepPath = join(dir, 'cube.step');
    const built = jsonOf(t.call('create_model', { script: 'return Shape.box3(1,1,1);', exact: true }));
    writeFileSync(
      stepPath,
      readFileSync(jsonOf(t.call('export', { model_id: built.model_id, format: 'step' })).path),
    );
    jsonOf(t.call('import_step', { path: stepPath, name: 'cube' }));

    const models = jsonOf(t.call('list_models')).models;
    const byName = Object.fromEntries(models.map((m) => [m.name, m]));
    assert.equal(byName.ball.source, 'script');
    assert.equal(byName.cube.source, 'step');
  });
});
