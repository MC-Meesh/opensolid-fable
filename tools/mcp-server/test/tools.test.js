// End-to-end tests for the MCP tool handlers, exercising the real wasm
// kernel. Requires the built pkg (`npm run build`).

import { test } from 'node:test';
import assert from 'node:assert/strict';
import { mkdtempSync, existsSync, readFileSync, statSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { createTools } from '../src/tools.js';

function freshTools() {
  return createTools({ outputDir: mkdtempSync(join(tmpdir(), 'osmcp-')) });
}

function jsonOf(result) {
  assert.equal(result.isError, undefined, `unexpected error: ${result.content?.[0]?.text}`);
  return JSON.parse(result.content[0].text);
}

test('create_model registers a model and reports stats', () => {
  const t = freshTools();
  const out = jsonOf(t.call('create_model', { script: 'return Shape.box3(1, 0.5, 0.75);', name: 'block' }));
  assert.match(out.model_id, /^model-\d+-[0-9a-f]{4}$/);
  assert.equal(out.name, 'block');
  assert.ok(out.mesh.triangles > 0);
  assert.equal(out.valid, true);
  assert.deepEqual(out.boundingBox.size, [2, 1, 1.5]);
});

test('create_model rejects scripts that do not return a Shape', () => {
  const t = freshTools();
  const bad = t.call('create_model', { script: 'return 42;' });
  assert.equal(bad.isError, true);
  assert.match(bad.content[0].text, /must return a Shape/);
});

test('create_model surfaces syntax errors', () => {
  const t = freshTools();
  const bad = t.call('create_model', { script: 'return Shape.sphere(' });
  assert.equal(bad.isError, true);
  assert.match(bad.content[0].text, /syntax error|failed/i);
});

test('measure returns the exact box volume', () => {
  const t = freshTools();
  const id = jsonOf(t.call('create_model', { script: 'return Shape.box3(1, 0.5, 0.75);' })).model_id;
  const full = jsonOf(t.call('measure', { model_id: id }));
  assert.ok(Math.abs(full.volume - 3) < 0.05, `volume ${full.volume}`);
  assert.ok(Math.abs(full.centroid[0]) < 1e-3);

  const volumeOnly = jsonOf(t.call('measure', { model_id: id, query: 'volume' }));
  assert.deepEqual(Object.keys(volumeOnly).sort(), ['exact', 'volume']);
});

test('validate reports a watertight boolean result as valid', () => {
  const t = freshTools();
  const id = jsonOf(
    t.call('create_model', {
      script: 'return Shape.box3(1,1,1).subtract(Shape.cylinder(0.4, 2));',
    }),
  ).model_id;
  const report = jsonOf(t.call('validate', { model_id: id }));
  assert.equal(report.valid, true);
  assert.equal(report.closedManifold, true);
  assert.deepEqual(report.issues, []);
});

test('get_screenshot returns a valid PNG image', () => {
  const t = freshTools();
  const id = jsonOf(t.call('create_model', { script: 'return Shape.sphere(1);' })).model_id;
  const shot = t.call('get_screenshot', { model_id: id, view: 'front', width: 64, height: 64 });
  assert.equal(shot.isError, undefined);
  const img = shot.content[0];
  assert.equal(img.type, 'image');
  assert.equal(img.mimeType, 'image/png');
  const bytes = Buffer.from(img.data, 'base64');
  assert.equal(bytes.subarray(0, 8).toString('hex'), '89504e470d0a1a0a');
});

test('export writes step, stl, and obj files', () => {
  const t = freshTools();
  const id = jsonOf(t.call('create_model', { script: 'return Shape.box3(1,1,1);', name: 'cube' })).model_id;
  for (const format of ['step', 'stl', 'obj']) {
    const out = jsonOf(t.call('export', { model_id: id, format }));
    assert.equal(out.format, format);
    assert.ok(existsSync(out.path), `${format} file exists`);
    assert.equal(statSync(out.path).size, out.bytes);
    assert.ok(out.bytes > 0);
  }
  // STEP is a Part 21 file.
  const step = jsonOf(t.call('export', { model_id: id, format: 'step', path: 'cube2.step' }));
  assert.match(readFileSync(step.path, 'utf8'), /ISO-10303-21/);
});

test('unknown model_id and unknown tool return errors, not throws', () => {
  const t = freshTools();
  assert.equal(t.call('measure', { model_id: 'missing' }).isError, true);
  assert.equal(t.call('nope', {}).isError, true);
});

test('export surfaces the kernel error message, not "undefined"', () => {
  // wasm-bindgen rejects a Rust Result::Err(String) by throwing the raw string
  // (not an Error), so `err.message` is undefined. A thin toothed disk whose
  // teeth reach the bounding box makes the faceted STEP path decline — the
  // handler must report the kernel's reason, never a bare "undefined".
  const t = freshTools();
  const id = jsonOf(
    t.call('create_model', {
      script:
        'let g = Shape.cylinder(16, 4);' +
        'const tooth = Shape.box3(3, 2.2, 4).translate(17.5, 0, 0);' +
        'for (let i = 0; i < 16; i++) g = g.union(tooth.rotate(0, 0, 1, (360 * i) / 16));' +
        'return g.subtract(Shape.cylinder(4, 6));',
      name: 'gear',
    }),
  ).model_id;
  const bad = t.call('export', { model_id: id, format: 'step' });
  assert.equal(bad.isError, true);
  assert.doesNotMatch(bad.content[0].text, /undefined/);
  assert.match(bad.content[0].text, /export failed: .*meshing/i);
  // STL of the same model still works — different code path.
  assert.equal(jsonOf(t.call('export', { model_id: id, format: 'stl' })).format, 'stl');
});

test('exact booleans flag is honored per model', () => {
  const t = freshTools();
  const exact = jsonOf(
    t.call('create_model', {
      script: 'return Shape.box3(1,1,1).subtract(Shape.box3(0.5,0.5,2));',
      exact: true,
    }),
  );
  assert.equal(exact.exact, true);
  // Exact box-minus-box is a clean solid.
  assert.equal(exact.valid, true);
});
