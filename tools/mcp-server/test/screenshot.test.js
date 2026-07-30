// The visual inspection channel at the MCP surface (of-2y4.6): framing,
// section views and edge line-work driven through `get_screenshot` against the
// real wasm kernel. The renderer's own maths is covered in render.test.js.

import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import { mkdtempSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { createTools } from '../src/tools.js';
import { decodePng } from '../src/png.js';

function freshTools() {
  return createTools({ outputDir: mkdtempSync(join(tmpdir(), 'osmcp-shot-')) });
}

function jsonOf(result) {
  assert.equal(result.isError, undefined, `unexpected error: ${result.content?.[0]?.text}`);
  return JSON.parse(result.content[0].text);
}

/** Take a shot and unpack it into `{ png, meta }`. */
function shoot(tools, args) {
  const out = tools.call('get_screenshot', { width: 200, height: 150, ...args });
  assert.equal(out.isError, undefined, `unexpected error: ${out.content?.[0]?.text}`);
  assert.equal(out.content[0].type, 'image');
  assert.equal(out.content[0].mimeType, 'image/png');
  assert.equal(out.content[1].type, 'text');
  return {
    png: Buffer.from(out.content[0].data, 'base64'),
    meta: JSON.parse(out.content[1].text),
  };
}

function countPixels(png, predicate) {
  const { width, height, rgba } = decodePng(png);
  let n = 0;
  for (let i = 0; i < width * height; i++) {
    if (predicate(rgba[i * 4], rgba[i * 4 + 1], rgba[i * 4 + 2])) n += 1;
  }
  return n;
}

/** The RGB triple at a pixel of the decoded framebuffer. */
function pixelAt(png, x, y) {
  const { width, rgba } = decodePng(png);
  const p = (y * width + x) * 4;
  return [rgba[p], rgba[p + 1], rgba[p + 2]];
}

const isAmber = (r, g, b) => r > b + 30 && r > 80;
const isInk = (r, g, b) => r < 60 && g < 60 && b < 60;
const isPaper = (r, g, b) => r > 200 && g > 200 && b > 200;
const isSteelBlue = (r, g, b) => b > r && b > g && b > 60;
const isBackdrop = (r, g, b) => r < 40 && g < 40 && b < 50;

/** A plate with a through-hole on Y — the part every framing claim is made about. */
const PLATE = 'return Shape.box3(20, 4, 10).subtract(Shape.cylinder(3, 20));';

function plateModel(tools) {
  return jsonOf(tools.call('create_model', { script: PLATE, name: 'plate' })).model_id;
}

describe('the shot carries the camera that produced it', () => {
  test('image first, then a machine-readable camera', () => {
    const t = freshTools();
    const id = plateModel(t);
    const { png, meta } = shoot(t, { model_id: id, view: 'front' });
    assert.equal(png.subarray(0, 8).toString('hex'), '89504e470d0a1a0a');
    assert.deepEqual([decodePng(png).width, decodePng(png).height], [200, 150]);
    assert.equal(meta.model_id, id);
    assert.ok(meta.accuracy > 0, 'the mesh accuracy is reported');
    assert.equal(meta.camera.view, 'front');
    assert.equal(meta.camera.mode, 'shaded');
    assert.equal(meta.camera.section, null);
    assert.deepEqual([meta.camera.width, meta.camera.height], [200, 150]);
    assert.ok(meta.camera.scale > 0);
    assert.equal(meta.camera.visibleExtent.length, 2);
  });

  test('a custom direction is reported normalized, with no named view', () => {
    const t = freshTools();
    const { meta } = shoot(t, { model_id: plateModel(t), direction: [0, -2, 0] });
    assert.equal(meta.camera.view, null);
    assert.deepEqual(meta.camera.direction, [0, -1, 0]);
  });
});

describe('framing', () => {
  test('a region from a measured bounding box zooms to that box', () => {
    const t = freshTools();
    const id = plateModel(t);
    const { boundingBox } = jsonOf(t.call('measure', { model_id: id, query: 'bbox' }));
    const whole = shoot(t, { model_id: id, view: 'top' });
    // The bore, as an agent would name it: the middle third of the plate.
    const bore = {
      min: [-4, boundingBox.min[1], -4],
      max: [4, boundingBox.max[1], 4],
    };
    const zoomed = shoot(t, { model_id: id, view: 'top', region: bore });
    assert.ok(zoomed.meta.camera.scale > whole.meta.camera.scale * 2, 'framing tightened');
    assert.deepEqual(zoomed.meta.camera.target, [0, 0, 0]);
    assert.ok(!zoomed.png.equals(whole.png));
  });

  test('axis-on, a through-hole and a blind hole are different pixels', () => {
    // The guide tells an agent to read blind-vs-through off exactly this shot,
    // so the distinction is a checked claim, not a stylistic one: looking down
    // the bore, a hole that goes through shows the backdrop and one that stops
    // short shows its lit floor. It is also the reason that row says `shaded` —
    // in an edge mode both holes are the same circle of ink.
    const t = freshTools();
    const bore = { min: [4, -12, -8], max: [20, 12, 8] };
    const through = jsonOf(
      t.call('create_model', {
        script: 'return Shape.box3(40,20,30).subtract(Shape.cylinder(4,60).translate(12,0,0));',
      }),
    ).model_id;
    const blind = jsonOf(
      t.call('create_model', {
        script: 'return Shape.box3(40,20,30).subtract(Shape.cylinder(4,16).translate(12,6,0));',
      }),
    ).model_id;
    const a = shoot(t, { model_id: through, view: 'top', region: bore });
    const b = shoot(t, { model_id: blind, view: 'top', region: bore });
    const centre = [100, 75]; // the frame centre, which the region puts on the bore
    assert.ok(isBackdrop(...pixelAt(a.png, ...centre)), 'through-hole shows the backdrop');
    assert.ok(isSteelBlue(...pixelAt(b.png, ...centre)), 'blind hole shows its lit floor');
  });

  test('zoom and target are reported back exactly as applied', () => {
    const t = freshTools();
    const id = plateModel(t);
    const base = shoot(t, { model_id: id, view: 'top' });
    const close = shoot(t, { model_id: id, view: 'top', zoom: 2.5, target: [8, 0, 0] });
    assert.equal(close.meta.camera.zoom, 2.5);
    assert.deepEqual(close.meta.camera.target, [8, 0, 0]);
    assert.equal(close.meta.camera.scale, base.meta.camera.scale * 2.5);
  });
});

describe('section views', () => {
  test('a section exposes the bore and shades the cut face', () => {
    const t = freshTools();
    const id = plateModel(t);
    const plain = shoot(t, { model_id: id, view: 'front' });
    const cut = shoot(t, { model_id: id, view: 'front', section: { axis: 'Z' } });
    assert.equal(countPixels(plain.png, isAmber), 0, 'no cut face without a section');
    assert.ok(countPixels(cut.png, isAmber) > 500, 'the cut face is visible');
    assert.deepEqual(cut.meta.camera.section, { axis: 'Z', offset: 0, flip: false });
  });

  test('flip keeps the other half, and an explicit offset moves the plane', () => {
    const t = freshTools();
    const id = plateModel(t);
    const near = shoot(t, { model_id: id, view: 'iso', section: { axis: 'X' } });
    const far = shoot(t, { model_id: id, view: 'iso', section: { axis: 'X', flip: true } });
    const offset = shoot(t, { model_id: id, view: 'iso', section: { axis: 'X', offset: 6 } });
    assert.ok(!near.png.equals(far.png));
    assert.ok(!near.png.equals(offset.png));
    assert.equal(offset.meta.camera.section.offset, 6);
    assert.equal(far.meta.camera.section.flip, true);
  });
});

describe('edge modes', () => {
  test('edges mode draws line-work on a light ground', () => {
    const t = freshTools();
    const { png, meta } = shoot(t, { model_id: plateModel(t), view: 'iso', mode: 'edges' });
    assert.equal(meta.camera.mode, 'edges');
    assert.equal(meta.camera.lineWidth, 2);
    assert.ok(countPixels(png, isPaper) > 5000, 'light ground');
    assert.ok(countPixels(png, isInk) > 200, 'ink present');
  });

  test('a sphere still gets an outline, which only silhouette edges can give it', () => {
    // A tessellated sphere has no crease edges at all — every facet seam is
    // shallower than the feature-edge threshold. If the silhouette were not
    // wired up, this would render as a blank sheet of paper.
    const t = freshTools();
    const id = jsonOf(t.call('create_model', { script: 'return Shape.sphere(5);' })).model_id;
    const { png } = shoot(t, { model_id: id, view: 'front', mode: 'edges' });
    assert.ok(countPixels(png, isInk) > 100, 'the sphere has an outline');
  });

  test('shaded_edges keeps the shaded solid', () => {
    const t = freshTools();
    const id = plateModel(t);
    const shaded = shoot(t, { model_id: id, view: 'iso' });
    const overlaid = shoot(t, { model_id: id, view: 'iso', mode: 'shaded_edges' });
    assert.equal(overlaid.meta.camera.lineWidth, 1);
    assert.equal(countPixels(overlaid.png, isPaper), 0, 'still on the dark ground');
    assert.ok(countPixels(overlaid.png, isInk) > countPixels(shaded.png, isInk));
  });
});

describe('determinism', () => {
  const shots = [
    { view: 'iso' },
    { view: 'front', mode: 'edges' },
    { view: 'top', mode: 'shaded_edges', section: { axis: 'Z', offset: 1 } },
    { direction: [-1, -3, -2], zoom: 1.5, target: [2, 0, 0] },
    { region: { min: [-4, -2, -4], max: [4, 2, 4] }, view: 'top', accuracy: 0.05 },
  ];

  test('the same request against the same model is byte-identical', () => {
    const t = freshTools();
    const id = plateModel(t);
    for (const shot of shots) {
      const a = shoot(t, { model_id: id, ...shot });
      const b = shoot(t, { model_id: id, ...shot });
      assert.ok(a.png.equals(b.png), `unstable render for ${JSON.stringify(shot)}`);
      assert.deepEqual(a.meta, b.meta);
    }
  });

  test('the same request against a rebuilt model is byte-identical too', () => {
    // Determinism has to survive the model being built again from source —
    // otherwise a shot can only be compared with itself, never with the shot
    // from the previous session that showed the part before the change.
    const first = freshTools();
    const second = freshTools();
    const a = shoot(first, { model_id: plateModel(first), view: 'iso', mode: 'shaded_edges' });
    const b = shoot(second, { model_id: plateModel(second), view: 'iso', mode: 'shaded_edges' });
    assert.ok(a.png.equals(b.png));
    assert.equal(a.meta.accuracy, b.meta.accuracy);
  });

  test('accuracy pins the tessellation the shot is taken of', () => {
    const t = freshTools();
    const id = jsonOf(t.call('create_model', { script: 'return Shape.sphere(5);' })).model_id;
    const coarse = shoot(t, { model_id: id, view: 'iso', accuracy: 0.4 });
    const fine = shoot(t, { model_id: id, view: 'iso', accuracy: 0.02 });
    assert.equal(coarse.meta.accuracy, 0.4);
    assert.equal(fine.meta.accuracy, 0.02);
    assert.ok(!coarse.png.equals(fine.png), 'accuracy visibly changes the mesh');
  });
});

describe('errors name the argument that was wrong', () => {
  const cases = [
    [{ view: 'isometric' }, /unknown view 'isometric'/],
    [{ mode: 'wireframe' }, /unknown mode 'wireframe'/],
    [{ zoom: -1 }, /zoom must be a positive number/],
    [{ section: { axis: 'W' } }, /section\.axis must be one of X, Y, Z/],
    [{ region: { min: [1, 1, 1], max: [0, 0, 0] } }, /region\.max\[0\]/],
    [{ direction: [0, 0, 0] }, /direction must be a non-zero vector/],
    [{ accuracy: 0 }, /accuracy must be a positive number/],
  ];

  for (const [args, message] of cases) {
    test(`rejects ${JSON.stringify(args)}`, () => {
      const t = freshTools();
      const out = t.call('get_screenshot', { model_id: plateModel(t), ...args });
      assert.equal(out.isError, true);
      assert.match(out.content[0].text, message);
    });
  }

  test('an unknown model_id is an error, not a blank image', () => {
    const out = freshTools().call('get_screenshot', { model_id: 'model-404-abcd' });
    assert.equal(out.isError, true);
    assert.match(out.content[0].text, /unknown model_id/);
  });

  test('an empty model says so rather than rendering the background', () => {
    const t = freshTools();
    const id = jsonOf(
      t.call('create_model', { script: 'return Shape.box3(1,1,1).subtract(Shape.box3(2,2,2));' }),
    ).model_id;
    const out = t.call('get_screenshot', { model_id: id });
    assert.equal(out.isError, true);
    assert.match(out.content[0].text, /empty mesh/);
  });
});
