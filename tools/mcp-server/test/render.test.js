// Renderer tests: framing, section cutting, edge line-work, determinism.
//
// Pure JS — the meshes here are hand-built so every expectation is exact and
// nothing depends on the tessellator. The kernel-backed side of the visual
// inspection channel (silhouette edges, the MCP surface) is in
// test/screenshot.test.js.

import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import { renderScene, renderPng, viewDirection, VIEW_NAMES, RENDER_MODES } from '../src/render.js';
import { decodePng } from '../src/png.js';

/**
 * A closed, outward-wound axis-aligned box, as flat mesh buffers.
 * Positions are ordered so bit 0/1/2 of the index selects max on x/y/z.
 */
function boxMesh([x0, y0, z0], [x1, y1, z1]) {
  const positions = [];
  for (let i = 0; i < 8; i++) {
    positions.push(i & 1 ? x1 : x0, i & 2 ? y1 : y0, i & 4 ? z1 : z0);
  }
  // Each face wound counter-clockwise seen from outside.
  const indices = [
    // -x (0,2,6,4)            +x (1,5,7,3)
    0, 4, 6, 0, 6, 2, 1, 3, 7, 1, 7, 5,
    // -y (0,1,5,4)            +y (2,6,7,3)
    0, 1, 5, 0, 5, 4, 2, 6, 7, 2, 7, 3,
    // -z (0,2,3,1)            +z (4,5,7,6)
    0, 2, 3, 0, 3, 1, 4, 5, 7, 4, 7, 6,
  ];
  return {
    positions: new Float32Array(positions),
    indices: new Uint32Array(indices),
    bounds: [x0, y0, z0, x1, y1, z1],
  };
}

/** The 12 edges of a box, as a flat [x0,y0,z0,x1,y1,z1,…] segment buffer. */
function boxEdges([x0, y0, z0], [x1, y1, z1]) {
  const corner = (i) => [i & 1 ? x1 : x0, i & 2 ? y1 : y0, i & 4 ? z1 : z0];
  const flat = [];
  for (let a = 0; a < 8; a++) {
    for (const bit of [1, 2, 4]) {
      const b = a | bit;
      if (b === a) continue;
      flat.push(...corner(a), ...corner(b));
    }
  }
  return new Float32Array(flat);
}

/** Concatenate meshes into one buffer pair, re-basing the indices. */
function concatMeshes(...meshes) {
  const positions = [];
  const indices = [];
  const bounds = [Infinity, Infinity, Infinity, -Infinity, -Infinity, -Infinity];
  for (const m of meshes) {
    const base = positions.length / 3;
    positions.push(...m.positions);
    for (const i of m.indices) indices.push(i + base);
    for (let k = 0; k < 3; k++) {
      bounds[k] = Math.min(bounds[k], m.bounds[k]);
      bounds[k + 3] = Math.max(bounds[k + 3], m.bounds[k + 3]);
    }
  }
  return {
    positions: new Float32Array(positions),
    indices: new Uint32Array(indices),
    bounds,
  };
}

const CUBE = boxMesh([-1, -1, -1], [1, 1, 1]);

/** Count pixels matching a predicate over the decoded framebuffer. */
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

const isSteelBlue = (r, g, b) => b > r && b > g && b > 60;
const isAmber = (r, g, b) => r > b + 30 && r > 80;
const isInk = (r, g, b) => r < 60 && g < 60 && b < 60;
const isPaper = (r, g, b) => r > 200 && g > 200 && b > 200;

describe('framing', () => {
  test('reports the camera it resolved, not the camera it was asked for', () => {
    const { camera } = renderScene(CUBE, CUBE.bounds, { view: 'front', width: 200, height: 100 });
    assert.equal(camera.view, 'front');
    assert.deepEqual(camera.direction, [0, 0, -1]);
    assert.deepEqual(camera.up, [0, 1, 0]);
    assert.deepEqual(camera.target, [0, 0, 0]);
    assert.equal(camera.zoom, 1);
    assert.equal(camera.mode, 'shaded');
    assert.equal(camera.section, null);
    // Fit is the short axis: 100 px * 0.9 margin over a 2-unit span.
    assert.equal(camera.scale, 45);
    assert.deepEqual(camera.visibleExtent, [200 / 45, 100 / 45].map((n) => Math.round(n * 1e4) / 1e4));
  });

  test('zoom is an exact multiplier on the fitted scale', () => {
    const base = renderScene(CUBE, CUBE.bounds, { view: 'front', width: 200, height: 200 }).camera;
    const close = renderScene(CUBE, CUBE.bounds, { view: 'front', width: 200, height: 200, zoom: 4 }).camera;
    assert.equal(close.scale, base.scale * 4);
    assert.equal(close.zoom, 4);
  });

  test('a region frames that box instead of the whole model', () => {
    const whole = renderScene(CUBE, CUBE.bounds, { view: 'front', width: 200, height: 200 }).camera;
    const quarter = renderScene(CUBE, CUBE.bounds, {
      view: 'front',
      width: 200,
      height: 200,
      region: { min: [0, 0, -1], max: [1, 1, 1] },
    }).camera;
    // Half the span in view, so twice the scale, centred on the region.
    assert.equal(quarter.scale, whole.scale * 2);
    assert.deepEqual(quarter.target, [0.5, 0.5, 0]);
  });

  test('target re-centres the frame without changing the fit', () => {
    const base = renderScene(CUBE, CUBE.bounds, { view: 'front', width: 200, height: 200 }).camera;
    const shifted = renderScene(CUBE, CUBE.bounds, {
      view: 'front',
      width: 200,
      height: 200,
      target: [1, 0, 0],
    });
    assert.equal(shifted.camera.scale, base.scale);
    assert.deepEqual(shifted.camera.target, [1, 0, 0]);
    // Re-centring on the right-hand face moves the part left in frame.
    const left = countPixels(shifted.png, isSteelBlue);
    assert.ok(left > 0, 'the part is still visible');
    assert.notDeepEqual(shifted.png, renderScene(CUBE, CUBE.bounds, { view: 'front', width: 200, height: 200 }).png);
  });

  test('an arbitrary direction reports no named view and a derived up', () => {
    const { camera } = renderScene(CUBE, CUBE.bounds, { direction: [0, 0, -2], width: 64, height: 64 });
    assert.equal(camera.view, null);
    assert.deepEqual(camera.direction, [0, 0, -1]); // normalized
    assert.deepEqual(camera.up, [0, 1, 0]);
  });

  test('a straight-down direction falls back to the named top view convention', () => {
    const custom = renderScene(CUBE, CUBE.bounds, { direction: [0, -1, 0], width: 64, height: 64 });
    const named = renderScene(CUBE, CUBE.bounds, { view: 'top', width: 64, height: 64 });
    assert.deepEqual(custom.camera.up, named.camera.up);
    assert.ok(custom.png.equals(named.png), 'same camera, same image');
  });

  test('a heavy zoom clips line-work to the frame instead of walking off it', () => {
    // Without segment clipping this walks millions of pixels per edge.
    const started = process.hrtime.bigint();
    const { png } = renderScene(CUBE, CUBE.bounds, {
      view: 'front',
      width: 64,
      height: 64,
      zoom: 5000,
      mode: 'edges',
      edges: { feature: boxEdges([-1, -1, -1], [1, 1, 1]) },
    });
    const ms = Number(process.hrtime.bigint() - started) / 1e6;
    assert.ok(ms < 2000, `render took ${ms} ms`);
    assert.equal(decodePng(png).width, 64);
  });

  test('viewDirection agrees with the direction the render reports', () => {
    for (const view of VIEW_NAMES) {
      const { camera } = renderScene(CUBE, CUBE.bounds, { view, width: 32, height: 32 });
      assert.deepEqual(
        viewDirection({ view }).map((n) => Math.round(n * 1e4) / 1e4),
        camera.direction,
      );
    }
    assert.deepEqual(viewDirection({ direction: [0, 0, -5] }), [0, 0, -1]);
  });
});

describe('render modes', () => {
  const featureEdges = boxEdges([-1, -1, -1], [1, 1, 1]);

  test('shaded fills the solid on a dark ground', () => {
    const png = renderPng(CUBE, CUBE.bounds, { view: 'iso', width: 120, height: 120 });
    assert.ok(countPixels(png, isSteelBlue) > 1000);
    assert.equal(countPixels(png, isPaper), 0);
  });

  test('edges draws line-work on a light ground and no material', () => {
    const png = renderPng(CUBE, CUBE.bounds, {
      view: 'iso',
      width: 120,
      height: 120,
      mode: 'edges',
      edges: { feature: featureEdges },
    });
    assert.ok(countPixels(png, isPaper) > 1000, 'light ground');
    assert.ok(countPixels(png, isInk) > 100, 'ink present');
    assert.equal(countPixels(png, isSteelBlue), 0, 'no shaded material');
  });

  test('shaded_edges keeps the solid and adds ink', () => {
    const shaded = renderPng(CUBE, CUBE.bounds, { view: 'iso', width: 120, height: 120 });
    const overlaid = renderPng(CUBE, CUBE.bounds, {
      view: 'iso',
      width: 120,
      height: 120,
      mode: 'shaded_edges',
      edges: { feature: featureEdges },
    });
    assert.ok(countPixels(overlaid, isSteelBlue) > 0);
    assert.ok(countPixels(overlaid, isInk) > countPixels(shaded, isInk));
  });

  test('hidden line-work is removed, not drawn through the solid', () => {
    // A small box parked entirely in the shadow of a bigger one. Front-on,
    // every edge of the rear box is occluded, so supplying them must change
    // nothing at all about the image.
    const front = boxMesh([-2, -2, 0.5], [2, 2, 1]);
    const rear = boxMesh([-1, -1, -1], [1, 1, -0.5]);
    const scene = concatMeshes(front, rear);
    const opts = { view: 'front', width: 160, height: 160, mode: 'edges' };
    const visibleOnly = renderPng(scene, scene.bounds, {
      ...opts,
      edges: { feature: boxEdges([-2, -2, 0.5], [2, 2, 1]) },
    });
    const withHidden = renderPng(scene, scene.bounds, {
      ...opts,
      edges: {
        feature: Float32Array.from([
          ...boxEdges([-2, -2, 0.5], [2, 2, 1]),
          ...boxEdges([-1, -1, -1], [1, 1, -0.5]),
        ]),
      },
    });
    assert.ok(withHidden.equals(visibleOnly), 'occluded edges left no ink');
  });

  test('line_width thickens the ink', () => {
    const opts = {
      view: 'iso',
      width: 120,
      height: 120,
      mode: 'edges',
      edges: { feature: featureEdges },
    };
    const thin = renderPng(CUBE, CUBE.bounds, { ...opts, lineWidth: 1 });
    const thick = renderPng(CUBE, CUBE.bounds, { ...opts, lineWidth: 5 });
    assert.ok(countPixels(thick, isInk) > countPixels(thin, isInk) * 2);
  });

  test('every mode renders for every named view', () => {
    for (const mode of RENDER_MODES) {
      for (const view of VIEW_NAMES) {
        const png = renderPng(CUBE, CUBE.bounds, {
          view,
          mode,
          width: 40,
          height: 40,
          edges: { feature: featureEdges },
        });
        assert.equal(png.subarray(0, 8).toString('hex'), '89504e470d0a1a0a', `${mode}/${view}`);
      }
    }
  });
});

describe('section views', () => {
  test('the cut face is shaded in its own colour, not as material', () => {
    const plain = renderPng(CUBE, CUBE.bounds, { view: 'front', width: 120, height: 120 });
    const cut = renderPng(CUBE, CUBE.bounds, {
      view: 'front',
      width: 120,
      height: 120,
      section: { axis: 'Z' },
    });
    assert.equal(countPixels(plain, isAmber), 0);
    // Front-on through a cube cut at mid-Z: the whole silhouette is cut face,
    // down to a one-pixel rim of side wall at the silhouette itself.
    assert.ok(isAmber(...pixelAt(cut, 60, 60)), 'the middle of the frame is cut face');
    const amber = countPixels(cut, isAmber);
    assert.ok(amber > 1000, `cut face covers ${amber} px`);
    assert.ok(countPixels(cut, isSteelBlue) * 20 < amber, 'material is only the silhouette rim');
  });

  test('an omitted offset re-seats the plane at the model midpoint', () => {
    const implied = renderScene(CUBE, CUBE.bounds, { view: 'front', section: { axis: 'X' } });
    const explicit = renderScene(CUBE, CUBE.bounds, {
      view: 'front',
      section: { axis: 'X', offset: 0 },
    });
    assert.equal(implied.camera.section.offset, 0);
    assert.ok(implied.png.equals(explicit.png));
  });

  test('flip keeps the other half', () => {
    const opts = { view: 'top', width: 120, height: 120 };
    const near = renderPng(CUBE, CUBE.bounds, { ...opts, section: { axis: 'X', offset: 0 } });
    const far = renderPng(CUBE, CUBE.bounds, {
      ...opts,
      section: { axis: 'X', offset: 0, flip: true },
    });
    assert.ok(!near.equals(far), 'the two halves are different images');
    // Each half is half the material of the whole.
    const whole = countPixels(renderPng(CUBE, CUBE.bounds, opts), isSteelBlue);
    const half = countPixels(near, isSteelBlue) + countPixels(near, isAmber);
    assert.ok(Math.abs(half - whole / 2) < whole * 0.05, `half ${half} of whole ${whole}`);
  });

  test('a plane clear of the model changes nothing', () => {
    const opts = { view: 'iso', width: 100, height: 100 };
    const plain = renderPng(CUBE, CUBE.bounds, opts);
    const uncut = renderPng(CUBE, CUBE.bounds, { ...opts, section: { axis: 'Y', offset: 5 } });
    assert.ok(uncut.equals(plain), 'nothing clipped and no cap drawn');
  });

  test('a plane past the model removes it entirely', () => {
    const png = renderPng(CUBE, CUBE.bounds, {
      view: 'iso',
      width: 100,
      height: 100,
      section: { axis: 'Y', offset: -5 },
    });
    assert.equal(countPixels(png, isSteelBlue), 0);
    assert.equal(countPixels(png, isAmber), 0);
  });

  test('a section hides the cut-away half of the line-work and outlines the cut', () => {
    const edges = { feature: boxEdges([-1, -1, -1], [1, 1, 1]) };
    const opts = { view: 'iso', width: 160, height: 160, mode: 'edges', edges };
    const whole = renderPng(CUBE, CUBE.bounds, opts);
    const cut = renderPng(CUBE, CUBE.bounds, { ...opts, section: { axis: 'X' } });
    assert.ok(!cut.equals(whole));
    assert.ok(countPixels(cut, isInk) > 100, 'the cut outline is drawn');
  });

  test('a section plane edge-on to the camera draws no cap', () => {
    // The camera lies *in* an X plane when it looks along Z: the cut face
    // projects to a zero-width line, so there is nothing to shade.
    const png = renderPng(CUBE, CUBE.bounds, {
      view: 'front',
      width: 100,
      height: 100,
      section: { axis: 'X' },
    });
    assert.equal(countPixels(png, isAmber), 0);
    assert.ok(countPixels(png, isSteelBlue) > 0, 'the kept half is still shaded');
  });
});

describe('determinism', () => {
  const shots = [
    { view: 'iso' },
    { view: 'front', mode: 'edges', edges: { feature: boxEdges([-1, -1, -1], [1, 1, 1]) } },
    { view: 'top', section: { axis: 'Y', offset: 0.25 }, mode: 'shaded_edges', edges: { feature: boxEdges([-1, -1, -1], [1, 1, 1]) } },
    { direction: [-1, -2, -3], zoom: 1.75, target: [0.2, 0, -0.1] },
    { region: { min: [-0.5, -0.5, -1], max: [0.5, 0.5, 1] }, view: 'front' },
  ];

  test('identical requests produce byte-identical images', () => {
    for (const shot of shots) {
      const a = renderPng(CUBE, CUBE.bounds, { width: 96, height: 72, ...shot });
      const b = renderPng(CUBE, CUBE.bounds, { width: 96, height: 72, ...shot });
      assert.ok(a.equals(b), `unstable render for ${JSON.stringify(shot)}`);
    }
  });

  test('each framing lever actually changes the image', () => {
    const base = renderPng(CUBE, CUBE.bounds, { view: 'iso', width: 96, height: 72 });
    for (const shot of shots.slice(1)) {
      const other = renderPng(CUBE, CUBE.bounds, { width: 96, height: 72, ...shot });
      assert.ok(!other.equals(base), `no visible effect from ${JSON.stringify(shot)}`);
    }
  });
});

describe('argument validation', () => {
  const cases = [
    [{ view: 'isometric' }, /unknown view 'isometric'/],
    [{ mode: 'wireframe' }, /unknown mode 'wireframe'/],
    [{ zoom: 0 }, /zoom must be a positive number/],
    [{ zoom: -3 }, /zoom must be a positive number/],
    [{ direction: [0, 0, 0] }, /direction must be a non-zero vector/],
    [{ direction: [1, 2] }, /direction must be \[x, y, z\]/],
    [{ target: 'origin' }, /target must be \[x, y, z\]/],
    [{ section: { axis: 'W' } }, /section\.axis must be one of X, Y, Z/],
    [{ section: { axis: 'X', offset: 'middle' } }, /section\.offset must be a finite number/],
    [{ section: 'X' }, /section must be an object/],
    [{ region: { min: [0, 0, 0] } }, /region needs both min and max/],
    [{ region: { min: [0, 0, 0], max: [1, -1, 1] } }, /region\.max\[1\] \(-1\) is below region\.min\[1\] \(0\)/],
    [{ region: [0, 0, 0, 1, 1, 1] }, /region must be an object/],
  ];

  for (const [opts, message] of cases) {
    test(`rejects ${JSON.stringify(opts)}`, () => {
      assert.throws(() => renderPng(CUBE, CUBE.bounds, opts), message);
    });
  }

  test('an empty mesh renders the background rather than throwing', () => {
    const empty = { positions: new Float32Array(0), indices: new Uint32Array(0) };
    const png = renderPng(empty, [0, 0, 0, 0, 0, 0], { width: 32, height: 32 });
    assert.equal(countPixels(png, isSteelBlue), 0);
    assert.equal(decodePng(png).width, 32);
  });
});
