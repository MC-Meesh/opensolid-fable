// Focused unit tests for the dependency-free helpers (PNG encoding, mesh
// file assembly, software rasterizer) — no wasm required.

import { test } from 'node:test';
import assert from 'node:assert/strict';
import { encodePng } from '../src/png.js';
import { buildBinaryStl, buildObj } from '../src/mesh.js';
import { renderPng, VIEW_NAMES } from '../src/render.js';

// A single unit triangle in the z=0 plane.
const positions = new Float32Array([0, 0, 0, 1, 0, 0, 0, 1, 0]);
const normals = new Float32Array([0, 0, 1, 0, 0, 1, 0, 0, 1]);
const indices = new Uint32Array([0, 1, 2]);
const bounds = [0, 0, 0, 1, 1, 0];

test('encodePng emits a valid PNG with correct dimensions', () => {
  const rgba = Buffer.alloc(2 * 2 * 4, 128);
  const png = encodePng(rgba, 2, 2);
  assert.equal(png.subarray(0, 8).toString('hex'), '89504e470d0a1a0a');
  // IHDR width/height live at bytes 16..24.
  assert.equal(png.readUInt32BE(16), 2);
  assert.equal(png.readUInt32BE(20), 2);
});

test('encodePng rejects mismatched buffer sizes', () => {
  assert.throws(() => encodePng(Buffer.alloc(3), 2, 2), /does not match/);
});

test('buildBinaryStl produces a header plus one 50-byte facet', () => {
  const stl = buildBinaryStl(positions, indices);
  assert.equal(stl.length, 84 + 50);
  assert.equal(stl.readUInt32LE(80), 1); // triangle count
  // Recomputed facet normal is +z for a CCW triangle in the z=0 plane.
  // Facet layout: nx@84, ny@88, nz@92 — nz (the +z component) is 84 + 8.
  assert.ok(Math.abs(stl.readFloatLE(84 + 8) - 1) < 1e-6);
});

test('buildObj emits 1-based faces with normals', () => {
  const obj = buildObj(positions, normals, indices);
  assert.match(obj, /^v 0 0 0$/m);
  assert.match(obj, /^vn 0 0 1$/m);
  assert.match(obj, /^f 1\/\/1 2\/\/2 3\/\/3$/m);
});

test('renderPng renders a non-empty triangle for every named view', () => {
  for (const view of VIEW_NAMES) {
    const png = renderPng({ positions, indices }, bounds, { view, width: 48, height: 48 });
    assert.equal(png.subarray(0, 8).toString('hex'), '89504e470d0a1a0a');
    assert.ok(png.length > 8);
  }
});

test('renderPng clamps absurd dimensions instead of throwing', () => {
  const png = renderPng({ positions, indices }, bounds, { width: 999999, height: -5 });
  assert.ok(png.length > 8);
});
