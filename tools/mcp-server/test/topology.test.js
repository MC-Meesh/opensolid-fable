// Correctness oracles for agents (of-2y4.5).
//
// The bar these tests hold: the friction log's bracket, whose four holes were
// bored sideways, satisfied every oracle the agent had — `valid: true`, a
// plausible screenshot, a clean STL, a volume only ~4% light. So it is not
// enough that these tools return numbers. Each test below pairs a *correct*
// part with a *specifically wrong* one and requires the tool to separate them.
// A test that only asserts "the right part passes" would have passed on the
// wrong bracket too.

import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import { mkdtempSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { createTools, flattenProbes } from '../src/tools.js';
import { Shape } from '../src/kernel.js';
import { getMesh } from '../src/mesh.js';
import {
  fitCircle,
  smallestEigenvector,
  meshTopology,
  segmentPlanarFaces,
  detectRims,
  groupCylinders,
  probeAxis,
  inspect,
} from '../src/topology.js';

function freshTools() {
  return createTools({ outputDir: mkdtempSync(join(tmpdir(), 'osmcp-topo-')) });
}

function callJson(t, name, args) {
  const out = t.call(name, args);
  assert.equal(out.isError, undefined, out.content[0].text);
  return JSON.parse(out.content[0].text);
}

/** A 30 × 5 × 20 plate. Sweeps run along +Y, so its thickness is y. */
const PLATE_SCRIPT = 'return Shape.box3(15, 2.5, 10);';
/** The same plate with one Ø5 hole through its 5 mm thickness (correct: +Y). */
const RIGHT_AXIS_SCRIPT =
  'return Shape.box3(15, 2.5, 10).subtract(Shape.cylinder(2.5, 10));';
/**
 * The of-4tu bug, reproduced: the docs said `cylinder` was "+Z", so the drill
 * was rotated onto z. The hole now runs along the plate's 20 mm dimension
 * instead of through its 5 mm thickness.
 */
const WRONG_AXIS_SCRIPT =
  'return Shape.box3(15, 2.5, 10).subtract(Shape.cylinder(2.5, 10).rotate(1, 0, 0, 90));';
/** Four Ø5 holes through the thickness, as a mounting plate really has. */
const FOUR_HOLES_SCRIPT = `
  let s = Shape.box3(15, 2.5, 10);
  for (const [x, z] of [[10, 6], [-10, 6], [10, -6], [-10, -6]]) {
    s = s.subtract(Shape.cylinder(2.5, 10).translate(x, 0, z));
  }
  return s;`;

describe('mesh topology: exact combinatorics, no tolerance', () => {
  test('genus counts the holes through a plate and does not drift with accuracy', () => {
    const four = Shape.box3(15, 2.5, 10)
      .subtract(Shape.cylinder(2.5, 10).translate(10, 0, 6))
      .subtract(Shape.cylinder(2.5, 10).translate(-10, 0, 6))
      .subtract(Shape.cylinder(2.5, 10).translate(10, 0, -6))
      .subtract(Shape.cylinder(2.5, 10).translate(-10, 0, -6));
    // Two accuracies a factor of two apart: the triangle count changes a lot,
    // the genus must not change at all. That invariance is the whole reason to
    // prefer this over a volume as an oracle.
    const coarse = meshTopology(...meshArrays(four, 0.3));
    const fine = meshTopology(...meshArrays(four, 0.15));
    assert.ok(fine.triangles > coarse.triangles * 1.2, 'the two meshes should differ materially');
    assert.equal(coarse.genus, 4);
    assert.equal(fine.genus, 4);
    assert.equal(coarse.components, 1);
    assert.equal(coarse.closed, true);
    // V - E + F = 2(C - G).
    assert.equal(coarse.eulerCharacteristic, 2 * (coarse.components - coarse.genus));
  });

  test('a plain box is genus 0 and a severed part is two shells', () => {
    const box = meshTopology(...meshArrays(Shape.box3(10, 5, 8), 0.2));
    assert.equal(box.genus, 0);
    assert.equal(box.components, 1);

    // A bore longer than the plate's z extent cuts it clean in half. The volume
    // is nearly the same as a proper hole's; the shell count is not.
    const severed = meshTopology(
      ...meshArrays(Shape.box3(15, 2.5, 10).subtract(Shape.cylinder(2.5, 20).rotate(1, 0, 0, 90)), 0.2),
    );
    assert.equal(severed.components, 2, 'the plate fell apart');
  });

  test('genus is null rather than wrong on an open surface', () => {
    // Two triangles sharing an edge: a closed formula would report nonsense.
    const positions = new Float32Array([0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0]);
    const indices = new Uint32Array([0, 1, 2, 1, 3, 2]);
    const t = meshTopology(positions, indices);
    assert.equal(t.closed, false);
    assert.equal(t.genus, null);
  });

  test('vertices are welded, so a duplicated-corner buffer is still one surface', () => {
    // The same single triangle written twice with distinct vertex ids. Without
    // welding this reads as two components and six vertices.
    const positions = new Float32Array([0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0]);
    const indices = new Uint32Array([0, 1, 2, 3, 4, 5]);
    const t = meshTopology(positions, indices);
    assert.equal(t.vertices, 3);
    assert.equal(t.components, 1);
  });
});

describe('planar face recovery', () => {
  test('a box has six faces, not one per bevel facet', () => {
    // The naive crease-cutting algorithm reports 194 here; the whole point of
    // seed-plane growth is that this number is 6.
    const [positions, indices] = meshArrays(Shape.box3(10, 5, 8), 0.1);
    const planar = segmentPlanarFaces(positions, indices);
    assert.equal(planar.faces.length, 6);
    // The six normals are the six axis directions, each once.
    const dirs = planar.faces
      .map((f) => f.normal.map((c) => Math.round(c)).join(','))
      .sort();
    assert.deepEqual(dirs, ['-1,0,0', '0,-1,0', '0,0,-1', '0,0,1', '0,1,0', '1,0,0']);
    // Faces run a little under their analytic area because the bevel is not
    // theirs; `planarAreaFraction` is that discount, and it must be honest.
    assert.ok(
      planar.planarAreaFraction > 0.95 && planar.planarAreaFraction < 1,
      `planarAreaFraction ${planar.planarAreaFraction}`,
    );
    const biggest = planar.faces[0].area; // 20 x 16 = 320 analytically
    assert.ok(biggest > 0.95 * 320 && biggest <= 320, `largest face area ${biggest}`);
  });

  test('a sphere has no planar faces and says so', () => {
    const planar = segmentPlanarFaces(...meshArrays(Shape.sphere(5), 0.1));
    assert.equal(planar.faces.length, 0);
    assert.equal(planar.planarAreaFraction, 0);
    assert.ok(planar.totalArea > 0);
  });

  test('an empty mesh yields an empty census rather than throwing', () => {
    const planar = segmentPlanarFaces(new Float32Array(0), new Uint32Array(0));
    assert.deepEqual(planar.faces, []);
    assert.equal(planar.totalArea, 0);
  });
});

describe('circle fitting', () => {
  test('smallestEigenvector finds the normal of a set of coplanar points', () => {
    // Covariance of points spread in x and y only: the small direction is z.
    const n = smallestEigenvector([
      [4, 0, 0],
      [0, 9, 0],
      [0, 0, 1e-12],
    ]);
    assert.ok(Math.abs(Math.abs(n[2]) - 1) < 1e-6, `expected ±z, got ${n}`);
  });

  test('fits a ring in an arbitrary plane, in any point order', () => {
    // A radius-3 ring in the plane spanned by (1,1,0)/√2 and (0,0,1).
    const u = [Math.SQRT1_2, Math.SQRT1_2, 0];
    const v = [0, 0, 1];
    const centre = [2, -1, 5];
    const pts = [];
    for (let i = 0; i < 24; i += 1) {
      const a = (2 * Math.PI * i) / 24;
      pts.push([
        centre[0] + 3 * Math.cos(a) * u[0] + 3 * Math.sin(a) * v[0],
        centre[1] + 3 * Math.cos(a) * u[1] + 3 * Math.sin(a) * v[1],
        centre[2] + 3 * Math.cos(a) * u[2] + 3 * Math.sin(a) * v[2],
      ]);
    }
    // Shuffled deterministically: the fit must not depend on ring order, which
    // is exactly what the ordered-walk fit this replaces did depend on.
    const shuffled = pts.filter((_, i) => i % 2 === 0).concat(pts.filter((_, i) => i % 2 === 1));
    const fit = fitCircle(shuffled);
    assert.ok(fit, 'a clean ring must fit');
    assert.ok(Math.abs(fit.radius - 3) < 1e-6, `radius ${fit.radius}`);
    for (let k = 0; k < 3; k += 1) {
      assert.ok(Math.abs(fit.center[k] - centre[k]) < 1e-6, `centre ${fit.center}`);
    }
    assert.ok(fit.coverageDeg > 340, `coverage ${fit.coverageDeg}`);
  });

  test('tolerates spur outliers — the failure that hid multi-hole rims', () => {
    // A real rim recovered from a mesh arrives with short spurs hanging off it
    // where the crease test also fired on the mesher's bevel: 37 of 186
    // vertices, measured on a four-hole plate. They sit just outside the rim,
    // near its plane. The fit must survive them, or every hole in a multi-hole
    // part goes undetected — which is exactly what the ordered-ring fit did.
    const ring = [];
    for (let i = 0; i < 40; i += 1) {
      const a = (2 * Math.PI * i) / 40;
      ring.push([5 * Math.cos(a), 5 * Math.sin(a), 0]);
    }
    const spurs = [];
    for (let i = 0; i < 10; i += 1) {
      const a = (2 * Math.PI * i) / 10;
      const r = 5.5 + 0.2 * (i % 3);
      spurs.push([r * Math.cos(a), r * Math.sin(a), 0.15 * ((i % 2) * 2 - 1)]);
    }
    const fit = fitCircle(ring.concat(spurs));
    assert.ok(fit, 'must still fit with 20% outliers');
    assert.ok(Math.abs(fit.radius - 5) < 0.02, `radius ${fit.radius}`);
    assert.equal(fit.inliers, 40);
    assert.equal(fit.total, 50);
  });

  test('rejects an arc, so a fillet is not mistaken for a rim', () => {
    const arc = [];
    for (let i = 0; i < 20; i += 1) {
      const a = (Math.PI / 2) * (i / 19); // a quarter turn
      arc.push([4 * Math.cos(a), 4 * Math.sin(a), 0]);
    }
    assert.equal(fitCircle(arc), null);
  });

  test('rejects a point set that is not a circle at all', () => {
    const square = [];
    for (let i = 0; i < 40; i += 1) {
      const t = i / 10;
      const side = Math.floor(i / 10);
      const s = t - side;
      square.push(
        [
          [s, 0, 0],
          [1, s, 0],
          [1 - s, 1, 0],
          [0, 1 - s, 0],
        ][side],
      );
    }
    assert.equal(fitCircle(square), null);
  });
});

describe('hole detection separates a right-axis hole from a wrong-axis one', () => {
  test('the correct plate reports one +Y through-hole of the right size', () => {
    const shape = Shape.box3(15, 2.5, 10).subtract(Shape.cylinder(2.5, 10));
    const census = inspect(shape, getMesh(shape, { accuracy: 0.2 }));
    assert.equal(census.counts.throughHoles, 1);
    assert.equal(census.counts.genus, 1);
    const hole = census.cylinders[0];
    assert.equal(hole.kind, 'through-hole');
    assert.ok(Math.abs(hole.diameter - 5) < 0.1, `diameter ${hole.diameter}`);
    // Axis along y (either sign — a hole has no preferred direction).
    assert.ok(Math.abs(Math.abs(hole.axis[1]) - 1) < 0.02, `axis ${hole.axis}`);
    // Through a 5 mm thickness.
    assert.ok(Math.abs(hole.depth - 5) < 0.15, `depth ${hole.depth}`);
  });

  test('four holes are found individually, at their four centres', () => {
    let shape = Shape.box3(15, 2.5, 10);
    for (const [x, z] of [[10, 6], [-10, 6], [10, -6], [-10, -6]]) {
      shape = shape.subtract(Shape.cylinder(2.5, 10).translate(x, 0, z));
    }
    const census = inspect(shape, getMesh(shape, { accuracy: 0.2 }));
    assert.equal(census.counts.throughHoles, 4);
    assert.equal(census.counts.genus, 4);
    const centres = census.cylinders
      .map((c) => `${Math.round(c.center[0])},${Math.round(c.center[2])}`)
      .sort();
    assert.deepEqual(centres, ['-10,-6', '-10,6', '10,-6', '10,6']);
    for (const c of census.cylinders) {
      assert.ok(Math.abs(Math.abs(c.axis[1]) - 1) < 0.02, `axis ${c.axis}`);
    }
  });

  test('a probe along the intended axis fails on the wrong-axis part', () => {
    const right = Shape.box3(15, 2.5, 10).subtract(Shape.cylinder(2.5, 10));
    const wrong = Shape.box3(15, 2.5, 10).subtract(
      Shape.cylinder(2.5, 10).rotate(1, 0, 0, 90),
    );
    // Down the intended bore: the line is inside the hole its whole length, so
    // it never meets material. True of both parts, since both bores pass
    // through the origin — a probe along the bore cannot tell them apart.
    for (const [name, shape] of [['right', right], ['wrong', wrong]]) {
      const down = probeAxis(shape, [0, 1, 0], [0, 0, 0]);
      assert.equal(down.materialLength, 0, `${name}: no material down the bore`);
    }
    // Across the plate along z, through the same point, does tell them apart.
    // The correct part: the bore runs along y, so the z line crosses 15 mm of
    // plate in two runs either side of the Ø5 gap.
    const acrossRight = probeAxis(right, [0, 0, 1], [0, 0, 0]);
    assert.equal(acrossRight.throughHole, true);
    assert.ok(Math.abs(acrossRight.materialLength - 15) < 0.05, `material ${acrossRight.materialLength}`);
    // The wrong part: the bore runs *along z*, so the same line is inside it
    // end to end and meets nothing. Same volume to within a few percent, same
    // `valid: true`, and a completely different part.
    const acrossWrong = probeAxis(wrong, [0, 0, 1], [0, 0, 0]);
    assert.equal(acrossWrong.materialLength, 0, 'the line runs down the mis-drilled bore');
    // Off the bore, the intended axis crosses the full 5 mm thickness.
    const solid = probeAxis(right, [0, 1, 0], [10, 0, 0]);
    assert.equal(solid.throughHole, false);
    assert.ok(Math.abs(solid.materialLength - 5) < 1e-6, `thickness ${solid.materialLength}`);
  });

  test('a probe across a hole reports material, gap, material', () => {
    const shape = Shape.box3(15, 2.5, 10).subtract(Shape.cylinder(2.5, 10));
    const across = probeAxis(shape, [1, 0, 0], [0, 0, 0]);
    assert.equal(across.throughHole, true);
    assert.equal(across.solidSpans, 2);
    assert.equal(across.voidSpans, 1);
    // The gap across the bore's centre is its diameter.
    assert.ok(Math.abs(across.gapLength - 5) < 0.05, `gap ${across.gapLength}`);
    // And the two solid runs sum to the plate minus the bore.
    assert.ok(Math.abs(across.materialLength - 25) < 0.05, `material ${across.materialLength}`);
  });

  test('a boss is not reported as a hole', () => {
    // A pin standing on a plate: two coaxial rims, bore full of material.
    const shape = Shape.box3(15, 2.5, 10).union(Shape.cylinder(3, 5).translate(0, 5, 0));
    const census = inspect(shape, getMesh(shape, { accuracy: 0.2 }));
    assert.equal(census.counts.throughHoles, 0, 'a pin is not a hole');
    assert.equal(census.counts.genus, 0);
  });

  test('probeAxis rejects a degenerate axis or point', () => {
    const shape = Shape.box3(1, 1, 1);
    assert.throws(() => probeAxis(shape, [0, 0, 0], [0, 0, 0]), /non-zero/);
    assert.throws(() => probeAxis(shape, [0, 1, 0], [0, 0]), /finite \[x,y,z\] point/);
    assert.throws(() => probeAxis(shape, [0, 1, 0], [0, NaN, 0]), /finite \[x,y,z\] point/);
  });

  test('a blind pocket is a pocket, not a through-hole', () => {
    // The subtle case: a pocket has *two* fitted rims — its mouth and the circle
    // where its floor meets its wall — with clear space between them, so a bore
    // emptiness test alone calls it a through-hole. What separates them is that
    // material lies past one rim and air past the other.
    const shape = Shape.box3(15, 5, 10).subtract(Shape.cylinder(2.5, 3).translate(0, 4, 0));
    const features = groupCylinders(detectRims(...meshArrays(shape, 0.2)));
    assert.ok(features.length >= 1, 'the pocket is found as a cylindrical feature');
    const census = inspect(shape, getMesh(shape, { accuracy: 0.2 }));
    assert.equal(census.counts.throughHoles, 0, 'a blind pocket does not go through');
    assert.equal(census.counts.pockets, 1);
    assert.equal(census.counts.genus, 0, 'and it adds no handle to the surface');
    const pocket = census.cylinders.find((c) => c.kind === 'pocket');
    // Cut from y=1 up through the top face at y=5: 4 deep.
    assert.ok(Math.abs(pocket.depth - 4) < 0.2, `pocket depth ${pocket.depth}`);
    assert.ok(Math.abs(pocket.diameter - 5) < 0.15, `pocket Ø${pocket.diameter}`);
  });
});

describe('the inspect_topology tool', () => {
  test('reports hole axes, genus and probes for a drilled plate', () => {
    const t = freshTools();
    const model = callJson(t, 'create_model', { script: FOUR_HOLES_SCRIPT, name: 'plate4' });
    const out = callJson(t, 'inspect_topology', {
      model_id: model.model_id,
      accuracy: 0.2,
      probes: [
        { axis: [0, 1, 0], at: [10, 0, 6] },
        { axis: [1, 0, 0], at: [10, 0, 6] },
      ],
    });
    assert.equal(out.counts.genus, 4);
    assert.equal(out.counts.throughHoles, 4);
    assert.equal(out.counts.shells, 1);
    assert.equal(out.counts.planarFaces, 6);
    // Probe 1 runs down a bore: no material at all.
    assert.equal(out.probes[0].materialLength, 0);
    // Probe 2 crosses it: material, the Ø5 gap, material.
    assert.equal(out.probes[1].throughHole, true);
    assert.ok(Math.abs(out.probes[1].gapLength - 5) < 0.05);
    // An SDF-path model has no B-Rep, and the report says so rather than
    // implying the counts came from one.
    assert.equal(out.brep.available, false);
    assert.match(out.brep.reason, /exact/);
  });

  test('include_faces:false drops the face list but keeps the counts', () => {
    const t = freshTools();
    const model = callJson(t, 'create_model', { script: PLATE_SCRIPT });
    const out = callJson(t, 'inspect_topology', {
      model_id: model.model_id,
      accuracy: 0.3,
      include_faces: false,
    });
    assert.equal(out.planarFaces.faces, undefined);
    assert.equal(out.planarFaces.count, 6);
  });

  test('an exact model carries the B-Rep entity census', () => {
    const t = freshTools();
    const model = callJson(t, 'create_model', { script: RIGHT_AXIS_SCRIPT, exact: true });
    const out = callJson(t, 'inspect_topology', { model_id: model.model_id, accuracy: 0.3 });
    assert.equal(out.brep.available, true);
    // The true counts, which no tessellation can supply: a drilled block is
    // six planes plus the bore wall.
    assert.equal(out.brep.counts.surfaceKinds.cylinder, 1);
    assert.equal(out.brep.counts.surfaceKinds.plane, 6);
    assert.equal(out.brep.counts.innerLoops, 2, 'the bore breaks through two faces');
  });

  // An analytic tessellation is the opposite regime from an SDF mesh: a
  // rectangular face arrives as exactly two triangles, not hundreds. Both the
  // planar census and the B-Rep census have to hold up on it.
  test('a re-imported STEP body reports six faces and its own entity counts', () => {
    const t = freshTools();
    const built = callJson(t, 'create_model', { script: RIGHT_AXIS_SCRIPT, exact: true });
    const written = callJson(t, 'export', {
      model_id: built.model_id,
      format: 'step',
      path: 'topology-roundtrip.step',
    });
    const imported = callJson(t, 'import_step', { path: written.path });
    assert.equal(imported.mesher, 'step-reader');
    assert.equal(imported.brepChecked, true);

    const out = callJson(t, 'inspect_topology', { model_id: imported.model_id });
    // The regression this guards: screening planar regions by triangle count
    // dropped every rectangular face here and reported 2 instead of 6.
    assert.equal(out.counts.planarFaces, 6, JSON.stringify(out.planarFaces.faces));
    assert.equal(out.counts.genus, 1);
    assert.equal(out.counts.throughHoles, 1);
    // And the authoritative counts, from the imported body itself: six planes
    // plus the bore wall, with a hole loop on each face the bore breaks through.
    assert.equal(out.brep.available, true);
    assert.equal(out.brep.source, 'step-import');
    assert.equal(out.brep.counts.faces, 7);
    assert.equal(out.brep.counts.surfaceKinds.plane, 6);
    assert.equal(out.brep.counts.surfaceKinds.cylinder, 1);
    assert.equal(out.brep.counts.innerLoops, 2);
    assert.equal(out.brep.counts.genus, 1);
  });

  test('an imported body is checked by validate — unknown provenance is the point', () => {
    const t = freshTools();
    const built = callJson(t, 'create_model', { script: RIGHT_AXIS_SCRIPT, exact: true });
    const written = callJson(t, 'export', {
      model_id: built.model_id,
      format: 'step',
      path: 'validate-roundtrip.step',
    });
    const imported = callJson(t, 'import_step', { path: written.path });
    const v = callJson(t, 'validate', { model_id: imported.model_id, deep: true });
    assert.equal(v.brep.available, true);
    assert.equal(v.brep.source, 'step-import');
    assert.equal(v.brep.selfIntersectionChecked, true);
    assert.deepEqual(v.brep.failures, [], 'our own analytic STEP should re-import sound');
    assert.equal(v.valid, true);
    assert.equal(v.mesher, 'step-reader');
  });

  test('a bad probe and an unknown model are clean errors', () => {
    const t = freshTools();
    const model = callJson(t, 'create_model', { script: PLATE_SCRIPT });
    const bad = t.call('inspect_topology', {
      model_id: model.model_id,
      probes: [{ axis: [0, 0, 0], at: [0, 0, 0] }],
    });
    assert.equal(bad.isError, true);
    assert.match(bad.content[0].text, /probe failed/);
    const missing = t.call('inspect_topology', { model_id: 'nope' });
    assert.equal(missing.isError, true);
    assert.match(missing.content[0].text, /unknown model_id/);
  });
});

describe('the assert_model tool', () => {
  test('the wrong-axis hole fails the axis assertion and names what is there', () => {
    const t = freshTools();
    const wrong = callJson(t, 'create_model', { script: WRONG_AXIS_SCRIPT, name: 'wrong' });
    const out = callJson(t, 'assert_model', {
      model_id: wrong.model_id,
      accuracy: 0.2,
      expect: [
        { type: 'closed_solid' },
        { type: 'through_holes', value: 1, axis: [0, 1, 0], diameter: 5 },
      ],
    });
    // This is the entire point of the tool: the part is a perfectly valid
    // closed solid, and the hole is on the wrong axis.
    const closed = out.checks.find((c) => c.type === 'closed_solid');
    const holes = out.checks.find((c) => c.type === 'through_holes');
    assert.equal(closed.ok, true, 'the wrong part really is a valid solid');
    assert.equal(holes.ok, false, 'and its hole is on the wrong axis');
    assert.equal(out.ok, false);
    assert.match(holes.message, /axis/);
  });

  test('the correct part passes the same assertions', () => {
    const t = freshTools();
    const right = callJson(t, 'create_model', { script: RIGHT_AXIS_SCRIPT, name: 'right' });
    const out = callJson(t, 'assert_model', {
      model_id: right.model_id,
      accuracy: 0.2,
      expect: [
        { type: 'closed_solid' },
        { type: 'through_holes', value: 1, axis: [0, 1, 0], diameter: 5 },
        { type: 'genus', value: 1 },
        { type: 'shells', value: 1 },
        { type: 'planar_faces', value: 6 },
        // 30 x 5 x 20 minus a Ø5 bore through the 5 mm thickness.
        { type: 'volume', value: 3000 - Math.PI * 2.5 * 2.5 * 5, relative_tolerance: 0.02 },
        { type: 'bbox_size', value: [30, 5, 20], tolerance: 0.2 },
        { type: 'centroid', value: [0, 0, 0], tolerance: 0.05 },
        // The bore's own axis (+Y) and a point on its centreline.
        { type: 'hole_at', at: [0, 0, 0], axis: [0, 1, 0], diameter: 5, tolerance: 0.1 },
        { type: 'material_at', at: [14, 0, 9], value: true },
        { type: 'material_at', at: [0, 0, 0], value: false },
        { type: 'clearance', probes: [[0, 20, 0]], min: 10 },
      ],
    });
    assert.equal(out.ok, true, JSON.stringify(out.checks.filter((c) => !c.ok), null, 2));
    assert.equal(out.failed, 0);
    assert.equal(out.passed, 12);
  });

  test('an expectation that cannot be evaluated fails rather than abstaining', () => {
    const t = freshTools();
    // An SDF-only model (a smooth blend) has no B-Rep, so `brep_sound` verified
    // nothing — and must not report a pass for it.
    const blended = callJson(t, 'create_model', {
      script: 'return Shape.sphere(5).smoothUnion(Shape.box3(4, 4, 4), 1);',
    });
    const out = callJson(t, 'assert_model', {
      model_id: blended.model_id,
      accuracy: 0.3,
      expect: [{ type: 'brep_sound' }],
    });
    assert.equal(out.ok, false);
    assert.match(out.checks[0].message, /no B-Rep to check/);
  });

  test('an exact model passes brep_sound', () => {
    const t = freshTools();
    const model = callJson(t, 'create_model', { script: RIGHT_AXIS_SCRIPT, exact: true });
    const out = callJson(t, 'assert_model', {
      model_id: model.model_id,
      accuracy: 0.3,
      expect: [{ type: 'brep_sound' }],
    });
    assert.equal(out.ok, true, out.checks[0].message);
  });

  test('hole_at needs a clear axis AND enclosing material, not either alone', () => {
    const t = freshTools();
    const model = callJson(t, 'create_model', { script: RIGHT_AXIS_SCRIPT });
    const at = (spec) =>
      callJson(t, 'assert_model', { model_id: model.model_id, accuracy: 0.2, expect: [spec] })
        .checks[0];

    // The real bore: clear along +Y, enclosed across it.
    assert.equal(at({ type: 'hole_at', at: [0, 0, 0], axis: [0, 1, 0] }).ok, true);

    // Free space beside the part is also "clear along +Y" — and must not pass,
    // because nothing encloses it. This is the check that stops the assertion
    // from being satisfied by pointing it at thin air.
    const air = at({ type: 'hole_at', at: [100, 0, 0], axis: [0, 1, 0] });
    assert.equal(air.ok, false);
    assert.match(air.message, /open space beside the part/);

    // Naming an axis the bore does not run along fails on the material it hits.
    const sideways = at({ type: 'hole_at', at: [0, 0, 0], axis: [0, 0, 1] });
    assert.equal(sideways.ok, false);
    assert.match(sideways.message, /crosses .* of material/);

    // A point off the centreline reads narrower than the diameter.
    const offCentre = at({ type: 'hole_at', at: [2, 0, 0], axis: [0, 1, 0], diameter: 5, tolerance: 0.1 });
    assert.equal(offCentre.ok, false);
    assert.match(offCentre.message, /off the bore centreline/);
  });

  test('a clearance violation reports the offending probe', () => {
    const t = freshTools();
    const model = callJson(t, 'create_model', { script: PLATE_SCRIPT });
    const out = callJson(t, 'assert_model', {
      model_id: model.model_id,
      expect: [{ type: 'clearance', probes: [[0, 20, 0], [0, 0, 0]], min: 1 }],
    });
    assert.equal(out.ok, false);
    // The second probe is inside the plate: a negative clearance.
    assert.ok(out.checks[0].actual < 0, `actual ${out.checks[0].actual}`);
    assert.match(out.checks[0].message, /inside the material/);
  });

  test('unknown types and malformed expectations are reported, not ignored', () => {
    const t = freshTools();
    const model = callJson(t, 'create_model', { script: PLATE_SCRIPT });
    const out = callJson(t, 'assert_model', {
      model_id: model.model_id,
      expect: [{ type: 'perimeter', value: 1 }, { type: 'volume', value: 'lots' }],
    });
    assert.equal(out.failed, 2);
    assert.match(out.checks[0].message, /unknown assertion type 'perimeter'/);
    assert.match(out.checks[1].message, /'value' must be a finite number/);

    const empty = t.call('assert_model', { model_id: model.model_id, expect: [] });
    assert.equal(empty.isError, true);
    assert.match(empty.content[0].text, /non-empty array/);
  });

  test('a null component in a vector expectation skips that axis', () => {
    const t = freshTools();
    // Shift the plate in x; the centroid follows, and an expectation that only
    // pins y and z still passes.
    const model = callJson(t, 'create_model', {
      script: 'return Shape.box3(15, 2.5, 10).translate(7, 0, 0);',
    });
    const out = callJson(t, 'assert_model', {
      model_id: model.model_id,
      expect: [{ type: 'centroid', value: [null, 0, 0], tolerance: 0.05 }],
    });
    assert.equal(out.ok, true, out.checks[0].message);
  });
});

describe('the diff_models tool', () => {
  test('reports the volume the holes removed, and checks it against the spec', () => {
    const t = freshTools();
    const blank = callJson(t, 'create_model', { script: PLATE_SCRIPT, name: 'blank' });
    const drilled = callJson(t, 'create_model', { script: FOUR_HOLES_SCRIPT, name: 'drilled' });
    // Four Ø5 bores through a 5 mm plate: 4 · π · 2.5² · 5.
    const expected = -4 * Math.PI * 2.5 * 2.5 * 5;
    const out = callJson(t, 'diff_models', {
      model_id_a: blank.model_id,
      model_id_b: drilled.model_id,
      accuracy: 0.2,
      expect_volume_delta: { value: expected, relative_tolerance: 0.05 },
    });
    assert.ok(out.delta.volume < 0, 'drilling removes material');
    assert.equal(out.volumeDeltaCheck.ok, true, out.volumeDeltaCheck.message);
    // The structural half: four more handles in the surface.
    assert.equal(out.delta.counts.genus, 4);
    assert.equal(out.delta.counts.throughHoles, 4);
    assert.equal(out.delta.counts.shells, 0);
  });

  test('a wrong-axis hole fails a volume-delta check against the intended one', () => {
    const t = freshTools();
    const blank = callJson(t, 'create_model', { script: PLATE_SCRIPT });
    const wrong = callJson(t, 'create_model', { script: WRONG_AXIS_SCRIPT });
    // The volume a Ø5 bore through the 5 mm thickness should remove.
    const intended = -Math.PI * 2.5 * 2.5 * 5;
    const out = callJson(t, 'diff_models', {
      model_id_a: blank.model_id,
      model_id_b: wrong.model_id,
      accuracy: 0.2,
      expect_volume_delta: { value: intended, relative_tolerance: 0.1 },
    });
    // A hole along the plate's 20 mm dimension removes several times as much.
    assert.equal(out.volumeDeltaCheck.ok, false, JSON.stringify(out.volumeDeltaCheck));
  });

  test('reports both models and an unknown id cleanly', () => {
    const t = freshTools();
    const a = callJson(t, 'create_model', { script: PLATE_SCRIPT, name: 'a' });
    const out = callJson(t, 'diff_models', { model_id_a: a.model_id, model_id_b: a.model_id });
    assert.equal(out.delta.volume, 0, 'a model against itself has no delta');
    assert.equal(out.a.name, 'a');
    const bad = t.call('diff_models', { model_id_a: a.model_id, model_id_b: 'nope' });
    assert.equal(bad.isError, true);
    assert.match(bad.content[0].text, /unknown model_id/);
  });
});

describe('the measure_clearance tool', () => {
  test('probe distances are signed, and the softmin comes back with them', () => {
    const t = freshTools();
    const model = callJson(t, 'create_model', { script: 'return Shape.sphere(5);' });
    const out = callJson(t, 'measure_clearance', {
      model_id: model.model_id,
      probes: [[10, 0, 0], [0, 0, 0], [0, 7, 0]],
    });
    assert.equal(out.probes.count, 3);
    assert.ok(Math.abs(out.probes.distances[0] - 5) < 1e-6, 'a point 10 out from r=5 is 5 clear');
    assert.ok(out.probes.distances[1] < 0, 'the centre is inside');
    assert.equal(out.probes.probesInsideMaterial, 1);
    assert.equal(out.probes.clear, false);
    assert.ok(Math.abs(out.probes.minDistance - -5) < 1e-6);
    assert.deepEqual(out.probes.nearestProbe, [0, 0, 0]);
    assert.ok(Number.isFinite(out.probes.softMin), 'the differentiable value is present too');
  });

  test('a flat probe array is accepted as well as a nested one', () => {
    const t = freshTools();
    const model = callJson(t, 'create_model', { script: 'return Shape.sphere(1);' });
    const nested = callJson(t, 'measure_clearance', {
      model_id: model.model_id,
      probes: [[3, 0, 0], [0, 4, 0]],
    });
    const flat = callJson(t, 'measure_clearance', {
      model_id: model.model_id,
      probes: [3, 0, 0, 0, 4, 0],
    });
    assert.deepEqual(flat.probes.distances, nested.probes.distances);
  });

  test('detects interference between two models, and clearance when there is none', () => {
    const t = freshTools();
    const block = callJson(t, 'create_model', { script: 'return Shape.box3(5, 5, 5);' });
    const overlapping = callJson(t, 'create_model', {
      script: 'return Shape.sphere(3).translate(6, 0, 0);',
    });
    const clear = callJson(t, 'create_model', {
      script: 'return Shape.sphere(3).translate(20, 0, 0);',
    });

    const hit = callJson(t, 'measure_clearance', {
      model_id: block.model_id,
      against_model_id: overlapping.model_id,
      accuracy: 0.1,
    });
    assert.equal(hit.interference.interferes, true);
    assert.ok(hit.interference.minDistance < 0, `min ${hit.interference.minDistance}`);
    assert.ok(hit.interference.verticesInsideMaterial > 0);

    const miss = callJson(t, 'measure_clearance', {
      model_id: block.model_id,
      against_model_id: clear.model_id,
      accuracy: 0.1,
    });
    assert.equal(miss.interference.interferes, false);
    // Sphere surface nearest point is at x=17, block face at x=5: 12 apart.
    assert.ok(Math.abs(miss.interference.minDistance - 12) < 0.2, `gap ${miss.interference.minDistance}`);
  });

  test('needs something to measure against, and rejects malformed probes', () => {
    const t = freshTools();
    const model = callJson(t, 'create_model', { script: 'return Shape.sphere(1);' });
    const bare = t.call('measure_clearance', { model_id: model.model_id });
    assert.equal(bare.isError, true);
    assert.match(bare.content[0].text, /probes.*against_model_id/);

    for (const probes of [[], [[1, 2]], [1, 2], [[1, 2, NaN]]]) {
      const out = t.call('measure_clearance', { model_id: model.model_id, probes });
      assert.equal(out.isError, true, `probes ${JSON.stringify(probes)} should be rejected`);
    }
  });

  test('flattenProbes normalizes both shapes and rejects the rest', () => {
    assert.deepEqual([...flattenProbes([[1, 2, 3], [4, 5, 6]])], [1, 2, 3, 4, 5, 6]);
    assert.deepEqual([...flattenProbes([1, 2, 3])], [1, 2, 3]);
    assert.throws(() => flattenProbes([]), /non-empty/);
    assert.throws(() => flattenProbes([1, 2]), /whole number of points/);
    assert.throws(() => flattenProbes([[1, 2]]), /must be \[x,y,z\]/);
    assert.throws(() => flattenProbes([1, 2, Infinity]), /non-finite/);
  });
});

describe('validate surfaces the B-Rep check', () => {
  test('an exact model reports a checked, sound body', () => {
    const t = freshTools();
    const model = callJson(t, 'create_model', { script: RIGHT_AXIS_SCRIPT, exact: true });
    const out = callJson(t, 'validate', { model_id: model.model_id, accuracy: 0.3 });
    assert.equal(out.valid, true);
    assert.equal(out.brep.available, true);
    assert.deepEqual(out.brep.failures, []);
    assert.equal(out.brep.selfIntersectionChecked, false);
    assert.ok(out.brep.counts.faces > 0);
  });

  test('deep:true runs the self-intersection pass and says it did', () => {
    const t = freshTools();
    const model = callJson(t, 'create_model', { script: RIGHT_AXIS_SCRIPT, exact: true });
    const out = callJson(t, 'validate', { model_id: model.model_id, accuracy: 0.3, deep: true });
    assert.equal(out.brep.selfIntersectionChecked, true);
    assert.deepEqual(out.brep.failures, []);
    assert.equal(out.valid, true);
  });

  test('an SDF-only model says why nothing was checked, and stays valid', () => {
    const t = freshTools();
    const model = callJson(t, 'create_model', {
      script: 'return Shape.sphere(5).smoothUnion(Shape.box3(4, 4, 4), 1);',
    });
    const out = callJson(t, 'validate', { model_id: model.model_id, accuracy: 0.3 });
    assert.equal(out.valid, true, 'the mesh oracle still applies');
    assert.equal(out.brep.available, false);
    assert.match(out.brep.reason, /outside exact coverage/);
    assert.equal(out.mesher, 'adaptive-sdf');
  });

  test('create_model discloses which mesher answered and whether a B-Rep was checked', () => {
    const t = freshTools();
    const sdf = callJson(t, 'create_model', { script: PLATE_SCRIPT });
    assert.equal(sdf.mesher, 'adaptive-sdf');
    // A bare primitive carries an exact companion even with the boolean mode off.
    assert.equal(sdf.brepChecked, true);
    const blended = callJson(t, 'create_model', {
      script: 'return Shape.sphere(5).smoothUnion(Shape.box3(4, 4, 4), 1);',
    });
    assert.equal(blended.brepChecked, false);
  });
});

/** Mesh a shape and hand back `[positions, indices]` for the pure functions. */
function meshArrays(shape, accuracy) {
  const mesh = getMesh(shape, { accuracy });
  return [mesh.positions, mesh.indices];
}
