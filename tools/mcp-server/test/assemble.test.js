// End-to-end tests for the `assemble` tool (of-2y4.8): placement, mate
// solving, interference, aggregate mass properties, and the assembly_id
// behaving as a first-class model. Requires the built pkg (`npm run build`).

import { test } from 'node:test';
import assert from 'node:assert/strict';
import { mkdtempSync, existsSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { createTools } from '../src/tools.js';

function freshTools() {
  return createTools({ outputDir: mkdtempSync(join(tmpdir(), 'osmcp-asm-')) });
}

function jsonOf(result) {
  assert.equal(result.isError, undefined, `unexpected error: ${result.content?.[0]?.text}`);
  return JSON.parse(result.content[0].text);
}

/** A registered unit cube ([-0.5, 0.5]³) and its id. */
function cubeId(t, name) {
  return jsonOf(t.call('create_model', { script: 'return Shape.box3(0.5, 0.5, 0.5);', name }))
    .model_id;
}

test('assemble places instances without mates and reports no clash when apart', () => {
  const t = freshTools();
  const cube = cubeId(t, 'cube');
  const out = jsonOf(
    t.call('assemble', {
      instances: [
        { model_id: cube, fixed: true, name: 'base' },
        { model_id: cube, transform: { translation: [3, 0, 0] }, name: 'far' },
      ],
    }),
  );

  assert.match(out.assembly_id, /^model-\d+-[0-9a-f]{4}$/);
  assert.equal(out.solve.status, 'skipped');
  assert.equal(out.interference.interferes, false);
  assert.equal(out.interference.checkedPairs, 1);
  assert.deepEqual(out.interference.pairs, []);
  // Two placed unit cubes: volume ~2, bbox spans x in [-0.5, 3.5].
  assert.ok(Math.abs(out.volume - 2) < 0.05, `volume ${out.volume}`);
  assert.ok(Math.abs(out.boundingBox.min[0] + 0.5) < 0.05);
  assert.ok(Math.abs(out.boundingBox.max[0] - 3.5) < 0.05);
  assert.equal(out.valid, true);
  // The resolved transforms echo the placements.
  assert.deepEqual(out.instances[1].transform.translation, [3, 0, 0]);
  assert.equal(out.instances[0].name, 'base');
  assert.equal(out.instances[0].fixed, true);
});

test('a coincident mate seats a floating cube on the fixed one', () => {
  const t = freshTools();
  const cube = cubeId(t, 'cube');
  const out = jsonOf(
    t.call('assemble', {
      instances: [
        { model_id: cube, fixed: true, name: 'base' },
        { model_id: cube, transform: { translation: [0, 0, 5] }, name: 'lid' },
      ],
      mates: [
        {
          kind: 'coincident',
          a: { instance: 0, feature: { type: 'plane', point: [0, 0, 0.5], normal: [0, 0, 1] } },
          b: { instance: 1, feature: { type: 'plane', point: [0, 0, -0.5], normal: [0, 0, -1] } },
        },
      ],
    }),
  );

  assert.equal(out.solve.status, 'converged');
  assert.ok(out.solve.residualNorm < 1e-8);
  // One plane–plane mate: in-plane translation (2) + spin (1) remain free.
  assert.equal(out.solve.freeDof, 3);
  // The lid's center dropped from z=5 to z=1 (flush faces at z=0.5).
  const lid = out.instances[1].transform.translation;
  assert.ok(Math.abs(lid[2] - 1) < 1e-8, `lid z ${lid[2]}`);
  // Seated flush means no interference.
  assert.equal(out.interference.interferes, false);
});

test('overlapping instances are reported with their clash volume', () => {
  const t = freshTools();
  const cube = cubeId(t, 'cube');
  const out = jsonOf(
    t.call('assemble', {
      instances: [
        { model_id: cube, fixed: true, name: 'a' },
        { model_id: cube, transform: { translation: [0.5, 0, 0] }, name: 'b' },
      ],
    }),
  );
  assert.equal(out.interference.interferes, true);
  assert.equal(out.interference.pairs.length, 1);
  const pair = out.interference.pairs[0];
  assert.equal(pair.a, 0);
  assert.equal(pair.b, 1);
  assert.equal(pair.aName, 'a');
  assert.equal(pair.bName, 'b');
  // Half the cube overlaps: volume ~0.5.
  assert.ok(Math.abs(pair.volume - 0.5) < 0.05, `clash volume ${pair.volume}`);
});

test('mass properties aggregate density-weighted instances', () => {
  const t = freshTools();
  const cube = cubeId(t, 'cube');
  const out = jsonOf(
    t.call('assemble', {
      instances: [
        { model_id: cube, fixed: true, density: 1 },
        { model_id: cube, transform: { translation: [2, 0, 0] }, density: 3 },
      ],
    }),
  );
  const mp = out.massProperties;
  assert.ok(Math.abs(mp.volume - 2) < 0.05, `volume ${mp.volume}`);
  assert.ok(Math.abs(mp.mass - 4) < 0.1, `mass ${mp.mass}`);
  // Centroid pulled toward the dense instance: (1·0 + 3·2)/4 = 1.5.
  assert.ok(Math.abs(mp.centroid[0] - 1.5) < 0.05, `centroid x ${mp.centroid[0]}`);
  assert.equal(mp.massErrors.length, 0);
  assert.equal(mp.inertia.length, 3);
});

test('the assembly_id is a first-class model: measure, screenshot, export, list, get', () => {
  const t = freshTools();
  const cube = cubeId(t, 'cube');
  const out = jsonOf(
    t.call('assemble', {
      instances: [
        { model_id: cube, fixed: true },
        { model_id: cube, transform: { translation: [2, 0, 0] } },
      ],
      name: 'pair',
    }),
  );
  const id = out.assembly_id;

  const measured = jsonOf(t.call('measure', { model_id: id }));
  assert.ok(Math.abs(measured.volume - 2) < 0.05);

  const shot = t.call('get_screenshot', { model_id: id, width: 160, height: 120 });
  assert.equal(shot.isError, undefined, shot.content?.[0]?.text);
  assert.equal(shot.content[0].type, 'image');

  const exported = jsonOf(t.call('export', { model_id: id, format: 'stl' }));
  assert.ok(existsSync(exported.path));
  assert.ok(exported.bytes > 0);

  const listed = jsonOf(t.call('list_models', {})).models.find((m) => m.model_id === id);
  assert.equal(listed.source, 'assembly');
  assert.equal(listed.name, 'pair');

  // get_model returns the recipe: instances with resolved transforms + mates.
  const source = jsonOf(t.call('get_model', { model_id: id }));
  assert.equal(source.source, 'assembly');
  assert.equal(source.assembly.instances.length, 2);
  assert.equal(source.assembly.instances[0].model_id, cube);
  assert.deepEqual(source.assembly.instances[1].transform.translation, [2, 0, 0]);
  assert.deepEqual(source.assembly.mates, []);
  assert.equal(source.params.length, 0);
});

test('a concentric mate lines up a shaft with a bore axis', () => {
  const t = freshTools();
  const plate = jsonOf(
    t.call('create_model', {
      // 4×1×4 plate (thin in y) with a Ø1 bore through it. Cylinders run
      // along y in this kernel, so the bore pierces the y-thin slab.
      script:
        'return Shape.box3(2, 0.5, 2).subtract(Shape.cylinder(0.5, 1));',
      name: 'plate',
    }),
  ).model_id;
  const pin = jsonOf(
    t.call('create_model', { script: 'return Shape.cylinder(0.4, 2);', name: 'pin' }),
  ).model_id;

  const out = jsonOf(
    t.call('assemble', {
      instances: [
        { model_id: plate, fixed: true },
        { model_id: pin, transform: { translation: [5, 0, 3] } },
      ],
      mates: [
        {
          kind: 'concentric',
          a: { instance: 0, feature: { type: 'axis', point: [0, 0, 0], direction: [0, 1, 0] } },
          b: { instance: 1, feature: { type: 'axis', point: [0, 0, 0], direction: [0, 1, 0] } },
        },
      ],
    }),
  );
  assert.equal(out.solve.status, 'converged');
  // The pin's axis snapped onto the bore axis: x = z = 0.
  const p = out.instances[1].transform.translation;
  assert.ok(Math.hypot(p[0], p[2]) < 1e-6, `pin at [${p}]`);
  // A Ø0.8 pin in a Ø1 bore: no interference.
  assert.equal(out.interference.interferes, false);
});

test('conflicting mates report over_constrained instead of failing', () => {
  const t = freshTools();
  const cube = cubeId(t, 'cube');
  const mateAtZ = (z, sign) => ({
    kind: 'coincident',
    a: { instance: 0, feature: { type: 'plane', point: [0, 0, z], normal: [0, 0, 1] } },
    b: { instance: 1, feature: { type: 'plane', point: [0, 0, -0.5 * sign], normal: [0, 0, -sign] } },
  });
  const out = jsonOf(
    t.call('assemble', {
      instances: [
        { model_id: cube, fixed: true },
        { model_id: cube, transform: { translation: [0, 0, 3] } },
      ],
      // Two mates demanding the same face pair sit at two different planes.
      mates: [mateAtZ(0.5, 1), mateAtZ(2.5, 1)],
    }),
  );
  assert.equal(out.solve.status, 'over_constrained');
  assert.ok(out.solve.residualNorm > 1e-3);
});

test('assemble validates its inputs with actionable errors', () => {
  const t = freshTools();
  const cube = cubeId(t, 'cube');

  const empty = t.call('assemble', { instances: [] });
  assert.equal(empty.isError, true);
  assert.match(empty.content[0].text, /non-empty array/);

  const unknown = t.call('assemble', { instances: [{ model_id: 'model-999-beef' }] });
  assert.equal(unknown.isError, true);
  assert.match(unknown.content[0].text, /instances\[0\].*unknown model_id/s);

  const badAxis = t.call('assemble', {
    instances: [
      { model_id: cube, transform: { rotation: { axis: [0, 0, 0], angle_deg: 45 } } },
    ],
  });
  assert.equal(badAxis.isError, true);
  assert.match(badAxis.content[0].text, /rotation axis/);

  const badKind = t.call('assemble', {
    instances: [{ model_id: cube }, { model_id: cube }],
    mates: [
      {
        kind: 'weld',
        a: { instance: 0, feature: { type: 'plane', point: [0, 0, 0], normal: [0, 0, 1] } },
        b: { instance: 1, feature: { type: 'plane', point: [0, 0, 0], normal: [0, 0, 1] } },
      },
    ],
  });
  assert.equal(badKind.isError, true);
  assert.match(badKind.content[0].text, /kind/);

  const badFeature = t.call('assemble', {
    instances: [{ model_id: cube }, { model_id: cube }],
    mates: [
      {
        kind: 'concentric',
        a: { instance: 0, feature: { type: 'plane', point: [0, 0, 0], normal: [0, 0, 1] } },
        b: { instance: 1, feature: { type: 'plane', point: [0, 0, 0], normal: [0, 0, 1] } },
      },
    ],
  });
  assert.equal(badFeature.isError, true);
  assert.match(badFeature.content[0].text, /axis/);

  const outOfRange = t.call('assemble', {
    instances: [{ model_id: cube }],
    mates: [
      {
        kind: 'coincident',
        a: { instance: 0, feature: { type: 'plane', point: [0, 0, 0], normal: [0, 0, 1] } },
        b: { instance: 5, feature: { type: 'plane', point: [0, 0, 0], normal: [0, 0, 1] } },
      },
    ],
  });
  assert.equal(outOfRange.isError, true);
  assert.match(outOfRange.content[0].text, /instance 5/);

  const missingValue = t.call('assemble', {
    instances: [{ model_id: cube }, { model_id: cube }],
    mates: [
      {
        kind: 'distance',
        a: { instance: 0, feature: { type: 'plane', point: [0, 0, 0], normal: [0, 0, 1] } },
        b: { instance: 1, feature: { type: 'plane', point: [0, 0, 0], normal: [0, 0, 1] } },
      },
    ],
  });
  assert.equal(missingValue.isError, true);
  assert.match(missingValue.content[0].text, /value/);
});

test('solve: false leaves mated instances where they were placed', () => {
  const t = freshTools();
  const cube = cubeId(t, 'cube');
  const out = jsonOf(
    t.call('assemble', {
      instances: [
        { model_id: cube, fixed: true },
        { model_id: cube, transform: { translation: [0, 0, 5] } },
      ],
      mates: [
        {
          kind: 'coincident',
          a: { instance: 0, feature: { type: 'plane', point: [0, 0, 0.5], normal: [0, 0, 1] } },
          b: { instance: 1, feature: { type: 'plane', point: [0, 0, -0.5], normal: [0, 0, -1] } },
        },
      ],
      solve: false,
    }),
  );
  assert.equal(out.solve.status, 'skipped');
  assert.deepEqual(out.instances[1].transform.translation, [0, 0, 5]);
});

test('a rotated placement carries the geometry with it', () => {
  const t = freshTools();
  const slab = jsonOf(
    t.call('create_model', { script: 'return Shape.box3(0.5, 0.5, 2);', name: 'slab' }),
  ).model_id;
  const out = jsonOf(
    t.call('assemble', {
      instances: [
        {
          model_id: slab,
          transform: { rotation: { axis: [1, 0, 0], angle_deg: 90 } },
        },
      ],
    }),
  );
  // The 4-long z-extent now lies along y.
  assert.ok(Math.abs(out.boundingBox.size[1] - 4) < 0.1, `size ${out.boundingBox.size}`);
  assert.ok(out.boundingBox.size[2] < 1.5, `size ${out.boundingBox.size}`);
  assert.ok(Math.abs(out.instances[0].transform.rotationDeg - 90) < 1e-9);
});
