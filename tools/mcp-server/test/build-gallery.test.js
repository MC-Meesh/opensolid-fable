// Unit tests for the gallery generator's orchestration helpers. These cover the
// failure-isolation and single-example manifest-merge behavior (of-18t) without
// driving the real kernel: driveExamples/mergeManifest are pure w.r.t. their
// inputs, so the tests inject synthetic examples and a fake renderOne.

import { test } from 'node:test';
import assert from 'node:assert/strict';
import { driveExamples, mergeManifest } from '../examples/agent-gallery/build-gallery.mjs';

// A synthetic example list; renderOne throws for whichever slug is in `boom`.
function fixture(boom = new Set()) {
  const examples = ['a', 'b', 'c'].map((slug) => ({
    spec: { slug, title: `title ${slug}` },
    drive: () => {},
  }));
  const renderOne = (spec) => {
    if (boom.has(spec.slug)) throw new Error(`kaboom ${spec.slug}`);
    return {
      manifestEntry: { slug: spec.slug, ok: true },
      indexRow: `| ${spec.title} | ${spec.slug} |`,
    };
  };
  return { examples, renderOne };
}

test('driveExamples renders every example when none throw', () => {
  const { examples, renderOne } = fixture();
  const { manifest, indexRows, failures } = driveExamples(examples, { renderOne });
  assert.deepEqual(
    manifest.map((e) => e.slug),
    ['a', 'b', 'c'],
  );
  assert.equal(indexRows.length, 3);
  assert.equal(failures.length, 0);
});

test('driveExamples isolates a throwing example — the others still render', () => {
  // The heart of of-18t: one example returning a null volume (and its narration
  // then throwing on `.toFixed`) must not strand the rest of the run.
  const { examples, renderOne } = fixture(new Set(['b']));
  const { manifest, indexRows, failures } = driveExamples(examples, { renderOne });
  assert.deepEqual(
    manifest.map((e) => e.slug),
    ['a', 'c'],
    'the surviving examples still produce manifest entries',
  );
  assert.equal(indexRows.length, 2, 'the broken example contributes no index row');
  assert.deepEqual(failures, [{ slug: 'b', message: 'kaboom b' }]);
});

test('driveExamples records every failure and still finishes the run', () => {
  const { examples, renderOne } = fixture(new Set(['a', 'c']));
  const { manifest, indexRows, failures } = driveExamples(examples, { renderOne });
  assert.deepEqual(
    manifest.map((e) => e.slug),
    ['b'],
  );
  assert.equal(indexRows.length, 1);
  assert.deepEqual(
    failures.map((f) => f.slug),
    ['a', 'c'],
  );
});

test('driveExamples honors the `only` filter', () => {
  const { examples, renderOne } = fixture();
  const { manifest, failures } = driveExamples(examples, { only: 'b', renderOne });
  assert.deepEqual(
    manifest.map((e) => e.slug),
    ['b'],
  );
  assert.equal(failures.length, 0);
});

test('mergeManifest replaces a matching slug in place and preserves order', () => {
  const existing = [
    { slug: 'a', v: 1 },
    { slug: 'b', v: 1 },
    { slug: 'c', v: 1 },
  ];
  const merged = mergeManifest(existing, [{ slug: 'b', v: 2 }]);
  assert.deepEqual(merged, [
    { slug: 'a', v: 1 },
    { slug: 'b', v: 2 },
    { slug: 'c', v: 1 },
  ]);
  assert.notEqual(merged, existing, 'does not mutate the caller array');
  assert.equal(existing[1].v, 1, 'the original entry is untouched');
});

test('mergeManifest appends an entry whose slug is not yet present', () => {
  const merged = mergeManifest([{ slug: 'a', v: 1 }], [{ slug: 'z', v: 9 }]);
  assert.deepEqual(merged, [
    { slug: 'a', v: 1 },
    { slug: 'z', v: 9 },
  ]);
});

test('mergeManifest tolerates a missing/non-array existing manifest', () => {
  assert.deepEqual(mergeManifest(undefined, [{ slug: 'a' }]), [{ slug: 'a' }]);
  assert.deepEqual(mergeManifest(null, [{ slug: 'a' }]), [{ slug: 'a' }]);
});
