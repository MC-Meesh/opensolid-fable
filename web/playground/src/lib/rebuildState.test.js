import { describe, expect, it } from 'vitest';
import { computeRebuildState, rebuildStateTitle } from './rebuildState.js';

/** Minimal feature list like buildFeatures emits. */
const features = [
  { key: 'box:1', id: 1, kind: 'primitive', depth: 0 },
  { key: 'extrude:1', id: 3, kind: 'sweep', depth: 0 },
  { key: 'sketch:1', id: 3, kind: 'sketch', depth: 1, parentKey: 'extrude:1' },
  { key: 'union:1', id: 4, kind: 'boolean', depth: 0 },
];

describe('computeRebuildState', () => {
  it('marks every feature ok when nothing is flagged', () => {
    const state = computeRebuildState(features);
    for (const f of features) expect(state.get(f.key)).toEqual({ status: 'ok' });
  });

  it('flags a feature whose reference is dangling', () => {
    const refs = new Map([['extrude:1', { status: 'dangling', reason: 'nearest face too far' }]]);
    const state = computeRebuildState(features, refs);
    expect(state.get('extrude:1')).toEqual({ status: 'dangling', reason: 'nearest face too far' });
    expect(state.get('box:1')).toEqual({ status: 'ok' });
  });

  it('propagates a sweep dangling status onto its nested sketch row', () => {
    const refs = new Map([['extrude:1', { status: 'dangling', reason: 'gone' }]]);
    const state = computeRebuildState(features, refs);
    expect(state.get('sketch:1').status).toBe('dangling');
  });

  it('an ok reference does not override an ok feature', () => {
    const refs = new Map([['extrude:1', { status: 'ok' }]]);
    expect(computeRebuildState(features, refs).get('extrude:1')).toEqual({ status: 'ok' });
  });

  it('error outranks dangling on the same feature', () => {
    const refs = new Map([['extrude:1', { status: 'dangling', reason: 'gone' }]]);
    const state = computeRebuildState(features, refs, ['extrude:1']);
    expect(state.get('extrude:1').status).toBe('error');
  });

  it('marks a feature listed in errorKeys', () => {
    const state = computeRebuildState(features, new Map(), ['union:1']);
    expect(state.get('union:1').status).toBe('error');
  });
});

describe('rebuildStateTitle', () => {
  it('is empty for ok and undefined', () => {
    expect(rebuildStateTitle({ status: 'ok' })).toBe('');
    expect(rebuildStateTitle(undefined)).toBe('');
  });

  it('describes dangling and error with their reason', () => {
    expect(rebuildStateTitle({ status: 'dangling', reason: 'face gone' })).toMatch(/Dangling.*face gone/);
    expect(rebuildStateTitle({ status: 'error', reason: 'nan' })).toMatch(/Rebuild error.*nan/);
  });

  it('falls back to a default reason when none is given', () => {
    expect(rebuildStateTitle({ status: 'dangling' })).toMatch(/no longer exists/);
  });
});
