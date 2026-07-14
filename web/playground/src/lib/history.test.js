import { describe, expect, it } from 'vitest';
import {
  MAX_HISTORY,
  canRedo,
  canUndo,
  commit,
  createHistory,
  depth,
  redo,
  undo,
} from './history.js';

describe('feature history', () => {
  it('starts at the initial snapshot with nothing to undo or redo', () => {
    const h = createHistory('v0');
    expect(h.current).toBe('v0');
    expect(canUndo(h)).toBe(false);
    expect(canRedo(h)).toBe(false);
    expect(undo(h)).toBeNull();
    expect(redo(h)).toBeNull();
    expect(depth(h)).toEqual({ undo: 0, redo: 0 });
  });

  it('records commits and undoes to the previous snapshot', () => {
    const h = createHistory('v0');
    expect(commit(h, 'v1')).toBe(true);
    expect(commit(h, 'v2')).toBe(true);
    expect(h.current).toBe('v2');
    expect(depth(h)).toEqual({ undo: 2, redo: 0 });

    expect(undo(h)).toBe('v1');
    expect(h.current).toBe('v1');
    expect(canRedo(h)).toBe(true);
    expect(undo(h)).toBe('v0');
    expect(h.current).toBe('v0');
    expect(canUndo(h)).toBe(false);
    expect(undo(h)).toBeNull();
  });

  it('redo reapplies an undone commit', () => {
    const h = createHistory('v0');
    commit(h, 'v1');
    undo(h);
    expect(h.current).toBe('v0');
    expect(redo(h)).toBe('v1');
    expect(h.current).toBe('v1');
    expect(canRedo(h)).toBe(false);
  });

  it('walks a full undo/redo round trip preserving order', () => {
    const h = createHistory('v0');
    for (const v of ['v1', 'v2', 'v3']) commit(h, v);
    expect(undo(h)).toBe('v2');
    expect(undo(h)).toBe('v1');
    expect(redo(h)).toBe('v2');
    expect(redo(h)).toBe('v3');
    expect(depth(h)).toEqual({ undo: 3, redo: 0 });
  });

  it('a no-op commit (unchanged snapshot) does not record history', () => {
    const h = createHistory('v0');
    expect(commit(h, 'v0')).toBe(false);
    expect(canUndo(h)).toBe(false);
    commit(h, 'v1');
    expect(commit(h, 'v1')).toBe(false); // committing the same value twice
    expect(depth(h)).toEqual({ undo: 1, redo: 0 });
  });

  it('a new commit after undo clears the redo stack (branch discard)', () => {
    const h = createHistory('v0');
    commit(h, 'v1');
    commit(h, 'v2');
    undo(h); // current v1, redo has v2
    expect(canRedo(h)).toBe(true);
    commit(h, 'v3'); // diverge from v1
    expect(canRedo(h)).toBe(false);
    expect(h.current).toBe('v3');
    expect(undo(h)).toBe('v1');
  });

  it('caps the undo stack at MAX_HISTORY, dropping the oldest', () => {
    const h = createHistory('v0');
    for (let i = 1; i <= MAX_HISTORY + 25; i += 1) commit(h, `v${i}`);
    expect(h.past.length).toBe(MAX_HISTORY);
    // The current snapshot is the latest; the oldest survivor is not v0.
    expect(h.current).toBe(`v${MAX_HISTORY + 25}`);
    expect(h.past).not.toContain('v0');
  });
});
