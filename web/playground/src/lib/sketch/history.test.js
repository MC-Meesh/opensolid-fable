import { describe, expect, it } from 'vitest';
import {
  canRedo,
  canUndo,
  createHistory,
  record,
  redoTo,
  snapshot,
  undoTo,
} from './history.js';
import { addLine, addPoint, createSketch } from './model.js';

describe('sketch history', () => {
  it('starts empty', () => {
    const h = createHistory();
    expect(canUndo(h)).toBe(false);
    expect(canRedo(h)).toBe(false);
    expect(undoTo(h, createSketch())).toBeNull();
    expect(redoTo(h, createSketch())).toBeNull();
  });

  it('snapshot deep-clones the sketch', () => {
    const s = createSketch();
    const pid = addPoint(s, 1, 2);
    const snap = snapshot(s);
    s.points[pid].x = 99;
    expect(snap.points[pid].x).toBe(1);
  });

  it('undo restores the pre-mutation state and redo reapplies', () => {
    const h = createHistory();
    let sketch = createSketch();

    const before = snapshot(sketch);
    const a = addPoint(sketch, 0, 0);
    const b = addPoint(sketch, 5, 0);
    addLine(sketch, a, b);
    record(h, before);

    expect(canUndo(h)).toBe(true);
    const undone = undoTo(h, sketch);
    expect(Object.keys(undone.points)).toHaveLength(0);
    expect(canRedo(h)).toBe(true);

    const redone = redoTo(h, undone);
    expect(Object.keys(redone.points)).toHaveLength(2);
    expect(Object.keys(redone.entities)).toHaveLength(1);
  });

  it('recording clears the redo stack', () => {
    const h = createHistory();
    let sketch = createSketch();

    const before1 = snapshot(sketch);
    addPoint(sketch, 0, 0);
    record(h, before1);

    sketch = undoTo(h, sketch);
    expect(canRedo(h)).toBe(true);

    const before2 = snapshot(sketch);
    addPoint(sketch, 3, 3);
    record(h, before2);
    expect(canRedo(h)).toBe(false);
  });

  it('caps history depth at 100', () => {
    const h = createHistory();
    const sketch = createSketch();
    for (let i = 0; i < 150; i++) {
      const before = snapshot(sketch);
      addPoint(sketch, i, i);
      record(h, before);
    }
    expect(h.past.length).toBe(100);
    // The oldest surviving snapshot is from iteration 50.
    expect(Object.keys(h.past[0].points)).toHaveLength(50);
  });

  it('supports multiple undo/redo round trips', () => {
    const h = createHistory();
    let sketch = createSketch();
    const counts = [0];
    for (let i = 1; i <= 3; i++) {
      const before = snapshot(sketch);
      addPoint(sketch, i, 0);
      record(h, before);
      counts.push(i);
    }
    for (let i = 2; i >= 0; i--) {
      sketch = undoTo(h, sketch);
      expect(Object.keys(sketch.points)).toHaveLength(counts[i]);
    }
    for (let i = 1; i <= 3; i++) {
      sketch = redoTo(h, sketch);
      expect(Object.keys(sketch.points)).toHaveLength(counts[i]);
    }
  });
});
