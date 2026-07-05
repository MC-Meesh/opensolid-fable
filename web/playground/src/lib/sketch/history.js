/**
 * Undo/redo history over sketch snapshots.
 *
 * The sketch model is plain serializable data, so snapshots are deep clones.
 * The convention: take a snapshot of the sketch BEFORE mutating it, then call
 * `record` with that snapshot once the mutation commits. `undoTo`/`redoTo`
 * exchange the current sketch for the adjacent snapshot and return it (the
 * caller swaps its live sketch reference).
 */

const MAX_DEPTH = 100;

export function createHistory() {
  return { past: [], future: [] };
}

/** Deep-clone a sketch (plain data only). */
export function snapshot(sketch) {
  return JSON.parse(JSON.stringify(sketch));
}

/**
 * Record a committed mutation: `before` is the snapshot taken before the
 * mutation. Clears the redo stack and caps history depth.
 */
export function record(history, before) {
  history.past.push(before);
  if (history.past.length > MAX_DEPTH) history.past.shift();
  history.future = [];
}

export function canUndo(history) {
  return history.past.length > 0;
}

export function canRedo(history) {
  return history.future.length > 0;
}

/**
 * Undo: push the current sketch onto the redo stack and return the previous
 * snapshot, or null when there is nothing to undo.
 */
export function undoTo(history, current) {
  if (!canUndo(history)) return null;
  history.future.push(snapshot(current));
  return history.past.pop();
}

/**
 * Redo: push the current sketch onto the undo stack and return the next
 * snapshot, or null when there is nothing to redo.
 */
export function redoTo(history, current) {
  if (!canRedo(history)) return null;
  history.past.push(snapshot(current));
  return history.future.pop();
}
