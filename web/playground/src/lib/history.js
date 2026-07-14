/**
 * Feature-level (session-wide) undo/redo over the construction tree.
 *
 * The script text is the single source of truth for the model — it fully
 * serializes the construction tree (App.jsx commitScript), so a script string
 * IS a construction-tree snapshot. This history is therefore a plain linear
 * stack of immutable script strings: no cloning is needed (contrast
 * src/lib/sketch/history.js, which deep-clones mutable sketch objects).
 *
 * Convention: the history owns the current committed snapshot. Every store
 * commit calls `commit`, which pushes the previous snapshot onto the undo
 * stack (a no-op when the snapshot is unchanged, so redundant re-evaluations
 * — e.g. the boot commit, or toggling exact booleans — never create history).
 * `undo`/`redo` swap the current snapshot for an adjacent one and return it.
 */

// Cap the undo depth so a long editing session can't grow the stacks without
// bound. Matches the sketch history depth for a consistent feel.
export const MAX_HISTORY = 100;

/** Create a history rooted at `current` (the initial committed snapshot). */
export function createHistory(current) {
  return { current, past: [], future: [] };
}

/**
 * Record a newly committed snapshot. Pushes the previous snapshot onto the
 * undo stack, clears the redo stack, and caps depth. No-ops (returns false)
 * when the snapshot is identical to the current one, so redundant commits
 * don't pollute history.
 */
export function commit(history, next) {
  if (next === history.current) return false;
  history.past.push(history.current);
  if (history.past.length > MAX_HISTORY) history.past.shift();
  history.future = [];
  history.current = next;
  return true;
}

export function canUndo(history) {
  return history.past.length > 0;
}

export function canRedo(history) {
  return history.future.length > 0;
}

/**
 * Undo: push the current snapshot onto the redo stack and make the previous
 * snapshot current, returning it. Returns null when there is nothing to undo.
 */
export function undo(history) {
  if (!canUndo(history)) return null;
  history.future.push(history.current);
  history.current = history.past.pop();
  return history.current;
}

/**
 * Redo: push the current snapshot onto the undo stack and make the next
 * snapshot current, returning it. Returns null when there is nothing to redo.
 */
export function redo(history) {
  if (!canRedo(history)) return null;
  history.past.push(history.current);
  history.current = history.future.pop();
  return history.current;
}

/** Available undo/redo depth — surfaced in the UI (button counts/tooltips). */
export function depth(history) {
  return { undo: history.past.length, redo: history.future.length };
}
