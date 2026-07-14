// Rebuild state (of-fsl.8): the per-feature status the feature tree paints as
// a badge after a parametric rebuild. Folds three inputs into one map keyed by
// feature key:
//
//   ok       — evaluated cleanly, any persistent reference resolved
//   dangling — a persistent reference could not be re-resolved (face gone)
//   error    — the feature itself failed to evaluate
//
// `error` is modeled for completeness; today evaluation fails whole-script
// (the error banner covers it), so only ok/dangling are reachable until
// per-feature error isolation lands. See docs/parametric-rebuild.md.
//
// Pure — no React/WASM — so the badge logic is unit-tested directly.

/** Rank so a feature with more than one issue reports the most severe. */
const SEVERITY = { ok: 0, dangling: 1, error: 2 };

/**
 * Compute `Map<featureKey, { status, reason? }>` for a feature list.
 *
 * - `refStatuses` (`Map<featureKey, { status, reason? }>`) come from
 *   `resolveRefs`: a dangling entry flags the owning feature.
 * - `errorKeys` (iterable of feature keys) mark features whose own evaluation
 *   failed; these outrank a dangling reference.
 *
 * Features absent from both inputs are `ok`. A sketch feature inherits its
 * owning sweep's status (they share `id`) so the badge shows on the sketch row
 * too — the reference the user picked lives on the sketch.
 */
export function computeRebuildState(features, refStatuses = new Map(), errorKeys = []) {
  const errors = new Set(errorKeys);
  const state = new Map();

  const worst = (a, b) => (SEVERITY[b.status] > SEVERITY[a.status] ? b : a);
  const put = (key, entry) => {
    const prev = state.get(key);
    state.set(key, prev ? worst(prev, entry) : entry);
  };

  for (const f of features) {
    let entry = { status: 'ok' };
    const ref = refStatuses.get(f.key);
    if (ref && ref.status !== 'ok') entry = worst(entry, ref);
    if (errors.has(f.key)) entry = worst(entry, { status: 'error' });
    put(f.key, entry);

    // Propagate a sweep's status onto its nested sketch child (shared node id):
    // the reference belongs to the sketch the user placed on the face.
    if (f.parentKey) {
      const parent = features.find((p) => p.key === f.parentKey);
      if (parent) {
        const parentRef = refStatuses.get(parent.key);
        if (parentRef && parentRef.status !== 'ok') put(f.key, parentRef);
        if (errors.has(parent.key)) put(f.key, { status: 'error' });
      }
    }
  }
  return state;
}

/** Human tooltip for a rebuild-state entry, or '' for ok/unknown. */
export function rebuildStateTitle(entry) {
  if (!entry || entry.status === 'ok') return '';
  if (entry.status === 'dangling') {
    return `Dangling reference: ${entry.reason ?? 'the referenced face no longer exists'}`;
  }
  if (entry.status === 'error') {
    return `Rebuild error: ${entry.reason ?? 'this feature failed to evaluate'}`;
  }
  return '';
}
