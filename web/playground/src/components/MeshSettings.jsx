/**
 * Meshing controls, shown in the main toolbar's overflow menu (they are
 * set-and-forget, so they don't earn a permanent toolbar slot).
 *
 * Meshing accuracy is fixed at a high-precision default (see App's
 * MESH_ACCURACY) — there is deliberately no slider; scripts that need a
 * different target can call the wasm `meshAdaptive(accuracy)` API.
 *
 * The exact-booleans toggle routes sharp booleans through the kernel's
 * exact B-Rep pipeline (validated, F-Rep fallback); flipping it re-runs
 * the script, so it reports through `onExactBooleansChange`.
 */
export default function MeshSettings({ exactBooleans, onExactBooleansChange, disabled }) {
  return (
    <div className="mesh-settings">
      <label className="exact-booleans" title="Sharp booleans of spheres, boxes, cylinders and tori use the kernel's exact B-Rep pipeline; anything it can't prove correct falls back to SDF meshing.">
        <input
          type="checkbox"
          checked={exactBooleans}
          disabled={disabled}
          onChange={(event) => onExactBooleansChange(event.target.checked)}
        />
        Exact booleans
      </label>
    </div>
  );
}
