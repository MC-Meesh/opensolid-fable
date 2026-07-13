/**
 * Run / STL / STEP buttons plus meshing controls. Display toggles
 * (wireframe, views) live in the viewport's MainToolbar.
 *
 * Meshing accuracy is fixed at a high-precision default (see App's
 * MESH_ACCURACY) — there is deliberately no slider; scripts that need a
 * different target can call the wasm `meshAdaptive(accuracy)` API.
 *
 * The exact-booleans toggle routes sharp booleans through the kernel's
 * exact B-Rep pipeline (validated, F-Rep fallback); flipping it re-runs
 * the script, so it reports through `onExactBooleansChange`.
 */
export default function Toolbar({
  exactBooleans,
  onExactBooleansChange,
  onRun,
  onDownloadStl,
  onDownloadStep,
  disabled,
}) {
  return (
    <div className="toolbar">
      <button onClick={onRun} disabled={disabled}>
        Run
      </button>
      <button className="secondary" onClick={onDownloadStl} disabled={disabled}>
        Download STL
      </button>
      <button
        className="secondary"
        onClick={onDownloadStep}
        disabled={disabled}
        title="Export STEP (AP203). Exact B-Rep models export analytic surfaces; organic shapes export as faceted geometry."
      >
        Download STEP
      </button>
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
