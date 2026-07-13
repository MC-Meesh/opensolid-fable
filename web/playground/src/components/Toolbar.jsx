const COMMIT_KEYS = new Set([
  'ArrowLeft',
  'ArrowRight',
  'ArrowUp',
  'ArrowDown',
  'PageUp',
  'PageDown',
  'Home',
  'End',
]);

/**
 * Run / STL buttons plus meshing controls. Display toggles (wireframe,
 * views) live in the viewport's MainToolbar.
 *
 * The resolution slider reports every movement through `onResolutionChange`
 * (live label update) but only fires `onResolutionCommit` when the drag or
 * keyboard adjustment finishes, since committing triggers a remesh.
 *
 * The exact-booleans toggle routes sharp booleans through the kernel's
 * exact B-Rep pipeline (validated, F-Rep fallback); flipping it re-runs
 * the script, so it reports through `onExactBooleansChange`.
 */
export default function Toolbar({
  resolution,
  onResolutionChange,
  onResolutionCommit,
  exactBooleans,
  onExactBooleansChange,
  onRun,
  onDownloadStl,
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
      <label>
        Resolution
        <input
          type="range"
          min="32"
          max="128"
          step="8"
          value={resolution}
          disabled={disabled}
          onChange={(event) => onResolutionChange(Number(event.target.value))}
          onPointerUp={onResolutionCommit}
          onKeyUp={(event) => {
            if (COMMIT_KEYS.has(event.key)) onResolutionCommit();
          }}
        />
        <span className="resolution-value">{resolution}</span>
      </label>
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
