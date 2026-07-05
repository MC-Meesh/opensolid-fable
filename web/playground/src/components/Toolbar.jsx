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
 */
export default function Toolbar({
  resolution,
  onResolutionChange,
  onResolutionCommit,
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
    </div>
  );
}
