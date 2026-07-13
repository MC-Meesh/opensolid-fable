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

// The accuracy slider is logarithmic: equal steps feel like equal quality
// changes across the 0.002..0.05 model-unit range (finer to the left, like
// a resolution slider's "more detail" direction reversed onto accuracy).
const LOG_MIN = Math.log10(0.002);
const LOG_MAX = Math.log10(0.05);
const SLIDER_STEPS = 100;

function accuracyToSlider(accuracy) {
  const t = (Math.log10(accuracy) - LOG_MIN) / (LOG_MAX - LOG_MIN);
  return Math.round((1 - Math.min(Math.max(t, 0), 1)) * SLIDER_STEPS);
}

function sliderToAccuracy(value) {
  const t = 1 - value / SLIDER_STEPS;
  return 10 ** (LOG_MIN + t * (LOG_MAX - LOG_MIN));
}

/**
 * Run / STL buttons plus meshing controls. Display toggles (wireframe,
 * views) live in the viewport's MainToolbar.
 *
 * The accuracy slider sets the adaptive mesher's target: maximum chordal
 * deviation from the exact surface, in model units. It reports every
 * movement through `onAccuracyChange` (live label update) but only fires
 * `onAccuracyCommit` when the drag or keyboard adjustment finishes, since
 * committing triggers a remesh.
 *
 * The exact-booleans toggle routes sharp booleans through the kernel's
 * exact B-Rep pipeline (validated, F-Rep fallback); flipping it re-runs
 * the script, so it reports through `onExactBooleansChange`.
 */
export default function Toolbar({
  accuracy,
  onAccuracyChange,
  onAccuracyCommit,
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
        Accuracy
        <input
          type="range"
          min="0"
          max={SLIDER_STEPS}
          step="1"
          value={accuracyToSlider(accuracy)}
          disabled={disabled}
          onChange={(event) => onAccuracyChange(sliderToAccuracy(Number(event.target.value)))}
          onPointerUp={onAccuracyCommit}
          onKeyUp={(event) => {
            if (COMMIT_KEYS.has(event.key)) onAccuracyCommit();
          }}
        />
        <span className="resolution-value">{accuracy.toPrecision(2)}</span>
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
