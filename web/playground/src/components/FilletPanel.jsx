/**
 * Parameter panel for a pending edge fillet/chamfer (of-rpo).
 *
 * The tool has two states, both driven by the same `fillet` descriptor:
 *   - **armed** (`fillet.armed`): waiting for the user to click an edge. The
 *     panel shows the pick prompt and a mode toggle only.
 *   - **picked**: an edge has been resolved to a union node in the tree. The
 *     panel shows the radius control and Apply; live preview and script
 *     serialization both flow from this descriptor (see edgeFillet.js), so
 *     what the panel shows is what the tree commits.
 *
 * Fillet rounds the edge (radius = fillet radius); chamfer bevels it (radius =
 * setback). The kernel blend is windowed to the picked crease polyline, so
 * every other edge stays sharp.
 */
export default function FilletPanel({ fillet, error, onMode, onRadius, onApply, onCancel }) {
  if (!fillet) return null;
  const mode = fillet.mode ?? 'fillet';
  const armed = Boolean(fillet.armed);
  const range = fillet.range || 1;
  const radius = fillet.radius ?? range / 10;

  const commit = (raw) => {
    const value = Number(raw);
    if (!Number.isFinite(value) || value <= 0) return;
    onRadius(value);
  };

  return (
    <div className="fillet-panel">
      <div className="fillet-title">
        Edge Blend
        <span className="fillet-hint">
          {armed ? 'Click an edge where two bodies meet' : 'Adjust the radius, then Apply'}
        </span>
      </div>

      <div className="fillet-modes" role="radiogroup" aria-label="Blend mode">
        {['fillet', 'chamfer'].map((m) => (
          <button
            key={m}
            type="button"
            role="radio"
            aria-checked={mode === m}
            className={mode === m ? 'fillet-mode-active' : 'secondary'}
            onClick={() => onMode(m)}
          >
            {m === 'fillet' ? 'Fillet' : 'Chamfer'}
          </button>
        ))}
      </div>

      {!armed && (
        <label className="fillet-field">
          {mode === 'fillet' ? 'Radius' : 'Setback'}
          <input
            type="range"
            min={range / 100}
            max={range}
            step={range / 100}
            value={Math.min(radius, range)}
            onChange={(event) => commit(event.target.value)}
          />
          <input
            className="fillet-value"
            type="number"
            min="0"
            step="any"
            value={radius}
            onChange={(event) => commit(event.target.value)}
          />
        </label>
      )}

      {error && <div className="fillet-error">{error}</div>}
      <div className="fillet-actions">
        {!armed && (
          <button onClick={onApply} disabled={Boolean(error)}>
            Apply
          </button>
        )}
        <button className="secondary" onClick={onCancel}>
          Cancel
        </button>
      </div>
    </div>
  );
}
