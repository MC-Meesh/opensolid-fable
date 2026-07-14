/**
 * Parameter panel for a pending edge fillet/chamfer (of-rpo).
 *
 * The tool starts with no edge selected; the panel prompts the user to click a
 * feature edge in the viewport (lib/edgePick.js resolves the click to a crease
 * polyline). Once an edge is picked, a mode toggle (Fillet rounds / Chamfer
 * bevels) and a radius control drive a live preview — both preview and the
 * committed script flow from the same descriptor (lib/fillet.js), so what the
 * panel shows is what the tree commits.
 */
export default function FilletPanel({ fillet, error, onChange, onField, onApply, onCancel }) {
  if (!fillet) return null;
  const mode = fillet.mode ?? 'fillet';
  const radius = fillet.radius ?? 0.1;
  const hasEdge = Array.isArray(fillet.edge) && fillet.edge.length >= 6;
  const max = fillet.range ?? 1;

  const commit = (raw) => {
    const value = Number(raw);
    if (!Number.isFinite(value) || value <= 0) return;
    onChange(value);
  };

  return (
    <div className="sweep-panel fillet-panel">
      <div className="sweep-title">
        Edge blend
        <span className="sweep-plane">{mode === 'chamfer' ? 'chamfer' : 'fillet'}</span>
      </div>

      <div className="sweep-modes" role="radiogroup" aria-label="Blend mode">
        {['fillet', 'chamfer'].map((m) => (
          <button
            key={m}
            type="button"
            role="radio"
            aria-checked={mode === m}
            className={mode === m ? 'sweep-mode-active' : 'secondary'}
            onClick={() => onField({ mode: m })}
          >
            {m === 'fillet' ? 'Fillet' : 'Chamfer'}
          </button>
        ))}
      </div>

      <div className={hasEdge ? 'sweep-target' : 'sweep-target sweep-target-empty'}>
        {hasEdge
          ? `Edge selected (${fillet.segments ?? 0} segment${fillet.segments === 1 ? '' : 's'})`
          : 'Click a feature edge to blend'}
      </div>

      <label className="sweep-field">
        {mode === 'chamfer' ? 'Setback' : 'Radius'}
        <input
          type="range"
          min={max / 100}
          max={max}
          step={max / 100}
          value={Math.min(radius, max)}
          disabled={!hasEdge}
          onChange={(event) => commit(event.target.value)}
        />
        <input
          className="sweep-value"
          type="number"
          min="0"
          step="any"
          value={radius}
          disabled={!hasEdge}
          onChange={(event) => commit(event.target.value)}
        />
      </label>

      {error && <div className="sweep-error">{error}</div>}
      <div className="sweep-actions">
        <button onClick={onApply} disabled={!hasEdge || Boolean(error)}>
          Apply
        </button>
        <button className="secondary" onClick={onCancel}>
          Cancel
        </button>
      </div>
    </div>
  );
}
