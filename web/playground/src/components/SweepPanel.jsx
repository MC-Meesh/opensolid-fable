import { planeLabel } from '../lib/sketch/profile.js';

/**
 * Parameter panel for a pending extrude/revolve: a live-updating value
 * (height or angle), Apply to commit the operation into the script/tree,
 * Cancel to return to the sketch.
 *
 * Extrude heights are signed — the slider and number edit the magnitude and
 * "Flip direction" carries the sign, so the solid can grow to either side
 * of the sketch plane (both directions of a face sketch, of-4eh.16).
 */
export default function SweepPanel({ sweep, error, onChange, onApply, onCancel }) {
  if (!sweep) return null;
  const isExtrude = sweep.kind === 'extrude';
  const label = isExtrude ? 'Height' : 'Angle';
  const min = isExtrude ? sweep.range / 100 : 1;
  const max = isExtrude ? sweep.range : 360;
  const step = isExtrude ? sweep.range / 100 : 1;
  const sign = isExtrude && sweep.value < 0 ? -1 : 1;
  const magnitude = Math.abs(sweep.value);

  const commit = (raw) => {
    let value = Number(raw);
    if (!Number.isFinite(value) || value <= 0) return;
    if (!isExtrude) value = Math.min(value, 360);
    onChange(sign * value);
  };

  return (
    <div className="sweep-panel">
      <div className="sweep-title">
        {isExtrude ? 'Extrude' : 'Revolve'}
        <span className="sweep-plane">{planeLabel(sweep.plane)} sketch</span>
      </div>
      <label className="sweep-field">
        {label}
        <input
          type="range"
          min={min}
          max={max}
          step={step}
          value={Math.min(magnitude, max)}
          onChange={(event) => commit(event.target.value)}
        />
        <input
          className="sweep-value"
          type="number"
          min="0"
          step="any"
          value={magnitude}
          onChange={(event) => commit(event.target.value)}
        />
        {!isExtrude && <span className="sweep-unit">°</span>}
      </label>
      {isExtrude && (
        <label className="sweep-flip">
          <input
            type="checkbox"
            checked={sign < 0}
            onChange={(event) => onChange(magnitude * (event.target.checked ? -1 : 1))}
          />
          Flip direction
        </label>
      )}
      {error && <div className="sweep-error">{error}</div>}
      <div className="sweep-actions">
        <button onClick={onApply} disabled={Boolean(error)}>
          Apply
        </button>
        <button className="secondary" onClick={onCancel}>
          Cancel
        </button>
      </div>
    </div>
  );
}
