import { planeLabel } from '../lib/sketch/profile.js';

const END_LABELS = {
  blind: 'Blind',
  symmetric: 'Symmetric',
  through: 'Through all',
  toFace: 'Up to face',
};

/**
 * Parameter panel for a pending extrude/revolve.
 *
 * Revolve keeps a single angle. Extrude carries SolidWorks-parity controls:
 * a mode (Boss adds / Cut removes material), an end condition (Blind,
 * Symmetric, Through all, Up to face), and a draft angle. Live preview and
 * script serialization both flow from the same sweep descriptor
 * (lib/sweep.js), so what the panel shows is what the tree commits.
 *
 * Blind heights are signed — the slider/number edit the magnitude and "Flip
 * direction" carries the sign (of-4eh.16). Symmetric and Through all are
 * centered on the sketch plane (no flip); Through all and Up to face size
 * themselves from the scene / the picked face, so they hide the height field.
 */
export default function SweepPanel({ sweep, error, onChange, onField, onApply, onCancel }) {
  if (!sweep) return null;
  const isExtrude = sweep.kind === 'extrude';
  const end = sweep.end ?? 'blind';
  const mode = sweep.mode ?? 'boss';
  const label = isExtrude ? 'Height' : 'Angle';
  const min = isExtrude ? sweep.range / 100 : 1;
  const max = isExtrude ? sweep.range : 360;
  const step = isExtrude ? sweep.range / 100 : 1;
  const sign = isExtrude && sweep.value < 0 ? -1 : 1;
  const magnitude = Math.abs(sweep.value);
  // Height is user-driven only for blind/symmetric; through-all and up-to-face
  // derive their extent, so the numeric field would be meaningless.
  const showHeight = !isExtrude || end === 'blind' || end === 'symmetric';
  const canFlip = isExtrude && end === 'blind';

  const commit = (raw) => {
    let value = Number(raw);
    if (!Number.isFinite(value) || value <= 0) return;
    if (!isExtrude) value = Math.min(value, 360);
    onChange(sign * value);
  };

  const commitDraft = (raw) => {
    const value = Number(raw);
    if (!Number.isFinite(value)) return;
    // Kernel rejects |draft| ≥ ~80°; keep the control inside that.
    onField({ draft: Math.max(-80, Math.min(80, value)) });
  };

  return (
    <div className="sweep-panel">
      <div className="sweep-title">
        {isExtrude ? 'Extrude' : 'Revolve'}
        <span className="sweep-plane">{planeLabel(sweep.plane)} sketch</span>
      </div>

      {isExtrude && (
        <div className="sweep-modes" role="radiogroup" aria-label="Extrude mode">
          {['boss', 'cut'].map((m) => (
            <button
              key={m}
              type="button"
              role="radio"
              aria-checked={mode === m}
              className={mode === m ? 'sweep-mode-active' : 'secondary'}
              onClick={() => onField({ mode: m })}
            >
              {m === 'boss' ? 'Boss' : 'Cut'}
            </button>
          ))}
        </div>
      )}

      {isExtrude && (
        <label className="sweep-field">
          End
          <select value={end} onChange={(event) => onField({ end: event.target.value })}>
            {Object.entries(END_LABELS).map(([value, text]) => (
              <option key={value} value={value}>
                {text}
              </option>
            ))}
          </select>
        </label>
      )}

      {showHeight && (
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
      )}

      {isExtrude && end === 'toFace' && (
        <div className={sweep.target ? 'sweep-target' : 'sweep-target sweep-target-empty'}>
          {sweep.target ? 'Target face selected' : 'Click a flat face to terminate'}
        </div>
      )}

      {isExtrude && (
        <label className="sweep-field">
          Draft
          <input
            className="sweep-value"
            type="number"
            step="any"
            value={sweep.draft ?? 0}
            onChange={(event) => commitDraft(event.target.value)}
          />
          <span className="sweep-unit">°</span>
        </label>
      )}

      {canFlip && (
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
