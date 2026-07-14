import { SECTION_AXES } from '../lib/sectionView.js';

/**
 * Controls for an active section view (of-fsl.18): pick which axis the plane
 * cuts across, flip which half is kept, and slide the plane along its normal.
 * The offset also tracks the 3D drag handle in the viewport — this panel and
 * the handle drive the same value. Display-only; nothing here edits the model.
 */
export default function SectionPanel({ section, range, onAxisChange, onFlip, onOffsetChange, onClose }) {
  if (!section) return null;
  const step = Math.max((range.max - range.min) / 200, 1e-4);

  const commit = (raw) => {
    const value = Number(raw);
    if (Number.isFinite(value)) onOffsetChange(value);
  };

  return (
    <div className="section-panel">
      <div className="section-title">
        Section View
        <button
          className="section-close"
          onClick={onClose}
          title="Turn off the section view"
          aria-label="Close section view"
        >
          ×
        </button>
      </div>
      <div className="section-axes" role="group" aria-label="Section axis">
        {SECTION_AXES.map((axis) => (
          <button
            key={axis}
            className={section.axis === axis ? 'active' : 'secondary'}
            aria-pressed={section.axis === axis}
            title={`Cut across the ${axis} axis`}
            onClick={() => onAxisChange(axis)}
          >
            {axis}
          </button>
        ))}
      </div>
      <label className="section-field">
        Offset
        <input
          type="range"
          min={range.min}
          max={range.max}
          step={step}
          value={Math.min(Math.max(section.offset, range.min), range.max)}
          onChange={(event) => commit(event.target.value)}
        />
        <input
          className="section-value"
          type="number"
          step="any"
          value={Number(section.offset.toFixed(4))}
          onChange={(event) => commit(event.target.value)}
        />
      </label>
      <label className="section-flip">
        <input type="checkbox" checked={section.flip} onChange={(event) => onFlip(event.target.checked)} />
        Flip side
      </label>
    </div>
  );
}
