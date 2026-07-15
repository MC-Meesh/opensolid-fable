// Parameter panel for a pending pattern / mirror feature (of-fsl.6). Mirrors
// SweepPanel: it edits a plain "pending feature" object live, Apply commits it
// as a scene-tree node wrapping the selected body, Cancel discards it.
//
// The feature acts on the currently selected body; axis/plane presets and the
// numeric fields drive a live preview re-meshed on every change.

const TITLES = {
  linearPattern: 'Linear Pattern',
  circularPattern: 'Circular Pattern',
  mirror: 'Mirror',
};

// Direction presets for the pattern axis / mirror normal.
const AXES = [
  { label: 'X', vec: [1, 0, 0] },
  { label: 'Y', vec: [0, 1, 0] },
  { label: 'Z', vec: [0, 0, 1] },
];

function NumberField({ label, unit, value, step = 0.1, min, max, onChange }) {
  return (
    <label className="feature-field">
      <span className="feature-field-label">{label}</span>
      <input
        type="number"
        step={step}
        min={min}
        max={max}
        value={value}
        onChange={(event) => {
          const v = Number(event.target.value);
          if (Number.isFinite(v)) onChange(v);
        }}
      />
      {unit && <span className="feature-unit">{unit}</span>}
    </label>
  );
}

function AxisPresets({ current, onPick }) {
  const norm = Math.hypot(current[0], current[1], current[2]) || 1;
  const active = AXES.find(
    (a) => Math.abs(current[0] / norm - a.vec[0]) < 1e-6
      && Math.abs(current[1] / norm - a.vec[1]) < 1e-6
      && Math.abs(current[2] / norm - a.vec[2]) < 1e-6
  );
  return (
    <div className="feature-presets">
      {AXES.map((a) => (
        <button
          key={a.label}
          type="button"
          className={`feature-preset${active === a ? ' active' : ''}`}
          onClick={() => onPick(a.vec)}
        >
          {a.label}
        </button>
      ))}
    </div>
  );
}

export default function FeaturePanel({ feature, error, onChange, onApply, onCancel }) {
  if (!feature) return null;
  const title = TITLES[feature.kind] ?? 'Feature';

  return (
    <div className="feature-panel">
      <div className="feature-title">{title}</div>

      {feature.kind === 'linearPattern' && (
        <>
          <div className="feature-group-label">Direction</div>
          <AxisPresets
            current={[feature.dx, feature.dy, feature.dz]}
            onPick={([x, y, z]) => {
              const mag = Math.hypot(feature.dx, feature.dy, feature.dz) || 1;
              onChange({ dx: x * mag, dy: y * mag, dz: z * mag });
            }}
          />
          <div className="feature-row">
            <NumberField label="dx" unit="mm" value={feature.dx} onChange={(dx) => onChange({ dx })} />
            <NumberField label="dy" unit="mm" value={feature.dy} onChange={(dy) => onChange({ dy })} />
            <NumberField label="dz" unit="mm" value={feature.dz} onChange={(dz) => onChange({ dz })} />
          </div>
          <NumberField
            label="Count"
            value={feature.count}
            step={1}
            min={1}
            onChange={(count) => onChange({ count: Math.max(1, Math.round(count)) })}
          />
        </>
      )}

      {feature.kind === 'circularPattern' && (
        <>
          <div className="feature-group-label">Axis</div>
          <AxisPresets
            current={[feature.ax, feature.ay, feature.az]}
            onPick={([ax, ay, az]) => onChange({ ax, ay, az })}
          />
          <div className="feature-row">
            <NumberField label="cx" unit="mm" value={feature.cx} onChange={(cx) => onChange({ cx })} />
            <NumberField label="cy" unit="mm" value={feature.cy} onChange={(cy) => onChange({ cy })} />
            <NumberField label="cz" unit="mm" value={feature.cz} onChange={(cz) => onChange({ cz })} />
          </div>
          <div className="feature-row">
            <NumberField
              label="Count"
              value={feature.count}
              step={1}
              min={1}
              onChange={(count) => onChange({ count: Math.max(1, Math.round(count)) })}
            />
            <NumberField
              label="Angle"
              unit="°"
              value={feature.angleDeg}
              step={5}
              onChange={(angleDeg) => onChange({ angleDeg })}
            />
          </div>
        </>
      )}

      {feature.kind === 'mirror' && (
        <>
          <div className="feature-group-label">Plane normal</div>
          <AxisPresets
            current={[feature.nx, feature.ny, feature.nz]}
            onPick={([nx, ny, nz]) => onChange({ nx, ny, nz })}
          />
          <div className="feature-row">
            <NumberField label="px" unit="mm" value={feature.px} onChange={(px) => onChange({ px })} />
            <NumberField label="py" unit="mm" value={feature.py} onChange={(py) => onChange({ py })} />
            <NumberField label="pz" unit="mm" value={feature.pz} onChange={(pz) => onChange({ pz })} />
          </div>
        </>
      )}

      {feature.picked && <div className="feature-hint">Using the picked face plane.</div>}
      {error && <div className="feature-error">{error}</div>}
      <div className="feature-actions">
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
