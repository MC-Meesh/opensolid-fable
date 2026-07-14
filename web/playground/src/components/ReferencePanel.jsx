import { useMemo, useState } from 'react';
import { buildReference } from '../lib/referenceGeometry.js';

/**
 * Reference-geometry creation panel (of-fsl.14): pick a datum type + method,
 * fill in the numeric parameters (and base planes for methods that need them),
 * and add a reference plane / axis / point / coordinate system to the model's
 * parallel reference collection. Display-only over App state — it builds plain
 * geometry via lib/referenceGeometry.js and hands it up through `onAdd`.
 *
 * Base pickers offer the three named world planes plus any reference planes
 * already created, so you can stack an offset plane off another datum (the
 * SolidWorks "offset plane above a face" flow).
 */

// Field specs per method. Field kinds: 'base' (plane dropdown), 'vec3' (three
// numbers), 'scalar' (one number). `optional` vec3 fields left at all-zero are
// dropped so the constructor takes its default.
const METHODS = {
  plane: [
    {
      id: 'plane-offset',
      label: 'Offset',
      fields: [
        { key: 'base', kind: 'base', label: 'Base plane' },
        { key: 'distance', kind: 'scalar', label: 'Distance', default: 10 },
      ],
    },
    {
      id: 'plane-angled',
      label: 'At angle',
      fields: [
        { key: 'base', kind: 'base', label: 'Base plane' },
        { key: 'angleDeg', kind: 'scalar', label: 'Angle (°)', default: 30 },
        { key: 'axisDir', kind: 'vec3', label: 'Hinge axis (optional)', optional: true },
      ],
    },
    {
      id: 'plane-mid',
      label: 'Mid-plane',
      fields: [
        { key: 'base', kind: 'base', label: 'Plane A' },
        { key: 'base2', kind: 'base', label: 'Plane B', default: 'XZ' },
      ],
    },
  ],
  axis: [
    {
      id: 'axis-2pt',
      label: 'Two points',
      fields: [
        { key: 'p1', kind: 'vec3', label: 'Point 1' },
        { key: 'p2', kind: 'vec3', label: 'Point 2', default: [0, 0, 10] },
      ],
    },
    {
      id: 'axis-ptdir',
      label: 'Point + direction',
      fields: [
        { key: 'point', kind: 'vec3', label: 'Point' },
        { key: 'direction', kind: 'vec3', label: 'Direction', default: [0, 0, 1] },
      ],
    },
    {
      id: 'axis-intersect',
      label: 'Plane intersection',
      fields: [
        { key: 'base', kind: 'base', label: 'Plane A' },
        { key: 'base2', kind: 'base', label: 'Plane B', default: 'XZ' },
      ],
    },
  ],
  point: [
    {
      id: 'point-coords',
      label: 'Coordinates',
      fields: [{ key: 'coords', kind: 'vec3', label: 'X, Y, Z' }],
    },
    {
      id: 'point-mid',
      label: 'Midpoint',
      fields: [
        { key: 'p1', kind: 'vec3', label: 'Point 1' },
        { key: 'p2', kind: 'vec3', label: 'Point 2', default: [0, 0, 10] },
      ],
    },
  ],
  csys: [
    {
      id: 'csys-plane',
      label: 'From plane',
      fields: [{ key: 'base', kind: 'base', label: 'Plane' }],
    },
    {
      id: 'csys-ptaxes',
      label: 'Point + axes',
      fields: [
        { key: 'origin', kind: 'vec3', label: 'Origin' },
        { key: 'xDir', kind: 'vec3', label: 'X direction', default: [1, 0, 0] },
        { key: 'yHint', kind: 'vec3', label: 'Y hint', default: [0, 1, 0] },
      ],
    },
  ],
};

const KIND_LABELS = { plane: 'Plane', axis: 'Axis', point: 'Point', csys: 'Coord System' };
const NAMED_PLANES = ['XY', 'XZ', 'YZ'];

/** Initial form state for a method: its field defaults. */
function initialForm(method) {
  const form = {};
  for (const f of method.fields) {
    if (f.kind === 'vec3') form[f.key] = f.default ?? [0, 0, 0];
    else if (f.kind === 'scalar') form[f.key] = f.default ?? 0;
    else if (f.kind === 'base') form[f.key] = f.default ?? 'XY';
  }
  return form;
}

export default function ReferencePanel({ open, refGeom = [], onAdd, onClose }) {
  const [kind, setKind] = useState('plane');
  const [methodId, setMethodId] = useState('plane-offset');
  const method = useMemo(
    () => METHODS[kind].find((m) => m.id === methodId) ?? METHODS[kind][0],
    [kind, methodId]
  );
  const [form, setForm] = useState(() => initialForm(METHODS.plane[0]));
  const [error, setError] = useState(null);

  // Base-plane options: named world planes + existing reference planes.
  const baseOptions = useMemo(() => {
    const refs = refGeom
      .filter((r) => r.kind === 'plane')
      .map((r, i) => ({ token: `ref:${r.id}`, label: r.name || `Plane${i + 1}` }));
    return [...NAMED_PLANES.map((p) => ({ token: p, label: p })), ...refs];
  }, [refGeom]);

  if (!open) return null;

  const selectKind = (k) => {
    const first = METHODS[k][0];
    setKind(k);
    setMethodId(first.id);
    setForm(initialForm(first));
    setError(null);
  };

  const selectMethod = (id) => {
    const m = METHODS[kind].find((x) => x.id === id);
    setMethodId(id);
    setForm(initialForm(m));
    setError(null);
  };

  const setScalar = (key, value) => setForm((f) => ({ ...f, [key]: value }));
  const setVec = (key, i, value) =>
    setForm((f) => {
      const next = [...f[key]];
      next[i] = value;
      return { ...f, [key]: next };
    });

  // Resolve a base token to the value referenceGeometry expects: the named
  // string, or the stored reference-plane geom object.
  const resolveBase = (token) => {
    if (NAMED_PLANES.includes(token)) return token;
    const item = refGeom.find((r) => `ref:${r.id}` === token);
    return item?.geom ?? 'XY';
  };

  const num = (v) => {
    const n = Number(v);
    return Number.isFinite(n) ? n : 0;
  };
  const vec = (arr) => arr.map(num);
  const isZeroVec = (arr) => vec(arr).every((c) => c === 0);

  const submit = () => {
    const params = {};
    for (const f of method.fields) {
      if (f.kind === 'base') params[f.key] = resolveBase(form[f.key]);
      else if (f.kind === 'scalar') params[f.key] = num(form[f.key]);
      else if (f.kind === 'vec3') {
        if (f.optional && isZeroVec(form[f.key])) continue;
        params[f.key] = vec(form[f.key]);
      }
    }
    try {
      const { kind: k, geom } = buildReference(method.id, params);
      onAdd(k, geom);
      setError(null);
    } catch (err) {
      setError(String(err.message ?? err));
    }
  };

  return (
    <div className="reference-panel">
      <div className="reference-title">
        Reference Geometry
        <button
          className="reference-close"
          onClick={onClose}
          title="Close reference geometry"
          aria-label="Close reference geometry"
        >
          ×
        </button>
      </div>

      <div className="reference-kinds" role="group" aria-label="Reference type">
        {Object.keys(METHODS).map((k) => (
          <button
            key={k}
            className={kind === k ? 'active' : 'secondary'}
            aria-pressed={kind === k}
            onClick={() => selectKind(k)}
          >
            {KIND_LABELS[k]}
          </button>
        ))}
      </div>

      <label className="reference-field">
        Method
        <select value={methodId} onChange={(e) => selectMethod(e.target.value)}>
          {METHODS[kind].map((m) => (
            <option key={m.id} value={m.id}>
              {m.label}
            </option>
          ))}
        </select>
      </label>

      {method.fields.map((f) => (
        <label className="reference-field" key={f.key}>
          {f.label}
          {f.kind === 'base' && (
            <select value={form[f.key]} onChange={(e) => setScalar(f.key, e.target.value)}>
              {baseOptions.map((o) => (
                <option key={o.token} value={o.token}>
                  {o.label}
                </option>
              ))}
            </select>
          )}
          {f.kind === 'scalar' && (
            <input
              type="number"
              step="any"
              value={form[f.key]}
              onChange={(e) => setScalar(f.key, e.target.value)}
            />
          )}
          {f.kind === 'vec3' && (
            <span className="reference-vec">
              {[0, 1, 2].map((i) => (
                <input
                  key={i}
                  type="number"
                  step="any"
                  aria-label={`${f.label} ${'XYZ'[i]}`}
                  value={form[f.key][i]}
                  onChange={(e) => setVec(f.key, i, e.target.value)}
                />
              ))}
            </span>
          )}
        </label>
      ))}

      {error && <div className="reference-error">{error}</div>}

      <button className="reference-create" onClick={submit}>
        Add {KIND_LABELS[kind]}
      </button>
    </div>
  );
}
