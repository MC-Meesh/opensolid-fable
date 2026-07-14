import { useMemo, useState } from 'react';
import { referenceEntityById } from '../lib/refGeomStore.js';

// Reference-geometry creation panel (of-fsl.14). A floating panel (SweepPanel
// sibling) that builds a datum plane / axis / point / coordinate system from a
// method + numeric params + base pickers, then hands the resolved spec to App
// (which builds and appends the entity through refGeomStore).
//
// Kept declarative: FORMS describes each method's fields, and the renderer maps
// field kinds to inputs and assembles the params object. Base pickers resolve
// a selection ('XY' | 'ref:<id>') to a plane string / entity here, so App only
// sees concrete geometry.

const NAMED_PLANES = ['XY', 'XZ', 'YZ'];

const KINDS = [
  { kind: 'plane', label: 'Plane' },
  { kind: 'axis', label: 'Axis' },
  { kind: 'point', label: 'Point' },
  { kind: 'csys', label: 'CSys' },
];

const num = (label, def) => ({ type: 'number', label, def });
const vec3 = (label, def) => ({ type: 'vec3', label, def });
const planeBase = (label = 'Base plane') => ({ type: 'planeBase', label, def: 'XY' });
const axisRef = (label = 'Axis') => ({ type: 'axisRef', label });

// method key -> ordered [name, fieldSpec] fields. Field names match the
// refGeomStore builder params exactly.
const FORMS = {
  plane: {
    offset: { label: 'Offset', fields: { base: planeBase(), distance: num('Distance', 10) } },
    angled: {
      label: 'At angle',
      fields: {
        base: planeBase(),
        angleDeg: num('Angle (°)', 45),
        hinge: { type: 'hinge', label: 'Hinge axis', def: 'u' },
      },
    },
    mid: { label: 'Mid plane', fields: { a: planeBase('Plane A'), b: planeBase('Plane B') } },
  },
  axis: {
    'two-points': { label: 'Two points', fields: { p1: vec3('Point 1', [0, 0, 0]), p2: vec3('Point 2', [1, 0, 0]) } },
    'point-direction': {
      label: 'Point + direction',
      fields: { origin: vec3('Origin', [0, 0, 0]), direction: vec3('Direction', [0, 0, 1]) },
    },
    'plane-intersection': {
      label: 'Plane intersection',
      fields: { a: planeBase('Plane A'), b: planeBase('Plane B') },
    },
  },
  point: {
    coords: { label: 'Coordinates', fields: { position: vec3('Position', [0, 0, 0]) } },
    midpoint: { label: 'Midpoint', fields: { p1: vec3('Point 1', [0, 0, 0]), p2: vec3('Point 2', [1, 0, 0]) } },
    'axis-plane': { label: 'Axis ∩ plane', fields: { axis: axisRef(), plane: planeBase() } },
  },
  csys: {
    plane: { label: 'From plane', fields: { plane: planeBase() } },
    'point-axes': {
      label: 'Point + axes',
      fields: {
        origin: vec3('Origin', [0, 0, 0]),
        xDir: vec3('X direction', [1, 0, 0]),
        yHint: vec3('Y direction', [0, 1, 0]),
      },
    },
  },
};

/** Initial raw form values for a method (mutable copies of the defaults). */
function initialValues(kind, method) {
  const out = {};
  for (const [name, spec] of Object.entries(FORMS[kind][method].fields)) {
    if (spec.type === 'vec3') out[name] = [...spec.def];
    else if (spec.type === 'axisRef') out[name] = '';
    else out[name] = spec.def;
  }
  return out;
}

export default function ReferencePanel({ open, refGeom = [], error, onCreate, onClose }) {
  const [kind, setKind] = useState('plane');
  const [method, setMethod] = useState('offset');
  const [values, setValues] = useState(() => initialValues('plane', 'offset'));
  const [name, setName] = useState('');

  const planeOptions = useMemo(
    () => [
      ...NAMED_PLANES.map((p) => ({ value: p, label: p })),
      ...refGeom
        .filter((item) => item.kind === 'plane')
        .map((item) => ({ value: `ref:${item.id}`, label: item.name })),
    ],
    [refGeom]
  );
  const axisOptions = useMemo(
    () => refGeom.filter((item) => item.kind === 'axis').map((item) => ({ value: `ref:${item.id}`, label: item.name })),
    [refGeom]
  );

  if (!open) return null;

  const chooseKind = (k) => {
    const firstMethod = Object.keys(FORMS[k])[0];
    setKind(k);
    setMethod(firstMethod);
    setValues(initialValues(k, firstMethod));
    setName('');
  };
  const chooseMethod = (m) => {
    setMethod(m);
    setValues(initialValues(kind, m));
  };

  const setField = (fieldName, value) => setValues((v) => ({ ...v, [fieldName]: value }));
  const setVecComponent = (fieldName, axis, value) =>
    setValues((v) => {
      const next = [...v[fieldName]];
      next[axis] = value;
      return { ...v, [fieldName]: next };
    });

  const resolvePlane = (value) =>
    typeof value === 'string' && value.startsWith('ref:')
      ? referenceEntityById(refGeom, Number(value.slice(4)))
      : value;

  // Assemble concrete params from the raw form values, coercing numbers and
  // resolving base pickers to geometry.
  const assembleParams = () => {
    const fields = FORMS[kind][method].fields;
    const params = {};
    for (const [fieldName, spec] of Object.entries(fields)) {
      const raw = values[fieldName];
      if (spec.type === 'number') params[fieldName] = Number(raw);
      else if (spec.type === 'vec3') params[fieldName] = raw.map(Number);
      else if (spec.type === 'planeBase') params[fieldName] = resolvePlane(raw);
      else if (spec.type === 'axisRef') params[fieldName] = referenceEntityById(refGeom, Number(String(raw).slice(4)));
      else params[fieldName] = raw;
    }
    return params;
  };

  const fields = FORMS[kind][method].fields;
  const needsAxis = Object.values(fields).some((s) => s.type === 'axisRef');
  const missingAxis = needsAxis && axisOptions.length === 0;

  const submit = () => {
    if (missingAxis) return;
    onCreate({ kind, method, params: assembleParams(), name: name.trim() || undefined });
  };

  return (
    <div className="ref-panel" role="dialog" aria-label="Reference geometry">
      <div className="ref-panel-title">
        Reference Geometry
        <button className="ref-panel-close" title="Close" onClick={onClose} aria-label="Close">
          ×
        </button>
      </div>

      <div className="ref-kind-tabs" role="tablist" aria-label="Reference type">
        {KINDS.map((k) => (
          <button
            key={k.kind}
            role="tab"
            aria-selected={kind === k.kind}
            className={`ref-kind${kind === k.kind ? ' active' : ''}`}
            onClick={() => chooseKind(k.kind)}
          >
            {k.label}
          </button>
        ))}
      </div>

      <label className="ref-field">
        Method
        <select value={method} onChange={(e) => chooseMethod(e.target.value)}>
          {Object.entries(FORMS[kind]).map(([m, spec]) => (
            <option key={m} value={m}>
              {spec.label}
            </option>
          ))}
        </select>
      </label>

      {Object.entries(fields).map(([fieldName, spec]) => (
        <div className="ref-field" key={fieldName}>
          <span className="ref-field-label">{spec.label}</span>
          {spec.type === 'number' && (
            <input
              type="number"
              step="any"
              value={values[fieldName]}
              onChange={(e) => setField(fieldName, e.target.value)}
            />
          )}
          {spec.type === 'vec3' && (
            <span className="ref-vec">
              {[0, 1, 2].map((axis) => (
                <input
                  key={axis}
                  type="number"
                  step="any"
                  aria-label={`${spec.label} ${['X', 'Y', 'Z'][axis]}`}
                  value={values[fieldName][axis]}
                  onChange={(e) => setVecComponent(fieldName, axis, e.target.value)}
                />
              ))}
            </span>
          )}
          {spec.type === 'planeBase' && (
            <select value={values[fieldName]} onChange={(e) => setField(fieldName, e.target.value)}>
              {planeOptions.map((o) => (
                <option key={o.value} value={o.value}>
                  {o.label}
                </option>
              ))}
            </select>
          )}
          {spec.type === 'hinge' && (
            <select value={values[fieldName]} onChange={(e) => setField(fieldName, e.target.value)}>
              <option value="u">U axis</option>
              <option value="v">V axis</option>
            </select>
          )}
          {spec.type === 'axisRef' && (
            <select value={values[fieldName]} onChange={(e) => setField(fieldName, e.target.value)}>
              <option value="">
                {axisOptions.length ? 'Select an axis…' : 'No reference axes yet'}
              </option>
              {axisOptions.map((o) => (
                <option key={o.value} value={o.value}>
                  {o.label}
                </option>
              ))}
            </select>
          )}
        </div>
      ))}

      <label className="ref-field">
        Name (optional)
        <input
          type="text"
          placeholder="auto"
          value={name}
          onChange={(e) => setName(e.target.value)}
        />
      </label>

      {missingAxis && (
        <div className="ref-error">Create a reference axis first to use this method.</div>
      )}
      {error && <div className="ref-error">{error}</div>}

      <div className="ref-actions">
        <button onClick={submit} disabled={missingAxis}>
          Create
        </button>
        <button className="secondary" onClick={onClose}>
          Done
        </button>
      </div>
    </div>
  );
}
