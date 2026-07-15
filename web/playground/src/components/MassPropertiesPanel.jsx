import { MATERIALS } from '../lib/materials.js';
import { formatMass, formatNumber } from '../lib/massProps.js';
import { unitLabel } from '../lib/units.js';

/**
 * Mass properties readout (of-fsl.19), the playground's Evaluate > Mass
 * Properties: assign a material density and read back volume, surface area,
 * mass, centre of mass, and the inertia tensor.
 *
 * Display-only — nothing here edits the model. The material is document
 * metadata, not geometry: changing it rescales mass and inertia and leaves the
 * shape untouched, which is why a density edit needs no re-measure (App
 * re-derives from the cached measurement, see lib/massProps.js).
 *
 * Geometry is reported in document units, mass and inertia in SI. Mixing the
 * two is deliberate: you author in millimetres and want to read millimetres
 * back, but a kilogram is a kilogram regardless of document unit.
 */
export default function MassPropertiesPanel({
  report,
  material,
  density,
  unit,
  onMaterialChange,
  onDensityChange,
  onClose,
}) {
  if (!report) return null;
  const u = unitLabel(unit);

  return (
    <div className="mass-panel">
      <div className="mass-title">
        Mass Properties
        <button
          className="mass-close"
          onClick={onClose}
          title="Close the mass properties readout"
          aria-label="Close mass properties"
        >
          ×
        </button>
      </div>

      <label className="mass-field">
        Material
        <select
          value={material}
          aria-label="Material"
          onChange={(event) => onMaterialChange(event.target.value)}
        >
          {MATERIALS.map((m) => (
            <option key={m.key} value={m.key}>
              {m.name}
            </option>
          ))}
        </select>
      </label>

      <label className="mass-field">
        Density
        <input
          className="mass-density"
          type="number"
          step="any"
          min="0"
          value={density}
          aria-label="Density in kilograms per cubic metre"
          onChange={(event) => onDensityChange(event.target.value)}
        />
        <span className="mass-unit">kg/m³</span>
      </label>

      {!report.ok ? (
        <p className="mass-error" role="status">
          {report.error}
        </p>
      ) : (
        <>
          <dl className="mass-values">
            <dt>Mass</dt>
            <dd>{formatMass(report.massKg)}</dd>
            <dt>Volume</dt>
            <dd>{`${formatNumber(report.volume)} ${u}³`}</dd>
            <dt>Surface area</dt>
            <dd>
              {report.surfaceArea == null ? '—' : `${formatNumber(report.surfaceArea)} ${u}²`}
            </dd>
            <dt>Center of mass</dt>
            <dd>{`${report.centroid.map((c) => formatNumber(c, 5)).join(', ')} ${u}`}</dd>
          </dl>

          {report.inertia && (
            <div className="mass-inertia">
              <div className="mass-inertia-title">Moments of inertia (kg·m², about the center of mass)</div>
              <table className="mass-tensor">
                <tbody>
                  {report.inertia.map((row, i) => (
                    <tr key={i}>
                      {row.map((v, j) => (
                        <td key={j}>{formatNumber(v, 4)}</td>
                      ))}
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          )}
        </>
      )}

      <div className="mass-footnote">
        {`${
          report.exact
            ? 'Exact: integrated over the validated B-Rep tessellation.'
            : 'Approximate: integrated over an adaptive SDF mesh.'
        }${report.triangles != null ? ` ${report.triangles} triangles.` : ''}`}
      </div>
    </div>
  );
}
