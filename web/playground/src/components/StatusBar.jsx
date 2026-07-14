import { DEFAULT_LENGTH_UNIT, unitLabel } from '../lib/units.js';

/** Mesh statistics overlay: triangle/vertex counts, mesh source, mesh time. */
export default function StatusBar({ stats, documentUnit = DEFAULT_LENGTH_UNIT }) {
  const { triangles, vertices, accuracy, exact, elapsedMs } = stats;
  return (
    <div className="stats">
      {triangles.toLocaleString()} triangles · {vertices.toLocaleString()}{' '}
      vertices ·{' '}
      {exact ? 'exact B-Rep' : `±${accuracy.toPrecision(2)} ${unitLabel(documentUnit)}`} ·{' '}
      {elapsedMs.toFixed(0)} ms
    </div>
  );
}
