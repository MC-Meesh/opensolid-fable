/** Mesh statistics overlay: triangle/vertex counts, mesh source, mesh time. */
export default function StatusBar({ stats }) {
  const { triangles, vertices, resolution, exact, elapsedMs } = stats;
  return (
    <div className="stats">
      {triangles.toLocaleString()} triangles · {vertices.toLocaleString()}{' '}
      vertices · {exact ? 'exact B-Rep' : `${resolution}³ grid`} ·{' '}
      {elapsedMs.toFixed(0)} ms
    </div>
  );
}
