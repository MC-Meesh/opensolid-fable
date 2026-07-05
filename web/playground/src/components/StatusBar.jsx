/** Mesh statistics overlay: triangle/vertex counts, grid size, mesh time. */
export default function StatusBar({ stats }) {
  const { triangles, vertices, resolution, elapsedMs } = stats;
  return (
    <div className="stats">
      {triangles.toLocaleString()} triangles · {vertices.toLocaleString()}{' '}
      vertices · {resolution}³ grid · {elapsedMs.toFixed(0)} ms
    </div>
  );
}
