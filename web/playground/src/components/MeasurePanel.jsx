// Measure-tool readout (of-fsl.17): a floating panel that mirrors SolidWorks'
// Measure dialog. It always shows the body's bounding-box dimensions, and —
// as the user clicks up to two entities (vertex / edge / face / circular rim)
// — the single-entity measurement (coordinate, length, radius, area) or the
// pair relationship (distance + per-axis deltas, and angle / gap / plane
// distance when the kinds make it meaningful). All numbers come from the pure
// lib/measure.js helpers; this component only formats them.

const ENTITY_LABEL = {
  vertex: 'Vertex',
  edge: 'Edge',
  circle: 'Circle',
  face: 'Face',
  point: 'Point',
};

/** Compact number: 3 decimals under 1000, integer above, trailing zeros cut. */
function num(x) {
  if (!Number.isFinite(x)) return '—';
  const s = Math.abs(x) >= 1000 ? x.toFixed(0) : x.toFixed(3);
  return s.replace(/\.?0+$/, '') || '0';
}

function coord(p) {
  return `(${num(p[0])}, ${num(p[1])}, ${num(p[2])})`;
}

function Row({ label, value }) {
  return (
    <div className="measure-row">
      <span className="measure-label">{label}</span>
      <span className="measure-value">{value}</span>
    </div>
  );
}

function SingleReadout({ single }) {
  switch (single.kind) {
    case 'vertex':
      return <Row label="Coordinate" value={coord(single.coord)} />;
    case 'edge':
      return <Row label="Length" value={num(single.length)} />;
    case 'circle':
      return (
        <>
          <Row label="Radius" value={num(single.radius)} />
          <Row label="Diameter" value={num(single.diameter)} />
          <Row label="Center" value={coord(single.center)} />
        </>
      );
    case 'face':
      return <Row label="Area" value={num(single.area)} />;
    default:
      return <Row label="Point" value={coord(single.coord)} />;
  }
}

function PairReadout({ pair }) {
  return (
    <>
      <Row label="Distance" value={num(pair.distance)} />
      <Row label="ΔX, ΔY, ΔZ" value={`${num(pair.delta[0])}, ${num(pair.delta[1])}, ${num(pair.delta[2])}`} />
      {pair.angle != null && <Row label="Angle" value={`${pair.angle.toFixed(1)}°`} />}
      {pair.gap != null && <Row label="Parallel gap" value={num(pair.gap)} />}
      {pair.planeDistance != null && (
        <Row label="Distance to face" value={num(pair.planeDistance)} />
      )}
    </>
  );
}

export default function MeasurePanel({ active, bbox, entities, single, pair, onClear }) {
  if (!active) return null;
  const kinds = entities.map((e) => ENTITY_LABEL[e.kind] ?? 'Point');
  return (
    <div className="measure-panel">
      <div className="measure-title">
        Measure
        {entities.length > 0 && (
          <button className="measure-clear" onClick={onClear} title="Clear selection (Esc)">
            Clear
          </button>
        )}
      </div>

      {bbox && (
        <div className="measure-section">
          <div className="measure-section-title">Body size</div>
          <Row
            label="X × Y × Z"
            value={`${num(bbox.size[0])} × ${num(bbox.size[1])} × ${num(bbox.size[2])}`}
          />
          <Row label="Diagonal" value={num(bbox.diagonal)} />
        </div>
      )}

      <div className="measure-section">
        <div className="measure-section-title">
          {entities.length === 0
            ? 'Selection'
            : kinds.join(' → ')}
        </div>
        {entities.length === 0 && (
          <div className="measure-hint">Click a vertex, edge, face, or hole rim.</div>
        )}
        {single && <SingleReadout single={single} />}
        {pair && <PairReadout pair={pair} />}
      </div>
    </div>
  );
}
