// Measure readout (of-fsl.17): the floating panel shown while the Measure
// tool is active. It always reports the whole-body bounding-box dimensions,
// and — as entities are picked — the single-entity properties (coordinate,
// length, radius/diameter, area) or the pairwise distance/angle between two
// entities. Presentation only; the numbers come from lib/measure.js.

/** Trim a float to 4 decimals, dropping trailing zeros. */
function fmt(n) {
  return Number.isFinite(n) ? String(Number(n.toFixed(4))) : '—';
}

const vec = (v) => v.map(fmt).join(', ');

const ENTITY_LABEL = {
  vertex: 'Vertex',
  point: 'Point',
  edge: 'Edge',
  circle: 'Circle',
  face: 'Face',
};

function Row({ label, value }) {
  return (
    <div className="measure-row">
      <span className="measure-key">{label}</span>
      <span className="measure-val">{value}</span>
    </div>
  );
}

function SingleReadout({ single }) {
  switch (single.kind) {
    case 'vertex':
    case 'point':
      return <Row label="Coordinate" value={vec(single.coord)} />;
    case 'edge':
      return (
        <>
          <Row label={single.closed ? 'Perimeter' : 'Length'} value={fmt(single.length)} />
          {!single.closed && <Row label="From" value={vec(single.from)} />}
          {!single.closed && <Row label="To" value={vec(single.to)} />}
        </>
      );
    case 'circle':
      return (
        <>
          <Row label="Radius" value={fmt(single.radius)} />
          <Row label="Diameter" value={fmt(single.diameter)} />
          <Row label="Circumference" value={fmt(single.circumference)} />
          <Row label="Center" value={vec(single.center)} />
        </>
      );
    case 'face':
      return (
        <>
          <Row label="Area" value={fmt(single.area)} />
          <Row label="Centroid" value={vec(single.centroid)} />
          <Row label="Normal" value={vec(single.normal)} />
        </>
      );
    default:
      return null;
  }
}

function PairReadout({ pair }) {
  return (
    <>
      <Row label="Distance" value={fmt(pair.distance)} />
      <Row label="ΔX, ΔY, ΔZ" value={vec(pair.delta)} />
      {pair.angle !== undefined && <Row label="Angle" value={`${fmt(pair.angle)}°`} />}
      {pair.planeDistance !== undefined && (
        <Row label="Normal distance" value={fmt(pair.planeDistance)} />
      )}
    </>
  );
}

export default function MeasurePanel({ readout, onClear, onClose }) {
  const { bbox, single, pair, count } = readout;
  return (
    <div className="measure-panel" role="dialog" aria-label="Measure">
      <div className="measure-header">
        <span className="measure-title">Measure</span>
        <button className="measure-close" onClick={onClose} title="Close Measure (M)" aria-label="Close">
          ×
        </button>
      </div>

      {count === 2 && pair ? (
        <div className="measure-section">
          <div className="measure-section-label">Between 2 entities</div>
          <PairReadout pair={pair} />
        </div>
      ) : count === 1 && single ? (
        <div className="measure-section">
          <div className="measure-section-label">{ENTITY_LABEL[single.kind] ?? 'Entity'}</div>
          <SingleReadout single={single} />
        </div>
      ) : (
        <div className="measure-hint">Click a vertex, edge, or face. Pick two to measure between them.</div>
      )}

      {bbox && (
        <div className="measure-section">
          <div className="measure-section-label">Body dimensions</div>
          <Row label="Size X, Y, Z" value={vec(bbox.size)} />
          <Row label="Diagonal" value={fmt(bbox.diagonal)} />
        </div>
      )}

      {count > 0 && (
        <button className="measure-clear" onClick={onClear}>
          Clear selection
        </button>
      )}
    </div>
  );
}
