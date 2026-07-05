import { argLabel, PALETTE } from '../lib/shapeGraph.js';

function NumberField({ label, value, onCommit, disabled }) {
  return (
    <label className="param">
      <span>{label}</span>
      <input
        type="number"
        step="0.05"
        defaultValue={value}
        key={value}
        disabled={disabled}
        onBlur={(event) => {
          const v = Number(event.target.value);
          if (Number.isFinite(v) && v !== value) onCommit(v);
        }}
        onKeyDown={(event) => {
          if (event.key === 'Enter') event.target.blur();
        }}
      />
    </label>
  );
}

function NodeParams({ node, onUpdateArg, disabled }) {
  const rows = [];
  node.chain.forEach((link, linkIndex) => {
    if (!link.args) return;
    link.args.forEach((arg, argIndex) => {
      if (arg.kind !== 'num') return;
      rows.push(
        <NumberField
          key={`${linkIndex}.${argIndex}`}
          label={`${link.name}.${argLabel(link.name, argIndex)}`}
          value={arg.value}
          disabled={disabled}
          onCommit={(v) => onUpdateArg(node.id, linkIndex, argIndex, v)}
        />
      );
    });
  });
  if (rows.length === 0) return null;
  return <div className="node-params">{rows}</div>;
}

/**
 * Scene tree + shape palette: the GUI view of the shape operation graph.
 *
 * Every row is one script statement. Selecting a `def` row shows its numeric
 * parameters (edits write back into the script) and drives the viewport
 * gizmo. Raw rows are hand-written code the parser leaves alone.
 */
export default function ScenePanel({
  nodes,
  selected,
  onSelect,
  onAddShape,
  onDeleteShape,
  onUpdateArg,
  disabled,
}) {
  return (
    <div className="scene-panel">
      <div className="palette">
        {PALETTE.map((item) => (
          <button
            key={item.ctor}
            className="secondary"
            disabled={disabled}
            title={`Add ${item.label.toLowerCase()} to the scene`}
            onClick={() => onAddShape(item.ctor, item.args)}
          >
            + {item.label}
          </button>
        ))}
      </div>
      <ul className="scene-list">
        {nodes.map((node) => {
          if (node.kind === 'raw') {
            return (
              <li key={node.id} className="node raw" title="Hand-written code — edit in the script">
                <code>{node.label}</code>
              </li>
            );
          }
          const isSelected = node.id === selected;
          return (
            <li key={node.id} className={`node${isSelected ? ' selected' : ''}`}>
              <div
                className="node-row"
                onClick={() => onSelect(isSelected ? null : node.id)}
              >
                <span className="node-name">
                  {node.kind === 'ret' ? 'output' : node.name}
                </span>
                <span className="node-label">{node.label}</span>
                {node.kind === 'def' && (
                  <button
                    className="node-delete"
                    title={`Delete ${node.name}`}
                    disabled={disabled}
                    onClick={(event) => {
                      event.stopPropagation();
                      onDeleteShape(node.name);
                    }}
                  >
                    ×
                  </button>
                )}
              </div>
              {isSelected && (
                <NodeParams node={node} onUpdateArg={onUpdateArg} disabled={disabled} />
              )}
            </li>
          );
        })}
      </ul>
    </div>
  );
}
