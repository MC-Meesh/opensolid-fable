import { useRef } from 'react';
import {
  BOOLEAN_CHOICES,
  clampDisplay,
  displayValue,
  opSpec,
} from '../lib/propertyEdit.js';

function fmt(value) {
  return String(Number(value.toFixed(4)));
}

/**
 * Numeric field with drag-to-adjust: drag the label horizontally to scrub
 * the value (hold Shift for coarse steps), or type into the input and
 * commit with Enter/blur. Values are clamped to the field's range.
 */
function DragNumber({ field, value, disabled, onCommit }) {
  const dragRef = useRef(null);

  const commit = (next) => {
    const clamped = clampDisplay(field, next);
    if (clamped !== value) onCommit(clamped);
  };

  const onPointerDown = (event) => {
    if (disabled) return;
    event.preventDefault();
    event.currentTarget.setPointerCapture(event.pointerId);
    dragRef.current = { x: event.clientX, start: value, last: value };
  };

  const onPointerMove = (event) => {
    const drag = dragRef.current;
    if (!drag) return;
    const dx = event.clientX - drag.x;
    const perPixel = (field.step * (event.shiftKey ? 5 : 1)) / 2;
    const next = Number(clampDisplay(field, drag.start + dx * perPixel).toFixed(4));
    if (next !== drag.last) {
      drag.last = next;
      onCommit(next);
    }
  };

  const onPointerUp = () => {
    dragRef.current = null;
  };

  return (
    <label className="prop-field">
      <span
        className={`prop-drag${disabled ? '' : ' enabled'}`}
        title="Drag to adjust (Shift = coarse)"
        onPointerDown={onPointerDown}
        onPointerMove={onPointerMove}
        onPointerUp={onPointerUp}
        onPointerCancel={onPointerUp}
      >
        {field.label}
      </span>
      <input
        type="number"
        key={value}
        defaultValue={fmt(value)}
        min={field.min}
        max={field.max}
        step={field.step}
        disabled={disabled}
        onBlur={(event) => {
          const v = Number(event.target.value);
          if (event.target.value !== '' && Number.isFinite(v)) {
            commit(v);
          } else {
            event.target.value = fmt(value);
          }
        }}
        onKeyDown={(event) => {
          if (event.key === 'Enter') event.target.blur();
        }}
      />
      <span className="prop-unit">{field.unit}</span>
    </label>
  );
}

/**
 * Property panel for the selected scene-tree node. Shows editable parameter
 * fields (grouped XYZ for transforms, dimensions for primitives, a blend
 * radius for smooth unions) and an operation dropdown for booleans. Edits
 * flow through the bidirectional sync: the script rewrites and the mesh
 * re-evaluates on every change.
 */
export default function PropertyPanel({ node, disabled, onEditArg, onChangeOp }) {
  const spec = opSpec(node.op);
  return (
    <div className="prop-panel">
      <div className="prop-title">{spec?.title ?? node.op}</div>
      {spec?.kind === 'boolean' && (
        <label className="prop-op">
          <span>Operation</span>
          <select
            value={node.op}
            disabled={disabled}
            onChange={(event) => onChangeOp(node.id, event.target.value)}
          >
            {BOOLEAN_CHOICES.map((choice) => (
              <option key={choice.op} value={choice.op}>
                {choice.label}
              </option>
            ))}
          </select>
        </label>
      )}
      {spec?.groups.map((group) => (
        <div className="prop-group" key={group.label}>
          <div className="prop-group-label">{group.label}</div>
          <div className="prop-fields">
            {group.fields.map((f) => (
              <DragNumber
                key={`${node.id}.${f.arg}`}
                field={f}
                value={displayValue(f, node)}
                disabled={disabled}
                onCommit={(v) => onEditArg(node.id, f.arg, v)}
              />
            ))}
          </div>
        </div>
      ))}
      {!spec && <div className="prop-empty">No editable parameters for this step.</div>}
    </div>
  );
}
