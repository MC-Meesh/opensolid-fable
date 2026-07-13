import { useState } from 'react';

// Feature-type icons (12×12 stroke glyphs, SolidWorks-ish shorthand).
const ICON_PATHS = {
  sketch: 'M2 10 L10 2 M2 10 l2.4 -0.6 l-1.8 -1.8 Z',
  extrude: 'M3 5 h6 v5 h-6 Z M3 5 l2 -2.5 h6 l-2 2.5 M9 5 l2 -2.5 v5 l-2 2.5',
  revolve: 'M6 2 v8 M8 3.5 a4 2.5 0 1 1 -1 6.2',
  primitive: 'M2.5 4.5 L6 2.5 l3.5 2 v3.5 L6 10 l-3.5 -2 Z M2.5 4.5 L6 6.5 l3.5 -2 M6 6.5 V10',
  boolean: 'M4.5 7.5 a3 3 0 1 1 3 -3 M7.5 4.5 a3 3 0 1 1 -3 3',
  transform: 'M6 1.5 v9 M6 1.5 l-1.5 2 M6 1.5 l1.5 2 M1.5 6 h9 M10.5 6 l-2 -1.5 M10.5 6 l-2 1.5',
};

function FeatureIcon({ kind }) {
  const d = ICON_PATHS[kind] ?? ICON_PATHS.primitive;
  return (
    <svg className="feature-icon" viewBox="0 0 12 12" aria-hidden="true">
      <path d={d} />
    </svg>
  );
}

function EyeIcon({ off }) {
  return (
    <svg viewBox="0 0 12 12" aria-hidden="true">
      <path d="M1 6 q5 -4.6 10 0 q-5 4.6 -10 0 Z" />
      <circle cx="6" cy="6" r="1.6" />
      {off && <path d="M2 10 L10 2" className="eye-slash" />}
    </svg>
  );
}

function FeatureRow({
  feature,
  selected,
  hidden,
  suppressed,
  expandable,
  open,
  onToggleOpen,
  disabled,
  onSelect,
  onRename,
  onToggleHide,
  onToggleSuppress,
  onDelete,
}) {
  const [renaming, setRenaming] = useState(false);
  const isSketch = feature.kind === 'sketch';

  const commitRename = (value) => {
    setRenaming(false);
    const name = value.trim();
    if (name !== feature.name) onRename(feature.key, name);
  };

  return (
    <div
      className={`feature-row${selected ? ' selected' : ''}${
        suppressed ? ' suppressed' : ''
      }${hidden ? ' hidden-feature' : ''}`}
      style={{ paddingLeft: 6 + feature.depth * 16 }}
      role="treeitem"
      aria-selected={selected}
      aria-expanded={expandable ? open : undefined}
      tabIndex={0}
      onClick={() => onSelect(feature)}
      onKeyDown={(event) => {
        if (event.key === 'Enter' || event.key === ' ') {
          event.preventDefault();
          onSelect(feature);
        }
      }}
    >
      {expandable ? (
        <span
          className="feature-expander"
          onClick={(event) => {
            event.stopPropagation();
            onToggleOpen();
          }}
        >
          {open ? '▾' : '▸'}
        </span>
      ) : (
        <span className="feature-expander" />
      )}
      <FeatureIcon kind={feature.kind} />
      {renaming ? (
        <input
          className="feature-rename"
          defaultValue={feature.name}
          autoFocus
          onClick={(event) => event.stopPropagation()}
          onBlur={(event) => commitRename(event.target.value)}
          onKeyDown={(event) => {
            if (event.key === 'Enter') event.target.blur();
            if (event.key === 'Escape') {
              event.stopPropagation();
              setRenaming(false);
            }
          }}
        />
      ) : (
        <span
          className="feature-name"
          title={`${feature.name} — double-click to rename`}
          onDoubleClick={(event) => {
            event.stopPropagation();
            if (!disabled) setRenaming(true);
          }}
        >
          {feature.name}
        </span>
      )}
      {!isSketch && (
        <span className="feature-actions">
          <button
            className="feature-action"
            title={suppressed ? `Unsuppress ${feature.name}` : `Suppress ${feature.name}`}
            disabled={disabled}
            onClick={(event) => {
              event.stopPropagation();
              onToggleSuppress(feature.key);
            }}
          >
            {suppressed ? '▶' : '⏸'}
          </button>
          <button
            className="feature-action delete"
            title={`Delete ${feature.name}`}
            disabled={disabled}
            onClick={(event) => {
              event.stopPropagation();
              onDelete(feature);
            }}
          >
            ×
          </button>
          <button
            className={`feature-action eye${hidden ? ' off' : ''}`}
            title={hidden ? `Show ${feature.name}` : `Hide ${feature.name}`}
            disabled={disabled}
            onClick={(event) => {
              event.stopPropagation();
              onToggleHide(feature.key);
            }}
          >
            <EyeIcon off={hidden} />
          </button>
        </span>
      )}
    </div>
  );
}

/**
 * Docked feature tree (SolidWorks FeatureManager style): the chronological
 * feature history of the model. Rows have a type icon, a renameable name
 * (double-click), an eye visibility toggle, and hover actions (suppress,
 * delete). Sketch features nest under the sweep that consumed them; clicking
 * one re-enters sketch mode on it, clicking any other feature opens its
 * parameters in the property panel.
 *
 * Purely presentational — every action is delegated to App, which owns the
 * script/tree source of truth.
 *
 * `embedded` drops the panel's own header/collapse chrome: the sidebar's
 * Tree tab already names and shows/hides it. The standalone (dockable)
 * rendering keeps the header and can collapse to a thin strip.
 */
export default function FeatureTree({
  features,
  selectedId,
  hiddenKeys,
  suppressedKeys,
  collapsed,
  embedded = false,
  disabled,
  onToggleCollapse,
  onSelect,
  onRename,
  onToggleHide,
  onToggleSuppress,
  onDelete,
}) {
  const [closedKeys, setClosedKeys] = useState(() => new Set());

  if (collapsed && !embedded) {
    return (
      <div className="feature-tree collapsed">
        <button
          className="feature-collapse"
          title="Show feature tree"
          onClick={onToggleCollapse}
        >
          ▸
        </button>
        <span className="feature-tree-side-label">Features</span>
      </div>
    );
  }

  const toggleOpen = (key) => {
    setClosedKeys((prev) => {
      const next = new Set(prev);
      if (next.has(key)) next.delete(key);
      else next.add(key);
      return next;
    });
  };

  const childCount = new Map();
  for (const f of features) {
    if (f.parentKey) childCount.set(f.parentKey, (childCount.get(f.parentKey) ?? 0) + 1);
  }

  return (
    <div className={`feature-tree${embedded ? ' embedded' : ''}`}>
      {!embedded && (
        <div className="feature-tree-header">
          <span>Features</span>
          <button
            className="feature-collapse"
            title="Collapse feature tree"
            onClick={onToggleCollapse}
          >
            ◂
          </button>
        </div>
      )}
      <div className="feature-tree-body" role="tree">
        {features.length === 0 && (
          <div className="feature-tree-empty">
            Run a script to see its feature history.
          </div>
        )}
        {features.map((feature) => {
          if (feature.parentKey && closedKeys.has(feature.parentKey)) return null;
          return (
            <FeatureRow
              key={feature.key}
              feature={feature}
              selected={feature.kind !== 'sketch' && feature.id === selectedId}
              hidden={hiddenKeys.has(feature.key)}
              suppressed={suppressedKeys.has(feature.key)}
              expandable={(childCount.get(feature.key) ?? 0) > 0}
              open={!closedKeys.has(feature.key)}
              onToggleOpen={() => toggleOpen(feature.key)}
              disabled={disabled}
              onSelect={onSelect}
              onRename={onRename}
              onToggleHide={onToggleHide}
              onToggleSuppress={onToggleSuppress}
              onDelete={onDelete}
            />
          );
        })}
      </div>
    </div>
  );
}
