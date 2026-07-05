import { useState } from 'react';
import { nodeLabel } from '../lib/sceneTree.js';

function TreeRow({ node, depth, selectedId, onSelect }) {
  const [open, setOpen] = useState(true);
  const hasChildren = node.children.length > 0;
  const selected = node.id === selectedId;

  const activate = () => onSelect(node);
  const toggle = (event) => {
    event.stopPropagation();
    setOpen((o) => !o);
  };

  return (
    <>
      <div
        className={`tree-row${selected ? ' selected' : ''}`}
        style={{ paddingLeft: 8 + depth * 14 }}
        role="treeitem"
        aria-selected={selected}
        aria-expanded={hasChildren ? open : undefined}
        tabIndex={0}
        onClick={activate}
        onKeyDown={(event) => {
          if (event.key === 'Enter' || event.key === ' ') {
            event.preventDefault();
            activate();
          }
        }}
      >
        {hasChildren ? (
          <span className="tree-toggle" onClick={toggle}>
            {open ? '▾' : '▸'}
          </span>
        ) : (
          <span className="tree-toggle" />
        )}
        <span className="tree-label">{nodeLabel(node)}</span>
      </div>
      {open &&
        node.children.map((child, i) => (
          // A shared (DAG) child can appear under several parents, so the id
          // alone is not unique — include the position in the key.
          <TreeRow
            key={`${child.id}-${i}`}
            node={child}
            depth={depth + 1}
            selectedId={selectedId}
            onSelect={onSelect}
          />
        ))}
    </>
  );
}

/**
 * Sidebar view of the script's construction tree. The final shape is the
 * root; clicking a node isolates that intermediate shape in the viewport,
 * clicking it again (or the root) shows the full model.
 */
export default function SceneTree({ root, selectedId, onSelect }) {
  return (
    <div className="scene-tree">
      <div className="scene-tree-header">
        <span>Scene</span>
        <span className="scene-tree-hint">click a step to isolate it</span>
      </div>
      <div className="scene-tree-body" role="tree">
        {root ? (
          <TreeRow node={root} depth={0} selectedId={selectedId} onSelect={onSelect} />
        ) : (
          <div className="scene-tree-empty">Run a script to see its construction tree.</div>
        )}
      </div>
    </div>
  );
}
