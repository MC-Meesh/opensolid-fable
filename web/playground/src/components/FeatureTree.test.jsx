/**
 * Server-render smoke tests, matching SweepPanel.test.jsx: catch reference
 * errors and broken JSX in the render path without a browser. The feature
 * derivation and pruning logic is unit-tested in lib/featureTree.test.js.
 */
import { describe, expect, it } from 'vitest';
import { renderToString } from 'react-dom/server';
import FeatureTree from './FeatureTree.jsx';
import { buildFeatures } from '../lib/featureTree.js';

const noop = () => {};

function render(props = {}) {
  return renderToString(
    <FeatureTree
      features={[]}
      selectedId={null}
      hiddenKeys={new Set()}
      suppressedKeys={new Set()}
      collapsed={false}
      disabled={false}
      onToggleCollapse={noop}
      onSelect={noop}
      onRename={noop}
      onToggleHide={noop}
      onToggleSuppress={noop}
      onDelete={noop}
      {...props}
    />
  );
}

function sampleFeatures() {
  const profile = { start: [0, 0], segs: [{ x: 1, y: 0, bulge: 0 }] };
  const ext = { id: 1, op: 'extrude', args: [2], children: [], profile };
  const box = { id: 2, op: 'box3', args: [1, 1, 1], children: [] };
  const union = { id: 3, op: 'union', args: [], children: [ext, box] };
  return buildFeatures(union, { 'box:1': 'Pocket stock' });
}

describe('FeatureTree', () => {
  it('renders the empty state without a model', () => {
    const html = render();
    expect(html).toContain('Features');
    expect(html).toContain('Run a script to see its feature history');
  });

  it('renders chronological feature rows with icons, names and actions', () => {
    const html = render({ features: sampleFeatures() });
    expect(html).toContain('Extrude1');
    expect(html).toContain('Sketch1');
    expect(html).toContain('Pocket stock'); // rename applied
    expect(html).toContain('Union1');
    expect(html).toContain('feature-icon');
    expect(html).toContain('Hide Extrude1');
    expect(html).toContain('Suppress Union1');
    expect(html).toContain('Delete Extrude1');
    // Sketch rows have no visibility/suppress/delete of their own.
    expect(html).not.toContain('Hide Sketch1');
  });

  it('marks selected, hidden and suppressed rows', () => {
    const html = render({
      features: sampleFeatures(),
      selectedId: 3,
      hiddenKeys: new Set(['box:1']),
      suppressedKeys: new Set(['extrude:1']),
    });
    expect(html).toContain('selected');
    expect(html).toContain('hidden-feature');
    expect(html).toContain('suppressed');
    expect(html).toContain('Show Pocket stock');
    expect(html).toContain('Unsuppress Extrude1');
  });

  it('collapses to a thin docked strip', () => {
    const html = render({ collapsed: true });
    expect(html).toContain('feature-tree collapsed');
    expect(html).toContain('Show feature tree');
    expect(html).not.toContain('feature-row');
  });

  it('drops its own header chrome when embedded in the sidebar Tree tab', () => {
    const html = render({ embedded: true, features: sampleFeatures() });
    expect(html).toContain('feature-tree embedded');
    expect(html).not.toContain('feature-tree-header');
    expect(html).not.toContain('Collapse feature tree');
    expect(html).toContain('Extrude1'); // rows still render
  });
});
