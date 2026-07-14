/**
 * Server-render smoke tests, matching SweepPanel.test.jsx: catch reference
 * errors and broken JSX in the render path without a browser. The feature
 * derivation and pruning logic is unit-tested in lib/featureTree.test.js.
 */
import { describe, expect, it } from 'vitest';
import { renderToString } from 'react-dom/server';
import FeatureTree from './FeatureTree.jsx';
import { buildFeatures, buildReferenceFeatures } from '../lib/featureTree.js';

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

  it('paints a rebuild badge for a dangling reference and none for ok', () => {
    const html = render({
      features: sampleFeatures(),
      rebuildState: new Map([
        ['extrude:1', { status: 'dangling', reason: 'nearest face too far' }],
        ['box:1', { status: 'ok' }],
      ]),
    });
    expect(html).toContain('feature-badge dangling');
    expect(html).toContain('rebuild-dangling');
    expect(html).toContain('Dangling reference');
    // An ok feature carries no badge markup.
    expect(html).not.toContain('feature-badge error');
  });

  it('paints an error badge for a feature that failed to rebuild', () => {
    const html = render({
      features: sampleFeatures(),
      rebuildState: new Map([['union:1', { status: 'error', reason: 'nan bounds' }]]),
    });
    expect(html).toContain('feature-badge error');
    expect(html).toContain('Rebuild error');
  });

  it('renders reference-geometry rows with rename + delete but no eye/suppress (of-fsl.14)', () => {
    const refGeom = [
      { id: 1, name: 'Plane1', kind: 'plane', entity: { kind: 'plane' } },
      { id: 2, name: 'Axis1', kind: 'axis', entity: { kind: 'axis' } },
    ];
    const html = render({
      features: [...buildReferenceFeatures(refGeom), ...sampleFeatures()],
    });
    expect(html).toContain('Plane1');
    expect(html).toContain('Axis1');
    expect(html).toContain('Delete Plane1');
    // Datums are not part of the mesh recompute: no hide/suppress affordances.
    expect(html).not.toContain('Hide Plane1');
    expect(html).not.toContain('Suppress Plane1');
    // Model features below still get the full action set.
    expect(html).toContain('Hide Extrude1');
  });

  it('marks the selected reference row via selectedRefId', () => {
    const refGeom = [{ id: 7, name: 'Plane1', kind: 'plane', entity: { kind: 'plane' } }];
    const html = render({
      features: buildReferenceFeatures(refGeom),
      selectedRefId: 7,
    });
    expect(html).toContain('selected');
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
