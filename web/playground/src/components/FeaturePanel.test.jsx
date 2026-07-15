/**
 * Server-render smoke tests, matching SectionPanel.test.jsx: catch reference
 * errors and check the controls render for each feature kind. The pattern
 * argument math itself is unit-tested in lib/pattern.test.js.
 */
import { describe, expect, it } from 'vitest';
import { renderToString } from 'react-dom/server';
import FeaturePanel from './FeaturePanel.jsx';

const noop = () => {};

function render(feature, error) {
  return renderToString(
    <FeaturePanel
      feature={feature}
      error={error}
      onChange={noop}
      onApply={noop}
      onCancel={noop}
    />
  );
}

const LINEAR = { kind: 'linearPattern', dx: 2, dy: 0, dz: 0, count: 3 };
const CIRCULAR = {
  kind: 'circularPattern',
  ax: 0,
  ay: 1,
  az: 0,
  cx: 0,
  cy: 0,
  cz: 0,
  count: 6,
  angleDeg: 360,
};
const MIRROR = { kind: 'mirror', nx: 1, ny: 0, nz: 0, px: 0, py: 0, pz: 0 };

describe('FeaturePanel', () => {
  it('renders nothing without a pending feature', () => {
    expect(render(null)).toBe('');
  });

  it('renders the direction presets and count for a linear pattern', () => {
    const html = render(LINEAR);
    expect(html).toContain('Linear Pattern');
    expect(html).toContain('Direction');
    expect(html).toContain('Count');
    for (const axis of ['>X<', '>Y<', '>Z<']) {
      expect(html).toContain(axis);
    }
  });

  it('renders the axis, center, count, and angle for a circular pattern', () => {
    const html = render(CIRCULAR);
    expect(html).toContain('Circular Pattern');
    expect(html).toContain('Axis');
    expect(html).toContain('Count');
    expect(html).toContain('Angle');
  });

  it('renders the plane normal and point for a mirror', () => {
    const html = render(MIRROR);
    expect(html).toContain('Mirror');
    expect(html).toContain('Plane normal');
  });

  it('shows the error and disables Apply when the feature is invalid', () => {
    const html = render({ ...LINEAR, count: 0 }, 'count must be at least 1');
    expect(html).toContain('count must be at least 1');
    expect(html).toContain('disabled');
  });

  it('notes when the plane came from a picked face', () => {
    expect(render({ ...MIRROR, picked: true })).toContain('Using the picked face plane.');
  });
});
