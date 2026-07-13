/**
 * Server-render smoke tests, matching SketchCanvas.test.jsx: catch reference
 * errors and broken JSX in the render path without a browser. The sweep
 * math itself is unit-tested in lib/sweep.test.js.
 */
import { describe, expect, it } from 'vitest';
import { renderToString } from 'react-dom/server';
import SweepPanel from './SweepPanel.jsx';

const noop = () => {};

describe('SweepPanel', () => {
  it('renders nothing without a pending sweep', () => {
    expect(
      renderToString(
        <SweepPanel sweep={null} error={null} onChange={noop} onApply={noop} onCancel={noop} />
      )
    ).toBe('');
  });

  it('renders extrude with a height field and actions', () => {
    const html = renderToString(
      <SweepPanel
        sweep={{ kind: 'extrude', plane: 'XY', ops: null, value: 2, range: 8 }}
        error={null}
        onChange={noop}
        onApply={noop}
        onCancel={noop}
      />
    );
    expect(html).toContain('Extrude');
    expect(html).toMatch(/<span class="sweep-plane">XY/);
    expect(html).toContain('Height');
    expect(html).toContain('Flip direction');
    expect(html).toContain('Apply');
    expect(html).toContain('Cancel');
  });

  it('shows the magnitude and checks Flip for reverse extrudes', () => {
    const html = renderToString(
      <SweepPanel
        sweep={{ kind: 'extrude', plane: 'XY', ops: null, value: -2, range: 8 }}
        error={null}
        onChange={noop}
        onApply={noop}
        onCancel={noop}
      />
    );
    expect(html).toMatch(/class="sweep-value"[^>]*value="2"/);
    expect(html).toMatch(/<input type="checkbox" checked/);
  });

  it('labels a face-plane sketch', () => {
    const face = {
      origin: [0, 0, 0],
      normal: [0, 0, 1],
      u: [1, 0, 0],
      v: [0, 1, 0],
      extent: 1,
    };
    const html = renderToString(
      <SweepPanel
        sweep={{ kind: 'extrude', plane: face, ops: null, value: 2, range: 8 }}
        error={null}
        onChange={noop}
        onApply={noop}
        onCancel={noop}
      />
    );
    expect(html).toMatch(/<span class="sweep-plane">Face/);
  });

  it('renders revolve with an angle field and shows errors', () => {
    const html = renderToString(
      <SweepPanel
        sweep={{ kind: 'revolve', plane: 'XZ', ops: null, value: 360, range: 360 }}
        error="revolve profile must lie in u >= 0"
        onChange={noop}
        onApply={noop}
        onCancel={noop}
      />
    );
    expect(html).toContain('Revolve');
    expect(html).toContain('Angle');
    expect(html).toContain('u &gt;= 0');
    // Apply is disabled while the sweep is invalid.
    expect(html).toMatch(/<button[^>]*disabled[^>]*>Apply<\/button>/);
  });
});
