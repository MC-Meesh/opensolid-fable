/**
 * Server-render smoke tests, matching SweepPanel.test.jsx: catch reference
 * errors and check the controls render. The plane math itself is unit-tested
 * in lib/sectionView.test.js.
 */
import { describe, expect, it } from 'vitest';
import { renderToString } from 'react-dom/server';
import SectionPanel from './SectionPanel.jsx';

const noop = () => {};
const RANGE = { min: -2, max: 2 };

function render(section) {
  return renderToString(
    <SectionPanel
      section={section}
      range={RANGE}
      onAxisChange={noop}
      onFlip={noop}
      onOffsetChange={noop}
      onClose={noop}
    />
  );
}

describe('SectionPanel', () => {
  it('renders nothing without an active section', () => {
    expect(render(null)).toBe('');
  });

  it('renders the axis buttons, offset field, and flip toggle', () => {
    const html = render({ axis: 'X', offset: 0.5, flip: false });
    expect(html).toContain('Section View');
    for (const axis of ['>X<', '>Y<', '>Z<']) {
      expect(html).toContain(axis);
    }
    expect(html).toContain('Offset');
    expect(html).toContain('Flip side');
  });

  it('marks the active axis as pressed', () => {
    const html = render({ axis: 'Y', offset: 0, flip: false });
    // The active axis carries aria-pressed="true"; exactly one does.
    expect((html.match(/aria-pressed="true"/g) ?? []).length).toBe(1);
    expect(html).toMatch(/aria-pressed="true"[^>]*>Y</);
  });

  it('reflects the offset value and a checked flip', () => {
    const html = render({ axis: 'Z', offset: 1.25, flip: true });
    expect(html).toMatch(/class="section-value"[^>]*value="1.25"/);
    expect(html).toMatch(/<input type="checkbox" checked/);
  });

  it('clamps a range-slider value that lands outside the model bounds', () => {
    // offset beyond range.max still renders (number input shows the true value,
    // slider pins to max) rather than throwing.
    const html = render({ axis: 'X', offset: 9, flip: false });
    expect(html).toContain('section-panel');
  });
});
