/**
 * Server-render smoke tests for the orientation triad. The projection math
 * itself is unit-tested in lib/triad.test.js.
 */
import { describe, expect, it } from 'vitest';
import { renderToString } from 'react-dom/server';
import ViewTriad from './ViewTriad.jsx';

const IDENTITY = [0, 0, 0, 1];

describe('ViewTriad', () => {
  it('renders one labeled tip per world axis', () => {
    const html = renderToString(<ViewTriad quat={IDENTITY} onSelectView={() => {}} />);
    expect(html).toContain('view-triad');
    expect(html).toContain('>X</text>');
    expect(html).toContain('>Y</text>');
    expect(html).toContain('>Z</text>');
    expect((html.match(/triad-tip/g) ?? []).length).toBe(3);
  });

  it('names the snapped view in each tip tooltip', () => {
    const html = renderToString(<ViewTriad quat={IDENTITY} onSelectView={() => {}} />);
    expect(html).toContain('right view');
    expect(html).toContain('top view');
    expect(html).toContain('front view');
  });
});
