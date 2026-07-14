/**
 * Server-render tests: length fields follow the document unit while
 * angle/scale fields keep their intrinsic unit.
 */
import { describe, expect, it } from 'vitest';
import { renderToString } from 'react-dom/server';
import PropertyPanel from './PropertyPanel.jsx';

const noop = () => {};

function render(node, documentUnit) {
  return renderToString(
    <PropertyPanel
      node={node}
      disabled={false}
      onEditArg={noop}
      onChangeOp={noop}
      documentUnit={documentUnit}
    />
  );
}

describe('PropertyPanel document units', () => {
  const box = { id: 'n1', op: 'box3', args: [1, 0.5, 0.8] };

  it('labels length fields in the document unit', () => {
    const html = render(box, 'in');
    expect(html).toContain('>in</span>');
    expect(html).not.toContain('>mm</span>');
  });

  it('defaults length fields to millimetres', () => {
    const html = render(box, undefined);
    expect(html).toContain('>mm</span>');
  });

  it('leaves angle fields in degrees regardless of the document unit', () => {
    const rotate = { id: 'n2', op: 'rotate', args: [0, 0, 1, Math.PI / 4] };
    const html = render(rotate, 'in');
    expect(html).toContain('>°</span>');
    // Axis components are unitless; the rotation stays in degrees, so no
    // length unit leaks in.
    expect(html).not.toContain('>in</span>');
  });

  it('does not rescale the stored value when the unit changes', () => {
    // The half-extent 1 is shown verbatim in both units (metadata, not a
    // conversion).
    expect(render(box, 'mm')).toContain('value="1"');
    expect(render(box, 'in')).toContain('value="1"');
  });
});
