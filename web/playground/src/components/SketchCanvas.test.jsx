/**
 * Server-render smoke tests: catch reference errors and broken JSX in the
 * render path without a browser. Interaction logic lives in lib/sketch/ and
 * is unit-tested there.
 */
import { describe, expect, it } from 'vitest';
import { renderToString } from 'react-dom/server';
import SketchCanvas from './SketchCanvas.jsx';

describe('SketchCanvas', () => {
  it('renders the overlay with toolbar, plane picker, and status', () => {
    const html = renderToString(
      <SketchCanvas
        open
        plane="XY"
        onPlaneChange={() => {}}
        onProfileChange={() => {}}
      />
    );
    expect(html).toContain('sketch-overlay');
    expect(html).toContain('sketch-toolbar');
    for (const label of ['Select', 'Line', 'Rect', 'Circle', 'Arc', 'Extrude', 'Revolve']) {
      expect(html).toContain(label);
    }
    for (const plane of ['XY', 'XZ', 'YZ']) {
      expect(html).toContain(plane);
    }
    // Empty sketch reports an open profile.
    expect(html).toContain('empty sketch');
  });

  it('renders sketch-mode controls: Finish, undo/redo, and the tool chip', () => {
    const html = renderToString(
      <SketchCanvas
        open
        plane="XY"
        onPlaneChange={() => {}}
        onProfileChange={() => {}}
      />
    );
    expect(html).toContain('Finish');
    expect(html).toContain('Undo (Cmd/Ctrl+Z)');
    expect(html).toContain('Redo (Shift+Cmd/Ctrl+Z)');
    // The active tool is always visible in the status bar.
    expect(html).toContain('tool-chip');
    expect(html).toContain('sketch-hint');
  });

  it('lists reference planes in the plane picker (of-fsl.14)', () => {
    const refPlanes = [
      { id: 1, name: 'Plane1', plane: { kind: 'plane', name: 'Plane1' } },
    ];
    const html = renderToString(
      <SketchCanvas
        open
        plane="XY"
        refPlanes={refPlanes}
        onPlaneChange={() => {}}
        onProfileChange={() => {}}
      />
    );
    expect(html).toContain('Plane1');
    expect(html).toContain('Sketch on reference Plane1');
  });

  it('labels a reference plane instead of showing the Face chip (of-fsl.14)', () => {
    const refPlane = { kind: 'plane', name: 'Plane2' };
    const html = renderToString(
      <SketchCanvas
        open
        plane={refPlane}
        refPlanes={[{ id: 2, name: 'Plane2', plane: refPlane }]}
        onPlaneChange={() => {}}
        onProfileChange={() => {}}
      />
    );
    // A persistent reference plane is not an ephemeral face pick.
    expect(html).not.toContain('>Face<');
  });

  it('renders hidden when closed', () => {
    const html = renderToString(
      <SketchCanvas
        open={false}
        plane="XZ"
        onPlaneChange={() => {}}
        onProfileChange={() => {}}
      />
    );
    expect(html).toContain('hidden');
  });
});
