// Drawing-mode overlay (of-fsl.26.2 + of-fsl.26.3, DRAWINGS.md §4, §6, §7).
//
// A full-canvas SVG overlay that sits over the 3D viewport — parallel to
// SketchCanvas — showing a 2D orthographic drawing of the current body. It
// projects the mesh into the standard views (project.js), lays them out on a
// sheet (sheet.js), and draws the placed line-work with the same pan/zoom math
// the sketch overlay uses (sketchView.js).
//
// of-fsl.26.3 adds the two remaining MVP pieces: manual **driven dimensions**
// (linear point-to-point + radius, dimensions.js) placed by clicking view
// vertices, and **SVG export** (svg.js). Dimensions are static (v1): the anchor
// points are the clicked sheet coordinates and the value is measured once —
// associative re-resolve is deferred (§8 item 4). HLR is still deferred (§8);
// every projected edge draws solid.
//
// The pan/zoom view `{ cx, cy, scale }` (sheet center + px per sheet unit) is
// owned by the parent, mirroring how App owns the sketch view; this overlay
// pans/zooms it and self-fits the sheet when first opened.

import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { createSheet, DEFAULT_VIEWS, fitView } from '../lib/drawing/sheet.js';
import { sketchWorldToScreen, sketchScreenToWorld } from '../lib/sketchView.js';
import {
  createLinearDim,
  createRadiusDim,
  dimensionGeometry,
} from '../lib/drawing/dimensions.js';
import { sheetToSvg } from '../lib/drawing/svg.js';

const MIN_SCALE = 0.2;
const MAX_SCALE = 20000;
const DEFAULT_VIEW = { cx: 0, cy: 0, scale: 60 };
// Snap radius (screen px) for anchoring a dimension pick to a view vertex.
const SNAP_PX = 12;

// Human labels for the view chips, in the standard reading order.
const VIEW_LABELS = {
  front: 'Front',
  top: 'Top',
  right: 'Right',
  iso: 'Iso',
};

// The active pointer tool. 'pan' is the default navigate mode; the dimension
// tools place a linear or radius dimension by clicking view vertices.
const TOOLS = [
  { id: 'pan', label: 'Pan', hint: 'Drag to pan · scroll to zoom' },
  {
    id: 'linear',
    label: 'Linear',
    hint: 'Click two points, then click to place the dimension line',
  },
  {
    id: 'radius',
    label: 'Radius',
    hint: 'Click the arc center, then a point on the arc',
  },
];

export default function DrawingCanvas({ open, mesh, view, onViewChange, onExit }) {
  const svgRef = useRef(null);
  const [size, setSize] = useState({ w: 0, h: 0 });
  const [scale, setScale] = useState(1); // sheet units per model unit
  const [angle, setAngle] = useState('third');
  const [activeViews, setActiveViews] = useState(DEFAULT_VIEWS);
  const [tool, setTool] = useState('pan');
  const [dims, setDims] = useState([]);
  const [picks, setPicks] = useState([]); // in-progress dimension anchors (sheet coords)

  const v = view ?? DEFAULT_VIEW;
  const viewRef = useRef(v);
  viewRef.current = v;
  const onViewChangeRef = useRef(onViewChange);
  onViewChangeRef.current = onViewChange;

  const sheet = useMemo(
    () => createSheet(mesh, { views: activeViews, scale, angle }),
    [mesh, activeViews, scale, angle]
  );

  // Snap targets: every vertex of every placed view, in sheet coordinates.
  const vertices = useMemo(() => {
    const out = [];
    for (const placed of sheet.views) {
      for (const seg of placed.segments) {
        for (const p of seg.pts) out.push(p);
      }
    }
    return out;
  }, [sheet]);

  // Dimensions and in-progress picks are tied to this sheet layout (static v1);
  // rebuild invalidates them, so clear when the geometry basis changes.
  useEffect(() => {
    setDims([]);
    setPicks([]);
  }, [mesh, scale, angle, activeViews]);

  // ---- resize --------------------------------------------------------------
  useEffect(() => {
    const el = svgRef.current;
    if (!el) return undefined;
    const observer = new ResizeObserver(() => {
      setSize({ w: el.clientWidth, h: el.clientHeight });
    });
    observer.observe(el);
    setSize({ w: el.clientWidth, h: el.clientHeight });
    return () => observer.disconnect();
  }, []);

  // Self-fit: frame the sheet whenever the overlay opens (once we know the
  // viewport size and have geometry). A ref guards against re-fitting on every
  // pan while open.
  const fittedRef = useRef(false);
  useEffect(() => {
    if (!open) {
      fittedRef.current = false;
      return;
    }
    if (fittedRef.current || size.w === 0 || !sheet.bounds) return;
    fittedRef.current = true;
    onViewChangeRef.current?.(fitView(sheet.bounds, size));
  }, [open, size, sheet]);

  const fitToSheet = useCallback(() => {
    onViewChangeRef.current?.(fitView(sheet.bounds, size));
  }, [sheet, size]);

  // ---- wheel zoom (cursor-anchored) ---------------------------------------
  useEffect(() => {
    const el = svgRef.current;
    if (!el) return undefined;
    const onWheel = (event) => {
      event.preventDefault();
      const rect = el.getBoundingClientRect();
      const sx = event.clientX - rect.left;
      const sy = event.clientY - rect.top;
      const cur = viewRef.current;
      const factor = Math.exp(-event.deltaY * 0.0015);
      const scl = Math.min(MAX_SCALE, Math.max(MIN_SCALE, cur.scale * factor));
      const wx = (sx - el.clientWidth / 2) / cur.scale + cur.cx;
      const wy = (el.clientHeight / 2 - sy) / cur.scale + cur.cy;
      onViewChangeRef.current?.({
        scale: scl,
        cx: wx - (sx - el.clientWidth / 2) / scl,
        cy: wy - (el.clientHeight / 2 - sy) / scl,
      });
    };
    el.addEventListener('wheel', onWheel, { passive: false });
    return () => el.removeEventListener('wheel', onWheel);
  }, []);

  // Sheet coordinates of a pointer event, snapped to the nearest view vertex
  // within SNAP_PX when the pointer is close enough (for dimension anchoring).
  const eventSheetPoint = useCallback(
    (event, snap) => {
      const el = svgRef.current;
      const rect = el.getBoundingClientRect();
      const sx = event.clientX - rect.left;
      const sy = event.clientY - rect.top;
      const cur = viewRef.current;
      const world = sketchScreenToWorld(cur, size, sx, sy);
      const raw = [world.x, world.y];
      if (!snap) return raw;
      let best = null;
      let bestD = SNAP_PX;
      for (const p of vertices) {
        const [px, py] = sketchWorldToScreen(cur, size, p[0], p[1]);
        const d = Math.hypot(px - sx, py - sy);
        if (d < bestD) {
          bestD = d;
          best = p;
        }
      }
      return best ? [best[0], best[1]] : raw;
    },
    [size, vertices]
  );

  // Advance a dimension placement with one more pick (in sheet coordinates).
  // Completing a dimension appends it and resets the pick buffer; the state
  // setters stay pure (no setState nested inside an updater).
  const addPick = useCallback(
    (pt) => {
      const next = [...picks, pt];
      if (tool === 'radius' && next.length === 2) {
        const [c, rim] = next;
        const r = Math.hypot(rim[0] - c[0], rim[1] - c[1]);
        setDims((ds) => [...ds, createRadiusDim(c, rim, r * 0.4, scale)]);
        setPicks([]);
        return;
      }
      if (tool === 'linear' && next.length === 3) {
        const [a, b, off] = next;
        const dir = [b[0] - a[0], b[1] - a[1]];
        const len = Math.hypot(dir[0], dir[1]) || 1;
        const nrm = [-dir[1] / len, dir[0] / len];
        const offset = (off[0] - a[0]) * nrm[0] + (off[1] - a[1]) * nrm[1];
        setDims((ds) => [...ds, createLinearDim(a, b, offset, scale)]);
        setPicks([]);
        return;
      }
      setPicks(next);
    },
    [tool, scale, picks]
  );

  // ---- pan (drag) / dimension pick -----------------------------------------
  const dragRef = useRef(null);
  const onPointerDown = useCallback(
    (event) => {
      // Middle button always pans; left button pans only in the pan tool, and
      // places a dimension anchor in the dimension tools.
      const panning = event.button === 1 || (event.button === 0 && tool === 'pan');
      if (panning) {
        dragRef.current = {
          startX: event.clientX,
          startY: event.clientY,
          view0: viewRef.current,
        };
        event.target.setPointerCapture?.(event.pointerId);
        event.preventDefault();
        return;
      }
      if (event.button === 0 && tool !== 'pan') {
        // Linear: 3rd pick (the offset point) is free; anchors snap to vertices.
        const snap = !(tool === 'linear' && picks.length === 2);
        addPick(eventSheetPoint(event, snap));
        event.preventDefault();
      }
    },
    [tool, picks.length, addPick, eventSheetPoint]
  );

  const onPointerMove = useCallback((event) => {
    const drag = dragRef.current;
    if (!drag) return;
    const dx = (event.clientX - drag.startX) / drag.view0.scale;
    const dy = (event.clientY - drag.startY) / drag.view0.scale;
    onViewChangeRef.current?.({
      ...drag.view0,
      cx: drag.view0.cx - dx,
      cy: drag.view0.cy + dy,
    });
  }, []);

  const onPointerUp = useCallback((event) => {
    if (dragRef.current) {
      event.target.releasePointerCapture?.(event.pointerId);
      dragRef.current = null;
    }
  }, []);

  const toggleView = useCallback((name) => {
    setActiveViews((prev) =>
      prev.includes(name)
        ? prev.filter((n) => n !== name)
        : DEFAULT_VIEWS.filter((n) => prev.includes(n) || n === name)
    );
  }, []);

  const clearDims = useCallback(() => {
    setDims([]);
    setPicks([]);
  }, []);

  // Export the current sheet + dimensions as an SVG download (DRAWINGS.md §6).
  const exportSvg = useCallback(() => {
    if (!sheet.bounds) return;
    const svg = sheetToSvg(sheet, dims, {
      title: 'Drawing',
      titleBlock: true,
      date: new Date().toISOString().slice(0, 10),
    });
    const blob = new Blob([svg], { type: 'image/svg+xml' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = 'drawing.svg';
    document.body.appendChild(a);
    a.click();
    a.remove();
    URL.revokeObjectURL(url);
  }, [sheet, dims]);

  // Keyboard: Esc cancels an in-progress pick, else leaves drawing mode.
  useEffect(() => {
    if (!open) return undefined;
    const onKey = (event) => {
      if (event.key !== 'Escape') return;
      if (picks.length > 0) setPicks([]);
      else onExit?.();
    };
    window.addEventListener('keydown', onKey);
    return () => window.removeEventListener('keydown', onKey);
  }, [open, onExit, picks.length]);

  const toScreen = useCallback(
    (x, y) => sketchWorldToScreen(v, size, x, y),
    [v, size]
  );

  // Dimension visual metrics kept ~constant on screen: convert px to sheet units
  // through the current pan/zoom scale.
  const dimSizes = useMemo(() => {
    const s = v.scale || 1;
    return { arrow: 9 / s, ext: 6 / s, gap: 2 / s, textGap: 12 / s };
  }, [v.scale]);

  const empty = sheet.views.length === 0;
  const activeTool = TOOLS.find((t) => t.id === tool) ?? TOOLS[0];

  return (
    <div className={`drawing-overlay${open ? '' : ' hidden'}`}>
      <svg
        ref={svgRef}
        className={`drawing-svg${tool !== 'pan' ? ' picking' : ''}`}
        onPointerDown={onPointerDown}
        onPointerMove={onPointerMove}
        onPointerUp={onPointerUp}
        onContextMenu={(event) => event.preventDefault()}
      >
        {sheet.views.map((placed) => {
          const b = placed.bounds;
          const [lx, ly] = toScreen(b.minX, b.minY); // bottom-left in world → screen
          return (
            <g key={placed.view} className="drawing-view">
              {placed.segments.map((seg, i) => {
                const pts = seg.pts
                  .map(([x, y]) => {
                    const [sx, sy] = toScreen(x, y);
                    return `${sx},${sy}`;
                  })
                  .join(' ');
                return (
                  <polyline
                    key={i}
                    className={`edge ${seg.style}`}
                    points={pts}
                    fill="none"
                  />
                );
              })}
              <text className="view-label" x={lx} y={ly + 16}>
                {VIEW_LABELS[placed.view] ?? placed.view}
              </text>
            </g>
          );
        })}

        {/* Placed dimensions */}
        <g className="drawing-dims">
          {dims.map((dim) => {
            const g = dimensionGeometry(dim, dimSizes);
            const lines = g.lines.map((line, i) => {
              const [[x0, y0], [x1, y1]] = line;
              const [sx0, sy0] = toScreen(x0, y0);
              const [sx1, sy1] = toScreen(x1, y1);
              return (
                <line key={i} className="dim-line" x1={sx0} y1={sy0} x2={sx1} y2={sy1} />
              );
            });
            const heads = g.arrowheads.map((head, i) => {
              const pts = head
                .map(([x, y]) => {
                  const [sx, sy] = toScreen(x, y);
                  return `${sx},${sy}`;
                })
                .join(' ');
              return <polygon key={i} className="dim-arrow" points={pts} />;
            });
            let label = null;
            if (g.text) {
              const [sx, sy] = toScreen(g.text.pos[0], g.text.pos[1]);
              const deg = (-g.text.angle * 180) / Math.PI;
              label = (
                <text
                  className="dim-label"
                  x={sx}
                  y={sy}
                  textAnchor="middle"
                  transform={deg !== 0 ? `rotate(${deg} ${sx} ${sy})` : undefined}
                >
                  {g.text.label}
                </text>
              );
            }
            return (
              <g key={dim.id} className="dim">
                {lines}
                {heads}
                {label}
              </g>
            );
          })}

          {/* In-progress pick markers */}
          {picks.map((p, i) => {
            const [sx, sy] = toScreen(p[0], p[1]);
            return <circle key={i} className="dim-pick" cx={sx} cy={sy} r={4} />;
          })}
        </g>
      </svg>

      {empty && (
        <div className="drawing-empty">
          Nothing to draw — model a body, then open Drawing.
        </div>
      )}

      <div className="drawing-toolbar">
        <div className="group">
          <span className="group-label">Views</span>
          {DEFAULT_VIEWS.map((name) => (
            <button
              key={name}
              className={`tool-btn${activeViews.includes(name) ? ' active' : ''}`}
              onClick={() => toggleView(name)}
              title={`Toggle the ${VIEW_LABELS[name]} view`}
            >
              {VIEW_LABELS[name]}
            </button>
          ))}
        </div>
        <div className="group">
          <span className="group-label">Angle</span>
          <button
            className="tool-btn"
            onClick={() => setAngle((a) => (a === 'third' ? 'first' : 'third'))}
            title="Toggle first-angle / third-angle projection layout"
          >
            {angle === 'third' ? 'Third-angle' : 'First-angle'}
          </button>
        </div>
        <div className="group">
          <span className="group-label">Scale</span>
          <button className="tool-btn" onClick={() => setScale((s) => s / 2)} title="Halve the drawing scale">
            −
          </button>
          <span className="scale-value">{scale}:1</span>
          <button className="tool-btn" onClick={() => setScale((s) => s * 2)} title="Double the drawing scale">
            +
          </button>
        </div>
        <div className="group">
          <span className="group-label">Tool</span>
          {TOOLS.map((t) => (
            <button
              key={t.id}
              className={`tool-btn${tool === t.id ? ' active' : ''}`}
              onClick={() => {
                setTool(t.id);
                setPicks([]);
              }}
              title={t.hint}
            >
              {t.label}
            </button>
          ))}
        </div>
        <div className="group">
          <button
            className="tool-btn"
            onClick={clearDims}
            disabled={dims.length === 0 && picks.length === 0}
            title="Remove all dimensions"
          >
            Clear dims
          </button>
          <button
            className="tool-btn"
            onClick={exportSvg}
            disabled={empty}
            title="Export the sheet as SVG"
          >
            Export SVG
          </button>
          <button className="tool-btn" onClick={fitToSheet} title="Fit the sheet in the view (F)">
            Fit
          </button>
          <button className="tool-btn primary" onClick={onExit} title="Leave drawing mode (Esc)">
            Finish
          </button>
        </div>
      </div>

      <div className="drawing-status">
        <span className="tool-chip">Drawing</span>
        <span className="drawing-hint">{activeTool.hint} · visible edges only</span>
      </div>
    </div>
  );
}
