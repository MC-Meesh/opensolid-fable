// Drawing-mode overlay (of-fsl.26.2 + .26.3, DRAWINGS.md §7).
//
// A full-canvas SVG overlay that sits over the 3D viewport — parallel to
// SketchCanvas — showing a 2D orthographic drawing of the current body. It
// projects the mesh into the standard views (project.js), lays them out on a
// sheet (sheet.js), and draws the placed line-work with the same pan/zoom math
// the sketch overlay uses (sketchView.js). MVP: visible edges only (no HLR).
//
// of-fsl.26.3 layers on manual dimensions and SVG export: Linear / Radius tools
// snap to view vertices (12px) and drop driven, static dimensions (dimensions.js);
// Export SVG serializes the placed sheet + dims (svg.js) to a downloaded file.
//
// The pan/zoom view `{ cx, cy, scale }` (sheet center + px per sheet unit) is
// owned by the parent, mirroring how App owns the sketch view; this overlay
// pans/zooms it and self-fits the sheet when first opened.

import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { createSheet, DEFAULT_VIEWS, fitView } from '../lib/drawing/sheet.js';
import {
  defaultMetrics,
  dimensionGeometry,
} from '../lib/drawing/dimensions.js';
import { sheetToSvg } from '../lib/drawing/svg.js';
import { sketchWorldToScreen } from '../lib/sketchView.js';

const MIN_SCALE = 0.2;
const MAX_SCALE = 20000;
const DEFAULT_VIEW = { cx: 0, cy: 0, scale: 60 };
const SNAP_PX = 12; // vertex snap radius when placing a dimension anchor

// Human labels for the view chips, in the standard reading order.
const VIEW_LABELS = {
  front: 'Front',
  top: 'Top',
  right: 'Right',
  iso: 'Iso',
};

const TOOLS = [
  { id: 'pan', label: 'Pan', hint: 'Drag to pan, scroll to zoom' },
  { id: 'linear', label: 'Linear', hint: 'Click two vertices for a distance' },
  { id: 'radius', label: 'Radius', hint: 'Click center then rim for a radius' },
];

export default function DrawingCanvas({ open, mesh, view, onViewChange, onExit }) {
  const svgRef = useRef(null);
  const [size, setSize] = useState({ w: 0, h: 0 });
  const [scale, setScale] = useState(1); // sheet units per model unit
  const [angle, setAngle] = useState('third');
  const [activeViews, setActiveViews] = useState(DEFAULT_VIEWS);
  const [tool, setTool] = useState('pan');
  const [dims, setDims] = useState([]);
  const [pending, setPending] = useState(null); // first-picked anchor, awaiting second

  const v = view ?? DEFAULT_VIEW;
  const viewRef = useRef(v);
  viewRef.current = v;
  const onViewChangeRef = useRef(onViewChange);
  onViewChangeRef.current = onViewChange;

  const sheet = useMemo(
    () => createSheet(mesh, { views: activeViews, scale, angle }),
    [mesh, activeViews, scale, angle]
  );

  // Dimensions anchor to sheet-space coordinates, which are re-derived whenever
  // the layout (scale/angle/view set) changes — stale dims would float free, so
  // relaying-out the sheet clears them (v1 dims are static; DRAWINGS.md §8).
  const layoutKey = `${scale}|${angle}|${activeViews.join(',')}`;
  const layoutRef = useRef(layoutKey);
  useEffect(() => {
    if (layoutRef.current !== layoutKey) {
      layoutRef.current = layoutKey;
      setDims([]);
      setPending(null);
    }
  }, [layoutKey]);

  // Metrics (arrow/label sizes) scaled to the sheet diagonal — shared by the
  // on-canvas render and the SVG export so they match.
  const metrics = useMemo(() => {
    const b = sheet.bounds;
    const diag = b ? Math.hypot(b.maxX - b.minX, b.maxY - b.minY) : 0;
    return defaultMetrics(diag);
  }, [sheet]);

  // Flat list of snap targets: every view vertex in sheet coordinates.
  const snapPoints = useMemo(() => {
    const pts = [];
    for (const pv of sheet.views) {
      for (const seg of pv.segments) for (const p of seg.pts) pts.push(p);
    }
    return pts;
  }, [sheet]);

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

  const toScreen = useCallback(
    (x, y) => sketchWorldToScreen(v, size, x, y),
    [v, size]
  );

  // Nearest snap vertex to a screen point, within SNAP_PX; null if none close.
  const snapAt = useCallback(
    (sx, sy) => {
      let best = null;
      let bestD = SNAP_PX;
      for (const p of snapPoints) {
        const [px, py] = toScreen(p[0], p[1]);
        const d = Math.hypot(px - sx, py - sy);
        if (d < bestD) {
          bestD = d;
          best = p;
        }
      }
      return best;
    },
    [snapPoints, toScreen]
  );

  // Place a dimension anchor from a click; two anchors complete a dimension.
  const placePick = useCallback(
    (sx, sy) => {
      const p = snapAt(sx, sy);
      if (!p) return; // v1 requires snapping to a vertex
      if (!pending) {
        setPending({ tool, a: p });
        return;
      }
      if (tool === 'linear') {
        setDims((d) => [...d, { kind: 'linear', a: pending.a, b: p }]);
      } else if (tool === 'radius') {
        setDims((d) => [...d, { kind: 'radius', center: pending.a, rim: p }]);
      }
      setPending(null);
    },
    [snapAt, pending, tool]
  );

  // ---- pointer: pan (pan tool / middle button) or dimension pick ----------
  const dragRef = useRef(null);
  const onPointerDown = useCallback(
    (event) => {
      const isPan = tool === 'pan' || event.button === 1;
      if (event.button === 0 && !isPan) {
        // Dimension pick — snap relative to the SVG element.
        const rect = svgRef.current?.getBoundingClientRect();
        if (rect) placePick(event.clientX - rect.left, event.clientY - rect.top);
        event.preventDefault();
        return;
      }
      if (event.button !== 0 && event.button !== 1) return;
      dragRef.current = {
        startX: event.clientX,
        startY: event.clientY,
        view0: viewRef.current,
      };
      event.target.setPointerCapture?.(event.pointerId);
      event.preventDefault();
    },
    [tool, placePick]
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
    setPending(null);
  }, []);

  // Export the current sheet + dimensions as a downloaded SVG file.
  const exportSvg = useCallback(() => {
    const svg = sheetToSvg(sheet, dims, {
      scale,
      titleBlock: true,
      title: 'Drawing',
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
  }, [sheet, dims, scale]);

  // Keyboard: Esc cancels a pending pick, else leaves drawing mode.
  useEffect(() => {
    if (!open) return undefined;
    const onKey = (event) => {
      if (event.key !== 'Escape') return;
      if (pending) {
        setPending(null);
      } else {
        onExit?.();
      }
    };
    window.addEventListener('keydown', onKey);
    return () => window.removeEventListener('keydown', onKey);
  }, [open, onExit, pending]);

  const empty = sheet.views.length === 0;
  const picking = tool !== 'pan';

  // Expand dimensions to screen-space primitives for the overlay.
  const dimRender = useMemo(
    () =>
      dims.map((dim) => {
        const g = dimensionGeometry(dim, scale, metrics);
        return {
          lines: g.lines.map((ln) => ln.map(([x, y]) => toScreen(x, y))),
          arrows: g.arrows.map((ar) => ar.map(([x, y]) => toScreen(x, y))),
          label: g.label
            ? {
                pos: toScreen(g.label.pos[0], g.label.pos[1]),
                text: g.label.text,
                size: Math.max(g.label.height * v.scale, 8),
              }
            : null,
        };
      }),
    [dims, scale, metrics, toScreen, v.scale]
  );

  return (
    <div className={`drawing-overlay${open ? '' : ' hidden'}`}>
      <svg
        ref={svgRef}
        className={`drawing-svg${picking ? ' picking' : ''}`}
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

        {dimRender.map((d, i) => (
          <g key={`dim-${i}`} className="dim">
            {d.lines.map((ln, j) => (
              <line
                key={`l${j}`}
                className="dim-line"
                x1={ln[0][0]}
                y1={ln[0][1]}
                x2={ln[1][0]}
                y2={ln[1][1]}
              />
            ))}
            {d.arrows.map((ar, j) => (
              <polygon
                key={`a${j}`}
                className="dim-arrow"
                points={ar.map(([x, y]) => `${x},${y}`).join(' ')}
              />
            ))}
            {d.label && (
              <text
                className="dim-label"
                x={d.label.pos[0]}
                y={d.label.pos[1]}
                fontSize={d.label.size}
                textAnchor="middle"
                dominantBaseline="middle"
              >
                {d.label.text}
              </text>
            )}
          </g>
        ))}

        {pending && (
          <circle
            className="dim-pending"
            cx={toScreen(pending.a[0], pending.a[1])[0]}
            cy={toScreen(pending.a[0], pending.a[1])[1]}
            r={5}
          />
        )}
      </svg>

      {empty && (
        <div className="drawing-empty">
          Nothing to draw — model a body, then open Drawing.
        </div>
      )}

      <div className="drawing-toolbar">
        <div className="group">
          <span className="group-label">Tool</span>
          {TOOLS.map((t) => (
            <button
              key={t.id}
              className={`tool-btn${tool === t.id ? ' active' : ''}`}
              onClick={() => {
                setTool(t.id);
                setPending(null);
              }}
              title={t.hint}
            >
              {t.label}
            </button>
          ))}
        </div>
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
          <button
            className="tool-btn"
            onClick={clearDims}
            disabled={dims.length === 0 && !pending}
            title="Remove all dimensions"
          >
            Clear dims
          </button>
          <button className="tool-btn" onClick={exportSvg} title="Download the drawing as SVG">
            Export SVG
          </button>
        </div>
        <div className="group">
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
        <span className="drawing-hint">
          {picking
            ? pending
              ? 'Click the second vertex · Esc cancels'
              : 'Click a vertex to start · snaps to view corners'
            : 'Drag to pan · scroll to zoom · visible edges only (HLR coming)'}
        </span>
      </div>
    </div>
  );
}
