// Main viewport toolbar, grouped by workflow like the SolidWorks CommandManager:
// Sketch | Features | View | Export, plus an overflow menu for the rarely
// touched meshing settings. Disabled buttons keep a tooltip explaining why.
// One row, no wrapping: view buttons are icon-only (tooltips carry the names)
// so the whole strip fits beside the side panel at 1280px-wide windows.
import MeshSettings from './MeshSettings.jsx';
import { LENGTH_UNITS } from '../lib/units.js';

const ICON = {
  viewBox: '0 0 16 16',
  width: 15,
  height: 15,
  fill: 'none',
  stroke: 'currentColor',
  strokeWidth: 1.4,
  strokeLinecap: 'round',
  strokeLinejoin: 'round',
  'aria-hidden': true,
};

const Icons = {
  undo: (
    <svg {...ICON}>
      <path d="M4 8 H10 c2.2 0 3.5 1.3 3.5 3.2 S12.2 14.4 10 14.4 H6.5" />
      <path d="M6.5 5 L3.5 8 L6.5 11" />
    </svg>
  ),
  redo: (
    <svg {...ICON}>
      <path d="M12 8 H6 c-2.2 0-3.5 1.3-3.5 3.2 S3.8 14.4 6 14.4 H9.5" />
      <path d="M9.5 5 L12.5 8 L9.5 11" />
    </svg>
  ),
  sketch: (
    <svg {...ICON}>
      <path d="M2 14 L3 10.5 L11.5 2 L14 4.5 L5.5 13 Z" />
      <path d="M10 3.5 L12.5 6" />
    </svg>
  ),
  extrude: (
    <svg {...ICON}>
      <rect x="3" y="9.5" width="10" height="4" />
      <path d="M8 8 V2.5 M5.5 5 L8 2.5 L10.5 5" />
    </svg>
  ),
  drawing: (
    <svg {...ICON}>
      <rect x="2.5" y="2.5" width="11" height="11" rx="0.5" />
      <path d="M4.5 4.5 h3 v3 h-3 Z M9 5 h2.5 M9 7 h2.5 M4.5 10 h7" />
    </svg>
  ),
  revolve: (
    <svg {...ICON}>
      <path d="M8 1.5 V14.5" strokeDasharray="2 1.6" />
      <path d="M10.5 4.5 v7 M10.5 11.5 c3 0 4 -1.5 4 -3.5 s-1 -3.5 -4 -3.5" />
      <path d="M12.7 12.6 L10.5 11.5 L12 9.6" />
    </svg>
  ),
  sweep: (
    <svg {...ICON}>
      <path d="M3 13 C3 7 10 9 13 3" strokeDasharray="2 1.5" />
      <ellipse cx="3" cy="13" rx="2" ry="1" />
      <path d="M1 13 V11.5 M5 13 V11.5" />
    </svg>
  ),
  loft: (
    <svg {...ICON}>
      <rect x="2.5" y="11" width="7" height="2.5" />
      <ellipse cx="11" cy="4" rx="2.5" ry="1.2" />
      <path d="M2.5 11 L9 4.2 M9.5 11 L13 4.6" />
    </svg>
  ),
  measure: (
    // A caliper: a graduated bar with two jaws bracketing a span.
    <svg {...ICON}>
      <path d="M2 4.5 L14 4.5" />
      <path d="M2 4.5 V11 M14 4.5 V11" />
      <path d="M4.5 4.5 V6.5 M7 4.5 V6.5 M9.5 4.5 V6.5 M12 4.5 V6.5" />
    </svg>
  ),
  shell: (
    // A hollowed body: the outer solid with the scooped-out void inside it.
    <svg {...ICON}>
      <path d="M2.5 5 L8 2.2 L13.5 5 V11 L8 13.8 L2.5 11 Z" />
      <path d="M5 6.4 L8 4.9 L11 6.4 V9.6 L8 11.1 L5 9.6 Z" />
    </svg>
  ),
  linearPattern: (
    <svg {...ICON}>
      <rect x="2" y="6" width="3.5" height="3.5" />
      <rect x="6.25" y="6" width="3.5" height="3.5" opacity="0.7" />
      <rect x="10.5" y="6" width="3.5" height="3.5" opacity="0.45" />
    </svg>
  ),
  circularPattern: (
    <svg {...ICON}>
      <circle cx="8" cy="8" r="5.2" strokeDasharray="1.6 1.4" />
      <rect x="6.5" y="1.4" width="3" height="3" />
      <rect x="11.6" y="6.5" width="3" height="3" opacity="0.6" />
      <rect x="1.4" y="6.5" width="3" height="3" opacity="0.6" />
    </svg>
  ),
  mirror: (
    <svg {...ICON}>
      <path d="M8 1.5 V14.5" strokeDasharray="2 1.6" />
      <path d="M6 4 L2.5 8 L6 12 Z" />
      <path d="M10 4 L13.5 8 L10 12 Z" opacity="0.55" />
    </svg>
  ),
  fillet: (
    <svg {...ICON}>
      <path d="M3 13 V7 c0 -2.2 1.8 -4 4 -4 H13" />
      <path d="M3 13 H3.02 M13 3 H13.02" strokeWidth="2.4" />
    </svg>
  ),
  fit: (
    <svg {...ICON}>
      <path d="M2 5.5 V2 h3.5 M10.5 2 H14 v3.5 M14 10.5 V14 h-3.5 M5.5 14 H2 v-3.5" />
      <circle cx="8" cy="8" r="2.4" />
    </svg>
  ),
  front: (
    <svg {...ICON}>
      <path d="M3 5 L6 2.5 H13 V9.5 L10 12" opacity="0.55" />
      <rect x="3" y="5" width="7" height="7" fill="currentColor" fillOpacity="0.25" />
    </svg>
  ),
  top: (
    <svg {...ICON}>
      <path d="M3 6 V13 H10 L13 10.5 V3.5" opacity="0.55" />
      <path d="M3 6 L6 3.5 H13 L10 6 Z" fill="currentColor" fillOpacity="0.25" />
    </svg>
  ),
  right: (
    <svg {...ICON}>
      <path d="M10 5 L6.5 2.5 H3 V9.5 L6.5 12" opacity="0.55" />
      <path d="M10 5 L13 3.5 V10.5 L10 12 Z" fill="currentColor" fillOpacity="0.25" />
    </svg>
  ),
  iso: (
    <svg {...ICON}>
      <path d="M8 1.8 L14 5 V11 L8 14.2 L2 11 V5 Z" />
      <path d="M2 5 L8 8.2 L14 5 M8 8.2 V14.2" />
    </svg>
  ),
  wireframe: (
    <svg {...ICON}>
      <path d="M2 13 L8 2.5 L14 13 Z" />
      <path d="M5 13 L8 7.5 L11 13 M6.5 5.2 L8 7.5 L9.5 5.2" />
    </svg>
  ),
  section: (
    <svg {...ICON}>
      <path d="M3 4 H10 V11 H3 Z" opacity="0.55" />
      <path d="M3 4 L6 1.5 H13 V8.5 L10 11" opacity="0.55" />
      <path d="M3 4 H10 V11 H3 Z" fill="currentColor" fillOpacity="0.28" stroke="none" />
      <path d="M1.5 7.5 H11.5" />
    </svg>
  ),
  reference: (
    <svg {...ICON}>
      <path d="M2.5 6 L8 3 L13.5 6 L8 9 Z" fill="currentColor" fillOpacity="0.22" />
      <path d="M8 9 V14 M8 14 L6 12.5 M8 14 L10 12.5" />
    </svg>
  ),
  mass: (
    // A balance scale: the beam with a pan hanging at each end.
    <svg {...ICON}>
      <path d="M8 3 V13.4" />
      <path d="M5.5 13.4 H10.5" />
      <path d="M2.6 4.8 H13.4" />
      <path d="M1.2 7.6 H4 L2.6 4.8 Z" />
      <path d="M12 7.6 H14.8 L13.4 4.8 Z" />
    </svg>
  ),
  stl: (
    <svg {...ICON}>
      <path d="M8 2 v7.5 M5.2 6.8 L8 9.5 L10.8 6.8" />
      <path d="M2.5 12.5 h11" />
    </svg>
  ),
  step: (
    <svg {...ICON}>
      <path d="M2.5 5 L8 2.2 L13.5 5 V11 L8 13.8 L2.5 11 Z" />
      <path d="M2.5 5 L8 7.8 L13.5 5 M8 7.8 V13.8" />
    </svg>
  ),
  menu: (
    <svg {...ICON}>
      <circle cx="3" cy="8" r="1" fill="currentColor" />
      <circle cx="8" cy="8" r="1" fill="currentColor" />
      <circle cx="13" cy="8" r="1" fill="currentColor" />
    </svg>
  ),
};

function ToolButton({ icon, label, title, disabledReason, active, disabled, compact, onClick }) {
  return (
    <span className="tool-wrap" title={disabled ? disabledReason : title}>
      <button
        type="button"
        className={`main-tool${active ? ' active' : ''}`}
        disabled={disabled}
        aria-pressed={active}
        aria-label={label}
        onClick={onClick}
      >
        {Icons[icon]}
        {!compact && <span className="tool-label">{label}</span>}
      </button>
    </span>
  );
}

export default function MainToolbar({
  disabled,
  canUndo = false,
  canRedo = false,
  undoDepth = 0,
  redoDepth = 0,
  onUndo,
  onRedo,
  sketchOpen,
  sketchOnFace = false,
  onSketchToggle,
  drawingOpen,
  onDrawingToggle,
  measureOpen = false,
  onMeasureToggle,
  canSweep,
  sweepDisabledReason,
  onSweep,
  onAddFeature,
  canShell,
  onShell,
  canPattern,
  patternDisabledReason,
  onPattern,
  filletActive = false,
  canFillet = false,
  filletDisabledReason,
  onFilletToggle,
  onView,
  onFit,
  wireframe,
  onWireframeChange,
  section,
  onSectionToggle,
  referenceOpen = false,
  onReferenceToggle,
  massOpen = false,
  onMassPropsToggle = () => {},
  onDownloadStl,
  onDownloadStep,
  documentUnit = 'mm',
  onDocumentUnitChange = () => {},
  exactBooleans,
  onExactBooleansChange,
}) {
  const notReady = 'Still loading the WASM kernel';
  return (
    <div className="main-toolbar" role="toolbar" aria-label="Main toolbar">
      <div className="tool-group" aria-label="Edit">
        <span className="tool-group-label">Edit</span>
        <ToolButton
          icon="undo"
          label="Undo"
          title={canUndo ? `Undo (Ctrl+Z) — ${undoDepth} step${undoDepth === 1 ? '' : 's'}` : 'Nothing to undo'}
          disabledReason={disabled ? notReady : 'Nothing to undo'}
          disabled={disabled || !canUndo}
          compact
          onClick={onUndo}
        />
        <ToolButton
          icon="redo"
          label="Redo"
          title={canRedo ? `Redo (Ctrl+Shift+Z) — ${redoDepth} step${redoDepth === 1 ? '' : 's'}` : 'Nothing to redo'}
          disabledReason={disabled ? notReady : 'Nothing to redo'}
          disabled={disabled || !canRedo}
          compact
          onClick={onRedo}
        />
      </div>
      <div className="tool-sep" />
      <div className="tool-group" aria-label="Sketch">
        <span className="tool-group-label">Sketch</span>
        <ToolButton
          icon="sketch"
          label={sketchOpen ? 'Exit Sketch' : 'Sketch'}
          title={
            sketchOnFace && !sketchOpen
              ? 'Open a 2D sketch on the picked face'
              : 'Open a 2D sketch on a standard plane (or pick a flat face first)'
          }
          disabledReason={disabled ? notReady : 'Exit the drawing first'}
          active={sketchOpen}
          disabled={disabled || drawingOpen}
          onClick={onSketchToggle}
        />
      </div>
      <div className="tool-sep" />
      <div className="tool-group" aria-label="Features">
        <span className="tool-group-label">Features</span>
        <ToolButton
          icon="extrude"
          label="Extrude"
          title="Extrude the closed profile along the plane normal"
          disabledReason={disabled ? notReady : sweepDisabledReason}
          disabled={disabled || !canSweep}
          onClick={() => onSweep('extrude')}
        />
        <ToolButton
          icon="revolve"
          label="Revolve"
          title="Revolve the closed profile around the sketch V axis"
          disabledReason={disabled ? notReady : sweepDisabledReason}
          disabled={disabled || !canSweep}
          onClick={() => onSweep('revolve')}
        />
        <ToolButton
          icon="sweep"
          label="Sweep"
          title="Add a Sweep feature: a profile swept along a 3D path (edit the path in the script)"
          disabledReason={notReady}
          disabled={disabled}
          onClick={() => onAddFeature('sweep')}
        />
        <ToolButton
          icon="loft"
          label="Loft"
          title="Add a Loft feature: a blend between two profiles on parallel planes (edit in the script)"
          disabledReason={notReady}
          disabled={disabled}
          onClick={() => onAddFeature('loft')}
        />
        <ToolButton
          icon="shell"
          label="Shell"
          title="Hollow the selected body (or the whole model) to a uniform wall thickness"
          disabledReason={disabled ? notReady : 'Run a script that produces a solid first'}
          disabled={disabled || !canShell}
          onClick={onShell}
        />
        <ToolButton
          icon="linearPattern"
          label="Linear Pattern"
          title="Repeat the selected body along a direction"
          disabledReason={disabled ? notReady : patternDisabledReason}
          disabled={disabled || !canPattern}
          onClick={() => onPattern('linearPattern')}
        />
        <ToolButton
          icon="circularPattern"
          label="Circular Pattern"
          title="Repeat the selected body around an axis"
          disabledReason={disabled ? notReady : patternDisabledReason}
          disabled={disabled || !canPattern}
          onClick={() => onPattern('circularPattern')}
        />
        <ToolButton
          icon="mirror"
          label="Mirror"
          title="Mirror the selected body across a plane"
          disabledReason={disabled ? notReady : patternDisabledReason}
          disabled={disabled || !canPattern}
          onClick={() => onPattern('mirror')}
        />
        <ToolButton
          icon="fillet"
          label={filletActive ? 'Exit Fillet' : 'Fillet'}
          title="Round or bevel a picked feature edge (click an edge, then set the radius)"
          disabledReason={disabled ? notReady : filletDisabledReason}
          active={filletActive}
          disabled={disabled || (!filletActive && !canFillet)}
          onClick={onFilletToggle}
        />
        <ToolButton
          icon="reference"
          label="Reference"
          title="Reference geometry: datum planes, axes, points, coordinate systems"
          disabledReason={notReady}
          active={referenceOpen}
          disabled={disabled}
          onClick={onReferenceToggle}
        />
      </div>
      <div className="tool-sep" />
      <div className="tool-group" aria-label="Inspect">
        <span className="tool-group-label">Inspect</span>
        <ToolButton
          icon="measure"
          label="Measure"
          title="Measure distance, angle, radius between entities (M)"
          disabledReason={notReady}
          active={measureOpen}
          disabled={disabled}
          onClick={onMeasureToggle}
        />
      </div>
      <div className="tool-sep" />
      <div className="tool-group" aria-label="View">
        <span className="tool-group-label">View</span>
        <ToolButton
          icon="fit"
          label="Fit"
          title="Zoom to fit (F or Space)"
          disabledReason={notReady}
          disabled={disabled}
          onClick={onFit}
        />
        <ToolButton
          icon="front"
          label="Front"
          title="Front view (1)"
          disabledReason={notReady}
          disabled={disabled}
          compact
          onClick={() => onView('front')}
        />
        <ToolButton
          icon="top"
          label="Top"
          title="Top view (5)"
          disabledReason={notReady}
          disabled={disabled}
          compact
          onClick={() => onView('top')}
        />
        <ToolButton
          icon="right"
          label="Right"
          title="Right view (4)"
          disabledReason={notReady}
          disabled={disabled}
          compact
          onClick={() => onView('right')}
        />
        <ToolButton
          icon="iso"
          label="Iso"
          title="Isometric view (7)"
          disabledReason={notReady}
          disabled={disabled}
          compact
          onClick={() => onView('iso')}
        />
        <ToolButton
          icon="wireframe"
          label="Wireframe"
          title="Toggle wireframe rendering"
          disabledReason={notReady}
          active={wireframe}
          disabled={disabled}
          compact
          onClick={() => onWireframeChange(!wireframe)}
        />
        <ToolButton
          icon="section"
          label="Section"
          title="Section view: slice the model with a movable clipping plane"
          disabledReason={notReady}
          active={section}
          disabled={disabled}
          compact
          onClick={onSectionToggle}
        />
        <ToolButton
          icon="mass"
          label="Mass"
          title="Mass properties: volume, surface area, mass, center of mass, and inertia"
          disabledReason={notReady}
          active={massOpen}
          disabled={disabled}
          compact
          onClick={onMassPropsToggle}
        />
      </div>
      <div className="tool-sep" />
      <div className="tool-group" aria-label="Drawing">
        <span className="tool-group-label">Drawing</span>
        <ToolButton
          icon="drawing"
          label={drawingOpen ? 'Exit Drawing' : 'Drawing'}
          title="Open a 2D orthographic drawing of the current body (front/top/right/iso)"
          disabledReason={disabled ? notReady : 'Exit the sketch first'}
          active={drawingOpen}
          disabled={disabled || sketchOpen}
          onClick={onDrawingToggle}
        />
      </div>
      <div className="tool-sep" />
      <div className="tool-group" aria-label="Export">
        <span className="tool-group-label">Export</span>
        <label className="tool-units" title="Document unit: labels dimensions and declares the STEP SI_UNIT (numbers are unchanged)">
          <span className="tool-units-label">Units</span>
          <select
            className="tool-units-select"
            aria-label="Document unit"
            value={documentUnit}
            disabled={disabled}
            onChange={(event) => onDocumentUnitChange(event.target.value)}
          >
            {LENGTH_UNITS.map((u) => (
              <option key={u.key} value={u.key}>
                {u.name}
              </option>
            ))}
          </select>
        </label>
        <ToolButton
          icon="stl"
          label="STL"
          title="Download the displayed mesh as binary STL"
          disabledReason={notReady}
          disabled={disabled}
          onClick={onDownloadStl}
        />
        <ToolButton
          icon="step"
          label="STEP"
          title="Export STEP (AP203) in the document unit. Exact B-Rep models export analytic surfaces; organic shapes export as faceted geometry."
          disabledReason={notReady}
          disabled={disabled}
          onClick={onDownloadStep}
        />
      </div>
      {/* Rare, set-and-forget controls live behind an overflow menu so the
          strip stays one row. <details> keeps it stateless and SSR-safe. */}
      <details className="tool-menu">
        <summary
          className="main-tool"
          title="Meshing settings (exact booleans)"
          aria-label="Meshing settings"
        >
          {Icons.menu}
        </summary>
        <div className="tool-menu-panel">
          <MeshSettings
            exactBooleans={exactBooleans}
            onExactBooleansChange={onExactBooleansChange}
            disabled={disabled}
          />
        </div>
      </details>
    </div>
  );
}
