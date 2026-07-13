// Main viewport toolbar, grouped by workflow like the SolidWorks CommandManager:
// Sketch | Features | View. Disabled buttons keep a tooltip explaining why.

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
  revolve: (
    <svg {...ICON}>
      <path d="M8 1.5 V14.5" strokeDasharray="2 1.6" />
      <path d="M10.5 4.5 v7 M10.5 11.5 c3 0 4 -1.5 4 -3.5 s-1 -3.5 -4 -3.5" />
      <path d="M12.7 12.6 L10.5 11.5 L12 9.6" />
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
};

function ToolButton({ icon, label, title, disabledReason, active, disabled, onClick }) {
  return (
    <span className="tool-wrap" title={disabled ? disabledReason : title}>
      <button
        type="button"
        className={`main-tool${active ? ' active' : ''}`}
        disabled={disabled}
        aria-pressed={active}
        onClick={onClick}
      >
        {Icons[icon]}
        <span className="tool-label">{label}</span>
      </button>
    </span>
  );
}

export default function MainToolbar({
  disabled,
  sketchOpen,
  sketchOnFace = false,
  onSketchToggle,
  canSweep,
  sweepDisabledReason,
  onSweep,
  onView,
  onFit,
  wireframe,
  onWireframeChange,
}) {
  const notReady = 'Still loading the WASM kernel';
  return (
    <div className="main-toolbar" role="toolbar" aria-label="Main toolbar">
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
          disabledReason={notReady}
          active={sketchOpen}
          disabled={disabled}
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
          onClick={() => onView('front')}
        />
        <ToolButton
          icon="top"
          label="Top"
          title="Top view (5)"
          disabledReason={notReady}
          disabled={disabled}
          onClick={() => onView('top')}
        />
        <ToolButton
          icon="right"
          label="Right"
          title="Right view (4)"
          disabledReason={notReady}
          disabled={disabled}
          onClick={() => onView('right')}
        />
        <ToolButton
          icon="iso"
          label="Iso"
          title="Isometric view (7)"
          disabledReason={notReady}
          disabled={disabled}
          onClick={() => onView('iso')}
        />
        <ToolButton
          icon="wireframe"
          label="Wireframe"
          title="Toggle wireframe rendering"
          disabledReason={notReady}
          active={wireframe}
          disabled={disabled}
          onClick={() => onWireframeChange(!wireframe)}
        />
      </div>
    </div>
  );
}
