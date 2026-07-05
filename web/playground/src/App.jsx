import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { useWasm } from './wasm/WasmContext.jsx';
import ErrorBoundary from './components/ErrorBoundary.jsx';
import WasmErrorScreen from './components/WasmErrorScreen.jsx';
import ScriptEditor from './components/ScriptEditor.jsx';
import Viewport3D from './components/Viewport3D.jsx';
import Toolbar from './components/Toolbar.jsx';
import StatusBar from './components/StatusBar.jsx';
import ScenePanel from './components/ScenePanel.jsx';
import SceneTree from './components/SceneTree.jsx';
import PropertyPanel from './components/PropertyPanel.jsx';
import SketchCanvas from './components/SketchCanvas.jsx';
import SweepPanel from './components/SweepPanel.jsx';
import { DEFAULT_SCRIPT } from './lib/defaultScript.js';
import { freeNodes, nodeLabel, runTracedScript, serializeTree } from './lib/sceneTree.js';
import { buildBinaryStl } from './lib/stl.js';
import { pickCandidates, pickNodeAt } from './lib/picking.js';
import { applyTranslate, applyRotate, applyScale, pathTo, nodeAt } from './lib/transformEdit.js';
import { setNodeArg, setBooleanOp } from './lib/propertyEdit.js';
import {
  addShape,
  deleteShape,
  listNodes,
  parseScript,
  updateNumericArg,
} from './lib/shapeGraph.js';
import { buildSweepShape, opsBounds, profileToOps, sweepTreeNode } from './lib/sweep.js';

const DEFAULT_RESOLUTION = 64;
const EDIT_DEBOUNCE_MS = 400;

function meshShape(shape, resolution) {
  const data = shape.mesh(resolution);
  const positions = data.positions;
  const normals = data.normals;
  const indices = data.indices;
  data.free();
  return { positions, normals, indices };
}

function shapePivot(shape) {
  const b = shape.bounds();
  return [(b[0] + b[3]) / 2, (b[1] + b[4]) / 2, (b[2] + b[5]) / 2];
}

export default function App() {
  // The WASM lifecycle lives in one store (src/wasm/loader.js, surfaced via
  // WasmContext) — App only reads status and the bound API classes.
  const { status: wasmStatus, error: wasmError, api: wasm, ready: wasmReady, retry: retryWasm } = useWasm();
  const [error, setError] = useState(null);
  const [resolution, setResolution] = useState(DEFAULT_RESOLUTION);
  const [wireframe, setWireframe] = useState(false);
  const [mesh, setMesh] = useState(null);
  const [stats, setStats] = useState(null);
  const [tree, setTree] = useState(null);
  const [selectedNode, setSelectedNode] = useState(null);
  const [selectedMesh, setSelectedMesh] = useState(null);
  const [selectedPivot, setSelectedPivot] = useState(null);
  const [gizmoMode, setGizmoMode] = useState('translate');
  const [sketchOpen, setSketchOpen] = useState(false);
  const [sketchPlane, setSketchPlane] = useState('XY');
  const [sweep, setSweep] = useState(null);
  const [sweepError, setSweepError] = useState(null);
  const [previewMesh, setPreviewMesh] = useState(null);
  const profileRef = useRef(null);

  // Bidirectional sync: the shape operation graph parsed from the script.
  // GUI mutations (palette, parameter edits) rewrite individual statements
  // and push the new script text into the editor, preserving all hand-written
  // code. The graph is re-derived after every change from either side.
  const [graph, setGraph] = useState(() => parseScript(DEFAULT_SCRIPT));

  const scriptRef = useRef(DEFAULT_SCRIPT);
  const graphRef = useRef(graph);
  const resolutionRef = useRef(DEFAULT_RESOLUTION);
  const shapeRef = useRef(null);
  const tracedRef = useRef(null);
  const meshRef = useRef(null);
  const meshKeyRef = useRef(0);
  const editorRef = useRef(null);
  const selectedPathRef = useRef(null);
  const editTimerRef = useRef(null);
  const wasmRef = useRef(null);
  wasmRef.current = wasm;

  const clearSelection = useCallback(() => {
    setSelectedNode(null);
    setSelectedMesh(null);
    setSelectedPivot(null);
    selectedPathRef.current = null;
  }, []);

  const remesh = useCallback(({ reframe = false } = {}) => {
    const shape = shapeRef.current;
    if (!shape) return;
    setError(null);
    const res = resolutionRef.current;
    const started = performance.now();
    let data;
    try {
      data = shape.mesh(res);
    } catch (err) {
      setError(`Meshing failed: ${String(err)}`);
      return;
    }
    const elapsedMs = performance.now() - started;

    const positions = data.positions;
    const normals = data.normals;
    const indices = data.indices;
    data.free();

    if (indices.length === 0) {
      setError(
        'Mesh is empty: the surface never crosses the sampled region. ' +
          'Check the shape is non-degenerate.'
      );
    }

    let frame = null;
    if (reframe) {
      const b = shape.bounds();
      frame = {
        center: [(b[0] + b[3]) / 2, (b[1] + b[4]) / 2, (b[2] + b[5]) / 2],
        radius: Math.max(
          Math.hypot(b[3] - b[0], b[4] - b[1], b[5] - b[2]) / 2,
          0.1
        ),
      };
    }

    meshRef.current = { positions, indices };
    setMesh({ positions, normals, indices, frame, key: ++meshKeyRef.current });
    setStats({
      triangles: indices.length / 3,
      vertices: positions.length / 3,
      resolution: res,
      elapsedMs,
    });
  }, []);

  const evaluateScript = useCallback(() => {
    const api = wasmRef.current;
    if (!api) return;
    setError(null);
    let traced;
    try {
      traced = runTracedScript(scriptRef.current, api.WasmShape, api.WasmProfile2D);
    } catch (err) {
      setError(String(err?.stack || err));
      return;
    }
    if (tracedRef.current) freeNodes(tracedRef.current.nodes);
    tracedRef.current = traced;
    setTree(traced.root);
    shapeRef.current = traced.root.shape;
    remesh({ reframe: true });

    if (selectedPathRef.current) {
      const restored = nodeAt(traced.root, selectedPathRef.current);
      if (restored && restored.shape) {
        setSelectedNode(restored);
        try {
          const res = resolutionRef.current;
          setSelectedMesh(meshShape(restored.shape, res));
          setSelectedPivot(shapePivot(restored.shape));
        } catch {
          clearSelection();
        }
      } else {
        clearSelection();
      }
    } else {
      clearSelection();
    }
  }, [remesh, clearSelection]);

  const commitGraph = useCallback(() => {
    const next = parseScript(scriptRef.current);
    graphRef.current = next;
    setGraph(next);
  }, []);

  // Script -> GUI: re-parse and re-evaluate, debounced behind keystrokes.
  const handleScriptChange = useCallback(
    (source) => {
      if (source === scriptRef.current) return;
      scriptRef.current = source;
      clearTimeout(editTimerRef.current);
      editTimerRef.current = setTimeout(() => {
        commitGraph();
        evaluateScript();
      }, EDIT_DEBOUNCE_MS);
    },
    [commitGraph, evaluateScript]
  );

  const runNow = useCallback(() => {
    clearTimeout(editTimerRef.current);
    commitGraph();
    evaluateScript();
  }, [commitGraph, evaluateScript]);

  useEffect(() => () => clearTimeout(editTimerRef.current), []);

  // GUI -> Script: apply a shapeGraph mutation, push the rewritten script
  // into the editor, and refresh graph + shape immediately.
  const applyMutation = useCallback(
    (result) => {
      if (result.error) {
        setError(result.error);
        return false;
      }
      scriptRef.current = result.source;
      editorRef.current?.setDoc(result.source);
      commitGraph();
      evaluateScript();
      return true;
    },
    [commitGraph, evaluateScript]
  );

  const handleAddShape = useCallback(
    (ctor, args) => {
      const result = addShape(graphRef.current, ctor, args);
      applyMutation(result);
    },
    [applyMutation]
  );

  const handleDeleteShape = useCallback(
    (name) => {
      applyMutation(deleteShape(graphRef.current, name));
    },
    [applyMutation]
  );

  const handleUpdateArg = useCallback(
    (nodeId, linkIndex, argIndex, value) => {
      applyMutation(updateNumericArg(graphRef.current, nodeId, linkIndex, argIndex, value));
    },
    [applyMutation]
  );

  const selectNode = useCallback(
    (node) => {
      const root = tracedRef.current?.root;
      if (!root) return;
      if (!node || node === root || node.id === selectedNode?.id) {
        clearSelection();
        return;
      }
      setSelectedNode(node);
      selectedPathRef.current = pathTo(root, node.id);
      if (node.shape) {
        try {
          const res = resolutionRef.current;
          setSelectedMesh(meshShape(node.shape, res));
          setSelectedPivot(shapePivot(node.shape));
        } catch {
          clearSelection();
        }
      }
    },
    [selectedNode, clearSelection]
  );

  const handlePick = useCallback(
    (point) => {
      const root = tracedRef.current?.root;
      if (!root) return;
      if (!point) {
        clearSelection();
        return;
      }
      const candidates = pickCandidates(root);
      const picked = pickNodeAt(candidates, point);
      if (picked) {
        selectNode(picked);
      } else {
        clearSelection();
      }
    },
    [selectNode, clearSelection]
  );

  const handleTransform = useCallback(
    (event) => {
      const root = tracedRef.current?.root;
      if (!root || !selectedNode) return;

      const path = pathTo(root, selectedNode.id);
      let newRoot;

      if (event.mode === 'translate') {
        newRoot = applyTranslate(root, selectedNode.id, event.delta);
      } else if (event.mode === 'rotate') {
        newRoot = applyRotate(root, selectedNode.id, event.axis, event.angle, event.pivot);
      } else if (event.mode === 'scale') {
        newRoot = applyScale(root, selectedNode.id, event.factors, event.pivot);
      } else {
        return;
      }

      selectedPathRef.current = path;
      const script = serializeTree(newRoot);
      scriptRef.current = script;
      editorRef.current?.setDoc(script);
      commitGraph();
      evaluateScript();
    },
    [selectedNode, commitGraph, evaluateScript]
  );

  // Property panel edits: mutate the traced tree, then push the serialized
  // script through the same sync path the gizmo uses.
  const applyTreeEdit = useCallback(
    (result, nodeId) => {
      if (result.error) {
        setError(result.error);
        return;
      }
      if (result.root === tracedRef.current?.root) return;
      selectedPathRef.current = pathTo(result.root, nodeId) ?? selectedPathRef.current;
      const script = serializeTree(result.root);
      scriptRef.current = script;
      editorRef.current?.setDoc(script);
      commitGraph();
      evaluateScript();
    },
    [commitGraph, evaluateScript]
  );

  const handleEditArg = useCallback(
    (nodeId, argIndex, value) => {
      const root = tracedRef.current?.root;
      if (!root) return;
      applyTreeEdit(setNodeArg(root, nodeId, argIndex, value), nodeId);
    },
    [applyTreeEdit]
  );

  const handleChangeOp = useCallback(
    (nodeId, op) => {
      const root = tracedRef.current?.root;
      if (!root) return;
      applyTreeEdit(setBooleanOp(root, nodeId, op), nodeId);
    },
    [applyTreeEdit]
  );

  const downloadStl = useCallback(() => {
    const current = meshRef.current;
    if (!current || current.indices.length === 0) {
      setError('Nothing to export yet: run a script that produces a mesh.');
      return;
    }
    const buffer = buildBinaryStl(current.positions, current.indices);
    const blob = new Blob([buffer], { type: 'model/stl' });
    const link = document.createElement('a');
    link.href = URL.createObjectURL(blob);
    link.download = 'opensolid.stl';
    link.click();
    URL.revokeObjectURL(link.href);
  }, []);

  // ---- extrude / revolve workflow -----------------------------------------

  const handleSweepStart = useCallback(
    (kind) => {
      const profile = profileRef.current;
      if (!profile?.closed) return;
      let ops;
      try {
        ops = profileToOps(profile);
      } catch (err) {
        setError(String(err));
        return;
      }
      const { min, max } = opsBounds(ops);
      const extent = Math.max(max[0] - min[0], max[1] - min[1]) || 1;
      clearSelection();
      setSweepError(null);
      setSketchOpen(false);
      setSweep(
        kind === 'extrude'
          ? { kind, plane: profile.plane, ops, value: extent, range: extent * 4 }
          : { kind, plane: profile.plane, ops, value: 360, range: 360 }
      );
    },
    [clearSelection]
  );

  const handleSweepChange = useCallback((value) => {
    setSweep((current) => (current ? { ...current, value } : current));
  }, []);

  const cancelSweep = useCallback(() => {
    setSweep(null);
    setSweepError(null);
    setSketchOpen(true);
  }, []);

  // Commit the pending sweep: graft it onto the tree (unioned with any
  // existing shape), then push the serialized script through the same sync
  // path the gizmo uses.
  const applySweep = useCallback(() => {
    if (!sweep) return;
    const script = serializeTree(sweepTreeNode(tracedRef.current?.root ?? null, sweep));
    scriptRef.current = script;
    editorRef.current?.setDoc(script);
    setSweep(null);
    setSweepError(null);
    commitGraph();
    evaluateScript();
  }, [sweep, commitGraph, evaluateScript]);

  // Live preview: remesh the pending sweep whenever its parameters change.
  useEffect(() => {
    if (!sweep || !wasm) {
      setPreviewMesh(null);
      return;
    }
    let shape = null;
    try {
      shape = buildSweepShape(wasm.WasmShape, wasm.WasmProfile2D, sweep);
      setPreviewMesh(meshShape(shape, resolutionRef.current));
      setSweepError(null);
    } catch (err) {
      setPreviewMesh(null);
      setSweepError(String(err));
    } finally {
      shape?.free?.();
    }
  }, [sweep, wasm]);

  const handleSketchToggle = useCallback(() => {
    setSweep(null);
    setSweepError(null);
    setSketchOpen((v) => !v);
  }, []);

  // First successful WASM init runs the default script once.
  const bootedRef = useRef(false);
  useEffect(() => {
    if (!wasmReady || bootedRef.current) return;
    bootedRef.current = true;
    evaluateScript();
  }, [wasmReady, evaluateScript]);

  useEffect(() => {
    function onKeyDown(event) {
      const tag = event.target.tagName;
      if (tag === 'INPUT' || tag === 'TEXTAREA') return;
      const cm = event.target.closest('.cm-editor');
      if (cm) return;
      // Sketch mode owns the keyboard (tools, dimensions, undo, Esc).
      if (sketchOpen) return;

      if (event.key === 't' || event.key === 'T') {
        setGizmoMode('translate');
      } else if (event.key === 'r' || event.key === 'R') {
        setGizmoMode('rotate');
      } else if (event.key === 's' || event.key === 'S') {
        setGizmoMode('scale');
      } else if (event.key === 'Escape') {
        if (sweep) cancelSweep();
        else clearSelection();
      }
    }
    window.addEventListener('keydown', onKeyDown);
    return () => window.removeEventListener('keydown', onKeyDown);
  }, [clearSelection, sweep, cancelSweep, sketchOpen]);

  const handleResolutionChange = useCallback((value) => {
    resolutionRef.current = value;
    setResolution(value);
  }, []);

  const handleResolutionCommit = useCallback(() => {
    remesh();
  }, [remesh]);

  const handleProfileChange = useCallback((profile) => {
    profileRef.current = profile;
  }, []);

  const graphNodes = useMemo(() => listNodes(graph), [graph]);

  return (
    <div className="app">
      <div className="left">
        <header>
          <h1>OpenSolid Playground</h1>
          <p>Edit the script or the scene — both stay in sync.</p>
        </header>
        <ScenePanel
          nodes={graphNodes}
          selected={null}
          onSelect={() => {}}
          onAddShape={handleAddShape}
          onDeleteShape={handleDeleteShape}
          onUpdateArg={handleUpdateArg}
          disabled={!wasmReady}
        />
        <SceneTree root={tree} selectedId={selectedNode?.id} onSelect={selectNode} />
        <ErrorBoundary name="Script editor">
          <ScriptEditor
            ref={editorRef}
            initialDoc={DEFAULT_SCRIPT}
            onChange={handleScriptChange}
            onRun={runNow}
          />
        </ErrorBoundary>
        {error && <pre className="error">{error}</pre>}
        <Toolbar
          resolution={resolution}
          onResolutionChange={handleResolutionChange}
          onResolutionCommit={handleResolutionCommit}
          wireframe={wireframe}
          onWireframeChange={setWireframe}
          onRun={runNow}
          onDownloadStl={downloadStl}
          disabled={!wasmReady}
        />
      </div>
      <div className="right">
        <ErrorBoundary name="3D viewport">
          <Viewport3D
            mesh={mesh}
            wireframe={wireframe}
            sketchPlane={sketchOpen ? sketchPlane : null}
            gizmoMode={gizmoMode}
            selectedMesh={selectedMesh}
            selectedPivot={selectedPivot}
            previewMesh={previewMesh}
            onPick={handlePick}
            onTransform={handleTransform}
          />
        </ErrorBoundary>
        <SweepPanel
          sweep={sweep}
          error={sweepError}
          onChange={handleSweepChange}
          onApply={applySweep}
          onCancel={cancelSweep}
        />
        {selectedNode && (
          <div className="gizmo-bar">
            <button
              className={gizmoMode === 'translate' ? 'gizmo-active' : 'secondary'}
              onClick={() => setGizmoMode('translate')}
              title="Translate (T)"
            >
              Move
            </button>
            <button
              className={gizmoMode === 'rotate' ? 'gizmo-active' : 'secondary'}
              onClick={() => setGizmoMode('rotate')}
              title="Rotate (R)"
            >
              Rotate
            </button>
            <button
              className={gizmoMode === 'scale' ? 'gizmo-active' : 'secondary'}
              onClick={() => setGizmoMode('scale')}
              title="Scale (S)"
            >
              Scale
            </button>
            <span className="gizmo-label">{nodeLabel(selectedNode)}</span>
            <button className="secondary" onClick={clearSelection}>
              Deselect
            </button>
          </div>
        )}
        {selectedNode && !sketchOpen && (
          <PropertyPanel
            node={selectedNode}
            disabled={!wasmReady}
            onEditArg={handleEditArg}
            onChangeOp={handleChangeOp}
          />
        )}
        <ErrorBoundary name="Sketch canvas">
          <SketchCanvas
            open={sketchOpen}
            plane={sketchPlane}
            onPlaneChange={setSketchPlane}
            onProfileChange={handleProfileChange}
            onSweep={handleSweepStart}
            onExit={() => setSketchOpen(false)}
          />
        </ErrorBoundary>
        <button
          className={`secondary sketch-toggle${sketchOpen ? ' active' : ''}`}
          onClick={handleSketchToggle}
        >
          {sketchOpen ? 'Exit sketch' : 'Sketch'}
        </button>
        {stats && !sketchOpen && <StatusBar stats={stats} />}
        {(wasmStatus === 'idle' || wasmStatus === 'loading') && (
          <div className="loading">Loading WASM…</div>
        )}
        {wasmStatus === 'failed' && (
          <WasmErrorScreen error={wasmError} onRetry={retryWasm} />
        )}
      </div>
    </div>
  );
}
