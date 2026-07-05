import { useCallback, useEffect, useRef, useState } from 'react';
import init, { WasmShape } from '../pkg/opensolid_wasm.js';
import ScriptEditor from './components/ScriptEditor.jsx';
import Viewport3D from './components/Viewport3D.jsx';
import Toolbar from './components/Toolbar.jsx';
import StatusBar from './components/StatusBar.jsx';
import SceneTree from './components/SceneTree.jsx';
import SketchCanvas from './components/SketchCanvas.jsx';
import { DEFAULT_SCRIPT } from './lib/defaultScript.js';
import { freeNodes, nodeLabel, runTracedScript, serializeTree } from './lib/sceneTree.js';
import { buildBinaryStl } from './lib/stl.js';
import { pickCandidates, pickNodeAt } from './lib/picking.js';
import { applyTranslate, applyRotate, applyScale, pathTo, nodeAt } from './lib/transformEdit.js';

let wasmInit = null;
function ensureWasm() {
  wasmInit ??= init();
  return wasmInit;
}

const DEFAULT_RESOLUTION = 64;

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
  const [wasmReady, setWasmReady] = useState(false);
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
  const profileRef = useRef(null);

  const scriptRef = useRef(DEFAULT_SCRIPT);
  const resolutionRef = useRef(DEFAULT_RESOLUTION);
  const shapeRef = useRef(null);
  const tracedRef = useRef(null);
  const meshRef = useRef(null);
  const meshKeyRef = useRef(0);
  const editorRef = useRef(null);
  const selectedPathRef = useRef(null);

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
    setError(null);
    let traced;
    try {
      traced = runTracedScript(scriptRef.current, WasmShape);
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
      evaluateScript();
    },
    [selectedNode, evaluateScript]
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

  const bootedRef = useRef(false);
  useEffect(() => {
    let cancelled = false;
    ensureWasm()
      .then(() => {
        if (cancelled || bootedRef.current) return;
        bootedRef.current = true;
        setWasmReady(true);
        evaluateScript();
      })
      .catch((err) => {
        if (!cancelled) setError(`Failed to load WASM module: ${String(err)}`);
      });
    return () => {
      cancelled = true;
    };
  }, [evaluateScript]);

  useEffect(() => {
    function onKeyDown(event) {
      const tag = event.target.tagName;
      if (tag === 'INPUT' || tag === 'TEXTAREA') return;
      const cm = event.target.closest('.cm-editor');
      if (cm) return;

      if (event.key === 't' || event.key === 'T') {
        setGizmoMode('translate');
      } else if (event.key === 'r' || event.key === 'R') {
        setGizmoMode('rotate');
      } else if (event.key === 's' || event.key === 'S') {
        setGizmoMode('scale');
      } else if (event.key === 'Escape') {
        clearSelection();
      }
    }
    window.addEventListener('keydown', onKeyDown);
    return () => window.removeEventListener('keydown', onKeyDown);
  }, [clearSelection]);

  const handleResolutionChange = useCallback((value) => {
    resolutionRef.current = value;
    setResolution(value);
  }, []);

  const handleResolutionCommit = useCallback(() => {
    remesh();
  }, [remesh]);

  const handleScriptChange = useCallback((source) => {
    scriptRef.current = source;
  }, []);

  const handleProfileChange = useCallback((profile) => {
    profileRef.current = profile;
  }, []);

  return (
    <div className="app">
      <div className="left">
        <header>
          <h1>OpenSolid Playground</h1>
          <p>Write a script that returns a Shape, then Run (Ctrl/Cmd+Enter).</p>
        </header>
        <SceneTree root={tree} selectedId={selectedNode?.id} onSelect={selectNode} />
        <ScriptEditor
          ref={editorRef}
          initialDoc={DEFAULT_SCRIPT}
          onChange={handleScriptChange}
          onRun={evaluateScript}
        />
        {error && <pre className="error">{error}</pre>}
        <Toolbar
          resolution={resolution}
          onResolutionChange={handleResolutionChange}
          onResolutionCommit={handleResolutionCommit}
          wireframe={wireframe}
          onWireframeChange={setWireframe}
          onRun={evaluateScript}
          onDownloadStl={downloadStl}
          disabled={!wasmReady}
        />
      </div>
      <div className="right">
        <Viewport3D
          mesh={mesh}
          wireframe={wireframe}
          sketchPlane={sketchOpen ? sketchPlane : null}
          gizmoMode={gizmoMode}
          selectedMesh={selectedMesh}
          selectedPivot={selectedPivot}
          onPick={handlePick}
          onTransform={handleTransform}
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
        <SketchCanvas
          open={sketchOpen}
          plane={sketchPlane}
          onPlaneChange={setSketchPlane}
          onProfileChange={handleProfileChange}
        />
        <button
          className={`secondary sketch-toggle${sketchOpen ? ' active' : ''}`}
          onClick={() => setSketchOpen((v) => !v)}
        >
          {sketchOpen ? 'Exit sketch' : 'Sketch'}
        </button>
        {stats && !sketchOpen && <StatusBar stats={stats} />}
        {!wasmReady && <div className="loading">Loading WASM…</div>}
      </div>
    </div>
  );
}
