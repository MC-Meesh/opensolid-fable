// OpenSolid playground: live-edited shape script -> WASM meshing -> three.js.

import * as THREE from 'three';
import { OrbitControls } from './vendor/OrbitControls.js';
import init, { WasmShape } from './pkg/opensolid_wasm.js';
import { buildBinaryStl } from './stl.js';

const DEFAULT_SCRIPT = `// Build a shape with the OpenSolid API and return it.
//
// Constructors (all centered at the origin, y is up):
//   Shape.sphere(r)
//   Shape.box3(hx, hy, hz)              half-extents
//   Shape.roundedBox(hx, hy, hz, r)     edge radius r
//   Shape.cylinder(r, halfHeight)       axis along y
//   Shape.torus(major, minor)           ring in the xz plane
//   Shape.capsule(x1,y1,z1, x2,y2,z2, r)
// Operations (each returns a new shape):
//   .translate(x, y, z)
//   .union(other)  .intersect(other)  .subtract(other)
//   .smoothUnion(other, radius?)

const body = Shape.roundedBox(1.0, 0.55, 0.8, 0.15);
const bump = Shape.sphere(0.55).translate(0, 0.65, 0);
const solid = body.smoothUnion(bump, 0.25);
const hole = Shape.cylinder(0.28, 2.0);
return solid.subtract(hole);
`;

const editor = document.getElementById('editor');
const errorPane = document.getElementById('error');
const runButton = document.getElementById('run');
const stlButton = document.getElementById('stl');
const resolutionSlider = document.getElementById('resolution');
const resolutionValue = document.getElementById('resolution-value');
const wireframeToggle = document.getElementById('wireframe');
const statsPane = document.getElementById('stats');
const viewport = document.getElementById('viewport');
const loadingPane = document.getElementById('loading');

editor.value = DEFAULT_SCRIPT;

// --- three.js scene ---------------------------------------------------------

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
viewport.appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x14171c);

const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 1000);
camera.position.set(3, 2.5, 4);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;

scene.add(new THREE.HemisphereLight(0xbfd4ff, 0x3a3226, 0.9));
const keyLight = new THREE.DirectionalLight(0xffffff, 1.6);
keyLight.position.set(4, 6, 3);
scene.add(keyLight);
const fillLight = new THREE.DirectionalLight(0x88aaff, 0.4);
fillLight.position.set(-4, -2, -3);
scene.add(fillLight);

const grid = new THREE.GridHelper(10, 20, 0x2b323d, 0x22272f);
scene.add(grid);

const material = new THREE.MeshStandardMaterial({
  color: 0x5f9ee8,
  metalness: 0.15,
  roughness: 0.5,
});
const meshObject = new THREE.Mesh(new THREE.BufferGeometry(), material);
scene.add(meshObject);

function resize() {
  const { clientWidth: w, clientHeight: h } = viewport;
  if (w === 0 || h === 0) return;
  renderer.setSize(w, h);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(viewport);
resize();

renderer.setAnimationLoop(() => {
  controls.update();
  renderer.render(scene, camera);
});

// --- shape script evaluation and meshing ------------------------------------

let currentShape = null; // last successfully evaluated WasmShape
let currentMesh = null; // { positions, normals, indices } of the shown mesh

function showError(message) {
  errorPane.textContent = message;
  errorPane.style.display = 'block';
}

function clearError() {
  errorPane.style.display = 'none';
}

function evaluateScript() {
  clearError();
  let shape;
  try {
    const build = new Function('Shape', `"use strict";\n${editor.value}`);
    shape = build(WasmShape);
  } catch (err) {
    showError(String(err.stack || err));
    return;
  }
  if (!(shape instanceof WasmShape)) {
    showError(
      'Script must return a Shape, e.g. end with:\n  return solid;'
    );
    return;
  }
  if (currentShape) currentShape.free();
  currentShape = shape;
  remesh({ reframe: true });
}

function remesh({ reframe = false } = {}) {
  if (!currentShape) return;
  clearError();
  const resolution = Number(resolutionSlider.value);
  const started = performance.now();
  let data;
  try {
    data = currentShape.mesh(resolution);
  } catch (err) {
    showError(`Meshing failed: ${String(err)}`);
    return;
  }
  const elapsed = performance.now() - started;

  const positions = data.positions;
  const normals = data.normals;
  const indices = data.indices;
  data.free();

  if (indices.length === 0) {
    showError(
      'Mesh is empty: the surface never crosses the sampled region. ' +
        'Check the shape is non-degenerate.'
    );
  }

  currentMesh = { positions, normals, indices };
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  geometry.setAttribute('normal', new THREE.BufferAttribute(normals, 3));
  geometry.setIndex(new THREE.BufferAttribute(indices, 1));
  meshObject.geometry.dispose();
  meshObject.geometry = geometry;

  statsPane.textContent =
    `${(indices.length / 3).toLocaleString()} triangles · ` +
    `${(positions.length / 3).toLocaleString()} vertices · ` +
    `${resolution}³ grid · ${elapsed.toFixed(0)} ms`;

  if (reframe) frameCamera();
}

function frameCamera() {
  const b = currentShape.bounds();
  const center = new THREE.Vector3(
    (b[0] + b[3]) / 2,
    (b[1] + b[4]) / 2,
    (b[2] + b[5]) / 2
  );
  const radius = Math.max(
    Math.hypot(b[3] - b[0], b[4] - b[1], b[5] - b[2]) / 2,
    0.1
  );
  const direction = new THREE.Vector3(1, 0.7, 1.2).normalize();
  camera.position.copy(center).addScaledVector(direction, radius * 2.6);
  camera.near = radius / 100;
  camera.far = radius * 100;
  camera.updateProjectionMatrix();
  controls.target.copy(center);
}

function downloadStl() {
  if (!currentMesh || currentMesh.indices.length === 0) {
    showError('Nothing to export yet: run a script that produces a mesh.');
    return;
  }
  const buffer = buildBinaryStl(currentMesh.positions, currentMesh.indices);
  const blob = new Blob([buffer], { type: 'model/stl' });
  const link = document.createElement('a');
  link.href = URL.createObjectURL(blob);
  link.download = 'opensolid.stl';
  link.click();
  URL.revokeObjectURL(link.href);
}

// --- wire up UI --------------------------------------------------------------

runButton.addEventListener('click', evaluateScript);
editor.addEventListener('keydown', (event) => {
  if ((event.ctrlKey || event.metaKey) && event.key === 'Enter') {
    event.preventDefault();
    evaluateScript();
  }
});
stlButton.addEventListener('click', downloadStl);
resolutionSlider.addEventListener('input', () => {
  resolutionValue.textContent = resolutionSlider.value;
});
resolutionSlider.addEventListener('change', () => remesh());
wireframeToggle.addEventListener('change', () => {
  material.wireframe = wireframeToggle.checked;
});

// --- boot --------------------------------------------------------------------

await init();
loadingPane.remove();
evaluateScript();
