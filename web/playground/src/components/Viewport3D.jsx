import { useEffect, useRef } from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { TransformControls } from 'three/addons/controls/TransformControls.js';

function frameCamera({ camera, controls }, { center, radius }) {
  const target = new THREE.Vector3(...center);
  const direction = new THREE.Vector3(1, 0.7, 1.2).normalize();
  camera.position.copy(target).addScaledVector(direction, radius * 2.6);
  camera.near = radius / 100;
  camera.far = radius * 100;
  camera.updateProjectionMatrix();
  controls.target.copy(target);
}

const SKETCH_PLANES = {
  XY: { rotation: [0, 0, 0], color: 0x4f9cf9 },
  XZ: { rotation: [-Math.PI / 2, 0, 0], color: 0x5fdf8a },
  YZ: { rotation: [0, Math.PI / 2, 0], color: 0xef6f6f },
};

const HIGHLIGHT_MATERIAL = new THREE.MeshStandardMaterial({
  color: 0x4fc3f7,
  metalness: 0.1,
  roughness: 0.4,
  transparent: true,
  opacity: 0.35,
  depthWrite: false,
});

const PREVIEW_MATERIAL = new THREE.MeshStandardMaterial({
  color: 0x7ce38b,
  metalness: 0.1,
  roughness: 0.45,
  transparent: true,
  opacity: 0.55,
  depthWrite: false,
});

export default function Viewport3D({
  mesh,
  wireframe,
  sketchPlane,
  gizmoMode,
  selectedMesh,
  selectedPivot,
  previewMesh,
  onPick,
  onTransform,
}) {
  const containerRef = useRef(null);
  const sceneRef = useRef(null);

  useEffect(() => {
    const container = containerRef.current;

    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(window.devicePixelRatio);
    container.appendChild(renderer.domElement);

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x14171c);

    const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 1000);
    camera.position.set(3, 2.5, 4);

    const orbitControls = new OrbitControls(camera, renderer.domElement);
    orbitControls.enableDamping = true;

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

    const ghostMesh = new THREE.Mesh(new THREE.BufferGeometry(), HIGHLIGHT_MATERIAL);
    ghostMesh.renderOrder = 1;
    ghostMesh.visible = false;

    const previewObject = new THREE.Mesh(new THREE.BufferGeometry(), PREVIEW_MATERIAL);
    previewObject.renderOrder = 2;
    previewObject.visible = false;
    scene.add(previewObject);

    const anchor = new THREE.Group();
    anchor.add(ghostMesh);
    anchor.visible = false;
    scene.add(anchor);

    const transformControls = new TransformControls(camera, renderer.domElement);
    transformControls.attach(anchor);
    transformControls.visible = false;
    transformControls.enabled = false;
    scene.add(transformControls.getHelper());

    transformControls.addEventListener('dragging-changed', (event) => {
      orbitControls.enabled = !event.value;
    });

    const raycaster = new THREE.Raycaster();
    const pointerDown = new THREE.Vector2();
    let pointerDownTime = 0;

    function onPointerDown(event) {
      const rect = container.getBoundingClientRect();
      pointerDown.set(event.clientX - rect.left, event.clientY - rect.top);
      pointerDownTime = performance.now();
    }

    function onPointerUp(event) {
      if (performance.now() - pointerDownTime > 500) return;
      const rect = container.getBoundingClientRect();
      const upX = event.clientX - rect.left;
      const upY = event.clientY - rect.top;
      const dist = Math.hypot(upX - pointerDown.x, upY - pointerDown.y);
      if (dist > 5) return;
      if (transformControls.axis) return;

      const ndc = new THREE.Vector2(
        (upX / rect.width) * 2 - 1,
        -(upY / rect.height) * 2 + 1,
      );
      raycaster.setFromCamera(ndc, camera);
      const hits = raycaster.intersectObject(meshObject);
      const cb = sceneRef.current?._onPick;
      if (!cb) return;
      if (hits.length > 0) {
        const p = hits[0].point;
        cb([p.x, p.y, p.z]);
      } else {
        cb(null);
      }
    }

    container.addEventListener('pointerdown', onPointerDown);
    container.addEventListener('pointerup', onPointerUp);

    function resize() {
      const { clientWidth: w, clientHeight: h } = container;
      if (w === 0 || h === 0) return;
      renderer.setSize(w, h);
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
    }
    const observer = new ResizeObserver(resize);
    observer.observe(container);
    resize();

    renderer.setAnimationLoop(() => {
      orbitControls.update();
      renderer.render(scene, camera);
    });

    function onKeyDown(event) {
      if (event.shiftKey && transformControls.enabled) {
        transformControls.setTranslationSnap(0.5);
        transformControls.setRotationSnap(THREE.MathUtils.degToRad(15));
        transformControls.setScaleSnap(0.25);
      }
    }

    function onKeyUp(event) {
      if (!event.shiftKey) {
        transformControls.setTranslationSnap(null);
        transformControls.setRotationSnap(null);
        transformControls.setScaleSnap(null);
      }
    }

    window.addEventListener('keydown', onKeyDown);
    window.addEventListener('keyup', onKeyUp);

    sceneRef.current = {
      renderer,
      camera,
      controls: orbitControls,
      material,
      meshObject,
      transformControls,
      anchor,
      ghostMesh,
      previewObject,
      _onPick: null,
      _onTransform: null,
    };

    return () => {
      container.removeEventListener('pointerdown', onPointerDown);
      container.removeEventListener('pointerup', onPointerUp);
      window.removeEventListener('keydown', onKeyDown);
      window.removeEventListener('keyup', onKeyUp);
      observer.disconnect();
      renderer.setAnimationLoop(null);
      transformControls.detach();
      transformControls.dispose();
      orbitControls.dispose();
      meshObject.geometry.dispose();
      ghostMesh.geometry.dispose();
      previewObject.geometry.dispose();
      material.dispose();
      grid.geometry.dispose();
      grid.material.dispose();
      renderer.dispose();
      renderer.domElement.remove();
      sceneRef.current = null;
    };
  }, []);

  useEffect(() => {
    const ctx = sceneRef.current;
    if (!ctx || !mesh) return;
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(mesh.positions, 3));
    geometry.setAttribute('normal', new THREE.BufferAttribute(mesh.normals, 3));
    geometry.setIndex(new THREE.BufferAttribute(mesh.indices, 1));
    ctx.meshObject.geometry.dispose();
    ctx.meshObject.geometry = geometry;
    if (mesh.frame) frameCamera(ctx, mesh.frame);
  }, [mesh]);

  useEffect(() => {
    const ctx = sceneRef.current;
    if (ctx) ctx.material.wireframe = wireframe;
  }, [wireframe]);

  useEffect(() => {
    const ctx = sceneRef.current;
    if (!ctx) return;
    if (previewMesh) {
      const geo = new THREE.BufferGeometry();
      geo.setAttribute('position', new THREE.BufferAttribute(previewMesh.positions, 3));
      geo.setAttribute('normal', new THREE.BufferAttribute(previewMesh.normals, 3));
      geo.setIndex(new THREE.BufferAttribute(previewMesh.indices, 1));
      ctx.previewObject.geometry.dispose();
      ctx.previewObject.geometry = geo;
      ctx.previewObject.visible = true;
    } else {
      ctx.previewObject.visible = false;
    }
  }, [previewMesh]);

  useEffect(() => {
    const ctx = sceneRef.current;
    if (!ctx || !sketchPlane) return undefined;
    const spec = SKETCH_PLANES[sketchPlane];
    if (!spec) return undefined;

    const scene = ctx.meshObject.parent;
    const group = new THREE.Group();
    group.rotation.set(...spec.rotation);

    const fillGeometry = new THREE.PlaneGeometry(10, 10);
    const fillMaterial = new THREE.MeshBasicMaterial({
      color: spec.color,
      transparent: true,
      opacity: 0.12,
      side: THREE.DoubleSide,
      depthWrite: false,
    });
    group.add(new THREE.Mesh(fillGeometry, fillMaterial));

    const edgeGeometry = new THREE.EdgesGeometry(fillGeometry);
    const edgeMaterial = new THREE.LineBasicMaterial({
      color: spec.color,
      transparent: true,
      opacity: 0.6,
    });
    group.add(new THREE.LineSegments(edgeGeometry, edgeMaterial));

    scene.add(group);
    return () => {
      scene.remove(group);
      fillGeometry.dispose();
      fillMaterial.dispose();
      edgeGeometry.dispose();
      edgeMaterial.dispose();
    };
  }, [sketchPlane]);

  useEffect(() => {
    const ctx = sceneRef.current;
    if (!ctx) return;
    if (selectedMesh && selectedPivot) {
      const geo = new THREE.BufferGeometry();
      geo.setAttribute('position', new THREE.BufferAttribute(selectedMesh.positions, 3));
      geo.setAttribute('normal', new THREE.BufferAttribute(selectedMesh.normals, 3));
      geo.setIndex(new THREE.BufferAttribute(selectedMesh.indices, 1));
      ctx.ghostMesh.geometry.dispose();
      ctx.ghostMesh.geometry = geo;
      ctx.ghostMesh.visible = true;

      const [px, py, pz] = selectedPivot;
      ctx.anchor.position.set(px, py, pz);
      ctx.ghostMesh.position.set(-px, -py, -pz);
      ctx.anchor.quaternion.identity();
      ctx.anchor.scale.set(1, 1, 1);
      ctx.anchor.visible = true;

      ctx.transformControls.visible = true;
      ctx.transformControls.enabled = true;
    } else {
      ctx.ghostMesh.visible = false;
      ctx.anchor.visible = false;
      ctx.transformControls.visible = false;
      ctx.transformControls.enabled = false;
    }
  }, [selectedMesh, selectedPivot]);

  useEffect(() => {
    const ctx = sceneRef.current;
    if (!ctx) return;
    if (gizmoMode && ctx.transformControls.enabled) {
      ctx.transformControls.setMode(gizmoMode);
    }
  }, [gizmoMode]);

  const onTransformRef = useRef(onTransform);
  onTransformRef.current = onTransform;
  const onPickRef = useRef(onPick);
  onPickRef.current = onPick;

  useEffect(() => {
    const ctx = sceneRef.current;
    if (!ctx) return;
    ctx._onPick = (...args) => onPickRef.current?.(...args);
  });

  useEffect(() => {
    const ctx = sceneRef.current;
    if (!ctx) return;

    function onDragEnd() {
      const cb = onTransformRef.current;
      if (!cb) return;
      const pivot = [ctx.anchor.position.x, ctx.anchor.position.y, ctx.anchor.position.z];
      const mode = ctx.transformControls.mode;

      if (mode === 'translate') {
        const startPivot = sceneRef.current?._startPivot;
        if (startPivot) {
          cb({
            mode: 'translate',
            delta: [pivot[0] - startPivot[0], pivot[1] - startPivot[1], pivot[2] - startPivot[2]],
            pivot: startPivot,
          });
        }
      } else if (mode === 'rotate') {
        const q = ctx.anchor.quaternion;
        const angle = 2 * Math.acos(Math.min(1, Math.abs(q.w)));
        if (angle > 1e-6) {
          const s = Math.sin(angle / 2);
          const axis = [q.x / s, q.y / s, q.z / s];
          cb({ mode: 'rotate', axis, angle, pivot });
        }
      } else if (mode === 'scale') {
        const sc = ctx.anchor.scale;
        if (Math.abs(sc.x - 1) > 1e-6 || Math.abs(sc.y - 1) > 1e-6 || Math.abs(sc.z - 1) > 1e-6) {
          cb({ mode: 'scale', factors: [sc.x, sc.y, sc.z], pivot });
        }
      }

      ctx.anchor.quaternion.identity();
      ctx.anchor.scale.set(1, 1, 1);
    }

    function onDragStart() {
      const pos = ctx.anchor.position;
      sceneRef.current._startPivot = [pos.x, pos.y, pos.z];
    }

    ctx.transformControls.addEventListener('mouseDown', onDragStart);
    ctx.transformControls.addEventListener('mouseUp', onDragEnd);
    return () => {
      ctx.transformControls.removeEventListener('mouseDown', onDragStart);
      ctx.transformControls.removeEventListener('mouseUp', onDragEnd);
    };
  }, []);

  return <div className="viewport" ref={containerRef} />;
}
