import { useEffect, useRef } from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

function frameCamera({ camera, controls }, { center, radius }) {
  const target = new THREE.Vector3(...center);
  const direction = new THREE.Vector3(1, 0.7, 1.2).normalize();
  camera.position.copy(target).addScaledVector(direction, radius * 2.6);
  camera.near = radius / 100;
  camera.far = radius * 100;
  camera.updateProjectionMatrix();
  controls.target.copy(target);
}

/** Orientation and tint for each selectable sketch plane. */
const SKETCH_PLANES = {
  XY: { rotation: [0, 0, 0], color: 0x4f9cf9 }, // normal +Z
  XZ: { rotation: [-Math.PI / 2, 0, 0], color: 0x5fdf8a }, // normal +Y
  YZ: { rotation: [0, Math.PI / 2, 0], color: 0xef6f6f }, // normal +X
};

/**
 * three.js canvas with orbit controls.
 *
 * `mesh` carries flat buffers ({ positions, normals, indices }) plus an
 * optional `frame` ({ center, radius }) that recenters the camera, and a
 * monotonically increasing `key` so identical-looking remeshes still apply.
 * `sketchPlane` ('XY' | 'XZ' | 'YZ' | null) shows a translucent plane
 * indicating the active sketch plane's orientation.
 */
export default function Viewport3D({ mesh, wireframe, sketchPlane }) {
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
      controls.update();
      renderer.render(scene, camera);
    });

    sceneRef.current = { renderer, camera, controls, material, meshObject };

    return () => {
      observer.disconnect();
      renderer.setAnimationLoop(null);
      controls.dispose();
      meshObject.geometry.dispose();
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

  return <div className="viewport" ref={containerRef} />;
}
