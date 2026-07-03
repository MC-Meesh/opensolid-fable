//! WASM bindings for the OpenSolid F-Rep kernel.
//!
//! Exposes [`WasmShape`], a JS-friendly builder mirroring
//! [`opensolid_frep::Shape`]: primitive constructors, CSG combinators, and
//! `mesh()` producing flat `Float32Array`/`Uint32Array` buffers ready for
//! GPU upload (e.g. three.js `BufferGeometry`).
//!
//! All geometry and bounds logic lives in [`bounded`] as plain Rust so it is
//! covered by native `cargo test`; this layer only adapts types for
//! wasm-bindgen. Builds for `wasm32-unknown-unknown` with no threading
//! assumptions (the frep crate has no rayon dependency).

pub mod bounded;

use bounded::{BoundedShape, flatten_mesh};
use opensolid_core::types::{Point3, Vector3};
use wasm_bindgen::prelude::*;

/// Mesh buffers for JS consumption: xyz-interleaved positions and normals
/// (`Float32Array`), and flat triangle indices (`Uint32Array`), three per
/// triangle, wound counter-clockwise seen from outside.
#[wasm_bindgen(getter_with_clone)]
pub struct MeshData {
    pub positions: Vec<f32>,
    pub normals: Vec<f32>,
    pub indices: Vec<u32>,
}

/// Runtime-composable SDF shape. Methods never mutate: each returns a new
/// shape, so intermediate shapes can be reused freely from JS.
#[wasm_bindgen]
pub struct WasmShape(BoundedShape);

#[wasm_bindgen]
impl WasmShape {
    /// Sphere of the given radius, centered at the origin.
    pub fn sphere(radius: f64) -> WasmShape {
        WasmShape(BoundedShape::sphere(radius))
    }

    /// Axis-aligned box with half-extents `(hx, hy, hz)`, centered at the
    /// origin.
    pub fn box3(hx: f64, hy: f64, hz: f64) -> WasmShape {
        WasmShape(BoundedShape::box3(hx, hy, hz))
    }

    /// Box with rounded edges: outer half-extents `(hx, hy, hz)` including
    /// the rounding, edge radius `radius` (≤ the smallest half-extent),
    /// centered at the origin.
    #[wasm_bindgen(js_name = roundedBox)]
    pub fn rounded_box(hx: f64, hy: f64, hz: f64, radius: f64) -> WasmShape {
        WasmShape(BoundedShape::rounded_box(hx, hy, hz, radius))
    }

    /// Cylinder along the y axis: radius in the xz plane, y ∈ ±half_height.
    pub fn cylinder(radius: f64, half_height: f64) -> WasmShape {
        WasmShape(BoundedShape::cylinder(radius, half_height))
    }

    /// Torus with its ring in the xz plane, centered at the origin.
    pub fn torus(major_radius: f64, minor_radius: f64) -> WasmShape {
        WasmShape(BoundedShape::torus(major_radius, minor_radius))
    }

    /// Capsule (sphere-swept segment) from `(x1,y1,z1)` to `(x2,y2,z2)`.
    #[allow(clippy::too_many_arguments)]
    pub fn capsule(x1: f64, y1: f64, z1: f64, x2: f64, y2: f64, z2: f64, radius: f64) -> WasmShape {
        WasmShape(BoundedShape::capsule(
            Point3::new(x1, y1, z1),
            Point3::new(x2, y2, z2),
            radius,
        ))
    }

    /// This shape moved by `(x, y, z)`.
    pub fn translate(&self, x: f64, y: f64, z: f64) -> WasmShape {
        WasmShape(self.0.translate(Vector3::new(x, y, z)))
    }

    /// Boolean union with `other`.
    pub fn union(&self, other: &WasmShape) -> WasmShape {
        WasmShape(self.0.union(&other.0))
    }

    /// Boolean intersection with `other`.
    pub fn intersect(&self, other: &WasmShape) -> WasmShape {
        WasmShape(self.0.intersect(&other.0))
    }

    /// Boolean subtraction of `other` from this shape.
    pub fn subtract(&self, other: &WasmShape) -> WasmShape {
        WasmShape(self.0.subtract(&other.0))
    }

    /// Smooth (filleted) union with `other`. Omitting `radius` picks 10% of
    /// the combined bounding box's largest extent.
    #[wasm_bindgen(js_name = smoothUnion)]
    pub fn smooth_union(&self, other: &WasmShape, radius: Option<f64>) -> WasmShape {
        WasmShape(self.0.smooth_union(&other.0, radius))
    }

    /// Conservative axis-aligned bounding box of the surface as
    /// `[min_x, min_y, min_z, max_x, max_y, max_z]` (useful for camera
    /// framing).
    pub fn bounds(&self) -> Vec<f64> {
        let b = &self.0.bounds;
        vec![b.min.x, b.min.y, b.min.z, b.max.x, b.max.y, b.max.z]
    }

    /// Mesh the shape on a `resolution`³ dual-contouring grid. With `bound`
    /// set, the grid covers the explicit cube `[-bound, bound]³` (the surface
    /// must lie strictly inside it); otherwise bounds are derived from the
    /// shape's tracked bounding box with padding.
    pub fn mesh(&self, resolution: u32, bound: Option<f64>) -> MeshData {
        let mesh = self.0.mesh(resolution as usize, bound);
        let flat = flatten_mesh(&mesh);
        MeshData {
            positions: flat.positions,
            normals: flat.normals,
            indices: flat.indices,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_valid(data: &MeshData) {
        assert!(!data.positions.is_empty());
        assert_eq!(data.positions.len(), data.normals.len());
        assert_eq!(data.positions.len() % 3, 0);
        assert_eq!(data.indices.len() % 3, 0);
        let vertex_count = (data.positions.len() / 3) as u32;
        assert!(data.indices.iter().all(|&i| i < vertex_count));
    }

    #[test]
    fn sphere_meshes_via_wasm_api() {
        let data = WasmShape::sphere(1.0).mesh(24, None);
        assert_valid(&data);
    }

    #[test]
    fn playground_default_demo_meshes() {
        // The playground's default snippet: rounded box smooth-united with a
        // sphere, with a cylinder hole subtracted.
        let body = WasmShape::rounded_box(1.0, 0.6, 0.8, 0.15);
        let bump = WasmShape::sphere(0.55).translate(0.0, 0.7, 0.0);
        let hole = WasmShape::cylinder(0.3, 2.0);
        let part = body.smooth_union(&bump, Some(0.25)).subtract(&hole);
        assert_valid(&part.mesh(48, None));
    }

    #[test]
    fn builder_chain_meshes() {
        let base = WasmShape::box3(1.0, 0.4, 1.0);
        let hole = WasmShape::cylinder(0.4, 1.0);
        let bump = WasmShape::sphere(0.5).translate(0.0, 0.5, 0.0);
        let part = base.subtract(&hole).smooth_union(&bump, Some(0.2));
        let data = part.mesh(32, None);
        assert_valid(&data);

        // Operands stay usable after being combined (no move semantics).
        assert_valid(&base.mesh(16, None));
        assert_valid(&hole.union(&bump).mesh(16, None));
    }

    #[test]
    fn torus_capsule_and_explicit_bound() {
        assert_valid(&WasmShape::torus(1.0, 0.25).mesh(24, None));
        let cap = WasmShape::capsule(-0.5, 0.0, 0.0, 0.5, 0.5, 0.0, 0.3);
        assert_valid(&cap.mesh(24, None));
        assert_valid(&cap.mesh(24, Some(2.0)));
    }

    #[test]
    fn bounds_reports_translated_box() {
        let b = WasmShape::sphere(1.0).translate(2.0, 0.0, 0.0).bounds();
        assert_eq!(b, vec![1.0, -1.0, -1.0, 3.0, 1.0, 1.0]);
    }
}
