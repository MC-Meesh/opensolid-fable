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
use opensolid_frep::Profile2D;
use wasm_bindgen::prelude::*;

/// Closed 2D profile builder for [`WasmShape::extrude`] and
/// [`WasmShape::revolve`]: start at a point, chain `lineTo`/`arcTo`, then
/// `close()`. Arcs use the DXF bulge convention: `bulge = tan(sweep / 4)`,
/// positive sweeping counter-clockwise (`1` is a CCW semicircle).
#[wasm_bindgen]
pub struct WasmProfile2D {
    points: Vec<[f64; 2]>,
    /// Bulge of the segment leaving `points[i]`; `len == points.len() - 1`
    /// until `close()` completes the loop.
    bulges: Vec<f64>,
    closed: bool,
}

#[wasm_bindgen]
impl WasmProfile2D {
    /// Start a profile at `(x, y)`.
    #[wasm_bindgen(constructor)]
    pub fn new(x: f64, y: f64) -> WasmProfile2D {
        WasmProfile2D {
            points: vec![[x, y]],
            bulges: Vec::new(),
            closed: false,
        }
    }

    /// Straight segment from the current point to `(x, y)`. Ignored after
    /// `close()`.
    #[wasm_bindgen(js_name = lineTo)]
    pub fn line_to(&mut self, x: f64, y: f64) {
        self.arc_to(x, y, 0.0);
    }

    /// Circular arc from the current point to `(x, y)` with the given
    /// bulge (`tan(sweep / 4)`, positive = counter-clockwise; `0` is a
    /// straight line). Ignored after `close()`.
    #[wasm_bindgen(js_name = arcTo)]
    pub fn arc_to(&mut self, x: f64, y: f64, bulge: f64) {
        if !self.closed {
            self.points.push([x, y]);
            self.bulges.push(bulge);
        }
    }

    /// Close the loop with a straight segment back to the start point (a
    /// no-op segment if the profile already ends there). Further segments
    /// are ignored.
    pub fn close(&mut self) {
        self.closed = true;
    }
}

impl WasmProfile2D {
    /// Assemble the validated frep profile. Fails if the profile is not
    /// closed or violates [`Profile2D::new`]'s constraints.
    fn build(&self) -> Result<Profile2D, String> {
        if !self.closed {
            return Err("profile must be closed before sweeping (call close())".into());
        }
        let mut verts = self.points.clone();
        let mut bulges = self.bulges.clone();
        // Drop an explicit return to the start point; otherwise the
        // implicit closing segment is a straight line (bulge 0).
        let n = verts.len();
        if n >= 2 {
            let first = verts[0];
            let last = verts[n - 1];
            if (last[0] - first[0]).hypot(last[1] - first[1]) < 1e-9 {
                verts.pop();
            } else {
                bulges.push(0.0);
            }
        } else {
            bulges.push(0.0);
        }
        Profile2D::new(verts, bulges).map_err(|e| e.to_string())
    }
}

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

    /// The closed profile swept along +Y over `y ∈ [0, height]`; profile
    /// `(x, y)` coordinates map to world `(x, z)`.
    pub fn extrude(profile: &WasmProfile2D, height: f64) -> Result<WasmShape, String> {
        let p = profile.build()?;
        BoundedShape::extrude(p, height)
            .map(WasmShape)
            .map_err(|e| e.to_string())
    }

    /// The closed profile revolved around the Y axis through
    /// `angle_degrees` (in `(0, 360]`), sweeping from the +X half-plane
    /// towards +Z. Profile `(x, y)` maps to `(radius, y)`, so the profile
    /// must lie in `x >= 0`.
    pub fn revolve(profile: &WasmProfile2D, angle_degrees: f64) -> Result<WasmShape, String> {
        let p = profile.build()?;
        BoundedShape::revolve(p, angle_degrees.to_radians())
            .map(WasmShape)
            .map_err(|e| e.to_string())
    }

    /// This shape moved by `(x, y, z)`.
    pub fn translate(&self, x: f64, y: f64, z: f64) -> WasmShape {
        WasmShape(self.0.translate(Vector3::new(x, y, z)))
    }

    /// This shape rotated about the origin by `angle` radians around the
    /// axis `(ax, ay, az)` (any non-zero length). A zero or non-finite
    /// axis or angle is the identity rotation.
    pub fn rotate(&self, ax: f64, ay: f64, az: f64, angle: f64) -> WasmShape {
        let axis = Vector3::new(ax, ay, az);
        let axis_angle = if axis.norm().is_normal() && angle.is_finite() {
            axis.normalize() * angle
        } else {
            Vector3::zeros()
        };
        WasmShape(self.0.rotate(axis_angle))
    }

    /// This shape scaled per-axis about the origin (each factor `> 0`).
    /// Booleans and meshing stay correct, but the field is no longer an
    /// exact distance, so smooth-blend radii applied afterwards are
    /// distorted; prefer `uniformScale` when the factors are equal.
    pub fn scale(&self, sx: f64, sy: f64, sz: f64) -> Result<WasmShape, String> {
        self.0
            .scale(Vector3::new(sx, sy, sz))
            .map(WasmShape)
            .map_err(|e| e.to_string())
    }

    /// This shape scaled uniformly about the origin (`factor > 0`).
    #[wasm_bindgen(js_name = uniformScale)]
    pub fn uniform_scale(&self, factor: f64) -> Result<WasmShape, String> {
        self.0
            .uniform_scale(factor)
            .map(WasmShape)
            .map_err(|e| e.to_string())
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

    /// Signed distance from `(x, y, z)` to the surface: negative inside,
    /// positive outside. After smooth blends or anisotropic scaling this is
    /// not an exact Euclidean distance, but the sign and zero set stay
    /// correct, so nearest-surface queries can compare magnitudes.
    pub fn distance(&self, x: f64, y: f64, z: f64) -> f64 {
        self.0.distance(Point3::new(x, y, z))
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
    fn distance_via_wasm_api() {
        let s = WasmShape::sphere(1.0).translate(2.0, 0.0, 0.0);
        assert!((s.distance(4.0, 0.0, 0.0) - 1.0).abs() < 1e-12);
        assert!(s.distance(3.0, 0.0, 0.0).abs() < 1e-12);
        assert!(s.distance(2.0, 0.0, 0.0) < 0.0);
    }

    #[test]
    fn bounds_reports_translated_box() {
        let b = WasmShape::sphere(1.0).translate(2.0, 0.0, 0.0).bounds();
        assert_eq!(b, vec![1.0, -1.0, -1.0, 3.0, 1.0, 1.0]);
    }

    #[test]
    fn rotate_and_scale_mesh_via_wasm_api() {
        let s = WasmShape::box3(1.0, 0.4, 0.6)
            .rotate(0.0, 0.0, 1.0, std::f64::consts::FRAC_PI_2)
            .scale(1.5, 1.0, 2.0)
            .expect("valid factors")
            .translate(0.2, -0.1, 0.3);
        assert_valid(&s.mesh(32, None));

        // Quarter turn about z swaps the box's x/y bounds (then scaled).
        let b = WasmShape::box3(2.0, 1.0, 0.5)
            .rotate(0.0, 0.0, 1.0, std::f64::consts::FRAC_PI_2)
            .bounds();
        assert!((b[3] - 1.0).abs() < 1e-12 && (b[4] - 2.0).abs() < 1e-12);
    }

    #[test]
    fn uniform_scale_via_wasm_api() {
        let b = WasmShape::sphere(1.0)
            .uniform_scale(2.5)
            .expect("valid factor")
            .bounds();
        assert_eq!(b, vec![-2.5, -2.5, -2.5, 2.5, 2.5, 2.5]);
        assert!(WasmShape::sphere(1.0).uniform_scale(-1.0).is_err());
        assert!(WasmShape::sphere(1.0).scale(1.0, 0.0, 1.0).is_err());
    }

    #[test]
    fn zero_axis_rotation_is_identity() {
        let b = WasmShape::box3(2.0, 1.0, 0.5)
            .rotate(0.0, 0.0, 0.0, 1.0)
            .bounds();
        assert_eq!(b, vec![-2.0, -1.0, -0.5, 2.0, 1.0, 0.5]);
        let b = WasmShape::box3(2.0, 1.0, 0.5)
            .rotate(0.0, 0.0, 1.0, f64::NAN)
            .bounds();
        assert_eq!(b, vec![-2.0, -1.0, -0.5, 2.0, 1.0, 0.5]);
    }

    fn closed_square() -> WasmProfile2D {
        let mut p = WasmProfile2D::new(0.0, 0.0);
        p.line_to(1.0, 0.0);
        p.line_to(1.0, 1.0);
        p.line_to(0.0, 1.0);
        p.close();
        p
    }

    #[test]
    fn extrude_square_via_wasm_api() {
        let shape = WasmShape::extrude(&closed_square(), 2.0).expect("valid extrude");
        assert_eq!(shape.bounds(), vec![0.0, 0.0, 0.0, 1.0, 2.0, 1.0]);
        assert_valid(&shape.mesh(32, None));
    }

    #[test]
    fn extrude_profile_with_arcs_via_wasm_api() {
        // Rounded slot: two straight edges joined by semicircular caps.
        let mut p = WasmProfile2D::new(-0.5, -0.25);
        p.line_to(0.5, -0.25);
        p.arc_to(0.5, 0.25, 1.0);
        p.line_to(-0.5, 0.25);
        p.arc_to(-0.5, -0.25, 1.0); // explicit arc back to the start
        p.close();
        let shape = WasmShape::extrude(&p, 0.5).expect("valid extrude");
        let b = shape.bounds();
        // Semicircular caps extend the x reach by their radius 0.25.
        assert!((b[0] + 0.75).abs() < 1e-9 && (b[3] - 0.75).abs() < 1e-9);
        assert_valid(&shape.mesh(32, None));
    }

    #[test]
    fn revolve_full_and_partial_via_wasm_api() {
        let mut p = WasmProfile2D::new(0.5, 0.0);
        p.line_to(1.0, 0.0);
        p.line_to(1.0, 0.4);
        p.line_to(0.5, 0.4);
        p.close();
        let full = WasmShape::revolve(&p, 360.0).expect("valid revolve");
        assert_eq!(full.bounds(), vec![-1.0, 0.0, -1.0, 1.0, 0.4, 1.0]);
        assert_valid(&full.mesh(32, None));

        let partial = WasmShape::revolve(&p, 135.0).expect("valid revolve");
        assert_valid(&partial.mesh(32, None));
    }

    #[test]
    fn profile_errors_surface_as_strings() {
        // Unclosed profile.
        let mut open = WasmProfile2D::new(0.0, 0.0);
        open.line_to(1.0, 0.0);
        open.line_to(1.0, 1.0);
        let err = match WasmShape::extrude(&open, 1.0) {
            Ok(_) => panic!("must require close()"),
            Err(e) => e,
        };
        assert!(err.contains("close"), "got: {err}");

        // Too few segments.
        let mut tiny = WasmProfile2D::new(0.0, 0.0);
        tiny.close();
        assert!(WasmShape::extrude(&tiny, 1.0).is_err());

        // Bad height / angle / negative-x revolve profile.
        assert!(WasmShape::extrude(&closed_square(), 0.0).is_err());
        assert!(WasmShape::revolve(&closed_square(), 0.0).is_err());
        assert!(WasmShape::revolve(&closed_square(), 400.0).is_err());
        let mut neg = WasmProfile2D::new(-1.0, 0.0);
        neg.line_to(1.0, 0.0);
        neg.line_to(1.0, 1.0);
        neg.close();
        assert!(WasmShape::revolve(&neg, 360.0).is_err());
    }

    #[test]
    fn segments_after_close_are_ignored() {
        let mut p = closed_square();
        p.line_to(5.0, 5.0);
        p.arc_to(9.0, 9.0, 1.0);
        let shape = WasmShape::extrude(&p, 1.0).expect("valid extrude");
        assert_eq!(shape.bounds(), vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0]);
    }

    #[test]
    fn swept_shapes_compose_with_csg() {
        let plate = WasmShape::extrude(&closed_square(), 0.3).expect("valid extrude");
        let hole = WasmShape::cylinder(0.2, 1.0).translate(0.5, 0.15, 0.5);
        assert_valid(&plate.subtract(&hole).mesh(40, None));
    }
}
