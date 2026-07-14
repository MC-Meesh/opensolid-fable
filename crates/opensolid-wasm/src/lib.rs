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
pub mod exact;
pub mod step;

use bounded::{BoundedShape, flatten_mesh};
use exact::{ExactPrim, ExactRep, ExactSpec};
use opensolid_core::mesh::TriangleMesh;
use opensolid_core::types::{Point3, Vector3};
use opensolid_frep::{OpenPath2D, Profile2D, RibSide};
use opensolid_kernel::brep::BooleanOp;
use opensolid_kernel::mass_properties;
use std::rc::Rc;
use wasm_bindgen::prelude::*;

/// One queued segment of a WASM profile/path builder, replayed onto the
/// frep [`Profile2DBuilder`](opensolid_frep::Profile2DBuilder) /
/// [`OpenPath2DBuilder`](opensolid_frep::OpenPath2DBuilder) at `build()` time.
/// Each continues from the previous segment's endpoint.
#[derive(Clone, Copy)]
enum PathSeg {
    /// Straight segment to `to`.
    Line { to: [f64; 2] },
    /// Circular arc to `to` with the DXF `bulge` (`tan(sweep / 4)`).
    Arc { to: [f64; 2], bulge: f64 },
    /// Elliptical arc to endpoint `to` along the ellipse centred at `center`
    /// with semi-axes `rx`/`ry` rotated by `rotation` (radians), sweeping
    /// counter-clockwise when `ccw` else clockwise. Both the current point
    /// and `to` must lie on that ellipse; the eccentric-angle sweep the
    /// kernel needs is recovered from the two endpoints at build time.
    Ellipse {
        to: [f64; 2],
        center: [f64; 2],
        rx: f64,
        ry: f64,
        rotation: f64,
        ccw: bool,
    },
    /// Cubic Bézier to `to` with control points `c1`, `c2`.
    Cubic {
        c1: [f64; 2],
        c2: [f64; 2],
        to: [f64; 2],
    },
}

/// Eccentric-angle sweep (radians) taking `a` to `b` along the ellipse
/// (`center`, semi-axes `rx`/`ry`, rotation `phi` radians) in the `ccw`
/// direction. Both endpoints are projected onto the ellipse frame to recover
/// their eccentric angles; the signed difference is wrapped into the half-open
/// turn matching `ccw` (a coincident `a == b` yields a full ±2π sweep). Feeds
/// the frep builder's `ellipse_to`, which re-derives the geometry.
fn ellipse_endpoint_sweep(
    a: [f64; 2],
    b: [f64; 2],
    center: [f64; 2],
    rx: f64,
    ry: f64,
    phi: f64,
    ccw: bool,
) -> f64 {
    let (cphi, sphi) = (phi.cos(), phi.sin());
    let eccentric = |p: [f64; 2]| {
        let (dx, dy) = (p[0] - center[0], p[1] - center[1]);
        let lx = cphi * dx + sphi * dy;
        let ly = -sphi * dx + cphi * dy;
        (ly / ry).atan2(lx / rx)
    };
    let mut sweep = eccentric(b) - eccentric(a);
    // Wrap into the turn implied by the direction: CCW → (0, 2π], CW → [-2π, 0).
    // NaN radii (rejected later by the kernel) fail both comparisons and pass
    // through unchanged rather than looping forever.
    if ccw {
        while sweep <= 0.0 {
            sweep += std::f64::consts::TAU;
        }
    } else {
        while sweep >= 0.0 {
            sweep -= std::f64::consts::TAU;
        }
    }
    sweep
}

/// Closed 2D profile builder for [`WasmShape::extrude`] and
/// [`WasmShape::revolve`]: start at a point, chain
/// `lineTo`/`arcTo`/`ellipseArcTo`/`cubicTo`, then `close()`. Arcs use the
/// DXF bulge convention: `bulge = tan(sweep / 4)`, positive sweeping
/// counter-clockwise (`1` is a CCW semicircle). `ellipseArcTo` takes the
/// arc's endpoint, ellipse centre/radii, a `rotation` in **degrees**, and a
/// `ccw` direction flag.
#[wasm_bindgen]
pub struct WasmProfile2D {
    start: [f64; 2],
    segs: Vec<PathSeg>,
    closed: bool,
}

#[wasm_bindgen]
impl WasmProfile2D {
    /// Start a profile at `(x, y)`.
    #[wasm_bindgen(constructor)]
    pub fn new(x: f64, y: f64) -> WasmProfile2D {
        WasmProfile2D {
            start: [x, y],
            segs: Vec::new(),
            closed: false,
        }
    }

    /// Straight segment from the current point to `(x, y)`. Ignored after
    /// `close()`.
    #[wasm_bindgen(js_name = lineTo)]
    pub fn line_to(&mut self, x: f64, y: f64) {
        if !self.closed {
            self.segs.push(PathSeg::Line { to: [x, y] });
        }
    }

    /// Circular arc from the current point to `(x, y)` with the given
    /// bulge (`tan(sweep / 4)`, positive = counter-clockwise; `0` is a
    /// straight line). Ignored after `close()`.
    #[wasm_bindgen(js_name = arcTo)]
    pub fn arc_to(&mut self, x: f64, y: f64, bulge: f64) {
        if !self.closed {
            self.segs.push(PathSeg::Arc { to: [x, y], bulge });
        }
    }

    /// Elliptical arc from the current point to endpoint `(x, y)` along the
    /// ellipse centred at `(cx, cy)` with semi-axes `rx`/`ry` rotated by
    /// `rotationDegrees`, sweeping counter-clockwise when `ccw` else
    /// clockwise. Both the current point and `(x, y)` must lie on that
    /// ellipse. Ignored after `close()`.
    #[wasm_bindgen(js_name = ellipseArcTo)]
    #[allow(clippy::too_many_arguments)]
    pub fn ellipse_arc_to(
        &mut self,
        x: f64,
        y: f64,
        cx: f64,
        cy: f64,
        rx: f64,
        ry: f64,
        rotation_degrees: f64,
        ccw: bool,
    ) {
        if !self.closed {
            self.segs.push(PathSeg::Ellipse {
                to: [x, y],
                center: [cx, cy],
                rx,
                ry,
                rotation: rotation_degrees.to_radians(),
                ccw,
            });
        }
    }

    /// Cubic Bézier from the current point to `(x, y)` with control points
    /// `(c1x, c1y)` and `(c2x, c2y)`. Ignored after `close()`.
    #[wasm_bindgen(js_name = cubicTo)]
    #[allow(clippy::too_many_arguments)]
    pub fn cubic_to(&mut self, c1x: f64, c1y: f64, c2x: f64, c2y: f64, x: f64, y: f64) {
        if !self.closed {
            self.segs.push(PathSeg::Cubic {
                c1: [c1x, c1y],
                c2: [c2x, c2y],
                to: [x, y],
            });
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
    /// closed or violates the [`Profile2DBuilder`](opensolid_frep::Profile2DBuilder)
    /// constraints. The builder closes the loop with a straight segment
    /// back to the start (a no-op if the path already ends there).
    fn build(&self) -> Result<Profile2D, String> {
        if !self.closed {
            return Err("profile must be closed before sweeping (call close())".into());
        }
        let mut b = Profile2D::builder(self.start);
        let mut cursor = self.start;
        for seg in &self.segs {
            b = match *seg {
                PathSeg::Line { to } => {
                    cursor = to;
                    b.line_to(to)
                }
                PathSeg::Arc { to, bulge } => {
                    cursor = to;
                    b.arc_to(to, bulge)
                }
                PathSeg::Ellipse {
                    to,
                    center,
                    rx,
                    ry,
                    rotation,
                    ccw,
                } => {
                    let sweep = ellipse_endpoint_sweep(cursor, to, center, rx, ry, rotation, ccw);
                    cursor = to;
                    b.ellipse_to(center, rx, ry, rotation, sweep)
                }
                PathSeg::Cubic { c1, c2, to } => {
                    cursor = to;
                    b.cubic_to(c1, c2, to)
                }
            };
        }
        b.build().map_err(|e| e.to_string())
    }
}

/// Build an [`EdgeRegion`](opensolid_frep::EdgeRegion) from a flat
/// `[x, y, z, …]` polyline. A trailing partial point (length not a multiple
/// of 3) is dropped; fewer than two points yields an empty region (no blend).
fn polyline_region(edge: &[f64]) -> opensolid_frep::EdgeRegion {
    let points: Vec<Point3> = edge
        .chunks_exact(3)
        .map(|c| Point3::new(c[0], c[1], c[2]))
        .collect();
    opensolid_frep::EdgeRegion::from_polyline(&points)
}

/// Open 2D path builder for [`WasmShape::rib`]: start at a point, chain
/// `lineTo`/`arcTo`/`ellipseArcTo`/`cubicTo`, and pass it straight to `rib`
/// (no `close()` — the path stays open). Arcs use the same DXF bulge
/// convention as [`WasmProfile2D`]: `bulge = tan(sweep / 4)`, positive
/// sweeping counter-clockwise. `ellipseArcTo` takes the same endpoint +
/// `ccw` form as [`WasmProfile2D::ellipse_arc_to`].
#[wasm_bindgen]
pub struct WasmOpenPath2D {
    start: [f64; 2],
    segs: Vec<PathSeg>,
}

#[wasm_bindgen]
impl WasmOpenPath2D {
    /// Start a path at `(x, y)`.
    #[wasm_bindgen(constructor)]
    pub fn new(x: f64, y: f64) -> WasmOpenPath2D {
        WasmOpenPath2D {
            start: [x, y],
            segs: Vec::new(),
        }
    }

    /// Straight segment from the current point to `(x, y)`.
    #[wasm_bindgen(js_name = lineTo)]
    pub fn line_to(&mut self, x: f64, y: f64) {
        self.segs.push(PathSeg::Line { to: [x, y] });
    }

    /// Circular arc from the current point to `(x, y)` with the given bulge
    /// (`tan(sweep / 4)`, positive = counter-clockwise; `0` is a straight
    /// line).
    #[wasm_bindgen(js_name = arcTo)]
    pub fn arc_to(&mut self, x: f64, y: f64, bulge: f64) {
        self.segs.push(PathSeg::Arc { to: [x, y], bulge });
    }

    /// Elliptical arc from the current point to endpoint `(x, y)` (see
    /// [`WasmProfile2D::ellipse_arc_to`]): along the ellipse centred at
    /// `(cx, cy)` with semi-axes `rx`/`ry` rotated by `rotationDegrees`,
    /// sweeping counter-clockwise when `ccw` else clockwise.
    #[wasm_bindgen(js_name = ellipseArcTo)]
    #[allow(clippy::too_many_arguments)]
    pub fn ellipse_arc_to(
        &mut self,
        x: f64,
        y: f64,
        cx: f64,
        cy: f64,
        rx: f64,
        ry: f64,
        rotation_degrees: f64,
        ccw: bool,
    ) {
        self.segs.push(PathSeg::Ellipse {
            to: [x, y],
            center: [cx, cy],
            rx,
            ry,
            rotation: rotation_degrees.to_radians(),
            ccw,
        });
    }

    /// Cubic Bézier from the current point to `(x, y)` with control points
    /// `(c1x, c1y)` and `(c2x, c2y)`.
    #[wasm_bindgen(js_name = cubicTo)]
    #[allow(clippy::too_many_arguments)]
    pub fn cubic_to(&mut self, c1x: f64, c1y: f64, c2x: f64, c2y: f64, x: f64, y: f64) {
        self.segs.push(PathSeg::Cubic {
            c1: [c1x, c1y],
            c2: [c2x, c2y],
            to: [x, y],
        });
    }
}

impl WasmOpenPath2D {
    /// Assemble the validated frep open path. Fails per the
    /// [`OpenPath2DBuilder`](opensolid_frep::OpenPath2DBuilder) constraints
    /// (no segments, coincident endpoints, non-finite input).
    fn build(&self) -> Result<OpenPath2D, String> {
        let mut b = OpenPath2D::builder(self.start);
        let mut cursor = self.start;
        for seg in &self.segs {
            b = match *seg {
                PathSeg::Line { to } => {
                    cursor = to;
                    b.line_to(to)
                }
                PathSeg::Arc { to, bulge } => {
                    cursor = to;
                    b.arc_to(to, bulge)
                }
                PathSeg::Ellipse {
                    to,
                    center,
                    rx,
                    ry,
                    rotation,
                    ccw,
                } => {
                    let sweep = ellipse_endpoint_sweep(cursor, to, center, rx, ry, rotation, ccw);
                    cursor = to;
                    b.ellipse_to(center, rx, ry, rotation, sweep)
                }
                PathSeg::Cubic { c1, c2, to } => {
                    cursor = to;
                    b.cubic_to(c1, c2, to)
                }
            };
        }
        b.build().map_err(|e| e.to_string())
    }
}

/// 3D polyline path builder for [`WasmShape::sweep`]: start at a point, chain
/// `lineTo` for each subsequent vertex. Unlike a profile it is not closed —
/// it is an open path the profile is swept along.
#[wasm_bindgen]
pub struct WasmPath3D {
    points: Vec<[f64; 3]>,
}

#[wasm_bindgen]
impl WasmPath3D {
    /// Start a path at `(x, y, z)`.
    #[wasm_bindgen(constructor)]
    pub fn new(x: f64, y: f64, z: f64) -> WasmPath3D {
        WasmPath3D {
            points: vec![[x, y, z]],
        }
    }

    /// Extend the path with a straight segment to `(x, y, z)`.
    #[wasm_bindgen(js_name = lineTo)]
    pub fn line_to(&mut self, x: f64, y: f64, z: f64) {
        self.points.push([x, y, z]);
    }
}

/// Dihedral threshold (radians) above which a mesh edge counts as a crease
/// when populating [`MeshData::feature_edges`]. 40° keeps genuine CSG/box
/// edges while ignoring the shallow facet seams of a tessellated curved
/// surface.
const FEATURE_EDGE_DIHEDRAL: f64 = 40.0 * std::f64::consts::PI / 180.0;

/// Flatten a list of 3D edge segments into the `[x0,y0,z0, x1,y1,z1, …]`
/// buffer JS consumes (two points per edge), the same convention as the
/// `filletEdge`/`chamferEdge` polyline input.
fn flatten_edges(edges: &[[Point3; 2]]) -> Vec<f32> {
    edges
        .iter()
        .flat_map(|[a, b]| {
            [
                a.x as f32, a.y as f32, a.z as f32, b.x as f32, b.y as f32, b.z as f32,
            ]
        })
        .collect()
}

/// Build a [`MeshData`] from a mesh, including its crease/boundary feature
/// edges at [`FEATURE_EDGE_DIHEDRAL`].
fn mesh_data(mesh: &TriangleMesh) -> MeshData {
    let flat = flatten_mesh(mesh);
    MeshData {
        positions: flat.positions,
        normals: flat.normals,
        indices: flat.indices,
        feature_edges: flatten_edges(&mesh.feature_edges(FEATURE_EDGE_DIHEDRAL)),
    }
}

/// Mesh buffers for JS consumption: xyz-interleaved positions and normals
/// (`Float32Array`), and flat triangle indices (`Uint32Array`), three per
/// triangle, wound counter-clockwise seen from outside.
///
/// `feature_edges` (JS `featureEdges`) is the raw drawing line-work source:
/// crease/boundary edges of the tessellated solid as a flat
/// `[x0,y0,z0, x1,y1,z1, …]` segment buffer (two points per edge), the same
/// convention `filletEdge`/`chamferEdge` take as input. Coplanar facet seams
/// are excluded; silhouette (outline) edges are view-dependent and come from
/// [`WasmShape::silhouette_edges`] instead.
#[wasm_bindgen(getter_with_clone)]
pub struct MeshData {
    pub positions: Vec<f32>,
    pub normals: Vec<f32>,
    pub indices: Vec<u32>,
    #[wasm_bindgen(js_name = featureEdges)]
    pub feature_edges: Vec<f32>,
}

/// Runtime-composable SDF shape. Methods never mutate: each returns a new
/// shape, so intermediate shapes can be reused freely from JS.
///
/// Shapes within the kernel's exact coverage (sphere/box/cylinder/torus,
/// rigid transforms, uniform scale, sharp booleans) also carry an exact
/// B-Rep companion ([`exact`]). With `setExactBooleans(true)`, booleans
/// try the kernel's exact pipeline first and `mesh()` serves the
/// validated analytic tessellation; anything outside exact coverage
/// falls back to the SDF path unchanged.
#[wasm_bindgen]
pub struct WasmShape {
    inner: BoundedShape,
    exact: Option<ExactRep>,
}

impl WasmShape {
    fn sdf_only(inner: BoundedShape) -> WasmShape {
        WasmShape { inner, exact: None }
    }

    fn with_prim(inner: BoundedShape, prim: ExactPrim) -> WasmShape {
        WasmShape {
            inner,
            exact: Some(ExactRep::Spec(ExactSpec::new(prim))),
        }
    }

    /// The exact spec transformed by `f`, if this shape is still a
    /// (transformed) primitive; boolean results drop exactness under
    /// transforms (their store-backed body is shared — future work).
    fn map_spec(&self, f: impl FnOnce(&ExactSpec) -> Option<ExactSpec>) -> Option<ExactRep> {
        match self.exact.as_ref()? {
            ExactRep::Spec(spec) => f(spec).map(ExactRep::Spec),
            ExactRep::Boolean(_) => None,
        }
    }

    /// Try the exact pipeline for a sharp boolean; `None` leaves the SDF
    /// composition standing alone, exactly as with the toggle off.
    fn try_exact_boolean(&self, other: &WasmShape, op: BooleanOp) -> Option<ExactRep> {
        if !exact::exact_enabled() {
            return None;
        }
        let (a, b) = (self.exact.as_ref()?, other.exact.as_ref()?);
        exact::exact_boolean(op, a, b).map(|out| ExactRep::Boolean(Rc::new(out)))
    }

    /// Whether `measure`/`validate` will read the validated exact
    /// tessellation rather than an adaptive SDF mesh.
    fn measured_exact(&self) -> bool {
        exact::exact_enabled()
            && self
                .exact
                .as_ref()
                .is_some_and(|r| r.exact_mesh().is_some())
    }

    /// Run `f` over the mesh that measurement should use: the validated
    /// exact tessellation when this shape resolves to one (see
    /// [`measured_exact`](Self::measured_exact)), otherwise an adaptive SDF
    /// mesh at `accuracy` (non-finite/non-positive/absent falls back to 0.5%
    /// of the shape's extent — the same default as STEP export).
    fn with_measure_mesh<R>(&self, accuracy: Option<f64>, f: impl FnOnce(&TriangleMesh) -> R) -> R {
        if exact::exact_enabled() {
            if let Some(mesh) = self.exact.as_ref().and_then(|r| r.exact_mesh()) {
                return f(mesh);
            }
        }
        let size = self.inner.bounds.max - self.inner.bounds.min;
        let extent = size.x.max(size.y).max(size.z).max(1e-9);
        let accuracy = match accuracy {
            Some(a) if a.is_finite() && a > 0.0 => a,
            _ => 5e-3 * extent,
        };
        f(&self.inner.mesh_adaptive(accuracy, None))
    }
}

/// Format a float as a JSON number, emitting `null` for non-finite values
/// (JSON has no NaN/Infinity literal).
fn json_num(x: f64) -> String {
    if x.is_finite() {
        format!("{x}")
    } else {
        "null".to_string()
    }
}

/// Escape a string for embedding as a JSON string literal (backslash and
/// quote only — kernel error messages contain no control characters).
fn json_escape(s: &str) -> String {
    s.replace('\\', "\\\\").replace('"', "\\\"")
}

#[wasm_bindgen]
impl WasmShape {
    /// Sphere of the given radius, centered at the origin.
    pub fn sphere(radius: f64) -> WasmShape {
        WasmShape::with_prim(BoundedShape::sphere(radius), ExactPrim::Sphere { radius })
    }

    /// Axis-aligned box with half-extents `(hx, hy, hz)`, centered at the
    /// origin.
    pub fn box3(hx: f64, hy: f64, hz: f64) -> WasmShape {
        WasmShape::with_prim(
            BoundedShape::box3(hx, hy, hz),
            ExactPrim::Block { hx, hy, hz },
        )
    }

    /// Box with rounded edges: outer half-extents `(hx, hy, hz)` including
    /// the rounding, edge radius `radius` (≤ the smallest half-extent),
    /// centered at the origin.
    #[wasm_bindgen(js_name = roundedBox)]
    pub fn rounded_box(hx: f64, hy: f64, hz: f64, radius: f64) -> WasmShape {
        WasmShape::sdf_only(BoundedShape::rounded_box(hx, hy, hz, radius))
    }

    /// Cylinder along the y axis: radius in the xz plane, y ∈ ±half_height.
    pub fn cylinder(radius: f64, half_height: f64) -> WasmShape {
        WasmShape::with_prim(
            BoundedShape::cylinder(radius, half_height),
            ExactPrim::Cylinder {
                radius,
                half_height,
            },
        )
    }

    /// Torus with its ring in the xz plane, centered at the origin.
    pub fn torus(major_radius: f64, minor_radius: f64) -> WasmShape {
        WasmShape::with_prim(
            BoundedShape::torus(major_radius, minor_radius),
            ExactPrim::Torus {
                major: major_radius,
                minor: minor_radius,
            },
        )
    }

    /// Cone/frustum along the y axis: `radius_bottom` at `y = -half_height`,
    /// `radius_top` at `y = +half_height`. Either radius may be zero for a
    /// pointed tip (but not both).
    pub fn cone(radius_bottom: f64, radius_top: f64, half_height: f64) -> WasmShape {
        WasmShape::with_prim(
            BoundedShape::cone(radius_bottom, radius_top, half_height),
            ExactPrim::Cone {
                radius_bottom,
                radius_top,
                half_height,
            },
        )
    }

    /// Capsule (sphere-swept segment) from `(x1,y1,z1)` to `(x2,y2,z2)`.
    #[allow(clippy::too_many_arguments)]
    pub fn capsule(x1: f64, y1: f64, z1: f64, x2: f64, y2: f64, z2: f64, radius: f64) -> WasmShape {
        WasmShape::sdf_only(BoundedShape::capsule(
            Point3::new(x1, y1, z1),
            Point3::new(x2, y2, z2),
            radius,
        ))
    }

    /// The closed profile swept along +Y over `y ∈ [0, height]`; profile
    /// `(x, y)` coordinates map to world `(x, z)`. The optional `draftDegrees`
    /// tapers the walls along the sweep — positive narrows toward the top
    /// cap (mold-release draft), negative flares outward; omitted or `0` is a
    /// straight prism. `|draft|` must stay under ~80°.
    #[wasm_bindgen(js_name = extrude)]
    pub fn extrude(
        profile: &WasmProfile2D,
        height: f64,
        draft_degrees: Option<f64>,
    ) -> Result<WasmShape, String> {
        let p = profile.build()?;
        let draft = draft_degrees.unwrap_or(0.0).to_radians();
        BoundedShape::extrude_draft(p, height, draft)
            .map(WasmShape::sdf_only)
            .map_err(|e| e.to_string())
    }

    /// A half-space for terminating an "up to face" extrude: the solid half
    /// on the negative side of the plane through `(px, py, pz)` with outward
    /// normal `(nx, ny, nz)`. Unbounded on its own — intersect it with a
    /// through-all extrude to clip the extrude at that plane.
    #[wasm_bindgen(js_name = halfSpace)]
    #[allow(clippy::too_many_arguments)]
    pub fn half_space(px: f64, py: f64, pz: f64, nx: f64, ny: f64, nz: f64) -> WasmShape {
        WasmShape::sdf_only(BoundedShape::half_space(
            Point3::new(px, py, pz),
            Vector3::new(nx, ny, nz),
        ))
    }

    /// The closed profile revolved around the Y axis through
    /// `angle_degrees` (in `(0, 360]`), sweeping from the +X half-plane
    /// towards +Z. Profile `(x, y)` maps to `(radius, y)`, so the profile
    /// must lie in `x >= 0`.
    pub fn revolve(profile: &WasmProfile2D, angle_degrees: f64) -> Result<WasmShape, String> {
        let p = profile.build()?;
        BoundedShape::revolve(p, angle_degrees.to_radians())
            .map(WasmShape::sdf_only)
            .map_err(|e| e.to_string())
    }

    /// The open path thickened into a support rib and swept along +Y over
    /// `y ∈ [0, height]`; path `(x, y)` maps to world `(x, z)`. `side` picks
    /// which side of the path receives material: `"both"` (symmetric,
    /// `thickness/2` each way — the exact-distance default), `"first"` (the
    /// full `thickness` on the left of the path's travel direction), or
    /// `"second"` (the full `thickness` on the right). Union the result with
    /// the parent body at the script level.
    pub fn rib(
        path: &WasmOpenPath2D,
        thickness: f64,
        height: f64,
        side: &str,
    ) -> Result<WasmShape, String> {
        let p = path.build()?;
        let side = match side.to_ascii_lowercase().as_str() {
            "both" => RibSide::Both,
            "first" => RibSide::First,
            "second" => RibSide::Second,
            other => {
                return Err(format!(
                    "unknown rib side {other:?} (want both/first/second)"
                ));
            }
        };
        BoundedShape::rib(p, thickness, height, side)
            .map(WasmShape::sdf_only)
            .map_err(|e| e.to_string())
    }

    /// The closed profile swept along the polyline `path`. The profile's
    /// local `(x, y)` origin rides on the path, kept twist-free (constant
    /// orientation) along each segment; joints are mitred by the union of the
    /// per-segment prisms. MVP: constant profile, no twist.
    pub fn sweep(profile: &WasmProfile2D, path: &WasmPath3D) -> Result<WasmShape, String> {
        let p = profile.build()?;
        BoundedShape::sweep(p, &path.points)
            .map(WasmShape::sdf_only)
            .map_err(|e| e.to_string())
    }

    /// A loft between two closed profiles on parallel planes: `bottom` on
    /// `y = 0` and `top` on `y = height`, blended by linearly morphing their
    /// signed distances along `y`. MVP: parallel planes only, linear morph.
    pub fn loft(
        bottom: &WasmProfile2D,
        top: &WasmProfile2D,
        height: f64,
    ) -> Result<WasmShape, String> {
        let b = bottom.build()?;
        let t = top.build()?;
        BoundedShape::loft(b, t, height)
            .map(WasmShape::sdf_only)
            .map_err(|e| e.to_string())
    }

    /// This shape moved by `(x, y, z)`.
    pub fn translate(&self, x: f64, y: f64, z: f64) -> WasmShape {
        let offset = Vector3::new(x, y, z);
        WasmShape {
            inner: self.inner.translate(offset),
            exact: self.map_spec(|s| Some(s.translated(offset))),
        }
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
        WasmShape {
            inner: self.inner.rotate(axis_angle),
            exact: self.map_spec(|s| Some(s.rotated(axis_angle))),
        }
    }

    /// This shape scaled per-axis about the origin (each factor `> 0`).
    /// Booleans and meshing stay correct, but the field is no longer an
    /// exact distance, so smooth-blend radii applied afterwards are
    /// distorted; prefer `uniformScale` when the factors are equal.
    pub fn scale(&self, sx: f64, sy: f64, sz: f64) -> Result<WasmShape, String> {
        self.inner
            .scale(Vector3::new(sx, sy, sz))
            .map(WasmShape::sdf_only)
            .map_err(|e| e.to_string())
    }

    /// This shape scaled uniformly about the origin (`factor > 0`).
    #[wasm_bindgen(js_name = uniformScale)]
    pub fn uniform_scale(&self, factor: f64) -> Result<WasmShape, String> {
        let inner = self
            .inner
            .uniform_scale(factor)
            .map_err(|e| e.to_string())?;
        Ok(WasmShape {
            inner,
            exact: self.map_spec(|s| s.uniform_scaled(factor)),
        })
    }

    /// This shape tapered (drafted) about the plane through
    /// `(nx, ny, nz)` with pull direction `(px, py, pz)`, by draft
    /// `angle_degrees`. Side walls flare toward the pull direction above the
    /// neutral plane and pinch below it — the mold-release draft about a
    /// parting plane. The field stays sign- and surface-exact but is no
    /// longer an exact distance, so blends applied afterward are distorted;
    /// this is the F-Rep whole-body approximation of a face-selective draft.
    #[allow(clippy::too_many_arguments)]
    pub fn taper(
        &self,
        px: f64,
        py: f64,
        pz: f64,
        nx: f64,
        ny: f64,
        nz: f64,
        angle_degrees: f64,
    ) -> Result<WasmShape, String> {
        self.inner
            .taper(
                Vector3::new(px, py, pz),
                Point3::new(nx, ny, nz),
                angle_degrees.to_radians(),
            )
            .map(WasmShape::sdf_only)
            .map_err(|e| e.to_string())
    }

    /// Boolean union with `other`.
    pub fn union(&self, other: &WasmShape) -> WasmShape {
        WasmShape {
            exact: self.try_exact_boolean(other, BooleanOp::Unite),
            inner: self.inner.union(&other.inner),
        }
    }

    /// Boolean intersection with `other`.
    pub fn intersect(&self, other: &WasmShape) -> WasmShape {
        WasmShape {
            exact: self.try_exact_boolean(other, BooleanOp::Intersect),
            inner: self.inner.intersect(&other.inner),
        }
    }

    /// Boolean subtraction of `other` from this shape.
    pub fn subtract(&self, other: &WasmShape) -> WasmShape {
        WasmShape {
            exact: self.try_exact_boolean(other, BooleanOp::Subtract),
            inner: self.inner.subtract(&other.inner),
        }
    }

    /// Smooth (filleted) union with `other`. Omitting `radius` picks 10% of
    /// the combined bounding box's largest extent. Organic: SDF-only, no
    /// exact companion.
    #[wasm_bindgen(js_name = smoothUnion)]
    pub fn smooth_union(&self, other: &WasmShape, radius: Option<f64>) -> WasmShape {
        WasmShape::sdf_only(self.inner.smooth_union(&other.inner, radius))
    }

    /// Edge-selective **fillet**: a rounded blend of `radius` applied only
    /// along the selected edge of the union of `self` and `other`. `edge` is
    /// a flat `[x0, y0, z0, x1, y1, z1, …]` polyline of the picked feature
    /// edge (the CSG-edge points the mesher recovers); other edges stay
    /// sharp. SDF-only, no exact companion.
    #[wasm_bindgen(js_name = filletEdge)]
    pub fn fillet_edge(&self, other: &WasmShape, radius: f64, edge: Vec<f64>) -> WasmShape {
        WasmShape::sdf_only(self.inner.blend_edge(
            &other.inner,
            opensolid_frep::BooleanKind::Union,
            opensolid_frep::BlendMode::Fillet,
            radius,
            polyline_region(&edge),
        ))
    }

    /// Edge-selective **chamfer**: a planar bevel of setback `radius` applied
    /// only along the selected edge of the union of `self` and `other`.
    /// `edge` is a flat `[x0, y0, z0, …]` polyline; other edges stay sharp.
    /// SDF-only, no exact companion.
    #[wasm_bindgen(js_name = chamferEdge)]
    pub fn chamfer_edge(&self, other: &WasmShape, radius: f64, edge: Vec<f64>) -> WasmShape {
        WasmShape::sdf_only(self.inner.blend_edge(
            &other.inner,
            opensolid_frep::BooleanKind::Union,
            opensolid_frep::BlendMode::Chamfer,
            radius,
            polyline_region(&edge),
        ))
    }

    /// Hollow the shape into a shell of total wall `thickness`, centered on
    /// the surface (extending `thickness / 2` to each side). Organic:
    /// SDF-only, no exact companion.
    ///
    /// # Errors
    /// `thickness` must be positive and finite.
    pub fn shell(&self, thickness: f64) -> Result<WasmShape, String> {
        self.inner
            .shell(thickness)
            .map(WasmShape::sdf_only)
            .map_err(|e| e.to_string())
    }

    /// Signed distance from `(x, y, z)` to the surface: negative inside,
    /// positive outside. After smooth blends or anisotropic scaling this is
    /// not an exact Euclidean distance, but the sign and zero set stay
    /// correct, so nearest-surface queries can compare magnitudes.
    pub fn distance(&self, x: f64, y: f64, z: f64) -> f64 {
        self.inner.distance(Point3::new(x, y, z))
    }

    /// Outward unit surface normal at `(x, y, z)` as `[nx, ny, nz]`, the
    /// normalized field gradient. "Sketch on a curved face" reads this at
    /// the picked hit point to build the tangent-plane sketch frame
    /// (origin = pick point, normal = this vector).
    #[wasm_bindgen(js_name = normalAt)]
    pub fn normal_at(&self, x: f64, y: f64, z: f64) -> Vec<f64> {
        let n = self.inner.surface_normal(Point3::new(x, y, z));
        vec![n.x, n.y, n.z]
    }

    /// Conservative axis-aligned bounding box of the surface as
    /// `[min_x, min_y, min_z, max_x, max_y, max_z]` (useful for camera
    /// framing).
    pub fn bounds(&self) -> Vec<f64> {
        let b = &self.inner.bounds;
        vec![b.min.x, b.min.y, b.min.z, b.max.x, b.max.y, b.max.z]
    }

    /// Enable or disable the exact B-Rep boolean path globally (the
    /// playground toggle). Off by default; flipping it re-routes booleans
    /// and meshing without rebuilding existing shapes.
    #[wasm_bindgen(js_name = setExactBooleans)]
    pub fn set_exact_booleans(enabled: bool) {
        exact::set_exact_enabled(enabled);
    }

    /// Whether `mesh()` will serve a validated exact B-Rep tessellation
    /// (this shape is an exact boolean result and the mode is on).
    #[wasm_bindgen(js_name = isExact)]
    pub fn is_exact(&self) -> bool {
        exact::exact_enabled()
            && self
                .exact
                .as_ref()
                .is_some_and(|rep| rep.exact_mesh().is_some())
    }

    /// Mesh the shape. Exact boolean results (see `isExact`) serve their
    /// validated analytic tessellation, which ignores `resolution` — it is
    /// already crisp at any zoom. Otherwise: dual-contouring on a
    /// `resolution`³ grid. With `bound` set, the grid covers the explicit
    /// cube `[-bound, bound]³` (the surface must lie strictly inside it);
    /// otherwise bounds are derived from the shape's tracked bounding box
    /// with padding.
    pub fn mesh(&self, resolution: u32, bound: Option<f64>) -> MeshData {
        let exact_mesh = if exact::exact_enabled() {
            self.exact.as_ref().and_then(|rep| rep.exact_mesh())
        } else {
            None
        };
        match exact_mesh {
            Some(mesh) => mesh_data(mesh),
            None => mesh_data(&self.inner.mesh(resolution as usize, bound)),
        }
    }

    /// Serialize the shape to STEP AP203 text (a complete Part 21 file).
    ///
    /// Shapes with an exact B-Rep representation — a supported primitive
    /// chain, or a boolean built with exact booleans on — export analytic
    /// surfaces. Everything else (smooth blends, rounded boxes, sweeps)
    /// exports a faceted-but-valid B-Rep recovered from the SDF at the
    /// given `accuracy` (target chordal deviation, same knob as
    /// `meshAdaptive`; omitted or invalid falls back to 0.5% of the
    /// shape's extent — the exact path ignores it).
    ///
    /// Throws a string error when the shape cannot produce a valid solid
    /// (e.g. an empty boolean result).
    ///
    /// `unit` is the document unit key (`"mm"`, `"cm"`, `"m"`, `"in"`); it
    /// sets the STEP length-unit declaration only — coordinates are written
    /// verbatim, never rescaled — and omitted or unknown keys default to
    /// millimetres.
    #[wasm_bindgen(js_name = exportStep)]
    pub fn export_step(
        &self,
        accuracy: Option<f64>,
        unit: Option<String>,
    ) -> Result<String, String> {
        step::export_step(&self.inner, self.exact.as_ref(), accuracy, unit.as_deref())
            .map(|e| e.text)
    }

    /// Mesh the shape adaptively to a target `accuracy`: the maximum
    /// chordal deviation of the mesh from the exact surface, in model
    /// units. The octree refines near curvature and CSG feature edges
    /// (kept crisp by QEF vertex placement) and stays coarse over flat
    /// regions, so triangle counts track surface complexity instead of a
    /// global grid resolution. With `bound` set, the grid covers the
    /// explicit cube `[-bound, bound]³` (the surface must lie strictly
    /// inside it); otherwise bounds are derived from the shape's tracked
    /// bounding box with padding. Accuracies far below 1/500th of the
    /// shape's extent are clamped by the cell budget and degrade
    /// gracefully.
    /// Exact boolean results (see `isExact`) serve their validated analytic
    /// tessellation, which ignores `accuracy` — it is already crisp.
    #[wasm_bindgen(js_name = meshAdaptive)]
    pub fn mesh_adaptive(&self, accuracy: f64, bound: Option<f64>) -> MeshData {
        let exact_mesh = if exact::exact_enabled() {
            self.exact.as_ref().and_then(|rep| rep.exact_mesh())
        } else {
            None
        };
        match exact_mesh {
            Some(mesh) => mesh_data(mesh),
            None => mesh_data(&self.inner.mesh_adaptive(accuracy, bound)),
        }
    }

    /// Silhouette (outline) edges of the shape for an orthographic view along
    /// `(vx, vy, vz)`: the mesh edges where the surface turns away from the
    /// view (adjacent faces flip front/back), the view-dependent companion to
    /// [`MeshData::feature_edges`]. Returned as a flat
    /// `[x0,y0,z0, x1,y1,z1, …]` segment buffer (two points per edge), the
    /// same convention as `feature_edges`/`filletEdge`.
    ///
    /// Computed against the same mesh measurement uses — the validated exact
    /// tessellation when the exact mode resolves one, otherwise an adaptive
    /// SDF mesh at `accuracy` (same knob as `meshAdaptive`; omitted or invalid
    /// falls back to 0.5% of the shape's extent). Silhouettes are
    /// view-dependent — recompute per view. `(vx, vy, vz)` need not be
    /// normalized.
    #[wasm_bindgen(js_name = silhouetteEdges)]
    pub fn silhouette_edges(&self, vx: f64, vy: f64, vz: f64, accuracy: Option<f64>) -> Vec<f32> {
        let view_dir = Vector3::new(vx, vy, vz);
        self.with_measure_mesh(accuracy, |mesh| {
            flatten_edges(&mesh.silhouette_edges(view_dir))
        })
    }

    /// Mass properties of the enclosed solid, as a JSON object string:
    /// `{ volume, surfaceArea, centroid:[x,y,z], inertia:[[…],[…],[…]],
    /// boundingBox:{min,max,size}, triangles, vertices, exact }`.
    ///
    /// Volume, centroid, and inertia (about the centroid, unit density) are
    /// exact polyhedral integrals over the measured mesh — the validated
    /// exact tessellation when [`isExact`](Self::is_exact), otherwise an
    /// adaptive SDF mesh at `accuracy` (same knob as `meshAdaptive`,
    /// defaulting to 0.5% of the shape's extent). When the mesh does not
    /// bound a finite non-zero volume those fields are `null` and
    /// `massError` explains why; the bounding box is always present.
    pub fn measure(&self, accuracy: Option<f64>) -> String {
        let bounds = &self.inner.bounds;
        let (min, max) = (bounds.min, bounds.max);
        let bbox = format!(
            "\"boundingBox\":{{\"min\":[{},{},{}],\"max\":[{},{},{}],\"size\":[{},{},{}]}}",
            json_num(min.x),
            json_num(min.y),
            json_num(min.z),
            json_num(max.x),
            json_num(max.y),
            json_num(max.z),
            json_num(max.x - min.x),
            json_num(max.y - min.y),
            json_num(max.z - min.z),
        );
        let exact = self.measured_exact();
        self.with_measure_mesh(accuracy, |mesh| {
            let counts = format!(
                "\"triangles\":{},\"vertices\":{},\"exact\":{}",
                mesh.triangle_count(),
                mesh.vertex_count(),
                exact,
            );
            match mass_properties(mesh) {
                Ok(mp) => {
                    let i = &mp.inertia;
                    format!(
                        "{{\"volume\":{},\"surfaceArea\":{},\"centroid\":[{},{},{}],\
                         \"inertia\":[[{},{},{}],[{},{},{}],[{},{},{}]],{},{}}}",
                        json_num(mp.volume),
                        json_num(mp.surface_area),
                        json_num(mp.centroid.x),
                        json_num(mp.centroid.y),
                        json_num(mp.centroid.z),
                        json_num(i[(0, 0)]),
                        json_num(i[(0, 1)]),
                        json_num(i[(0, 2)]),
                        json_num(i[(1, 0)]),
                        json_num(i[(1, 1)]),
                        json_num(i[(1, 2)]),
                        json_num(i[(2, 0)]),
                        json_num(i[(2, 1)]),
                        json_num(i[(2, 2)]),
                        bbox,
                        counts,
                    )
                }
                Err(e) => format!(
                    "{{\"volume\":null,\"surfaceArea\":null,\"centroid\":null,\
                     \"inertia\":null,{},{},\"massError\":\"{}\"}}",
                    bbox,
                    counts,
                    json_escape(&e.to_string()),
                ),
            }
        })
    }

    /// A structural check report for the shape's measured mesh, as a JSON
    /// object string: `{ valid, closedManifold, triangles, vertices, volume,
    /// exact, issues:[…] }`. `valid` is true exactly when the mesh is
    /// non-empty, a closed and consistently oriented 2-manifold, and encloses
    /// a finite non-zero volume; otherwise `issues` names each failure.
    pub fn validate(&self, accuracy: Option<f64>) -> String {
        let exact = self.measured_exact();
        self.with_measure_mesh(accuracy, |mesh| {
            let mut issues: Vec<String> = Vec::new();
            if mesh.triangle_count() == 0 {
                issues.push("mesh is empty".to_string());
            }
            let closed = mesh.is_closed_manifold();
            if !closed {
                issues.push("mesh is not a closed, consistently oriented manifold".to_string());
            }
            let volume = match mass_properties(mesh) {
                Ok(mp) => json_num(mp.volume),
                Err(e) => {
                    issues.push(e.to_string());
                    "null".to_string()
                }
            };
            let issues_json = issues
                .iter()
                .map(|s| format!("\"{}\"", json_escape(s)))
                .collect::<Vec<_>>()
                .join(",");
            format!(
                "{{\"valid\":{},\"closedManifold\":{},\"triangles\":{},\"vertices\":{},\
                 \"volume\":{},\"exact\":{},\"issues\":[{}]}}",
                issues.is_empty(),
                closed,
                mesh.triangle_count(),
                mesh.vertex_count(),
                volume,
                exact,
                issues_json,
            )
        })
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
        // Feature edges are two 3D points per segment: a multiple of 6 floats.
        assert_eq!(data.feature_edges.len() % 6, 0);
    }

    #[test]
    fn box_mesh_exports_feature_edges() {
        // A box's crease edges are its 12 sharp edges; the mesh path recovers
        // them (count varies with the tessellation, but there must be some,
        // and every segment is two finite points).
        let data = WasmShape::box3(1.0, 1.0, 1.0).mesh(24, None);
        assert_valid(&data);
        assert!(
            !data.feature_edges.is_empty(),
            "box should surface crease edges"
        );
        assert!(data.feature_edges.iter().all(|f| f.is_finite()));
    }

    #[test]
    fn sphere_has_no_sharp_feature_edges() {
        // A smooth sphere has no crease steeper than 40°: its facet seams are
        // all shallow, so the feature-edge buffer is empty.
        let data = WasmShape::sphere(1.0).mesh(32, None);
        assert_valid(&data);
        assert!(
            data.feature_edges.is_empty(),
            "sphere facet seams leaked as feature edges"
        );
    }

    #[test]
    fn box_silhouette_is_view_dependent() {
        let shape = WasmShape::box3(1.0, 1.0, 1.0);
        let front = shape.silhouette_edges(0.0, 0.0, 1.0, Some(0.02));
        let side = shape.silhouette_edges(1.0, 0.0, 0.0, Some(0.02));
        // Two 3D points per edge segment.
        assert_eq!(front.len() % 6, 0);
        assert!(!front.is_empty(), "box has an outline from +z");
        assert!(front.iter().all(|f| f.is_finite()));
        // A different view yields a different outline set.
        assert_ne!(front, side, "silhouette must depend on view direction");
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
    fn taper_via_wasm_api() {
        // Draft a box about the y = 0 plane pulling along +Y by 8°.
        let drafted = WasmShape::box3(1.0, 1.0, 1.0)
            .taper(0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 8.0)
            .expect("valid taper");
        assert_valid(&drafted.mesh(40, Some(1.6)));
        // At mid-height y = 0.5 the +Y wall has flared out to x = k(0.5),
        // so a point at x = 1 is now strictly inside; at y = −0.5 it pinches
        // in past x = 1, leaving that point outside.
        let tan8 = 8.0_f64.to_radians().tan();
        assert!(drafted.distance(1.0 + 0.5 * tan8, 0.5, 0.0).abs() < 1e-6);
        assert!(drafted.distance(1.0, 0.5, 0.0) < 0.0, "top not flared");
        assert!(drafted.distance(1.0, -0.5, 0.0) > 0.0, "bottom not pinched");
        // A draft at ±90° collapses the section and is rejected.
        assert!(
            WasmShape::box3(1.0, 1.0, 1.0)
                .taper(0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 90.0)
                .is_err()
        );
    }

    #[test]
    fn shell_via_wasm_api() {
        // Hollow a 2×2×2 cube to a 0.3-thick wall straddling the old surface.
        let t = 0.3;
        let hollow = WasmShape::box3(1.0, 1.0, 1.0)
            .shell(t)
            .expect("valid shell");
        assert_valid(&hollow.mesh(64, None));
        // The wall is centered on the original boundary at x = 1: the old
        // surface sits mid-wall, and each face of the wall is t/2 away.
        assert!((hollow.distance(1.0, 0.0, 0.0) + t / 2.0).abs() < 1e-6);
        assert!(hollow.distance(1.0 - t / 2.0, 0.0, 0.0).abs() < 1e-6);
        assert!(hollow.distance(1.0 + t / 2.0, 0.0, 0.0).abs() < 1e-6);
        // The deep interior is hollowed out — that is the whole point.
        assert!(hollow.distance(0.0, 0.0, 0.0) > 0.0, "interior not hollow");
        // The hollowed body still tessellates to a closed, oriented solid.
        let report = hollow.validate(Some(0.01));
        assert!(
            report.contains("\"closedManifold\":true") && report.contains("\"valid\":true"),
            "shell failed validation: {report}"
        );
        // A non-positive or non-finite wall is rejected, not silently clamped.
        assert!(WasmShape::box3(1.0, 1.0, 1.0).shell(0.0).is_err());
        assert!(WasmShape::box3(1.0, 1.0, 1.0).shell(-0.2).is_err());
        assert!(WasmShape::box3(1.0, 1.0, 1.0).shell(f64::NAN).is_err());
    }

    #[test]
    fn mesh_adaptive_via_wasm_api() {
        let body = WasmShape::rounded_box(1.0, 0.6, 0.8, 0.15);
        let bump = WasmShape::sphere(0.55).translate(0.0, 0.7, 0.0);
        let hole = WasmShape::cylinder(0.3, 2.0);
        let part = body.smooth_union(&bump, Some(0.25)).subtract(&hole);
        assert_valid(&part.mesh_adaptive(0.01, None));
        assert_valid(&part.mesh_adaptive(0.01, Some(2.5)));
        // Coarser accuracy must not cost more triangles.
        let fine = part.mesh_adaptive(0.005, None);
        let coarse = part.mesh_adaptive(0.05, None);
        assert!(coarse.indices.len() < fine.indices.len());
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

    /// Pull `"<key>":<number>` out of a JSON object string (no braces or
    /// nested arrays around the value). Returns `None` for `null`.
    fn json_field(json: &str, key: &str) -> Option<f64> {
        let needle = format!("\"{key}\":");
        let start = json.find(&needle)? + needle.len();
        let rest = &json[start..];
        let end = rest.find([',', '}', ']']).unwrap_or(rest.len());
        rest[..end].trim().parse().ok()
    }

    #[test]
    fn measure_reports_box_volume_and_centroid() {
        // Half-extents (1, 0.5, 0.75) → 2×1×1.5 box, volume 3, centroid at
        // the origin. Adaptive SDF meshing, exact booleans off (the default).
        let json = WasmShape::box3(1.0, 0.5, 0.75).measure(None);
        let volume = json_field(&json, "volume").expect("volume present");
        assert!(
            (volume - 3.0).abs() < 0.05,
            "volume {volume} ≉ 3.0 in {json}"
        );
        assert!(json.contains("\"boundingBox\""));
        assert!(json.contains("\"exact\":false"));
        // Translating the box shifts the centroid but not the volume.
        let moved = WasmShape::box3(1.0, 0.5, 0.75)
            .translate(4.0, 0.0, 0.0)
            .measure(None);
        assert!((json_field(&moved, "volume").unwrap() - 3.0).abs() < 0.05);
    }

    #[test]
    fn validate_accepts_solid_and_reports_boolean_result() {
        let sphere = WasmShape::sphere(1.0).validate(None);
        assert!(sphere.contains("\"valid\":true"), "{sphere}");
        assert!(sphere.contains("\"closedManifold\":true"));
        let volume = json_field(&sphere, "volume").expect("volume present");
        // Sphere volume 4/3·π ≈ 4.19; adaptive mesh is a slight under-estimate.
        assert!((volume - 4.18879).abs() < 0.1, "sphere volume {volume}");

        // A boolean difference still validates as a watertight solid.
        let part = WasmShape::box3(1.0, 1.0, 1.0)
            .subtract(&WasmShape::cylinder(0.4, 2.0))
            .validate(None);
        assert!(part.contains("\"valid\":true"), "{part}");
    }

    #[test]
    fn normal_at_via_wasm_api() {
        // Radial normal on the unit sphere, as [nx, ny, nz].
        let s = WasmShape::sphere(1.0);
        let n = s.normal_at(0.0, 0.0, 1.0);
        assert!((n[0]).abs() < 1e-4);
        assert!((n[1]).abs() < 1e-4);
        assert!((n[2] - 1.0).abs() < 1e-4);
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
        let shape = WasmShape::extrude(&closed_square(), 2.0, None).expect("valid extrude");
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
        let shape = WasmShape::extrude(&p, 0.5, None).expect("valid extrude");
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
    fn rib_via_wasm_api() {
        // Open path with an arc, thickened both sides then meshed.
        let mut path = WasmOpenPath2D::new(-0.6, 0.0);
        path.line_to(0.0, 0.0);
        path.arc_to(0.6, 0.0, 0.5);
        let shape = WasmShape::rib(&path, 0.2, 0.8, "both").expect("valid rib");
        let b = shape.bounds();
        // y spans [0, height]; x/z grown by the full thickness.
        assert!((b[1]).abs() < 1e-9 && (b[4] - 0.8).abs() < 1e-9);
        assert_valid(&shape.mesh(32, None));

        // Side is case-insensitive; first/second both build.
        let mut line = WasmOpenPath2D::new(-0.5, 0.0);
        line.line_to(0.5, 0.0);
        assert!(WasmShape::rib(&line, 0.2, 0.5, "First").is_ok());
        assert!(WasmShape::rib(&line, 0.2, 0.5, "SECOND").is_ok());
    }

    #[test]
    fn rib_errors_surface_as_strings() {
        let mut line = WasmOpenPath2D::new(0.0, 0.0);
        line.line_to(1.0, 0.0);
        // Unknown side name.
        let err = match WasmShape::rib(&line, 0.2, 0.5, "middle") {
            Ok(_) => panic!("unknown side must error"),
            Err(e) => e,
        };
        assert!(err.contains("middle"), "got: {err}");
        // Bad thickness / height propagate from Rib::new.
        assert!(WasmShape::rib(&line, 0.0, 0.5, "both").is_err());
        assert!(WasmShape::rib(&line, 0.2, -1.0, "both").is_err());
        // A single-vertex path is not a valid open path.
        let lone = WasmOpenPath2D::new(0.0, 0.0);
        assert!(WasmShape::rib(&lone, 0.2, 0.5, "both").is_err());
    }

    #[test]
    fn profile_errors_surface_as_strings() {
        // Unclosed profile.
        let mut open = WasmProfile2D::new(0.0, 0.0);
        open.line_to(1.0, 0.0);
        open.line_to(1.0, 1.0);
        let err = match WasmShape::extrude(&open, 1.0, None) {
            Ok(_) => panic!("must require close()"),
            Err(e) => e,
        };
        assert!(err.contains("close"), "got: {err}");

        // Too few segments.
        let mut tiny = WasmProfile2D::new(0.0, 0.0);
        tiny.close();
        assert!(WasmShape::extrude(&tiny, 1.0, None).is_err());

        // Bad height / angle / negative-x revolve profile.
        assert!(WasmShape::extrude(&closed_square(), 0.0, None).is_err());
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
        // Curved segments after close() are dropped too.
        p.ellipse_arc_to(-20.0, 20.0, 20.0, 20.0, 3.0, 3.0, 0.0, true);
        p.cubic_to(30.0, 30.0, 31.0, 31.0, 32.0, 32.0);
        let shape = WasmShape::extrude(&p, 1.0, None).expect("valid extrude");
        assert_eq!(shape.bounds(), vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0]);
    }

    fn closed_disk(r: f64) -> WasmProfile2D {
        let mut p = WasmProfile2D::new(-r, 0.0);
        p.arc_to(r, 0.0, 1.0);
        p.arc_to(-r, 0.0, 1.0);
        p.close();
        p
    }

    #[test]
    fn sweep_straight_and_bent_via_wasm_api() {
        // Straight sweep of a disk = a cylinder spanning y ∈ [0, 1].
        let mut path = WasmPath3D::new(0.0, 0.0, 0.0);
        path.line_to(0.0, 1.0, 0.0);
        let shape = WasmShape::sweep(&closed_disk(0.3), &path).expect("valid sweep");
        let b = shape.bounds();
        assert!(
            (b[1]).abs() < 1e-9 && (b[4] - 1.0).abs() < 1e-9,
            "y span: {b:?}"
        );
        assert_valid(&shape.mesh(40, None));

        // Bent path still meshes.
        let mut bent = WasmPath3D::new(0.0, 0.0, 0.0);
        bent.line_to(0.0, 0.8, 0.0);
        bent.line_to(0.8, 0.8, 0.0);
        let bent_shape = WasmShape::sweep(&closed_disk(0.2), &bent).expect("valid sweep");
        assert_valid(&bent_shape.mesh(48, None));
    }

    #[test]
    fn loft_via_wasm_api() {
        let shape = WasmShape::loft(&closed_disk(0.3), &closed_square(), 1.0).expect("valid loft");
        let b = shape.bounds();
        // y spans [0, 1]; xz spans the square (half-extent to 1 in +).
        assert!((b[1]).abs() < 1e-9 && (b[4] - 1.0).abs() < 1e-9);
        assert_valid(&shape.mesh(40, None));
    }

    #[test]
    fn sweep_and_loft_errors_surface_as_strings() {
        // Unclosed profile is rejected before path handling.
        let mut open = WasmProfile2D::new(0.0, 0.0);
        open.line_to(1.0, 0.0);
        open.line_to(1.0, 1.0);
        let single = WasmPath3D::new(0.0, 0.0, 0.0);
        assert!(WasmShape::sweep(&open, &single).is_err());
        // Single-point path (no segment) is rejected.
        assert!(WasmShape::sweep(&closed_disk(0.3), &single).is_err());
        // Bad loft height.
        assert!(WasmShape::loft(&closed_square(), &closed_square(), 0.0).is_err());
    }

    #[test]
    fn swept_path_shapes_compose_with_csg() {
        let mut path = WasmPath3D::new(0.0, 0.0, 0.0);
        path.line_to(0.0, 1.0, 0.0);
        let post = WasmShape::sweep(&closed_disk(0.4), &path).expect("valid sweep");
        let hole = WasmShape::cylinder(0.2, 2.0).translate(0.0, 0.5, 0.0);
        assert_valid(&post.subtract(&hole).mesh(40, None));
    }

    #[test]
    fn extrude_ellipse_profile_via_wasm_api() {
        // A full axis-aligned ellipse rx=1.5, ry=0.6 from two CCW half-arcs
        // (start (1.5,0) → (-1.5,0) over the top, then back), extruded and
        // meshed. Endpoints given directly (endpoint + ccw contract).
        let mut p = WasmProfile2D::new(1.5, 0.0);
        p.ellipse_arc_to(-1.5, 0.0, 0.0, 0.0, 1.5, 0.6, 0.0, true);
        p.ellipse_arc_to(1.5, 0.0, 0.0, 0.0, 1.5, 0.6, 0.0, true);
        p.close();
        let shape = WasmShape::extrude(&p, 0.8, None).expect("valid ellipse extrude");
        let b = shape.bounds();
        // Bounds span the ellipse axes: x ∈ [-1.5, 1.5], z ∈ [-0.6, 0.6],
        // y ∈ [0, 0.8]. The axis extremes are captured even though only the
        // two arc endpoints are explicit vertices.
        assert!(
            (b[0] + 1.5).abs() < 1e-6 && (b[3] - 1.5).abs() < 1e-6,
            "x: {b:?}"
        );
        assert!(
            (b[2] + 0.6).abs() < 1e-6 && (b[5] - 0.6).abs() < 1e-6,
            "z: {b:?}"
        );
        assert!((b[1]).abs() < 1e-9 && (b[4] - 0.8).abs() < 1e-9, "y: {b:?}");
        assert_valid(&shape.mesh(40, None));
    }

    #[test]
    fn ellipse_endpoint_ccw_recovers_circle_geometry() {
        // rx = ry = 1 makes the ellipse a unit circle; two CCW half-arcs
        // from (1,0) → (-1,0) → (1,0) must reproduce the exact circle SDF.
        // This pins the endpoint+ccw → eccentric-sweep conversion.
        let mut p = WasmProfile2D::new(1.0, 0.0);
        p.ellipse_arc_to(-1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, true);
        p.ellipse_arc_to(1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, true);
        p.close();
        // Extrude tall (height 4) and probe the mid-plane y = 2: the caps sit
        // 2 away, so the interior field is the 2D circle distance
        // hypot(x,z) − 1 wherever the wall is nearer than the caps.
        let shape = WasmShape::extrude(&p, 4.0, None).expect("valid circle extrude");
        for (x, z, want) in [
            (0.0, 0.0, -1.0),
            (0.5, 0.0, -0.5),
            (0.0, -0.9, -0.1),
            (1.5, 0.0, 0.5),
        ] {
            let d = shape.distance(x, 2.0, z);
            assert!((d - want).abs() < 1e-6, "at ({x},{z}): {d} vs {want}");
        }
        // Bounds are the unit circle box in x/z (axis extremes captured).
        let b = shape.bounds();
        assert!(
            (b[0] + 1.0).abs() < 1e-6 && (b[3] - 1.0).abs() < 1e-6,
            "{b:?}"
        );
        assert!(
            (b[2] + 1.0).abs() < 1e-6 && (b[5] - 1.0).abs() < 1e-6,
            "{b:?}"
        );
    }

    #[test]
    fn ellipse_cw_direction_takes_the_other_arc() {
        // Same endpoints (1,0) → (-1,0) but CW sweeps under the bottom, so
        // the enclosed region differs from the CCW (over-the-top) arc. Build
        // a half-disk each way and check the interior side flips.
        // CCW half from (1,0) to (-1,0) bulges through +y, closing straight
        // back along y = 0 encloses the upper half-disk (z > 0 in world).
        let mut up = WasmProfile2D::new(1.0, 0.0);
        up.ellipse_arc_to(-1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, true);
        up.close();
        let up_shape = WasmShape::extrude(&up, 1.0, None).expect("valid upper half");
        // CW half from (1,0) to (-1,0) bulges through -y → lower half-disk.
        let mut down = WasmProfile2D::new(1.0, 0.0);
        down.ellipse_arc_to(-1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, false);
        down.close();
        let down_shape = WasmShape::extrude(&down, 1.0, None).expect("valid lower half");
        // World z = +profile-y. Upper half contains (0, mid, +0.5); lower
        // half contains (0, mid, -0.5). Each excludes the other's point.
        assert!(up_shape.distance(0.0, 0.5, 0.5) < 0.0, "upper missing +z");
        assert!(up_shape.distance(0.0, 0.5, -0.5) > 0.0, "upper has -z");
        assert!(
            down_shape.distance(0.0, 0.5, -0.5) < 0.0,
            "lower missing -z"
        );
        assert!(down_shape.distance(0.0, 0.5, 0.5) > 0.0, "lower has +z");
    }

    #[test]
    fn extrude_cubic_profile_via_wasm_api() {
        // A teardrop: two cubic Béziers bowing out from a sharp corner at
        // the origin, extruded and meshed.
        let mut p = WasmProfile2D::new(0.0, 0.0);
        p.cubic_to(0.9, 0.2, 0.6, 0.9, 0.0, 1.0);
        p.cubic_to(-0.6, 0.9, -0.9, 0.2, 0.0, 0.0);
        p.close();
        let shape = WasmShape::extrude(&p, 0.7, None).expect("valid cubic extrude");
        assert_valid(&shape.mesh(44, None));
    }

    #[test]
    fn rib_with_cubic_path_via_wasm_api() {
        // Open path with a cubic wiggle, thickened into a rib and meshed.
        let mut path = WasmOpenPath2D::new(0.1, 0.1);
        path.cubic_to(0.4, 0.5, 0.6, -0.3, 0.9, 0.2);
        let shape = WasmShape::rib(&path, 0.15, 0.6, "both").expect("valid cubic rib");
        assert_valid(&shape.mesh(40, None));
    }

    #[test]
    fn curved_builder_errors_surface_as_strings() {
        // Ellipse with a zero radius is rejected at build time.
        let mut bad = WasmProfile2D::new(1.0, 0.0);
        bad.ellipse_arc_to(-1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, true);
        bad.ellipse_arc_to(1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, true);
        bad.close();
        assert!(WasmShape::extrude(&bad, 1.0, None).is_err());

        // A non-finite cubic control point is rejected.
        let mut nan = WasmProfile2D::new(0.0, 0.0);
        nan.cubic_to(f64::NAN, 0.5, 0.75, 0.5, 1.0, 0.0);
        nan.line_to(0.0, 0.0);
        nan.close();
        assert!(WasmShape::extrude(&nan, 1.0, None).is_err());
    }

    #[test]
    fn swept_shapes_compose_with_csg() {
        let plate = WasmShape::extrude(&closed_square(), 0.3, None).expect("valid extrude");
        let hole = WasmShape::cylinder(0.2, 1.0).translate(0.5, 0.15, 0.5);
        assert_valid(&plate.subtract(&hole).mesh(40, None));
    }

    /// Serialize tests that flip the global exact-boolean mode, and
    /// restore "off" when done (even on panic).
    fn exact_mode_on() -> impl Drop {
        use std::sync::{Mutex, MutexGuard, PoisonError};
        static LOCK: Mutex<()> = Mutex::new(());
        struct Guard(#[allow(dead_code)] MutexGuard<'static, ()>);
        impl Drop for Guard {
            fn drop(&mut self) {
                WasmShape::set_exact_booleans(false);
            }
        }
        let guard = LOCK.lock().unwrap_or_else(PoisonError::into_inner);
        WasmShape::set_exact_booleans(true);
        Guard(guard)
    }

    /// With the toggle on, a sharp boolean of exact primitives serves the
    /// kernel's analytic tessellation: far fewer vertices than any SDF
    /// grid, and unchanged by the resolution knob.
    #[test]
    fn exact_boolean_serves_analytic_mesh() {
        let _mode = exact_mode_on();
        let part = WasmShape::box3(1.0, 0.4, 1.0).subtract(&WasmShape::cylinder(0.4, 1.0));
        assert!(part.is_exact());

        let coarse = part.mesh(16, None);
        let fine = part.mesh(128, None);
        assert_valid(&coarse);
        assert_eq!(
            coarse.positions, fine.positions,
            "exact mesh must ignore the SDF resolution knob"
        );

        let sdf_verts = WasmShape::box3(1.0, 0.4, 1.0)
            .subtract(&WasmShape::cylinder(0.4, 1.0))
            .inner
            .mesh(128, None)
            .positions
            .len();
        assert!(
            coarse.positions.len() / 3 < sdf_verts,
            "analytic tessellation should be leaner than a 128-grid SDF mesh"
        );
    }

    /// The adaptive path serves the same analytic tessellation for exact
    /// boolean results: identical buffers at any accuracy, matching `mesh()`.
    #[test]
    fn exact_boolean_serves_analytic_mesh_adaptively() {
        let _mode = exact_mode_on();
        let part = WasmShape::box3(1.0, 0.4, 1.0).subtract(&WasmShape::cylinder(0.4, 1.0));
        assert!(part.is_exact());

        let coarse = part.mesh_adaptive(0.05, None);
        let fine = part.mesh_adaptive(0.002, None);
        assert_valid(&coarse);
        assert_eq!(
            coarse.positions, fine.positions,
            "exact mesh must ignore the accuracy knob"
        );
        assert_eq!(
            coarse.positions,
            part.mesh(16, None).positions,
            "adaptive and uniform paths must serve the same exact tessellation"
        );
    }

    /// Transformed primitives stay in exact reach; organic ops and shapes
    /// without exact support fall back to the SDF path.
    #[test]
    fn exact_coverage_boundaries() {
        let _mode = exact_mode_on();

        // Rigid transforms and uniform scale keep the spec exact: the
        // sphere bites a shallow cap out of the moved box's top face.
        let moved = WasmShape::box3(1.0, 1.0, 1.0)
            .rotate(0.0, 1.0, 0.0, 0.3)
            .uniform_scale(2.0)
            .expect("valid factor")
            .translate(3.0, 0.0, 0.0);
        let bitten = moved.subtract(&WasmShape::sphere(0.8).translate(3.0, 2.5, 0.0));
        assert!(bitten.is_exact());
        assert_valid(&bitten.mesh(24, None));

        // Anisotropic scale, organic blends, and unsupported primitives
        // drop to SDF-only — booleans still mesh, just not exactly.
        let squashed = WasmShape::box3(1.0, 1.0, 1.0)
            .scale(1.0, 0.5, 1.0)
            .expect("valid factors");
        assert!(!squashed.subtract(&WasmShape::sphere(0.8)).is_exact());
        let blended = WasmShape::box3(1.0, 1.0, 1.0).smooth_union(&WasmShape::sphere(0.8), None);
        assert!(!blended.is_exact());
        let rounded = WasmShape::rounded_box(1.0, 1.0, 1.0, 0.2);
        assert!(!rounded.union(&WasmShape::sphere(0.5)).is_exact());
        assert_valid(&rounded.union(&WasmShape::sphere(0.5)).mesh(24, None));
    }

    /// Flipping the toggle off reverts meshing to the SDF path without
    /// rebuilding shapes; primitives alone never claim exactness.
    #[test]
    fn exact_mode_toggle_reroutes_meshing() {
        // Hold the serialization guard for the WHOLE test: `is_exact()` reads
        // the process-global exact flag, so the off-mode assertion must run
        // while the lock is still held — otherwise a concurrent exact-mode
        // test flips the global back on between the guard drop and the check.
        let _mode = exact_mode_on();
        let part = WasmShape::box3(1.0, 1.0, 1.0)
            .subtract(&WasmShape::box3(0.5, 0.5, 0.5).translate(1.0, 1.0, 1.0));
        assert!(part.is_exact());
        assert!(!WasmShape::sphere(1.0).is_exact());
        let exact_mesh = part.mesh(64, None);
        let sdf_mesh_len = part.inner.mesh(64, None).positions.len() * 3;
        assert_ne!(exact_mesh.positions.len(), sdf_mesh_len);

        // Flip the mode off explicitly (still under the lock) and confirm the
        // meshing reverts to the SDF path; the guard's drop restores "off".
        WasmShape::set_exact_booleans(false);
        assert!(!part.is_exact(), "mode off: no exact claim");
        assert_valid(&part.mesh(24, None));
    }

    /// STEP export picks its path per shape: exact boolean results emit
    /// analytic surfaces, SDF-only shapes emit a faceted body, and both
    /// are complete Part 21 files.
    #[test]
    fn export_step_serves_exact_and_faceted_paths() {
        let _mode = exact_mode_on();
        let part = WasmShape::sphere(1.0).subtract(&WasmShape::cylinder(0.4, 2.0));
        assert!(part.is_exact());
        let text = part.export_step(None, None).expect("exact export");
        assert!(text.starts_with("ISO-10303-21;"));
        assert!(text.contains("SPHERICAL_SURFACE"), "analytic surfaces");
        // No unit key defaults to millimetres.
        assert!(text.contains("SI_UNIT(.MILLI.,.METRE.)"), "default mm unit");

        let organic = WasmShape::rounded_box(0.6, 0.4, 0.5, 0.1);
        let text = organic
            .export_step(Some(0.08), None)
            .expect("faceted export");
        assert!(text.starts_with("ISO-10303-21;"));
        assert!(
            !text.contains("SPHERICAL_SURFACE") && text.contains("PLANE"),
            "organic shapes export as faceted planar geometry"
        );
    }

    /// The document unit key flows into the STEP unit declaration without
    /// touching coordinates: an inch export declares a `CONVERSION_BASED_UNIT`
    /// and an unknown key falls back to millimetres.
    #[test]
    fn export_step_honours_the_document_unit() {
        let _mode = exact_mode_on();
        let part = WasmShape::sphere(1.0).subtract(&WasmShape::cylinder(0.4, 2.0));
        let inch = part
            .export_step(None, Some("in".to_string()))
            .expect("inch export");
        assert!(inch.contains("CONVERSION_BASED_UNIT('INCH'"), "inch unit");

        let bogus = part
            .export_step(None, Some("furlong".to_string()))
            .expect("unknown unit still exports");
        assert!(
            bogus.contains("SI_UNIT(.MILLI.,.METRE.)"),
            "unknown unit falls back to mm"
        );
    }

    /// With the toggle off (the default), booleans carry no exact
    /// companion at all — the mode is checked before any store is built.
    #[test]
    fn default_mode_stays_pure_sdf() {
        // Hold the mode lock (with the flag on, then force it off) so no
        // concurrently running exact test can flip it mid-assertion.
        let _mode = exact_mode_on();
        WasmShape::set_exact_booleans(false);
        let part = WasmShape::box3(1.0, 0.4, 1.0).subtract(&WasmShape::cylinder(0.4, 1.0));
        assert!(part.exact.is_none(), "toggle off: no exact pipeline work");
        assert!(!part.is_exact());
        assert_valid(&part.mesh(24, None));
    }
}
