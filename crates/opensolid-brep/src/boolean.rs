//! B-Rep boolean pipeline MVP: clash, imprint, classify, reconstruct
//! (`spec/04-booleans.md` at the spec/12 "minimum viable" level).
//!
//! [`unite`], [`subtract`], and [`intersect`] combine two transversal
//! solid bodies living in one [`TopologyStore`]/[`GeometryStore`] pair —
//! closed outward-oriented shells whose faces bind analytic surfaces
//! ([`Surface3`], planes and cylinders for now) and whose edges bind exact
//! [`Curve3`] geometry, exactly what [`crate::primitives`] and
//! [`crate::transform::translate_body`] produce — through the spec
//! pipeline:
//!
//! 1. **Clash**: dilated face bounding boxes (from sampled boundaries) are
//!    indexed in a [`Bvh`] per solid and candidate face pairs come from a
//!    simultaneous box-overlap descent of both trees
//!    ([`Bvh::overlap_pairs`], the of-pb7.1 index).
//! 2. **SSI**: each candidate pair is intersected analytically
//!    ([`crate::ssi::intersect`]); the resulting curves are clipped to the
//!    trimmed regions of *both* faces and become imprint curves.
//! 3. **Imprint**: imprint curves and original edges are split at their
//!    mutual meeting points *globally* (one canonical 3D split set per
//!    curve), so both sides of every future shared edge agree exactly.
//! 4. **Split & classify**: each face's parameter-space arrangement of
//!    boundary and imprint polylines is traced into regions; each region is
//!    classified inside/outside the other solid by ray casting from an
//!    interior sample point.
//! 5. **Reconstruct**: kept regions (per the operation's table) become the
//!    result's faces; shared atoms become manifold edges; shells are the
//!    connected components, with genus recovered from the Euler-Poincaré
//!    formula. The result carries a validated [`TopologyStore`]
//!    ([`BooleanOutput::check`]) and tessellates to a closed manifold mesh
//!    ([`BooleanOutput::tessellate`]).
//!
//! **Transversal only.** Coincident faces, tangent contacts (surfaces
//! touching without crossing, including single-point tangencies) are
//! rejected with a structured [`CoreError::NotImplemented`] — the
//! degeneracy ladder of spec/04 §10 is the hardening pass. Faces other
//! than planes and cylinders are likewise `NotImplemented` for now (the
//! parameter charts and ray intersections below extend naturally).
//!
//! The result body binds real geometry through its own stores
//! ([`BooleanOutput::store`] + [`BooleanOutput::geo`]): every face carries
//! the [`Surface3`] of its host input face (regions split from one face
//! share one surface id), and every edge carries its source [`Curve3`] —
//! an original input edge's curve or the exact SSI intersection curve —
//! with the parameter range recovered by closest-point projection
//! ([`CurveProject`]). Vertices sit on sampled polylines, so edge and
//! vertex tolerances record the actual curve-endpoint-to-vertex residual
//! (tolerant modeling, `spec/08-tolerances.md`).

use crate::check::CheckFailure;
use crate::curve::{Curve3, CurveEval, TWO_PI, plane_basis};
use crate::geometry::GeometryStore;
use crate::project::CurveProject;
use crate::ssi::{
    IntersectionKind, MarchedCurve, SurfaceIntersection, intersect as ssi_intersect,
    intersect_marched,
    marching::{pin_intersection_point, tighten_boundary_point},
};
use crate::surface::{Surface3, SurfaceEval};
use crate::topology::{
    Body, BodyType, FaceSense, FinSense, LoopType, SYSTEM_RESOLUTION, ShellOrientation,
    TopologyStore,
};
use opensolid_core::EntityId;
use opensolid_core::bvh::Bvh;
use opensolid_core::error::{CoreError, CoreResult};
use opensolid_core::mesh::{Triangle, TriangleMesh};
use opensolid_core::tolerance::ToleranceContext;
use opensolid_core::types::{BoundingBox3, Point3, Vector3};
use std::collections::HashMap;

/// Samples for a full circular edge or imprint curve.
const SAMPLES_PER_CIRCLE: usize = 96;
/// Samples for a straight edge.
const LINE_SAMPLES: usize = 8;
/// Samples for a straight imprint curve piece.
const IMPRINT_LINE_SAMPLES: usize = 64;
/// Bisection iterations when refining an imprint's exit point through a
/// face boundary.
const CLIP_REFINE_ITERATIONS: usize = 50;
/// Acceptance band for matching an imprint endpoint (or seam crossing) to
/// the original edge it splits, as a multiple of the feature-derived
/// [`geometric_snap`]. Endpoints are placed on the region-boundary
/// polyline by bisection, so genuine residuals measure below `1e-7 * snap`;
/// this band clears rounding noise while staying ~7 orders under the
/// inter-edge gap (O(feature size) ≈ `1e8 * snap`), so it never grabs the
/// wrong edge. Scaling off `snap` (not an absolute floor) is what keeps
/// this correct at any model scale (of-lxk).
const EDGE_MATCH_SNAP: f64 = 10.0;
/// Half-width of the "near a region boundary" band that rejects
/// ray-classification hits whose parity could flip under polyline
/// discretization, as a fraction of the local face extent
/// (`Pipeline::face_extents`). Sized to the polyline sagitta —
/// `r * (1 - cos(π / SAMPLES_PER_CIRCLE)) ≈ 5.4e-4 * r` — so a hit within a
/// chord's worth of a curved boundary is retried, while genuine interior
/// hits (nearest ≈ `3e-3 * face_extent`) are not. Keying off the face
/// extent (not `snap`, whose ULP floor inflates with distance from the
/// origin) keeps the band a fixed fraction of the feature at any scale and
/// position; an absolute band instead swallows whole small faces and
/// misses grazing hits on large ones (of-lxk, of-260).
const BOUNDARY_BAND_FRAC: f64 = 5e-4;
/// Fixed ray directions for point classification; each is retried in turn
/// when a cast grazes a surface or hits near a face boundary.
const RAY_DIRECTIONS: [[f64; 3]; 6] = [
    [0.7716, 0.3123, 0.5541],
    [-0.3661, 0.8151, 0.4489],
    [0.1741, -0.5389, 0.8242],
    [-0.6928, -0.4127, -0.5906],
    [0.9032, -0.1211, -0.4118],
    [-0.2113, 0.6934, -0.6892],
];

/// The boolean operation to perform.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BooleanOp {
    /// Material of A or B.
    Unite,
    /// Material of A and not B.
    Subtract,
    /// Material of A and B.
    Intersect,
}

/// A directed use of a solid's edge in a face loop.
#[derive(Debug, Clone, Copy)]
struct DirectedEdge {
    /// Index into [`AnalyticSolid`]'s edge list.
    edge: usize,
    /// Traversal direction relative to the edge's parameterization.
    forward: bool,
}

/// An exact edge of an [`AnalyticSolid`]: a [`Curve3`] restricted to
/// `[t0, t1]`. A closed edge (full circle) has `t1 - t0` equal to the
/// period and identical endpoints.
#[derive(Debug, Clone)]
struct SolidEdge {
    curve: Curve3,
    t0: f64,
    t1: f64,
    /// Whether the edge is a closed ring (start point = end point).
    closed: bool,
}

/// A face of an [`AnalyticSolid`]: an analytic surface trimmed by loops of
/// directed edges. `loops[0]` is the outer loop; loops wind
/// counterclockwise (outer) / clockwise (holes) as seen from the face's
/// outward side.
#[derive(Debug, Clone)]
struct AnalyticFace {
    surface: Surface3,
    /// Whether the face's outward normal equals the surface normal.
    outward_along_normal: bool,
    loops: Vec<Vec<DirectedEdge>>,
}

/// The pipeline's working view of one input body: a closed,
/// outward-oriented solid with exact analytic geometry, extracted from a
/// store-backed body by [`extract_solid`].
#[derive(Debug, Clone)]
struct AnalyticSolid {
    edges: Vec<SolidEdge>,
    faces: Vec<AnalyticFace>,
}

/// Flatten a store-backed body into the pipeline's [`AnalyticSolid`] view:
/// dereference every face's [`Surface3`] and every edge's [`Curve3`]
/// through `geo`, and turn loop fins into indexed directed edges.
///
/// `argument` names the body in error messages (`"a"` or `"b"`).
///
/// # Errors
/// [`CoreError::InvalidArgument`] if the body id is stale, the body is not
/// a [`BodyType::Solid`], or any face/edge lacks bound geometry.
fn extract_solid(
    store: &TopologyStore,
    geo: &GeometryStore,
    body: EntityId<Body>,
    argument: &'static str,
) -> CoreResult<AnalyticSolid> {
    let invalid = |reason: String| CoreError::InvalidArgument { argument, reason };
    let body_data = store
        .body(body)
        .ok_or_else(|| invalid(format!("stale body id {body:?}")))?;
    if body_data.body_type != BodyType::Solid {
        return Err(invalid(format!(
            "boolean inputs must be solid bodies, got {:?}",
            body_data.body_type
        )));
    }

    let mut edges: Vec<SolidEdge> = Vec::new();
    let mut edge_index: HashMap<EntityId<crate::topology::Edge>, usize> = HashMap::new();
    let mut faces = Vec::new();
    for &shell_id in store.shells_of_body(body) {
        let shell = store.shell(shell_id).expect("live shell");
        for &face_id in store.faces_of_shell(shell_id) {
            let face = store.face(face_id).expect("live face");
            let surface_id = face
                .surface
                .ok_or_else(|| invalid(format!("{face_id:?} has no bound surface")))?;
            let surface = geo
                .surface(surface_id)
                .ok_or_else(|| invalid(format!("{face_id:?} references a dead surface id")))?
                .clone();
            // Face normal = surface normal XOR sense; outward = face normal
            // XOR shell orientation.
            let outward_along_normal = (face.sense == FaceSense::Positive)
                == (shell.orientation == ShellOrientation::Outward);
            let mut loops = Vec::new();
            for loop_id in store.loops_of_face(face_id) {
                let mut directed = Vec::new();
                for &fin_id in store.fins_of_loop(loop_id) {
                    let fin = store.fin(fin_id).expect("live fin");
                    let edge_id = fin.edge;
                    let index = match edge_index.get(&edge_id) {
                        Some(&i) => i,
                        None => {
                            let edge = store.edge(edge_id).expect("live edge");
                            let curve_id = edge.curve.ok_or_else(|| {
                                invalid(format!("{edge_id:?} has no bound curve"))
                            })?;
                            let curve = geo
                                .curve(curve_id)
                                .ok_or_else(|| {
                                    invalid(format!("{edge_id:?} references a dead curve id"))
                                })?
                                .clone();
                            edges.push(SolidEdge {
                                curve,
                                t0: edge.t_start,
                                t1: edge.t_end,
                                closed: edge.start_vertex == edge.end_vertex,
                            });
                            edge_index.insert(edge_id, edges.len() - 1);
                            edges.len() - 1
                        }
                    };
                    directed.push(DirectedEdge {
                        edge: index,
                        forward: fin.sense == FinSense::Forward,
                    });
                }
                if directed.is_empty() {
                    return Err(invalid(format!(
                        "{face_id:?} has a degenerate loop; booleans need fin-bounded loops"
                    )));
                }
                loops.push(directed);
            }
            if loops.is_empty() {
                return Err(invalid(format!("{face_id:?} has no loops")));
            }
            faces.push(AnalyticFace {
                surface,
                outward_along_normal,
                loops,
            });
        }
    }
    Ok(AnalyticSolid { edges, faces })
}

/// The result of a boolean operation: reconstructed topology binding real
/// geometry through its own [`GeometryStore`], plus the per-face data
/// needed to tessellate.
pub struct BooleanOutput {
    /// The reconstructed topology graph. Every face binds a [`Surface3`]
    /// and every edge a [`Curve3`] in [`BooleanOutput::geo`].
    pub store: TopologyStore,
    /// The geometry referenced by [`BooleanOutput::store`].
    pub geo: GeometryStore,
    /// The result body inside [`BooleanOutput::store`].
    pub body: EntityId<Body>,
    /// Tessellation payload: one entry per kept face region.
    mesh_faces: Vec<MeshFace>,
    face_count: usize,
    shell_count: usize,
}

impl std::fmt::Debug for BooleanOutput {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("BooleanOutput")
            .field("body", &self.body)
            .field("face_count", &self.face_count)
            .field("shell_count", &self.shell_count)
            .finish_non_exhaustive()
    }
}

impl BooleanOutput {
    /// Number of faces in the result.
    pub fn face_count(&self) -> usize {
        self.face_count
    }

    /// Number of shells in the result.
    pub fn shell_count(&self) -> usize {
        self.shell_count
    }

    /// Validate the result body with the full checker
    /// ([`TopologyStore::check`]). Empty means valid.
    pub fn check(&self) -> Vec<CheckFailure> {
        self.store.check(self.body)
    }

    /// Tessellate the result into a triangle mesh (closed and manifold for
    /// valid results). Boundary polylines are shared exactly between
    /// adjacent faces, so the welded mesh is watertight. Same as
    /// [`BooleanOutput::tessellate_measured`] with the deviation discarded.
    pub fn tessellate(&self) -> CoreResult<TriangleMesh> {
        Ok(self.tessellate_measured()?.0)
    }

    /// Tessellate the result, also reporting the mesh's worst chordal
    /// deviation from the analytic face surfaces.
    ///
    /// Planar faces are triangulated exactly by ear clipping. Curved faces
    /// (cylinder bands) are ear-clipped for their boundary and then
    /// retriangulated to a constrained Delaunay mesh over a lattice of
    /// interior parameter-space points, so the flat 3D chords hug the surface
    /// instead of cutting long secants through it (see `refine_curved_region`).
    /// The returned deviation is the largest distance from any triangle edge's
    /// 3D midpoint to the surface point at its parameter-space midpoint — on
    /// the order of the boundary sampling — which the kernel's hybrid boolean
    /// still checks before trusting the exact mesh over the F-Rep fallback.
    pub fn tessellate_measured(&self) -> CoreResult<(TriangleMesh, f64)> {
        let mut triangles = Vec::new();
        // Weld epsilon from the mesh's feature extent (see `geometric_snap`
        // — must not grow with distance from the origin).
        let weld_eps = geometric_snap(
            self.mesh_faces
                .iter()
                .flat_map(|mf| mf.rings.iter())
                .flat_map(|ring| ring.points.iter().copied()),
        );
        let mut deviation: f64 = 0.0;
        for mf in &self.mesh_faces {
            let (tris, dev) = triangulate_mesh_face(mf, weld_eps)?;
            triangles.extend(tris);
            deviation = deviation.max(dev);
        }
        Ok((
            TriangleMesh::from_triangles(&triangles).weld(weld_eps),
            deviation,
        ))
    }
}

/// Material of A or B. Both bodies live in the shared `store`/`geo` pair
/// (e.g. built by [`crate::primitives`]); the result is returned in its
/// own stores ([`BooleanOutput`]).
///
/// # Errors
/// [`CoreError::InvalidArgument`] if either body is stale, not a solid, or
/// has faces/edges without bound geometry;
/// [`CoreError::NotImplemented`] for coincident or tangent face contacts
/// (transversal MVP) and for face surfaces without a parameter chart yet;
/// [`CoreError::Degenerate`] if a region cannot be classified robustly.
pub fn unite(
    store: &TopologyStore,
    geo: &GeometryStore,
    a: EntityId<Body>,
    b: EntityId<Body>,
    tol: &ToleranceContext,
) -> CoreResult<BooleanOutput> {
    boolean(BooleanOp::Unite, store, geo, a, b, tol)
}

/// Material of A and not B. See [`unite`] for errors.
pub fn subtract(
    store: &TopologyStore,
    geo: &GeometryStore,
    a: EntityId<Body>,
    b: EntityId<Body>,
    tol: &ToleranceContext,
) -> CoreResult<BooleanOutput> {
    boolean(BooleanOp::Subtract, store, geo, a, b, tol)
}

/// Material of A and B. See [`unite`] for errors.
pub fn intersect(
    store: &TopologyStore,
    geo: &GeometryStore,
    a: EntityId<Body>,
    b: EntityId<Body>,
    tol: &ToleranceContext,
) -> CoreResult<BooleanOutput> {
    boolean(BooleanOp::Intersect, store, geo, a, b, tol)
}

// ---------------------------------------------------------------------
// Parameter charts
// ---------------------------------------------------------------------

/// Invertible parameterization of the supported analytic surfaces.
#[derive(Debug, Clone)]
enum Chart {
    Plane {
        origin: Point3,
        e_u: Vector3,
        e_v: Vector3,
        normal: Vector3,
    },
    Cylinder {
        origin: Point3,
        axis: Vector3,
        e_u: Vector3,
        e_v: Vector3,
        radius: f64,
    },
    /// Latitude/longitude chart of a sphere: `u` is the longitude
    /// (radians, period 2π) about `axis`, `v` is the latitude (radians,
    /// clamped to `[-π/2, π/2]`, **not** periodic). The `u`-circle collapses
    /// at the two poles (`v = ±π/2`), where longitude is undefined — see
    /// [`Chart::param`] for the pole convention.
    Sphere {
        center: Point3,
        axis: Vector3,
        e_u: Vector3,
        e_v: Vector3,
        radius: f64,
    },
    /// Doubly-periodic chart of a torus: `u` is the major angle (radians,
    /// period 2π) about `axis`, `v` is the minor angle (radians, period 2π)
    /// around the tube. Both a `u`-seam meridian and a `v`-seam meridian
    /// wrap; neither degenerates.
    Torus {
        center: Point3,
        axis: Vector3,
        e_u: Vector3,
        e_v: Vector3,
        major_radius: f64,
        minor_radius: f64,
    },
}

impl Chart {
    /// Build the parameter chart for any analytic surface OpenSolid can
    /// invert. Spheres and tori are admitted (the of-7ld promotion);
    /// cones are rejected with [`CoreError::NotImplemented`] so they
    /// route through the F-Rep fallback.
    fn build(surface: &Surface3) -> CoreResult<Self> {
        match surface {
            Surface3::Plane { origin, normal } => {
                let (e_u, e_v) = plane_basis(normal);
                Ok(Chart::Plane {
                    origin: *origin,
                    e_u,
                    e_v,
                    normal: *normal,
                })
            }
            Surface3::Cylinder {
                origin,
                axis,
                radius,
            } => {
                let (e_u, e_v) = plane_basis(axis);
                Ok(Chart::Cylinder {
                    origin: *origin,
                    axis: *axis,
                    e_u,
                    e_v,
                    radius: *radius,
                })
            }
            Surface3::Sphere {
                center,
                axis,
                radius,
            } => {
                let (e_u, e_v) = plane_basis(axis);
                Ok(Chart::Sphere {
                    center: *center,
                    axis: *axis,
                    e_u,
                    e_v,
                    radius: *radius,
                })
            }
            Surface3::Torus {
                center,
                axis,
                major_radius,
                minor_radius,
            } => {
                let (e_u, e_v) = plane_basis(axis);
                Ok(Chart::Torus {
                    center: *center,
                    axis: *axis,
                    e_u,
                    e_v,
                    major_radius: *major_radius,
                    minor_radius: *minor_radius,
                })
            }
            Surface3::Cone { .. } => Err(CoreError::NotImplemented {
                feature: "boolean parameter chart for cones",
            }),
        }
    }

    /// Parameters of a point assumed to lie on the surface.
    ///
    /// Angles are unwrapped toward `hint` (shifted by whole periods to land
    /// within half a period of the hint's corresponding angle) so a sampled
    /// polyline stays continuous across a seam: `u` for cylinder, sphere,
    /// and torus charts, and additionally `v` for the torus.
    ///
    /// **Sphere pole convention.** Longitude is undefined at the poles
    /// (`v = ±π/2`), where the whole `u`-circle collapses to one point. A
    /// point within a hair of a pole (its distance from the axis below a
    /// relative floor) therefore *inherits* the longitude of `hint` — or
    /// `0` when there is no hint. An imprint threaded through a pole thus
    /// keeps a continuous longitude across the pole instead of snapping to
    /// an arbitrary `atan2` of near-zero components, which would otherwise
    /// spawn a zero-width UV wedge (a degenerate region) at the pole.
    fn param(&self, p: &Point3, hint: Option<(f64, f64)>) -> (f64, f64) {
        match self {
            Chart::Plane {
                origin, e_u, e_v, ..
            } => {
                let d = p - origin;
                (d.dot(e_u), d.dot(e_v))
            }
            Chart::Cylinder {
                origin,
                axis,
                e_u,
                e_v,
                ..
            } => {
                let d = p - origin;
                let u = unwrap_angle(d.dot(e_v).atan2(d.dot(e_u)), hint.map(|(u, _)| u));
                (u, d.dot(axis))
            }
            Chart::Sphere {
                center,
                axis,
                e_u,
                e_v,
                radius,
            } => {
                let d = p - center;
                let z = d.dot(axis);
                let v = (z / radius).clamp(-1.0, 1.0).asin();
                let (x, y) = (d.dot(e_u), d.dot(e_v));
                // Pole: the horizontal component vanishes and longitude is
                // undefined; keep the hint's u for a continuous crossing.
                let u = if x.hypot(y) <= radius * POLE_REL_EPS {
                    hint.map(|(u, _)| u).unwrap_or(0.0)
                } else {
                    unwrap_angle(y.atan2(x), hint.map(|(u, _)| u))
                };
                (u, v)
            }
            Chart::Torus {
                center,
                axis,
                e_u,
                e_v,
                major_radius,
                ..
            } => {
                let d = p - center;
                let z = d.dot(axis);
                let (x, y) = (d.dot(e_u), d.dot(e_v));
                let u = unwrap_angle(y.atan2(x), hint.map(|(u, _)| u));
                // Minor angle from the tube center: radial excess over the
                // major radius (cos side) against the axial height (sin side).
                let w = x.hypot(y) - major_radius;
                let v = unwrap_angle(z.atan2(w), hint.map(|(_, v)| v));
                (u, v)
            }
        }
    }

    /// Outward unit surface normal at parameters `(u, v)`. `v` is ignored
    /// for planes and cylinders but genuinely needed for the sphere and
    /// torus, whose normal tilts with latitude / minor angle.
    fn normal(&self, u: f64, v: f64) -> Vector3 {
        match self {
            Chart::Plane { normal, .. } => *normal,
            Chart::Cylinder { e_u, e_v, .. } => e_u * u.cos() + e_v * u.sin(),
            Chart::Sphere { e_u, e_v, axis, .. } | Chart::Torus { e_u, e_v, axis, .. } => {
                let radial = e_u * u.cos() + e_v * u.sin();
                radial * v.cos() + axis * v.sin()
            }
        }
    }

    /// Arc-length scale factors `(du_scale, dv_scale)` at latitude/minor
    /// angle `v`: multiplying a small parameter step by these yields the
    /// model-space displacement, putting both axes in one metric so uv
    /// distances mix units correctly (of-9n8).
    ///
    /// - Plane: `(1, 1)` — both axes already model units.
    /// - Cylinder: `(radius, 1)` — `u` is an angle, `v` a length.
    /// - Sphere: `(radius·cos v, radius)` — the longitude circle shrinks
    ///   toward the poles (scale → 0 there), latitude is uniform.
    /// - Torus: `(major + minor·cos v, minor)` — the major circle's radius
    ///   breathes with the minor angle; the minor circle is uniform.
    fn uv_scale(&self, v: f64) -> (f64, f64) {
        match self {
            Chart::Plane { .. } => (1.0, 1.0),
            Chart::Cylinder { radius, .. } => (*radius, 1.0),
            Chart::Sphere { radius, .. } => (radius * v.cos(), *radius),
            Chart::Torus {
                major_radius,
                minor_radius,
                ..
            } => (major_radius + minor_radius * v.cos(), *minor_radius),
        }
    }

    /// Period of the `u` axis (radians), or `None` when `u` is unbounded
    /// (planes). Every curved chart wraps `u` every 2π.
    fn period_u(&self) -> Option<f64> {
        match self {
            Chart::Plane { .. } => None,
            _ => Some(TWO_PI),
        }
    }

    /// Period of the `v` axis (radians), set only for the torus (whose
    /// minor angle wraps). Cylinder `v` is an unbounded length and sphere
    /// `v` is clamped latitude, so both are `None`.
    fn period_v(&self) -> Option<f64> {
        match self {
            Chart::Torus { .. } => Some(TWO_PI),
            _ => None,
        }
    }

    /// Latitude (`±π/2`) of the sphere pole that `p` sits on, or `None`
    /// when `p` is not on a pole (always `None` for non-sphere charts,
    /// whose parameterizations never collapse a point). Uses the same
    /// axis-distance floor as [`Chart::param`]'s pole convention, so a
    /// point reads as a pole here exactly when `param` would refuse to
    /// give it a longitude of its own.
    fn pole_v(&self, p: &Point3) -> Option<f64> {
        let Chart::Sphere {
            center,
            axis,
            e_u,
            e_v,
            radius,
        } = self
        else {
            return None;
        };
        let d = p - center;
        (d.dot(e_u).hypot(d.dot(e_v)) <= radius * POLE_REL_EPS)
            .then(|| std::f64::consts::FRAC_PI_2.copysign(d.dot(axis)))
    }

    /// The chart's pole points `[south, north]` (`v = -π/2, +π/2`), where
    /// the `u`-circle collapses to a point — only spheres have them.
    fn pole_points(&self) -> Option<[Point3; 2]> {
        let Chart::Sphere {
            center,
            axis,
            radius,
            ..
        } = self
        else {
            return None;
        };
        Some([center - axis * *radius, center + axis * *radius])
    }
}

/// Relative floor (fraction of the sphere radius) on a point's distance
/// from the polar axis below which its longitude is treated as undefined.
const POLE_REL_EPS: f64 = 1e-9;

/// Shift `angle` by whole turns so it lands within π of `hint` (no-op when
/// `hint` is `None`).
fn unwrap_angle(angle: f64, hint: Option<f64>) -> f64 {
    let Some(h) = hint else {
        return angle;
    };
    let mut a = angle;
    while a - h > std::f64::consts::PI {
        a -= TWO_PI;
    }
    while h - a > std::f64::consts::PI {
        a += TWO_PI;
    }
    a
}

// ---------------------------------------------------------------------
// Discretization
// ---------------------------------------------------------------------

/// A sampled edge or imprint curve: an ordered 3D polyline.
#[derive(Debug, Clone)]
struct SampledCurve {
    points: Vec<Point3>,
    closed: bool,
}

fn sample_edge(edge: &SolidEdge) -> SampledCurve {
    let n = match edge.curve {
        Curve3::Line { .. } => LINE_SAMPLES,
        _ => {
            let span = (edge.t1 - edge.t0).abs() / TWO_PI;
            ((SAMPLES_PER_CIRCLE as f64 * span).ceil() as usize).max(8)
        }
    };
    let count = if edge.closed { n } else { n + 1 };
    let points = (0..count)
        .map(|i| {
            let t = edge.t0 + (edge.t1 - edge.t0) * i as f64 / n as f64;
            edge.curve.point(t)
        })
        .collect();
    SampledCurve {
        points,
        closed: edge.closed,
    }
}

/// Per-face discretized boundary in parameter space, kept alongside its 3D
/// points; loop polylines close implicitly (last point connects to first).
#[derive(Debug, Clone)]
struct FaceRegionPoly {
    chart: Chart,
    /// One closed polyline per loop: (uv, 3D point).
    loops: Vec<Vec<((f64, f64), Point3)>>,
}

impl FaceRegionPoly {
    /// Even-odd point containment over all loops.
    fn contains(&self, uv: (f64, f64)) -> bool {
        let mut inside = false;
        for lp in &self.loops {
            let n = lp.len();
            for i in 0..n {
                let (a, _) = lp[i];
                let (b, _) = lp[(i + 1) % n];
                if crosses_upward(a, b, uv) {
                    inside = !inside;
                }
            }
        }
        inside
    }

    /// Containment for imprint clipping: like `contains` after
    /// `localize`, but robust on periodic charts. A sample that lands
    /// exactly on a parameter cover's seam meridian (`u = 0 / u = 2π`, and
    /// for a torus the `v` seam too) sits on the cover polygon's boundary,
    /// where the strict even-odd test is a float coin flip — yet such a
    /// point is geometrically interior to the face whenever its opposite
    /// coordinate is. Resolve by retrying with the on-seam coordinate
    /// nudged off the seam by a sub-tolerance step (`snap`, a model-space
    /// length, expressed as an angle via the local arc-length scale); a
    /// point genuinely outside the region stays outside under the nudge.
    fn contains_for_clip(&self, uv: (f64, f64), snap: f64) -> bool {
        let local = self.localize(uv);
        if self.contains(local) {
            return true;
        }
        let (u_scale, v_scale) = self.chart.uv_scale(local.1);
        if self.chart.period_u().is_some() {
            let eps = (snap / u_scale.max(1e-12)).max(1e-12);
            if self.contains(self.localize((local.0 + eps, local.1)))
                || self.contains(self.localize((local.0 - eps, local.1)))
            {
                return true;
            }
        }
        if self.chart.period_v().is_some() {
            let eps = (snap / v_scale.max(1e-12)).max(1e-12);
            if self.contains(self.localize((local.0, local.1 + eps)))
                || self.contains(self.localize((local.0, local.1 - eps)))
            {
                return true;
            }
        }
        false
    }

    /// Bring angle-like coordinates into this polygon's neighborhood by
    /// shifting whole periods on each periodic axis (no-op on axes that do
    /// not wrap, so planes are untouched and only the torus shifts `v`).
    fn localize(&self, uv: (f64, f64)) -> (f64, f64) {
        let period_u = self.chart.period_u();
        let period_v = self.chart.period_v();
        if period_u.is_none() && period_v.is_none() {
            return uv;
        }
        let (mut lo_u, mut hi_u) = (f64::INFINITY, f64::NEG_INFINITY);
        let (mut lo_v, mut hi_v) = (f64::INFINITY, f64::NEG_INFINITY);
        for lp in &self.loops {
            for ((u, v), _) in lp {
                lo_u = lo_u.min(*u);
                hi_u = hi_u.max(*u);
                lo_v = lo_v.min(*v);
                hi_v = hi_v.max(*v);
            }
        }
        let u = match period_u {
            Some(p) => shift_into_window(uv.0, 0.5 * (lo_u + hi_u), p),
            None => uv.0,
        };
        let v = match period_v {
            Some(p) => shift_into_window(uv.1, 0.5 * (lo_v + hi_v), p),
            None => uv.1,
        };
        (u, v)
    }
}

/// Shift `x` by whole periods so it lands within half a period of `center`.
fn shift_into_window(x: f64, center: f64, period: f64) -> f64 {
    let mut x = x;
    while x - center > 0.5 * period {
        x -= period;
    }
    while center - x > 0.5 * period {
        x += period;
    }
    x
}

/// Does segment `a -> b` cross the upward ray from `p` (even-odd rule)?
fn crosses_upward(a: (f64, f64), b: (f64, f64), p: (f64, f64)) -> bool {
    let (ax, ay) = a;
    let (bx, by) = b;
    let (px, py) = p;
    if (ay > py) == (by > py) {
        return false;
    }
    let x_at = ax + (py - ay) / (by - ay) * (bx - ax);
    x_at > px
}

/// Map a 3D polyline into a face's parameter space with angle unwrapping.
fn map_polyline(chart: &Chart, points: &[Point3]) -> Vec<(f64, f64)> {
    let mut out: Vec<(f64, f64)> = Vec::with_capacity(points.len());
    for p in points {
        let hint = out.last().copied();
        out.push(chart.param(p, hint));
    }
    out
}

/// Angular slack under which a walk's departure meridian from a sphere
/// pole reads as the arrival meridian retraced (a doubling-back, as at a
/// seam-edge tip), rather than a genuinely distinct meridian. Well above
/// the longitude noise of edge/imprint samples one sample step away from a
/// pole, and far below any real inter-meridian angle the SSI repertoire
/// produces.
const POLE_TURN_EPS: f64 = 1e-6;

/// Incremental embedding of a 3D walk into a chart's parameter cover:
/// angle unwrapping toward the previous point, plus explicit handling of
/// sphere poles, where the whole `u`-circle collapses to one point and the
/// cover polygon needs a **pole closure edge** — zero length in 3D, up to
/// a full period wide in `uv` (of-7ld.5).
///
/// A walk that touches a pole gets two cover points there: one at the
/// arrival longitude `u_in` (inherited from the previous point, per
/// [`Chart::param`]'s pole convention) and one at the departure longitude
/// `u_out` of the next point's meridian. The horizontal segment between
/// them is the pole row of the cover; it never affects even-odd
/// containment (rays are cast in `+v`) but restores the shoelace area
/// that a collapsed cover loses. Without it, a sphere face bounded only
/// by its seam meridian embeds as two coincident vertical traversals —
/// a zero-area polygon in which no interior sample point exists.
///
/// `u_out` is the representative of the departure meridian's longitude
/// chosen so the pole row sweeps exactly the face-interior meridians. For
/// a cycle wound CCW in the chart the interior lies left of the walk, so
/// the row runs toward `-u` at the north pole and toward `+u` at the
/// south pole (mirrored when `ccw` is `false`); a departure meridian
/// within [`POLE_TURN_EPS`] of the arrival meridian is a doubling-back
/// (e.g. the seam tip of a full sphere) and sweeps the full period.
struct CoverEmbedder<'c> {
    chart: &'c Chart,
    /// Intended chart winding of the walk's cycles. `reconstruct` traces
    /// every cycle CCW-in-chart; stored face loops are CCW only when the
    /// face's outward side follows the surface normal.
    ccw: bool,
    last_uv: Option<(f64, f64)>,
    /// Set while the walk stands on a pole: (pole latitude, pole point).
    at_pole: Option<(f64, Point3)>,
}

impl<'c> CoverEmbedder<'c> {
    fn new(chart: &'c Chart, ccw: bool) -> Self {
        CoverEmbedder {
            chart,
            ccw,
            last_uv: None,
            at_pole: None,
        }
    }

    /// Embed the walk's next point, appending one cover point — or two
    /// when this point leaves a pole (the departure end of the pole row,
    /// carrying the pole's 3D point, precedes it).
    fn push(&mut self, p: Point3, out: &mut Vec<CoverPoint>) {
        if let Some(vp) = self.chart.pole_v(&p) {
            let u = self.last_uv.map_or(0.0, |(u, _)| u);
            out.push(((u, vp), p));
            self.last_uv = Some((u, vp));
            self.at_pole = Some((vp, p));
            return;
        }
        let uv = if let Some((vp, pole_pt)) = self.at_pole.take() {
            let (u_in, _) = self.last_uv.expect("standing on a pole implies a uv");
            let (raw_u, v) = self.chart.param(&p, None);
            // Interior meridians sweep from the departure to the arrival
            // longitude going the row's way; a CCW cycle keeps them left
            // of the walk (north row toward -u, south row toward +u).
            let toward_neg_u = self.ccw == (vp > 0.0);
            let turn = if toward_neg_u {
                (u_in - raw_u).rem_euclid(TWO_PI)
            } else {
                (raw_u - u_in).rem_euclid(TWO_PI)
            };
            let turn = if turn < POLE_TURN_EPS || TWO_PI - turn < POLE_TURN_EPS {
                TWO_PI
            } else {
                turn
            };
            let u_out = if toward_neg_u {
                u_in - turn
            } else {
                u_in + turn
            };
            out.push(((u_out, vp), pole_pt));
            (u_out, v)
        } else {
            self.chart.param(&p, self.last_uv)
        };
        out.push((uv, p));
        self.last_uv = Some(uv);
    }

    /// Shift the embedder's continuity hint after the caller relocated the
    /// emitted cover points by whole periods.
    fn shift(&mut self, du: f64, dv: f64) {
        if let Some((u, v)) = &mut self.last_uv {
            *u += du;
            *v += dv;
        }
    }
}

// ---------------------------------------------------------------------
// Pipeline data
// ---------------------------------------------------------------------

/// Which solid an entity belongs to.
type SolidTag = usize; // 0 = A, 1 = B

/// A curve in the global split network: an original edge of one solid or
/// an imprint shared by one face of each solid.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
enum CurveSource {
    Edge { solid: SolidTag, edge: usize },
    Imprint { index: usize },
}

/// An imprint: one transversal intersection curve piece clipped to both
/// host face regions.
#[derive(Debug)]
struct Imprint {
    face_a: usize,
    face_b: usize,
    /// The exact SSI curve the samples came from (bound to result edges).
    curve: Curve3,
    sampled: SampledCurve,
}

/// A maximal un-split piece of a source curve: the atomic arrangement edge
/// and future topology edge (its provenance lives in the pipeline's
/// by-source index).
#[derive(Debug, Clone)]
struct Atom {
    points: Vec<Point3>,
    /// Ring atom: closed polyline (points[0] is the seam; the polyline does
    /// not repeat it at the end).
    closed: bool,
}

/// Snap length for a point cloud: 1e-9 of the cloud's bounding-box
/// extent — the feature size — floored at ~100 ULPs of the largest
/// coordinate magnitude.
///
/// The snap must key off feature size, not point magnitude `|p|`: a part
/// has to weld and classify the same at (1e6, 0, 0) as at the origin, and
/// a magnitude-derived snap inflates every tolerance band with distance
/// from the origin until interior probes read as on-surface and
/// classification fails (of-260). The ULP floor exists because distances
/// below the f64 spacing of the coordinates themselves cannot be
/// resolved, so a tighter snap would fail to merge points that differ
/// only by rounding noise.
fn geometric_snap<I: IntoIterator<Item = Point3>>(points: I) -> f64 {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for p in points {
        for k in 0..3 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    let mut extent: f64 = 0.0;
    let mut magnitude: f64 = 0.0;
    for k in 0..3 {
        if hi[k] >= lo[k] {
            extent = extent.max(hi[k] - lo[k]);
            magnitude = magnitude.max(lo[k].abs()).max(hi[k].abs());
        }
    }
    (1e-9 * extent)
        .max(100.0 * f64::EPSILON * magnitude)
        .max(f64::MIN_POSITIVE)
}

fn quantize(p: &Point3, snap: f64) -> (i64, i64, i64) {
    (
        (p.x / snap).round() as i64,
        (p.y / snap).round() as i64,
        (p.z / snap).round() as i64,
    )
}

/// Spatial hash merging points within `tol` of each other. Lookups probe
/// the 3×3×3 cell neighborhood with a true distance test, so coincident
/// points that straddle a quantization cell boundary still match — a
/// single-cell exact-key lookup would split them into distinct entries.
struct SnapMap<V> {
    cells: HashMap<(i64, i64, i64), Vec<(Point3, V)>>,
    tol: f64,
    /// Cell size is `2 * tol`: two points within `tol` then land at most
    /// one cell index apart on every axis (round(a) and round(b) can only
    /// differ by ≥ 2 when |a − b| > 1/2 in cell units), so the ±1 probe
    /// in [`Self::matches`] is exhaustive.
    cell: f64,
}

impl<V: Copy> SnapMap<V> {
    fn new(tol: f64) -> Self {
        SnapMap {
            cells: HashMap::new(),
            tol,
            cell: tol * 2.0,
        }
    }

    fn insert(&mut self, p: Point3, v: V) {
        self.cells
            .entry(quantize(&p, self.cell))
            .or_default()
            .push((p, v));
    }

    /// All values stored within `tol` of `p`, nearest first.
    fn matches(&self, p: &Point3) -> Vec<V> {
        let (i, j, k) = quantize(p, self.cell);
        let mut found: Vec<(f64, V)> = Vec::new();
        for di in -1..=1 {
            for dj in -1..=1 {
                for dk in -1..=1 {
                    let Some(entries) = self.cells.get(&(i + di, j + dj, k + dk)) else {
                        continue;
                    };
                    for &(q, v) in entries {
                        let d = (q - *p).norm();
                        if d <= self.tol {
                            found.push((d, v));
                        }
                    }
                }
            }
        }
        found.sort_by(|a, b| a.0.total_cmp(&b.0));
        found.into_iter().map(|(_, v)| v).collect()
    }

    /// The stored value nearest to `p` within `tol`, if any.
    fn nearest(&self, p: &Point3) -> Option<V> {
        self.matches(p).into_iter().next()
    }
}

// ---------------------------------------------------------------------
// The pipeline
// ---------------------------------------------------------------------

struct Pipeline<'a> {
    solids: [&'a AnalyticSolid; 2],
    tol: ToleranceContext,
    snap: f64,
    /// Sampled original edges, per solid.
    edge_samples: [Vec<SampledCurve>; 2],
    /// Discretized face regions, per solid.
    face_polys: [Vec<FaceRegionPoly>; 2],
    /// Local feature length of each face (boundary bounding-box diagonal),
    /// per solid. Magnitude-independent, so classification bands stay a
    /// fixed fraction of the face at any position and scale (of-lxk).
    face_extents: [Vec<f64>; 2],
    imprints: Vec<Imprint>,
    /// Split points per curve source, as 3D points.
    splits: HashMap<CurveSource, Vec<Point3>>,
    /// Seam-crossing points per (solid, face): where closed imprints cross
    /// that face's seam. On the host face these are boundary junctions, so
    /// `merge_imprint_chains` must terminate chains there instead of
    /// re-merging the seam-split chords of a winding-0 ring back into a
    /// full ring whose uv embedding would straddle the cover edge (of-43n).
    seam_barriers: HashMap<(SolidTag, usize), Vec<Point3>>,
}

/// Broad-phase bounding box for one face.
///
/// Boundary samples are useless for a closed face: a full sphere's only
/// boundary is the seam meridian (a half-circle in one plane), so its
/// sample box is flat along the seam-plane normal and misses shallow
/// clashes entirely (of-7ld.6). Bounded surfaces (sphere, torus) instead
/// use their exact surface box — a safe overestimate for partial faces,
/// since the broad phase only feeds candidates to SSI. `contact` dilates
/// the box so touching contacts still clash and reach SSI (which rejects
/// them as tangent, not silently misses them).
fn broad_phase_face_box(
    surface: &Surface3,
    boundary: impl Iterator<Item = Point3>,
    contact: f64,
) -> BoundingBox3 {
    match surface.bounding_box() {
        Some(exact) => exact.dilate(contact),
        None => {
            let bounds = BoundingBox3::from_points(boundary);
            // Boundary samples underestimate curved interiors; dilate by a
            // fraction of the face extent to cover the sagitta.
            bounds.dilate(bounds.extents().norm() * 0.05 + contact)
        }
    }
}

/// Signed implicit residual of `p` against a primitive's locus: zero on
/// the surface, smooth nearby. Used to polish marched clip endpoints onto
/// exact face-boundary junctions. Cones are outside the marched MVP.
fn surface_residual(s: &Surface3, p: &Point3) -> Option<f64> {
    match *s {
        Surface3::Plane { origin, normal } => Some(normal.dot(&(p - origin))),
        Surface3::Sphere { center, radius, .. } => Some((p - center).norm() - radius),
        Surface3::Cylinder {
            origin,
            axis,
            radius,
        } => {
            let d = p - origin;
            Some((d - axis * axis.dot(&d)).norm() - radius)
        }
        Surface3::Torus {
            center,
            axis,
            major_radius,
            minor_radius,
        } => {
            let d = p - center;
            let h = axis.dot(&d);
            let rho = (d - axis * h).norm();
            Some((rho - major_radius).hypot(h) - minor_radius)
        }
        Surface3::Cone { .. } => None,
    }
}

/// Spatial gradient of [`surface_residual`] at `p` (the unit normal of the
/// residual's level set). `None` on the residual's singular sets (axis,
/// center, tube centerline), where the polish gives up.
fn surface_residual_gradient(s: &Surface3, p: &Point3) -> Option<Vector3> {
    let unit = |v: Vector3| {
        let n = v.norm();
        (n > f64::MIN_POSITIVE).then(|| v / n)
    };
    match *s {
        Surface3::Plane { normal, .. } => Some(normal),
        Surface3::Sphere { center, .. } => unit(p - center),
        Surface3::Cylinder { origin, axis, .. } => {
            let d = p - origin;
            unit(d - axis * axis.dot(&d))
        }
        Surface3::Torus {
            center,
            axis,
            major_radius,
            ..
        } => {
            let d = p - center;
            let h = axis.dot(&d);
            let radial = d - axis * h;
            let rho = radial.norm();
            if rho <= f64::MIN_POSITIVE {
                return None;
            }
            unit((radial / rho) * (rho - major_radius) + axis * h)
        }
        Surface3::Cone { .. } => None,
    }
}

/// Whether [`intersect_marched`] covers this surface pair (see its docs):
/// the analytic pairs whose general configurations have no closed form.
/// Mirrors its dispatch so the pipeline can keep the analytic error —
/// which names the actual unsupported configuration — for every other
/// pair.
fn marched_ssi_supported(a: &Surface3, b: &Surface3) -> bool {
    use Surface3::*;
    matches!(
        (a, b),
        (Sphere { .. }, Cylinder { .. })
            | (Cylinder { .. }, Sphere { .. })
            | (
                Torus { .. },
                Plane { .. } | Cylinder { .. } | Sphere { .. } | Torus { .. }
            )
            | (Plane { .. } | Cylinder { .. } | Sphere { .. }, Torus { .. })
    )
}

/// Boolean pipeline entry point.
fn boolean(
    op: BooleanOp,
    store: &TopologyStore,
    geo: &GeometryStore,
    a: EntityId<Body>,
    b: EntityId<Body>,
    tol: &ToleranceContext,
) -> CoreResult<BooleanOutput> {
    let solid_a = extract_solid(store, geo, a, "a")?;
    let solid_b = extract_solid(store, geo, b, "b")?;
    let mut pipe = Pipeline::new(&solid_a, &solid_b, tol)?;
    pipe.find_imprints()?;
    pipe.collect_splits();
    let (atoms, atoms_by_source) = pipe.build_atoms();
    pipe.reconstruct(op, atoms, atoms_by_source)
}

impl<'a> Pipeline<'a> {
    fn new(a: &'a AnalyticSolid, b: &'a AnalyticSolid, tol: &ToleranceContext) -> CoreResult<Self> {
        let solids = [a, b];
        let edge_samples = [
            a.edges.iter().map(sample_edge).collect::<Vec<_>>(),
            b.edges.iter().map(sample_edge).collect::<Vec<_>>(),
        ];
        // Snap length from the joint feature extent of both solids (see
        // `geometric_snap` — must not grow with distance from the origin).
        let snap = geometric_snap(
            edge_samples
                .iter()
                .flatten()
                .flat_map(|s| s.points.iter().copied()),
        );

        let mut face_polys: [Vec<FaceRegionPoly>; 2] = [Vec::new(), Vec::new()];
        for s in 0..2 {
            for face in &solids[s].faces {
                let chart = Chart::build(&face.surface)?;
                let mut loops = Vec::new();
                for lp in &face.loops {
                    let mut pts3: Vec<Point3> = Vec::new();
                    for de in lp {
                        let sampled = &edge_samples[s][de.edge];
                        append_directed(&mut pts3, sampled, de.forward);
                    }
                    // Stored loops wind CCW as seen from the face's
                    // outward side, which is CCW in the chart only when
                    // that side follows the surface normal; the embedder
                    // needs the true winding to orient pole closure rows.
                    let mut emb = CoverEmbedder::new(&chart, face.outward_along_normal);
                    let mut cover: Vec<CoverPoint> = Vec::with_capacity(pts3.len());
                    for p in pts3 {
                        emb.push(p, &mut cover);
                    }
                    loops.push(cover);
                }
                face_polys[s].push(FaceRegionPoly { chart, loops });
            }
        }
        let face_extents: [Vec<f64>; 2] = [0, 1].map(|s| {
            face_polys[s]
                .iter()
                .map(|fp| {
                    let bb = BoundingBox3::from_points(fp.loops.iter().flatten().map(|&(_, p)| p));
                    bb.extents().norm()
                })
                .collect()
        });
        Ok(Self {
            solids,
            tol: *tol,
            snap,
            edge_samples,
            face_polys,
            face_extents,
            imprints: Vec::new(),
            splits: HashMap::new(),
            seam_barriers: HashMap::new(),
        })
    }

    /// Phases 1-2: clash detection and SSI, producing clipped imprints.
    fn find_imprints(&mut self) -> CoreResult<()> {
        let boxes: [Vec<BoundingBox3>; 2] = [self.face_boxes(0), self.face_boxes(1)];
        // Broad phase: one BVH per solid, candidate pairs from a dual-tree
        // box-overlap descent. Sorted so imprints are found in the same
        // (face A, face B) order the old pairwise scan produced.
        let bvhs: [Bvh<usize>; 2] =
            [0, 1].map(|s| Bvh::build(boxes[s].iter().enumerate().map(|(f, &b)| (b, f)).collect()));
        let mut pairs: Vec<(usize, usize)> = bvhs[0]
            .overlap_pairs(&bvhs[1])
            .into_iter()
            .map(|(&fa, &fb)| (fa, fb))
            .collect();
        pairs.sort_unstable();
        for (fa, fb) in pairs {
            let (box_a, box_b) = (&boxes[0][fa], &boxes[1][fb]);
            let sa = &self.solids[0].faces[fa].surface;
            let sb = &self.solids[1].faces[fb].surface;
            match ssi_intersect(sa, sb, &self.tol) {
                Ok(SurfaceIntersection::Empty) => {}
                Ok(SurfaceIntersection::Coincident) => {
                    return Err(CoreError::NotImplemented {
                        feature: "boolean operations on coincident faces \
                                  (transversal MVP)",
                    });
                }
                Ok(SurfaceIntersection::TangentPoint(_)) => {
                    return Err(CoreError::NotImplemented {
                        feature: "boolean operations with tangent face contact \
                                  (transversal MVP)",
                    });
                }
                Ok(SurfaceIntersection::Curves(curves)) => {
                    for ic in curves {
                        if ic.kind == IntersectionKind::Tangential {
                            return Err(CoreError::NotImplemented {
                                feature: "boolean operations with tangential \
                                          intersection curves (transversal MVP)",
                            });
                        }
                        self.clip_imprint(&ic.curve, fa, fb, box_a, box_b);
                    }
                }
                // Analytic SSI classifies the special configurations of the
                // sphere/torus pairs exactly and reports the general
                // positions (oblique plane-torus quartics, non-coaxial
                // torus-torus, cylinder-sphere, ...) as NotImplemented;
                // those are marched instead (of-yet). Pairs the marcher
                // does not cover keep the analytic error.
                Err(CoreError::NotImplemented { .. }) if marched_ssi_supported(sa, sb) => {
                    for mc in intersect_marched(sa, sb, &self.tol)? {
                        for curve in self.marched_polylines(mc, sa, sb) {
                            self.clip_imprint(&curve, fa, fb, box_a, box_b);
                        }
                    }
                }
                Err(err) => return Err(err),
            }
        }
        Ok(())
    }

    /// Wrap a marched intersection curve as [`Curve3::Polyline`]s the
    /// arrangement can host:
    ///
    /// - Open fragments get their endpoints re-polished onto the exact
    ///   intersection ([`tighten_boundary_point`]): the tracer stops at its
    ///   own looser gap tolerance, and the fragments meeting at one seam
    ///   crossing must agree on it far below the pipeline's welding snap
    ///   for their chains and vertices to rejoin.
    /// - Closed curves are cut at any chart-seam crossings they carry. The
    ///   tracer's hard domain bounds cut *most* wrapping curves into open
    ///   fragments, but a loop seeded near a seam can close (the closure
    ///   check fires before the boundary pin) and then wraps a periodic
    ///   parameter; hosted as-is it would cross the face cover mid-atom
    ///   and tear the arrangement. The wrap shows up as a near-period jump
    ///   between consecutive parameter samples (interior samples stay
    ///   inside the domain box), and each jump is cut at the exact seam
    ///   point ([`pin_intersection_point`]).
    fn marched_polylines(&self, mut mc: MarchedCurve, sa: &Surface3, sb: &Surface3) -> Vec<Curve3> {
        if !mc.closed {
            for k in [0, mc.points.len() - 1] {
                if let Some(p) =
                    tighten_boundary_point(sa, sb, mc.params_a[k], mc.params_b[k], self.snap * 0.5)
                {
                    mc.points[k] = p;
                }
            }
            return vec![Curve3::Polyline {
                points: mc.points,
                closed: false,
            }];
        }

        // Seam cuts on a closed curve: scalar ring position (vertex index
        // + segment fraction) and the refined crossing point.
        let track = |k: usize, i: usize| -> f64 {
            match k {
                0 => mc.params_a[i].0,
                1 => mc.params_a[i].1,
                2 => mc.params_b[i].0,
                _ => mc.params_b[i].1,
            }
        };
        let periods = [sa.period_u(), sa.period_v(), sb.period_u(), sb.period_v()];
        let bounds = [sa.domain_u(), sa.domain_v(), sb.domain_u(), sb.domain_v()];
        let mut cuts: Vec<(f64, Point3)> = Vec::new();
        for i in 0..mc.points.len() - 1 {
            for k in 0..4 {
                let Some(period) = periods[k] else { continue };
                let (w0, w1) = (track(k, i), track(k, i + 1));
                let delta = w1 - w0;
                if delta.abs() <= period / 2.0 {
                    continue;
                }
                // Forward through the high bound (w drops by ~a period) or
                // backward through the low one.
                let (pin_value, w1_unwrapped) = if delta < 0.0 {
                    (bounds[k].1, w1 + period)
                } else {
                    (bounds[k].0, w1 - period)
                };
                let s = ((pin_value - w0) / (w1_unwrapped - w0)).clamp(0.0, 1.0);
                let seed = [track(0, i), track(1, i), track(2, i), track(3, i)];
                let point = pin_intersection_point(sa, sb, seed, k, pin_value, self.snap * 0.5)
                    .unwrap_or_else(|| mc.points[i] + (mc.points[i + 1] - mc.points[i]) * s);
                cuts.push((i as f64 + s, point));
            }
        }
        if cuts.is_empty() {
            return vec![Curve3::Polyline {
                points: mc.points,
                closed: true,
            }];
        }
        cuts.sort_by(|a, b| a.0.total_cmp(&b.0));

        // Reassemble the ring as open fragments between consecutive cuts.
        let n = mc.points.len() - 1; // distinct vertices
        let mut fragments = Vec::new();
        for j in 0..cuts.len() {
            let (q0, p0) = cuts[j];
            let (mut q1, p1) = cuts[(j + 1) % cuts.len()];
            if j + 1 == cuts.len() {
                q1 += n as f64;
            }
            let mut points = vec![p0];
            let mut v = q0.floor() as usize + 1;
            while (v as f64) < q1 - 1e-12 {
                points.push(mc.points[v % n]);
                v += 1;
            }
            points.push(p1);
            if polyline_length(&points) > self.snap {
                fragments.push(Curve3::Polyline {
                    points,
                    closed: false,
                });
            }
        }
        fragments
    }

    fn face_boxes(&self, s: SolidTag) -> Vec<BoundingBox3> {
        self.face_polys[s]
            .iter()
            .zip(&self.solids[s].faces)
            .map(|(fp, face)| {
                broad_phase_face_box(
                    &face.surface,
                    fp.loops.iter().flatten().map(|&(_, p)| p),
                    self.tol.linear + self.snap,
                )
            })
            .collect()
    }

    /// Clip one SSI curve to both face regions; store surviving pieces as
    /// imprints.
    fn clip_imprint(
        &mut self,
        curve: &Curve3,
        fa: usize,
        fb: usize,
        box_a: &BoundingBox3,
        box_b: &BoundingBox3,
    ) {
        // Sampling stations: the full period for closed conics, a bbox
        // slab clip for lines, the vertices for (marched) polylines. For
        // closed curves the stations cover one period without repeating
        // the start, and `period` drives the wrap arithmetic below.
        let (ts, closed_curve, period) = match curve {
            Curve3::Line { origin, dir } => {
                let joint = box_a.intersection(box_b);
                let Some((t_lo, t_hi)) = clip_line_to_box(origin, dir, &joint) else {
                    return;
                };
                let n = IMPRINT_LINE_SAMPLES;
                let ts = (0..=n)
                    .map(|i| t_lo + (t_hi - t_lo) * i as f64 / n as f64)
                    .collect::<Vec<f64>>();
                (ts, false, 0.0)
            }
            Curve3::Polyline { points, closed } => {
                // Vertex-index parameterization: sampling at the integers
                // walks the polyline exactly (the last vertex of a closed
                // polyline repeats the first and is dropped).
                let distinct = if *closed {
                    points.len() - 1
                } else {
                    points.len()
                };
                let ts = (0..distinct).map(|i| i as f64).collect::<Vec<f64>>();
                (ts, *closed, (points.len() - 1) as f64)
            }
            _ => {
                let n = SAMPLES_PER_CIRCLE;
                let ts = (0..n)
                    .map(|i| TWO_PI * i as f64 / n as f64)
                    .collect::<Vec<f64>>();
                (ts, true, TWO_PI)
            }
        };
        let inside = |t: f64| -> bool {
            let p = curve.point(t);
            let pa = self.face_polys[0][fa].chart.param(&p, None);
            let pb = self.face_polys[1][fb].chart.param(&p, None);
            self.face_polys[0][fa].contains_for_clip(pa, self.snap)
                && self.face_polys[1][fb].contains_for_clip(pb, self.snap)
        };
        let flags: Vec<bool> = ts.iter().map(|&t| inside(t)).collect();

        if closed_curve && flags.iter().all(|&f| f) {
            // Entire ring inside both regions.
            self.imprints.push(Imprint {
                face_a: fa,
                face_b: fb,
                curve: curve.clone(),
                sampled: SampledCurve {
                    points: ts.iter().map(|&t| curve.point(t)).collect(),
                    closed: true,
                },
            });
            return;
        }

        // Extract maximal inside runs; refine both ends by bisection.
        let total = ts.len();
        let step = |i: usize| (i + 1) % total;
        let run_allowed = |i: usize| flags[i];
        let mut visited = vec![false; total];
        for start in 0..total {
            if !run_allowed(start) || visited[start] {
                continue;
            }
            // Walk back to the run's first sample (for closed curves the
            // run may wrap).
            let mut first = start;
            loop {
                let prev = (first + total - 1) % total;
                if (closed_curve || first > 0) && run_allowed(prev) && prev != start {
                    first = prev;
                    if first == start {
                        break;
                    }
                    continue;
                }
                break;
            }
            // Collect the run.
            let mut run = vec![first];
            visited[first] = true;
            let mut i = first;
            loop {
                let next = step(i);
                if !closed_curve && next == 0 {
                    break;
                }
                if run_allowed(next) && !visited[next] {
                    visited[next] = true;
                    run.push(next);
                    i = next;
                } else {
                    break;
                }
            }
            let mut pts: Vec<Point3> = Vec::with_capacity(run.len() + 2);
            // Refine entry point (before the run's first sample).
            let first_idx = run[0];
            let prev_idx = (first_idx + total - 1) % total;
            let marched = matches!(curve, Curve3::Polyline { .. });
            if (closed_curve || first_idx > 0) && !flags[prev_idx] {
                let mut t_out = ts[prev_idx];
                let t_in = ts[first_idx];
                if closed_curve && t_out > t_in {
                    t_out -= period;
                }
                let p = curve.point(refine_crossing(&inside, t_out, t_in));
                pts.push(if marched {
                    self.polish_clip_endpoint(p, fa, fb)
                } else {
                    p
                });
            }
            pts.extend(run.iter().map(|&i| curve.point(ts[i])));
            // Refine exit point (after the run's last sample).
            let last_idx = *run.last().expect("non-empty run");
            let next_idx = step(last_idx);
            if (closed_curve || last_idx + 1 < total) && !flags[next_idx] {
                let mut t_out = ts[next_idx];
                let t_in = ts[last_idx];
                if closed_curve && t_out < t_in {
                    t_out += period;
                }
                let p = curve.point(refine_crossing(&inside, t_out, t_in));
                pts.push(if marched {
                    self.polish_clip_endpoint(p, fa, fb)
                } else {
                    p
                });
            }
            // An open marched fragment can genuinely close on itself: a
            // ring cut at a single chart seam ends where it starts. Keep
            // it as an open imprint (the arrangement's ring path accepts
            // chains whose endpoints coincide) instead of dropping it
            // through the endpoint-coincidence guard below, which only
            // means "degenerate sliver" for analytic runs.
            let endpoints_coincide = (pts[0] - pts[pts.len() - 1]).norm() <= self.snap;
            let seam_cut_ring = !closed_curve
                && matches!(curve, Curve3::Polyline { .. })
                && endpoints_coincide
                && polyline_length(&pts) > self.snap * 4.0;
            if pts.len() >= 2 && (!endpoints_coincide || seam_cut_ring) {
                self.imprints.push(Imprint {
                    face_a: fa,
                    face_b: fb,
                    curve: curve.clone(),
                    sampled: SampledCurve {
                        points: pts,
                        closed: false,
                    },
                });
            } else if closed_curve && polyline_length(&pts) > self.snap * 4.0 {
                // A run on a closed curve whose refined endpoints coincide
                // (within snap) is the full ring minus a zero-width gap:
                // the excluded stretch is below resolution, so the ring is
                // inside. Dropping it instead would silently no-op the
                // boolean (of-ipt.5).
                self.imprints.push(Imprint {
                    face_a: fa,
                    face_b: fb,
                    curve: curve.clone(),
                    sampled: SampledCurve {
                        points: ts.iter().map(|&t| curve.point(t)).collect(),
                        closed: true,
                    },
                });
            }
        }
    }

    /// Polish a marched imprint's clip endpoint onto the exact junction it
    /// approximates. The bisected crossing lies on a polyline **chord**, up
    /// to a chord sagitta off the true intersection curve — but the true
    /// endpoint is where that curve crosses a boundary edge of one host
    /// face, i.e. the point on the crossed edge's exact curve where the
    /// *other* solid's surface residual vanishes (the edge already lies on
    /// its own solid's surface). Every imprint ending at that junction
    /// solves the same one-dimensional root, so the polished endpoints
    /// weld exactly; unpolished, two chords disagree by O(sagitta), far
    /// beyond the welding snap, and the arrangement tears (chains end
    /// mid-face, split points duplicate on the edge).
    ///
    /// Returns `p` unchanged when no boundary edge is plausibly involved
    /// (a fragment endpoint pinned on a chart seam and already exact) or
    /// the root refinement fails.
    fn polish_clip_endpoint(&self, p: Point3, fa: usize, fb: usize) -> Point3 {
        // The crossed edge: nearest over both faces' boundaries, accepted
        // within a discretization-scale band (the endpoint can sit a chord
        // sagitta off a curved boundary, but distinct edges are a face
        // extent apart).
        let mut best: Option<(f64, SolidTag, usize)> = None;
        for (s, f) in [(0usize, fa), (1usize, fb)] {
            let band = 0.02 * self.face_extents[s][f];
            for lp in &self.solids[s].faces[f].loops {
                for de in lp {
                    let sampled = &self.edge_samples[s][de.edge];
                    let d = polyline_distance(&sampled.points, sampled.closed, &p);
                    if d <= band && best.map(|(bd, _, _)| d < bd).unwrap_or(true) {
                        best = Some((d, s, de.edge));
                    }
                }
            }
        }
        let Some((_, s, e)) = best else {
            return p;
        };
        let edge = &self.solids[s].edges[e];
        let other = &self.solids[1 - s];
        // The imprint's face on the other solid carries the surface the
        // junction must also lie on.
        let sf = if s == 0 { fb } else { fa };
        let surface = &other.faces[sf].surface;
        let proj = edge.curve.project_point(&p);
        let mut t = if edge.closed {
            proj.t
        } else {
            proj.t.clamp(edge.t0, edge.t1)
        };
        // Newton on residual(edge(t)) = 0; the seed is within a chord
        // sagitta of the root, far inside its basin for transversal
        // crossings.
        let sanity = 0.05 * self.face_extents[0][fa].min(self.face_extents[1][fb]);
        for _ in 0..CLIP_REFINE_ITERATIONS {
            let q = edge.curve.point(t);
            let Some(r) = surface_residual(surface, &q) else {
                return p;
            };
            if r.abs() <= self.snap * 0.5 {
                return if (q - p).norm() <= sanity { q } else { p };
            }
            let Some(g) = surface_residual_gradient(surface, &q) else {
                return p;
            };
            let slope = g.dot(&edge.curve.derivative(t));
            if slope.abs() <= f64::MIN_POSITIVE {
                return p;
            }
            t -= r / slope;
            if !t.is_finite() {
                return p;
            }
            if !edge.closed {
                t = t.clamp(edge.t0, edge.t1);
            }
        }
        p
    }

    /// Phase 3: register the global split events. Every open imprint
    /// endpoint lies on some original edge of one of the solids and splits
    /// it there. Closed imprints hosted on a periodic face additionally
    /// straddle that face's parameter cover; they are cut at EVERY point
    /// where they cross the face's seam on each wrapped axis — `u` for
    /// cylinder/sphere/torus covers, `v` too for the doubly-periodic torus
    /// — splitting both the imprint and the seam edge, so the cover
    /// polygon sees them as boundary-to-boundary chords (of-7ld.7). A ring
    /// that wraps the axis crosses once (plus backtracking pairs); a
    /// winding-0 ring straddling the seam crosses an even number of times
    /// and becomes that many chords (of-43n).
    fn collect_splits(&mut self) {
        self.snap_imprint_endpoints_to_poles();
        let mut events: Vec<(CurveSource, Point3)> = Vec::new();
        let mut barriers: Vec<((SolidTag, usize), Point3)> = Vec::new();
        for (ii, imp) in self.imprints.iter().enumerate() {
            // Pole crossings: an imprint threaded through a sphere pole of
            // a host chart passes through an existing topology vertex (the
            // seam edge's endpoint) and must be split there, so its pieces
            // anchor at the pole vertex like any boundary-hitting imprint
            // (of-rb4). The pole is checked against the exact curve; for
            // open imprints it must also lie strictly inside the clipped
            // run (endpoint poles are handled by the snapping pre-pass and
            // need no split). Every pole an imprint touches — mid-run or
            // at a snapped endpoint — is also a chain barrier on that host
            // face: the pole is on the face boundary, so chains terminate
            // there instead of fusing through the junction into a false
            // interior ring.
            for (s, f) in [(0usize, imp.face_a), (1usize, imp.face_b)] {
                let chart = &self.face_polys[s][f].chart;
                let Some(poles) = chart.pole_points() else {
                    continue;
                };
                for pole in poles {
                    if !imp.sampled.closed {
                        let pts = &imp.sampled.points;
                        if (pts[0] - pole).norm() <= self.snap * 4.0
                            || (pts[pts.len() - 1] - pole).norm() <= self.snap * 4.0
                        {
                            barriers.push(((s, f), pole));
                            continue;
                        }
                    }
                    let t = imp.curve.project_point(&pole).t;
                    if (imp.curve.point(t) - pole).norm() > self.snap * EDGE_MATCH_SNAP {
                        continue;
                    }
                    if !imp.sampled.closed && !Self::interior_curve_param(imp, t, self.snap) {
                        continue;
                    }
                    events.push((CurveSource::Imprint { index: ii }, pole));
                    barriers.push(((s, f), pole));
                }
            }
            if !imp.sampled.closed {
                for endpoint in [
                    imp.sampled.points[0],
                    imp.sampled.points[imp.sampled.points.len() - 1],
                ] {
                    // No early break: a marched fragment endpoint can land
                    // on a seam edge of EACH solid at once (symmetric
                    // torus-torus / sphere-cylinder contacts put the seam
                    // crossings on shared symmetry planes), and both face
                    // covers then need their boundary vertex.
                    for (s, f) in [(0usize, imp.face_a), (1usize, imp.face_b)] {
                        // The exact-curve variant is the fallback: marched
                        // fragment endpoints sit bit-exact on seam-edge
                        // curves whose parameter range starts below zero
                        // (the sphere meridian), where the plain variant's
                        // clamped projection misses the wrap-around.
                        if let Some(edge) = self
                            .nearest_edge_of_face(&endpoint, s, f)
                            .or_else(|| self.nearest_edge_of_face_exact(&endpoint, s, f))
                        {
                            events.push((CurveSource::Edge { solid: s, edge }, endpoint));
                        }
                    }
                }
                continue;
            }
            for (s, f) in [(0usize, imp.face_a), (1usize, imp.face_b)] {
                let fp = &self.face_polys[s][f];
                for (axis, period) in [
                    (SeamAxis::U, fp.chart.period_u()),
                    (SeamAxis::V, fp.chart.period_v()),
                ] {
                    if period.is_none() {
                        continue;
                    }
                    for seam_point in seam_crossings(fp, &imp.curve, &imp.sampled.points, axis) {
                        events.push((CurveSource::Imprint { index: ii }, seam_point));
                        // Exact-curve edge matching: sphere/torus seam edges
                        // are circular arcs whose sampled polylines sag far
                        // beyond the acceptance band (of-7ld.5).
                        if let Some(edge) = self.nearest_edge_of_face_exact(&seam_point, s, f) {
                            events.push((CurveSource::Edge { solid: s, edge }, seam_point));
                        }
                        barriers.push(((s, f), seam_point));
                    }
                }
            }
        }
        for (source, p) in events {
            self.splits.entry(source).or_default().push(p);
        }
        for (key, p) in barriers {
            self.seam_barriers.entry(key).or_default().push(p);
        }
    }

    /// Canonicalize open imprints that terminate at a sphere pole of a
    /// host chart: the clip bisection converges to the pole only to its
    /// refinement precision (observed ~1e-8 off), which is wider than the
    /// vertex weld snap — the resulting endpoint vertex would not merge
    /// with the pole vertex the seam edge ends in. Snap such endpoints to
    /// the exact pole point so chains anchor at the existing vertex
    /// (of-rb4).
    fn snap_imprint_endpoints_to_poles(&mut self) {
        let band = self.snap * EDGE_MATCH_SNAP * 4.0;
        for imp in &mut self.imprints {
            if imp.sampled.closed {
                continue;
            }
            for (s, f) in [(0usize, imp.face_a), (1usize, imp.face_b)] {
                let Some(poles) = self.face_polys[s][f].chart.pole_points() else {
                    continue;
                };
                let last = imp.sampled.points.len() - 1;
                for i in [0, last] {
                    for pole in poles {
                        let d = (imp.sampled.points[i] - pole).norm();
                        if d > 0.0 && d <= band {
                            imp.sampled.points[i] = pole;
                        }
                    }
                }
            }
        }
    }

    /// Is curve parameter `t` strictly inside the parameter range of an
    /// open imprint's clipped run (not within snap of either end)? Used to
    /// keep pole splits off run endpoints, which the snapping pre-pass
    /// already canonicalizes.
    fn interior_curve_param(imp: &Imprint, t: f64, snap: f64) -> bool {
        let pts = &imp.sampled.points;
        let t0 = imp.curve.project_point(&pts[0]).t;
        let t1 = imp.curve.project_point(&pts[pts.len() - 1]).t;
        // Margin: the snap length expressed as a curve-parameter step via
        // the local speed (finite difference over one refinement step).
        let dt = 1e-4;
        let speed = (imp.curve.point(t + dt) - imp.curve.point(t)).norm() / dt;
        let margin = (snap * 4.0) / speed.max(1e-12);
        match imp.curve.period() {
            Some(period) => {
                let mut span = (t1 - t0).rem_euclid(period);
                // Coincident endpoints mean the open run wraps the whole
                // period (a ring cut open at a single point), not a
                // zero-length run — sub-snap runs never survive clipping.
                if span < margin {
                    span = period;
                }
                let tp = (t - t0).rem_euclid(period);
                tp > margin && tp < span - margin
            }
            None => {
                let (lo, hi) = (t0.min(t1), t0.max(t1));
                t > lo + margin && t < hi - margin
            }
        }
    }

    /// The boundary edge of face `(s, f)` nearest to `p`, if within
    /// acceptance distance. Each edge is measured both against its sampled
    /// polyline (open-imprint endpoints are bisected onto the region
    /// boundary polyline, so they sit on it) and against its exact curve
    /// (seam-crossing events are refined onto the exact imprint curve, so
    /// on a curved seam edge they sit a full sagitta off the polyline —
    /// of-7ld.7). The exact-curve parameter is clamped to the edge's range
    /// so an unbounded line extension cannot capture a distant point.
    fn nearest_edge_of_face(&self, p: &Point3, s: SolidTag, f: usize) -> Option<usize> {
        let mut best: Option<(f64, usize)> = None;
        for lp in &self.solids[s].faces[f].loops {
            for de in lp {
                let sampled = &self.edge_samples[s][de.edge];
                let mut d = polyline_distance(&sampled.points, sampled.closed, p);
                let edge = &self.solids[s].edges[de.edge];
                let proj = edge.curve.project_point(p);
                let t = if edge.closed {
                    proj.t
                } else {
                    proj.t.clamp(edge.t0, edge.t1)
                };
                d = d.min((p - edge.curve.point(t)).norm());
                if best.is_none() || d < best.expect("checked").0 {
                    best = Some((d, de.edge));
                }
            }
        }
        let (d, e) = best?;
        (d <= self.snap * EDGE_MATCH_SNAP).then_some(e)
    }

    /// Like [`Self::nearest_edge_of_face`], but measured against the
    /// edges' **exact curves** instead of their sampled polylines. Seam
    /// crossings are refined onto the exact imprint curve at the seam
    /// meridian's chart angle ([`refine_seam_point`]), so they sit on the
    /// seam edge's exact curve to root-find precision — but up to a
    /// polyline sagitta (`≈ 5.4e-4 * r`, far beyond the acceptance band)
    /// away from its samples when the seam edge is a circular arc, as on
    /// a sphere. Cylinder seams are straight, which is the only reason
    /// the polyline test ever worked for them (of-7ld.5).
    fn nearest_edge_of_face_exact(&self, p: &Point3, s: SolidTag, f: usize) -> Option<usize> {
        let mut best: Option<(f64, usize)> = None;
        for lp in &self.solids[s].faces[f].loops {
            for de in lp {
                let edge = &self.solids[s].edges[de.edge];
                let mut t = edge.curve.project_point(p).t;
                if let Some(period) = edge.curve.period() {
                    // Bring the projection into the edge's parameter
                    // window [t0, t0 + period).
                    t = edge.t0 + (t - edge.t0).rem_euclid(period);
                }
                // Off-range projections fall back to the nearer endpoint
                // (periodic wraparound makes a plain clamp wrong).
                let d = if t <= edge.t1 {
                    (edge.curve.point(t.max(edge.t0)) - p).norm()
                } else {
                    (edge.curve.point(edge.t0) - p)
                        .norm()
                        .min((edge.curve.point(edge.t1) - p).norm())
                };
                if best.is_none() || d < best.expect("checked").0 {
                    best = Some((d, de.edge));
                }
            }
        }
        let (d, e) = best?;
        (d <= self.snap * EDGE_MATCH_SNAP).then_some(e)
    }

    /// Phase 3b: split all source curves at their registered split points,
    /// producing the atomic arrangement/topology edges, grouped by source
    /// in polyline order.
    fn build_atoms(&self) -> (Vec<Atom>, HashMap<CurveSource, Vec<usize>>) {
        let mut atoms: Vec<Atom> = Vec::new();
        let mut by_source: HashMap<CurveSource, Vec<usize>> = HashMap::new();
        let push_all = |atoms: &mut Vec<Atom>,
                        by_source: &mut HashMap<CurveSource, Vec<usize>>,
                        pieces: Vec<Atom>,
                        source: CurveSource| {
            let ids: Vec<usize> = (atoms.len()..atoms.len() + pieces.len()).collect();
            atoms.extend(pieces);
            by_source.insert(source, ids);
        };
        for s in 0..2 {
            for (e, sampled) in self.edge_samples[s].iter().enumerate() {
                let source = CurveSource::Edge { solid: s, edge: e };
                let mut splits = self.splits.get(&source).cloned().unwrap_or_default();
                // A split closed edge loses embed_walk's ring rotation:
                // the loop walk still enters and leaves it at its
                // topological vertex, so the vertex must bound an atom
                // too or the traversal tears there (full-wrap imprints
                // crossing the torus seam circles, of-7ld.7). Inserted
                // first so a coincident imprint split defers to the
                // vertex's exact position in `split_sampled`'s dedup.
                if sampled.closed && !splits.is_empty() {
                    splits.insert(0, sampled.points[0]);
                }
                push_all(
                    &mut atoms,
                    &mut by_source,
                    split_sampled(sampled, &splits, self.snap),
                    source,
                );
            }
        }
        for (i, imp) in self.imprints.iter().enumerate() {
            let source = CurveSource::Imprint { index: i };
            let splits = self.splits.get(&source).cloned().unwrap_or_default();
            push_all(
                &mut atoms,
                &mut by_source,
                split_sampled(&imp.sampled, &splits, self.snap),
                source,
            );
        }
        (atoms, by_source)
    }

    /// Phases 4-5: per-face region splitting, classification, and topology
    /// reconstruction.
    fn reconstruct(
        &self,
        op: BooleanOp,
        atoms: Vec<Atom>,
        atoms_by_source: HashMap<CurveSource, Vec<usize>>,
    ) -> CoreResult<BooleanOutput> {
        let mut kept: Vec<KeptRegion> = Vec::new();
        for s in 0..2 {
            let other = 1 - s;
            for f in 0..self.solids[s].faces.len() {
                let face = &self.solids[s].faces[f];
                let face_poly = &self.face_polys[s][f];

                // Initial region: the face's own loops, atom by atom. The
                // stored loops wind CCW as seen from the face's outward side;
                // when that side opposes the surface normal (the chart normal),
                // that is CW in the chart, so the whole region machinery —
                // shoelace area sign, `region_interior_point`'s inward left
                // normal, `apply_chain`'s ring/hole split — would invert.
                // Reverse the traversal here so every traced cycle is
                // CCW-in-chart; `build_output` undoes it to restore the loop's
                // outward-CCW winding for the result topology.
                let flip = !face.outward_along_normal;
                let mut cycles = Vec::new();
                for lp in &face.loops {
                    let mut darts: DartChain = Vec::new();
                    for de in lp {
                        let source = CurveSource::Edge {
                            solid: s,
                            edge: de.edge,
                        };
                        let ids = &atoms_by_source[&source];
                        if de.forward {
                            darts.extend(ids.iter().map(|&ai| (ai, true)));
                        } else {
                            darts.extend(ids.iter().rev().map(|&ai| (ai, false)));
                        }
                    }
                    if flip {
                        darts = reverse_chain(&darts);
                    }
                    cycles.push(embed_cycle(face_poly, &atoms, darts));
                }
                let mut regions = vec![Region { cycles }];

                // Imprint atoms hosted on this face, merged into chains.
                let mut imprint_ids: Vec<usize> = Vec::new();
                for (i, imp) in self.imprints.iter().enumerate() {
                    let hosted = (s == 0 && imp.face_a == f) || (s == 1 && imp.face_b == f);
                    if hosted {
                        imprint_ids
                            .extend(atoms_by_source[&CurveSource::Imprint { index: i }].iter());
                    }
                }
                // Chain barriers: the face's registered seam crossings
                // (of-43n/of-rb4) plus every split point on its own
                // boundary edges — marched fragments arrive pre-cut at
                // chart seams, so their junctions END on a seam edge
                // without crossing it and appear only as edge splits
                // (of-yet).
                let mut barriers = self.seam_barriers.get(&(s, f)).cloned().unwrap_or_default();
                for lp in &face.loops {
                    for de in lp {
                        let source = CurveSource::Edge {
                            solid: s,
                            edge: de.edge,
                        };
                        barriers.extend(self.splits.get(&source).into_iter().flatten());
                    }
                }
                for chain in merge_imprint_chains(&atoms, &imprint_ids, self.snap, &barriers) {
                    apply_chain(face_poly, &atoms, &mut regions, chain, self.snap)?;
                }

                for region in regions {
                    let sample =
                        region_interior_point(&face_poly.chart, &region).ok_or_else(|| {
                            CoreError::Degenerate {
                                context: "boolean::classify",
                                reason: "could not find an interior sample point for a face \
                                     region"
                                    .into(),
                            }
                        })?;
                    let inside_other = self.contains_point(other, &sample)?;
                    let (keep, reverse) = keep_table(op, s, inside_other);
                    if keep {
                        kept.push(KeptRegion {
                            solid: s,
                            face: f,
                            region,
                            reverse,
                        });
                    }
                }
            }
        }

        // Invert the by-source index: which source curve produced each atom
        // (build_output binds that curve's geometry to the atom's edge).
        let mut atom_source: Vec<Option<CurveSource>> = vec![None; atoms.len()];
        for (&source, ids) in &atoms_by_source {
            for &ai in ids {
                atom_source[ai] = Some(source);
            }
        }
        let atom_source: Vec<CurveSource> = atom_source
            .into_iter()
            .map(|s| s.expect("every atom comes from exactly one source"))
            .collect();

        build_output(self, op, &atoms, &atom_source, kept)
    }

    /// Ray-parity containment of a point in one of the input solids.
    fn contains_point(&self, s: SolidTag, p: &Point3) -> CoreResult<bool> {
        'dirs: for raw in RAY_DIRECTIONS {
            let dir = Vector3::new(raw[0], raw[1], raw[2]).normalize();
            let mut hits = 0usize;
            for (f, face) in self.solids[s].faces.iter().enumerate() {
                let fp = &self.face_polys[s][f];
                for t in ray_surface_hits(&face.surface, p, &dir) {
                    let ambiguous = self.snap * 10.0;
                    if t < -ambiguous {
                        continue; // behind the ray: irrelevant
                    }
                    if t <= ambiguous {
                        // Would mean the sample point lies on this surface;
                        // only ambiguous if it is actually on the face.
                        let uv = fp.localize(fp.chart.param(p, None));
                        if fp.contains(uv) {
                            continue 'dirs;
                        }
                        continue;
                    }
                    let hit = p + dir * t;
                    // Grazing incidence: retry with another direction.
                    let (nu, nv) = fp.chart.param(&hit, None);
                    let n = fp.chart.normal(nu, nv);
                    if n.dot(&dir).abs() < 1e-6 {
                        continue 'dirs;
                    }
                    let uv = fp.localize(fp.chart.param(&hit, None));
                    if !fp.contains(uv) {
                        continue;
                    }
                    // Near a region boundary: retry.
                    if self.near_face_boundary(s, f, &hit) {
                        continue 'dirs;
                    }
                    hits += 1;
                }
            }
            return Ok(hits % 2 == 1);
        }
        Err(CoreError::Degenerate {
            context: "boolean::ray_classify",
            reason: format!("all classification rays from {p:?} hit degenerate cases"),
        })
    }

    fn near_face_boundary(&self, s: SolidTag, f: usize, p: &Point3) -> bool {
        let band = self.face_extents[s][f] * BOUNDARY_BAND_FRAC;
        for lp in &self.solids[s].faces[f].loops {
            for de in lp {
                let sampled = &self.edge_samples[s][de.edge];
                if polyline_distance(&sampled.points, sampled.closed, p) < band {
                    return true;
                }
            }
        }
        false
    }
}

/// (keep?, reverse orientation?) for a region of `solid` that is
/// `inside_other` the other body.
fn keep_table(op: BooleanOp, solid: SolidTag, inside_other: bool) -> (bool, bool) {
    match (op, solid, inside_other) {
        (BooleanOp::Unite, _, inside) => (!inside, false),
        (BooleanOp::Subtract, 0, inside) => (!inside, false),
        (BooleanOp::Subtract, 1, inside) => (inside, true),
        (BooleanOp::Intersect, _, inside) => (inside, false),
        _ => unreachable!(),
    }
}

// ---------------------------------------------------------------------
// Geometry helpers
// ---------------------------------------------------------------------

fn append_directed(out: &mut Vec<Point3>, sampled: &SampledCurve, forward: bool) {
    // Loops concatenate edge polylines; each edge contributes all its
    // samples except the final endpoint (the next edge starts there). For
    // closed (ring) edges the sample list already omits the repeated seam.
    if forward {
        if sampled.closed {
            out.extend_from_slice(&sampled.points);
        } else {
            out.extend_from_slice(&sampled.points[..sampled.points.len() - 1]);
        }
    } else if sampled.closed {
        out.push(sampled.points[0]);
        out.extend(sampled.points[1..].iter().rev());
    } else {
        out.extend(sampled.points[1..].iter().rev());
    }
}

/// Clip the line `origin + t·dir` to an axis-aligned box (slab method).
fn clip_line_to_box(origin: &Point3, dir: &Vector3, bx: &BoundingBox3) -> Option<(f64, f64)> {
    let (mut t0, mut t1) = (f64::NEG_INFINITY, f64::INFINITY);
    for k in 0..3 {
        let (o, d, lo, hi) = (origin[k], dir[k], bx.min[k], bx.max[k]);
        if d.abs() < 1e-15 {
            if o < lo || o > hi {
                return None;
            }
            continue;
        }
        let (mut a, mut b) = ((lo - o) / d, (hi - o) / d);
        if a > b {
            std::mem::swap(&mut a, &mut b);
        }
        t0 = t0.max(a);
        t1 = t1.min(b);
    }
    (t0 < t1 && t0.is_finite() && t1.is_finite()).then_some((t0, t1))
}

/// Bisection: `t_out` fails the predicate, `t_in` passes; return the
/// crossing parameter.
fn refine_crossing(inside: &dyn Fn(f64) -> bool, mut t_out: f64, mut t_in: f64) -> f64 {
    for _ in 0..CLIP_REFINE_ITERATIONS {
        let mid = 0.5 * (t_out + t_in);
        if inside(mid) {
            t_in = mid;
        } else {
            t_out = mid;
        }
    }
    0.5 * (t_out + t_in)
}

/// Distance from `p` to a polyline (closed if `closed`).
fn polyline_distance(points: &[Point3], closed: bool, p: &Point3) -> f64 {
    let n = points.len();
    let segs = if closed { n } else { n - 1 };
    let mut best = f64::INFINITY;
    for i in 0..segs {
        let a = points[i];
        let b = points[(i + 1) % n];
        let ab = b - a;
        let len2 = ab.norm_squared();
        let t = if len2 > 0.0 {
            ((p - a).dot(&ab) / len2).clamp(0.0, 1.0)
        } else {
            0.0
        };
        best = best.min((p - (a + ab * t)).norm());
    }
    best
}

/// Split a sampled curve at the given 3D points, producing atoms. Split
/// points are inserted into the polyline at their nearest segment.
fn split_sampled(sampled: &SampledCurve, splits: &[Point3], snap: f64) -> Vec<Atom> {
    // Position each split point on the polyline: (segment index, fraction).
    let n = sampled.points.len();
    let segs = if sampled.closed { n } else { n - 1 };
    let mut located: Vec<(usize, f64, Point3)> = Vec::new();
    'next_split: for sp in splits {
        let mut best = (f64::INFINITY, 0usize, 0.0f64);
        for i in 0..segs {
            let a = sampled.points[i];
            let b = sampled.points[(i + 1) % n];
            let ab = b - a;
            let len2 = ab.norm_squared();
            let t = if len2 > 0.0 {
                ((sp - a).dot(&ab) / len2).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let d = (sp - (a + ab * t)).norm();
            if d < best.0 {
                best = (d, i, t);
            }
        }
        // Merge duplicates (both host faces report the same endpoint).
        for (i, t, _) in &located {
            if *i == best.1
                && (t - best.2).abs() * (sampled.points[(*i + 1) % n] - sampled.points[*i]).norm()
                    < snap * 10.0
            {
                continue 'next_split;
            }
        }
        located.push((best.1, best.2, *sp));
    }
    if located.is_empty() {
        return vec![Atom {
            points: sampled.points.clone(),
            closed: sampled.closed,
        }];
    }
    located.sort_by(|a, b| (a.0, a.1).partial_cmp(&(b.0, b.1)).expect("finite"));

    // Build the augmented point list with split-point markers.
    let mut aug: Vec<(Point3, bool)> = Vec::new();
    let mut li = 0;
    for i in 0..n {
        aug.push((sampled.points[i], false));
        while li < located.len() && located[li].0 == i {
            let (_, t, sp) = located[li];
            if t <= 1e-9 {
                // Coincides with the segment start sample.
                let last = aug.last_mut().expect("just pushed");
                last.0 = sp;
                last.1 = true;
            } else {
                aug.push((sp, true));
            }
            li += 1;
        }
    }
    if !sampled.closed {
        // Ensure endpoints are markers so open curves split cleanly.
        aug.first_mut().expect("non-empty").1 = true;
        aug.last_mut().expect("non-empty").1 = true;
    }

    // Cut at markers.
    let m = aug.len();
    let marker_positions: Vec<usize> = aug
        .iter()
        .enumerate()
        .filter(|(_, (_, mk))| *mk)
        .map(|(i, _)| i)
        .collect();
    let mut atoms = Vec::new();
    if sampled.closed {
        let k = marker_positions.len();
        for w in 0..k {
            let start = marker_positions[w];
            let end = marker_positions[(w + 1) % k];
            let mut pts = Vec::new();
            let mut i = start;
            loop {
                pts.push(aug[i].0);
                if i == end && !pts.is_empty() && pts.len() > 1 {
                    break;
                }
                i = (i + 1) % m;
                if i == start {
                    // Single marker on a ring: the whole ring is one open
                    // atom from the marker back to itself.
                    pts.push(aug[start].0);
                    break;
                }
            }
            atoms.push(Atom {
                points: pts,
                closed: false,
            });
            if k == 1 {
                break;
            }
        }
    } else {
        for w in marker_positions.windows(2) {
            let pts: Vec<Point3> = aug[w[0]..=w[1]].iter().map(|(p, _)| *p).collect();
            if pts.len() >= 2 {
                atoms.push(Atom {
                    points: pts,
                    closed: false,
                });
            }
        }
    }
    atoms
        .into_iter()
        .filter(|a| a.closed || polyline_length(&a.points) > snap)
        .collect()
}

fn polyline_length(points: &[Point3]) -> f64 {
    points.windows(2).map(|w| (w[1] - w[0]).norm()).sum()
}

/// Which parameter axis of a chart a seam crossing works along: `U` for
/// the seam meridian of cylinder/sphere/torus covers, `V` for the torus's
/// second (minor-angle) seam.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SeamAxis {
    U,
    V,
}

impl SeamAxis {
    /// This axis's coordinate of a parameter pair.
    fn coord(self, uv: (f64, f64)) -> f64 {
        match self {
            SeamAxis::U => uv.0,
            SeamAxis::V => uv.1,
        }
    }
}

/// Every point where a closed imprint on a periodic face crosses the
/// face's seam along `axis` (the minimum side of its cover window on that
/// axis). The sampled polyline `points` locates the bracketing samples;
/// each crossing is then refined against the exact `curve` by bisecting
/// `w(t) = level` (`w` the axis coordinate), so the returned points lie
/// on the curve (and on the seam) to root-find precision. Interpolating
/// the bracketing chord instead would leave the point off the curve by up
/// to the sagitta `r * (1 - cos(pi / SAMPLES_PER_CIRCLE)) ≈ 5.4e-4 * r`,
/// which crosses
/// [`MAX_ALLOWED_TOLERANCE`](crate::check::MAX_ALLOWED_TOLERANCE) once
/// r ≳ 19 and would be recorded as the seam vertex's tolerance.
///
/// In unwrapped coordinates the seam is every level `seam_w + 2πk`, so
/// all instances inside the polyline's range are scanned. A ring that
/// wraps the period crosses an odd number of times (once when it is a
/// monotonic graph of `w`, as lines, circles, and ellipses are on their
/// host cover; non-monotonic backtracking adds cancelling pairs). A
/// winding-0 ring that straddles the seam without enclosing the chart's
/// axis crosses an even number of times — e.g. a sphere cap about a point
/// ON the seam meridian crosses it twice (of-43n) — and previously was
/// never split at all, leaving its uv embedding straddling the cover
/// edge. A sample landing exactly on a level registers its crossing from
/// both adjacent segments; `split_sampled`'s snap merge deduplicates the
/// repeat downstream.
fn seam_crossings(
    face_poly: &FaceRegionPoly,
    curve: &Curve3,
    points: &[Point3],
    axis: SeamAxis,
) -> Vec<Point3> {
    let uv = map_polyline(&face_poly.chart, points);
    // Closing segment: unwrap the first point relative to the last.
    let close_w = {
        let mut w = axis.coord(uv[0]);
        let last = axis.coord(uv[uv.len() - 1]);
        while w - last > std::f64::consts::PI {
            w -= TWO_PI;
        }
        while last - w > std::f64::consts::PI {
            w += TWO_PI;
        }
        w
    };
    // Seam level: the face cover's minimum along the axis.
    let mut seam_w = f64::INFINITY;
    for lp in &face_poly.loops {
        for (q, _) in lp {
            seam_w = seam_w.min(axis.coord(*q));
        }
    }
    let (w_min, w_max) = uv
        .iter()
        .map(|&q| axis.coord(q))
        .chain([close_w])
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), w| {
            (lo.min(w), hi.max(w))
        });
    // First seam level instance strictly above w_min; scan every instance
    // up to w_max.
    let mut level = seam_w;
    while level > w_min {
        level -= TWO_PI;
    }
    while level <= w_min {
        level += TWO_PI;
    }
    let n = uv.len();
    let mut crossings = Vec::new();
    while level <= w_max {
        for i in 0..n {
            let w0 = axis.coord(uv[i]);
            let w1 = if i + 1 < n {
                axis.coord(uv[i + 1])
            } else {
                close_w
            };
            if (w0 - level) * (w1 - level) <= 0.0 && (w1 - w0).abs() > 1e-15 {
                let p0 = points[i];
                let p1 = points[(i + 1) % n];
                crossings.push(refine_seam_point(
                    &face_poly.chart,
                    curve,
                    axis,
                    level,
                    (uv[i], w1),
                    (&p0, &p1),
                ));
            }
        }
        level += TWO_PI;
    }
    crossings
}

/// Refine a seam crossing bracketed by consecutive imprint samples
/// `p0`/`p1` (`hint_uv` the unwrapped parameters at `p0`, `w1` the axis
/// coordinate at `p1`, straddling `level`) to a point on the exact
/// `curve` at axis coordinate `level`, by bisecting `w(t) = level` over
/// the curve-parameter bracket recovered by closest-point projection.
fn refine_seam_point(
    chart: &Chart,
    curve: &Curve3,
    axis: SeamAxis,
    level: f64,
    (hint_uv, w1): ((f64, f64), f64),
    (p0, p1): (&Point3, &Point3),
) -> Point3 {
    let w0 = axis.coord(hint_uv);
    let t0 = curve.project_point(p0).t;
    let mut t1 = curve.project_point(p1).t;
    if let Some(period) = curve.period() {
        // Samples advance in curve parameter; unwrap the far bracket
        // forward past a period seam.
        while t1 <= t0 {
            t1 += period;
        }
    }
    // Angles along the bracket stay within a sample step of `hint_uv`,
    // far inside the ±π unwrap window of the hint.
    let residual = |t: f64| axis.coord(chart.param(&curve.point(t), Some(hint_uv))) - level;
    let (mut fa, fb) = (w0 - level, w1 - level);
    if fa == 0.0 {
        return curve.point(t0);
    }
    if fb == 0.0 {
        return curve.point(t1);
    }
    if fa * fb > 0.0 || t1 <= t0 {
        // Projection disagrees with the polyline bracketing (degenerate
        // segment); fall back to the chord point.
        return *p0 + (*p1 - *p0) * ((level - w0) / (w1 - w0));
    }
    let (mut ta, mut tb) = (t0, t1);
    for _ in 0..CLIP_REFINE_ITERATIONS {
        let tm = 0.5 * (ta + tb);
        let fm = residual(tm);
        if fm == 0.0 {
            return curve.point(tm);
        }
        if (fm > 0.0) == (fa > 0.0) {
            ta = tm;
            fa = fm;
        } else {
            tb = tm;
        }
    }
    curve.point(0.5 * (ta + tb))
}

// ---------------------------------------------------------------------
// Arrangement: region tracing in parameter space
// ---------------------------------------------------------------------

/// One traced region of a face: the outer cycle plus hole cycles, as
/// sequences of directed atoms with their param-space polylines.
#[derive(Debug, Clone)]
struct Region {
    /// cycles[0] is the outer boundary (positive area); the rest are holes.
    cycles: Vec<Cycle>,
}

#[derive(Debug, Clone)]
struct Cycle {
    /// (atom index, forward?) in traversal order.
    darts: Vec<(usize, bool)>,
    /// Concatenated param-space polyline (closed; last connects to first),
    /// paired with the 3D points.
    poly: Vec<((f64, f64), Point3)>,
    area: f64,
    /// Index into `poly` of each dart's first point (the cycle vertices).
    dart_offsets: Vec<usize>,
}

struct KeptRegion {
    solid: SolidTag,
    face: usize,
    region: Region,
    reverse: bool,
}

/// A sequence of directed atoms (the walk order of a cycle or chain).
type DartChain = Vec<(usize, bool)>;

/// A polyline vertex in a face's parameter cover: `(uv, 3D point)`.
type CoverPoint = ((f64, f64), Point3);

/// Directed 3D endpoints of an atom traversal.
fn dart_endpoints(atom: &Atom, forward: bool) -> (Point3, Point3) {
    let first = atom.points[0];
    let last = if atom.closed {
        first
    } else {
        atom.points[atom.points.len() - 1]
    };
    if forward {
        (first, last)
    } else {
        (last, first)
    }
}

/// Embed a walk of darts into the face's parameter cover: continuous angle
/// unwrapping along the walk, ring atoms rotated to start where the walk
/// stands, and the finished polyline shifted whole periods so every cycle
/// of a face shares one cover window.
fn embed_walk(
    face_poly: &FaceRegionPoly,
    atoms: &[Atom],
    darts: &[(usize, bool)],
    keep_final: bool,
) -> (Vec<CoverPoint>, Vec<usize>) {
    let mut poly: Vec<((f64, f64), Point3)> = Vec::new();
    let mut offsets = Vec::with_capacity(darts.len());
    let mut walk_pos: Option<Point3> = None;
    // A walk that STARTS at a sphere pole has no arrival longitude yet:
    // the embedder places the pole at a placeholder `u`. For cycles the
    // true arrival longitude is the walk's final meridian (the implicit
    // closure), so the placeholder is fixed up after the walk (of-rb4).
    let mut initial_pole: Option<f64> = None;
    // Every cycle handed to this walk is intended CCW-in-chart
    // (`reconstruct` reverses flipped face loops before embedding, and
    // `apply_chain` builds region outers), so pole closure rows orient CCW.
    let mut emb = CoverEmbedder::new(&face_poly.chart, true);
    for (k, &(ai, forward)) in darts.iter().enumerate() {
        let atom = &atoms[ai];
        let mut pts: Vec<Point3> = if forward {
            atom.points.clone()
        } else {
            atom.points.iter().rev().copied().collect()
        };
        if atom.closed {
            if let Some(prev) = walk_pos {
                let rot = pts
                    .iter()
                    .enumerate()
                    .min_by(|a, b| (a.1 - prev).norm().total_cmp(&(b.1 - prev).norm()))
                    .map(|(i, _)| i)
                    .unwrap_or(0);
                pts.rotate_left(rot);
            }
        }
        let last_dart = k + 1 == darts.len();
        let take = if atom.closed || (last_dart && keep_final) {
            pts.len()
        } else {
            pts.len() - 1
        };
        for (j, p) in pts[..take].iter().enumerate() {
            let first = poly.is_empty();
            if first {
                initial_pole = face_poly.chart.pole_v(p);
            }
            emb.push(*p, &mut poly);
            if j == 0 {
                // The dart's vertex is the point itself — when it leaves a
                // pole, the departure end of the pole row is emitted just
                // before it and belongs to the traversal, not the vertex.
                offsets.push(poly.len() - 1);
            }
            if first {
                // Start the walk inside the face's cover window so
                // intermediate unwrapping stays near it (the final
                // mean-shift below does the real alignment).
                let uv0 = poly[0].0;
                let (du, dv) = {
                    let local = face_poly.localize(uv0);
                    (local.0 - uv0.0, local.1 - uv0.1)
                };
                if du != 0.0 || dv != 0.0 {
                    for (uv, _) in poly.iter_mut() {
                        uv.0 += du;
                        uv.1 += dv;
                    }
                    emb.shift(du, dv);
                }
            }
        }
        walk_pos = Some(if atom.closed {
            pts[0]
        } else {
            pts[pts.len() - 1]
        });
    }
    // Fix up a cycle that started at a pole: the placeholder longitude at
    // poly[0] becomes the true arrival longitude (the final meridian of
    // the closing walk), and the departure end of the pole row — chosen
    // relative to the placeholder — is re-derived from it, shifting the
    // rest of the walk by the whole-period difference. Without this the
    // pole row sweeps meridians outside the region whenever the arrival
    // meridian is not the placeholder, and the cover polygon overlaps its
    // neighbors (of-rb4). Chains (`keep_final`) have no closure: their
    // start-pole longitude is only ever used for 3D-anchored matching.
    if !keep_final && poly.len() >= 3 {
        if let Some(vp) = initial_pole {
            let row_end_is_pole = face_poly.chart.pole_v(&poly[1].1) == Some(vp);
            let last_at_pole = face_poly.chart.pole_v(&poly[poly.len() - 1].1).is_some();
            if row_end_is_pole && !last_at_pole {
                let u_arr = poly[poly.len() - 1].0.0;
                let u_out_old = poly[1].0.0;
                // Same sweep rule as the embedder: CCW keeps the interior
                // meridians left of the walk (north row toward -u, south
                // row toward +u); a departure within POLE_TURN_EPS of the
                // arrival is a doubling-back and sweeps the full period.
                let toward_neg_u = vp > 0.0;
                let turn = if toward_neg_u {
                    (u_arr - u_out_old).rem_euclid(TWO_PI)
                } else {
                    (u_out_old - u_arr).rem_euclid(TWO_PI)
                };
                let turn = if turn < POLE_TURN_EPS || TWO_PI - turn < POLE_TURN_EPS {
                    TWO_PI
                } else {
                    turn
                };
                let u_out_new = if toward_neg_u {
                    u_arr - turn
                } else {
                    u_arr + turn
                };
                let delta = u_out_new - u_out_old;
                poly[0].0.0 = u_arr;
                if delta != 0.0 {
                    for ((u, _), _) in poly[1..].iter_mut() {
                        *u += delta;
                    }
                }
            }
        }
    }
    // Align the whole polyline into the face's cover window so cycles,
    // holes, and probes of one face are mutually comparable. Each periodic
    // axis is aligned independently (only the torus wraps `v`).
    if !poly.is_empty() {
        if face_poly.chart.period_u().is_some() {
            let mean = poly.iter().map(|((u, _), _)| u).sum::<f64>() / poly.len() as f64;
            let shift = face_poly.localize((mean, 0.0)).0 - mean;
            if shift.abs() > 1e-9 {
                for ((u, _), _) in poly.iter_mut() {
                    *u += shift;
                }
            }
        }
        if face_poly.chart.period_v().is_some() {
            let mean = poly.iter().map(|((_, v), _)| v).sum::<f64>() / poly.len() as f64;
            let shift = face_poly.localize((0.0, mean)).1 - mean;
            if shift.abs() > 1e-9 {
                for ((_, v), _) in poly.iter_mut() {
                    *v += shift;
                }
            }
        }
    }
    (poly, offsets)
}

fn shoelace(poly: &[((f64, f64), Point3)]) -> f64 {
    0.5 * poly
        .iter()
        .enumerate()
        .map(|(i, ((x0, y0), _))| {
            let ((x1, y1), _) = poly[(i + 1) % poly.len()];
            x0 * y1 - x1 * y0
        })
        .sum::<f64>()
}

fn embed_cycle(face_poly: &FaceRegionPoly, atoms: &[Atom], darts: DartChain) -> Cycle {
    let (poly, dart_offsets) = embed_walk(face_poly, atoms, &darts, false);
    let area = shoelace(&poly);
    Cycle {
        darts,
        poly,
        area,
        dart_offsets,
    }
}

fn reverse_chain(darts: &[(usize, bool)]) -> DartChain {
    darts.iter().rev().map(|&(a, f)| (a, !f)).collect()
}

/// Merge a face's imprint atoms into maximal chains through their shared
/// (interior) endpoints. Closed atoms become single-dart chains.
///
/// Chains never extend through a `barrier` point — the host face's own
/// seam crossings (see `Pipeline::seam_barriers`) and the split points on
/// its own boundary edges (marched fragment endpoints pre-cut at chart
/// seams, of-yet). Those junctions lie on the face boundary, where a
/// chain must terminate as a chord endpoint: a winding-0 ring seam-split
/// into two chords shares BOTH endpoints between its halves and would
/// otherwise re-merge into a full ring whose uv embedding straddles the
/// cover edge (of-43n). On the imprint's other host face the same points
/// are face-interior and carry no barrier, so the ring correctly
/// re-merges and applies as an interior ring there. Sphere poles an
/// imprint touches are barriers for the same reason: the pole is an
/// existing topology vertex on the face boundary, and merging through it
/// would fuse pole-to-pole chords into a closed chain that `apply_chain`
/// mistakes for an interior ring (of-rb4).
fn merge_imprint_chains(
    atoms: &[Atom],
    ids: &[usize],
    snap: f64,
    barriers: &[Point3],
) -> Vec<DartChain> {
    use std::collections::HashSet;
    let at_barrier = |p: &Point3| barriers.iter().any(|b| (p - b).norm() <= snap * 4.0);
    let mut chains: Vec<DartChain> = ids
        .iter()
        .filter(|&&ai| atoms[ai].closed)
        .map(|&ai| vec![(ai, true)])
        .collect();
    let open: Vec<usize> = ids
        .iter()
        .copied()
        .filter(|&ai| !atoms[ai].closed)
        .collect();
    let mut adjacency: SnapMap<(usize, bool)> = SnapMap::new(snap * 4.0);
    for &ai in &open {
        adjacency.insert(atoms[ai].points[0], (ai, true));
        adjacency.insert(atoms[ai].points[atoms[ai].points.len() - 1], (ai, false));
    }
    let mut used: HashSet<usize> = HashSet::new();
    for &seed in &open {
        if used.contains(&seed) {
            continue;
        }
        used.insert(seed);
        let mut chain: std::collections::VecDeque<(usize, bool)> = [(seed, true)].into();
        loop {
            let &(a, fwd) = chain.back().expect("non-empty");
            let (_, end) = dart_endpoints(&atoms[a], fwd);
            if at_barrier(&end) {
                break;
            }
            let cands = adjacency.matches(&end);
            match cands.iter().find(|(c, _)| !used.contains(c)) {
                Some(&(c, at_start)) => {
                    used.insert(c);
                    chain.push_back((c, at_start));
                }
                None => break,
            }
        }
        loop {
            let &(a, fwd) = chain.front().expect("non-empty");
            let (start, _) = dart_endpoints(&atoms[a], fwd);
            if at_barrier(&start) {
                break;
            }
            let cands = adjacency.matches(&start);
            match cands.iter().find(|(c, _)| !used.contains(c)) {
                Some(&(c, at_start)) => {
                    used.insert(c);
                    // The predecessor must END at our chain start: forward
                    // if the junction is its endpoint, reversed if its start.
                    chain.push_front((c, !at_start));
                }
                None => break,
            }
        }
        chains.push(chain.into());
    }
    chains
}

/// Match an embedded chain's endpoints to two distinct vertices of a
/// cycle, allowing a whole-period shift of the chain on each periodic
/// axis (seam chords match the two cover copies of one 3D point).
/// Distances and the acceptance band use the arc-length metric — `u`
/// scaled by `scale.0`, `v` by `scale.1` — because on curved charts the
/// parameter axes carry different units (cylinder: radians vs. model
/// length; sphere/torus: two angles at different radii), and a mixed
/// euclidean metric mis-ranks candidate vertices at extreme aspect
/// ratios.
///
/// ALL viable vertex pairs are returned, best score first. A chord with
/// BOTH endpoints on the same seam (a seam-split winding-0 ring half,
/// of-43n) matches the two cover copies of its endpoints equally well —
/// the scores tie at zero and the geometry alone cannot rank them —
/// but slicing the host cycle at the wrong copies winds one piece CW.
/// `apply_chain` disambiguates by trying candidates until the split
/// yields two CCW pieces.
///
/// Chain endpoints at a sphere pole match a pole vertex by 3D
/// coincidence instead of the uv metric — the vertex's stored uv is one
/// arbitrary representative of a whole collapsed pole row (of-rb4). And
/// a chain whose two ends coincide in 3D is a closed loop that may
/// legitimately anchor twice at ONE vertex (an imprint network closing
/// at a pole), so such chains may return pairs with `i == j`.
fn match_chain_to_cycle(
    cycle: &Cycle,
    chain_poly: &[((f64, f64), Point3)],
    periods: (Option<f64>, Option<f64>),
    scale: (f64, f64),
    chart: &Chart,
) -> Vec<(usize, usize)> {
    let (u_scale, v_scale) = scale;
    let (mut lo, mut hi) = (
        (f64::INFINITY, f64::INFINITY),
        (f64::NEG_INFINITY, f64::NEG_INFINITY),
    );
    for ((u, v), _) in &cycle.poly {
        lo = (lo.0.min(*u), lo.1.min(*v));
        hi = (hi.0.max(*u), hi.1.max(*v));
    }
    let eps = ((hi.0 - lo.0) * u_scale + (hi.1 - lo.1) * v_scale).max(1e-12) * 1e-5;
    let (s_uv, s_p) = chain_poly[0];
    let (e_uv, e_p) = chain_poly[chain_poly.len() - 1];
    let axis_shifts = |period: Option<f64>| -> Vec<f64> {
        match period {
            Some(p) => vec![0.0, -p, p],
            None => vec![0.0],
        }
    };
    let (shifts_u, shifts_v) = (axis_shifts(periods.0), axis_shifts(periods.1));
    let nearest = |uv: (f64, f64), p: &Point3| -> (usize, f64) {
        cycle
            .dart_offsets
            .iter()
            .enumerate()
            .map(|(k, &off)| {
                let (vuv, vp) = cycle.poly[off];
                // A vertex at a sphere pole is a whole pole row in uv: its
                // stored longitude is one arbitrary representative, so the
                // uv metric mis-ranks it. Endpoints at the same pole match
                // it by 3D coincidence instead (of-rb4).
                let d = match (chart.pole_v(p), chart.pole_v(&vp)) {
                    (Some(a), Some(b)) if a == b => (p - vp).norm(),
                    _ => (((vuv.0 - uv.0) * u_scale).powi(2) + ((vuv.1 - uv.1) * v_scale).powi(2))
                        .sqrt(),
                };
                (k, d)
            })
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .expect("cycle has darts")
    };
    // A chain whose two ends coincide in 3D is a closed loop; anchored at
    // a single boundary vertex it may legitimately match the same vertex
    // twice (an imprint network closing at a pole, of-rb4).
    let closed_chain = (s_p - e_p).norm() <= eps;
    let mut cands: Vec<(f64, usize, usize)> = Vec::new();
    for &su in &shifts_u {
        for &sv in &shifts_v {
            let (i, di) = nearest((s_uv.0 + su, s_uv.1 + sv), &s_p);
            let (j, dj) = nearest((e_uv.0 + su, e_uv.1 + sv), &e_p);
            if di <= eps
                && dj <= eps
                && (i != j || closed_chain)
                && !cands.iter().any(|&(_, a, b)| (a, b) == (i, j))
            {
                cands.push((di + dj, i, j));
            }
        }
    }
    cands.sort_by(|a, b| a.0.total_cmp(&b.0));
    cands.into_iter().map(|(_, i, j)| (i, j)).collect()
}

fn cyclic_slice(darts: &[(usize, bool)], from: usize, to: usize) -> DartChain {
    if from < to {
        darts[from..to].to_vec()
    } else {
        darts[from..].iter().chain(&darts[..to]).copied().collect()
    }
}

/// Shift an angle-like probe by whole periods so each periodic coordinate
/// lies within half a period of `poly`'s midrange on that axis (a no-op on
/// axes without a period, so the `u`-only cylinder/sphere case shifts only
/// `u` and the doubly-periodic torus shifts both). Each cycle of a face is
/// mean-aligned into the cover window independently, so two cycles hugging
/// opposite cover edges can sit a whole period apart even though they
/// overlap on the surface.
fn localize_to_window(
    poly: &[CoverPoint],
    p: (f64, f64),
    period: (Option<f64>, Option<f64>),
) -> (f64, f64) {
    let (period_u, period_v) = period;
    if period_u.is_none() && period_v.is_none() {
        return p;
    }
    let (mut lo_u, mut hi_u) = (f64::INFINITY, f64::NEG_INFINITY);
    let (mut lo_v, mut hi_v) = (f64::INFINITY, f64::NEG_INFINITY);
    for ((u, v), _) in poly {
        lo_u = lo_u.min(*u);
        hi_u = hi_u.max(*u);
        lo_v = lo_v.min(*v);
        hi_v = hi_v.max(*v);
    }
    let u = match period_u {
        Some(per) => shift_into_window(p.0, 0.5 * (lo_u + hi_u), per),
        None => p.0,
    };
    let v = match period_v {
        Some(per) => shift_into_window(p.1, 0.5 * (lo_v + hi_v), per),
        None => p.1,
    };
    (u, v)
}

/// Even-odd containment of a probe (already in the face cover) in a region.
fn region_contains(region: &Region, p: (f64, f64)) -> bool {
    let mut inside = false;
    for cy in &region.cycles {
        let n = cy.poly.len();
        for i in 0..n {
            let (a, _) = cy.poly[i];
            let (b, _) = cy.poly[(i + 1) % n];
            if crosses_upward(a, b, p) {
                inside = !inside;
            }
        }
    }
    inside
}

/// Apply one imprint chain to the current region set: split a region's
/// outer cycle when the chain is a boundary-to-boundary chord, or insert a
/// hole + disk when it closes on itself in the interior.
fn apply_chain(
    face_poly: &FaceRegionPoly,
    atoms: &[Atom],
    regions: &mut Vec<Region>,
    chain: DartChain,
    snap: f64,
) -> CoreResult<()> {
    let (chain_poly, _) = embed_walk(face_poly, atoms, &chain, true);
    let period = (face_poly.chart.period_u(), face_poly.chart.period_v());
    // Arc-length metric evaluated at the chain's mean minor angle / latitude
    // — the axis scales only vary with `v` on sphere/torus charts, and this
    // band is narrow enough for a representative `v` to hold.
    let rep_v = if chain_poly.is_empty() {
        0.0
    } else {
        chain_poly.iter().map(|((_, v), _)| v).sum::<f64>() / chain_poly.len() as f64
    };
    let scale = face_poly.chart.uv_scale(rep_v);
    // Locate the host region and split vertices. Endpoint proximity alone
    // cannot decide either — a seam chord's endpoints coincide in 3D with
    // vertex copies on EVERY region bordering the seam at those points,
    // and on the two cover copies within one region (score ties at zero,
    // of-43n); pole-anchored chords likewise tie on the pole vertex of
    // every region touching that pole (of-rb4). Splitting a CCW outer
    // cycle with a transversal chord of its region yields two CCW pieces,
    // so the first (region, vertex-pair) whose split comes out both-CCW
    // is the geometric host; a wrong copy pair or a neighboring region
    // slices a boundary complement instead and winds one piece CW. Falls
    // back to the best-scoring match if no split is both-CCW (preserving
    // the pre-of-43n behavior).
    let split_at = |ri: usize, (vi, vj): (usize, usize)| {
        let outer = &regions[ri].cycles[0];
        if vi == vj {
            // The chain is a closed loop anchored at a single boundary
            // vertex (an imprint network closing at a sphere pole,
            // of-rb4). One side is the loop alone; the other is the
            // outer cycle with the reversed loop spliced in at the
            // shared vertex — one pinched cycle, NOT an outer + hole
            // pair, whose vertex-touching ring would over-count R and
            // break the shell's Euler characteristic.
            let as_given = embed_cycle(face_poly, atoms, chain.clone());
            let (loop_darts, loop_cycle) = if as_given.area >= 0.0 {
                (chain.clone(), as_given)
            } else {
                let rev = reverse_chain(&chain);
                let cy = embed_cycle(face_poly, atoms, rev.clone());
                (rev, cy)
            };
            let mut pinched = reverse_chain(&loop_darts);
            pinched.extend(cyclic_slice(&outer.darts, vi, vi));
            return (loop_cycle, embed_cycle(face_poly, atoms, pinched));
        }
        let mut darts_one = chain.clone();
        darts_one.extend(cyclic_slice(&outer.darts, vj, vi));
        let mut darts_two = reverse_chain(&chain);
        darts_two.extend(cyclic_slice(&outer.darts, vi, vj));
        (
            embed_cycle(face_poly, atoms, darts_one),
            embed_cycle(face_poly, atoms, darts_two),
        )
    };
    let mut fallback: Option<(usize, (usize, usize))> = None;
    let mut chosen: Option<(usize, Cycle, Cycle)> = None;
    'regions: for (ri, region) in regions.iter().enumerate() {
        for (ci, cycle) in region.cycles.iter().enumerate() {
            let candidates =
                match_chain_to_cycle(cycle, &chain_poly, period, scale, &face_poly.chart);
            if candidates.is_empty() {
                continue;
            }
            if ci != 0 {
                return Err(CoreError::NotImplemented {
                    feature: "boolean imprints chording a hole boundary (transversal MVP)",
                });
            }
            for &pair in &candidates {
                let (one, two) = split_at(ri, pair);
                if one.area > 0.0 && two.area > 0.0 {
                    chosen = Some((ri, one, two));
                    break 'regions;
                }
            }
            if fallback.is_none() {
                fallback = Some((ri, candidates[0]));
            }
        }
    }
    if chosen.is_none() {
        if let Some((ri, pair)) = fallback {
            let (one, two) = split_at(ri, pair);
            chosen = Some((ri, one, two));
        }
    }
    if let Some((ri, cycle_one, cycle_two)) = chosen {
        let holes: Vec<Cycle> = regions[ri].cycles[1..].to_vec();
        let mut region_one = Region {
            cycles: vec![cycle_one],
        };
        let mut region_two = Region {
            cycles: vec![cycle_two],
        };
        for mut hole in holes {
            let probe = hole.poly[0].0;
            let one = localize_to_window(&region_one.cycles[0].poly, probe, period);
            let in_one = point_in_cycle(&region_one.cycles[0], one);
            let localized = if in_one {
                one
            } else {
                localize_to_window(&region_two.cycles[0].poly, probe, period)
            };
            // Keep the hole in its host's cover window so later
            // even-odd tests and tessellation see the outer cycle and
            // its holes as one polygon set.
            let shift = localized.0 - probe.0;
            if shift != 0.0 {
                for ((u, _), _) in hole.poly.iter_mut() {
                    *u += shift;
                }
            }
            if in_one {
                region_one.cycles.push(hole);
            } else {
                region_two.cycles.push(hole);
            }
        }
        regions[ri] = region_one;
        regions.push(region_two);
        return Ok(());
    }

    // No boundary match: the chain must close on itself (an interior ring).
    // A single closed atom is a ring by construction — its polyline closes
    // implicitly, so its first and last SAMPLES sit a full sample step
    // apart and must not be held to the snap-coincidence test.
    let is_ring_atom = chain.len() == 1 && atoms[chain[0].0].closed;
    let start = chain_poly[0].1;
    let end = chain_poly[chain_poly.len() - 1].1;
    if !is_ring_atom && (start - end).norm() > snap * 100.0 {
        return Err(CoreError::Degenerate {
            context: "boolean::imprint",
            reason: "an imprint chain ends in a face interior without closing \
                     or reaching the face boundary"
                .into(),
        });
    }
    let ring = embed_cycle(face_poly, atoms, chain);
    let probe = ring.poly[0].0;
    let host = regions
        .iter()
        .position(|r| region_contains(r, localize_to_window(&r.cycles[0].poly, probe, period)))
        .ok_or_else(|| CoreError::Degenerate {
            context: "boolean::imprint",
            reason: "an interior imprint ring lies in no region of its host face".into(),
        })?;
    let shift = localize_to_window(&regions[host].cycles[0].poly, probe, period).0 - probe.0;
    let (disk, mut hole) = if ring.area >= 0.0 {
        let hole = embed_cycle(face_poly, atoms, reverse_chain(&ring.darts));
        (ring, hole)
    } else {
        let disk = embed_cycle(face_poly, atoms, reverse_chain(&ring.darts));
        (disk, ring)
    };
    // The hole joins the host's cycle set: move it into the host's cover
    // window (the disk keeps its own window as a standalone region).
    if shift != 0.0 {
        for ((u, _), _) in hole.poly.iter_mut() {
            *u += shift;
        }
    }
    regions[host].cycles.push(hole);
    regions.push(Region { cycles: vec![disk] });
    Ok(())
}

fn point_in_cycle(cycle: &Cycle, p: (f64, f64)) -> bool {
    let n = cycle.poly.len();
    let mut inside = false;
    for i in 0..n {
        let (a, _) = cycle.poly[i];
        let (b, _) = cycle.poly[(i + 1) % n];
        if crosses_upward(a, b, p) {
            inside = !inside;
        }
    }
    inside
}

/// A robust interior point of a region, in 3D.
fn region_interior_point(chart: &Chart, region: &Region) -> Option<Point3> {
    let outer = &region.cycles[0];
    let n = outer.poly.len();
    let inside_region = |p: (f64, f64)| -> bool {
        let mut inside = false;
        for cy in &region.cycles {
            let m = cy.poly.len();
            for i in 0..m {
                let (a, _) = cy.poly[i];
                let (b, _) = cy.poly[(i + 1) % m];
                if crosses_upward(a, b, p) {
                    inside = !inside;
                }
            }
        }
        inside
    };
    // Param-space extent for offset scaling.
    let (mut lo, mut hi) = (
        (f64::INFINITY, f64::INFINITY),
        (f64::NEG_INFINITY, f64::NEG_INFINITY),
    );
    for (uv, _) in &outer.poly {
        lo = (lo.0.min(uv.0), lo.1.min(uv.1));
        hi = (hi.0.max(uv.0), hi.1.max(uv.1));
    }
    // Work in coordinates normalized by the region's own per-axis extents:
    // on cylinder charts `u` is radians while `v` is model units, so an
    // isotropic offset scaled by the summed extents cannot adapt to sliver
    // regions that are thin along only one axis.
    let du = (hi.0 - lo.0).max(1e-12);
    let dv = (hi.1 - lo.1).max(1e-12);
    // Coarse offsets first (bolder samples classify more robustly); the
    // sub-1e-3 tail reaches regions that are thin even diagonally.
    for scale in [5e-2, 1e-2, 1e-3, 1e-4, 1e-5, 1e-6, 1e-7] {
        for i in 0..n {
            let (a, _) = outer.poly[i];
            let (b, _) = outer.poly[(i + 1) % n];
            let (dx, dy) = ((b.0 - a.0) / du, (b.1 - a.1) / dv);
            let len = (dx * dx + dy * dy).sqrt();
            if len < 1e-9 {
                continue;
            }
            // Left normal of a CCW boundary points into the region
            // (orientation survives the positive per-axis scaling).
            let (nx, ny) = (-dy / len, dx / len);
            let mid = (
                (a.0 + b.0) * 0.5 + nx * scale * du,
                (a.1 + b.1) * 0.5 + ny * scale * dv,
            );
            if inside_region(mid) {
                return Some(chart_point(chart, mid));
            }
        }
    }
    None
}

fn chart_point(chart: &Chart, uv: (f64, f64)) -> Point3 {
    match chart {
        Chart::Plane {
            origin, e_u, e_v, ..
        } => origin + e_u * uv.0 + e_v * uv.1,
        Chart::Cylinder {
            origin,
            axis,
            e_u,
            e_v,
            radius,
        } => {
            let radial = e_u * uv.0.cos() + e_v * uv.0.sin();
            origin + radial * *radius + axis * uv.1
        }
        Chart::Sphere {
            center,
            axis,
            e_u,
            e_v,
            radius,
        } => {
            let radial = e_u * uv.0.cos() + e_v * uv.0.sin();
            center + (radial * uv.1.cos() + axis * uv.1.sin()) * *radius
        }
        Chart::Torus {
            center,
            axis,
            e_u,
            e_v,
            major_radius,
            minor_radius,
        } => {
            let radial = e_u * uv.0.cos() + e_v * uv.0.sin();
            center
                + radial * (major_radius + minor_radius * uv.1.cos())
                + axis * (minor_radius * uv.1.sin())
        }
    }
}

/// Analytic ray-surface intersection parameters (unbounded surface).
///
/// The torus branch is not closed-form: its quartic is sign-sampled and
/// bisected instead (see [`ray_torus_hits`]), which is sound for the ray
/// PARITY classification this feeds — a bracket hiding an even root pair
/// (a graze) flips no sign, and skipping a root pair changes no parity;
/// genuinely tangent hits are already rejected by the caller's grazing
/// check and retried along another direction.
fn ray_surface_hits(surface: &Surface3, p: &Point3, dir: &Vector3) -> Vec<f64> {
    match surface {
        Surface3::Plane { origin, normal } => {
            let denom = normal.dot(dir);
            if denom.abs() < 1e-12 {
                return Vec::new();
            }
            vec![normal.dot(&(origin - p)) / denom]
        }
        Surface3::Cylinder {
            origin,
            axis,
            radius,
        } => {
            // |(p + t d - o) perp axis|² = r².
            let oc = p - origin;
            let d_perp = dir - axis * dir.dot(axis);
            let o_perp = oc - axis * oc.dot(axis);
            let a = d_perp.norm_squared();
            let b = 2.0 * o_perp.dot(&d_perp);
            let c = o_perp.norm_squared() - radius * radius;
            if a < 1e-15 {
                return Vec::new();
            }
            let disc = b * b - 4.0 * a * c;
            if disc <= 0.0 {
                return Vec::new();
            }
            let sq = disc.sqrt();
            vec![(-b - sq) / (2.0 * a), (-b + sq) / (2.0 * a)]
        }
        Surface3::Sphere { center, radius, .. } => {
            // |p + t d - c|² = r².
            let oc = p - center;
            let a = dir.norm_squared();
            let b = 2.0 * oc.dot(dir);
            let c = oc.norm_squared() - radius * radius;
            let disc = b * b - 4.0 * a * c;
            if disc <= 0.0 {
                return Vec::new();
            }
            let sq = disc.sqrt();
            vec![(-b - sq) / (2.0 * a), (-b + sq) / (2.0 * a)]
        }
        Surface3::Torus {
            center,
            axis,
            major_radius,
            minor_radius,
        } => ray_torus_hits(center, axis, *major_radius, *minor_radius, p, dir),
        Surface3::Cone { .. } => Vec::new(),
    }
}

/// Samples along the ray window that can intersect the torus's bounding
/// sphere. 4·SAMPLES_PER_CIRCLE resolves every sign change of the degree-4
/// implicit residual whose odd-multiplicity roots are further apart than
/// ~1/384 of the window; closer pairs are even-parity brackets that the
/// parity classification may soundly skip (see [`ray_surface_hits`]).
const RAY_TORUS_SAMPLES: usize = 4 * SAMPLES_PER_CIRCLE;

/// Ray-torus intersection parameters by sign-sampled bisection of the
/// torus's implicit residual `(|q_perp| - R)² + q_z² - r²` along the ray,
/// restricted to the window where the ray is inside the torus's bounding
/// sphere (radius `R + r` about its center — outside it the residual is
/// strictly positive).
fn ray_torus_hits(
    center: &Point3,
    axis: &Vector3,
    major_radius: f64,
    minor_radius: f64,
    p: &Point3,
    dir: &Vector3,
) -> Vec<f64> {
    let residual = |t: f64| {
        let q = p + dir * t - center;
        let z = q.dot(axis);
        let w = (q - axis * z).norm() - major_radius;
        w * w + z * z - minor_radius * minor_radius
    };
    // Bounding-sphere window (dilated so window-edge roots stay interior).
    let bound = (major_radius + minor_radius) * 1.001;
    let oc = p - center;
    let a = dir.norm_squared();
    let b = 2.0 * oc.dot(dir);
    let c = oc.norm_squared() - bound * bound;
    let disc = b * b - 4.0 * a * c;
    if disc <= 0.0 {
        return Vec::new();
    }
    let sq = disc.sqrt();
    let (t_lo, t_hi) = ((-b - sq) / (2.0 * a), (-b + sq) / (2.0 * a));
    let mut hits = Vec::new();
    let step = (t_hi - t_lo) / RAY_TORUS_SAMPLES as f64;
    let mut prev_t = t_lo;
    let mut prev_f = residual(prev_t);
    for i in 1..=RAY_TORUS_SAMPLES {
        let t = t_lo + step * i as f64;
        let f = residual(t);
        if (prev_f <= 0.0) != (f <= 0.0) {
            let inside = |t: f64| (residual(t) <= 0.0) == (f <= 0.0);
            hits.push(refine_crossing(&inside, prev_t, t));
        }
        prev_t = t;
        prev_f = f;
    }
    hits
}

// ---------------------------------------------------------------------
// Output assembly
// ---------------------------------------------------------------------

/// Per-face tessellation payload.
struct MeshFace {
    chart: Chart,
    /// rings[0] outer, rest holes; param + 3D per vertex, in the *original*
    /// (pre-reversal) face orientation.
    rings: Vec<MeshRing>,
    /// Outward normal sign relative to the chart normal.
    normal_sign: f64,
}

struct MeshRing {
    uv: Vec<(f64, f64)>,
    points: Vec<Point3>,
}

/// The exact source curve an atom was sampled from.
fn source_curve<'p>(pipe: &'p Pipeline<'_>, source: CurveSource) -> &'p Curve3 {
    match source {
        CurveSource::Edge { solid, edge } => &pipe.solids[solid].edges[edge].curve,
        CurveSource::Imprint { index } => &pipe.imprints[index].curve,
    }
}

fn build_output(
    pipe: &Pipeline<'_>,
    _op: BooleanOp,
    atoms: &[Atom],
    atom_source: &[CurveSource],
    kept: Vec<KeptRegion>,
) -> CoreResult<BooleanOutput> {
    let snap = pipe.snap;
    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let body = store.create_body(BodyType::Solid);

    // Shell partition: union-find over kept regions via shared atoms.
    let mut parent: Vec<usize> = (0..kept.len()).collect();
    fn find(parent: &mut Vec<usize>, i: usize) -> usize {
        if parent[i] != i {
            let r = find(parent, parent[i]);
            parent[i] = r;
        }
        parent[i]
    }
    let mut atom_user: HashMap<usize, usize> = HashMap::new();
    for (ri, kr) in kept.iter().enumerate() {
        for cy in &kr.region.cycles {
            for &(ai, _) in &cy.darts {
                if let Some(&other) = atom_user.get(&ai) {
                    let (a, b) = (find(&mut parent, ri), find(&mut parent, other));
                    if a != b {
                        parent[a] = b;
                    }
                } else {
                    atom_user.insert(ai, ri);
                }
            }
        }
    }
    let mut shells: HashMap<usize, EntityId<crate::topology::Shell>> = HashMap::new();

    // Vertices and edges, deduplicated by snapped 3D position / atom id.
    // Geometry is shared: one output surface id per host face (all regions
    // split from it), one output curve id per source curve.
    let mut vertex_of: SnapMap<EntityId<crate::topology::Vertex>> = SnapMap::new(snap * 4.0);
    let mut edge_of_atom: HashMap<usize, EntityId<crate::topology::Edge>> = HashMap::new();
    let mut surface_of_host: HashMap<(SolidTag, usize), EntityId<Surface3>> = HashMap::new();
    let mut curve_of_source: HashMap<CurveSource, EntityId<Curve3>> = HashMap::new();

    let mut mesh_faces = Vec::new();
    let mut face_count = 0usize;

    for (ri, kr) in kept.iter().enumerate() {
        let root = find(&mut parent, ri);
        let shell = *shells
            .entry(root)
            .or_insert_with(|| store.create_shell(body, true, ShellOrientation::Outward));
        let face_data = &pipe.solids[kr.solid].faces[kr.face];
        let outward_along_surface = face_data.outward_along_normal != kr.reverse;
        let face_id = store.create_face(
            shell,
            if outward_along_surface {
                FaceSense::Positive
            } else {
                FaceSense::Negative
            },
        );
        let surface_id = *surface_of_host
            .entry((kr.solid, kr.face))
            .or_insert_with(|| geo.add_surface(face_data.surface.clone()));
        store.faces.get_mut(face_id).expect("just created").surface = Some(surface_id);
        face_count += 1;

        for (ci, cycle) in kr.region.cycles.iter().enumerate() {
            let loop_type = if ci == 0 {
                LoopType::Outer
            } else {
                LoopType::Inner
            };
            let mut entries: Vec<(EntityId<crate::topology::Edge>, FinSense)> = Vec::new();
            // `reconstruct` traced cycles CCW-in-chart, reversing the walk for
            // faces whose outward side opposes the surface normal. Undo that
            // extra reversal here (XOR with the flip) so the emitted loop winds
            // CCW as seen from the output face's outward side. `kr.reverse`
            // additionally flips the winding when this region's face is
            // oriented opposite its source (e.g. the tool in a subtract).
            // (XOR with the flip `!outward_along_normal`; `a != !b == a == b`.)
            let output_reverse = kr.reverse == face_data.outward_along_normal;
            let darts: Vec<(usize, bool)> = if output_reverse {
                cycle
                    .darts
                    .iter()
                    .rev()
                    .map(|&(a, fwd)| (a, !fwd))
                    .collect()
            } else {
                cycle.darts.clone()
            };
            for (ai, forward) in darts {
                let atom = &atoms[ai];
                let edge_id = match edge_of_atom.get(&ai) {
                    Some(&edge_id) => edge_id,
                    None => {
                        let start_p = atom.points[0];
                        let end_p = if atom.closed {
                            start_p
                        } else {
                            atom.points[atom.points.len() - 1]
                        };
                        let sv = vertex_of.nearest(&start_p).unwrap_or_else(|| {
                            let v = store.create_vertex(start_p, SYSTEM_RESOLUTION);
                            vertex_of.insert(start_p, v);
                            v
                        });
                        let ev = vertex_of.nearest(&end_p).unwrap_or_else(|| {
                            let v = store.create_vertex(end_p, SYSTEM_RESOLUTION);
                            vertex_of.insert(end_p, v);
                            v
                        });

                        // Bind the atom's source curve with the parameter
                        // range recovered by exact closest-point projection.
                        // Atom polylines run in increasing curve parameter,
                        // so a wrapped range on a periodic curve unwraps
                        // forward by one period.
                        let source = atom_source[ai];
                        let curve = source_curve(pipe, source);
                        let curve_id = *curve_of_source
                            .entry(source)
                            .or_insert_with(|| geo.add_curve(curve.clone()));
                        let t_first = curve.project_point(&start_p).t;
                        let (t_start, t_end) = if atom.closed {
                            let period = curve
                                .period()
                                .expect("closed atoms come from periodic curves");
                            (t_first, t_first + period)
                        } else {
                            let mut t_last = curve.project_point(&end_p).t;
                            if let Some(period) = curve.period() {
                                if t_last <= t_first {
                                    t_last += period;
                                }
                            }
                            (t_first, t_last)
                        };

                        // Tolerant modeling: vertices come from sampled
                        // polylines (imprint refinement, seam interpolation),
                        // so record the true curve-endpoint-to-vertex gap.
                        let d_start =
                            (curve.point(t_start) - store.vertex(sv).expect("live").point).norm();
                        let d_end =
                            (curve.point(t_end) - store.vertex(ev).expect("live").point).norm();
                        let tolerance = SYSTEM_RESOLUTION.max(d_start).max(d_end);
                        let edge_id = store
                            .create_edge_with_curve(sv, ev, tolerance, curve_id, t_start, t_end);
                        for (v, d) in [(sv, d_start), (ev, d_end)] {
                            let vertex = store.vertices.get_mut(v).expect("live");
                            vertex.tolerance = vertex.tolerance.max(d);
                        }
                        edge_of_atom.insert(ai, edge_id);
                        edge_id
                    }
                };
                entries.push((
                    edge_id,
                    if forward {
                        FinSense::Forward
                    } else {
                        FinSense::Reversed
                    },
                ));
            }
            store.create_loop(face_id, loop_type, &entries);
        }

        // Tessellation payload. Interior samples of straight (Line-sourced)
        // open darts are dropped: they are pure oversampling (a straight
        // segment needs only its endpoints), and the collinear runs they
        // form break ear clipping — when the SAME run lies on two adjacent
        // faces, both ear clippers skip a collinear midpoint with a chord
        // and emit the compensating zero-area sliver, putting FOUR
        // triangles on the chord edge (of-ny6). Both faces share the
        // atom's polyline, so they drop identical points and rim welding
        // is preserved; dart start points (the true topology vertices)
        // always survive.
        let fp = &pipe.face_polys[kr.solid][kr.face];
        let rings = kr
            .region
            .cycles
            .iter()
            .map(|cy| {
                let n = cy.poly.len();
                let dart_count = cy.dart_offsets.len();
                let mut keep = vec![true; n];
                for (k, &(ai, _)) in cy.darts.iter().enumerate() {
                    let straight =
                        matches!(source_curve(pipe, atom_source[ai]), Curve3::Line { .. });
                    if !straight || atoms[ai].closed {
                        continue;
                    }
                    let start = cy.dart_offsets[k];
                    let end = if k + 1 < dart_count {
                        cy.dart_offsets[k + 1]
                    } else {
                        n
                    };
                    for flag in &mut keep[start + 1..end] {
                        *flag = false;
                    }
                }
                let mut uv = Vec::with_capacity(n);
                let mut points = Vec::with_capacity(n);
                for ((q, p), _) in cy.poly.iter().zip(&keep).filter(|&(_, &k)| k) {
                    uv.push(*q);
                    points.push(*p);
                }
                MeshRing { uv, points }
            })
            .collect();
        let base_sign = if face_data.outward_along_normal {
            1.0
        } else {
            -1.0
        };
        mesh_faces.push(MeshFace {
            chart: fp.chart.clone(),
            rings,
            normal_sign: if kr.reverse { -base_sign } else { base_sign },
        });
    }

    // Genus per shell from the Euler-Poincaré formula (S = 1 per shell).
    //
    // The genus stored here is *derived* from the reconstructed graph's own
    // V/E/F/R counts, so the later `check()` re-derivation of the same formula
    // holds by construction — it corroborates the counts, it does not
    // independently validate them. What DOES carry signal is the shape of chi
    // itself: a valid single-component closed orientable shell always has an
    // even chi in `..=2` (chi = 2 - 2g, g >= 0). An odd chi means the shell is
    // not closed/orientable; chi > 2 means it is not a single component. Either
    // way the reconstruction is broken, so we reject it at build time rather
    // than silently labelling it genus 0 and deferring the failure to a
    // `check()` call that the caller may never make.
    let shell_ids: Vec<EntityId<crate::topology::Shell>> = shells.values().copied().collect();
    for shell_id in &shell_ids {
        let (v, e, f, r) = shell_counts(&store, *shell_id);
        let chi = v as i64 - e as i64 + f as i64 - r as i64;
        let genus = shell_genus_from_euler(chi).ok_or_else(|| CoreError::Degenerate {
            context: "boolean::build_output",
            reason: format!(
                "reconstructed shell has impossible Euler characteristic \
                 chi = V-E+F-R = {v}-{e}+{f}-{r} = {chi}; a valid closed \
                 orientable shell requires an even chi <= 2"
            ),
        })?;
        store
            .shells
            .get_mut(*shell_id)
            .expect("shell just created")
            .genus = genus;
    }

    let shell_count = shells.len();
    Ok(BooleanOutput {
        store,
        geo,
        body,
        mesh_faces,
        face_count,
        shell_count,
    })
}

/// Genus of a single closed orientable shell from its Euler characteristic
/// `chi = V - E + F - R`, or `None` when `chi` is impossible for such a shell.
///
/// For one connected closed orientable surface `chi = 2 - 2g` with `g >= 0`,
/// so `chi` is always even and at most 2, and `g = 1 - chi/2`. An odd `chi`
/// (shell not closed/orientable) or `chi > 2` (more than one component fused
/// into one shell, giving `g < 0`) has no valid genus and yields `None`.
fn shell_genus_from_euler(chi: i64) -> Option<u32> {
    if chi % 2 != 0 {
        return None;
    }
    // V - E + F - R = 2(1 - H)  =>  H = 1 - chi/2.
    let h = 1 - chi / 2;
    (h >= 0).then_some(h as u32)
}

/// V/E/F/R counts of one shell (vertices and edges reached via its faces).
fn shell_counts(
    store: &TopologyStore,
    shell: EntityId<crate::topology::Shell>,
) -> (usize, usize, usize, usize) {
    use std::collections::HashSet;
    let mut vs: HashSet<EntityId<crate::topology::Vertex>> = HashSet::new();
    let mut es: HashSet<EntityId<crate::topology::Edge>> = HashSet::new();
    let faces = store.faces_of_shell(shell).to_vec();
    let mut rings = 0usize;
    for &f in &faces {
        let loops = store.loops_of_face(f);
        rings += loops.len().saturating_sub(1);
        for lp in loops {
            for &fin in store.fins_of_loop(lp) {
                let fin_data = store.fin(fin).expect("live fin");
                let edge = store.edge(fin_data.edge).expect("live edge");
                es.insert(fin_data.edge);
                vs.insert(edge.start_vertex);
                vs.insert(edge.end_vertex);
            }
        }
    }
    (vs.len(), es.len(), faces.len(), rings)
}

// ---------------------------------------------------------------------
// Tessellation (ear clipping with hole bridging)
// ---------------------------------------------------------------------

/// Triangulate one face into 3D triangles. The second return value is the
/// face's worst chordal deviation: the largest distance between a triangle
/// edge's 3D midpoint and the surface point at its parameter-space
/// midpoint. Zero for planes (chords are exact); on curved charts it
/// exposes how badly wide ear-clip chords cut through the surface (a
/// trimmed full-wrap cylinder band is triangulated from its boundary
/// samples alone, so chords can stray by up to the radius), letting
/// callers judge the mesh instead of trusting it blindly.
fn triangulate_mesh_face(mf: &MeshFace, weld_eps: f64) -> CoreResult<(Vec<Triangle>, f64)> {
    // Ring polylines are concatenated dart chains, so chain joins (and the
    // ring closure) carry consecutive duplicate vertices. A duplicate is
    // uv-coincident with its neighbor but has its own index, so the ear
    // test — which skips coincident points by index only — sees it as a
    // blocking vertex on every ear at that corner, starving ear clipping
    // into the degenerate-corner fallback. Drop them up front.
    let dedupe = |uv: &[(f64, f64)], points: &[Point3]| -> (Vec<(f64, f64)>, Vec<Point3>) {
        let extent = uv
            .iter()
            .map(|p| p.0.abs().max(p.1.abs()))
            .fold(0.0, f64::max);
        let eps2 = (1e-12 * (1.0 + extent)).powi(2);
        let mut out_uv: Vec<(f64, f64)> = Vec::with_capacity(uv.len());
        let mut out_p: Vec<Point3> = Vec::with_capacity(points.len());
        for (q, p) in uv.iter().zip(points) {
            if out_uv.last().is_none_or(|last| dist2(*last, *q) > eps2) {
                out_uv.push(*q);
                out_p.push(*p);
            }
        }
        while out_uv.len() > 1 && dist2(out_uv[0], *out_uv.last().expect("non-empty")) <= eps2 {
            out_uv.pop();
            out_p.pop();
        }
        (out_uv, out_p)
    };

    // Combine outer ring and holes into a single polygon via bridges.
    let (mut all_uv, mut all_p) = dedupe(&mf.rings[0].uv, &mf.rings[0].points);
    let mut polygon: Vec<usize> = (0..all_uv.len()).collect();
    // Vertex-index range of each original (unbridged) ring in `all_uv`.
    // Ring edges are the true boundary constraints for curved-face
    // refinement: they are never subdivided, so adjacent faces weld exactly.
    let mut ring_ranges: Vec<(usize, usize)> = vec![(0, all_uv.len())];

    // Sort holes by max-u vertex, descending, and bridge each into the
    // polygon (Eberly's method, simplified with nearest-visible search).
    type HoleRing = (Vec<(f64, f64)>, Vec<Point3>);
    let mut holes: Vec<HoleRing> = mf.rings[1..]
        .iter()
        .map(|r| dedupe(&r.uv, &r.points))
        .collect();
    holes.sort_by(|a, b| {
        let ma = a.0.iter().map(|p| p.0).fold(f64::NEG_INFINITY, f64::max);
        let mb = b.0.iter().map(|p| p.0).fold(f64::NEG_INFINITY, f64::max);
        mb.total_cmp(&ma)
    });
    for hi in 0..holes.len() {
        let (huv, hp) = &holes[hi];
        let base = all_uv.len();
        all_uv.extend_from_slice(huv);
        all_p.extend_from_slice(hp);
        ring_ranges.push((base, huv.len()));
        // Hole vertex with max u.
        let (hi_local, _) = huv
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.0.total_cmp(&b.1.0))
            .expect("non-empty hole");
        let h_idx = base + hi_local;
        // Rings the bridge must not cross: this hole plus every hole not
        // yet spliced (spliced holes are already polygon segments). A
        // bridge that merely avoids the current hole can still cut
        // through a later one, and splicing that hole would then
        // self-intersect the polygon.
        let unspliced: Vec<&[(f64, f64)]> = holes[hi..].iter().map(|h| h.0.as_slice()).collect();
        // Polygon vertex to bridge to: nearest by distance whose connecting
        // segment crosses no polygon or hole boundary segment.
        let mut candidates: Vec<usize> = (0..polygon.len()).collect();
        candidates.sort_by(|&a, &b| {
            let da = dist2(all_uv[polygon[a]], all_uv[h_idx]);
            let db = dist2(all_uv[polygon[b]], all_uv[h_idx]);
            da.total_cmp(&db)
        });
        let mut bridged = false;
        for cand in candidates {
            let p_idx = polygon[cand];
            if bridge_is_clear(&all_uv, &polygon, &unspliced, all_uv[h_idx], all_uv[p_idx]) {
                // Splice: ...p, h, h+1.., h, p...
                let mut new_poly = Vec::with_capacity(polygon.len() + huv.len() + 2);
                new_poly.extend_from_slice(&polygon[..=cand]);
                let hn = huv.len();
                for k in 0..=hn {
                    new_poly.push(base + (hi_local + k) % hn);
                }
                new_poly.push(p_idx);
                new_poly.extend_from_slice(&polygon[cand + 1..]);
                polygon = new_poly;
                bridged = true;
                break;
            }
        }
        if !bridged {
            return Err(CoreError::Degenerate {
                context: "boolean::tessellate",
                reason: "could not bridge a hole into its outer boundary".into(),
            });
        }
    }

    // Ear clipping on the bridged polygon.
    let mut idx = polygon;
    let mut tris: Vec<[usize; 3]> = Vec::new();
    let mut guard = 0usize;
    while idx.len() > 3 {
        guard += 1;
        if guard > 100_000 {
            return Err(CoreError::Degenerate {
                context: "boolean::tessellate",
                reason: "ear clipping did not terminate".into(),
            });
        }
        let n = idx.len();
        let mut clipped = false;
        for i in 0..n {
            let (ia, ib, ic) = (idx[(i + n - 1) % n], idx[i], idx[(i + 1) % n]);
            let (a, b, c) = (all_uv[ia], all_uv[ib], all_uv[ic]);
            let cross = (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0);
            if cross <= 0.0 {
                continue; // reflex or degenerate corner
            }
            // No other polygon vertex strictly inside the ear.
            let mut ok = true;
            for &other in &idx {
                if other == ia || other == ib || other == ic {
                    continue;
                }
                if point_in_triangle(all_uv[other], a, b, c) {
                    ok = false;
                    break;
                }
            }
            if ok {
                tris.push([ia, ib, ic]);
                idx.remove(i);
                clipped = true;
                break;
            }
        }
        if !clipped {
            // Fallback: clip the least-reflex corner to guarantee progress
            // on nearly-degenerate polygons.
            let n = idx.len();
            let mut best = (f64::NEG_INFINITY, 0usize);
            for i in 0..n {
                let (a, b, c) = (
                    all_uv[idx[(i + n - 1) % n]],
                    all_uv[idx[i]],
                    all_uv[idx[(i + 1) % n]],
                );
                let cross = (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0);
                if cross > best.0 {
                    best = (cross, i);
                }
            }
            let i = best.1;
            let n = idx.len();
            tris.push([idx[(i + n - 1) % n], idx[i], idx[(i + 1) % n]]);
            idx.remove(i);
        }
    }
    if idx.len() == 3 {
        tris.push([idx[0], idx[1], idx[2]]);
    }

    let planar = matches!(mf.chart, Chart::Plane { .. });

    // Curved charts: ear clipping covers the parameter region exactly, but
    // a wide face (e.g. the full-wrap band left by a through-hole subtract)
    // gets triangles whose u-span is far coarser than the boundary
    // sampling, so their flat 3D chords cut deep inside the surface.
    // Bisecting the ear-clip result is unsafe on trimmed faces (split
    // midpoints land on shared boundary chords and weld non-manifold, and
    // the split order is nondeterministic). Instead lay a clean parameter
    // lattice of interior points strictly inside the region and retriangulate
    // to a constrained Delaunay mesh: interior vertices are lattice-clean and
    // never touch the boundary (so cross-face welding is untouched), boundary
    // ring edges are constraints that are never flipped, and the whole
    // construction is deterministic. See `refine_curved_region`.
    // Per-chart lattice parameters: the v-row spacing in v units, and the
    // arc-length scales that make the clearance metric isotropic in 3D.
    // Cylinder v is a model length (rows at one sampling pitch of arc);
    // sphere/torus v is an angle (rows at the angular pitch directly). The
    // u scale is each chart's widest u-circle radius — conservative for
    // clearance where the circle shrinks (sphere poles, torus bore).
    let pitch = TWO_PI / SAMPLES_PER_CIRCLE as f64;
    let lattice = match mf.chart {
        Chart::Cylinder { radius, .. } => Some((radius.abs() * pitch, (radius.abs(), 1.0))),
        Chart::Sphere { radius, .. } => Some((pitch, (radius.abs(), radius.abs()))),
        Chart::Torus {
            major_radius,
            minor_radius,
            ..
        } => Some((pitch, (major_radius + minor_radius, minor_radius))),
        Chart::Plane { .. } => None,
    };
    if let Some((pitch_v, scale)) = lattice {
        refine_curved_region(
            &mut tris,
            &mut all_uv,
            &mut all_p,
            &mf.chart,
            pitch_v,
            scale,
            &ring_ranges,
        );
    }

    // Emit 3D triangles; flip winding when the outward normal opposes the
    // chart normal (param-space CCW maps to the chart normal side).
    let mut deviation: f64 = 0.0;
    let mut out = Vec::with_capacity(tris.len());
    for t in tris {
        let (mut i0, i1, mut i2) = (t[0], t[1], t[2]);
        if mf.normal_sign < 0.0 {
            std::mem::swap(&mut i0, &mut i2);
        }
        let ps = [all_p[i0], all_p[i1], all_p[i2]];
        let normals = [
            mf.chart.normal(all_uv[i0].0, all_uv[i0].1) * mf.normal_sign,
            mf.chart.normal(all_uv[i1].0, all_uv[i1].1) * mf.normal_sign,
            mf.chart.normal(all_uv[i2].0, all_uv[i2].1) * mf.normal_sign,
        ];
        // Keep zero-area slivers (collinear boundary chains need them to
        // pair their chord edges) but drop triangles with a zero-length
        // edge: those come from duplicated bridge vertices, would weld
        // into degenerate indices, and their two remaining edges cancel
        // each other, so dropping them preserves edge pairing.
        let zero_length = (ps[1] - ps[0]).norm() <= weld_eps
            || (ps[2] - ps[1]).norm() <= weld_eps
            || (ps[0] - ps[2]).norm() <= weld_eps;
        if !zero_length {
            if !planar {
                for k in 0..3 {
                    let (i, j) = (t[k], t[(k + 1) % 3]);
                    let mid_uv = (
                        0.5 * (all_uv[i].0 + all_uv[j].0),
                        0.5 * (all_uv[i].1 + all_uv[j].1),
                    );
                    let mid_p = Point3::from((all_p[i].coords + all_p[j].coords) * 0.5);
                    deviation = deviation.max((chart_point(&mf.chart, mid_uv) - mid_p).norm());
                }
            }
            out.push(Triangle {
                positions: ps,
                normals,
            });
        }
    }
    Ok((out, deviation))
}

const NO_TRI: usize = usize::MAX;

/// Triangle mesh with per-edge adjacency, supporting point insertion and
/// constrained Lawson edge flips. All triangles are kept counter-clockwise
/// in parameter space. `adj[t][k]` is the triangle across the edge
/// `(v[k], v[(k+1)%3])`, or [`NO_TRI`] on the boundary.
struct FlipMesh {
    tris: Vec<[usize; 3]>,
    adj: Vec<[usize; 3]>,
}

fn orient2d(a: (f64, f64), b: (f64, f64), c: (f64, f64)) -> f64 {
    (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0)
}

/// True iff `d` is strictly and *reliably* inside the circumcircle of the
/// CCW triangle `(a, b, c)`. Used to decide Delaunay edge flips.
///
/// The raw incircle determinant catastrophically cancels on the
/// extreme-aspect quads that curved-face refinement produces (a hair-thin
/// seed sliver against a normal neighbour): the sign is then float noise,
/// and `make_delaunay` cycles A→B→A on such an edge until it hits its sweep
/// cap. This is Shewchuk's a-priori filter: compute the determinant with an
/// error bound on its own round-off, and only trust the sign when the
/// magnitude clears the bound. When it doesn't — near-cocircular, the noisy
/// case — report "not inside" so the edge is left unflipped. That keeps the
/// mesh valid (a hair off Delaunay at worst) and, crucially, makes the flip
/// loop monotone instead of cyclic.
fn in_circle(a: (f64, f64), b: (f64, f64), c: (f64, f64), d: (f64, f64)) -> bool {
    let (adx, ady) = (a.0 - d.0, a.1 - d.1);
    let (bdx, bdy) = (b.0 - d.0, b.1 - d.1);
    let (cdx, cdy) = (c.0 - d.0, c.1 - d.1);

    let bdxcdy = bdx * cdy;
    let cdxbdy = cdx * bdy;
    let cdxady = cdx * ady;
    let adxcdy = adx * cdy;
    let adxbdy = adx * bdy;
    let bdxady = bdx * ady;

    let alift = adx * adx + ady * ady;
    let blift = bdx * bdx + bdy * bdy;
    let clift = cdx * cdx + cdy * cdy;

    let det = alift * (bdxcdy - cdxbdy) + blift * (cdxady - adxcdy) + clift * (adxbdy - bdxady);

    // Shewchuk's incircle A-permanent: an upper bound on the accumulated
    // round-off in `det`, scaled by machine epsilon. Sign is trustworthy iff
    // |det| exceeds this.
    let permanent = (bdxcdy.abs() + cdxbdy.abs()) * alift
        + (cdxady.abs() + adxcdy.abs()) * blift
        + (adxbdy.abs() + bdxady.abs()) * clift;
    let errbound = ICC_ERRBOUND_A * permanent;
    if det.abs() > errbound {
        return det > 0.0;
    }
    // Uncertain / near-cocircular (e.g. lattice squares): never flip.
    false
}

/// A-priori round-off bound coefficient for [`in_circle`] (Shewchuk):
/// `(10 + 96·ε)·ε`.
const ICC_ERRBOUND_A: f64 = (10.0 + 96.0 * f64::EPSILON) * f64::EPSILON;

impl FlipMesh {
    /// Build from an existing triangulation (indices into a shared vertex
    /// list), orienting every triangle CCW. Interior edges shared by exactly
    /// two triangles are linked; any edge seen a third time is left as a
    /// boundary edge on the later triangles (conservative: it is simply never
    /// flipped).
    fn from_tris(tris: &[[usize; 3]], verts: &[(f64, f64)]) -> Self {
        let mut t = Vec::with_capacity(tris.len());
        for tri in tris {
            let mut v = *tri;
            if orient2d(verts[v[0]], verts[v[1]], verts[v[2]]) < 0.0 {
                v.swap(1, 2);
            }
            t.push(v);
        }
        let mut adj = vec![[NO_TRI; 3]; t.len()];
        // key -> (tri, edge, times seen)
        let mut edges: HashMap<(usize, usize), (usize, usize, u8)> = HashMap::new();
        for (ti, tri) in t.iter().enumerate() {
            for k in 0..3 {
                let (a, b) = (tri[k], tri[(k + 1) % 3]);
                let key = (a.min(b), a.max(b));
                match edges.get_mut(&key) {
                    None => {
                        edges.insert(key, (ti, k, 1));
                    }
                    Some(entry) if entry.2 == 1 => {
                        adj[ti][k] = entry.0;
                        adj[entry.0][entry.1] = ti;
                        entry.2 = 2;
                    }
                    Some(_) => {}
                }
            }
        }
        FlipMesh { tris: t, adj }
    }

    /// Edge index `k` of triangle `t` whose undirected endpoints are `{a, b}`.
    fn edge_index(&self, t: usize, a: usize, b: usize) -> Option<usize> {
        let tri = &self.tris[t];
        (0..3).find(|&k| {
            let (x, y) = (tri[k], tri[(k + 1) % 3]);
            (x == a && y == b) || (x == b && y == a)
        })
    }

    /// Point the neighbor `n`'s adjacency across edge `{a, b}` at `new_t`.
    fn relink(&mut self, n: usize, a: usize, b: usize, new_t: usize) {
        if n == NO_TRI {
            return;
        }
        if let Some(k) = self.edge_index(n, a, b) {
            self.adj[n][k] = new_t;
        }
    }

    /// Insert vertex `np` (index into the shared list), known to lie strictly
    /// inside triangle `t0`. Splits `t0` into three and legalizes.
    fn insert_in_triangle(
        &mut self,
        t0: usize,
        np: usize,
        verts: &[(f64, f64)],
        constraints: &std::collections::HashSet<(usize, usize)>,
    ) {
        let [a, b, c] = self.tris[t0];
        let [na, nb, nc] = self.adj[t0]; // across (a,b),(b,c),(c,a)
        let t1 = self.tris.len();
        let t2 = t1 + 1;
        self.tris[t0] = [a, b, np];
        self.adj[t0] = [na, t1, t2];
        self.tris.push([b, c, np]);
        self.adj.push([nb, t2, t0]);
        self.tris.push([c, a, np]);
        self.adj.push([nc, t0, t1]);
        // na still borders edge (a,b) on t0 — no relink. Move (b,c)->t1,
        // (c,a)->t2.
        self.relink(nb, b, c, t1);
        self.relink(nc, c, a, t2);
        // Legalize outward from the three new triangles.
        let stack = vec![(t0, 0usize), (t1, 0usize), (t2, 0usize)];
        self.legalize(stack, verts, constraints);
    }

    /// Insert vertex `np`, known to lie on the interior edge `k` of triangle
    /// `t` (shared with a neighbor), splitting both triangles into four.
    /// The edge must not be a constraint and must have a neighbor; the
    /// caller checks both.
    fn insert_on_edge(
        &mut self,
        t: usize,
        k: usize,
        np: usize,
        verts: &[(f64, f64)],
        constraints: &std::collections::HashSet<(usize, usize)>,
    ) {
        let n = self.adj[t][k];
        debug_assert_ne!(n, NO_TRI, "insert_on_edge requires an interior edge");
        let u = self.tris[t][k];
        let w = self.tris[t][(k + 1) % 3];
        let c = self.tris[t][(k + 2) % 3];
        let j = self.edge_index(n, u, w).expect("neighbor shares the edge");
        let q = self.tris[n][(j + 2) % 3];
        // Outer neighbors of the quad u-w-c / w-u-q around edge (u,w).
        let a_wc = self.adj[t][(k + 1) % 3];
        let a_cu = self.adj[t][(k + 2) % 3];
        let b_uq = self.adj[n][(j + 1) % 3];
        let b_qw = self.adj[n][(j + 2) % 3];
        let t2 = self.tris.len();
        let n2 = t2 + 1;
        // t := [u, np, c], t2 := [np, w, c], n := [w, np, q], n2 := [np, u, q]
        self.tris[t] = [u, np, c];
        self.adj[t] = [n2, t2, a_cu];
        self.tris.push([np, w, c]);
        self.adj.push([n, a_wc, t]);
        self.tris[n] = [w, np, q];
        self.adj[n] = [t2, n2, b_qw];
        self.tris.push([np, u, q]);
        self.adj.push([t, b_uq, n]);
        self.relink(a_wc, w, c, t2);
        self.relink(b_uq, u, q, n2);
        // a_cu still borders (c,u) on t; b_qw still borders (q,w) on n.
        let stack = vec![(t, 2), (t2, 1), (n, 2), (n2, 1)];
        self.legalize(stack, verts, constraints);
    }

    /// Flip edge `(t, k)` if it is a non-constraint edge that violates the
    /// Delaunay criterion and the flip keeps both triangles valid. Returns
    /// the four quad edges to recheck, or `None` if no flip happened.
    fn flip_edge(
        &mut self,
        t: usize,
        k: usize,
        verts: &[(f64, f64)],
        constraints: &std::collections::HashSet<(usize, usize)>,
    ) -> Option<[(usize, usize); 4]> {
        let n = self.adj[t][k];
        if n == NO_TRI {
            return None;
        }
        let u = self.tris[t][k];
        let w = self.tris[t][(k + 1) % 3];
        let p = self.tris[t][(k + 2) % 3];
        if constraints.contains(&(u.min(w), u.max(w))) {
            return None;
        }
        // `in_circle` needs (u, w, p) CCW. Every proper triangle is CCW; a
        // non-positive area here is a zero-area seed sliver — leave it be.
        if orient2d(verts[u], verts[w], verts[p]) <= 0.0 {
            return None;
        }
        let q = {
            let tri = &self.tris[n];
            (0..3)
                .map(|i| tri[i])
                .find(|&x| x != u && x != w)
                .expect("neighbor has a third vertex")
        };
        // (u, w, p) is CCW; flip only if q falls inside its circumcircle.
        if !in_circle(verts[u], verts[w], verts[p], verts[q]) {
            return None;
        }
        // Guard against a false positive on a non-convex quad: both new
        // triangles must be strictly CCW.
        if orient2d(verts[p], verts[u], verts[q]) <= 0.0
            || orient2d(verts[p], verts[q], verts[w]) <= 0.0
        {
            return None;
        }
        // Quad p-u-q-w neighbors before the flip.
        let a_pu = self.adj[t][(k + 2) % 3]; // across (p,u)
        let b_wp = self.adj[t][(k + 1) % 3]; // across (w,p)
        let jq = self.edge_index(n, u, q).expect("edge (u,q)");
        let jw = self.edge_index(n, q, w).expect("edge (q,w)");
        let c_uq = self.adj[n][jq];
        let d_qw = self.adj[n][jw];
        // Reuse slot t as [p,u,q] and slot n as [p,q,w].
        self.tris[t] = [p, u, q];
        self.adj[t] = [a_pu, c_uq, n];
        self.tris[n] = [p, q, w];
        self.adj[n] = [t, d_qw, b_wp];
        self.relink(c_uq, u, q, t);
        self.relink(b_wp, w, p, n);
        // a_pu still borders (p,u) on t; d_qw still borders (q,w) on n.
        Some([(t, 0), (t, 1), (n, 1), (n, 2)])
    }

    /// Drain a worklist of edges, flipping each illegal one and rechecking
    /// the edges it exposes.
    fn legalize(
        &mut self,
        mut stack: Vec<(usize, usize)>,
        verts: &[(f64, f64)],
        constraints: &std::collections::HashSet<(usize, usize)>,
    ) {
        let mut guard = 0usize;
        let cap = 64 * self.tris.len() + 64;
        while let Some((t, k)) = stack.pop() {
            guard += 1;
            if guard > cap {
                break;
            }
            if let Some(edges) = self.flip_edge(t, k, verts, constraints) {
                stack.extend_from_slice(&edges);
            }
        }
    }

    /// Flip the whole mesh toward constrained Delaunay: sweep every edge
    /// until a full pass makes no flip. This catches long chords left by the
    /// seed triangulation that no point insertion happened to touch. Returns
    /// the number of sweeps run — with the robust [`in_circle`] this settles
    /// in a handful; the old raw determinant cycled to the cap on every
    /// curved face (of-yud).
    fn make_delaunay(
        &mut self,
        verts: &[(f64, f64)],
        constraints: &std::collections::HashSet<(usize, usize)>,
    ) -> usize {
        // Each sweep is O(edges); a constrained-Delaunay mesh over this many
        // near-lattice points converges in a handful of sweeps. The cap is a
        // safety valve — the mesh stays valid (just possibly a hair off
        // Delaunay) if flips ever fail to settle.
        let max_sweeps = 256;
        let mut used = 0;
        for _ in 0..max_sweeps {
            used += 1;
            let mut flipped = false;
            for t in 0..self.tris.len() {
                for k in 0..3 {
                    if self.flip_edge(t, k, verts, constraints).is_some() {
                        flipped = true;
                    }
                }
            }
            if !flipped {
                break;
            }
        }
        used
    }
}

/// Squared distance from point `p` to segment `ab`.
fn point_seg_dist2(p: (f64, f64), a: (f64, f64), b: (f64, f64)) -> f64 {
    let (abx, aby) = (b.0 - a.0, b.1 - a.1);
    let len2 = abx * abx + aby * aby;
    if len2 <= 0.0 {
        return dist2(p, a);
    }
    let t = (((p.0 - a.0) * abx + (p.1 - a.1) * aby) / len2).clamp(0.0, 1.0);
    dist2(p, (a.0 + t * abx, a.1 + t * aby))
}

/// Even-odd containment of `uv` in the region bounded by the given rings
/// (index ranges into `verts`).
fn ring_contains(uv: (f64, f64), verts: &[(f64, f64)], ring_ranges: &[(usize, usize)]) -> bool {
    let mut inside = false;
    for &(start, len) in ring_ranges {
        for j in 0..len {
            let a = verts[start + j];
            let b = verts[start + (j + 1) % len];
            if crosses_upward(a, b, uv) {
                inside = !inside;
            }
        }
    }
    inside
}

/// Retriangulate a curved face's ear-clip result into a constrained
/// Delaunay mesh seeded with a lattice of interior points, so every triangle
/// edge spans at most about one sampling pitch in `u` (always an angle on
/// curved charts) and `pitch_v` in `v`. The flat 3D chords then hug the
/// surface instead of cutting long secants through it.
///
/// `pitch_v` is the interior row spacing in `v` units (model length on a
/// cylinder, radians on sphere/torus); `scale` converts each parameter axis
/// to model arc length for the boundary-clearance metric.
///
/// `tris` is a valid triangulation of the region using only boundary
/// vertices; `ring_ranges` gives each original boundary ring's index span in
/// `all_uv`/`all_p`. Ring edges are treated as constraints and never flipped,
/// so the boundary polylines stay bit-identical to the adjacent faces' copies
/// (welding preserved). Interior lattice points are kept strictly inside the
/// region and clear of the boundary, so they never weld to anything. The
/// construction is fully deterministic. `tris` is rewritten and interior
/// points are appended to `all_uv`/`all_p`.
fn refine_curved_region(
    tris: &mut Vec<[usize; 3]>,
    all_uv: &mut Vec<(f64, f64)>,
    all_p: &mut Vec<Point3>,
    chart: &Chart,
    pitch_v: f64,
    scale: (f64, f64),
    ring_ranges: &[(usize, usize)],
) {
    if tris.is_empty() || ring_ranges.is_empty() {
        return;
    }
    let pitch = TWO_PI / SAMPLES_PER_CIRCLE as f64;

    // Boundary constraint edges (never flipped) and bounding box.
    let mut constraints: std::collections::HashSet<(usize, usize)> =
        std::collections::HashSet::new();
    let (mut u0, mut u1, mut v0, mut v1) = (
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::INFINITY,
        f64::NEG_INFINITY,
    );
    for &(start, len) in ring_ranges {
        for j in 0..len {
            let a = start + j;
            let b = start + (j + 1) % len;
            constraints.insert((a.min(b), a.max(b)));
            let (u, v) = all_uv[a];
            u0 = u0.min(u);
            u1 = u1.max(u);
            v0 = v0.min(v);
            v1 = v1.max(v);
        }
    }
    if !(u1 > u0 && v1 > v0) {
        return; // degenerate region
    }

    let mut mesh = FlipMesh::from_tris(tris, all_uv);

    // Interior lattice of Steiner points, spread strictly inside the region
    // bounding box. `u` (always an angle here) is spaced at most one sampling
    // pitch apart so retriangulation can bound every edge's u-span; `v` rows
    // are spaced `pitch_v` apart, capped so thin or tall faces stay cheap.
    // At least one interior row and column are laid whenever the region has
    // area (so even a thin full-wrap band still gets its wide chords broken
    // up).
    let (su, sv) = (scale.0.abs().max(1e-12), scale.1.abs().max(1e-12));
    let n_cols = (((u1 - u0) / pitch).ceil() as usize).max(2) - 1;
    let step_u = (u1 - u0) / (n_cols + 1) as f64;
    let pitch_v = pitch_v.max(1e-12);
    let n_rows = (((v1 - v0) / pitch_v).ceil() as usize).clamp(2, 256) - 1;
    let step_v = (v1 - v0) / (n_rows + 1) as f64;

    // Keep interior points a quarter-cell clear of the boundary (in the
    // arc-length metric, each axis scaled to model length so it is isotropic
    // in 3D): enough that they never weld to a boundary vertex and never
    // insert on a boundary edge, while small enough that a thin band keeps
    // its row.
    let margin2 = {
        let m = 0.25 * (step_u * su).min(step_v * sv);
        m * m
    };

    // Staggered by half a cell: boundary polylines are sampled on the same
    // angular pitch the lattice uses (both anchor at the seam), so an
    // unstaggered lattice lands columns EXACTLY under boundary vertices.
    // The constrained triangulation then stacks zero-uv-area triangles
    // along that shared line, which on a doubly-curved chart map to two
    // overlapping bent 3D slivers with cancelling normals — a fold that
    // MeshSdf rejects (of-7ld.7). Half-step offsets keep every lattice
    // point maximally clear of the boundary sample grid while the extra
    // row/column keeps all gaps at or under one pitch.
    for iu in 1..=n_cols + 1 {
        for iv in 1..=n_rows + 1 {
            let uv = (
                u0 + (iu as f64 - 0.5) * step_u,
                v0 + (iv as f64 - 0.5) * step_v,
            );
            if !ring_contains(uv, all_uv, ring_ranges) {
                continue;
            }
            // Clearance from every boundary edge, in arc-length metric.
            let scaled = |p: (f64, f64)| (p.0 * su, p.1 * sv);
            let ps = scaled(uv);
            let mut clear = true;
            'rings: for &(start, len) in ring_ranges {
                for j in 0..len {
                    let a = scaled(all_uv[start + j]);
                    let b = scaled(all_uv[start + (j + 1) % len]);
                    if point_seg_dist2(ps, a, b) < margin2 {
                        clear = false;
                        break 'rings;
                    }
                }
            }
            if !clear {
                continue;
            }
            // Locate the containing triangle (linear scan; all slots live),
            // distinguishing STRICT containment (every edge orientation
            // clearly positive) from an on-edge landing — not the inclusive
            // point-in-triangle test. The boundary is sampled on the same
            // pitch the lattice uses, so long seed chords can have rational
            // slopes in lattice units and then pass EXACTLY through
            // staggered lattice points (e.g. a 53-column/53-row diagonal
            // contains every half-integer point of x + y = 53). Splitting a
            // triangle at a point on one of its edges mints a sliver whose
            // orientation is fp noise; when negative it can never flip
            // (flip_edge's orientation guards) and poisons every later
            // insertion along the same chord, leaving long secant triangles
            // through the surface (of-2ql: napkin-ring wall volume off 1%).
            // Strictly interior points split their host into three as
            // before; on-edge points split the edge's two triangles into
            // four; anything else is skipped.
            let np = all_uv.len();
            let target = uv;
            let eps_area = 1e-9 * step_u * step_v;
            // (triangle, edge landed on) — edge 3 means strictly inside.
            let mut found: Option<(usize, usize)> = None;
            for t in 0..mesh.tris.len() {
                let [a, b, c] = mesh.tris[t];
                let o = [
                    orient2d(all_uv[a], all_uv[b], target),
                    orient2d(all_uv[b], all_uv[c], target),
                    orient2d(all_uv[c], all_uv[a], target),
                ];
                if o[0] > eps_area && o[1] > eps_area && o[2] > eps_area {
                    found = Some((t, 3));
                    break;
                }
                if let Some(k) = (0..3).find(|&k| {
                    o[k].abs() <= eps_area && o[(k + 1) % 3] > eps_area && o[(k + 2) % 3] > eps_area
                }) {
                    found = Some((t, k));
                    break;
                }
            }
            let Some((host, k)) = found else {
                continue; // numerically outside every triangle; skip
            };
            if k == 3 {
                all_uv.push(uv);
                all_p.push(chart_point(chart, uv));
                mesh.insert_in_triangle(host, np, all_uv, &constraints);
                continue;
            }
            // On-edge: split the edge — but only an interior edge with a
            // proper (non-degenerate) neighbor. Constraint edges must never
            // be subdivided (boundary welding depends on them staying
            // bit-identical) and splitting a degenerate neighbor would mint
            // the very slivers this path exists to avoid.
            let (ea, eb) = (mesh.tris[host][k], mesh.tris[host][(k + 1) % 3]);
            if constraints.contains(&(ea.min(eb), ea.max(eb))) {
                continue;
            }
            let n = mesh.adj[host][k];
            if n == NO_TRI {
                continue;
            }
            let [na, nb, nc] = mesh.tris[n];
            if orient2d(all_uv[na], all_uv[nb], all_uv[nc]) <= eps_area {
                continue;
            }
            all_uv.push(uv);
            all_p.push(chart_point(chart, uv));
            mesh.insert_on_edge(host, k, np, all_uv, &constraints);
        }
    }

    // Insertion legalizes only locally; sweep the whole mesh to Delaunay so
    // any long seed chord no inserted point touched (e.g. on a thin band with
    // a single interior row) is flipped away too. The sweep converges in a
    // handful of passes now that `in_circle` is a robust filtered predicate
    // (of-yud): the old raw determinant returned float-noise signs on the
    // extreme-aspect quads a curved face produces, so a Delaunay-neutral edge
    // flipped A→B→A every pass and the loop ran to its 256-sweep cap on every
    // curved face.
    mesh.make_delaunay(all_uv, &constraints);

    // Read back. Insertion and flips only ever retriangulate the same point
    // set inside the seed's boundary, so the mesh still tiles exactly the
    // region the ear clip covered — nothing to cull. Zero-area uv slivers are
    // kept (as before: collinear boundary chains rely on them to pair their
    // chord edges; the emitter drops only zero-3D-length ones).
    tris.clear();
    tris.extend_from_slice(&mesh.tris);
}

fn dist2(a: (f64, f64), b: (f64, f64)) -> f64 {
    let (dx, dy) = (a.0 - b.0, a.1 - b.1);
    dx * dx + dy * dy
}

fn point_in_triangle(p: (f64, f64), a: (f64, f64), b: (f64, f64), c: (f64, f64)) -> bool {
    let sign = |p1: (f64, f64), p2: (f64, f64), p3: (f64, f64)| {
        (p1.0 - p3.0) * (p2.1 - p3.1) - (p2.0 - p3.0) * (p1.1 - p3.1)
    };
    let d1 = sign(p, a, b);
    let d2 = sign(p, b, c);
    let d3 = sign(p, c, a);
    let has_neg = d1 < 0.0 || d2 < 0.0 || d3 < 0.0;
    let has_pos = d1 > 0.0 || d2 > 0.0 || d3 > 0.0;
    !(has_neg && has_pos)
}

/// Does the candidate bridge segment cross any polygon segment or any
/// segment of the given hole rings (excluding segments sharing an
/// endpoint with it)? `hole_rings` must contain every hole not yet
/// spliced into the polygon, current hole included — already-spliced
/// holes are covered by the polygon segments.
fn bridge_is_clear(
    all_uv: &[(f64, f64)],
    polygon: &[usize],
    hole_rings: &[&[(f64, f64)]],
    from: (f64, f64),
    to: (f64, f64),
) -> bool {
    let n = polygon.len();
    for i in 0..n {
        let a = all_uv[polygon[i]];
        let b = all_uv[polygon[(i + 1) % n]];
        if segments_cross(from, to, a, b) {
            return false;
        }
    }
    for ring in hole_rings {
        let hn = ring.len();
        for i in 0..hn {
            if segments_cross(from, to, ring[i], ring[(i + 1) % hn]) {
                return false;
            }
        }
    }
    true
}

/// Strict proper crossing test (shared endpoints do not count).
fn segments_cross(p1: (f64, f64), p2: (f64, f64), q1: (f64, f64), q2: (f64, f64)) -> bool {
    let eps = 1e-14;
    if dist2(p1, q1) < eps || dist2(p1, q2) < eps || dist2(p2, q1) < eps || dist2(p2, q2) < eps {
        return false;
    }
    let d = |a: (f64, f64), b: (f64, f64), c: (f64, f64)| {
        (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0)
    };
    let d1 = d(q1, q2, p1);
    let d2 = d(q1, q2, p2);
    let d3 = d(p1, p2, q1);
    let d4 = d(p1, p2, q2);
    ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0))
        && ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::check::MAX_ALLOWED_TOLERANCE;
    use crate::primitives;
    use crate::surface::SurfaceEval;
    use crate::transform::{rotate_body, translate_body};

    fn tol() -> ToleranceContext {
        ToleranceContext::default()
    }

    #[test]
    fn shell_genus_from_euler_maps_valid_characteristics() {
        // Sphere (chi = 2) is genus 0, torus (chi = 0) is genus 1,
        // double torus (chi = -2) is genus 2.
        assert_eq!(shell_genus_from_euler(2), Some(0));
        assert_eq!(shell_genus_from_euler(0), Some(1));
        assert_eq!(shell_genus_from_euler(-2), Some(2));
        assert_eq!(shell_genus_from_euler(-100), Some(51));
    }

    #[test]
    fn shell_genus_from_euler_rejects_impossible_characteristics() {
        // Odd chi cannot come from a closed orientable surface.
        assert_eq!(shell_genus_from_euler(1), None);
        assert_eq!(shell_genus_from_euler(-1), None);
        assert_eq!(shell_genus_from_euler(3), None);
        // chi > 2 implies genus < 0 (more than one component in the shell).
        assert_eq!(shell_genus_from_euler(4), None);
    }

    // -----------------------------------------------------------------
    // Sphere and torus charts (of-7ld.1)
    // -----------------------------------------------------------------

    const PI: f64 = std::f64::consts::PI;
    const FRAC_PI_2: f64 = std::f64::consts::FRAC_PI_2;

    /// A general (non-axis-aligned) tilt, so tests exercise the `e_u`/`e_v`
    /// basis rather than accidentally passing on coordinate symmetry.
    fn tilted_axis() -> Vector3 {
        Vector3::new(1.0, 2.0, 3.0).normalize()
    }

    fn sphere_chart(center: Point3, radius: f64) -> Chart {
        Chart::build(&Surface3::sphere(center, tilted_axis(), radius).unwrap()).unwrap()
    }

    /// A chart without poles, for tests exercising the plain uv metric.
    fn poleless_chart() -> Chart {
        Chart::Plane {
            origin: Point3::origin(),
            e_u: Vector3::x(),
            e_v: Vector3::y(),
            normal: Vector3::z(),
        }
    }

    fn torus_chart(center: Point3, major: f64, minor: f64) -> Chart {
        Chart::build(&Surface3::torus(center, tilted_axis(), major, minor).unwrap()).unwrap()
    }

    #[test]
    fn chart_build_admits_sphere_and_torus_but_rejects_cone() {
        // Spheres and tori are in the boolean pipeline (of-7ld.4
        // promotion); cones still route through the F-Rep fallback.
        let sphere = Surface3::sphere(Point3::origin(), Vector3::z(), 2.0).unwrap();
        let torus = Surface3::torus(Point3::origin(), Vector3::z(), 3.0, 1.0).unwrap();
        let cone = Surface3::cone(Point3::origin(), Vector3::z(), 0.5, 1.0).unwrap();
        assert!(matches!(Chart::build(&sphere), Ok(Chart::Sphere { .. })));
        assert!(matches!(Chart::build(&torus), Ok(Chart::Torus { .. })));
        assert!(matches!(
            Chart::build(&cone),
            Err(CoreError::NotImplemented { .. })
        ));
    }

    // -----------------------------------------------------------------
    // Broad-phase face boxes (of-7ld.6)
    // -----------------------------------------------------------------

    /// Seam meridian of a unit-ish sphere about +Z: the half-circle in the
    /// xz-plane, the ONLY boundary loop a closed sphere face has.
    fn seam_meridian(center: Point3, radius: f64) -> Vec<Point3> {
        (0..=32)
            .map(|i| {
                let v = -FRAC_PI_2 + PI * i as f64 / 32.0;
                center + Vector3::new(v.cos(), 0.0, v.sin()) * radius
            })
            .collect()
    }

    /// Regression for of-7ld.6: two unit spheres 2−1e-3 apart (razor-thin
    /// lens). Boxes built from seam-only boundary samples are flat along
    /// the seam-plane normal and miss the clash; the exact surface boxes
    /// must overlap so the pair reaches SSI.
    #[test]
    fn broad_phase_sphere_boxes_clash_on_near_tangent_pair() {
        let d = 2.0 - 1e-3;
        let centers = [Point3::origin(), Point3::new(d, 0.0, 0.0)];
        let contact = tol().linear;

        // The failure mode being fixed: seam-sample boxes do not overlap.
        let seam_boxes = centers.map(|c| {
            let bb = BoundingBox3::from_points(seam_meridian(c, 1.0));
            bb.dilate(bb.extents().norm() * 0.05 + contact)
        });
        assert!(
            seam_boxes[0].intersection(&seam_boxes[1]).is_empty(),
            "seam-only boxes unexpectedly clash — scenario no longer exercises the bug"
        );

        // The fix: exact surface boxes clash.
        let face_boxes = centers.map(|c| {
            let surface = Surface3::sphere(c, Vector3::z(), 1.0).unwrap();
            broad_phase_face_box(&surface, seam_meridian(c, 1.0).into_iter(), contact)
        });
        assert!(
            !face_boxes[0].intersection(&face_boxes[1]).is_empty(),
            "near-tangent sphere pair produced no clash candidate: {face_boxes:?}"
        );
    }

    /// Bounded surfaces take the exact surface box (contact-dilated) even
    /// when boundary samples cover a tiny sliver of the surface.
    #[test]
    fn broad_phase_torus_box_is_exact_surface_box() {
        let surface =
            Surface3::torus(Point3::new(1.0, -2.0, 0.5), tilted_axis(), 3.0, 0.5).unwrap();
        let contact = 1e-4;
        // Boundary: a single tube meridian, hopelessly unrepresentative.
        let boundary: Vec<Point3> = (0..=16)
            .map(|i| surface.point(0.0, TWO_PI * i as f64 / 16.0))
            .collect();
        let bb = broad_phase_face_box(&surface, boundary.into_iter(), contact);
        let exact = surface.bounding_box().unwrap().dilate(contact);
        assert!(
            (bb.min - exact.min).norm() < 1e-12 && (bb.max - exact.max).norm() < 1e-12,
            "expected exact surface box {exact:?}, got {bb:?}"
        );
    }

    /// Unbounded surfaces keep the boundary-sample box with sagitta + contact
    /// dilation.
    #[test]
    fn broad_phase_plane_box_falls_back_to_boundary_samples() {
        let surface = Surface3::plane(Point3::origin(), Vector3::z()).unwrap();
        let corners = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(2.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        let contact = 1e-3;
        let bb = broad_phase_face_box(&surface, corners.into_iter(), contact);
        let raw = BoundingBox3::from_points(corners);
        let expected = raw.dilate(raw.extents().norm() * 0.05 + contact);
        assert!(
            (bb.min - expected.min).norm() < 1e-12 && (bb.max - expected.max).norm() < 1e-12,
            "expected sample box {expected:?}, got {bb:?}"
        );
    }

    #[test]
    fn chart_period_helpers_match_surface_topology() {
        let plane =
            Chart::build(&Surface3::plane(Point3::origin(), Vector3::z()).unwrap()).unwrap();
        let cyl = Chart::build(&Surface3::cylinder(Point3::origin(), Vector3::z(), 1.0).unwrap())
            .unwrap();
        let sph = sphere_chart(Point3::origin(), 2.0);
        let tor = torus_chart(Point3::origin(), 4.0, 1.0);
        assert_eq!((plane.period_u(), plane.period_v()), (None, None));
        assert_eq!((cyl.period_u(), cyl.period_v()), (Some(TWO_PI), None));
        assert_eq!((sph.period_u(), sph.period_v()), (Some(TWO_PI), None));
        assert_eq!(
            (tor.period_u(), tor.period_v()),
            (Some(TWO_PI), Some(TWO_PI))
        );
    }

    #[test]
    fn sphere_param_round_trips_off_axis() {
        let center = Point3::new(-1.0, 2.0, 0.5);
        let radius = 2.5;
        let chart = sphere_chart(center, radius);
        for &u in &[0.0, 0.7, 2.0, PI, 4.5, TWO_PI - 0.3] {
            for &v in &[-1.2, -0.4, 0.0, 0.6, 1.3] {
                let p = chart_point(&chart, (u, v));
                // Lands on the sphere.
                assert!(((p - center).norm() - radius).abs() < 1e-9);
                // Round-trips to the same parameters given a nearby hint.
                let (u2, v2) = chart.param(&p, Some((u, v)));
                assert!((u2 - u).abs() < 1e-9, "u {u} -> {u2}");
                assert!((v2 - v).abs() < 1e-9, "v {v} -> {v2}");
            }
        }
    }

    #[test]
    fn sphere_longitude_unwraps_toward_hint() {
        let chart = sphere_chart(Point3::origin(), 1.0);
        // A point just past the seam (u ≈ 0.1) reported near a hint at
        // u ≈ 2π stays on the hint's branch, not wrapped back to ~0.1.
        let p = chart_point(&chart, (0.1, 0.3));
        let (u, _) = chart.param(&p, Some((TWO_PI, 0.3)));
        assert!(
            (u - (TWO_PI + 0.1)).abs() < 1e-9,
            "expected ~2π+0.1, got {u}"
        );
    }

    #[test]
    fn sphere_poles_inherit_hint_longitude_and_do_not_degenerate() {
        let center = Point3::new(3.0, 0.0, -1.0);
        let radius = 1.7;
        let chart = sphere_chart(center, radius);
        let north = chart_point(&chart, (0.0, FRAC_PI_2));
        let south = chart_point(&chart, (0.0, -FRAC_PI_2));
        // At a pole the longitude is undefined; it inherits the hint...
        for &hint_u in &[0.0, 1.3, PI, 5.0] {
            let (u, v) = chart.param(&north, Some((hint_u, 0.0)));
            assert!((u - hint_u).abs() < 1e-12, "north kept hint u");
            assert!((v - FRAC_PI_2).abs() < 1e-9);
            let (u, v) = chart.param(&south, Some((hint_u, 0.0)));
            assert!((u - hint_u).abs() < 1e-12, "south kept hint u");
            assert!((v + FRAC_PI_2).abs() < 1e-9);
        }
        // ...and defaults to 0 with no hint (still a well-defined param, so
        // an imprint through the pole never spawns a zero-width UV wedge).
        let (u, _) = chart.param(&north, None);
        assert_eq!(u, 0.0);
    }

    #[test]
    fn sphere_pole_touch_keeps_longitude_constant_no_wedge() {
        // A meridian imprint that runs up to the pole and retreats down the
        // *same* meridian must keep a single longitude throughout — the pole
        // convention makes the singular sample inherit the incoming `u`, so
        // the UV polyline is a retraced vertical segment (zero enclosed
        // area) rather than a degenerate wedge fanning across longitudes.
        // (A great circle passing *through* the pole onto the antipodal
        // meridian genuinely flips `u` by π; that is inherent to lat/long
        // charts, not a defect, so this tests the touch-and-return case.)
        let chart = sphere_chart(Point3::origin(), 1.0);
        let l = 0.9_f64;
        let steps = 16;
        let mut samples = Vec::new();
        for i in 0..=steps {
            let t = i as f64 / steps as f64;
            samples.push(chart_point(&chart, (l, t * FRAC_PI_2)));
        }
        for i in 1..=steps {
            let t = i as f64 / steps as f64;
            samples.push(chart_point(&chart, (l, FRAC_PI_2 - t * FRAC_PI_2)));
        }
        let mut hint: Option<(f64, f64)> = None;
        let mut us = Vec::new();
        for p in &samples {
            let (u, v) = chart.param(p, hint);
            us.push(u);
            hint = Some((u, v));
        }
        for (k, u) in us.iter().enumerate() {
            assert!((u - l).abs() < 1e-9, "sample {k} longitude drifted: {u}");
        }
    }

    #[test]
    fn sphere_uv_scale_is_anisotropic_and_shrinks_at_poles() {
        let radius = 3.0;
        let chart = sphere_chart(Point3::origin(), radius);
        // Equator: longitude arc uses the full radius; latitude too.
        let (us, vs) = chart.uv_scale(0.0);
        assert!((us - radius).abs() < 1e-12);
        assert!((vs - radius).abs() < 1e-12);
        // Mid-latitude: longitude scale is radius·cos(v).
        let (us, vs) = chart.uv_scale(PI / 3.0);
        assert!((us - radius * 0.5).abs() < 1e-12);
        assert!((vs - radius).abs() < 1e-12);
        // Near the pole the longitude circle collapses.
        let (us, _) = chart.uv_scale(FRAC_PI_2);
        assert!(us.abs() < 1e-9);
    }

    #[test]
    fn sphere_normal_is_outward_radial() {
        let center = Point3::new(0.5, -2.0, 1.0);
        let radius = 1.4;
        let chart = sphere_chart(center, radius);
        for &(u, v) in &[(0.3, 0.2), (2.5, -0.9), (5.0, 1.1)] {
            let p = chart_point(&chart, (u, v));
            let n = chart.normal(u, v);
            assert!((n.norm() - 1.0).abs() < 1e-12, "unit normal");
            let outward = (p - center) / radius;
            assert!((n - outward).norm() < 1e-9, "normal is outward radial");
        }
    }

    #[test]
    fn torus_param_round_trips_across_both_seams() {
        let center = Point3::new(2.0, -1.0, 3.0);
        let (major, minor) = (5.0, 1.5);
        let chart = torus_chart(center, major, minor);
        for &u in &[0.0, 1.1, PI, 4.0, TWO_PI - 0.2] {
            for &v in &[0.0, 0.8, PI, 4.7, TWO_PI - 0.1] {
                let p = chart_point(&chart, (u, v));
                let (u2, v2) = chart.param(&p, Some((u, v)));
                assert!((u2 - u).abs() < 1e-9, "u {u} -> {u2}");
                assert!((v2 - v).abs() < 1e-9, "v {v} -> {v2}");
            }
        }
    }

    #[test]
    fn torus_param_unwraps_both_axes_toward_hint() {
        let chart = torus_chart(Point3::origin(), 4.0, 1.0);
        // A point just past both seams reported near a hint one full period
        // out on each axis stays on the hint's cover copy.
        let p = chart_point(&chart, (0.15, 0.25));
        let (u, v) = chart.param(&p, Some((TWO_PI, TWO_PI)));
        assert!(
            (u - (TWO_PI + 0.15)).abs() < 1e-9,
            "major angle unwrapped: {u}"
        );
        assert!(
            (v - (TWO_PI + 0.25)).abs() < 1e-9,
            "minor angle unwrapped: {v}"
        );
    }

    #[test]
    fn torus_uv_scale_breathes_with_the_minor_angle() {
        let (major, minor) = (6.0, 2.0);
        let chart = torus_chart(Point3::origin(), major, minor);
        // Outer equator (v = 0): major circle radius is major + minor.
        let (us, vs) = chart.uv_scale(0.0);
        assert!((us - (major + minor)).abs() < 1e-12);
        assert!((vs - minor).abs() < 1e-12);
        // Inner equator (v = π): major + minor·cos(π) = major - minor.
        let (us, _) = chart.uv_scale(PI);
        assert!((us - (major - minor)).abs() < 1e-12);
        // Top of the tube (v = π/2): back to the bare major radius.
        let (us, _) = chart.uv_scale(FRAC_PI_2);
        assert!((us - major).abs() < 1e-12);
    }

    #[test]
    fn torus_normal_points_away_from_the_tube_center() {
        let center = Point3::new(-1.0, 0.0, 2.0);
        let (major, minor) = (4.0, 1.2);
        let chart = torus_chart(center, major, minor);
        let Chart::Torus { e_u, e_v, .. } = &chart else {
            unreachable!()
        };
        for &(u, v) in &[(0.4, 0.3), (2.2, 3.0), (5.1, 4.4)] {
            let p = chart_point(&chart, (u, v));
            let n = chart.normal(u, v);
            assert!((n.norm() - 1.0).abs() < 1e-12);
            // Tube center at major angle u; the outward normal is the unit
            // vector from there to the surface point.
            let radial = e_u * u.cos() + e_v * u.sin();
            let tube_center = center + radial * major;
            let outward = (p - tube_center) / minor;
            assert!((n - outward).norm() < 1e-9, "normal away from tube axis");
        }
    }

    #[test]
    fn torus_face_localizes_both_periods_into_the_cover_window() {
        // A torus cover quad living in u,v ∈ [0.2, 0.6]; a probe an entire
        // period out on *each* axis must fold back into the window.
        let chart = torus_chart(Point3::origin(), 4.0, 1.0);
        let loop_uv = [(0.2, 0.2), (0.6, 0.2), (0.6, 0.6), (0.2, 0.6)];
        let cover: Vec<_> = loop_uv
            .iter()
            .map(|&(u, v)| ((u, v), chart_point(&chart, (u, v))))
            .collect();
        let fp = FaceRegionPoly {
            chart,
            loops: vec![cover],
        };
        let (u, v) = fp.localize((0.4 + TWO_PI, 0.4 + TWO_PI));
        assert!((u - 0.4).abs() < 1e-9, "u folded back: {u}");
        assert!((v - 0.4).abs() < 1e-9, "v folded back: {v}");
        let (u, v) = fp.localize((0.4 - TWO_PI, 0.4 - TWO_PI));
        assert!((u - 0.4).abs() < 1e-9);
        assert!((v - 0.4).abs() < 1e-9);
    }

    fn stores() -> (TopologyStore, GeometryStore) {
        (TopologyStore::new(), GeometryStore::new())
    }

    /// Origin-centered block moved to `center`.
    fn block_at(
        store: &mut TopologyStore,
        geo: &mut GeometryStore,
        sizes: (f64, f64, f64),
        center: (f64, f64, f64),
    ) -> EntityId<Body> {
        let body = primitives::block(store, geo, sizes.0, sizes.1, sizes.2).expect("valid block");
        translate_body(store, geo, body, Vector3::new(center.0, center.1, center.2))
            .expect("finite offset");
        body
    }

    /// Origin-centered +Z cylinder moved to `center`.
    fn cylinder_at(
        store: &mut TopologyStore,
        geo: &mut GeometryStore,
        radius: f64,
        height: f64,
        center: (f64, f64, f64),
    ) -> EntityId<Body> {
        let body = primitives::cylinder(store, geo, radius, height).expect("valid cylinder");
        translate_body(store, geo, body, Vector3::new(center.0, center.1, center.2))
            .expect("finite offset");
        body
    }

    fn assert_valid(out: &BooleanOutput, context: &str) {
        let failures = out.check();
        assert!(
            failures.is_empty(),
            "{context}: check() reported {} failures: {:#?}",
            failures.len(),
            failures
        );
        let mesh = out.tessellate().expect("tessellation succeeds");
        assert!(
            mesh.is_closed_manifold(),
            "{context}: tessellation is not a closed manifold"
        );
    }

    /// Every face binds a live surface and every edge a live curve whose
    /// endpoints interpolate the edge's vertices within the recorded
    /// tolerances.
    fn assert_geometry_bound(out: &BooleanOutput, context: &str) {
        for face in out.store.faces_of_body(out.body) {
            let surface_id = out
                .store
                .face(face)
                .unwrap()
                .surface
                .unwrap_or_else(|| panic!("{context}: {face:?} has no bound surface"));
            assert!(
                out.geo.surface(surface_id).is_some(),
                "{context}: {face:?} references a dead surface id"
            );
            for edge_id in out.store.edges_of_face(face) {
                let edge = out.store.edge(edge_id).unwrap();
                let curve_id = edge
                    .curve
                    .unwrap_or_else(|| panic!("{context}: {edge_id:?} has no bound curve"));
                let curve = out
                    .geo
                    .curve(curve_id)
                    .unwrap_or_else(|| panic!("{context}: {edge_id:?} dead curve id"));
                assert!(
                    edge.t_end > edge.t_start,
                    "{context}: {edge_id:?} has empty parameter range"
                );
                assert!(
                    edge.tolerance <= MAX_ALLOWED_TOLERANCE,
                    "{context}: {edge_id:?} tolerance {} above limit",
                    edge.tolerance
                );
                for (t, v) in [
                    (edge.t_start, edge.start_vertex),
                    (edge.t_end, edge.end_vertex),
                ] {
                    let vertex = out.store.vertex(v).unwrap();
                    let gap = (curve.point(t) - vertex.point).norm();
                    assert!(
                        gap <= edge.tolerance.max(vertex.tolerance) + 1e-12,
                        "{context}: {edge_id:?} endpoint off vertex by {gap:.2e} \
                         (edge tol {:.2e}, vertex tol {:.2e})",
                        edge.tolerance,
                        vertex.tolerance
                    );
                }
            }
        }
    }

    /// Re-encode a block's faces with inward-pointing surface normals: flip
    /// each planar surface's normal and flip the face's `FaceSense` to
    /// compensate. The declared outward direction, loop winding, and
    /// represented point-set are all unchanged, so the body is still a valid
    /// `check()`-clean solid of the same region — but every face now reports
    /// `outward_along_normal == false` in the pipeline, so its loop winds CW
    /// in the surface chart. This is the legitimate encoding an importer with
    /// inward normals produces; it exercises the region-tracing path the
    /// pipeline used to assume away (of-alr). Blocks only — all six faces are
    /// planes, the one surface kind whose normal can simply be negated (a
    /// `Surface3::Cylinder` normal is intrinsically radially outward).
    fn flip_block_face_encoding(
        store: &mut TopologyStore,
        geo: &mut GeometryStore,
        body: EntityId<Body>,
    ) {
        for face_id in store.faces_of_body(body) {
            let surface_id = store.face(face_id).unwrap().surface.expect("bound surface");
            let flipped = match geo.surface(surface_id).expect("live surface") {
                Surface3::Plane { origin, normal } => Surface3::Plane {
                    origin: *origin,
                    normal: -*normal,
                },
                other => panic!("flip_block_face_encoding expects planar faces, got {other:?}"),
            };
            let new_id = geo.add_surface(flipped);
            let face = store.faces.get_mut(face_id).expect("live face");
            face.surface = Some(new_id);
            face.sense = match face.sense {
                FaceSense::Positive => FaceSense::Negative,
                FaceSense::Negative => FaceSense::Positive,
            };
        }
    }

    #[test]
    fn unite_inward_normal_blocks_matches_normal_orientation() {
        // of-alr: a face whose declared outward opposes its surface normal
        // (outward_along_normal == false) is legal topology, but its outer
        // loop then winds CW in the surface chart. The region machinery
        // assumed CCW-in-chart, so region_interior_point's inward left normal
        // pointed OUT and every probe on a convex region failed — a spurious
        // CoreError::Degenerate. Two inward-normal blocks (same region as the
        // all-Positive originals) must unite identically.
        let (mut store, mut geo) = stores();
        let a = block_at(&mut store, &mut geo, (2.0, 2.0, 2.0), (0.0, 0.0, 0.0));
        let b = block_at(&mut store, &mut geo, (2.0, 2.0, 2.0), (1.0, 1.0, 1.0));
        flip_block_face_encoding(&mut store, &mut geo, a);
        flip_block_face_encoding(&mut store, &mut geo, b);
        // The re-encoding preserves the region — the inputs stay valid solids.
        assert!(
            store.check(a).is_empty(),
            "inward-normal A: {:?}",
            store.check(a)
        );
        assert!(
            store.check(b).is_empty(),
            "inward-normal B: {:?}",
            store.check(b)
        );

        let out = unite(&store, &geo, a, b, &tol()).expect("unite of inward-normal blocks");
        assert_valid(&out, "unite inward-normal blocks");
        assert_geometry_bound(&out, "unite inward-normal blocks");

        // Identical shape to the all-Positive union of the same two blocks.
        let (mut store2, mut geo2) = stores();
        let a2 = block_at(&mut store2, &mut geo2, (2.0, 2.0, 2.0), (0.0, 0.0, 0.0));
        let b2 = block_at(&mut store2, &mut geo2, (2.0, 2.0, 2.0), (1.0, 1.0, 1.0));
        let out2 = unite(&store2, &geo2, a2, b2, &tol()).expect("unite of normal blocks");
        assert_eq!(
            out.face_count(),
            out2.face_count(),
            "inward-normal union must match the normal union's face count"
        );
        assert_eq!(out.shell_count(), out2.shell_count());
    }

    #[test]
    fn subtract_inward_normal_blocks_matches_normal_orientation() {
        // of-alr, reverse × flip path: in a subtract the tool's kept faces
        // carry reverse == true. Combined with outward_along_normal == false
        // this is the corner apply_chain's ring/hole split and the output
        // winding both have to get right. Notch a block with an overlapping
        // block, both inward-normal encoded; the result must match the
        // all-Positive subtract exactly (a valid, closed, manifold notch).
        let (mut store, mut geo) = stores();
        let target = block_at(&mut store, &mut geo, (4.0, 4.0, 4.0), (0.0, 0.0, 0.0));
        let tool = block_at(&mut store, &mut geo, (2.0, 2.0, 2.0), (2.0, 2.0, 2.0));
        flip_block_face_encoding(&mut store, &mut geo, target);
        flip_block_face_encoding(&mut store, &mut geo, tool);
        assert!(
            store.check(target).is_empty(),
            "target: {:?}",
            store.check(target)
        );
        assert!(
            store.check(tool).is_empty(),
            "tool: {:?}",
            store.check(tool)
        );

        let out =
            subtract(&store, &geo, target, tool, &tol()).expect("subtract of inward-normal blocks");
        assert_valid(&out, "subtract inward-normal blocks");
        assert_geometry_bound(&out, "subtract inward-normal blocks");

        let (mut store2, mut geo2) = stores();
        let target2 = block_at(&mut store2, &mut geo2, (4.0, 4.0, 4.0), (0.0, 0.0, 0.0));
        let tool2 = block_at(&mut store2, &mut geo2, (2.0, 2.0, 2.0), (2.0, 2.0, 2.0));
        let out2 =
            subtract(&store2, &geo2, target2, tool2, &tol()).expect("subtract of normal blocks");
        assert_eq!(
            out.face_count(),
            out2.face_count(),
            "inward-normal subtract must match the normal subtract's face count"
        );
        assert_eq!(out.shell_count(), out2.shell_count());
    }

    #[test]
    fn through_hole_classification_is_scale_invariant() {
        // The same through-hole subtract must produce the same topology at
        // every scale: the classification/imprint bands are relative to
        // feature size (edge snap, face extent), not absolute floors. Before
        // of-lxk the absolute `1e-5` edge-accept floor and `tol.linear * 10`
        // boundary band swallowed sub-1e-4 features and produced a
        // `Degenerate` ray-classify at k = 1e-5.
        for k in [1.0, 1e-1, 1e-2, 1e-3, 1e-4, 1e-5] {
            let (mut store, mut geo) = stores();
            let block = block_at(
                &mut store,
                &mut geo,
                (4.0 * k, 4.0 * k, 2.0 * k),
                (0.0, 0.0, 0.0),
            );
            let tool = cylinder_at(&mut store, &mut geo, 1.0 * k, 4.0 * k, (0.0, 0.0, 0.0));
            let out = subtract(&store, &geo, block, tool, &tol())
                .unwrap_or_else(|e| panic!("k={k:e}: subtract failed: {e:?}"));
            assert_eq!(
                out.face_count(),
                7,
                "k={k:e}: 6 block faces + 1 cylinder band"
            );
            assert_eq!(out.shell_count(), 1, "k={k:e}: single shell");
            assert_eq!(
                out.store.euler_counts(out.body).genus,
                1,
                "k={k:e}: through hole must give genus 1"
            );
            assert!(
                out.check().is_empty(),
                "k={k:e}: topology check failures: {:?}",
                out.check()
            );
        }
    }

    #[test]
    fn subtract_block_minus_cylinder_makes_through_hole() {
        // Cylinder pierces the block completely: the result is the block
        // with a cylindrical hole — 6 block faces (top and bottom gaining
        // a circular inner loop) plus the trimmed cylinder wall.
        let (mut store, mut geo) = stores();
        let block = block_at(&mut store, &mut geo, (4.0, 4.0, 2.0), (2.0, 2.0, 1.0));
        let tool = cylinder_at(&mut store, &mut geo, 1.0, 4.0, (2.0, 2.0, 1.0));
        let out = subtract(&store, &geo, block, tool, &tol()).unwrap();
        assert_eq!(out.face_count(), 7, "6 block faces + 1 cylinder band");
        assert_eq!(out.shell_count(), 1);
        assert_valid(&out, "block minus cylinder");
        assert_geometry_bound(&out, "block minus cylinder");
        // Through-hole: genus 1.
        let counts = out.store.euler_counts(out.body);
        assert_eq!(counts.genus, 1, "through hole must give genus 1");
    }

    #[test]
    fn subtract_tilted_cylinder_seam_aligned_makes_through_hole() {
        // of-ipt.5 regression: a tool cylinder tilted 15° in the YZ plane
        // pierces the slab top and bottom. Both plane∩cylinder ellipses
        // cross the wall chart's seam meridian exactly (the configuration
        // is x-symmetric and the chart's e_u is x-aligned), so one clip
        // sample per ellipse lands exactly on the seam. That sample used
        // to coin-flip to "outside", demote the ring to an open run with
        // coincident refined endpoints, and silently drop the whole
        // imprint — subtract returned the slab unchanged with Ok.
        let (mut store, mut geo) = stores();
        let slab = block_at(&mut store, &mut geo, (6.0, 6.0, 2.0), (3.0, 3.0, 1.0));
        let theta = 15f64.to_radians();
        let tool = primitives::cylinder(&mut store, &mut geo, 0.5, 8.0).expect("valid cylinder");
        // Tilt the +Z axis to (0, sin θ, cos θ), then move to the slab.
        rotate_body(
            &mut store,
            &mut geo,
            tool,
            Point3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            -theta,
        )
        .expect("finite rotation");
        translate_body(&mut store, &mut geo, tool, Vector3::new(3.0, 3.0, 1.0))
            .expect("finite offset");
        let out = subtract(&store, &geo, slab, tool, &tol()).unwrap();
        assert_eq!(out.face_count(), 7, "6 slab faces + 1 tilted cylinder band");
        assert_eq!(out.shell_count(), 1);
        assert_valid(&out, "slab minus 15° tilted cylinder");
        assert_geometry_bound(&out, "slab minus 15° tilted cylinder");
        let counts = out.store.euler_counts(out.body);
        assert_eq!(counts.genus, 1, "through hole must give genus 1");
    }

    #[test]
    fn unite_overlapping_blocks_face_count() {
        // Corner overlap: each block keeps 3 untouched faces and 3
        // L-shaped trimmed faces — 12 faces total.
        let (mut store, mut geo) = stores();
        let a = block_at(&mut store, &mut geo, (2.0, 2.0, 2.0), (1.0, 1.0, 1.0));
        let b = block_at(&mut store, &mut geo, (2.0, 2.0, 2.0), (2.0, 2.0, 2.0));
        let out = unite(&store, &geo, a, b, &tol()).unwrap();
        assert_eq!(out.face_count(), 12);
        assert_eq!(out.shell_count(), 1);
        assert_valid(&out, "union of overlapping blocks");
        assert_geometry_bound(&out, "union of overlapping blocks");
        let counts = out.store.euler_counts(out.body);
        assert_eq!(counts.genus, 0);
    }

    #[test]
    fn intersect_overlapping_blocks_is_overlap_box() {
        let (mut store, mut geo) = stores();
        let a = block_at(&mut store, &mut geo, (2.0, 2.0, 2.0), (1.0, 1.0, 1.0));
        let b = block_at(&mut store, &mut geo, (2.0, 2.0, 2.0), (2.0, 2.0, 2.0));
        let out = intersect(&store, &geo, a, b, &tol()).unwrap();
        assert_eq!(out.face_count(), 6, "intersection of blocks is a block");
        assert_valid(&out, "intersection of overlapping blocks");
        assert_geometry_bound(&out, "intersection of overlapping blocks");
        let mesh = out.tessellate().unwrap();
        let bb = mesh.bounding_box().expect("non-empty mesh");
        for (got, want) in [
            (bb.min.x, 1.0),
            (bb.min.y, 1.0),
            (bb.min.z, 1.0),
            (bb.max.x, 2.0),
            (bb.max.y, 2.0),
            (bb.max.z, 2.0),
        ] {
            assert!(
                (got - want).abs() < 1e-6,
                "intersection extent {got} != {want}"
            );
        }
    }

    #[test]
    fn unite_disjoint_solids_keeps_two_shells() {
        let (mut store, mut geo) = stores();
        let a = block_at(&mut store, &mut geo, (1.0, 1.0, 1.0), (0.0, 0.0, 0.0));
        let b = block_at(&mut store, &mut geo, (1.0, 1.0, 1.0), (5.0, 5.0, 5.0));
        let out = unite(&store, &geo, a, b, &tol()).unwrap();
        assert_eq!(out.face_count(), 12);
        assert_eq!(out.shell_count(), 2);
        assert_valid(&out, "union of disjoint blocks");
        assert_geometry_bound(&out, "union of disjoint blocks");
    }

    #[test]
    fn intersect_disjoint_solids_is_empty() {
        let (mut store, mut geo) = stores();
        let a = block_at(&mut store, &mut geo, (1.0, 1.0, 1.0), (0.0, 0.0, 0.0));
        let b = block_at(&mut store, &mut geo, (1.0, 1.0, 1.0), (5.0, 5.0, 5.0));
        let out = intersect(&store, &geo, a, b, &tol()).unwrap();
        assert_eq!(out.face_count(), 0);
        assert_eq!(out.shell_count(), 0);
    }

    #[test]
    fn subtract_embedded_cylinder_makes_internal_void() {
        // Tool entirely inside the block: no face crossings at all; the
        // subtract keeps the whole reversed tool boundary as a void shell.
        let (mut store, mut geo) = stores();
        let block = block_at(&mut store, &mut geo, (4.0, 4.0, 4.0), (0.0, 0.0, 0.0));
        let tool = cylinder_at(&mut store, &mut geo, 1.0, 2.0, (0.0, 0.0, 0.0));
        let out = subtract(&store, &geo, block, tool, &tol()).unwrap();
        assert_eq!(out.face_count(), 9, "6 block faces + 3 reversed tool faces");
        assert_eq!(out.shell_count(), 2, "outer shell + void shell");
        assert_valid(&out, "block with internal void");
        assert_geometry_bound(&out, "block with internal void");
    }

    /// of-lcx: the through-hole cylinder band is a trimmed curved face. Its
    /// structured tessellation must hug the surface — worst chordal deviation
    /// on the order of the boundary sampling, not the cylinder radius — while
    /// staying a closed manifold. (The old boundary-only ear clip cut long
    /// secants: deviation ≈ the radius, volume ~22% high.)
    #[test]
    fn curved_band_tessellation_is_low_deviation_and_manifold() {
        let (mut store, mut geo) = stores();
        let slab = block_at(&mut store, &mut geo, (4.0, 4.0, 2.0), (0.0, 0.0, 0.0));
        let tool = cylinder_at(&mut store, &mut geo, 1.0, 4.0, (0.0, 0.0, 0.0));
        let out = subtract(&store, &geo, slab, tool, &tol()).unwrap();
        let (mesh, dev) = out.tessellate_measured().expect("tessellation");
        assert!(
            mesh.is_closed_manifold(),
            "band mesh must be closed manifold"
        );
        // One sampling pitch of chord on the unit cylinder deviates by
        // r·(1 − cos(π/96)) ≈ 5.4e-4; allow generous slack, but nothing near
        // the radius (the old ear-clip failure mode).
        assert!(
            dev < 5e-3,
            "worst chordal deviation {dev:e} should be sampling-scale, not radius-scale"
        );
        // Interior lattice points were inserted (far more triangles than the
        // ~2 per boundary quad an ear clip of the band alone would give).
        assert!(
            mesh.triangle_count() > 400,
            "structured band should be densely triangulated, got {}",
            mesh.triangle_count()
        );
    }

    /// of-lcx: a thin full-wrap band (plate much thinner than the hole
    /// radius) still spans the full 2π in u, so its wide chords must be
    /// broken up even though the band leaves little room in v for interior
    /// rows. The lattice guarantees at least one interior row/column.
    #[test]
    fn thin_curved_band_is_low_deviation_and_manifold() {
        let (mut store, mut geo) = stores();
        let plate = block_at(&mut store, &mut geo, (4.0, 4.0, 0.1), (0.0, 0.0, 0.0));
        let tool = cylinder_at(&mut store, &mut geo, 1.0, 4.0, (0.0, 0.0, 0.0));
        let out = subtract(&store, &geo, plate, tool, &tol()).unwrap();
        let (mesh, dev) = out.tessellate_measured().expect("tessellation");
        assert!(
            mesh.is_closed_manifold(),
            "thin band must be closed manifold"
        );
        assert!(
            dev < 5e-3,
            "thin-band deviation {dev:e} must stay sampling-scale, not radius-scale"
        );
    }

    /// of-lcx: tessellation must be deterministic run-to-run (the earlier
    /// refine-by-bisection attempt was sensitive to HashMap seed ordering).
    #[test]
    fn curved_tessellation_is_deterministic() {
        let build = || {
            let (mut store, mut geo) = stores();
            let slab = block_at(&mut store, &mut geo, (4.0, 4.0, 2.0), (0.0, 0.0, 0.0));
            let tool = cylinder_at(&mut store, &mut geo, 1.0, 4.0, (0.0, 0.0, 0.0));
            let out = subtract(&store, &geo, slab, tool, &tol()).unwrap();
            let (mesh, dev) = out.tessellate_measured().expect("tessellation");
            (mesh.triangle_count(), dev)
        };
        let (n1, d1) = build();
        let (n2, d2) = build();
        assert_eq!(n1, n2, "triangle count must be identical across runs");
        assert_eq!(
            d1.to_bits(),
            d2.to_bits(),
            "deviation must be bit-identical"
        );
    }

    #[test]
    fn geometric_snap_uses_extent_not_origin_distance() {
        let cloud = |off: f64| [Point3::new(off, 0.0, 0.0), Point3::new(off + 2.0, 1.0, 0.5)];
        // Near the origin: 1e-9 of the 2.0 bounding-box extent.
        let near = geometric_snap(cloud(0.0));
        assert!((near - 2e-9).abs() < 1e-18, "near snap {near:e}");
        // Translating the cloud must not change the snap while the ULP
        // floor is below the extent term.
        assert_eq!(geometric_snap(cloud(1e3)), near);
        // Far enough out the floor engages: 100 ULPs of the largest
        // coordinate, so welding never drops below f64 resolution.
        let far = geometric_snap(cloud(1e9));
        assert_eq!(far, 100.0 * f64::EPSILON * (1e9 + 2.0));
        // Degenerate clouds still give a positive, finite snap.
        assert!(geometric_snap([]) > 0.0);
        assert!(geometric_snap([Point3::origin()]) > 0.0);
    }

    /// of-260: snap bands must derive from feature extent, not distance
    /// from the origin. Micro-scale blocks that boolean cleanly at the
    /// origin must boolean identically when translated far from it (the
    /// old magnitude-derived snap failed from x = 1e3 on).
    #[test]
    fn subtract_small_blocks_far_from_origin() {
        let s = 1e-3;
        for off in [0.0, 1e2, 1e3, 1e4, 1e6] {
            let ctx = format!("1e-3 blocks at x = {off:e}");
            let (mut store, mut geo) = stores();
            let a = block_at(&mut store, &mut geo, (s, s, s), (off, 0.0, 0.0));
            let b = block_at(
                &mut store,
                &mut geo,
                (s, s, s),
                (off + s / 2.0, s / 2.0, s / 2.0),
            );
            let out = subtract(&store, &geo, a, b, &tol())
                .unwrap_or_else(|e| panic!("{ctx}: subtract failed: {e:?}"));
            assert_eq!(out.face_count(), 9, "{ctx}: face count");
            assert_eq!(out.shell_count(), 1, "{ctx}: shell count");
            assert_valid(&out, &ctx);
            assert_geometry_bound(&out, &ctx);
        }
    }

    /// of-260, curved case: a through-hole subtract must survive the
    /// whole assembly sitting far from the origin.
    #[test]
    fn subtract_through_hole_far_from_origin() {
        let off = 1e3;
        let (mut store, mut geo) = stores();
        let block = block_at(&mut store, &mut geo, (4.0, 4.0, 2.0), (off, 0.0, 0.0));
        let tool = cylinder_at(&mut store, &mut geo, 1.0, 4.0, (off, 0.0, 0.0));
        let out = subtract(&store, &geo, block, tool, &tol()).unwrap();
        assert_eq!(out.face_count(), 7, "6 block faces + 1 cylinder band");
        assert_valid(&out, "far block minus cylinder");
        assert_geometry_bound(&out, "far block minus cylinder");
        let counts = out.store.euler_counts(out.body);
        assert_eq!(counts.genus, 1, "through hole must give genus 1");
    }

    #[test]
    fn output_binds_host_surfaces_and_source_curves() {
        // Through-hole case: the band face must carry the tool's cylinder
        // surface, and the hole rims must be circles of the tool's radius.
        let (mut store, mut geo) = stores();
        let block = block_at(&mut store, &mut geo, (4.0, 4.0, 2.0), (0.0, 0.0, 0.0));
        let tool = cylinder_at(&mut store, &mut geo, 1.0, 4.0, (0.0, 0.0, 0.0));
        let out = subtract(&store, &geo, block, tool, &tol()).unwrap();

        let faces = out.store.faces_of_body(out.body);
        let cylinder_faces: Vec<_> = faces
            .iter()
            .filter(|&&f| {
                let id = out.store.face(f).unwrap().surface.unwrap();
                matches!(out.geo.surface(id).unwrap(), Surface3::Cylinder { .. })
            })
            .copied()
            .collect();
        assert_eq!(cylinder_faces.len(), 1, "exactly one cylindrical band");

        let mut rim_circles = 0;
        for edge_id in out.store.edges_of_face(cylinder_faces[0]) {
            let edge = out.store.edge(edge_id).unwrap();
            if let Curve3::Circle { radius, .. } = out.geo.curve(edge.curve.unwrap()).unwrap() {
                assert!((radius - 1.0).abs() < 1e-9, "rim radius must be the tool's");
                rim_circles += 1;
            }
        }
        assert!(rim_circles >= 2, "band must be rimmed by circular edges");
    }

    #[test]
    fn bodies_without_bound_geometry_are_rejected() {
        let (mut store, mut geo) = stores();
        let a = block_at(&mut store, &mut geo, (2.0, 2.0, 2.0), (0.0, 0.0, 0.0));
        let b = block_at(&mut store, &mut geo, (2.0, 2.0, 2.0), (1.0, 1.0, 1.0));
        // Strip one face's surface binding: extraction must refuse.
        let face = store.faces_of_body(b)[0];
        store.faces.get_mut(face).unwrap().surface = None;
        let err = unite(&store, &geo, a, b, &tol()).unwrap_err();
        assert!(
            matches!(err, CoreError::InvalidArgument { argument: "b", .. }),
            "expected InvalidArgument for unbound geometry, got {err:?}"
        );
        assert!(
            err.to_string().contains("surface"),
            "unhelpful error: {err}"
        );
    }

    #[test]
    fn sphere_inputs_take_the_exact_path() {
        // The of-7ld.4 promotion: sphere charts are admitted, so a
        // transversal block-sphere boolean runs the exact pipeline
        // end-to-end (a unit sphere at the origin dipping 0.5 deep into
        // the block's x = -0.5 face — the cap-bite configuration).
        let (mut store, mut geo) = stores();
        let a = block_at(&mut store, &mut geo, (2.0, 2.0, 2.0), (-2.5, -1.0, -1.0));
        let b = primitives::sphere(&mut store, &mut geo, 1.0).expect("valid sphere");
        let out = unite(&store, &geo, a, b, &tol()).expect("cap-bite union succeeds");
        assert!(
            out.check().is_empty(),
            "union of block and sphere cap must be a valid solid"
        );
    }

    #[test]
    fn coincident_faces_are_not_implemented() {
        // Blocks sharing the x = 1 plane: coincident faces.
        let (mut store, mut geo) = stores();
        let a = block_at(&mut store, &mut geo, (2.0, 2.0, 2.0), (0.0, 0.0, 0.0));
        let b = block_at(&mut store, &mut geo, (2.0, 2.0, 2.0), (2.0, 0.0, 0.0));
        let err = unite(&store, &geo, a, b, &tol()).unwrap_err();
        assert!(
            matches!(err, CoreError::NotImplemented { .. }),
            "expected NotImplemented for coincident faces, got {err:?}"
        );
        assert!(
            err.to_string().contains("coincident"),
            "unhelpful error: {err}"
        );
    }

    #[test]
    fn tangent_contact_is_not_implemented() {
        // Cylinder wall tangent to the block face plane x = 1.
        let (mut store, mut geo) = stores();
        let block = block_at(&mut store, &mut geo, (2.0, 2.0, 2.0), (0.0, 0.0, 0.0));
        let tool = cylinder_at(&mut store, &mut geo, 1.0, 4.0, (2.0, 0.0, 0.0));
        let err = subtract(&store, &geo, block, tool, &tol()).unwrap_err();
        assert!(
            matches!(err, CoreError::NotImplemented { .. }),
            "expected NotImplemented for tangent contact, got {err:?}"
        );
        assert!(
            err.to_string().contains("tangent"),
            "unhelpful error: {err}"
        );
    }

    /// A cylinder-wall cover polygon (one rectangular loop in `(u, v)`)
    /// whose seam meridian sits at `seam_u`, plus the matching chart.
    fn wall_poly(radius: f64, seam_u: f64, half_height: f64) -> FaceRegionPoly {
        let origin = Point3::new(0.0, 0.0, 0.0);
        let axis = Vector3::new(0.0, 0.0, 1.0);
        let e_u = Vector3::new(1.0, 0.0, 0.0);
        let e_v = Vector3::new(0.0, 1.0, 0.0);
        let corner = |u: f64, v: f64| {
            let p = origin + (e_u * u.cos() + e_v * u.sin()) * radius + axis * v;
            ((u, v), p)
        };
        FaceRegionPoly {
            chart: Chart::Cylinder {
                origin,
                axis,
                e_u,
                e_v,
                radius,
            },
            loops: vec![vec![
                corner(seam_u, -half_height),
                corner(seam_u + TWO_PI, -half_height),
                corner(seam_u + TWO_PI, half_height),
                corner(seam_u, half_height),
            ]],
        }
    }

    #[test]
    fn contains_for_clip_is_stable_on_the_seam_meridian() {
        // Points exactly on a cylinder cover's seam meridian sit on the
        // cover polygon's boundary, where plain even-odd containment is a
        // float coin flip (of-ipt.5). The clip-purpose test must read
        // them as inside — in every equivalent angle spelling — while
        // still rejecting points genuinely outside the region.
        let snap = 8e-9;
        let fp = wall_poly(0.5, 0.0, 4.0);
        for u in [0.0, -0.0, TWO_PI, -TWO_PI] {
            assert!(
                fp.contains_for_clip((u, 1.0), snap),
                "seam-meridian point (u = {u}) must be inside"
            );
        }
        // Interior point stays inside; outside-v stays outside in every
        // angle spelling, seam or not.
        assert!(fp.contains_for_clip((std::f64::consts::PI, 0.0), snap));
        for u in [0.0, std::f64::consts::PI, TWO_PI] {
            assert!(
                !fp.contains_for_clip((u, 4.5), snap),
                "point beyond the wall's v range (u = {u}) must stay outside"
            );
        }
    }

    /// Sample a closed curve exactly as [`clip_imprint`] does for a full
    /// ring: `SAMPLES_PER_CIRCLE` points uniform over one period.
    fn ring_samples(curve: &Curve3) -> Vec<Point3> {
        (0..SAMPLES_PER_CIRCLE)
            .map(|i| curve.point(TWO_PI * i as f64 / SAMPLES_PER_CIRCLE as f64))
            .collect()
    }

    #[test]
    fn seam_crossing_lands_on_curve_for_phase_misaligned_ring() {
        // A circular ring on an r = 25 wall, parameterized with its basis
        // rotated 0.3 rad against the chart (an equal-radius ellipse):
        // samples straddle the seam instead of landing on it, so a chord
        // point would be off the curve by the sagitta ≈ 5.35e-4 * 25 ≈
        // 0.013 > MAX_ALLOWED_TOLERANCE. The refined point must sit on
        // the curve at exactly the seam angle.
        let r = 25.0;
        let phase: f64 = 0.3;
        let fp = wall_poly(r, -std::f64::consts::PI, 2.0 * r);
        let curve = Curve3::Ellipse {
            center: Point3::new(0.0, 0.0, 0.0),
            axis: Vector3::new(0.0, 0.0, 1.0),
            major_dir: Vector3::new(phase.cos(), phase.sin(), 0.0),
            major_radius: r,
            minor_radius: r,
        };
        let points = ring_samples(&curve);
        let crossings = seam_crossings(&fp, &curve, &points, SeamAxis::U);
        assert_eq!(crossings.len(), 1, "wrap-once ring crosses the seam once");
        let p = crossings[0];
        // Seam angle u = π on this ring is curve parameter t = π - phase.
        let expected = curve.point(std::f64::consts::PI - phase);
        assert!(
            (p - expected).norm() < 1e-9,
            "seam vertex off the exact curve by {:.3e}",
            (p - expected).norm()
        );
    }

    #[test]
    fn seam_crossing_lands_on_curve_for_tilted_section_ellipse() {
        // Tilted-plane section of an r = 50 cylinder (the ellipse-imprint
        // shape a tilted tool would produce): semi-minor r on the wall,
        // semi-major r / cos(alpha). The seam is placed off the sample
        // grid so linear interpolation would err by ~0.02.
        let r = 50.0;
        let alpha: f64 = 0.5;
        let seam_u = -3.0; // not a multiple of the 2π/96 sample spacing
        let fp = wall_poly(r, seam_u, 2.0 * r);
        let curve = Curve3::Ellipse {
            center: Point3::new(0.0, 0.0, 0.0),
            axis: Vector3::new(alpha.sin(), 0.0, alpha.cos()),
            major_dir: Vector3::new(alpha.cos(), 0.0, -alpha.sin()),
            major_radius: r / alpha.cos(),
            minor_radius: r,
        };
        let points = ring_samples(&curve);
        let crossings = seam_crossings(&fp, &curve, &points, SeamAxis::U);
        assert_eq!(crossings.len(), 1, "wrap-once ring crosses the seam once");
        let p = crossings[0];
        // This section satisfies u(t) = t, so the seam angle is hit at
        // t = seam_u (mod 2π).
        let expected = curve.point(seam_u);
        assert!(
            (p - expected).norm() < 1e-9,
            "seam vertex off the exact curve by {:.3e}",
            (p - expected).norm()
        );
    }

    // -----------------------------------------------------------------
    // Sphere pole closure (of-7ld.5)
    // -----------------------------------------------------------------

    fn unit_sphere_chart() -> Chart {
        Chart::build(&Surface3::sphere(Point3::origin(), Vector3::z(), 1.0).expect("valid"))
            .expect("sphere charts build")
    }

    /// 3D samples of the `u = 0` seam meridian of the unit sphere, south
    /// pole to north pole inclusive (`n + 1` points).
    fn seam_samples(n: usize) -> Vec<Point3> {
        (0..=n)
            .map(|i| {
                let v = -FRAC_PI_2 + PI * i as f64 / n as f64;
                Point3::new(v.cos(), 0.0, v.sin())
            })
            .collect()
    }

    /// Embed a walk with a [`CoverEmbedder`] and return the cover.
    fn embed_points(chart: &Chart, points: &[Point3], ccw: bool) -> Vec<CoverPoint> {
        let mut emb = CoverEmbedder::new(chart, ccw);
        let mut out = Vec::new();
        for p in points {
            emb.push(*p, &mut out);
        }
        out
    }

    /// The sphere face's stored loop walk: up the seam, back down it
    /// (each traversal dropping its final point, as `append_directed`
    /// does when concatenating loop edges).
    fn seam_only_loop_walk() -> Vec<Point3> {
        let pts = seam_samples(48);
        let mut walk: Vec<Point3> = pts[..pts.len() - 1].to_vec();
        walk.extend(pts[1..].iter().rev());
        walk
    }

    #[test]
    fn pole_v_flags_poles_only_on_sphere_charts() {
        let chart = unit_sphere_chart();
        assert_eq!(chart.pole_v(&Point3::new(0.0, 0.0, 1.0)), Some(FRAC_PI_2));
        assert_eq!(chart.pole_v(&Point3::new(0.0, 0.0, -1.0)), Some(-FRAC_PI_2));
        assert_eq!(chart.pole_v(&Point3::new(1.0, 0.0, 0.0)), None);
        let wall = wall_poly(1.0, 0.0, 1.0);
        assert_eq!(wall.chart.pole_v(&Point3::new(0.0, 0.0, 1.0)), None);
    }

    #[test]
    fn seam_only_sphere_loop_covers_the_full_rectangle() {
        // A closed sphere face's only boundary is the seam meridian: its
        // cover polygon is two coincident vertical traversals unless the
        // pole closure rows are embedded explicitly — the of-7ld.5
        // zero-area collapse that broke every plane-sphere boolean.
        let chart = unit_sphere_chart();
        let cover = embed_points(&chart, &seam_only_loop_walk(), true);
        let area = shoelace(&cover);
        assert!(
            (area - TWO_PI * PI).abs() < 1e-6,
            "CCW cover must have the full 2π·π rectangle area, got {area}"
        );
        // Pole rows never affect even-odd containment (horizontal
        // segments), so any latitude/longitude interior probe must land
        // inside the cover.
        let fp = FaceRegionPoly {
            chart,
            loops: vec![cover],
        };
        for (u, v) in [(0.1, 0.0), (PI, 1.2), (TWO_PI - 0.1, -1.4)] {
            assert!(
                fp.contains(fp.localize((u, v))),
                "({u}, {v}) must be inside the full-sphere cover"
            );
        }
    }

    #[test]
    fn seam_only_sphere_loop_cover_orients_by_winding() {
        // Stored loops of a face whose outward side opposes the surface
        // normal (a dimple) wind CW in the chart; the pole rows must flip
        // with them so the cover polygon stays a simple rectangle.
        let chart = unit_sphere_chart();
        let cover = embed_points(&chart, &seam_only_loop_walk(), false);
        let area = shoelace(&cover);
        assert!(
            (area + TWO_PI * PI).abs() < 1e-6,
            "CW cover must have area -2π·π, got {area}"
        );
    }

    #[test]
    fn pole_row_sweeps_exactly_the_interior_meridians() {
        // Walk up the u = π meridian, over the north pole, down the
        // u = 0 meridian (a straight pole crossing): CCW keeps the face
        // interior left of the walk, so the north row runs from the
        // arrival meridian toward -u and must stop at u = 0, not at the
        // nearest-unwrap pick of ±π.
        let chart = unit_sphere_chart();
        let up: Vec<Point3> = (0..=24)
            .map(|i| {
                let v = -FRAC_PI_2 + PI * i as f64 / 24.0;
                Point3::new(-v.cos(), 0.0, v.sin())
            })
            .collect();
        let down: Vec<Point3> = (1..=24)
            .map(|i| {
                let v = FRAC_PI_2 - PI * i as f64 / 24.0;
                Point3::new(v.cos(), 0.0, v.sin())
            })
            .collect();
        let mut walk = up;
        walk.extend(down);
        let cover = embed_points(&chart, &walk, true);
        let north = Point3::new(0.0, 0.0, 1.0);
        let row: Vec<f64> = cover
            .iter()
            .filter(|(_, p)| (p - north).norm() < 1e-12)
            .map(|((u, _), _)| *u)
            .collect();
        assert_eq!(row.len(), 2, "pole must carry arrival and departure");
        assert!(
            (row[0] - PI).abs() < 1e-9 && row[1].abs() < 1e-9,
            "north row must run π → 0 (interior meridians), got {row:?}"
        );
    }

    #[test]
    fn region_interior_point_found_on_full_sphere_cover() {
        // The exact of-7ld.5 failure: with the collapsed cover no region
        // interior sample existed for a closed sphere face.
        let chart = unit_sphere_chart();
        let cover = embed_points(&chart, &seam_only_loop_walk(), true);
        let cycle = Cycle {
            darts: vec![(0, true)],
            area: shoelace(&cover),
            dart_offsets: vec![0],
            poly: cover,
        };
        let region = Region {
            cycles: vec![cycle],
        };
        let p = region_interior_point(&chart, &region)
            .expect("full-sphere region must yield an interior sample");
        assert!(
            (p.coords.norm() - 1.0).abs() < 1e-9,
            "sample must lie on the sphere, |p| = {}",
            p.coords.norm()
        );
    }

    #[test]
    fn embed_cycle_vertex_offsets_skip_pole_row_points() {
        // Seam split one sample above the south pole (the tiny-cap graze
        // shape): the dart after the pole junction starts at the split
        // vertex, and its recorded cycle vertex must be that split point
        // — not the pole-row departure point emitted just before it,
        // which sits half a latitude range away and made
        // `match_chain_to_cycle` reject valid seam chords.
        let chart = unit_sphere_chart();
        let pts = seam_samples(48);
        let v_x = -FRAC_PI_2 + PI / 48.0;
        let a0 = Atom {
            points: pts[..2].to_vec(), // south pole → split
            closed: false,
        };
        let a1 = Atom {
            points: pts[1..].to_vec(), // split → north pole
            closed: false,
        };
        let atoms = [a0, a1];
        let walk = seam_only_loop_walk();
        let fp = FaceRegionPoly {
            loops: vec![embed_points(&chart, &walk, true)],
            chart,
        };
        let cycle = embed_cycle(
            &fp,
            &atoms,
            vec![(0, true), (1, true), (1, false), (0, false)],
        );
        assert!(
            (cycle.area - TWO_PI * PI).abs() < 1e-6,
            "split seam cycle still covers the rectangle, got {}",
            cycle.area
        );
        assert_eq!(cycle.dart_offsets.len(), 4);
        // Dart 1 starts at the split vertex on the up-seam copy, dart 3
        // on the down-seam copy — one period apart at latitude v_x.
        let uv1 = cycle.poly[cycle.dart_offsets[1]].0;
        let uv3 = cycle.poly[cycle.dart_offsets[3]].0;
        assert!(
            (uv1.1 - v_x).abs() < 1e-9 && (uv3.1 - v_x).abs() < 1e-9,
            "dart vertices must sit at the split latitude, got {uv1:?} / {uv3:?}"
        );
        assert!(
            ((uv1.0 - uv3.0).abs() - TWO_PI).abs() < 1e-9,
            "the split vertex appears on both seam copies, got {uv1:?} / {uv3:?}"
        );
    }

    // -----------------------------------------------------------------
    // Imprints through pole vertices (of-rb4)
    // -----------------------------------------------------------------

    /// 3D samples of the unit-sphere meridian at longitude `u`, from
    /// latitude `v0` to `v1` inclusive (`n + 1` points).
    fn meridian_arc(u: f64, v0: f64, v1: f64, n: usize) -> Vec<Point3> {
        (0..=n)
            .map(|i| {
                let v = v0 + (v1 - v0) * i as f64 / n as f64;
                Point3::new(v.cos() * u.cos(), v.cos() * u.sin(), v.sin())
            })
            .collect()
    }

    /// The unit sphere face's cover polygon (seam-only loop).
    fn unit_sphere_poly() -> FaceRegionPoly {
        let chart = unit_sphere_chart();
        FaceRegionPoly {
            loops: vec![embed_points(&chart, &seam_only_loop_walk(), true)],
            chart,
        }
    }

    #[test]
    fn match_chain_anchors_pole_endpoints_by_3d_coincidence() {
        // A pole-to-pole chord carries its own longitude, while the
        // cycle's pole vertex is stored at one arbitrary representative
        // of the collapsed pole row — the uv metric can never match
        // them. The 3D pole test must.
        let fp = unit_sphere_poly();
        let atoms = [
            Atom {
                points: seam_samples(24),
                closed: false,
            },
            Atom {
                points: meridian_arc(FRAC_PI_2, -FRAC_PI_2, FRAC_PI_2, 24),
                closed: false,
            },
        ];
        let cycle = embed_cycle(&fp, &atoms, vec![(0, true), (0, false)]);
        let (chain_poly, _) = embed_walk(&fp, &atoms, &[(1, true)], true);
        let got = match_chain_to_cycle(
            &cycle,
            &chain_poly,
            (Some(TWO_PI), None),
            (1.0, 1.0),
            &fp.chart,
        );
        assert_eq!(
            got.first(),
            Some(&(0, 1)),
            "chord must anchor at the south (vertex 0) and north (vertex 1) poles"
        );
    }

    #[test]
    fn match_chain_allows_closed_loop_on_a_single_pole_vertex() {
        // An imprint network that closes on itself AT a pole (the octant
        // corner case) matches the same cycle vertex at both ends; the
        // i != j guard must not reject it.
        let fp = unit_sphere_poly();
        let south = Point3::new(0.0, 0.0, -1.0);
        let mut loop_pts = meridian_arc(FRAC_PI_2, -FRAC_PI_2, -0.3, 8);
        loop_pts.extend(meridian_arc(PI, -0.3, -FRAC_PI_2, 8));
        loop_pts.push(south);
        let atoms = [
            Atom {
                points: seam_samples(24),
                closed: false,
            },
            Atom {
                points: loop_pts,
                closed: false,
            },
        ];
        let cycle = embed_cycle(&fp, &atoms, vec![(0, true), (0, false)]);
        let (chain_poly, _) = embed_walk(&fp, &atoms, &[(1, true)], true);
        let got = match_chain_to_cycle(
            &cycle,
            &chain_poly,
            (Some(TWO_PI), None),
            (1.0, 1.0),
            &fp.chart,
        );
        assert_eq!(
            got.first(),
            Some(&(0, 0)),
            "closed loop must anchor twice at the south pole vertex"
        );
    }

    #[test]
    fn imprint_chains_terminate_at_pole_junctions() {
        // Two arcs sharing a pole endpoint: the pole is an existing
        // topology vertex, fed to chain merging as a barrier point, so
        // they stay separate chains; without the barrier the same
        // junction is interior and they merge.
        let atoms = [
            Atom {
                points: meridian_arc(0.0, -FRAC_PI_2, 0.0, 8),
                closed: false,
            },
            Atom {
                points: meridian_arc(FRAC_PI_2, -FRAC_PI_2, 0.0, 8),
                closed: false,
            },
        ];
        let snap = 1e-9;
        let south = Point3::new(0.0, 0.0, -1.0);
        let chains = merge_imprint_chains(&atoms, &[0, 1], snap, &[south]);
        assert_eq!(chains.len(), 2, "pole junction must terminate both chains");
        let chains = merge_imprint_chains(&atoms, &[0, 1], snap, &[]);
        assert_eq!(chains.len(), 1, "interior junction must merge the chains");
    }

    #[test]
    fn full_wrap_open_imprint_has_interior_poles() {
        // A ring cut open at a single point (the clip splitting a closed
        // imprint exactly at one pole) spans the whole period with
        // coincident endpoints: parameters away from the cut — like the
        // OTHER pole — are interior and must be split; the cut itself is
        // not.
        let curve = Curve3::circle(Point3::origin(), Vector3::x(), 1.0).expect("valid");
        let north = Point3::new(0.0, 0.0, 1.0);
        let south = Point3::new(0.0, 0.0, -1.0);
        let t_n = curve.project_point(&north).t;
        let points: Vec<Point3> = (0..=96)
            .map(|i| curve.point(t_n + TWO_PI * i as f64 / 96.0))
            .collect();
        let imp = Imprint {
            face_a: 0,
            face_b: 0,
            curve,
            sampled: SampledCurve {
                points,
                closed: false,
            },
        };
        let t_s = imp.curve.project_point(&south).t;
        let snap = 1e-9;
        assert!(
            Pipeline::interior_curve_param(&imp, t_s, snap),
            "the far pole lies mid-run and must be split"
        );
        assert!(
            !Pipeline::interior_curve_param(&imp, t_n, snap),
            "the cut point is the run boundary, not an interior split"
        );
    }

    #[test]
    fn embed_cycle_starting_at_pole_uses_arrival_meridian() {
        // A region cycle whose first dart leaves a pole has no arrival
        // longitude when the pole is embedded; the placeholder must be
        // fixed up to the walk's closing meridian or the pole row sweeps
        // meridians outside the region and the cover overlaps its
        // neighbors. Band between the u = π/2 and u = 3π/2 meridians,
        // walked up the 3π/2 side: rows must span exactly [π/2, 3π/2].
        let fp = unit_sphere_poly();
        let atoms = [
            Atom {
                points: meridian_arc(3.0 * FRAC_PI_2, -FRAC_PI_2, FRAC_PI_2, 24),
                closed: false,
            },
            Atom {
                points: meridian_arc(FRAC_PI_2, FRAC_PI_2, -FRAC_PI_2, 24),
                closed: false,
            },
        ];
        let cycle = embed_cycle(&fp, &atoms, vec![(0, true), (1, true)]);
        assert!(
            (cycle.area - PI * PI).abs() < 1e-6,
            "band area must be π·π, got {}",
            cycle.area
        );
        let (mut lo_u, mut hi_u) = (f64::INFINITY, f64::NEG_INFINITY);
        for ((u, _), _) in &cycle.poly {
            lo_u = lo_u.min(*u);
            hi_u = hi_u.max(*u);
        }
        assert!(
            (hi_u - lo_u - PI).abs() < 1e-6,
            "cover must span exactly one π-wide band, got [{lo_u}, {hi_u}]"
        );
        // Even-odd containment: a meridian inside the band is in, one
        // outside (which the unfixed placeholder row would swallow) out.
        let mid = localize_to_window(&cycle.poly, (PI, 0.0), (Some(TWO_PI), None));
        assert!(point_in_cycle(&cycle, mid), "u = π must be inside the band");
        let out = localize_to_window(&cycle.poly, (0.1, 0.0), (Some(TWO_PI), None));
        assert!(
            !point_in_cycle(&cycle, out),
            "u = 0.1 must be outside the band"
        );
    }

    #[test]
    fn seam_crossing_found_for_wrapping_sphere_cap_ring() {
        // A latitude cap circle wraps the sphere's u period once, so the
        // seam split machinery must cut it (and later the seam edge) at
        // the seam meridian — the sphere analog of the cylinder ring
        // case, enabled by the full-rectangle cover (of-7ld.5).
        let chart = unit_sphere_chart();
        let fp = FaceRegionPoly {
            loops: vec![embed_points(&chart, &seam_only_loop_walk(), true)],
            chart,
        };
        let v0: f64 = -0.4;
        let curve = Curve3::circle(Point3::new(0.0, 0.0, v0.sin()), Vector3::z(), v0.cos())
            .expect("valid cap circle");
        let points = ring_samples(&curve);
        let crossings = seam_crossings(&fp, &curve, &points, SeamAxis::U);
        assert_eq!(crossings.len(), 1, "cap ring wraps the period once");
        let p = crossings[0];
        assert!(
            (p - Point3::new(v0.cos(), 0.0, v0.sin())).norm() < 1e-9,
            "seam vertex must sit on the u = 0 meridian at the cap latitude, got {p:?}"
        );
    }

    /// A torus face's full `[0, 2π]²` cover (the fundamental square of the
    /// one-face torus primitives build), R = 2, r = 0.5, axis +Z.
    fn full_torus_poly() -> FaceRegionPoly {
        let chart = Chart::Torus {
            center: Point3::new(0.0, 0.0, 0.0),
            axis: Vector3::new(0.0, 0.0, 1.0),
            e_u: Vector3::new(1.0, 0.0, 0.0),
            e_v: Vector3::new(0.0, 1.0, 0.0),
            major_radius: 2.0,
            minor_radius: 0.5,
        };
        let corners = [(0.0, 0.0), (TWO_PI, 0.0), (TWO_PI, TWO_PI), (0.0, TWO_PI)];
        let lp = corners
            .iter()
            .map(|&uv| (uv, chart_point(&chart, uv)))
            .collect();
        FaceRegionPoly {
            chart,
            loops: vec![lp],
        }
    }

    #[test]
    fn seam_crossing_torus_latitude_ring_cuts_minor_seam_only() {
        // Latitude circle from the plane z = 0.3 (outer branch, radius
        // R + sqrt(r² − 0.3²) = 2.4): wraps `u`, constant `v`. Its basis is
        // rotated 0.3 rad against the chart so samples straddle the seam
        // (an equal-radius ellipse, as in the cylinder tests). It must cut
        // the minor (u = 0) seam at (2.4, 0, 0.3) exactly — and must NOT
        // report a `v` crossing (of-7ld.7).
        let fp = full_torus_poly();
        let phase: f64 = 0.3;
        let curve = Curve3::Ellipse {
            center: Point3::new(0.0, 0.0, 0.3),
            axis: Vector3::new(0.0, 0.0, 1.0),
            major_dir: Vector3::new(phase.cos(), phase.sin(), 0.0),
            major_radius: 2.4,
            minor_radius: 2.4,
        };
        let points = ring_samples(&curve);
        let crossings = seam_crossings(&fp, &curve, &points, SeamAxis::U);
        assert_eq!(crossings.len(), 1, "latitude ring wraps u once");
        let p = crossings[0];
        let expected = Point3::new(2.4, 0.0, 0.3);
        assert!(
            (p - expected).norm() < 1e-9,
            "u-seam vertex off the minor seam by {:.3e}",
            (p - expected).norm()
        );
        assert!(
            seam_crossings(&fp, &curve, &points, SeamAxis::V).is_empty(),
            "constant-v ring must not report a v-seam crossing"
        );
    }

    #[test]
    fn seam_crossing_torus_tube_ring_cuts_major_seam_only() {
        // Tube cross-section circle at u = π/2 (center (0, R, 0), plane
        // x = 0): wraps `v`, constant `u`. It must cut the major (v = 0)
        // seam at the outer equator point (0, R + r, 0) — and must NOT
        // report a `u` crossing (of-7ld.7).
        let fp = full_torus_poly();
        let curve = Curve3::circle(Point3::new(0.0, 2.0, 0.0), Vector3::x(), 0.5)
            .expect("valid tube circle");
        let points = ring_samples(&curve);
        let crossings = seam_crossings(&fp, &curve, &points, SeamAxis::V);
        assert_eq!(crossings.len(), 1, "tube ring wraps v once");
        let p = crossings[0];
        let expected = Point3::new(0.0, 2.5, 0.0);
        assert!(
            (p - expected).norm() < 1e-9,
            "v-seam vertex off the outer equator by {:.3e}",
            (p - expected).norm()
        );
        assert!(
            seam_crossings(&fp, &curve, &points, SeamAxis::U).is_empty(),
            "constant-u ring must not report a u-seam crossing"
        );
    }

    /// A unit sphere face's full cover (axis +Z, seam meridian through +X):
    /// `[0, 2π] × [-π/2, π/2]`, as the one-face sphere primitive covers it.
    fn full_sphere_poly() -> FaceRegionPoly {
        let chart = Chart::Sphere {
            center: Point3::new(0.0, 0.0, 0.0),
            axis: Vector3::new(0.0, 0.0, 1.0),
            e_u: Vector3::new(1.0, 0.0, 0.0),
            e_v: Vector3::new(0.0, 1.0, 0.0),
            radius: 1.0,
        };
        let corners = [
            (0.0, -FRAC_PI_2),
            (TWO_PI, -FRAC_PI_2),
            (TWO_PI, FRAC_PI_2),
            (0.0, FRAC_PI_2),
        ];
        let lp = corners
            .iter()
            .map(|&uv| (uv, chart_point(&chart, uv)))
            .collect();
        FaceRegionPoly {
            chart,
            loops: vec![lp],
        }
    }

    #[test]
    fn seam_crossings_winding0_sphere_cap_reports_both() {
        // Cap boundary circle about +X (half-angle 0.6) on the unit
        // sphere: it straddles the u = 0 seam meridian without enclosing
        // the polar axis (u-winding 0), so it crosses the seam TWICE — at
        // latitudes ±0.6 on the +X meridian. The old first-crossing-only
        // logic returned nothing for winding-0 rings, leaving the ring
        // whole and its uv embedding straddling the cover edge (of-43n).
        let fp = full_sphere_poly();
        let alpha: f64 = 0.6;
        let curve = Curve3::circle(
            Point3::new(alpha.cos(), 0.0, 0.0),
            Vector3::x(),
            alpha.sin(),
        )
        .expect("valid cap circle");
        let points = ring_samples(&curve);
        let mut crossings = seam_crossings(&fp, &curve, &points, SeamAxis::U);
        assert_eq!(
            crossings.len(),
            2,
            "winding-0 straddling ring crosses twice"
        );
        crossings.sort_by(|a, b| a.z.total_cmp(&b.z));
        for (p, z) in crossings.iter().zip([-alpha.sin(), alpha.sin()]) {
            let expected = Point3::new(alpha.cos(), 0.0, z);
            assert!(
                (p - expected).norm() < 1e-9,
                "seam crossing off the +X meridian by {:.3e}",
                (p - expected).norm()
            );
        }
    }

    #[test]
    fn seam_crossings_empty_for_cap_clear_of_the_seam() {
        // The same cap rotated to +Y: u spans a band around π/2, nowhere
        // near a seam level instance, so no crossing may be reported.
        let fp = full_sphere_poly();
        let alpha: f64 = 0.6;
        let curve = Curve3::circle(
            Point3::new(0.0, alpha.cos(), 0.0),
            Vector3::y(),
            alpha.sin(),
        )
        .expect("valid cap circle");
        let points = ring_samples(&curve);
        assert!(
            seam_crossings(&fp, &curve, &points, SeamAxis::U).is_empty(),
            "cap away from the seam must not report a crossing"
        );
    }

    #[test]
    fn merge_imprint_chains_terminates_at_barriers() {
        // Two open atoms sharing BOTH endpoints (a ring seam-split at two
        // crossings). With the junctions marked as barriers each half must
        // stay its own boundary-to-boundary chord; without barriers they
        // re-merge into one full-ring chain (the correct behavior on the
        // imprint's OTHER host face, where the seam points are interior).
        let snap = 1e-9;
        let p1 = Point3::new(1.0, 0.0, -0.5);
        let p2 = Point3::new(1.0, 0.0, 0.5);
        let atoms = vec![
            Atom {
                points: vec![p1, Point3::new(0.8, 0.6, 0.0), p2],
                closed: false,
            },
            Atom {
                points: vec![p2, Point3::new(0.8, -0.6, 0.0), p1],
                closed: false,
            },
        ];
        let split = merge_imprint_chains(&atoms, &[0, 1], snap, &[p1, p2]);
        assert_eq!(split.len(), 2, "barriers keep the chords separate");
        assert!(split.iter().all(|c| c.len() == 1));
        let merged = merge_imprint_chains(&atoms, &[0, 1], snap, &[]);
        assert_eq!(merged.len(), 1, "no barriers: halves re-merge");
        assert_eq!(merged[0].len(), 2);
    }

    #[test]
    fn match_chain_accepts_whole_period_shift_in_v() {
        // A v-wrapping seam chord embedded one whole v-period above its
        // host cycle's window (as a torus tube imprint can be after cover
        // alignment) must still match the two cover copies of its seam
        // vertex (of-7ld.7).
        let p = Point3::new(0.0, 0.0, 0.0);
        let verts = [
            (0.0, 0.0),
            (FRAC_PI_2, 0.0),
            (TWO_PI, 0.0),
            (TWO_PI, TWO_PI),
            (FRAC_PI_2, TWO_PI),
            (0.0, TWO_PI),
        ];
        let poly: Vec<CoverPoint> = verts.iter().map(|&uv| (uv, p)).collect();
        let cycle = Cycle {
            darts: vec![(0, true); 6],
            area: shoelace(&poly),
            dart_offsets: vec![0, 1, 2, 3, 4, 5],
            poly,
        };
        let chain = [((FRAC_PI_2, TWO_PI), p), ((FRAC_PI_2, 2.0 * TWO_PI), p)];
        let got = match_chain_to_cycle(
            &cycle,
            &chain,
            (Some(TWO_PI), Some(TWO_PI)),
            (2.5, 0.5),
            &poleless_chart(),
        );
        assert_eq!(
            got.first(),
            Some(&(1, 4)),
            "v-period shift must recover the chord"
        );
    }

    #[test]
    fn ray_torus_hits_are_parity_correct() {
        let center = Point3::new(0.0, 0.0, 0.0);
        let axis = Vector3::new(0.0, 0.0, 1.0);
        // Diameter ray through both tube lobes: four transversal hits at
        // x = ±1.5, ±2.5.
        let hits = ray_torus_hits(
            &center,
            &axis,
            2.0,
            0.5,
            &Point3::new(-5.0, 0.0, 0.0),
            &Vector3::new(1.0, 0.0, 0.0),
        );
        assert_eq!(hits.len(), 4, "expected 4 hits, got {hits:?}");
        let mut xs: Vec<f64> = hits.iter().map(|t| -5.0 + t).collect();
        xs.sort_by(f64::total_cmp);
        for (x, expected) in xs.iter().zip([-2.5, -1.5, 1.5, 2.5]) {
            assert!(
                (x - expected).abs() < 1e-9,
                "hit at x = {x}, expected {expected}"
            );
        }
        // Ray up the axis through the bore: no hits.
        let hits = ray_torus_hits(
            &center,
            &axis,
            2.0,
            0.5,
            &Point3::new(0.0, 0.0, -5.0),
            &Vector3::new(0.0, 0.0, 1.0),
        );
        assert!(hits.is_empty(), "bore ray must miss, got {hits:?}");
    }

    #[test]
    fn match_chain_uses_arc_length_metric_on_cylinder_charts() {
        // r = 1000: 0.005 rad of u is 5 model units of arc, while vertex B
        // sits only 0.008 units away along v. A uv-euclidean metric
        // mis-ranks A (0.005 "closer" than 0.008); the arc-length metric
        // must pick B (of-9n8).
        let p = Point3::new(0.0, 0.0, 0.0);
        let verts = [
            (0.0, 0.0),      // A: 5 units of arc from the chain start
            (0.005, 0.008),  // B: 0.008 units from the chain start
            (0.005, 1000.0), // C
            (0.0, 1000.0),   // D: nearest to the chain end
        ];
        let poly: Vec<CoverPoint> = verts.iter().map(|&uv| (uv, p)).collect();
        let cycle = Cycle {
            darts: vec![(0, true); 4],
            area: shoelace(&poly),
            dart_offsets: vec![0, 1, 2, 3],
            poly,
        };
        let chain = [((0.005, 0.0), p), ((0.0, 999.995), p)];
        let got = match_chain_to_cycle(
            &cycle,
            &chain,
            (None, None),
            (1000.0, 1.0),
            &poleless_chart(),
        );
        assert_eq!(
            got.first(),
            Some(&(1, 3)),
            "arc-length metric must prefer B over A"
        );
    }

    #[test]
    fn interior_point_found_in_grazing_sliver_band() {
        // A full-period band between two sinusoids 1e-4 units apart (the
        // uv shape of a grazing tilted cut on an r = 500 wall). Isotropic
        // probe offsets scaled by the summed extents (≈ 2π + 2) overshoot
        // the band from every segment; per-axis offsets must not (of-9n8).
        let chart = Chart::Cylinder {
            origin: Point3::new(0.0, 0.0, 0.0),
            axis: Vector3::new(0.0, 0.0, 1.0),
            e_u: Vector3::new(1.0, 0.0, 0.0),
            e_v: Vector3::new(0.0, 1.0, 0.0),
            radius: 500.0,
        };
        let h = 1e-4;
        let n = 96;
        let mut poly: Vec<CoverPoint> = Vec::new();
        for i in 0..=n {
            let u = TWO_PI * f64::from(i) / f64::from(n);
            let uv = (u, u.sin());
            poly.push((uv, chart_point(&chart, uv)));
        }
        for i in (0..=n).rev() {
            let u = TWO_PI * f64::from(i) / f64::from(n);
            let uv = (u, u.sin() + h);
            poly.push((uv, chart_point(&chart, uv)));
        }
        let cycle = Cycle {
            darts: vec![(0, true)],
            area: shoelace(&poly),
            dart_offsets: vec![0],
            poly,
        };
        assert!(cycle.area > 0.0, "band winds CCW");
        let region = Region {
            cycles: vec![cycle],
        };
        let p = region_interior_point(&chart, &region)
            .expect("sliver band must yield an interior sample");
        let radial = (p.x * p.x + p.y * p.y).sqrt();
        assert!(
            (radial - 500.0).abs() < 1e-9,
            "sample must lie on the chart surface, radial = {radial}"
        );
    }

    #[test]
    fn hole_probe_localizes_across_the_cover_edge() {
        // Cycles of one face are mean-aligned independently, so a hole
        // and its host can land a whole period apart when they hug
        // opposite edges of the cover window (of-9n8).
        let p = Point3::new(0.0, 0.0, 0.0);
        let square = [(2.6, 0.0), (3.6, 0.0), (3.6, 1.0), (2.6, 1.0)];
        let poly: Vec<CoverPoint> = square.iter().map(|&uv| (uv, p)).collect();
        let cycle = Cycle {
            darts: vec![(0, true); 4],
            area: shoelace(&poly),
            dart_offsets: vec![0, 1, 2, 3],
            poly,
        };
        let probe = (3.1 - TWO_PI, 0.5);
        assert!(
            !point_in_cycle(&cycle, probe),
            "raw probe sits outside the host's cover window"
        );
        let local = localize_to_window(&cycle.poly, probe, (Some(TWO_PI), None));
        assert!((local.0 - 3.1).abs() < 1e-12);
        assert_eq!(local.1, 0.5);
        assert!(point_in_cycle(&cycle, local));
        // Planar charts: no period, no shift.
        assert_eq!(localize_to_window(&cycle.poly, probe, (None, None)), probe);
    }

    #[test]
    fn subtract_grazing_cut_keeps_thin_band_on_tall_cylinder() {
        // of-9n8 stress: h = 2000, r = 500 cylinder; the block's bottom
        // plane grazes 1e-3 above the bottom cap, so the kept wall band
        // is a full-period sliver — 2π (radians) × 1e-3 (units) in uv.
        let (mut store, mut geo) = stores();
        let cyl = cylinder_at(&mut store, &mut geo, 500.0, 2000.0, (0.0, 0.0, 0.0));
        let band = 1e-3;
        let block = block_at(
            &mut store,
            &mut geo,
            (2400.0, 2400.0, 2001.0),
            (0.0, 0.0, -1000.0 + band + 1000.5),
        );
        let out = subtract(&store, &geo, cyl, block, &tol()).unwrap();
        assert_eq!(out.face_count(), 3, "bottom cap + wall band + new top cap");
        assert_eq!(out.shell_count(), 1);
        assert_valid(&out, "grazing cut on tall cylinder");
        assert_geometry_bound(&out, "grazing cut on tall cylinder");
        let mesh = out.tessellate().unwrap();
        let bb = mesh.bounding_box().expect("non-empty mesh");
        assert!((bb.min.z + 1000.0).abs() < 1e-6, "bottom cap preserved");
        assert!(
            (bb.max.z + 1000.0 - band).abs() < 1e-6,
            "sliver top sits at the graze plane, got {}",
            bb.max.z
        );
    }

    #[test]
    fn large_radius_through_hole_stays_within_tolerance_cap() {
        // r = 25 is past the radius where a chord-interpolated seam
        // vertex would exceed MAX_ALLOWED_TOLERANCE (r ≈ 19); the result
        // must still validate with honestly recorded tolerances.
        let (mut store, mut geo) = stores();
        let block = block_at(&mut store, &mut geo, (100.0, 100.0, 10.0), (0.0, 0.0, 0.0));
        let tool = cylinder_at(&mut store, &mut geo, 25.0, 20.0, (0.0, 0.0, 0.0));
        let out = subtract(&store, &geo, block, tool, &tol()).unwrap();
        assert_eq!(out.face_count(), 7, "6 block faces + 1 cylinder band");
        assert_valid(&out, "large block minus r=25 cylinder");
        assert_geometry_bound(&out, "large block minus r=25 cylinder");
        let counts = out.store.euler_counts(out.body);
        assert_eq!(counts.genus, 1, "through hole must give genus 1");
    }

    #[test]
    fn hole_bridge_avoids_unspliced_holes() {
        // of-299: the rightmost hole's nearest outer vertex lies on the
        // far side of the second (not yet spliced) hole. A bridge that is
        // only validated against the outer polygon and the current hole
        // cuts straight through the second hole, and splicing that hole
        // afterwards self-intersects the polygon, emitting overlapping
        // triangles.
        let ring = |uv: &[(f64, f64)]| MeshRing {
            uv: uv.to_vec(),
            points: uv.iter().map(|&(u, v)| Point3::new(u, v, 0.0)).collect(),
        };
        // Outer 10x2 rectangle (CCW).
        let outer = ring(&[(0.0, 0.0), (10.0, 0.0), (10.0, 2.0), (0.0, 2.0)]);
        // Rightmost hole: flat diamond around (2.5, 0.9), CW, edge slope
        // 0.2. Its max-u vertex (2.9, 0.9) is nearest to outer corner
        // (0, 0), and that bridge (slope 0.31) clears the flat diamond
        // itself but cuts straight through the second hole.
        let hole_a = ring(&[(2.9, 0.9), (2.5, 0.82), (2.1, 0.9), (2.5, 0.98)]);
        // Second hole: 0.7 square straddling the (2.9,0.9)-(0,0) segment, CW.
        let hole_b = ring(&[(0.45, 0.05), (0.45, 0.75), (1.15, 0.75), (1.15, 0.05)]);
        let mf = MeshFace {
            chart: Chart::Plane {
                origin: Point3::new(0.0, 0.0, 0.0),
                e_u: Vector3::new(1.0, 0.0, 0.0),
                e_v: Vector3::new(0.0, 1.0, 0.0),
                normal: Vector3::new(0.0, 0.0, 1.0),
            },
            rings: vec![outer, hole_a, hole_b],
            normal_sign: 1.0,
        };
        let (tris, dev) = triangulate_mesh_face(&mf, 1e-9).expect("face triangulates");
        assert_eq!(dev, 0.0, "planar face has no chordal deviation");
        // Covered area must equal outer minus both holes; overlapping or
        // inverted triangles from a self-intersecting splice break this.
        let area: f64 = tris
            .iter()
            .map(|t| {
                let ab = t.positions[1] - t.positions[0];
                let ac = t.positions[2] - t.positions[0];
                ab.cross(&ac).norm() * 0.5
            })
            .sum();
        let expected = 20.0 - 0.064 - 0.49;
        assert!(
            (area - expected).abs() < 1e-9,
            "triangulated area {area} != expected {expected}"
        );
    }

    #[test]
    fn snap_map_merges_points_straddling_cell_boundary() {
        // of-do9: two float-noise-identical points on opposite sides of a
        // quantization cell boundary. Exact single-cell keys split them;
        // the neighbor-probing SnapMap must merge them.
        let tol = 4e-9;
        let boundary = 0.5 * tol; // old scheme's cell edge for cell size `tol`
        let p1 = Point3::new(boundary - 1e-15, 1.0, -2.0);
        let p2 = Point3::new(boundary + 1e-15, 1.0, -2.0);
        assert_ne!(
            quantize(&p1, tol),
            quantize(&p2, tol),
            "test premise: the points straddle a single-cell boundary"
        );
        let mut map: SnapMap<u32> = SnapMap::new(tol);
        map.insert(p1, 7);
        assert_eq!(map.nearest(&p2), Some(7), "straddling point must match");

        // Sign-flip worst case: round() ties at ±0.5 cells away from zero.
        let q1 = Point3::new(-1e-15, 0.0, 0.0);
        let q2 = Point3::new(1e-15, 0.0, 0.0);
        let mut map: SnapMap<u32> = SnapMap::new(tol);
        map.insert(q1, 9);
        assert_eq!(map.nearest(&q2), Some(9));
    }

    #[test]
    fn snap_map_keeps_far_points_distinct_and_orders_by_distance() {
        let tol = 4e-9;
        let mut map: SnapMap<u32> = SnapMap::new(tol);
        map.insert(Point3::new(0.0, 0.0, 0.0), 1);
        map.insert(Point3::new(0.8 * tol, 0.0, 0.0), 2);
        map.insert(Point3::new(3.0 * tol, 0.0, 0.0), 3);
        assert_eq!(
            map.nearest(&Point3::new(3.0 * tol, 0.0, 0.0)),
            Some(3),
            "exact hit"
        );
        assert_eq!(
            map.nearest(&Point3::new(1.9 * tol, 0.0, 0.0)),
            None,
            "nothing within tol"
        );
        assert_eq!(
            map.matches(&Point3::new(0.1 * tol, 0.0, 0.0)),
            vec![1, 2],
            "in-tolerance values nearest first"
        );
    }

    /// of-2ql regression: FlipMesh::insert_on_edge splits both triangles of
    /// an interior edge into four valid CCW triangles with consistent
    /// adjacency and unchanged total area.
    #[test]
    fn flip_mesh_insert_on_edge_splits_quad_cleanly() {
        // Unit square as two triangles sharing the diagonal (0,0)-(1,1).
        let mut verts: Vec<(f64, f64)> = vec![(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)];
        let seed = [[0, 1, 2], [0, 2, 3]];
        let mut mesh = FlipMesh::from_tris(&seed, &verts);
        // Constrain the outer boundary so legalization cannot flip it.
        let constraints: std::collections::HashSet<(usize, usize)> =
            [(0, 1), (1, 2), (2, 3), (0, 3)].into_iter().collect();
        let t = 0;
        let k = mesh.edge_index(0, 0, 2).expect("diagonal edge");
        assert_ne!(mesh.adj[t][k], NO_TRI, "diagonal must be interior");
        verts.push((0.5, 0.5)); // exactly on the diagonal
        mesh.insert_on_edge(t, k, 4, &verts, &constraints);

        assert_eq!(mesh.tris.len(), 4);
        let mut area = 0.0;
        for (ti, tri) in mesh.tris.iter().enumerate() {
            let o = orient2d(verts[tri[0]], verts[tri[1]], verts[tri[2]]);
            assert!(o > 0.0, "triangle {ti} {tri:?} not CCW (orient {o})");
            area += 0.5 * o;
            // Adjacency must be mutual.
            for e in 0..3 {
                let n = mesh.adj[ti][e];
                if n != NO_TRI {
                    let (a, b) = (tri[e], tri[(e + 1) % 3]);
                    let back = mesh
                        .edge_index(n, a, b)
                        .unwrap_or_else(|| panic!("neighbor {n} misses edge ({a},{b})"));
                    assert_eq!(mesh.adj[n][back], ti, "adjacency not mutual");
                }
            }
        }
        assert!((area - 1.0).abs() < 1e-12, "area changed: {area}");
        // Every triangle uses the inserted vertex.
        assert!(mesh.tris.iter().all(|t| t.contains(&4)));
    }

    /// of-yud: the incircle predicate is correct on well-conditioned inputs
    /// and *stable* on the extreme-aspect cocircular quads a curved face's
    /// refinement produces — where the raw determinant is float noise. A
    /// cocircular point must read "not strictly inside" from every rotation,
    /// so neither diagonal of the quad is ever preferred and the Delaunay
    /// sweep cannot flip A→B→A forever.
    #[test]
    fn in_circle_filter_correct_and_stable() {
        // Well-conditioned: circumcircle of (0,0),(4,0),(0,4) is centred at
        // (2,2), radius √8 ≈ 2.83.
        let (a, b, c) = ((0.0, 0.0), (4.0, 0.0), (0.0, 4.0));
        assert!(in_circle(a, b, c, (1.0, 1.0)), "interior point is inside");
        assert!(!in_circle(a, b, c, (10.0, 10.0)), "far point is outside");
        // A vertex of the triangle sits exactly on the circle: not strictly
        // inside.
        assert!(!in_circle(a, b, c, (4.0, 4.0)), "cocircular 4th corner");

        // Extreme-aspect cocircular rectangle (1 wide, 1e-9 tall). Every
        // rotation is CCW and every test point is the rectangle's own 4th
        // corner, hence exactly on the circle. The raw determinant here is
        // ~1e-9 swamped by cancellation; the filtered predicate must report
        // `false` for all four so the flip loop stays put.
        let e = 1e-9;
        let quad = [(0.0, 0.0), (1.0, 0.0), (1.0, e), (0.0, e)];
        for r in 0..4 {
            let p0 = quad[r];
            let p1 = quad[(r + 1) % 4];
            let p2 = quad[(r + 2) % 4];
            let p3 = quad[(r + 3) % 4];
            assert!(
                orient2d(p0, p1, p2) > 0.0,
                "rotation {r} must present a CCW triangle"
            );
            assert!(
                !in_circle(p0, p1, p2, p3),
                "rotation {r}: cocircular corner must not read strictly inside"
            );
        }
    }

    /// of-yud: `make_delaunay` must converge in a handful of sweeps — not run
    /// to its 256-sweep cap — on a strip of extreme-aspect cocircular quads,
    /// the configuration a curved-face band tessellates into. With the raw
    /// incircle determinant these near-degenerate quads flip-cycled on float
    /// noise; the filtered predicate leaves them alone, so the sweep settles
    /// at once and the mesh stays a valid, area-preserving triangulation.
    #[test]
    fn make_delaunay_converges_on_extreme_aspect_strip() {
        let m = 16usize;
        let e = 1e-9;
        // Bottom vertex i -> index 2i at (i, 0); top vertex i -> 2i+1 at (i, e).
        let mut verts: Vec<(f64, f64)> = Vec::with_capacity(2 * (m + 1));
        for i in 0..=m {
            verts.push((i as f64, 0.0));
            verts.push((i as f64, e));
        }
        // Seed each thin cell with the same diagonal (bottom_i -> top_{i+1}).
        let mut seed = Vec::with_capacity(2 * m);
        for i in 0..m {
            seed.push([2 * i, 2 * i + 2, 2 * i + 3]);
            seed.push([2 * i, 2 * i + 3, 2 * i + 1]);
        }
        let mut mesh = FlipMesh::from_tris(&seed, &verts);
        // Constrain the whole boundary; only the interior cell diagonals are
        // free to flip.
        let mut constraints: std::collections::HashSet<(usize, usize)> =
            std::collections::HashSet::new();
        for i in 0..m {
            constraints.insert((2 * i, 2 * i + 2)); // bottom
            constraints.insert((2 * i + 1, 2 * i + 3)); // top
        }
        constraints.insert((0, 1)); // left end
        constraints.insert((2 * m, 2 * m + 1)); // right end

        let sweeps = mesh.make_delaunay(&verts, &constraints);
        assert!(
            sweeps <= 4,
            "make_delaunay ran {sweeps} sweeps — flip cycling is back"
        );

        // Mesh is still a valid triangulation: no inverted triangle, mutual
        // adjacency, total area unchanged (m thin rectangles of area e).
        let mut area = 0.0;
        for (ti, tri) in mesh.tris.iter().enumerate() {
            let o = orient2d(verts[tri[0]], verts[tri[1]], verts[tri[2]]);
            assert!(o >= 0.0, "triangle {ti} {tri:?} inverted (orient {o})");
            area += 0.5 * o;
            for k in 0..3 {
                let n = mesh.adj[ti][k];
                if n != NO_TRI {
                    let (x, y) = (tri[k], tri[(k + 1) % 3]);
                    let back = mesh
                        .edge_index(n, x, y)
                        .unwrap_or_else(|| panic!("neighbor {n} misses edge ({x},{y})"));
                    assert_eq!(mesh.adj[n][back], ti, "adjacency not mutual");
                }
            }
        }
        assert!(
            (area - m as f64 * e).abs() < 1e-18,
            "strip area changed: {area} vs {}",
            m as f64 * e
        );
    }

    /// of-2ql regression: sphere minus coaxial through-cylinder (napkin
    /// ring). The wall and band boundary polylines are sampled on the same
    /// angular pitch as the refinement lattice, so seed-chord diagonals pass
    /// exactly through staggered lattice points; splitting there used to
    /// mint negative uv slivers that blocked legalization and left secant
    /// triangles cutting ~radius deep through the cylinder wall (volume off
    /// 1.1e-2 relative). With on-edge insertion the tessellated volume must
    /// sit within the stress-suite budget of the closed form and the chordal
    /// deviation must stay near the one-pitch sag scale.
    #[test]
    fn napkin_ring_tessellation_volume_and_deviation() {
        let (mut store, mut geo) = stores();
        let ball = primitives::sphere(&mut store, &mut geo, 1.0).expect("valid sphere");
        let tool = primitives::cylinder(&mut store, &mut geo, 0.5, 4.0).expect("valid cylinder");
        let out = subtract(&store, &geo, ball, tool, &tol()).unwrap();
        assert_valid(&out, "napkin ring");

        let (mesh, deviation) = out.tessellate_measured().unwrap();
        assert!(
            deviation < 5e-3,
            "napkin ring chordal deviation {deviation:.3e} — secant triangles are back"
        );
        let mut volume = 0.0;
        for tri in &mesh.indices {
            let (a, b, c) = (
                mesh.positions[tri[0]],
                mesh.positions[tri[1]],
                mesh.positions[tri[2]],
            );
            volume += a.coords.dot(&b.coords.cross(&c.coords)) / 6.0;
        }
        let expected = 4.0 / 3.0 * PI * (1.0f64 - 0.25).powf(1.5);
        let rel = ((volume - expected) / expected).abs();
        assert!(
            rel <= 5e-3,
            "napkin ring volume {volume} vs {expected} — {rel:.3e} relative"
        );
    }

    #[test]
    fn boolean_output_valid_across_cell_boundary_sweep() {
        // of-do9 regression sweep: nudge the tool in sub-cell steps so
        // junction coordinates land at varying offsets relative to the
        // vertex-dedup quantization grid (cell size ≈ 1.6e-8 here). With
        // single-cell dedup, corners whose float-noise copies straddled a
        // cell boundary intermittently produced duplicate vertices (open /
        // non-manifold edges) or Degenerate chain-merge failures.
        for i in 0..16 {
            let dx = i as f64 * 1e-9;
            let (mut store, mut geo) = stores();
            let block = block_at(&mut store, &mut geo, (4.0, 4.0, 2.0), (2.0, 2.0, 1.0));
            let tool = cylinder_at(&mut store, &mut geo, 1.0, 4.0, (2.0 + dx, 2.0, 1.0));
            let out = subtract(&store, &geo, block, tool, &tol()).unwrap();
            let context = format!("sweep step {i} (dx = {dx:e})");
            assert_valid(&out, &context);
            assert_geometry_bound(&out, &context);
        }
    }
}
