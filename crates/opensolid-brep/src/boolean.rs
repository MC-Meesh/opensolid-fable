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
use crate::ssi::{IntersectionKind, SurfaceIntersection, intersect as ssi_intersect};
use crate::surface::Surface3;
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
    /// Trimmed faces are triangulated from their boundary samples alone
    /// (ear clipping), so a wide curved face — e.g. the full-wrap cylinder
    /// band left by a through-hole subtract — can be covered by long
    /// parameter-space chords that cut far inside the true surface while
    /// the mesh stays closed and manifold. The returned deviation is the
    /// largest distance from any triangle edge's 3D midpoint to the
    /// surface point at its parameter-space midpoint — use it to decide
    /// whether the mesh's geometric fidelity is acceptable (the kernel's
    /// hybrid boolean falls back to F-Rep meshing when it is not).
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
}

impl Chart {
    fn new(surface: &Surface3) -> CoreResult<Self> {
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
            other => Err(CoreError::NotImplemented {
                feature: match other {
                    Surface3::Cone { .. } => "boolean parameter chart for cones",
                    Surface3::Sphere { .. } => "boolean parameter chart for spheres",
                    Surface3::Torus { .. } => "boolean parameter chart for tori",
                    _ => unreachable!(),
                },
            }),
        }
    }

    /// Parameters of a point assumed to lie on the surface. For cylinders
    /// the angle is unwrapped to land within π of `hint`'s angle.
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
                let mut u = d.dot(e_v).atan2(d.dot(e_u));
                if let Some((hu, _)) = hint {
                    while u - hu > std::f64::consts::PI {
                        u -= TWO_PI;
                    }
                    while hu - u > std::f64::consts::PI {
                        u += TWO_PI;
                    }
                }
                (u, d.dot(axis))
            }
        }
    }

    /// Unit surface normal at parameters `(u, v)`.
    fn normal(&self, u: f64) -> Vector3 {
        match self {
            Chart::Plane { normal, .. } => *normal,
            Chart::Cylinder { e_u, e_v, .. } => e_u * u.cos() + e_v * u.sin(),
        }
    }
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
    /// exactly on the parameter cover's seam meridian (u = 0 / u = 2π on
    /// a cylinder) sits on the cover polygon's boundary, where the strict
    /// even-odd test is a float coin flip — yet such a point is
    /// geometrically interior to the face whenever its v is. Resolve by
    /// retrying with the angle nudged off the seam by a sub-tolerance
    /// step (`snap` expressed as an angle at the chart radius); a point
    /// genuinely outside the region stays outside under the nudge.
    fn contains_for_clip(&self, uv: (f64, f64), snap: f64) -> bool {
        let local = self.localize(uv);
        if self.contains(local) {
            return true;
        }
        let Chart::Cylinder { radius, .. } = self.chart else {
            return false;
        };
        let eps = (snap / radius).max(1e-12);
        self.contains(self.localize((local.0 + eps, local.1)))
            || self.contains(self.localize((local.0 - eps, local.1)))
    }

    /// Bring an angle-like `u` into this polygon's neighborhood by shifting
    /// whole periods (no-op for planes).
    fn localize(&self, uv: (f64, f64)) -> (f64, f64) {
        if !matches!(self.chart, Chart::Cylinder { .. }) {
            return uv;
        }
        let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
        for lp in &self.loops {
            for ((u, _), _) in lp {
                lo = lo.min(*u);
                hi = hi.max(*u);
            }
        }
        let center = 0.5 * (lo + hi);
        let mut u = uv.0;
        while u - center > std::f64::consts::PI {
            u -= TWO_PI;
        }
        while center - u > std::f64::consts::PI {
            u += TWO_PI;
        }
        (u, uv.1)
    }
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
    imprints: Vec<Imprint>,
    /// Split points per curve source, as 3D points.
    splits: HashMap<CurveSource, Vec<Point3>>,
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
                let chart = Chart::new(&face.surface)?;
                let mut loops = Vec::new();
                for lp in &face.loops {
                    let mut pts3: Vec<Point3> = Vec::new();
                    for de in lp {
                        let sampled = &edge_samples[s][de.edge];
                        append_directed(&mut pts3, sampled, de.forward);
                    }
                    let uv = map_polyline(&chart, &pts3);
                    loops.push(uv.into_iter().zip(pts3).collect());
                }
                face_polys[s].push(FaceRegionPoly { chart, loops });
            }
        }
        Ok(Self {
            solids,
            tol: *tol,
            snap,
            edge_samples,
            face_polys,
            imprints: Vec::new(),
            splits: HashMap::new(),
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
            match ssi_intersect(sa, sb, &self.tol)? {
                SurfaceIntersection::Empty => {}
                SurfaceIntersection::Coincident => {
                    return Err(CoreError::NotImplemented {
                        feature: "boolean operations on coincident faces \
                                  (transversal MVP)",
                    });
                }
                SurfaceIntersection::TangentPoint(_) => {
                    return Err(CoreError::NotImplemented {
                        feature: "boolean operations with tangent face contact \
                                  (transversal MVP)",
                    });
                }
                SurfaceIntersection::Curves(curves) => {
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
            }
        }
        Ok(())
    }

    fn face_boxes(&self, s: SolidTag) -> Vec<BoundingBox3> {
        self.face_polys[s]
            .iter()
            .map(|fp| {
                let bounds = BoundingBox3::from_points(fp.loops.iter().flatten().map(|&(_, p)| p));
                // Dilate: boundary samples underestimate curved interiors,
                // and touching contacts must still clash so they reach SSI
                // (which rejects them as tangent, not silently misses them).
                bounds.dilate(bounds.extents().norm() * 0.05 + self.tol.linear + self.snap)
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
        // Parameter window: full period for closed conics, bbox slab clip
        // for lines.
        let (t_lo, t_hi, closed_curve) = match curve {
            Curve3::Line { origin, dir } => {
                let joint = box_a.intersection(box_b);
                match clip_line_to_box(origin, dir, &joint) {
                    Some(range) => (range.0, range.1, false),
                    None => return,
                }
            }
            _ => (0.0, TWO_PI, true),
        };
        let n = if closed_curve {
            SAMPLES_PER_CIRCLE
        } else {
            IMPRINT_LINE_SAMPLES
        };
        let count = if closed_curve { n } else { n + 1 };
        let ts: Vec<f64> = (0..count)
            .map(|i| t_lo + (t_hi - t_lo) * i as f64 / n as f64)
            .collect();
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
            if (closed_curve || first_idx > 0) && !flags[prev_idx] {
                let mut t_out = ts[prev_idx];
                let t_in = ts[first_idx];
                if closed_curve && t_out > t_in {
                    t_out -= TWO_PI;
                }
                pts.push(curve.point(refine_crossing(&inside, t_out, t_in)));
            }
            pts.extend(run.iter().map(|&i| curve.point(ts[i])));
            // Refine exit point (after the run's last sample).
            let last_idx = *run.last().expect("non-empty run");
            let next_idx = step(last_idx);
            if (closed_curve || last_idx + 1 < total) && !flags[next_idx] {
                let mut t_out = ts[next_idx];
                let t_in = ts[last_idx];
                if closed_curve && t_out < t_in {
                    t_out += TWO_PI;
                }
                pts.push(curve.point(refine_crossing(&inside, t_out, t_in)));
            }
            if pts.len() >= 2 && (pts[0] - pts[pts.len() - 1]).norm() > self.snap {
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

    /// Phase 3: register the global split events. Every open imprint
    /// endpoint lies on some original edge of one of the solids and splits
    /// it there. Closed imprints hosted on a periodic face additionally
    /// wrap that face's parameter cover; they are cut where they cross the
    /// face's seam meridian (splitting both the imprint and the seam edge)
    /// so the cover polygon sees them as boundary-to-boundary chords.
    fn collect_splits(&mut self) {
        let mut events: Vec<(CurveSource, Point3)> = Vec::new();
        for (ii, imp) in self.imprints.iter().enumerate() {
            if !imp.sampled.closed {
                for endpoint in [
                    imp.sampled.points[0],
                    imp.sampled.points[imp.sampled.points.len() - 1],
                ] {
                    for (s, f) in [(0usize, imp.face_a), (1usize, imp.face_b)] {
                        if let Some(edge) = self.nearest_edge_of_face(&endpoint, s, f) {
                            events.push((CurveSource::Edge { solid: s, edge }, endpoint));
                            break;
                        }
                    }
                }
                continue;
            }
            for (s, f) in [(0usize, imp.face_a), (1usize, imp.face_b)] {
                let fp = &self.face_polys[s][f];
                if !matches!(fp.chart, Chart::Cylinder { .. }) {
                    continue;
                }
                let Some(seam_point) = seam_crossing(fp, &imp.curve, &imp.sampled.points) else {
                    continue;
                };
                events.push((CurveSource::Imprint { index: ii }, seam_point));
                if let Some(edge) = self.nearest_edge_of_face(&seam_point, s, f) {
                    events.push((CurveSource::Edge { solid: s, edge }, seam_point));
                }
            }
        }
        for (source, p) in events {
            self.splits.entry(source).or_default().push(p);
        }
    }

    /// The boundary edge of face `(s, f)` nearest to `p`, if within
    /// acceptance distance (bisection leaves imprint endpoints on the
    /// region boundary; polyline sag bounds the residual).
    fn nearest_edge_of_face(&self, p: &Point3, s: SolidTag, f: usize) -> Option<usize> {
        let mut best: Option<(f64, usize)> = None;
        for lp in &self.solids[s].faces[f].loops {
            for de in lp {
                let sampled = &self.edge_samples[s][de.edge];
                let d = polyline_distance(&sampled.points, sampled.closed, p);
                if best.is_none() || d < best.expect("checked").0 {
                    best = Some((d, de.edge));
                }
            }
        }
        let (d, e) = best?;
        let accept = self.tol.linear.max(self.snap * 10.0).max(1e-5);
        (d <= accept).then_some(e)
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
                let splits = self.splits.get(&source).cloned().unwrap_or_default();
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

                // Initial region: the face's own loops, atom by atom.
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
                for chain in merge_imprint_chains(&atoms, &imprint_ids, self.snap) {
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
                    let n = fp.chart.normal(fp.chart.param(&hit, None).0);
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
        for lp in &self.solids[s].faces[f].loops {
            for de in lp {
                let sampled = &self.edge_samples[s][de.edge];
                if polyline_distance(&sampled.points, sampled.closed, p) < self.tol.linear * 10.0 {
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

/// Where a closed imprint on a periodic face crosses the face's seam
/// meridian (the minimum-u side of its cover window), if it wraps the
/// period. The sampled polyline `points` locates the bracketing samples;
/// the crossing is then refined against the exact `curve` by bisecting
/// `u(t) = level`, so the returned point lies on the curve (and on the
/// seam meridian) to root-find precision. Interpolating the bracketing
/// chord instead would leave the point off the curve by up to the sagitta
/// `r * (1 - cos(pi / SAMPLES_PER_CIRCLE)) ≈ 5.4e-4 * r`, which crosses
/// [`MAX_ALLOWED_TOLERANCE`](crate::check::MAX_ALLOWED_TOLERANCE) once
/// r ≳ 19 and would be recorded as the seam vertex's tolerance.
///
/// Only the FIRST crossing of the seam level is returned. That is exact
/// for the current SSI repertoire — lines, circles, and ellipses are
/// u-monotonic graphs on a cylinder cover, so a closed imprint that wraps
/// the period crosses each seam level exactly once. Future non-monotonic
/// imprints (unequal-radius cylinder-cylinder quartics, NURBS SSI) can
/// cross a level several times; each crossing would need its own split or
/// the imprint is left as a non-chordable chain.
fn seam_crossing(face_poly: &FaceRegionPoly, curve: &Curve3, points: &[Point3]) -> Option<Point3> {
    let uv = map_polyline(&face_poly.chart, points);
    // Closing segment: unwrap the first point relative to the last.
    let close_u = {
        let mut u = uv[0].0;
        let last = uv[uv.len() - 1].0;
        while u - last > std::f64::consts::PI {
            u -= TWO_PI;
        }
        while last - u > std::f64::consts::PI {
            u += TWO_PI;
        }
        u
    };
    let winding = close_u - uv[0].0;
    if winding.abs() < 1.0 {
        return None; // does not wrap the period
    }
    // Seam level: the face cover's minimum u, brought into the polyline's
    // unwrapped range.
    let mut seam_u = f64::INFINITY;
    for lp in &face_poly.loops {
        for ((u, _), _) in lp {
            seam_u = seam_u.min(*u);
        }
    }
    let (u_min, u_max) = uv
        .iter()
        .map(|(u, _)| *u)
        .chain([close_u])
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), u| {
            (lo.min(u), hi.max(u))
        });
    let mut level = seam_u;
    while level <= u_min {
        level += TWO_PI;
    }
    while level > u_max {
        level -= TWO_PI;
    }
    if level <= u_min {
        return None;
    }
    let n = uv.len();
    for i in 0..n {
        let (u0, _) = uv[i];
        let u1 = if i + 1 < n { uv[i + 1].0 } else { close_u };
        if (u0 - level) * (u1 - level) <= 0.0 && (u1 - u0).abs() > 1e-15 {
            let p0 = points[i];
            let p1 = points[(i + 1) % n];
            return Some(refine_seam_point(
                &face_poly.chart,
                curve,
                level,
                (u0, u1),
                (&p0, &p1),
            ));
        }
    }
    None
}

/// Refine a seam crossing bracketed by consecutive imprint samples
/// `p0`/`p1` (unwrapped angles `u0`/`u1` straddling `level`) to a point on
/// the exact `curve` at chart angle `level`, by bisecting `u(t) = level`
/// over the curve-parameter bracket recovered by closest-point projection.
fn refine_seam_point(
    chart: &Chart,
    curve: &Curve3,
    level: f64,
    (u0, u1): (f64, f64),
    (p0, p1): (&Point3, &Point3),
) -> Point3 {
    let t0 = curve.project_point(p0).t;
    let mut t1 = curve.project_point(p1).t;
    if let Some(period) = curve.period() {
        // Samples advance in curve parameter; unwrap the far bracket
        // forward past a period seam.
        while t1 <= t0 {
            t1 += period;
        }
    }
    // Angles along the bracket stay within a sample step of `u0`, far
    // inside the ±π unwrap window of the hint.
    let residual = |t: f64| chart.param(&curve.point(t), Some((u0, 0.0))).0 - level;
    let (mut fa, fb) = (u0 - level, u1 - level);
    if fa == 0.0 {
        return curve.point(t0);
    }
    if fb == 0.0 {
        return curve.point(t1);
    }
    if fa * fb > 0.0 || t1 <= t0 {
        // Projection disagrees with the polyline bracketing (degenerate
        // segment); fall back to the chord point.
        return *p0 + (*p1 - *p0) * ((level - u0) / (u1 - u0));
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
    let mut last_uv: Option<(f64, f64)> = None;
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
        offsets.push(poly.len());
        let last_dart = k + 1 == darts.len();
        let take = if atom.closed || (last_dart && keep_final) {
            pts.len()
        } else {
            pts.len() - 1
        };
        for p in &pts[..take] {
            let raw = face_poly.chart.param(p, last_uv);
            let uv = if last_uv.is_none() {
                face_poly.localize(raw)
            } else {
                raw
            };
            poly.push((uv, *p));
            last_uv = Some(uv);
        }
        walk_pos = Some(if atom.closed {
            pts[0]
        } else {
            pts[pts.len() - 1]
        });
    }
    // Align the whole polyline into the face's cover window so cycles,
    // holes, and probes of one face are mutually comparable.
    if let Chart::Cylinder { .. } = face_poly.chart {
        if !poly.is_empty() {
            let mean = poly.iter().map(|((u, _), _)| u).sum::<f64>() / poly.len() as f64;
            let target = face_poly.localize((mean, 0.0)).0;
            let shift = target - mean;
            if shift.abs() > 1e-9 {
                for ((u, _), _) in poly.iter_mut() {
                    *u += shift;
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
fn merge_imprint_chains(atoms: &[Atom], ids: &[usize], snap: f64) -> Vec<DartChain> {
    use std::collections::HashSet;
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
    let key = |p: &Point3| quantize(p, snap * 4.0);
    let mut adjacency: HashMap<(i64, i64, i64), Vec<(usize, bool)>> = HashMap::new();
    for &ai in &open {
        adjacency
            .entry(key(&atoms[ai].points[0]))
            .or_default()
            .push((ai, true));
        adjacency
            .entry(key(&atoms[ai].points[atoms[ai].points.len() - 1]))
            .or_default()
            .push((ai, false));
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
            let Some(cands) = adjacency.get(&key(&end)) else {
                break;
            };
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
            let Some(cands) = adjacency.get(&key(&start)) else {
                break;
            };
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
/// cycle, allowing a whole-period shift of the chain (seam chords match
/// the two cover copies of one 3D point).
fn match_chain_to_cycle(
    cycle: &Cycle,
    chain_poly: &[((f64, f64), Point3)],
    period: Option<f64>,
) -> Option<(usize, usize)> {
    let (mut lo, mut hi) = (
        (f64::INFINITY, f64::INFINITY),
        (f64::NEG_INFINITY, f64::NEG_INFINITY),
    );
    for ((u, v), _) in &cycle.poly {
        lo = (lo.0.min(*u), lo.1.min(*v));
        hi = (hi.0.max(*u), hi.1.max(*v));
    }
    let eps = ((hi.0 - lo.0) + (hi.1 - lo.1)).max(1e-12) * 1e-5;
    let s_uv = chain_poly[0].0;
    let e_uv = chain_poly[chain_poly.len() - 1].0;
    let shifts: Vec<f64> = match period {
        Some(p) => vec![-p, 0.0, p],
        None => vec![0.0],
    };
    let nearest = |uv: (f64, f64)| -> (usize, f64) {
        cycle
            .dart_offsets
            .iter()
            .enumerate()
            .map(|(k, &off)| {
                let v = cycle.poly[off].0;
                (k, ((v.0 - uv.0).powi(2) + (v.1 - uv.1).powi(2)).sqrt())
            })
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .expect("cycle has darts")
    };
    let mut best: Option<(f64, usize, usize)> = None;
    for &sh in &shifts {
        let (i, di) = nearest((s_uv.0 + sh, s_uv.1));
        let (j, dj) = nearest((e_uv.0 + sh, e_uv.1));
        if di <= eps && dj <= eps && i != j {
            let score = di + dj;
            if best.is_none() || score < best.expect("checked").0 {
                best = Some((score, i, j));
            }
        }
    }
    best.map(|(_, i, j)| (i, j))
}

fn cyclic_slice(darts: &[(usize, bool)], from: usize, to: usize) -> DartChain {
    if from < to {
        darts[from..to].to_vec()
    } else {
        darts[from..].iter().chain(&darts[..to]).copied().collect()
    }
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
    let period = matches!(face_poly.chart, Chart::Cylinder { .. }).then_some(TWO_PI);

    for ri in 0..regions.len() {
        for ci in 0..regions[ri].cycles.len() {
            let Some((vi, vj)) = match_chain_to_cycle(&regions[ri].cycles[ci], &chain_poly, period)
            else {
                continue;
            };
            if ci != 0 {
                return Err(CoreError::NotImplemented {
                    feature: "boolean imprints chording a hole boundary (transversal MVP)",
                });
            }
            // The chain runs vertex vi -> vertex vj of the outer cycle.
            let outer = regions[ri].cycles[0].clone();
            let holes: Vec<Cycle> = regions[ri].cycles[1..].to_vec();
            let mut darts_one = chain.clone();
            darts_one.extend(cyclic_slice(&outer.darts, vj, vi));
            let mut darts_two = reverse_chain(&chain);
            darts_two.extend(cyclic_slice(&outer.darts, vi, vj));
            let cycle_one = embed_cycle(face_poly, atoms, darts_one);
            let cycle_two = embed_cycle(face_poly, atoms, darts_two);
            let mut region_one = Region {
                cycles: vec![cycle_one],
            };
            let mut region_two = Region {
                cycles: vec![cycle_two],
            };
            for hole in holes {
                let probe = hole.poly[0].0;
                if point_in_cycle(&region_one.cycles[0], probe) {
                    region_one.cycles.push(hole);
                } else {
                    region_two.cycles.push(hole);
                }
            }
            regions[ri] = region_one;
            regions.push(region_two);
            return Ok(());
        }
    }

    // No boundary match: the chain must close on itself (an interior ring).
    let start = chain_poly[0].1;
    let end = chain_poly[chain_poly.len() - 1].1;
    if (start - end).norm() > snap * 100.0 {
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
        .position(|r| region_contains(r, probe))
        .ok_or_else(|| CoreError::Degenerate {
            context: "boolean::imprint",
            reason: "an interior imprint ring lies in no region of its host face".into(),
        })?;
    let (disk, hole) = if ring.area >= 0.0 {
        let hole = embed_cycle(face_poly, atoms, reverse_chain(&ring.darts));
        (ring, hole)
    } else {
        let disk = embed_cycle(face_poly, atoms, reverse_chain(&ring.darts));
        (disk, ring)
    };
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
    let extent = ((hi.0 - lo.0).abs() + (hi.1 - lo.1).abs()).max(1e-12);
    for scale in [1e-3, 1e-2, 5e-2] {
        let off = extent * scale;
        for i in 0..n {
            let (a, _) = outer.poly[i];
            let (b, _) = outer.poly[(i + 1) % n];
            let (dx, dy) = (b.0 - a.0, b.1 - a.1);
            let len = (dx * dx + dy * dy).sqrt();
            if len < extent * 1e-9 {
                continue;
            }
            // Left normal of a CCW boundary points into the region.
            let (nx, ny) = (-dy / len, dx / len);
            let mid = ((a.0 + b.0) * 0.5 + nx * off, (a.1 + b.1) * 0.5 + ny * off);
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
    }
}

/// Analytic ray-surface intersection parameters (unbounded surface).
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
        _ => Vec::new(),
    }
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

    // Vertices and edges, deduplicated by quantized 3D position / atom id.
    // Geometry is shared: one output surface id per host face (all regions
    // split from it), one output curve id per source curve.
    let mut vertex_of: HashMap<(i64, i64, i64), EntityId<crate::topology::Vertex>> = HashMap::new();
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
            let darts: Vec<(usize, bool)> = if kr.reverse {
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
                        let sv = *vertex_of
                            .entry(quantize(&start_p, snap * 4.0))
                            .or_insert_with(|| store.create_vertex(start_p, SYSTEM_RESOLUTION));
                        let ev = *vertex_of
                            .entry(quantize(&end_p, snap * 4.0))
                            .or_insert_with(|| store.create_vertex(end_p, SYSTEM_RESOLUTION));

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

        // Tessellation payload.
        let fp = &pipe.face_polys[kr.solid][kr.face];
        let rings = kr
            .region
            .cycles
            .iter()
            .map(|cy| MeshRing {
                uv: cy.poly.iter().map(|(uv, _)| *uv).collect(),
                points: cy.poly.iter().map(|(_, p)| *p).collect(),
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
    let shell_ids: Vec<EntityId<crate::topology::Shell>> = shells.values().copied().collect();
    for shell_id in &shell_ids {
        let (v, e, f, r) = shell_counts(&store, *shell_id);
        let chi = v as i64 - e as i64 + f as i64 - r as i64;
        // V - E + F - R = 2(1 - H)  =>  H = 1 - chi/2.
        if chi % 2 == 0 {
            let h = 1 - chi / 2;
            if h >= 0 {
                store
                    .shells
                    .get_mut(*shell_id)
                    .expect("shell just created")
                    .genus = h as u32;
            }
        }
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
    // a wide face (e.g. the full-wrap band left by a through-hole
    // subtract) gets triangles whose u-span is far coarser than the
    // boundary sampling, so their flat 3D chords cut deep inside the
    // surface. Refine by bisecting any triangle edge whose u-span exceeds
    // the boundary pitch, evaluating the midpoint on the analytic surface.
    // The split decision depends only on the edge and midpoints are shared
    // through an edge-keyed map, so neighbors always split identically
    // (no cracks); boundary polyline edges are already at pitch and are
    // never split, so welding with adjacent faces is preserved.
    if !planar {
        // 1.5× the boundary pitch: edges at the sampling pitch (the shared
        // boundary polylines themselves) must never split — the adjacent
        // face doesn't refine its copy, and a one-sided split is a
        // T-junction (non-manifold weld). Chords under 1.5 pitch deviate
        // from the surface by < r·(1 − cos(0.75·2π/96)) ≈ 1e-3·r, in line
        // with the boundary sampling itself.
        let max_du = 1.5 * TWO_PI / SAMPLES_PER_CIRCLE as f64;
        let mut midpoint: HashMap<(usize, usize), usize> = HashMap::new();
        let mut queue: std::collections::VecDeque<[usize; 3]> = tris.drain(..).collect();
        while let Some(t) = queue.pop_front() {
            let span = |i: usize, j: usize| (all_uv[t[i]].0 - all_uv[t[j]].0).abs();
            let spans = [span(0, 1), span(1, 2), span(2, 0)];
            let (k, widest) = spans
                .iter()
                .enumerate()
                .max_by(|a, b| a.1.total_cmp(b.1))
                .expect("three edges");
            if *widest <= max_du {
                tris.push(t);
                continue;
            }
            let (i, j, o) = (t[k], t[(k + 1) % 3], t[(k + 2) % 3]);
            let m = *midpoint.entry((i.min(j), i.max(j))).or_insert_with(|| {
                let mid_uv = (
                    0.5 * (all_uv[i].0 + all_uv[j].0),
                    0.5 * (all_uv[i].1 + all_uv[j].1),
                );
                all_uv.push(mid_uv);
                all_p.push(chart_point(&mf.chart, mid_uv));
                all_uv.len() - 1
            });
            queue.push_back([i, m, o]);
            queue.push_back([m, j, o]);
        }
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
            mf.chart.normal(all_uv[i0].0) * mf.normal_sign,
            mf.chart.normal(all_uv[i1].0) * mf.normal_sign,
            mf.chart.normal(all_uv[i2].0) * mf.normal_sign,
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
    use crate::transform::{rotate_body, translate_body};

    fn tol() -> ToleranceContext {
        ToleranceContext::default()
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
    fn sphere_inputs_hit_the_chart_gap() {
        // Spheres extract fine but have no boolean parameter chart yet.
        let (mut store, mut geo) = stores();
        let a = block_at(&mut store, &mut geo, (2.0, 2.0, 2.0), (0.0, 0.0, 0.0));
        let b = primitives::sphere(&mut store, &mut geo, 1.0).expect("valid sphere");
        let err = unite(&store, &geo, a, b, &tol()).unwrap_err();
        assert!(
            matches!(err, CoreError::NotImplemented { .. }),
            "expected NotImplemented for sphere charts, got {err:?}"
        );
        assert!(err.to_string().contains("sphere"), "unhelpful error: {err}");
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
        let p = seam_crossing(&fp, &curve, &points).expect("ring wraps the period");
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
        let p = seam_crossing(&fp, &curve, &points).expect("ring wraps the period");
        // This section satisfies u(t) = t, so the seam angle is hit at
        // t = seam_u (mod 2π).
        let expected = curve.point(seam_u);
        assert!(
            (p - expected).norm() < 1e-9,
            "seam vertex off the exact curve by {:.3e}",
            (p - expected).norm()
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
}
