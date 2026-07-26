//! B-Rep tessellation MVP (`spec/07-tessellation.md`): convert bodies with
//! analytic face geometry into [`TriangleMesh`]es.
//!
//! Strategy, per face by surface kind:
//!
//! - **Planar faces**: every loop is sampled into a polygon (lines as
//!   single segments, circles/ellipses at the angular step); inner loops
//!   (holes) are bridged into the outer loop and the result is ear-clip
//!   triangulated (correct for concave outlines).
//! - **NURBS faces**: gridded over the patch's knot domain, one lattice line
//!   per knot (the polynomial pieces), each span subdivided by how far the
//!   surface *normal turns* across it rather than by parameter length — a
//!   knot domain is arbitrary, so a parameter-priced lattice would mesh
//!   identical geometry differently over `[0,1]` and `[3,13]`. Untrimmed
//!   patches only; see the MVP limitations below.
//! - **Quadric faces** (cylinder, cone, sphere, torus): sampled on a
//!   parameter grid. Periodic directions wrap by index, so seams close
//!   exactly; parameterization singularities (sphere poles, cone apex)
//!   collapse their grid row to a single vertex with the limit normal.
//!   Ruled directions (cylinder/cone `v`) use one segment; angular
//!   directions honor the angular step. The `v` range of an unbounded
//!   surface is recovered by projecting boundary-edge samples onto the
//!   surface.
//!
//! Per-vertex normals come from [`SurfaceEval::normal`], negated for
//! [`FaceSense::Negative`] faces so they point outward from the material;
//! triangle winding follows the same outward direction. Boolean outputs
//! routinely bind tool-derived faces with Negative sense (of-as6).
//!
//! [`tessellate_body`] concatenates the per-face meshes and welds them:
//! adjacent faces sample their shared edges at identical curve parameters,
//! so rim vertices coincide and welding stitches the body watertight.
//! Welded boundary vertices average the adjoining faces' normals.
//!
//! # MVP limitations (later hardening passes)
//!
//! - Cylinder/cone faces cover either their **full `u` period** (primitive and
//!   sweep walls) or a **partial arc that is a clean iso-parameter rectangle**
//!   (`[u_lo, u_hi] × [v_lo, v_hi]`, e.g. a quarter-cylinder notch from a
//!   boolean, of-2i3) — both recovered by projecting boundary samples onto the
//!   surface. A trim whose boundary is *not* such a rectangle (a slanted planar
//!   cut, or a face with inner loops) is *detected* and rejected with
//!   [`CoreError::NotImplemented`] instead of silently gridding it wrong
//!   (of-q6u); it arrives with the CDT pass
//!   ([`crate::boolean::BooleanOutput::tessellate`] already handles it).
//! - NURBS faces do **not** grid at all. They take the same
//!   constrained-Delaunay pass a boolean result's faces take
//!   ([`nurbs_face_cdt`], of-37i.6), which lifts both of the grid arm's
//!   limits at once: the face's boundary is sampled from its **edge curves**,
//!   so a curved patch welds to its neighbours (of-dvj — a grid sampled the
//!   shared edge by its own rational parameter and the two rims missed each
//!   other), and trimming and inner loops need no iso-rectangle. The one
//!   exception is a patch with a *collapsed control row*, which has no chart
//!   ([`crate::boolean`]'s `Chart::build` rejects it, of-37i.7): untrimmed,
//!   it still grids off the same turning measure ([`nurbs_lattice`]);
//!   trimmed, it is rejected.
//! - Sphere/torus faces must cover the **full `v` domain/period**: their
//!   boundary must consist purely of seams (every edge traversed with net-zero
//!   sense), as the primitive constructors and STEP reader produce. Trimmed
//!   sphere/torus faces (caps, zones, wedges) are likewise rejected for the CDT
//!   pass.
//! - The only fidelity control is [`TessellationOptions::angular_step`];
//!   chord tolerance, edge-length bounds, and adaptive refinement are
//!   deferred.

use crate::curve::{Curve3, CurveEval, plane_basis};
use crate::geometry::GeometryStore;
use crate::nurbs::{NurbsCurve, NurbsSurface};
use crate::project::SurfaceProject;
use crate::surface::{Surface3, SurfaceEval};
use crate::topology::{Body, Edge, Face, FaceSense, Fin, FinSense, Loop, TopologyStore};
use opensolid_core::error::{CoreError, CoreResult};
use opensolid_core::mesh::TriangleMesh;
use opensolid_core::tolerance::ToleranceContext;
use opensolid_core::{EntityId, Point3, Vector3};

/// Fidelity controls for tessellation.
///
/// The MVP exposes a single knob; the spec's full option set (chord
/// tolerance, edge-length bounds) is a later hardening pass.
#[derive(Debug, Clone)]
pub struct TessellationOptions {
    /// Maximum parameter step, in radians, when sampling angular directions
    /// (circular edges, quadric parameter grids). Smaller is finer: the
    /// default `2π/32` gives 32 segments around a full circle.
    pub angular_step: f64,
}

impl Default for TessellationOptions {
    fn default() -> Self {
        Self {
            angular_step: std::f64::consts::TAU / 32.0,
        }
    }
}

impl TessellationOptions {
    fn validate(&self) -> CoreResult<()> {
        if self.angular_step <= 0.0 || !self.angular_step.is_finite() {
            return Err(CoreError::InvalidArgument {
                argument: "angular_step",
                reason: format!("must be positive and finite, got {}", self.angular_step),
            });
        }
        Ok(())
    }
}

/// Segment count for sweeping an angular range at the configured step.
/// At least 3, so closed circles always produce a real polygon.
///
/// The count is `ceil(sweep / step)`, but a sweep within floating tolerance of
/// an exact multiple of the step snaps down to that multiple: a quarter, half,
/// or full revolution lands on an integer count, and two adjacent faces that
/// recover the *same* shared arc's sweep with independent rounding noise (the
/// wall projecting boundary samples, the cap reading its edge's parameter span)
/// must agree on the count, or their rim vertices land on different sample
/// positions and fail to weld (of-2i3). Snapping only nudges values already
/// within `1e-9` of an integer, well above float noise (~1e-14) yet far below
/// any sweep difference that changes fidelity.
fn angular_segments(sweep: f64, options: &TessellationOptions) -> usize {
    let raw = sweep.abs() / options.angular_step;
    ((raw - 1e-9).ceil() as usize).max(3)
}

/// Segment count for sweeping a freeform curve from `t_from` to `t_to`.
///
/// A NURBS curve has no angular parameter to price a pitch off, so — as
/// [`span_samples`] does for a patch — the count is derived from how far the
/// *tangent turns* over the swept range: `ceil(turn / angular_step)`. The
/// turn is the summed angle between tangents at `2·degree + 1` probes per
/// covered knot span, enough to resolve a polynomial piece of that degree;
/// zero-length tangents (a collapsed control run) contribute nothing rather
/// than a guessed angle.
///
/// The result is floored at the number of knot spans the range covers, so a
/// curve that is nearly straight but has interior knots — where a repeated
/// knot may hide a tangent discontinuity that turns *between* probes — still
/// gets a vertex per piece. Sampling stays uniform in `t`, matching every
/// other variant here, so interior knots are bracketed rather than hit
/// exactly; that costs chord accuracy, never correctness.
fn nurbs_curve_segments(
    nurbs: &NurbsCurve,
    t_from: f64,
    t_to: f64,
    options: &TessellationOptions,
) -> usize {
    let (lo, hi) = (t_from.min(t_to), t_from.max(t_to));
    // Distinct knots strictly inside the swept range split it into spans.
    let interior = nurbs
        .knot_vector()
        .knots()
        .windows(2)
        .filter(|w| w[1] > w[0] && w[1] > lo && w[1] < hi)
        .count();
    let spans = interior + 1;

    let probes = spans * (2 * nurbs.degree() + 1);
    let mut turn = 0.0;
    let mut previous: Option<Vector3> = None;
    for i in 0..=probes {
        let t = t_from + (t_to - t_from) * i as f64 / probes as f64;
        let tangent = nurbs.derivative(t);
        if tangent.norm() == 0.0 {
            continue;
        }
        if let Some(p) = previous {
            turn += p.angle(&tangent);
        }
        previous = Some(tangent);
    }
    // Snapped exactly as `angular_segments` snaps, and for the same
    // reason: a turn within float noise of a whole number of steps — a
    // half or full revolution, say — must land on that count rather than
    // one more, or two faces recovering the same shared arc disagree on
    // where its samples go and their rim vertices fail to weld.
    (((turn / options.angular_step) - 1e-9).ceil() as usize).max(spans)
}

/// Tessellate every face of `body` into one welded mesh.
///
/// For the closed solids produced by [`crate::primitives`] and
/// [`crate::sweep`], the result is a closed, consistently oriented
/// manifold (see [`TriangleMesh::is_closed_manifold`]).
///
/// # Errors
/// [`CoreError::InvalidArgument`] if `body` is stale, or any reached face
/// or edge lacks attached geometry; [`CoreError::NotImplemented`] for the
/// trimmed quadric faces the module docs list, and for a trimmed
/// collapsed-row NURBS patch; [`CoreError::Degenerate`] if a planar face's
/// inner loops do not bound a valid region, or a NURBS face's loops do not
/// bound a triangulable one.
pub fn tessellate_body(
    store: &TopologyStore,
    geo: &GeometryStore,
    body: EntityId<Body>,
    options: &TessellationOptions,
) -> CoreResult<TriangleMesh> {
    options.validate()?;
    if store.body(body).is_none() {
        return Err(CoreError::InvalidArgument {
            argument: "body",
            reason: format!("stale body id {body:?}"),
        });
    }

    let mut mesh = TriangleMesh::new();
    for face in store.faces_of_body(body) {
        tessellate_face_into(store, geo, face, options, &mut mesh)?;
    }

    // Adjacent faces sample shared edges at identical parameters, so their
    // rim vertices agree to floating-point noise; weld at a tolerance far
    // below any feature size to stitch them.
    let epsilon = mesh
        .bounding_box()
        .map(|b| (b.max - b.min).norm() * 1e-9)
        .unwrap_or(0.0);
    Ok(mesh.weld(epsilon))
}

/// Tessellate a single face (unwelded, open along its boundary unless the
/// face alone closes the surface).
///
/// # Errors
/// As [`tessellate_body`], for this face.
pub fn tessellate_face(
    store: &TopologyStore,
    geo: &GeometryStore,
    face: EntityId<Face>,
    options: &TessellationOptions,
) -> CoreResult<TriangleMesh> {
    options.validate()?;
    let mut mesh = TriangleMesh::new();
    tessellate_face_into(store, geo, face, options, &mut mesh)?;
    Ok(mesh)
}

fn invalid_face(face: EntityId<Face>, what: &str) -> CoreError {
    CoreError::InvalidArgument {
        argument: "body",
        reason: format!("face {face:?} {what}"),
    }
}

fn tessellate_face_into(
    store: &TopologyStore,
    geo: &GeometryStore,
    face_id: EntityId<Face>,
    options: &TessellationOptions,
    mesh: &mut TriangleMesh,
) -> CoreResult<()> {
    let face = store
        .face(face_id)
        .ok_or_else(|| invalid_face(face_id, "is stale"))?;
    let surface_id = face
        .surface
        .ok_or_else(|| invalid_face(face_id, "has no attached surface geometry"))?;
    let surface = geo
        .surface(surface_id)
        .ok_or_else(|| invalid_face(face_id, "references a stale surface id"))?;

    // A Negative-sense face's outward normal opposes its surface normal
    // (boolean outputs encode tool-derived faces this way — see
    // `crate::boolean`): flip emitted normals and winding to stay outward.
    let flip = face.sense == FaceSense::Negative;
    match surface {
        Surface3::Plane { .. } => {
            fan_planar_face(store, geo, face_id, face, surface, flip, options, mesh)
        }
        Surface3::Cylinder { .. } | Surface3::Cone { .. } => {
            let (u_span, v_lo, v_hi) = boundary_param_range(store, geo, face_id, face, surface)?;
            let period = surface.period_u().expect("quadric surfaces are u-periodic");
            let (u_lo, u_hi, wrap_u) = match u_span {
                QuadricUSpan::Full { u_anchor } => (u_anchor, u_anchor + period, true),
                QuadricUSpan::PartialRect { u_lo, u_hi } => (u_lo, u_hi, false),
            };
            grid_face(
                surface, u_lo, u_hi, wrap_u, v_lo, v_hi, false, 1, flip, options, mesh,
            );
            Ok(())
        }
        Surface3::Sphere { .. } => {
            require_seam_closed_boundary(store, face_id, face)?;
            let period = surface.period_u().expect("sphere is u-periodic");
            let (v_lo, v_hi) = surface.domain_v();
            let n_v = angular_segments(v_hi - v_lo, options);
            grid_face(
                surface, 0.0, period, true, v_lo, v_hi, false, n_v, flip, options, mesh,
            );
            Ok(())
        }
        Surface3::Torus { .. } => {
            require_seam_closed_boundary(store, face_id, face)?;
            let period_u = surface.period_u().expect("torus is u-periodic");
            let period_v = surface.period_v().expect("torus is v-periodic");
            let n_v = angular_segments(period_v, options);
            grid_face(
                surface, 0.0, period_u, true, 0.0, period_v, true, n_v, flip, options, mesh,
            );
            Ok(())
        }
        // A freeform patch takes the constrained-Delaunay pass (of-37i.6):
        // its boundary comes from the *edge curves*, so it welds to whatever
        // is on the other side of them, and its interior from a lattice
        // priced off how far the normal turns. That covers trimmed faces and
        // inner loops as well, which a grid cannot represent at all.
        //
        // The one patch that cannot take it is one with a collapsed control
        // row, which has no chart (`Chart::build` rejects it — the pole
        // machinery has no analogue for one). Untrimmed, it still grids
        // correctly off the same turning measure, so it keeps that path.
        Surface3::Nurbs(nurbs) if nurbs.has_degenerate_edge() => {
            let (us, vs) = nurbs_lattice(store, geo, face_id, face, surface, nurbs, options)?;
            let v_range = (vs[0], vs[vs.len() - 1]);
            emit_grid(surface, &us, &vs, v_range, false, false, flip, mesh);
            Ok(())
        }
        Surface3::Nurbs(_) => {
            nurbs_face_cdt(store, geo, face_id, face, surface, flip, options, mesh)
        }
    }
}

/// Tessellate a NURBS face through the of-lcx constrained-Delaunay pass —
/// the same one a boolean result's faces take
/// ([`crate::boolean::BooleanOutput::tessellate`]) — instead of gridding it
/// (of-37i.6).
///
/// Two things fall out of that, and they are the two limits the grid arm
/// could not lift:
///
/// - **The face welds.** Its boundary vertices are the *edge curves'* own
///   samples, taken by [`sample_loop`] — the very points the face on the
///   other side of each edge uses. A grid instead samples the shared edge
///   at its own parameter positions, which for a curved patch are not the
///   curve's (a rational quarter-cylinder's parameter is not angle-uniform,
///   so its rim missed the cap's by up to half a sample: of-dvj, 128 open
///   edges on a 124-triangle bore).
/// - **Trimming and inner loops work.** The boundary is whatever the loops
///   say it is, and holes are removed by ring parity rather than needing an
///   iso-rectangle. Nothing here requires the boundary to run along the
///   knot-domain border.
///
/// Each sample's `uv` comes from projecting its 3D curve point onto the
/// patch, so the ring is the curve's polyline first and a parameter-space
/// ring second. A projection is only ever used to place a point the
/// triangulation already owns, never to *produce* one, so its error costs
/// combinatorics and shading, not position.
#[allow(clippy::too_many_arguments)]
fn nurbs_face_cdt(
    store: &TopologyStore,
    geo: &GeometryStore,
    face_id: EntityId<Face>,
    face: &Face,
    surface: &Surface3,
    flip: bool,
    options: &TessellationOptions,
    mesh: &mut TriangleMesh,
) -> CoreResult<()> {
    let loop_id = face
        .outer_loop
        .ok_or_else(|| invalid_face(face_id, "has no outer loop"))?;
    let mut rings_p = vec![sample_loop(store, geo, face_id, loop_id, options)?];
    if rings_p[0].len() < 3 {
        return Err(invalid_face(
            face_id,
            "outer loop samples to fewer than 3 points",
        ));
    }
    for &inner_id in &face.inner_loops {
        rings_p.push(sample_loop(store, geo, face_id, inner_id, options)?);
    }
    let rings_uv: Vec<Vec<(f64, f64)>> = rings_p
        .iter()
        .map(|ring| {
            ring.iter()
                .map(|p| {
                    let projected = surface.project_point(p);
                    (projected.u, projected.v)
                })
                .collect()
        })
        .collect();

    // The tolerance only feeds `Chart::param`'s iterative inverse, which
    // this path never reaches — every boundary uv is already in hand.
    let (tris, uv, points) = crate::boolean::triangulate_trimmed_region(
        surface,
        &ToleranceContext::default(),
        options.angular_step,
        &rings_uv,
        &rings_p,
    )?;

    let base = mesh.positions.len();
    for (&(u, v), &p) in uv.iter().zip(&points) {
        mesh.positions.push(p);
        // `Chart::build` admitted this patch, so it has a normal
        // everywhere — a collapsed control row is the only NURBS analogue
        // of a pole and it is rejected there, before this point.
        let normal = surface.normal(u, v).unwrap_or_else(Vector3::zeros);
        mesh.normals.push(if flip { -normal } else { normal });
    }
    // Triangles come out counter-clockwise in parameter space, which is the
    // `du × dv` side; a Negative-sense face's outward direction opposes it.
    for t in tris {
        let tri = if flip {
            [base + t[0], base + t[2], base + t[1]]
        } else {
            [base + t[0], base + t[1], base + t[2]]
        };
        if tri[0] != tri[1] && tri[1] != tri[2] && tri[0] != tri[2] {
            mesh.indices.push(tri);
        }
    }
    Ok(())
}

/// Ear-clip triangulate a planar face, bridging any inner loops (holes) into
/// the outer loop first. Correct for concave outlines, unlike the old
/// first-vertex fan (of-6dw).
#[allow(clippy::too_many_arguments)]
fn fan_planar_face(
    store: &TopologyStore,
    geo: &GeometryStore,
    face_id: EntityId<Face>,
    face: &Face,
    surface: &Surface3,
    flip: bool,
    options: &TessellationOptions,
    mesh: &mut TriangleMesh,
) -> CoreResult<()> {
    let loop_id = face
        .outer_loop
        .ok_or_else(|| invalid_face(face_id, "has no outer loop"))?;
    let outer = sample_loop(store, geo, face_id, loop_id, options)?;
    if outer.len() < 3 {
        return Err(invalid_face(
            face_id,
            "outer loop samples to fewer than 3 points",
        ));
    }
    // Hole loops (every drilled plate has them, of-fc8) are sampled the same
    // way and bridged into the outer loop below. A hole sampling to fewer
    // than 3 points cuts nothing out; `ear_clip_rings` drops it.
    let mut rings_3d = vec![outer];
    for &inner_id in &face.inner_loops {
        rings_3d.push(sample_loop(store, geo, face_id, inner_id, options)?);
    }
    let polygon: Vec<Point3> = rings_3d.iter().flatten().copied().collect();

    let surface_normal = surface
        .normal(0.0, 0.0)
        .expect("planes have a normal everywhere");
    // The face's outward normal (of-as6): Negative-sense faces oppose their
    // surface normal, and their loops wind CCW about *outward* — building
    // the basis about outward keeps ear_clip's winding outward-facing.
    let normal = if flip {
        -surface_normal
    } else {
        surface_normal
    };
    let base = mesh.positions.len();
    for point in &polygon {
        mesh.positions.push(*point);
        mesh.normals.push(normal);
    }
    // Ear-clip the loop polygon so concave faces (U/S/C outlines) tile
    // without overlap; a first-vertex fan was silently wrong for any loop
    // not star-shaped from that vertex (of-6dw). Project onto a plane
    // basis with e_u × e_v = normal so ear_clip's counterclockwise
    // triples come out wound about +normal — the outward winding, since
    // the loop runs counterclockwise about the outward normal.
    let (e_u, e_v) = plane_basis(&normal);
    let origin = polygon[0];
    let project = |p: &Point3| {
        let d = p - origin;
        (d.dot(&e_u), d.dot(&e_v))
    };
    let rings: Vec<Vec<(f64, f64)>> = rings_3d
        .iter()
        .map(|ring| ring.iter().map(project).collect())
        .collect();
    // `ear_clip_rings` indexes the rings' concatenation, matching the order
    // `polygon` (and hence the emitted vertices) was built in.
    let tris = crate::triangulate::ear_clip_rings(&rings).ok_or_else(|| CoreError::Degenerate {
        context: "tessellate::planar_face",
        reason: format!("face {face_id:?} has an inner loop that cannot be bridged to its outer loop; the loops do not bound a valid region"),
    })?;
    for [a, b, c] in tris {
        mesh.indices.push([base + a, base + b, base + c]);
    }
    Ok(())
}

/// Sample a loop's boundary as a closed polygon, in loop order, one open
/// run of points per fin (each fin's end point is supplied by the next).
///
/// Each run *starts* at the fin's own vertex point rather than at the curve
/// evaluated there — see [`fin_start_point`] for why that distinction is the
/// whole of of-61f.
fn sample_loop(
    store: &TopologyStore,
    geo: &GeometryStore,
    face_id: EntityId<Face>,
    loop_id: EntityId<Loop>,
    options: &TessellationOptions,
) -> CoreResult<Vec<Point3>> {
    let mut points = Vec::new();
    for &fin_id in store.fins_of_loop(loop_id) {
        let (curve, t_from, t_to) = fin_curve(store, geo, face_id, fin_id)?;
        let segments = match curve {
            Curve3::Line { .. } => 1,
            Curve3::Circle { .. } | Curve3::Ellipse { .. } => {
                angular_segments(t_to - t_from, options)
            }
            // One parameter unit per chord: sampling at the vertices
            // reproduces the polyline exactly.
            Curve3::Polyline { .. } => ((t_to - t_from).abs().ceil() as usize).max(1),
            Curve3::Nurbs(nurbs) => nurbs_curve_segments(nurbs, t_from, t_to, options),
        };
        let corner = fin_start_point(store, fin_id);
        for k in 0..segments {
            let t = t_from + (t_to - t_from) * k as f64 / segments as f64;
            match corner.filter(|_| k == 0) {
                Some(point) => points.push(point),
                None => points.push(curve.point(t)),
            }
        }
    }
    Ok(points)
}

/// The point of the vertex a fin *starts* at, in traversal direction.
///
/// This is the one boundary sample that cannot come from the curve. On an
/// exact body the two agree to float noise, but on a **tolerant** one they do
/// not: after healing closes a gap the surviving vertex sits at the cluster
/// centroid while each adjacent edge's curve still runs to its own pre-merge
/// endpoint — precisely the displacement [`Vertex::tolerance`] records
/// (`spec/08-tolerances.md`). Two faces meeting at such a vertex reach it
/// along *different* edges, so sampling each edge's curve at its corner gave
/// them points up to the closed gap apart (1e-9..1e-5 mm measured), far above
/// `tessellate_body`'s weld epsilon of `bbox_diagonal * 1e-9`. The mesh then
/// failed to stitch and a healed import read as an open shell even though the
/// body passed [`TopologyStore::check`] — of-61f.
///
/// Taking the vertex's own point instead makes every loop through it emit the
/// identical corner, so the weld succeeds by construction rather than by
/// tolerance luck. The edge *interior* never needed this: the two faces across
/// an edge share its curve and sample it at the same parameters.
///
/// `None` when the vertex id is stale, which leaves the curve's endpoint in
/// place — a body that tessellated before must not start erroring over a
/// corner refinement.
fn fin_start_point(store: &TopologyStore, fin_id: EntityId<Fin>) -> Option<Point3> {
    let fin = store.fin(fin_id)?;
    let edge = store.edge(fin.edge)?;
    let vertex = match fin.sense {
        FinSense::Forward => edge.start_vertex,
        FinSense::Reversed => edge.end_vertex,
    };
    store.vertex(vertex).map(|v| v.point)
}

/// A fin's curve and its parameter sweep in traversal direction.
fn fin_curve<'g>(
    store: &TopologyStore,
    geo: &'g GeometryStore,
    face_id: EntityId<Face>,
    fin_id: EntityId<Fin>,
) -> CoreResult<(&'g Curve3, f64, f64)> {
    let fin = store
        .fin(fin_id)
        .ok_or_else(|| invalid_face(face_id, "loop references a stale fin"))?;
    let edge = store
        .edge(fin.edge)
        .ok_or_else(|| invalid_face(face_id, "fin references a stale edge"))?;
    let curve_id = edge
        .curve
        .ok_or_else(|| invalid_face(face_id, "has an edge with no attached curve geometry"))?;
    let curve = geo
        .curve(curve_id)
        .ok_or_else(|| invalid_face(face_id, "has an edge referencing a stale curve id"))?;
    let (t_from, t_to) = match fin.sense {
        FinSense::Forward => (edge.t_start, edge.t_end),
        FinSense::Reversed => (edge.t_end, edge.t_start),
    };
    Ok((curve, t_from, t_to))
}

/// Boundary samples per fin when recovering parameter ranges and checking
/// angular coverage. Fine enough that a full circular fin leaves `u` gaps
/// of `period/32` — well under the [`MIN_PERIOD_COVERAGE`] slack — so the
/// coverage guard cleanly separates full rings from trimmed wedges.
const BOUNDARY_SAMPLES: usize = 32;

/// Minimum fraction of the `u` period a cylinder/cone face's boundary must
/// cover for the full-period grid to be a faithful tessellation. Boundaries
/// missing more than this slack (trimmed wedges) are rejected rather than
/// silently rendered as the whole surface of revolution (of-q6u).
const MIN_PERIOD_COVERAGE: f64 = 0.9;

/// Guard that a face on a *closed* surface (sphere, torus) covers the whole
/// surface: its boundary must cancel, i.e. every edge appears in the face's
/// loops with as many `Forward` as `Reversed` fins — pure seams, as the
/// primitive constructors and STEP reader produce. A trimmed face (cap,
/// zone, wedge, imported partial revolve) has at least one real boundary
/// edge traversed once; gridding the full closed surface for it would be
/// grossly wrong (of-q6u). Faces closed only by singular vertex loops (no
/// fins) pass vacuously.
///
/// # Errors
/// [`CoreError::NotImplemented`] if any boundary edge is not a seam.
fn require_seam_closed_boundary(
    store: &TopologyStore,
    face_id: EntityId<Face>,
    face: &Face,
) -> CoreResult<()> {
    let mut net: std::collections::HashMap<EntityId<Edge>, i32> = std::collections::HashMap::new();
    for loop_id in face
        .outer_loop
        .into_iter()
        .chain(face.inner_loops.iter().copied())
    {
        for &fin_id in store.fins_of_loop(loop_id) {
            let fin = store
                .fin(fin_id)
                .ok_or_else(|| invalid_face(face_id, "loop references a stale fin"))?;
            *net.entry(fin.edge).or_insert(0) += match fin.sense {
                FinSense::Forward => 1,
                FinSense::Reversed => -1,
            };
        }
    }
    if net.values().any(|&n| n != 0) {
        return Err(CoreError::NotImplemented {
            feature: "tessellating trimmed sphere/torus faces \
                      (boundary edges are not all seams; needs the CDT pass)",
        });
    }
    Ok(())
}

/// How a cylinder/cone face maps onto its surface's `u` period, recovered
/// from boundary samples by [`boundary_param_range`].
enum QuadricUSpan {
    /// The boundary covers the full period (primitive/sweep walls): grid the
    /// whole revolution with wrap, columns starting at `u_anchor`. The `u`
    /// columns of a transformed body must start at the same arbitrary anchor
    /// angle its rims were re-anchored to ([`crate::transform`]) so rim
    /// vertices coincide with the adjacent faces' samples and weld watertight.
    Full { u_anchor: f64 },
    /// The boundary is a clean iso-parameter rectangle spanning a partial arc
    /// `[u_lo, u_hi]` (`u_hi` may exceed the period if the arc straddles the
    /// seam): grid that rectangle without `u` wrap. Boolean-trimmed walls such
    /// as a quarter-cylinder notch arrive this way (of-2i3).
    PartialRect { u_lo: f64, u_hi: f64 },
}

/// Classify how a cylinder/cone face maps onto its `u` period and recover the
/// `v` range its boundary spans, by projecting boundary-edge samples onto the
/// surface (cylinders and cones have an unbounded `v` domain).
///
/// A boundary covering the full period (to within [`MIN_PERIOD_COVERAGE`],
/// measured as the largest circular gap between samples) is a whole-revolution
/// wall — [`QuadricUSpan::Full`]. A boundary covering materially less is a
/// trimmed face; if it is a clean iso-parameter rectangle (every fin a rim arc
/// at `v_lo`/`v_hi` or an axial ruling at `u_lo`/`u_hi`) it grids faithfully as
/// [`QuadricUSpan::PartialRect`] (of-2i3). A trim whose boundary is *not* such
/// a rectangle — a slanted or otherwise curved-in-`uv` cut — cannot be gridded
/// without hole bridging and is rejected for the CDT pass (of-q6u).
///
/// Samples at parameterization singularities (cone apex) are excluded from the
/// `u` analysis — their `u` is arbitrary — but still bound the `v` range. That
/// includes the vertex of a degenerate loop, which is a boundary sample the
/// face carries without any fin to sample it from (of-26t).
///
/// # Errors
/// [`CoreError::NotImplemented`] if the boundary is trimmed and not a clean
/// iso-parameter rectangle.
fn boundary_param_range(
    store: &TopologyStore,
    geo: &GeometryStore,
    face_id: EntityId<Face>,
    face: &Face,
    surface: &Surface3,
) -> CoreResult<(QuadricUSpan, f64, f64)> {
    let period = surface.period_u().expect("quadric surfaces are u-periodic");
    let mut u_anchor = None;
    // (u wrapped into [0, period), v) for every non-singular boundary sample.
    let mut samples: Vec<(f64, f64)> = Vec::new();
    let mut lo = f64::INFINITY;
    let mut hi = f64::NEG_INFINITY;
    for loop_id in face
        .outer_loop
        .into_iter()
        .chain(face.inner_loops.iter().copied())
    {
        // A degenerate loop (STEP's VERTEX_LOOP, a sweep's pole loop) has no
        // fins: its single vertex is the boundary sample, and it is exactly
        // the apex the `v` range has to reach for the grid to close.
        if let Some(vertex) = store.loop_(loop_id).and_then(|lp| lp.vertex) {
            let point = store
                .vertex(vertex)
                .ok_or_else(|| invalid_face(face_id, "loop references a stale vertex"))?
                .point;
            let projected = surface.project_point(&point);
            lo = lo.min(projected.v);
            hi = hi.max(projected.v);
        }
        for &fin_id in store.fins_of_loop(loop_id) {
            let (curve, t_from, t_to) = fin_curve(store, geo, face_id, fin_id)?;
            for k in 0..=BOUNDARY_SAMPLES {
                let t = t_from + (t_to - t_from) * k as f64 / BOUNDARY_SAMPLES as f64;
                let projected = surface.project_point(&curve.point(t));
                lo = lo.min(projected.v);
                hi = hi.max(projected.v);
                if !surface.is_singular(projected.u, projected.v) {
                    if u_anchor.is_none() {
                        u_anchor = Some(projected.u);
                    }
                    samples.push((projected.u.rem_euclid(period), projected.v));
                }
            }
        }
    }
    if !(lo.is_finite() && hi.is_finite() && hi > lo) {
        return Err(invalid_face(
            face_id,
            "boundary does not span a v range on its unbounded surface",
        ));
    }
    let u_anchor = u_anchor.expect("v range implies samples");

    // Largest angular arc between consecutive samples (including the
    // wrap-around from last back to first) is the uncovered span; record where
    // it sits so the covered arc's ends can be recovered. `gap_after` indexes
    // the sample the gap starts at; `n - 1` denotes the wrap gap.
    let mut us: Vec<f64> = samples.iter().map(|&(u, _)| u).collect();
    us.sort_unstable_by(f64::total_cmp);
    let n = us.len();
    let mut max_gap = period - us[n - 1] + us[0];
    let mut gap_after = n - 1;
    for i in 0..n - 1 {
        let gap = us[i + 1] - us[i];
        if gap > max_gap {
            max_gap = gap;
            gap_after = i;
        }
    }

    if period - max_gap >= MIN_PERIOD_COVERAGE * period {
        return Ok((QuadricUSpan::Full { u_anchor }, lo, hi));
    }

    // Trimmed. The covered arc runs from the sample just after the gap around
    // to the one just before it; if the gap is the wrap, that is simply
    // [min, max].
    let (u_lo, u_hi) = if gap_after == n - 1 {
        (us[0], us[n - 1])
    } else {
        (us[gap_after + 1], us[gap_after] + period)
    };

    // Only a clean parameter rectangle [u_lo, u_hi] × [lo, hi] grids faithfully
    // without hole bridging: every boundary sample must lie on the rectangle's
    // border (each fin iso-parametric). A diagonal or curved-in-uv boundary
    // fails this and is deferred to the CDT pass (of-2i3).
    let tol_u = 1e-6 * (u_hi - u_lo) + 1e-9;
    let tol_v = 1e-6 * (hi - lo) + 1e-9;
    for &(u, v) in &samples {
        let u = if u < u_lo - tol_u { u + period } else { u };
        let inside = u >= u_lo - tol_u && u <= u_hi + tol_u;
        let on_border = (u - u_lo).abs() <= tol_u
            || (u - u_hi).abs() <= tol_u
            || (v - lo).abs() <= tol_v
            || (v - hi).abs() <= tol_v;
        if !inside || !on_border {
            return Err(CoreError::NotImplemented {
                feature: "tessellating non-rectangular trimmed cylinder/cone faces \
                          (boundary is not an iso-parameter rectangle; needs the CDT pass)",
            });
        }
    }
    Ok((QuadricUSpan::PartialRect { u_lo, u_hi }, lo, hi))
}

/// Tessellate a quadric face over its parameter rectangle: `u` over
/// `[u_lo, u_hi]` with `n_u = angular_segments(u_hi - u_lo)` segments (wrapped
/// by index if `wrap_u`, for a full-period revolution), `v` over `[v_lo, v_hi]`
/// with `n_v` segments (wrapped if `wrap_v`). Singular rows (sphere poles, cone
/// apex) collapse to a single vertex. `flip` reverses emitted normals and
/// winding, for Negative-sense faces whose outward direction opposes the
/// surface normal.
#[allow(clippy::too_many_arguments)]
fn grid_face(
    surface: &Surface3,
    u_lo: f64,
    u_hi: f64,
    wrap_u: bool,
    v_lo: f64,
    v_hi: f64,
    wrap_v: bool,
    n_v: usize,
    flip: bool,
    options: &TessellationOptions,
    mesh: &mut TriangleMesh,
) {
    let n_u = angular_segments(u_hi - u_lo, options);
    let uniform = |lo: f64, hi: f64, n: usize, wrap: bool| -> Vec<f64> {
        let count = if wrap { n } else { n + 1 };
        (0..count)
            .map(|i| {
                if !wrap && i == n {
                    hi // exact endpoint, no accumulation error
                } else {
                    lo + (hi - lo) * i as f64 / n as f64
                }
            })
            .collect()
    };
    emit_grid(
        surface,
        &uniform(u_lo, u_hi, n_u, wrap_u),
        &uniform(v_lo, v_hi, n_v, wrap_v),
        (v_lo, v_hi),
        wrap_u,
        wrap_v,
        flip,
        mesh,
    );
}

/// Emit a quad lattice at the given parameter samples. `us`/`vs` list the
/// columns and rows to emit — for a wrapped direction they omit the closing
/// duplicate, so the quad count is the list length rather than one less.
/// Singular rows (sphere poles, cone apex) collapse to a single vertex.
/// `flip` reverses emitted normals and winding, for Negative-sense faces
/// whose outward direction opposes the surface normal.
///
/// Split out of [`grid_face`] so the NURBS arm can drive the same emission
/// from a *non-uniform*, span-aware lattice ([`nurbs_lattice`]) — a freeform
/// patch has no single pitch to sweep at.
///
/// `v_range` is the face's full `v` extent, which a wrapped `vs` does not
/// end on (it omits the closing duplicate); [`grid_normal`] needs it to know
/// which way the surface interior lies.
#[allow(clippy::too_many_arguments)]
fn emit_grid(
    surface: &Surface3,
    us: &[f64],
    vs: &[f64],
    v_range: (f64, f64),
    wrap_u: bool,
    wrap_v: bool,
    flip: bool,
    mesh: &mut TriangleMesh,
) {
    let (col_count, row_count) = (us.len(), vs.len());
    let n_u = if wrap_u { col_count } else { col_count - 1 };
    let n_v = if wrap_v { row_count } else { row_count - 1 };
    let (v_lo, v_hi) = v_range;

    // rows[j] holds one vertex index per u column, or exactly one index for
    // a collapsed singular row.
    let mut rows: Vec<Vec<usize>> = Vec::with_capacity(row_count);
    for &v in vs {
        let singular = surface.is_singular(us[0], v);
        let columns = if singular { 1 } else { col_count };
        let mut row = Vec::with_capacity(columns);
        for &u in us.iter().take(columns) {
            row.push(mesh.positions.len());
            mesh.positions.push(surface.point(u, v));
            let normal = grid_normal(surface, u, v, v_lo, v_hi);
            mesh.normals.push(if flip { -normal } else { normal });
        }
        rows.push(row);
    }

    let at = |j: usize, i: usize| -> usize {
        let row = &rows[j % row_count];
        row[i % row.len()]
    };
    for j in 0..n_v {
        for i in 0..n_u {
            // Quad corners in (u, v): a --u--> b, then +v to c/d. Winding
            // follows du × dv, the surface normal — reversed when the
            // face's outward direction opposes it.
            let (a, b) = (at(j, i), at(j, i + 1));
            let (d, c) = (at(j + 1, i), at(j + 1, i + 1));
            for [p, q, r] in [[a, b, c], [a, c, d]] {
                let tri = if flip { [p, r, q] } else { [p, q, r] };
                if tri[0] != tri[1] && tri[1] != tri[2] && tri[0] != tri[2] {
                    mesh.indices.push(tri);
                }
            }
        }
    }
}

/// Parameter samples for gridding an **untrimmed** NURBS face over its knot
/// domain: `(us, vs)`, both running from the domain's low end to its high end
/// inclusive.
///
/// Only patches with a **collapsed control row** still reach this. Every
/// other NURBS face takes [`nurbs_face_cdt`], which welds and admits trimming;
/// a collapsed row has no chart at all ([`crate::boolean`]'s `Chart::build`
/// rejects it, of-37i.7), so this grid is what it has (of-37i.6). It does not
/// weld to its neighbours either — see [`span_samples`].
///
/// Two things make this different from the quadric path:
///
/// - **The lattice cannot be priced in parameter units.** A knot domain is
///   arbitrary — the same patch may be built over `[0,1]` or `[3,13]`
///   (§6/of-37i.3) — so `angular_segments` on the domain length would mesh
///   geometrically identical patches differently. Instead each knot span is
///   subdivided by how far the surface *normal turns* across it, which is a
///   property of the geometry alone. `angular_step` keeps its meaning
///   (maximum turn between adjacent samples), so a NURBS patch of exact
///   analytic form lands on the same pitch as the analytic surface it
///   duplicates: a rational quarter-cylinder turns 90°, giving the same eight
///   segments `angular_segments` gives that quarter arc. A degree-1 (planar
///   per span) patch turns not at all and grids to one segment per span,
///   which is *exact*.
/// - **Spans are honored individually.** Knot spans are the polynomial
///   pieces; the normal is only smooth within one, so turning is measured per
///   span and every interior knot is kept as a lattice line. This also puts a
///   sample on any crease the knot vector encodes.
///
/// # Errors
/// [`CoreError::NotImplemented`] if the face is *trimmed* — its boundary does
/// not run along the border of the patch's knot domain. Such a face needs
/// the CDT, which a collapsed-row patch cannot take for want of a chart; the
/// same deferral non-rectangular quadric trims take.
fn nurbs_lattice(
    store: &TopologyStore,
    geo: &GeometryStore,
    face_id: EntityId<Face>,
    face: &Face,
    surface: &Surface3,
    nurbs: &NurbsSurface,
    options: &TessellationOptions,
) -> CoreResult<(Vec<f64>, Vec<f64>)> {
    let (u0, u1) = surface.domain_u();
    let (v0, v1) = surface.domain_v();

    // Untrimmed check: every boundary sample must project onto the border of
    // the domain rectangle. A sample landing in the interior means the face is
    // a trimmed sub-region of the patch (or carries an inner loop), which this
    // grid cannot represent. Mirrors the iso-rectangle test the quadric path
    // applies to partial arcs (of-q6u).
    let tol_u = 1e-6 * (u1 - u0);
    let tol_v = 1e-6 * (v1 - v0);
    let trimmed = CoreError::NotImplemented {
        feature: "tessellating trimmed NURBS faces \
                  (boundary leaves the knot-domain border; needs the CDT pass)",
    };
    if !face.inner_loops.is_empty() {
        return Err(trimmed);
    }
    if let Some(loop_id) = face.outer_loop {
        for &fin_id in store.fins_of_loop(loop_id) {
            let (curve, t_from, t_to) = fin_curve(store, geo, face_id, fin_id)?;
            for k in 0..=BOUNDARY_SAMPLES {
                let t = t_from + (t_to - t_from) * k as f64 / BOUNDARY_SAMPLES as f64;
                let p = surface.project_point(&curve.point(t));
                let on_border = (p.u - u0).abs() <= tol_u
                    || (p.u - u1).abs() <= tol_u
                    || (p.v - v0).abs() <= tol_v
                    || (p.v - v1).abs() <= tol_v;
                if !on_border {
                    return Err(trimmed);
                }
            }
        }
    }

    Ok((
        span_samples(surface, nurbs, true, options),
        span_samples(surface, nurbs, false, options),
    ))
}

/// Samples along one parameter direction of an untrimmed NURBS patch: every
/// distinct knot in the domain, with each span subdivided into
/// `ceil(turn / angular_step)` equal steps, where `turn` is how far the
/// surface normal rotates across that span.
///
/// The turn is measured as the summed angle between normals at `2·degree + 1`
/// probes — enough to resolve a polynomial piece of that degree. Summing
/// consecutive angles recovers the full turn exactly while the normal rotates
/// monotonically, and under-counts only if it wiggles *within* a single span,
/// which the extra probes past the degree are there to catch. Probes where the
/// normal is undefined (a degenerate row) are skipped rather than guessed.
///
/// The cross-direction is probed at three stations rather than one, and the
/// worst turn wins: a patch can be flat along one edge and sharply curved
/// along the opposite one, and the lattice is shared by every row.
fn span_samples(
    surface: &Surface3,
    nurbs: &NurbsSurface,
    along_u: bool,
    options: &TessellationOptions,
) -> Vec<f64> {
    let knot_vector = if along_u {
        nurbs.knot_vector_u()
    } else {
        nurbs.knot_vector_v()
    };
    let degree = knot_vector.degree();
    let (lo, hi) = knot_vector.domain();
    // Distinct knots inside the domain (repeats are multiplicity, not spans).
    let mut breaks: Vec<f64> = vec![lo];
    for &k in knot_vector.knots() {
        if k > breaks[breaks.len() - 1] + f64::EPSILON && k <= hi {
            breaks.push(k);
        }
    }
    let (cross_lo, cross_hi) = if along_u {
        surface.domain_v()
    } else {
        surface.domain_u()
    };
    let cross_stations = [0.0, 0.5, 1.0].map(|f: f64| cross_lo + (cross_hi - cross_lo) * f);

    let normal_at = |t: f64, cross: f64| {
        if along_u {
            surface.normal(t, cross)
        } else {
            surface.normal(cross, t)
        }
    };

    let mut samples = vec![lo];
    for window in breaks.windows(2) {
        let (a, b) = (window[0], window[1]);
        let probes = 2 * degree + 1;
        let mut turn: f64 = 0.0;
        for cross in cross_stations {
            let mut station_turn = 0.0;
            let mut previous: Option<Vector3> = None;
            for i in 0..=probes {
                let t = a + (b - a) * i as f64 / probes as f64;
                let Some(n) = normal_at(t, cross) else {
                    continue;
                };
                if let Some(p) = previous {
                    station_turn += p.angle(&n);
                }
                previous = Some(n);
            }
            turn = turn.max(station_turn);
        }
        let steps = ((turn / options.angular_step).ceil() as usize).max(1);
        for i in 1..=steps {
            // Exact endpoint on the last step, so consecutive spans meet bit
            // for bit. This does *not* extend to the neighbouring face across
            // a shared edge — it samples that edge by its own rule, and for a
            // curved patch the two disagree. That is why only collapsed-row
            // patches, which have no chart and so cannot take the CDT, still
            // come through here (of-dvj, of-37i.6).
            let t = if i == steps {
                b
            } else {
                a + (b - a) * i as f64 / steps as f64
            };
            samples.push(t);
        }
    }
    samples
}

/// Surface normal for a grid vertex. Where the parameterization is
/// degenerate *and* has no limit normal (cone apex — sphere poles do have
/// one), nudge `v` toward the range interior for a usable shading normal.
fn grid_normal(surface: &Surface3, u: f64, v: f64, v_lo: f64, v_hi: f64) -> Vector3 {
    surface.normal(u, v).unwrap_or_else(|| {
        let mid = (v_lo + v_hi) / 2.0;
        let nudged = v + (mid - v) * 1e-6;
        surface.normal(u, nudged).unwrap_or_else(Vector3::zeros)
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::nurbs::KnotVector;
    use crate::primitives;
    use std::f64::consts::{FRAC_1_SQRT_2, PI, TAU};

    /// Exact unit circle as a rational quadratic: four knot spans, one per
    /// 90° arc (Piegl & Tiller §7.5).
    fn nurbs_unit_circle() -> NurbsCurve {
        let s = FRAC_1_SQRT_2;
        NurbsCurve::new(
            vec![
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
                Point3::new(-1.0, 1.0, 0.0),
                Point3::new(-1.0, 0.0, 0.0),
                Point3::new(-1.0, -1.0, 0.0),
                Point3::new(0.0, -1.0, 0.0),
                Point3::new(1.0, -1.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
            ],
            vec![1.0, s, 1.0, s, 1.0, s, 1.0, s, 1.0],
            KnotVector::new(
                2,
                vec![
                    0.0, 0.0, 0.0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1.0, 1.0, 1.0,
                ],
            )
            .unwrap(),
        )
        .unwrap()
    }

    /// A NURBS circle must be segmented like [`Curve3::Circle`] is: the
    /// tangent turns a full 2π, so the tangent-turn measure has to land on
    /// the same count `angular_segments` gives that sweep.
    #[test]
    fn nurbs_curve_segments_match_a_conic_of_the_same_turn() {
        let options = TessellationOptions::default();
        let circle = nurbs_unit_circle();
        assert_eq!(
            nurbs_curve_segments(&circle, 0.0, 1.0, &options),
            angular_segments(TAU, &options),
            "a full NURBS circle must segment like a full analytic circle"
        );
        // Half the domain is half the turn.
        assert_eq!(
            nurbs_curve_segments(&circle, 0.0, 0.5, &options),
            angular_segments(PI, &options)
        );
    }

    /// A straight curve turns nowhere, so the count falls back to the
    /// per-knot-span floor rather than collapsing to zero segments.
    #[test]
    fn nurbs_curve_segments_floor_at_one_per_knot_span() {
        let options = TessellationOptions::default();
        let line = NurbsCurve::bspline(
            vec![Point3::origin(), Point3::new(10.0, 0.0, 0.0)],
            KnotVector::new(1, vec![0.0, 0.0, 3.0, 3.0]).unwrap(),
        )
        .unwrap();
        assert_eq!(nurbs_curve_segments(&line, 0.0, 3.0, &options), 1);

        // Three collinear spans: still straight, but each polynomial piece
        // gets its own segment, because a repeated knot could hide a kink.
        let kinked = NurbsCurve::bspline(
            vec![
                Point3::origin(),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(2.0, 0.0, 0.0),
                Point3::new(3.0, 0.0, 0.0),
            ],
            KnotVector::new(1, vec![0.0, 0.0, 1.0, 2.0, 3.0, 3.0]).unwrap(),
        )
        .unwrap();
        assert_eq!(nurbs_curve_segments(&kinked, 0.0, 3.0, &options), 3);
        // Restricting to one span drops the floor with it.
        assert_eq!(nurbs_curve_segments(&kinked, 0.0, 1.0, &options), 1);
    }

    /// A finer `angular_step` must buy more segments on the same curve.
    #[test]
    fn nurbs_curve_segments_respect_the_angular_step() {
        let circle = nurbs_unit_circle();
        let coarse = nurbs_curve_segments(&circle, 0.0, 1.0, &TessellationOptions::default());
        let fine = nurbs_curve_segments(
            &circle,
            0.0,
            1.0,
            &TessellationOptions {
                angular_step: TAU / 128.0,
            },
        );
        assert!(fine > coarse, "finer step must refine: {fine} vs {coarse}");
    }

    fn build(
        make: impl FnOnce(&mut TopologyStore, &mut GeometryStore) -> CoreResult<EntityId<Body>>,
    ) -> TriangleMesh {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = make(&mut store, &mut geo).expect("valid primitive");
        tessellate_body(&store, &geo, body, &TessellationOptions::default())
            .expect("tessellation succeeds")
    }

    /// Signed volume via the divergence theorem: positive iff triangles
    /// wind outward consistently.
    fn signed_volume(mesh: &TriangleMesh) -> f64 {
        mesh.indices
            .iter()
            .map(|tri| {
                let [a, b, c] = tri.map(|i| mesh.positions[i].coords);
                a.dot(&b.cross(&c)) / 6.0
            })
            .sum()
    }

    /// Euler characteristic V - E + F of a closed mesh.
    fn euler_characteristic(mesh: &TriangleMesh) -> i64 {
        let mut edges = std::collections::HashSet::new();
        for tri in &mesh.indices {
            for e in 0..3 {
                let (a, b) = (tri[e], tri[(e + 1) % 3]);
                edges.insert((a.min(b), a.max(b)));
            }
        }
        mesh.vertex_count() as i64 - edges.len() as i64 + mesh.triangle_count() as i64
    }

    fn assert_within(actual: f64, expected: f64, fraction: f64, what: &str) {
        assert!(
            (actual - expected).abs() <= expected.abs() * fraction,
            "{what}: {actual} vs expected {expected} (>{:.1}%)",
            fraction * 100.0
        );
    }

    /// A boolean that leaves a partially-trimmed quadric wall — block minus a
    /// corner cylinder, whose kept wall is a quarter-cylinder (a clean
    /// parameter rectangle) — must now tessellate to a closed manifold with the
    /// right volume, matching [`crate::boolean::BooleanOutput::tessellate`]
    /// (of-2i3). Previously the full-period assumption rejected it (of-q6u).
    #[test]
    fn quarter_cylinder_notch_is_watertight() {
        use opensolid_core::Transform3;
        use opensolid_core::tolerance::ToleranceContext;
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let block = primitives::block(&mut store, &mut geo, 2.0, 2.0, 2.0).unwrap();
        let tool = primitives::cylinder(&mut store, &mut geo, 0.4, 3.0).unwrap();
        // Center the tool on the block's vertical corner edge (1, 1): only a
        // quarter of the tube lies inside the block, so the kept cylinder wall
        // is a quarter-arc — a partial-period, iso-rectangular quadric face.
        crate::transform::transform_body(
            &mut store,
            &mut geo,
            tool,
            &Transform3::translation(1.0, 1.0, 0.0),
        )
        .unwrap();
        let out = crate::boolean::subtract(&store, &geo, block, tool, &ToleranceContext::default())
            .expect("subtract");
        assert!(
            out.check().is_empty(),
            "boolean output invalid: {:?}",
            out.check()
        );
        let reference = out.tessellate().expect("BooleanOutput::tessellate");

        let mesh = tessellate_body(
            &out.store,
            &out.geo,
            out.body,
            &TessellationOptions::default(),
        )
        .expect("tessellate_body must grid the quarter-cylinder wall (of-2i3)");
        assert!(
            mesh.is_closed_manifold(),
            "notch mesh must be watertight, got {} tris",
            mesh.triangle_count()
        );
        // Block volume 8 minus a quarter-cylinder r=0.4 h=2: 8 - πr²h/4 ≈ 7.749.
        let expected = 8.0 - std::f64::consts::PI * 0.4 * 0.4 * 2.0 / 4.0;
        assert_within(signed_volume(&mesh), expected, 0.02, "notch volume");
        assert_within(
            signed_volume(&mesh),
            signed_volume(&reference),
            0.02,
            "notch volume vs BooleanOutput::tessellate",
        );
    }

    /// The of-fc8 case: a drilled plate. Its top and bottom faces are planes
    /// carrying a circular inner loop, which `tessellate_body` used to reject
    /// outright (`NotImplemented`) — so no real mechanical part could be
    /// measured. Bridging the hole into the outer loop tessellates it to a
    /// watertight genus-1 mesh with the drilled volume.
    #[test]
    fn drilled_plate_is_watertight_with_the_hole_open() {
        use opensolid_core::tolerance::ToleranceContext;
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let (w, t, r) = (4.0, 2.0, 1.0);
        let plate = primitives::block(&mut store, &mut geo, w, w, t).unwrap();
        // Concentric with the plate and longer than it: pierces clean through,
        // so top and bottom each gain a circular inner loop.
        let tool = primitives::cylinder(&mut store, &mut geo, r, 2.0 * t).unwrap();
        let out = crate::boolean::subtract(&store, &geo, plate, tool, &ToleranceContext::default())
            .expect("subtract");
        let mesh = tessellate_body(
            &out.store,
            &out.geo,
            out.body,
            &TessellationOptions::default(),
        )
        .expect("planar faces with holes must tessellate (of-fc8)");

        assert!(
            mesh.is_closed_manifold(),
            "drilled plate must be watertight, got {} tris",
            mesh.triangle_count()
        );
        // Genus 1: V - E + F = 2 - 2g = 0. This is what proves the hole is
        // actually open — a mesh that paved over it would have the right
        // topology only by closing the tube, and its volume would be wrong.
        assert_eq!(euler_characteristic(&mesh), 0, "through hole is genus 1");
        let expected = w * w * t - PI * r * r * t;
        assert_within(signed_volume(&mesh), expected, 0.02, "drilled plate volume");
        assert_within(
            signed_volume(&mesh),
            signed_volume(&out.tessellate().expect("BooleanOutput::tessellate")),
            0.02,
            "volume vs BooleanOutput::tessellate",
        );
    }

    #[test]
    fn block_mesh_is_exact() {
        let mesh = build(|s, g| primitives::block(s, g, 2.0, 3.0, 4.0));
        assert!(mesh.is_closed_manifold());
        assert_eq!(mesh.triangle_count(), 12, "two triangles per face");
        assert_eq!(mesh.vertex_count(), 8, "corners welded across faces");
        assert_eq!(euler_characteristic(&mesh), 2);
        // Flat faces tessellate exactly, not approximately.
        let area = 2.0 * (2.0 * 3.0 + 3.0 * 4.0 + 4.0 * 2.0);
        assert!((mesh.total_area() - area).abs() < 1e-9);
        assert!((signed_volume(&mesh) - 24.0).abs() < 1e-9);
        let bbox = mesh.bounding_box().unwrap();
        assert!((bbox.min - Point3::new(-1.0, -1.5, -2.0)).norm() < 1e-9);
        assert!((bbox.max - Point3::new(1.0, 1.5, 2.0)).norm() < 1e-9);
    }

    #[test]
    fn cylinder_mesh_is_closed_and_accurate() {
        let (r, h) = (1.5, 5.0);
        let mesh = build(|s, g| primitives::cylinder(s, g, r, h));
        assert!(mesh.is_closed_manifold());
        assert_eq!(euler_characteristic(&mesh), 2);
        assert_within(
            mesh.total_area(),
            TAU * r * h + TAU * r * r,
            0.05,
            "cylinder area",
        );
        assert_within(
            signed_volume(&mesh),
            PI * r * r * h,
            0.05,
            "cylinder volume",
        );
    }

    #[test]
    fn sphere_mesh_is_closed_and_accurate() {
        let r = 2.5;
        let mesh = build(|s, g| primitives::sphere(s, g, r));
        assert!(mesh.is_closed_manifold());
        assert_eq!(euler_characteristic(&mesh), 2);
        assert_within(mesh.total_area(), 2.0 * TAU * r * r, 0.05, "sphere area");
        assert_within(
            signed_volume(&mesh),
            2.0 / 3.0 * TAU * r * r * r,
            0.05,
            "sphere volume",
        );
    }

    #[test]
    fn torus_mesh_is_closed_genus_one_and_accurate() {
        let (major, minor) = (3.0, 1.0);
        let mesh = build(|s, g| primitives::torus(s, g, major, minor));
        assert!(mesh.is_closed_manifold());
        assert_eq!(euler_characteristic(&mesh), 0, "torus has genus 1");
        assert_within(
            mesh.total_area(),
            TAU * TAU * major * minor,
            0.05,
            "torus area",
        );
        assert_within(
            signed_volume(&mesh),
            PI * TAU * major * minor * minor,
            0.05,
            "torus volume",
        );
    }

    #[test]
    fn convex_body_normals_point_outward() {
        // All four bodies are centered at the origin; for the convex ones
        // every outward direction has positive dot with its position.
        for mesh in [
            build(|s, g| primitives::block(s, g, 2.0, 3.0, 4.0)),
            build(|s, g| primitives::cylinder(s, g, 1.5, 5.0)),
            build(|s, g| primitives::sphere(s, g, 2.5)),
        ] {
            for (position, normal) in mesh.positions.iter().zip(&mesh.normals) {
                assert!((normal.norm() - 1.0).abs() < 1e-9, "vertex normal not unit");
                assert!(
                    normal.dot(&position.coords) > 0.0,
                    "inward vertex normal at {position:?}"
                );
            }
            for tri in &mesh.indices {
                let [a, b, c] = tri.map(|i| mesh.positions[i]);
                let geometric = (b - a).cross(&(c - a));
                let centroid = (a.coords + b.coords + c.coords) / 3.0;
                assert!(
                    geometric.dot(&centroid) > 0.0,
                    "inward triangle winding at {centroid:?}"
                );
            }
        }
    }

    #[test]
    fn torus_normals_agree_with_surface() {
        // The inner ring's normals point toward the axis, so the convex
        // dot-with-position test does not apply; check against the exact
        // tube normal instead: (p - ring_center)/minor for each vertex.
        let (major, minor) = (3.0, 1.0);
        let mesh = build(|s, g| primitives::torus(s, g, major, minor));
        for (position, normal) in mesh.positions.iter().zip(&mesh.normals) {
            let ring = Vector3::new(position.x, position.y, 0.0).normalize() * major;
            let exact = (position.coords - ring) / minor;
            assert!(
                (normal - exact).norm() < 1e-6,
                "normal {normal:?} vs tube normal {exact:?}"
            );
        }
    }

    #[test]
    fn finer_angular_step_converges() {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = primitives::sphere(&mut store, &mut geo, 1.0).expect("valid sphere");
        let exact = 2.0 * TAU;
        let area = |step: f64| {
            tessellate_body(
                &store,
                &geo,
                body,
                &TessellationOptions { angular_step: step },
            )
            .expect("tessellation succeeds")
            .total_area()
        };
        let coarse = (area(TAU / 16.0) - exact).abs();
        let fine = (area(TAU / 64.0) - exact).abs();
        assert!(
            fine < coarse / 4.0,
            "quadratic convergence expected: coarse err {coarse}, fine err {fine}"
        );
    }

    #[test]
    fn concave_planar_face_tiles_without_overlap() {
        use crate::topology::{
            BodyType, FaceSense, FinSense, LoopType, SYSTEM_RESOLUTION, ShellOrientation,
        };
        // A concave U outline in the z=0 plane (of-6dw): the old
        // first-vertex fan spilled across the notch and emitted
        // overlapping, mixed-winding triangles that inflate the area. Ear
        // clipping tiles the polygon exactly.
        let outline = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(3.0, 0.0, 0.0),
            Point3::new(3.0, 3.0, 0.0),
            Point3::new(2.0, 3.0, 0.0),
            Point3::new(2.0, 1.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(1.0, 3.0, 0.0),
            Point3::new(0.0, 3.0, 0.0),
        ];
        let n = outline.len();

        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = store.create_body(BodyType::Solid);
        let shell = store.create_shell(body, true, ShellOrientation::Outward);
        let verts: Vec<_> = outline
            .iter()
            .map(|&p| store.create_vertex(p, SYSTEM_RESOLUTION))
            .collect();
        let plane = Surface3::plane(outline[0], Vector3::z()).expect("valid plane");
        let face = store.create_face(shell, FaceSense::Positive);
        store.faces.get_mut(face).expect("just created").surface = Some(geo.add_surface(plane));
        let loop_edges: Vec<_> = (0..n)
            .map(|i| {
                let (a, b) = (outline[i], outline[(i + 1) % n]);
                let curve = geo.add_curve(Curve3::line(a, b - a).expect("valid line"));
                let edge = store.create_edge_with_curve(
                    verts[i],
                    verts[(i + 1) % n],
                    SYSTEM_RESOLUTION,
                    curve,
                    0.0,
                    (b - a).norm(),
                );
                (edge, FinSense::Forward)
            })
            .collect();
        store.create_loop(face, LoopType::Outer, &loop_edges);

        let mesh = tessellate_face(&store, &geo, face, &TessellationOptions::default())
            .expect("tessellation succeeds");
        assert_eq!(
            mesh.triangle_count(),
            n - 2,
            "n-2 triangles tile the polygon"
        );
        // Exact area of the U outline (shoelace = 7).
        assert!(
            (mesh.total_area() - 7.0).abs() < 1e-9,
            "cap triangles overlap: area {} != 7",
            mesh.total_area()
        );
        // Every triangle winds counterclockwise about +z (outward).
        for tri in &mesh.indices {
            let [a, b, c] = tri.map(|i| mesh.positions[i]);
            let facing = (b - a).cross(&(c - a));
            assert!(facing.z > 0.0, "triangle winds inward: {facing:?}");
        }
    }

    /// of-as6: a subtract's tool-derived faces bind the tool's surfaces
    /// with `FaceSense::Negative` (outward opposes the surface normal).
    /// Ignoring the sense wound those caps inward, so the welded mesh
    /// failed the manifold orientation check on every imprinted edge.
    #[test]
    fn boolean_corner_notch_tessellates_closed() {
        use opensolid_core::Transform3;
        use opensolid_core::tolerance::ToleranceContext;

        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let a = primitives::block(&mut store, &mut geo, 2.0, 2.0, 2.0).expect("valid block");
        let b = primitives::block(&mut store, &mut geo, 2.0, 2.0, 2.0).expect("valid block");
        crate::transform::transform_body(
            &mut store,
            &mut geo,
            b,
            &Transform3::translation(1.0, 1.0, 1.0),
        )
        .expect("rigid translation");
        let out = crate::boolean::subtract(&store, &geo, a, b, &ToleranceContext::default())
            .expect("transversal subtract");
        assert!(out.check().is_empty(), "boolean result must be valid");

        let mesh = tessellate_body(
            &out.store,
            &out.geo,
            out.body,
            &TessellationOptions::default(),
        )
        .expect("tessellation succeeds");
        assert!(mesh.is_closed_manifold(), "L-shape mesh must be watertight");
        assert_eq!(euler_characteristic(&mesh), 2);
        // Unit corner removed from the 2×2×2 block: volume 8 - 1, area
        // unchanged at 24 (three notch walls replace the removed corner).
        assert!((signed_volume(&mesh) - 7.0).abs() < 1e-9);
        assert!((mesh.total_area() - 24.0).abs() < 1e-9);
    }

    /// A valid body re-encoded with inward surface normals (negated plane
    /// normal + Negative sense, as an importer may produce — of-alr) must
    /// tessellate identically to its all-Positive twin.
    #[test]
    fn flipped_encoding_block_tessellates_identically() {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = primitives::block(&mut store, &mut geo, 2.0, 3.0, 4.0).expect("valid block");
        for face_id in store.faces_of_body(body) {
            let surface_id = store.face(face_id).unwrap().surface.expect("bound surface");
            let flipped = match geo.surface(surface_id).expect("live surface") {
                Surface3::Plane { origin, normal } => {
                    Surface3::plane(*origin, -*normal).expect("valid plane")
                }
                other => panic!("block faces are planes, got {other:?}"),
            };
            let new_id = geo.add_surface(flipped);
            let face = store.faces.get_mut(face_id).expect("live face");
            face.surface = Some(new_id);
            face.sense = crate::topology::FaceSense::Negative;
        }

        let mesh = tessellate_body(&store, &geo, body, &TessellationOptions::default())
            .expect("tessellation succeeds");
        assert!(mesh.is_closed_manifold());
        assert!((signed_volume(&mesh) - 24.0).abs() < 1e-9);
        for (position, normal) in mesh.positions.iter().zip(&mesh.normals) {
            assert!(
                normal.dot(&position.coords) > 0.0,
                "inward vertex normal at {position:?}"
            );
        }
    }

    /// The quadric grid honors face sense too: flipping a sphere's faces to
    /// Negative reverses winding and normals wholesale, yielding a still-
    /// manifold mesh that bounds the same region from the other side.
    #[test]
    fn negative_sense_sphere_winds_inward() {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = primitives::sphere(&mut store, &mut geo, 1.0).expect("valid sphere");
        let outward = tessellate_body(&store, &geo, body, &TessellationOptions::default())
            .expect("tessellation succeeds");
        for face_id in store.faces_of_body(body) {
            store.faces.get_mut(face_id).expect("live face").sense =
                crate::topology::FaceSense::Negative;
        }
        let inward = tessellate_body(&store, &geo, body, &TessellationOptions::default())
            .expect("tessellation succeeds");
        assert!(inward.is_closed_manifold(), "flip preserves manifoldness");
        assert!(
            (signed_volume(&inward) + signed_volume(&outward)).abs() < 1e-9,
            "Negative sense reverses the enclosed signed volume"
        );
        // On a unit sphere at the origin the exact outward normal is the
        // position itself; Negative sense must emit the negation. (The two
        // meshes' vertex orders differ — weld numbers vertices in triangle
        // order — so compare against the analytic normal, not index-wise.)
        for (position, normal) in inward.positions.iter().zip(&inward.normals) {
            assert!(
                (normal + position.coords).norm() < 1e-9,
                "normal {normal:?} at {position:?} does not point inward"
            );
        }
    }

    #[test]
    fn single_face_mesh_is_open() {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = primitives::cylinder(&mut store, &mut geo, 1.0, 2.0).expect("valid cylinder");
        // Face order from the builder: bottom cap, top cap, wall.
        let wall = store.faces_of_body(body)[2];
        let mesh = tessellate_face(&store, &geo, wall, &TessellationOptions::default())
            .expect("tessellation succeeds");
        assert!(!mesh.is_empty());
        assert!(!mesh.is_closed_manifold(), "a lone wall is an open tube");
    }

    #[test]
    fn rejects_invalid_options_and_stale_body() {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = primitives::sphere(&mut store, &mut geo, 1.0).expect("valid sphere");

        for bad in [0.0, -0.1, f64::NAN] {
            let err = tessellate_body(
                &store,
                &geo,
                body,
                &TessellationOptions { angular_step: bad },
            )
            .unwrap_err();
            assert!(
                matches!(
                    err,
                    CoreError::InvalidArgument {
                        argument: "angular_step",
                        ..
                    }
                ),
                "step {bad}: got {err}"
            );
        }

        let stale = body;
        store.bodies.remove(body);
        let err =
            tessellate_body(&store, &geo, stale, &TessellationOptions::default()).unwrap_err();
        assert!(
            matches!(
                err,
                CoreError::InvalidArgument {
                    argument: "body",
                    ..
                }
            ),
            "got {err}"
        );
    }

    #[test]
    fn rejects_faces_without_geometry() {
        // An mvfs-seeded body has a face but no attached surface.
        let mut store = TopologyStore::new();
        let geo = GeometryStore::new();
        let (body, ..) = store.mvfs(Point3::origin());
        let err = tessellate_body(&store, &geo, body, &TessellationOptions::default()).unwrap_err();
        assert!(
            matches!(
                err,
                CoreError::InvalidArgument {
                    argument: "body",
                    ..
                }
            ),
            "got {err}"
        );
        assert!(err.to_string().contains("surface"), "unhelpful: {err}");
    }

    mod trimmed_face_guard {
        use super::*;
        use crate::topology::{BodyType, FinSense, LoopType, SYSTEM_RESOLUTION, ShellOrientation};

        /// Empty store pair plus one face on one shell, with `surface`
        /// attached — the scaffolding every trimmed-face fixture needs.
        fn face_on(
            store: &mut TopologyStore,
            geo: &mut GeometryStore,
            surface: Surface3,
        ) -> EntityId<Face> {
            let body = store.create_body(BodyType::Solid);
            let shell = store.create_shell(body, true, ShellOrientation::Outward);
            let face = store.create_face(shell, FaceSense::Positive);
            store.faces.get_mut(face).expect("just created").surface =
                Some(geo.add_surface(surface));
            face
        }

        fn expect_not_implemented(
            store: &TopologyStore,
            geo: &GeometryStore,
            face: EntityId<Face>,
        ) {
            let err = tessellate_face(store, geo, face, &TessellationOptions::default())
                .expect_err("trimmed quadric face must be rejected, not gridded in full");
            assert!(
                matches!(err, CoreError::NotImplemented { .. }),
                "got {err:?}"
            );
        }

        /// A half-period cylinder wedge (two half-rims and two axial sides) is
        /// a clean iso-parameter rectangle `[0, π] × [0, h]`, so it grids
        /// faithfully as a partial arc (of-2i3) rather than being rejected or
        /// rendered as the full cylinder.
        #[test]
        fn accepts_half_cylinder_wedge() {
            let mut store = TopologyStore::new();
            let mut geo = GeometryStore::new();
            let (r, h) = (1.0, 2.0);
            let axis = Vector3::z();
            let face = face_on(
                &mut store,
                &mut geo,
                Surface3::cylinder(Point3::origin(), axis, r).unwrap(),
            );

            let bottom = geo.add_curve(Curve3::circle(Point3::origin(), axis, r).unwrap());
            let top = geo.add_curve(Curve3::circle(Point3::new(0.0, 0.0, h), axis, r).unwrap());
            let side0 = geo.add_curve(Curve3::line(Point3::new(r, 0.0, 0.0), axis).unwrap());
            let side1 = geo.add_curve(Curve3::line(Point3::new(-r, 0.0, 0.0), axis).unwrap());

            let vb0 = store.create_vertex(Point3::new(r, 0.0, 0.0), SYSTEM_RESOLUTION);
            let vb1 = store.create_vertex(Point3::new(-r, 0.0, 0.0), SYSTEM_RESOLUTION);
            let vt0 = store.create_vertex(Point3::new(r, 0.0, h), SYSTEM_RESOLUTION);
            let vt1 = store.create_vertex(Point3::new(-r, 0.0, h), SYSTEM_RESOLUTION);

            let e_bottom =
                store.create_edge_with_curve(vb0, vb1, SYSTEM_RESOLUTION, bottom, 0.0, PI);
            let e_top = store.create_edge_with_curve(vt0, vt1, SYSTEM_RESOLUTION, top, 0.0, PI);
            let e_side0 = store.create_edge_with_curve(vb0, vt0, SYSTEM_RESOLUTION, side0, 0.0, h);
            let e_side1 = store.create_edge_with_curve(vb1, vt1, SYSTEM_RESOLUTION, side1, 0.0, h);
            store.create_loop(
                face,
                LoopType::Outer,
                &[
                    (e_bottom, FinSense::Forward),
                    (e_side1, FinSense::Forward),
                    (e_top, FinSense::Reversed),
                    (e_side0, FinSense::Reversed),
                ],
            );

            let mesh = tessellate_face(&store, &geo, face, &TessellationOptions::default())
                .expect("iso-rectangular half-cylinder wedge must grid (of-2i3)");
            // Half of the lateral surface: π·r·h, not the full TAU·r·h.
            assert_within(
                mesh.total_area(),
                PI * r * h,
                0.05,
                "half-cylinder wedge area",
            );
        }

        /// A trimmed cylinder face whose boundary is *not* an iso-parameter
        /// rectangle — a diagonal edge running across the surface in both `u`
        /// and `v` at once (as a slanted cut leaves) — cannot be gridded on the
        /// `u × v` lattice without hole bridging, and must defer to the CDT pass
        /// (of-2i3) rather than being gridded wrong.
        #[test]
        fn rejects_diagonal_cylinder_trim() {
            let mut store = TopologyStore::new();
            let mut geo = GeometryStore::new();
            let (r, h) = (1.0, 2.0);
            let axis = Vector3::z();
            let face = face_on(
                &mut store,
                &mut geo,
                Surface3::cylinder(Point3::origin(), axis, r).unwrap(),
            );

            // Right-triangle patch: a quarter rim arc (v = 0, u ∈ [0, π/2]), an
            // axial side (u = π/2, v ∈ [0, h]), and a diagonal hypotenuse whose
            // interior samples fall inside the parameter rectangle, not on its
            // border.
            let va = Point3::new(r, 0.0, 0.0); // u = 0,    v = 0
            let vb = Point3::new(0.0, r, 0.0); // u = π/2,  v = 0
            let vc = Point3::new(0.0, r, h); //   u = π/2,  v = h

            let arc = geo.add_curve(Curve3::circle(Point3::origin(), axis, r).unwrap());
            let side = geo.add_curve(Curve3::line(vb, axis).unwrap());
            let hyp = geo.add_curve(Curve3::line(vc, va - vc).unwrap());

            let vid_a = store.create_vertex(va, SYSTEM_RESOLUTION);
            let vid_b = store.create_vertex(vb, SYSTEM_RESOLUTION);
            let vid_c = store.create_vertex(vc, SYSTEM_RESOLUTION);

            let e_arc =
                store.create_edge_with_curve(vid_a, vid_b, SYSTEM_RESOLUTION, arc, 0.0, PI / 2.0);
            let e_side =
                store.create_edge_with_curve(vid_b, vid_c, SYSTEM_RESOLUTION, side, 0.0, h);
            let e_hyp = store.create_edge_with_curve(
                vid_c,
                vid_a,
                SYSTEM_RESOLUTION,
                hyp,
                0.0,
                (va - vc).norm(),
            );
            store.create_loop(
                face,
                LoopType::Outer,
                &[
                    (e_arc, FinSense::Forward),
                    (e_side, FinSense::Forward),
                    (e_hyp, FinSense::Forward),
                ],
            );

            expect_not_implemented(&store, &geo, face);
        }

        /// A spherical cap (one latitude-circle boundary, traversed once —
        /// not a seam) must be rejected, not rendered as the full sphere.
        #[test]
        fn rejects_sphere_cap() {
            let mut store = TopologyStore::new();
            let mut geo = GeometryStore::new();
            let r = 2.0;
            let latitude = PI / 4.0;
            let (rim_r, rim_z) = (r * latitude.cos(), r * latitude.sin());
            let face = face_on(
                &mut store,
                &mut geo,
                Surface3::sphere(Point3::origin(), Vector3::z(), r).unwrap(),
            );

            let rim = geo.add_curve(
                Curve3::circle(Point3::new(0.0, 0.0, rim_z), Vector3::z(), rim_r).unwrap(),
            );
            let v_rim = store.create_vertex(Point3::new(rim_r, 0.0, rim_z), SYSTEM_RESOLUTION);
            let e_rim =
                store.create_edge_with_curve(v_rim, v_rim, SYSTEM_RESOLUTION, rim, 0.0, TAU);
            store.create_loop(face, LoopType::Outer, &[(e_rim, FinSense::Forward)]);

            expect_not_implemented(&store, &geo, face);
        }

        /// A half-torus band (two tube-circle boundaries, each traversed
        /// once) must be rejected, not rendered as the full torus.
        #[test]
        fn rejects_half_torus_band() {
            let mut store = TopologyStore::new();
            let mut geo = GeometryStore::new();
            let (major, minor) = (3.0, 1.0);
            let face = face_on(
                &mut store,
                &mut geo,
                Surface3::torus(Point3::origin(), Vector3::z(), major, minor).unwrap(),
            );

            let tube_start = geo.add_curve(
                Curve3::circle(Point3::new(major, 0.0, 0.0), -Vector3::y(), minor).unwrap(),
            );
            let tube_end = geo.add_curve(
                Curve3::circle(Point3::new(-major, 0.0, 0.0), Vector3::y(), minor).unwrap(),
            );
            let v_start =
                store.create_vertex(Point3::new(major + minor, 0.0, 0.0), SYSTEM_RESOLUTION);
            let v_end =
                store.create_vertex(Point3::new(-major - minor, 0.0, 0.0), SYSTEM_RESOLUTION);
            let e_start = store.create_edge_with_curve(
                v_start,
                v_start,
                SYSTEM_RESOLUTION,
                tube_start,
                0.0,
                TAU,
            );
            let e_end =
                store.create_edge_with_curve(v_end, v_end, SYSTEM_RESOLUTION, tube_end, 0.0, TAU);
            // Revolve-style loop layout: end circle outer, start circle inner.
            store.create_loop(face, LoopType::Outer, &[(e_end, FinSense::Forward)]);
            store.create_loop(face, LoopType::Inner, &[(e_start, FinSense::Reversed)]);

            expect_not_implemented(&store, &geo, face);
        }

        /// A wall whose rims are split into two half-circle edges each (as
        /// imprinting produces) still covers the full period and must pass
        /// the coverage guard.
        #[test]
        fn accepts_full_ring_of_split_arcs() {
            let mut store = TopologyStore::new();
            let mut geo = GeometryStore::new();
            let (r, h) = (1.5, 2.0);
            let axis = Vector3::z();
            let face = face_on(
                &mut store,
                &mut geo,
                Surface3::cylinder(Point3::origin(), axis, r).unwrap(),
            );

            let bottom = geo.add_curve(Curve3::circle(Point3::origin(), axis, r).unwrap());
            let top = geo.add_curve(Curve3::circle(Point3::new(0.0, 0.0, h), axis, r).unwrap());
            let seam = geo.add_curve(Curve3::line(Point3::new(r, 0.0, 0.0), axis).unwrap());

            let vb0 = store.create_vertex(Point3::new(r, 0.0, 0.0), SYSTEM_RESOLUTION);
            let vb1 = store.create_vertex(Point3::new(-r, 0.0, 0.0), SYSTEM_RESOLUTION);
            let vt0 = store.create_vertex(Point3::new(r, 0.0, h), SYSTEM_RESOLUTION);
            let vt1 = store.create_vertex(Point3::new(-r, 0.0, h), SYSTEM_RESOLUTION);

            let e_b1 = store.create_edge_with_curve(vb0, vb1, SYSTEM_RESOLUTION, bottom, 0.0, PI);
            let e_b2 = store.create_edge_with_curve(vb1, vb0, SYSTEM_RESOLUTION, bottom, PI, TAU);
            let e_t1 = store.create_edge_with_curve(vt0, vt1, SYSTEM_RESOLUTION, top, 0.0, PI);
            let e_t2 = store.create_edge_with_curve(vt1, vt0, SYSTEM_RESOLUTION, top, PI, TAU);
            let e_seam = store.create_edge_with_curve(vb0, vt0, SYSTEM_RESOLUTION, seam, 0.0, h);
            store.create_loop(
                face,
                LoopType::Outer,
                &[
                    (e_b1, FinSense::Forward),
                    (e_b2, FinSense::Forward),
                    (e_seam, FinSense::Forward),
                    (e_t2, FinSense::Reversed),
                    (e_t1, FinSense::Reversed),
                    (e_seam, FinSense::Reversed),
                ],
            );

            let mesh = tessellate_face(&store, &geo, face, &TessellationOptions::default())
                .expect("full-ring boundary must pass the coverage guard");
            assert_within(mesh.total_area(), TAU * r * h, 0.05, "split-ring wall area");
        }
    }

    /// The NURBS arm (of-ew7): an untrimmed patch grids off how far its
    /// normal turns, so the lattice is a property of the geometry and not of
    /// the knot domain it happens to be parameterized over.
    mod nurbs_faces {
        use super::*;
        use crate::nurbs::{KnotVector, NurbsSurface};
        use crate::topology::{BodyType, FinSense, LoopType, SYSTEM_RESOLUTION, ShellOrientation};
        use std::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2};

        /// Clamped degree-1 knots for `n` control points over `[a, b]`.
        fn deg1_knots(n: usize, (a, b): (f64, f64)) -> KnotVector {
            let mut knots = vec![a];
            for i in 0..n {
                knots.push(a + (b - a) * i as f64 / (n - 1) as f64);
            }
            knots.push(b);
            KnotVector::new(1, knots).expect("valid clamped degree-1 knots")
        }

        /// A face carrying `surface`, bounded by the four straight edges
        /// through `corners` (in `(0,0) → (1,0) → (1,1) → (0,1)` domain
        /// order) — the untrimmed boundary of a patch whose domain border maps
        /// to those corners.
        fn quad_face(
            store: &mut TopologyStore,
            geo: &mut GeometryStore,
            surface: Surface3,
            corners: [Point3; 4],
        ) -> EntityId<Face> {
            let body = store.create_body(BodyType::Solid);
            let shell = store.create_shell(body, true, ShellOrientation::Outward);
            let face = store.create_face(shell, FaceSense::Positive);
            store.faces.get_mut(face).expect("just created").surface =
                Some(geo.add_surface(surface));
            let vertices = corners.map(|p| store.create_vertex(p, SYSTEM_RESOLUTION));
            let fins: Vec<_> = (0..4)
                .map(|i| {
                    let (a, b) = (corners[i], corners[(i + 1) % 4]);
                    let curve = geo.add_curve(Curve3::line(a, b - a).expect("distinct corners"));
                    let edge = store.create_edge_with_curve(
                        vertices[i],
                        vertices[(i + 1) % 4],
                        SYSTEM_RESOLUTION,
                        curve,
                        0.0,
                        (b - a).norm(),
                    );
                    (edge, FinSense::Forward)
                })
                .collect();
            store.create_loop(face, LoopType::Outer, &fins);
            face
        }

        /// [`quad_face`] made **tolerant**: the vertices sit on the true
        /// corners, but every edge's curve runs between corners displaced by
        /// `gap` along a per-edge direction — the state a healed body is left
        /// in, where the surviving vertex is the cluster centroid and each
        /// adjacent curve still passes through its own pre-merge endpoint.
        /// The displacement is what `Vertex::tolerance` records, so the
        /// vertices carry it.
        fn tolerant_quad_face(
            store: &mut TopologyStore,
            geo: &mut GeometryStore,
            surface: Surface3,
            corners: [Point3; 4],
            gap: f64,
        ) -> EntityId<Face> {
            let body = store.create_body(BodyType::Solid);
            let shell = store.create_shell(body, true, ShellOrientation::Outward);
            let face = store.create_face(shell, FaceSense::Positive);
            store.faces.get_mut(face).expect("just created").surface =
                Some(geo.add_surface(surface));
            let vertices = corners.map(|p| store.create_vertex(p, gap));
            // Each edge is pulled off its corners in a different in-plane
            // direction, so the two edges meeting at a corner disagree there
            // by O(gap) — exactly how two faces across a healed vertex used
            // to disagree.
            let drift = |i: usize| {
                let angle = FRAC_PI_2 * i as f64;
                Vector3::new(gap * angle.cos(), gap * angle.sin(), 0.0)
            };
            let fins: Vec<_> = (0..4)
                .map(|i| {
                    let j = (i + 1) % 4;
                    let (a, b) = (corners[i] + drift(i), corners[j] + drift(j));
                    let curve = geo.add_curve(Curve3::line(a, b - a).expect("distinct corners"));
                    let edge = store.create_edge_with_curve(
                        vertices[i],
                        vertices[j],
                        gap,
                        curve,
                        0.0,
                        (b - a).norm(),
                    );
                    (edge, FinSense::Forward)
                })
                .collect();
            store.create_loop(face, LoopType::Outer, &fins);
            face
        }

        /// A quarter-cylinder wall face of radius `r` and height `h` on the
        /// given surface, bounded the way a real body binds one: two circle
        /// arc edges for the rims and two straight seams. Unlike
        /// [`quad_face`]'s all-line boundary, this puts the *true* edge
        /// geometry on the curved sides, which is what the face's neighbours
        /// would share.
        fn quarter_wall_face(
            store: &mut TopologyStore,
            geo: &mut GeometryStore,
            surface: Surface3,
            r: f64,
            h: f64,
        ) -> EntityId<Face> {
            let body = store.create_body(BodyType::Solid);
            let shell = store.create_shell(body, true, ShellOrientation::Outward);
            let face = store.create_face(shell, FaceSense::Positive);
            store.faces.get_mut(face).expect("just created").surface =
                Some(geo.add_surface(surface));

            let corner = |k: usize, z: f64| {
                let angle = FRAC_PI_2 * k as f64;
                Point3::new(r * angle.cos(), r * angle.sin(), z)
            };
            let vertices = [corner(0, 0.0), corner(1, 0.0), corner(1, h), corner(0, h)]
                .map(|p| store.create_vertex(p, SYSTEM_RESOLUTION));
            // Rim arcs: the `Curve3::circle` parameter origin is
            // `plane_basis(+Z).0 = +X`, so the first quarter is `[0, π/2]`.
            let rim = |store: &mut TopologyStore, geo: &mut GeometryStore, z: f64, a, b| {
                let circle = Curve3::circle(Point3::new(0.0, 0.0, z), Vector3::z(), r)
                    .expect("valid rim circle");
                let curve = geo.add_curve(circle);
                store.create_edge_with_curve(a, b, SYSTEM_RESOLUTION, curve, 0.0, FRAC_PI_2)
            };
            let seam = |store: &mut TopologyStore, geo: &mut GeometryStore, k: usize, a, b| {
                let line = Curve3::line(corner(k, 0.0), Vector3::z()).expect("valid seam");
                let curve = geo.add_curve(line);
                store.create_edge_with_curve(a, b, SYSTEM_RESOLUTION, curve, 0.0, h)
            };
            let bottom = rim(store, geo, 0.0, vertices[0], vertices[1]);
            let top = rim(store, geo, h, vertices[3], vertices[2]);
            let seam_0 = seam(store, geo, 0, vertices[0], vertices[3]);
            let seam_1 = seam(store, geo, 1, vertices[1], vertices[2]);
            store.create_loop(
                face,
                LoopType::Outer,
                &[
                    (bottom, FinSense::Forward),
                    (seam_1, FinSense::Forward),
                    (top, FinSense::Reversed),
                    (seam_0, FinSense::Reversed),
                ],
            );
            face
        }

        /// Flat degree-1 patch over an arbitrary knot domain, spanning the
        /// unit square in the `z = 0` plane.
        fn flat_patch(domain: (f64, f64)) -> Surface3 {
            let grid = vec![
                vec![Point3::origin(), Point3::new(0.0, 1.0, 0.0)],
                vec![Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0)],
            ];
            Surface3::nurbs(
                NurbsSurface::bspline(grid, deg1_knots(2, domain), deg1_knots(2, domain))
                    .expect("2x2 bilinear grid"),
            )
        }

        fn unit_square_corners() -> [Point3; 4] {
            [
                Point3::origin(),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ]
        }

        /// A degree-1 patch is planar within each span, so its normal never
        /// turns and one segment per span is *exact*: two triangles, and the
        /// area is the true area with no discretization loss at all.
        #[test]
        fn flat_patch_grids_to_two_exact_triangles() {
            let mut store = TopologyStore::new();
            let mut geo = GeometryStore::new();
            let face = quad_face(
                &mut store,
                &mut geo,
                flat_patch((0.0, 1.0)),
                unit_square_corners(),
            );
            let mesh = tessellate_face(&store, &geo, face, &TessellationOptions::default())
                .expect("untrimmed degree-1 patch grids");
            assert_eq!(
                mesh.triangle_count(),
                2,
                "a planar patch needs no interior lattice"
            );
            assert!(
                (mesh.total_area() - 1.0).abs() < 1e-12,
                "planar patch area must be exact, got {}",
                mesh.total_area()
            );
        }

        /// §6/of-37i.3's normalization rule, at the tessellator: the same
        /// geometry over a wildly different knot domain must produce the same
        /// mesh. A lattice priced in *parameter* units (`angular_segments` on
        /// the domain length) would fail this outright — `[3, 13]` is ten
        /// times `[0, 1]` and would grid ten times finer for identical
        /// geometry.
        #[test]
        fn lattice_is_invariant_under_knot_domain_scaling() {
            let mesh_for = |domain: (f64, f64)| {
                let mut store = TopologyStore::new();
                let mut geo = GeometryStore::new();
                let face = quad_face(
                    &mut store,
                    &mut geo,
                    flat_patch(domain),
                    unit_square_corners(),
                );
                tessellate_face(&store, &geo, face, &TessellationOptions::default())
                    .expect("untrimmed patch grids")
            };
            let unit = mesh_for((0.0, 1.0));
            let scaled = mesh_for((3.0, 13.0));
            assert_eq!(
                unit.triangle_count(),
                scaled.triangle_count(),
                "knot domain must not change the lattice"
            );
            for (a, b) in unit.positions.iter().zip(&scaled.positions) {
                assert!((a - b).norm() < 1e-12, "vertex moved: {a:?} vs {b:?}");
            }
        }

        /// The exact rational quarter-cylinder of of-pb7.3 — the §9 gate's
        /// "NURBS patch of exact analytic form" — as a *face*, with the two
        /// rims carried by the circle edges a real body binds there.
        ///
        /// This is the of-dvj shape in miniature. The face's rim vertices
        /// must be the circle's own samples, at the same uniform angles the
        /// analytic cylinder and the planar cap take, and *not* the patch's
        /// own rational parameter — which runs faster near the ends and so
        /// lands elsewhere. Only then do the NURBS duplicate and the
        /// analytic surface it duplicates mesh alike, which is what lets
        /// adjacent faces weld.
        #[test]
        fn exact_quarter_cylinder_face_rims_follow_the_edge_circles() {
            let (r, h) = (2.0, 3.0);
            let w = FRAC_1_SQRT_2;
            // 90° arc: corner control point at the tangent intersection
            // (r, r), weight 1/√2, swept linearly in v.
            let arc = [
                Point3::new(r, 0.0, 0.0),
                Point3::new(r, r, 0.0),
                Point3::new(0.0, r, 0.0),
            ];
            let grid: Vec<Vec<Point3>> = arc
                .iter()
                .map(|p| vec![*p, Point3::new(p.x, p.y, h)])
                .collect();
            let weights = vec![vec![1.0, 1.0], vec![w, w], vec![1.0, 1.0]];
            let patch = NurbsSurface::new(
                grid,
                weights,
                KnotVector::new(2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0]).unwrap(),
                deg1_knots(2, (0.0, 1.0)),
            )
            .expect("rational quarter-cylinder patch");

            let mut store = TopologyStore::new();
            let mut geo = GeometryStore::new();
            let face = quarter_wall_face(&mut store, &mut geo, Surface3::nurbs(patch), r, h);
            let options = TessellationOptions::default();
            let mesh = tessellate_face(&store, &geo, face, &options).expect("patch tessellates");

            // Every sample the bottom circle edge takes must appear as a
            // mesh vertex, exactly. This is the welding contract: the cap on
            // the other side of that edge samples it identically.
            let segments = angular_segments(FRAC_PI_2, &options);
            for k in 0..=segments {
                let angle = FRAC_PI_2 * k as f64 / segments as f64;
                let rim = Point3::new(r * angle.cos(), r * angle.sin(), 0.0);
                assert!(
                    mesh.positions.iter().any(|p| (p - rim).norm() < 1e-12),
                    "rim sample at {angle} rad is missing from the face mesh — \
                     it would not weld to the neighbour across that edge"
                );
            }

            // The mesh inscribes the true wall, so it loses the chordal
            // deficit and nothing else. A regular `segments`-gon rim bounds
            // that loss from below; the interior lattice can only pull the
            // area back up toward exact, never past it.
            let exact = FRAC_PI_2 * r * h;
            let regular_gon =
                2.0 * r * (FRAC_PI_2 / (2.0 * segments as f64)).sin() * segments as f64 * h;
            let area = mesh.total_area();
            assert!(
                area < exact && area >= regular_gon * (1.0 - 1e-9),
                "quarter-cylinder wall area {area} must inscribe the exact \
                 {exact} and stay at or above the regular-gon chord area \
                 {regular_gon}"
            );
            assert_within(area, exact, 2e-3, "quarter-cylinder wall area");
        }

        /// A tolerant face's boundary must land on its **vertices**, not on
        /// its edge curves' endpoints (of-61f).
        ///
        /// On an exact body the two coincide, so nothing distinguished them.
        /// On a healed one they do not: the vertex is the merged cluster's
        /// centroid and each adjacent curve still runs to its own pre-merge
        /// point. Two faces meeting at that vertex arrive along *different*
        /// edges, so sampling each curve at its corner put them up to the
        /// closed gap apart — orders of magnitude above `tessellate_body`'s
        /// `bbox_diagonal * 1e-9` weld epsilon, which left a healed import
        /// meshing as an open shell.
        ///
        /// Both arms are checked, because both source their boundary from
        /// [`sample_loop`]: the planar ear-clip and the NURBS CDT. The CDT
        /// pass (of-37i.6) did not fix this on its own — it made a NURBS
        /// face's boundary come from the edge curves, which cured of-dvj (a
        /// grid sampling the *same* edge at different parameters), while this
        /// is a disagreement between *distinct* edges at a shared corner.
        #[test]
        fn a_tolerant_face_samples_its_vertices_not_its_curve_ends() {
            let corners = unit_square_corners();
            for gap in [1e-8, 1e-6, 1e-4] {
                for (arm, surface) in [
                    (
                        "planar",
                        Surface3::plane(Point3::origin(), Vector3::z()).expect("valid plane"),
                    ),
                    ("nurbs cdt", flat_patch((0.0, 1.0))),
                ] {
                    let mut store = TopologyStore::new();
                    let mut geo = GeometryStore::new();
                    let face = tolerant_quad_face(&mut store, &mut geo, surface, corners, gap);
                    let mesh = tessellate_face(&store, &geo, face, &TessellationOptions::default())
                        .expect("a tolerant face tessellates");
                    for corner in corners {
                        assert!(
                            mesh.positions.iter().any(|p| (p - corner).norm() == 0.0),
                            "{arm}, gap {gap:e}: corner {corner:?} is absent from the face \
                             mesh — it would not weld to the face across that vertex"
                        );
                    }
                }
            }
        }

        /// A *trimmed* NURBS face — its boundary cuts diagonally across the
        /// patch instead of running along the knot-domain border — used to
        /// be rejected outright, because a `u × v` grid cannot represent it
        /// (the same deferral a non-rectangular quadric trim still takes,
        /// of-q6u). The CDT pass has no such restriction: the boundary is
        /// whatever the loops say it is (of-37i.6).
        #[test]
        fn tessellates_a_trimmed_nurbs_face() {
            let mut store = TopologyStore::new();
            let mut geo = GeometryStore::new();
            // A triangular sub-region of the unit-square patch: the diagonal
            // from (1,0) to (0,1) leaves the domain border. The fourth corner
            // sits *on* the u = 0 edge, so the ring also carries a collinear
            // run — which the CDT has to place without minting a sliver.
            let face = quad_face(
                &mut store,
                &mut geo,
                flat_patch((0.0, 1.0)),
                [
                    Point3::origin(),
                    Point3::new(1.0, 0.0, 0.0),
                    Point3::new(0.0, 1.0, 0.0),
                    Point3::new(0.0, 0.5, 0.0),
                ],
            );
            let mesh = tessellate_face(&store, &geo, face, &TessellationOptions::default())
                .expect("a trimmed NURBS face tessellates through the CDT pass");
            assert!(
                (mesh.total_area() - 0.5).abs() < 1e-12,
                "the trimmed half of the unit patch must come out at area 0.5, got {}",
                mesh.total_area()
            );
        }

        /// A NURBS face with an **inner loop**: the grid arm rejected these
        /// outright (a hole cannot be gridded without bridging), and the CDT
        /// removes them by ring parity instead (of-37i.6).
        #[test]
        fn tessellates_a_nurbs_face_with_a_hole() {
            let mut store = TopologyStore::new();
            let mut geo = GeometryStore::new();
            let face = quad_face(
                &mut store,
                &mut geo,
                flat_patch((0.0, 1.0)),
                unit_square_corners(),
            );
            // A square hole [0.25, 0.75]², wound clockwise about the face's
            // outward normal as an inner loop is.
            let hole = [
                Point3::new(0.25, 0.25, 0.0),
                Point3::new(0.25, 0.75, 0.0),
                Point3::new(0.75, 0.75, 0.0),
                Point3::new(0.75, 0.25, 0.0),
            ];
            let vertices = hole.map(|p| store.create_vertex(p, SYSTEM_RESOLUTION));
            let fins: Vec<_> = (0..4)
                .map(|i| {
                    let (a, b) = (hole[i], hole[(i + 1) % 4]);
                    let curve = geo.add_curve(Curve3::line(a, b - a).expect("distinct corners"));
                    let edge = store.create_edge_with_curve(
                        vertices[i],
                        vertices[(i + 1) % 4],
                        SYSTEM_RESOLUTION,
                        curve,
                        0.0,
                        (b - a).norm(),
                    );
                    (edge, FinSense::Forward)
                })
                .collect();
            store.create_loop(face, LoopType::Inner, &fins);

            let mesh = tessellate_face(&store, &geo, face, &TessellationOptions::default())
                .expect("a NURBS face with a hole tessellates through the CDT pass");
            assert!(
                (mesh.total_area() - 0.75).abs() < 1e-12,
                "the unit patch less a 0.5 x 0.5 hole must come out at area 0.75, got {}",
                mesh.total_area()
            );
        }
    }
}
