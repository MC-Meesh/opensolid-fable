//! B-Rep tessellation MVP (`spec/07-tessellation.md`): convert bodies with
//! analytic face geometry into [`TriangleMesh`]es.
//!
//! Strategy, per face by surface kind:
//!
//! - **Planar faces**: the outer loop is sampled into a polygon (lines as
//!   single segments, circles/ellipses at the angular step) and
//!   ear-clip triangulated (correct for concave outlines).
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
//! - Planar faces are **ear-clip** triangulated (correct for concave outer
//!   loops), but faces with inner loops (holes) are still rejected with
//!   [`CoreError::NotImplemented`]. Full constrained Delaunay triangulation
//!   (hole bridging, as in [`crate::boolean`]) is a later pass.
//! - Cylinder/cone faces cover either their **full `u` period** (primitive and
//!   sweep walls) or a **partial arc that is a clean iso-parameter rectangle**
//!   (`[u_lo, u_hi] × [v_lo, v_hi]`, e.g. a quarter-cylinder notch from a
//!   boolean, of-2i3) — both recovered by projecting boundary samples onto the
//!   surface. A trim whose boundary is *not* such a rectangle (a slanted planar
//!   cut, or a face with inner loops) is *detected* and rejected with
//!   [`CoreError::NotImplemented`] instead of silently gridding it wrong
//!   (of-q6u); it arrives with the CDT pass
//!   ([`crate::boolean::BooleanOutput::tessellate`] already handles it).
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
use crate::project::SurfaceProject;
use crate::surface::{Surface3, SurfaceEval};
use crate::topology::{Body, Edge, Face, FaceSense, Fin, FinSense, Loop, TopologyStore};
use opensolid_core::error::{CoreError, CoreResult};
use opensolid_core::mesh::TriangleMesh;
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

/// Tessellate every face of `body` into one welded mesh.
///
/// For the closed solids produced by [`crate::primitives`] and
/// [`crate::sweep`], the result is a closed, consistently oriented
/// manifold (see [`TriangleMesh::is_closed_manifold`]).
///
/// # Errors
/// [`CoreError::InvalidArgument`] if `body` is stale, or any reached face
/// or edge lacks attached geometry; [`CoreError::NotImplemented`] for
/// planar faces with holes (see the module docs).
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
        // The grid path prices its lattice off an angular pitch about an
        // axis, which a freeform patch has none of; a curvature-derived
        // pitch is of-37i.6 (phase 4). Erroring here routes NURBS bodies to
        // the F-Rep fallback, which meshes them correctly today.
        Surface3::Nurbs(_) => Err(CoreError::NotImplemented {
            feature: "tessellating NURBS faces (of-37i.6: curvature-derived lattice pitch)",
        }),
    }
}

/// Ear-clip triangulate a planar face's outer loop polygon. Correct for
/// concave outlines, unlike the old first-vertex fan (of-6dw).
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
    if !face.inner_loops.is_empty() {
        return Err(CoreError::NotImplemented {
            feature: "tessellating planar faces with holes (needs constrained triangulation)",
        });
    }
    let loop_id = face
        .outer_loop
        .ok_or_else(|| invalid_face(face_id, "has no outer loop"))?;
    let polygon = sample_loop(store, geo, face_id, loop_id, options)?;
    if polygon.len() < 3 {
        return Err(invalid_face(
            face_id,
            "outer loop samples to fewer than 3 points",
        ));
    }

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
    let uv: Vec<(f64, f64)> = polygon
        .iter()
        .map(|p| {
            let d = p - origin;
            (d.dot(&e_u), d.dot(&e_v))
        })
        .collect();
    for [a, b, c] in crate::triangulate::ear_clip(&uv) {
        mesh.indices.push([base + a, base + b, base + c]);
    }
    Ok(())
}

/// Sample a loop's boundary as a closed polygon, in loop order, one open
/// run of points per fin (each fin's end point is supplied by the next).
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
        };
        for k in 0..segments {
            let t = t_from + (t_to - t_from) * k as f64 / segments as f64;
            points.push(curve.point(t));
        }
    }
    Ok(points)
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
/// `u` analysis — their `u` is arbitrary — but still bound the `v` range.
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
    let col_count = if wrap_u { n_u } else { n_u + 1 };
    let row_count = if wrap_v { n_v } else { n_v + 1 };

    // rows[j] holds one vertex index per u column, or exactly one index for
    // a collapsed singular row.
    let mut rows: Vec<Vec<usize>> = Vec::with_capacity(row_count);
    for j in 0..row_count {
        let v = if !wrap_v && j == n_v {
            v_hi // exact endpoint, no accumulation error
        } else {
            v_lo + (v_hi - v_lo) * j as f64 / n_v as f64
        };
        let singular = surface.is_singular(u_lo, v);
        let columns = if singular { 1 } else { col_count };
        let mut row = Vec::with_capacity(columns);
        for i in 0..columns {
            let u = if !wrap_u && i == n_u {
                u_hi // exact endpoint, no accumulation error
            } else {
                u_lo + (u_hi - u_lo) * i as f64 / n_u as f64
            };
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
    use crate::primitives;
    use std::f64::consts::{PI, TAU};

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
}
