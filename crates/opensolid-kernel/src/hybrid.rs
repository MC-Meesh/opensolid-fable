//! Hybrid boolean fast path: kernel-level booleans that never fail.
//!
//! [`boolean`] combines two bodies given in either representation:
//!
//! - **B-Rep × B-Rep** (both bodies in the same store pair): the exact
//!   analytic pipeline ([`opensolid_brep::boolean`]) runs first. The exact
//!   path wins only if its tessellation is closed, manifold, *and*
//!   geometrically faithful: the mesh's measured chordal deviation
//!   ([`BooleanOutput::tessellate_measured`]) must not exceed one F-Rep
//!   grid cell — the error the fallback itself would commit. On success
//!   the result keeps its exact topology ([`HybridPath::Brep`]) and the
//!   returned mesh is its tessellation.
//! - **Anything else** — an F-Rep operand, B-Rep operands living in
//!   different stores, or any exact-pipeline shortfall
//!   (coincident/tangent contacts, unsupported surfaces, classification
//!   degeneracies, a tessellation that comes out non-manifold or too
//!   coarse) — takes the F-Rep fallback: B-Rep operands are tessellated
//!   ([`tessellate_body`]) and wrapped as signed distance fields
//!   ([`MeshSdf`]), the operation becomes min/max CSG, and the combined
//!   field is dual-contoured back into a mesh. The field rides along in
//!   [`HybridPath::Frep`] for further composition or faceted B-Rep
//!   recovery ([`HybridBoolean::faceted_brep`] via [`sdf_to_brep`]).
//!
//! The fallback trades exactness for robustness: the result boundary
//! deviates from the true boolean by at most the operand tessellation's
//! chordal error plus the dual-contouring cell size, but it exists for any
//! pair of valid inputs — including the degenerate contacts the exact
//! pipeline rejects. This is the "booleans never fail" escape hatch of the
//! hybrid architecture.
//!
//! # Runtime result validation
//!
//! The exact pipeline can return `Ok` with *wrong* geometry (of-ipt.4:
//! removed volume off 12×; of-ipt.5: silent no-op) — failures the
//! mesh-quality gate alone cannot see, because the bad mesh is still
//! closed, manifold, and chord-faithful to the (wrong) analytic faces. So
//! a successful exact result additionally passes a validation gate
//! ([`ValidationOptions`], on by default) before it is kept: the result
//! body must pass the full topology checker ([`BooleanOutput::check`]),
//! and its enclosed volume ([`mass_properties`]) must agree with a coarse
//! F-Rep grid estimate of the same boolean. A result that fails either
//! test is discarded and the operation diverts to the F-Rep fallback,
//! recording why in [`HybridBoolean::diagnostic`]. Set
//! [`HybridOptions::validation`] to `None` to benchmark the pure B-Rep
//! path without the cross-check cost.

use crate::builder::Part;
use crate::convert::{MeshSdf, SdfToBrepOptions, sdf_to_brep};
use crate::massprops::mass_properties;
use crate::mesh::{MeshOptions, TriangleMesh, mesh_sdf_indexed};
use opensolid_brep::boolean::{
    intersect as brep_intersect, subtract as brep_subtract, unite as brep_unite,
};
use opensolid_brep::{
    Body, BooleanOutput, GeometryStore, TessellationOptions, TopologyStore, tessellate_body,
};
use opensolid_core::EntityId;
use opensolid_core::error::{CoreError, CoreResult};
use opensolid_core::tolerance::ToleranceContext;
use opensolid_core::types::{BoundingBox3, Point3, Vector3};
use opensolid_frep::primitives::Sdf;
use opensolid_frep::shape::Shape;

pub use opensolid_brep::BooleanOp;

/// Fewer cells than this across the sampling cube loses features; matches
/// the floor used by [`Part::mesh`].
const MIN_RESOLUTION: usize = 8;

/// A boolean operand in either of the kernel's representations.
pub enum HybridBody<'a> {
    /// An implicit body: the field plus conservative bounds containing its
    /// surface (dual contouring needs a sampling box).
    Frep { shape: Shape, bounds: BoundingBox3 },
    /// An exact analytic B-Rep solid: a store-backed body (e.g. built by
    /// [`opensolid_brep::primitives`]).
    Brep {
        store: &'a TopologyStore,
        geo: &'a GeometryStore,
        body: EntityId<Body>,
    },
}

impl std::fmt::Debug for HybridBody<'_> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            HybridBody::Frep { bounds, .. } => f
                .debug_struct("HybridBody::Frep")
                .field("bounds", bounds)
                .finish_non_exhaustive(),
            HybridBody::Brep { body, .. } => f
                .debug_struct("HybridBody::Brep")
                .field("body", body)
                .finish_non_exhaustive(),
        }
    }
}

impl<'a> HybridBody<'a> {
    /// Wrap an implicit body. `bounds` must be a non-empty finite box
    /// containing the body's whole surface.
    ///
    /// # Errors
    /// [`CoreError::InvalidArgument`] for an empty or non-finite `bounds`.
    pub fn frep(shape: Shape, bounds: BoundingBox3) -> CoreResult<Self> {
        check_bounds(&bounds)?;
        Ok(HybridBody::Frep { shape, bounds })
    }

    /// Wrap a store-backed B-Rep body. The body's validity is checked by
    /// the pipelines themselves ([`boolean`]).
    pub fn brep(store: &'a TopologyStore, geo: &'a GeometryStore, body: EntityId<Body>) -> Self {
        HybridBody::Brep { store, geo, body }
    }
}

/// A [`Part`] carries exactly the field-plus-bounds pair the F-Rep operand
/// needs.
impl From<Part> for HybridBody<'_> {
    fn from(part: Part) -> Self {
        HybridBody::Frep {
            bounds: part.bounds(),
            shape: part.into_shape(),
        }
    }
}

fn check_bounds(bounds: &BoundingBox3) -> CoreResult<()> {
    let finite = bounds
        .min
        .coords
        .iter()
        .chain(bounds.max.coords.iter())
        .all(|c| c.is_finite());
    if bounds.is_empty() || !finite {
        return Err(CoreError::InvalidArgument {
            argument: "bounds",
            reason: format!("must be a non-empty finite box, got {bounds:?}"),
        });
    }
    Ok(())
}

/// Options for [`boolean`].
#[derive(Debug, Clone, Copy)]
pub struct HybridOptions {
    /// Tolerances for the exact B-Rep pipeline attempt.
    pub tol: ToleranceContext,
    /// Grid resolution (cells across the sampling cube) for the F-Rep
    /// fallback's dual-contouring mesh. Also sets the exact path's
    /// mesh-quality bar: one grid cell of chordal deviation.
    pub resolution: usize,
    /// Runtime validation of a successful exact result (see the
    /// [module docs](self)). `None` disables the gate — the exact path
    /// then answers only to the mesh-quality bar, with no operand
    /// tessellation or grid-evaluation overhead (pure-B-Rep benchmarking).
    pub validation: Option<ValidationOptions>,
}

impl Default for HybridOptions {
    fn default() -> Self {
        Self {
            tol: ToleranceContext::default(),
            resolution: 64,
            validation: Some(ValidationOptions::default()),
        }
    }
}

/// Options for the exact-result validation gate (see the
/// [module docs](self)).
#[derive(Debug, Clone, Copy)]
pub struct ValidationOptions {
    /// Grid resolution (cells across the longest bounds axis) for the
    /// F-Rep volume estimate. Coarser than the fallback's meshing
    /// resolution: the estimate only has to expose gross volume errors,
    /// not reproduce the surface.
    pub resolution: usize,
    /// Maximum allowed relative divergence between the exact result's
    /// enclosed volume and the F-Rep estimate:
    /// `|v_brep − v_est| ≤ max_volume_divergence · max(v_brep, v_est)`.
    /// Must cover the estimate's own error (grid discretization plus
    /// operand tessellation), or correct exact results get discarded.
    pub max_volume_divergence: f64,
}

impl Default for ValidationOptions {
    fn default() -> Self {
        Self {
            resolution: 32,
            max_volume_divergence: 0.05,
        }
    }
}

/// Why the validation gate discarded a successful exact B-Rep result and
/// diverted to the F-Rep fallback. Carried on
/// [`HybridBoolean::diagnostic`].
#[derive(Debug, Clone, PartialEq)]
pub enum ValidationDiagnostic {
    /// The result body failed the full topology checker
    /// ([`BooleanOutput::check`]); `failures` is the number of check
    /// failures reported.
    CheckFailed { failures: usize },
    /// The result's enclosed volume disagreed with the F-Rep grid estimate
    /// of the same boolean by more than
    /// [`ValidationOptions::max_volume_divergence`].
    VolumeDivergence {
        /// Volume enclosed by the exact result's tessellation.
        brep_volume: f64,
        /// Coarse F-Rep grid estimate of the true boolean's volume.
        estimated_volume: f64,
    },
    /// The result mesh was closed and manifold but enclosed no measurable
    /// volume (e.g. a zero-thickness pillow).
    UnmeasurableVolume,
}

/// Which pipeline produced the result, with that pipeline's
/// representation-specific payload.
pub enum HybridPath {
    /// Exact B-Rep pipeline result: validated topology and analytic faces
    /// (boxed: the topology stores dwarf the F-Rep variant).
    Brep(Box<BooleanOutput>),
    /// F-Rep fallback: the combined field and the tight (pre-padding)
    /// bounds it was meshed within. Empty `bounds` means the result is
    /// provably empty (disjoint intersection) and the mesh is empty too.
    Frep { shape: Shape, bounds: BoundingBox3 },
}

impl std::fmt::Debug for HybridPath {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            HybridPath::Brep(out) => f.debug_tuple("HybridPath::Brep").field(out).finish(),
            HybridPath::Frep { bounds, .. } => f
                .debug_struct("HybridPath::Frep")
                .field("bounds", bounds)
                .finish_non_exhaustive(),
        }
    }
}

/// A hybrid boolean result: always a watertight mesh, plus whatever richer
/// representation the winning path produced.
#[derive(Debug)]
pub struct HybridBoolean {
    /// Triangle mesh of the result boundary (empty for an empty result).
    pub mesh: TriangleMesh,
    /// The pipeline that produced it.
    pub path: HybridPath,
    /// Set when the exact pipeline returned `Ok` but the validation gate
    /// discarded its result as wrong ([`ValidationDiagnostic`]); the
    /// F-Rep fallback result held here replaced it. `None` on the exact
    /// path, and on fallbacks taken for any other reason (mixed
    /// representations, exact-pipeline error, mesh-quality shortfall).
    pub diagnostic: Option<ValidationDiagnostic>,
}

impl HybridBoolean {
    /// Recover a faceted B-Rep body from an F-Rep fallback result via
    /// [`sdf_to_brep`], writing into the caller's stores.
    ///
    /// # Errors
    /// [`CoreError::InvalidArgument`] on an exact-path result — that
    /// already carries validated topology in [`HybridPath::Brep`], and a
    /// silent faceted downgrade would discard it — or on an empty result;
    /// plus any [`sdf_to_brep`] conversion error.
    pub fn faceted_brep(
        &self,
        store: &mut TopologyStore,
        geo: &mut GeometryStore,
        max_depth: u32,
    ) -> CoreResult<EntityId<Body>> {
        match &self.path {
            HybridPath::Frep { shape, bounds } => {
                if bounds.is_empty() {
                    return Err(CoreError::Degenerate {
                        context: "HybridBoolean::faceted_brep",
                        reason: "the result is empty; there is no boundary to recover".into(),
                    });
                }
                let cells = 1usize << max_depth.min(16);
                let sampling = sampling_cube(bounds, cells);
                sdf_to_brep(
                    shape,
                    store,
                    geo,
                    &SdfToBrepOptions::new(sampling, max_depth),
                )
            }
            HybridPath::Brep(_) => Err(CoreError::InvalidArgument {
                argument: "self",
                reason: "exact result already carries its topology (HybridPath::Brep); \
                         faceted recovery applies to the F-Rep fallback path"
                    .into(),
            }),
        }
    }
}

/// Material of A or B. See [`boolean`].
pub fn unite(a: &HybridBody, b: &HybridBody, opts: &HybridOptions) -> CoreResult<HybridBoolean> {
    boolean(BooleanOp::Unite, a, b, opts)
}

/// Material of A and not B. See [`boolean`].
pub fn subtract(a: &HybridBody, b: &HybridBody, opts: &HybridOptions) -> CoreResult<HybridBoolean> {
    boolean(BooleanOp::Subtract, a, b, opts)
}

/// Material of A and B. See [`boolean`].
pub fn intersect(
    a: &HybridBody,
    b: &HybridBody,
    opts: &HybridOptions,
) -> CoreResult<HybridBoolean> {
    boolean(BooleanOp::Intersect, a, b, opts)
}

/// Combine two bodies of either representation. Exact B-Rep pipeline when
/// both operands are B-Rep in the same store pair and it succeeds end to
/// end — including a closed, manifold tessellation within the F-Rep
/// grid-cell deviation bound; F-Rep CSG fallback otherwise. See the
/// [module docs](self).
///
/// # Errors
/// [`CoreError::InvalidArgument`] for a resolution below the meshing floor
/// or an operand with empty/non-finite bounds. Operand conversion errors
/// (a B-Rep body that fails to tessellate) also propagate. Exact-pipeline
/// failures do **not** propagate — they divert to the fallback.
pub fn boolean(
    op: BooleanOp,
    a: &HybridBody,
    b: &HybridBody,
    opts: &HybridOptions,
) -> CoreResult<HybridBoolean> {
    check_resolution("resolution", opts.resolution)?;
    if let Some(v) = &opts.validation {
        check_resolution("validation.resolution", v.resolution)?;
    }

    let mut diagnostic = None;
    if let (
        HybridBody::Brep {
            store: sa,
            geo: ga,
            body: ba,
        },
        HybridBody::Brep {
            store: sb,
            geo: gb,
            body: bb,
        },
    ) = (a, b)
        && std::ptr::eq(*sa, *sb)
        && std::ptr::eq(*ga, *gb)
    {
        let exact = match op {
            BooleanOp::Unite => brep_unite(sa, ga, *ba, *bb, &opts.tol),
            BooleanOp::Subtract => brep_subtract(sa, ga, *ba, *bb, &opts.tol),
            BooleanOp::Intersect => brep_intersect(sa, ga, *ba, *bb, &opts.tol),
        };
        // Any shortfall in the exact pipeline — an error, a tessellation
        // that comes out non-manifold, or one whose chords stray from the
        // analytic surfaces by more than an F-Rep grid cell (the fallback
        // would then be *more* accurate, not less) — diverts to the F-Rep
        // fallback below.
        if let Ok(out) = exact
            && let Ok((mesh, deviation)) = out.tessellate_measured()
            && mesh.is_closed_manifold()
            && let Some(bounds) = mesh.bounding_box()
            && deviation <= cell_size(&bounds, opts.resolution)
        {
            match opts
                .validation
                .as_ref()
                .and_then(|v| validate_exact(&out, &mesh, op, a, b, v))
            {
                None => {
                    return Ok(HybridBoolean {
                        mesh,
                        path: HybridPath::Brep(Box::new(out)),
                        diagnostic: None,
                    });
                }
                // Validation caught a silently-wrong exact result: discard
                // it and let the fallback rebuild from the operands.
                Some(d) => diagnostic = Some(d),
            }
        }
    }

    let mut result = frep_fallback(op, a, b, opts)?;
    result.diagnostic = diagnostic;
    Ok(result)
}

fn check_resolution(argument: &'static str, resolution: usize) -> CoreResult<()> {
    if resolution < MIN_RESOLUTION {
        return Err(CoreError::InvalidArgument {
            argument,
            reason: format!(
                "must be at least {MIN_RESOLUTION} grid cells per axis, got {resolution}"
            ),
        });
    }
    Ok(())
}

/// The validation gate: decide whether a successful exact result that
/// already passed the mesh-quality bar is *geometrically* trustworthy.
/// `None` keeps the exact result; `Some` discards it with the reason.
///
/// Operand-conversion failures here (a B-Rep operand that won't
/// tessellate) keep the exact result rather than discarding it: the
/// fallback would fail on the same conversion, so an unverifiable exact
/// result is strictly better than a guaranteed error.
fn validate_exact(
    out: &BooleanOutput,
    mesh: &TriangleMesh,
    op: BooleanOp,
    a: &HybridBody,
    b: &HybridBody,
    v: &ValidationOptions,
) -> Option<ValidationDiagnostic> {
    let failures = out.check();
    if !failures.is_empty() {
        return Some(ValidationDiagnostic::CheckFailed {
            failures: failures.len(),
        });
    }
    let brep_volume = match mass_properties(mesh) {
        Ok(mp) => mp.volume,
        // The mesh is already known closed and manifold, so the only
        // failure left is an enclosed volume of zero.
        Err(_) => return Some(ValidationDiagnostic::UnmeasurableVolume),
    };
    let Ok((shape, bounds)) = combined_field(op, a, b) else {
        return None;
    };
    let estimated_volume = grid_volume(&shape, &bounds, v.resolution);
    let scale = brep_volume.max(estimated_volume);
    if (brep_volume - estimated_volume).abs() > v.max_volume_divergence * scale {
        return Some(ValidationDiagnostic::VolumeDivergence {
            brep_volume,
            estimated_volume,
        });
    }
    None
}

/// Coarse volume of the region where `shape` is negative: field samples at
/// the centers of cubic cells tiling `bounds` (padded by one cell so
/// boundary-straddling cells are covered), each converted to an occupancy
/// fraction `clamp(0.5 − d/h, 0, 1)`. For a locally planar boundary the
/// occupancy model is exact up to interface orientation, so the estimate's
/// error stays a small fraction of (surface area × cell size) — far
/// tighter than binary inside/outside counting at the same resolution.
fn grid_volume(shape: &Shape, bounds: &BoundingBox3, resolution: usize) -> f64 {
    if bounds.is_empty() {
        return 0.0;
    }
    let e = bounds.extents();
    let h = e.x.max(e.y).max(e.z) / resolution as f64;
    if h <= 0.0 || !h.is_finite() {
        return 0.0;
    }
    let cells = |extent: f64| (extent / h).ceil() as usize + 2;
    let (nx, ny, nz) = (cells(e.x), cells(e.y), cells(e.z));
    let origin = bounds.min - Vector3::repeat(h);
    let cell_volume = h * h * h;
    let mut volume = 0.0;
    for i in 0..nx {
        for j in 0..ny {
            for k in 0..nz {
                let p = Point3::new(
                    origin.x + (i as f64 + 0.5) * h,
                    origin.y + (j as f64 + 0.5) * h,
                    origin.z + (k as f64 + 0.5) * h,
                );
                let occupancy = (0.5 - shape.eval(&p) / h).clamp(0.0, 1.0);
                volume += occupancy * cell_volume;
            }
        }
    }
    volume
}

/// The robustness path: every operand becomes a signed distance field, the
/// operation is min/max CSG, and the result is re-meshed.
fn frep_fallback(
    op: BooleanOp,
    a: &HybridBody,
    b: &HybridBody,
    opts: &HybridOptions,
) -> CoreResult<HybridBoolean> {
    let (shape, bounds) = combined_field(op, a, b)?;
    if bounds.is_empty() {
        // Disjoint intersection: provably empty result.
        return Ok(HybridBoolean {
            mesh: TriangleMesh::from_triangles(&[]),
            path: HybridPath::Frep { shape, bounds },
            diagnostic: None,
        });
    }
    let mesh = mesh_sdf_indexed(
        &shape,
        &MeshOptions {
            bounds: sampling_cube(&bounds, opts.resolution),
            resolution: opts.resolution,
        },
    );
    Ok(HybridBoolean {
        mesh,
        path: HybridPath::Frep { shape, bounds },
        diagnostic: None,
    })
}

/// Both operands as fields, combined per `op`: the fallback's CSG field
/// and tight (pre-padding) bounds of the result.
fn combined_field(
    op: BooleanOp,
    a: &HybridBody,
    b: &HybridBody,
) -> CoreResult<(Shape, BoundingBox3)> {
    let (fa, ba) = field_of(a)?;
    let (fb, bb) = field_of(b)?;
    Ok(match op {
        BooleanOp::Unite => (fa.union(fb), ba.union(&bb)),
        // A - B and A ∩ B are both subsets of A; B only carves.
        BooleanOp::Subtract => (fa.subtract(fb), ba),
        BooleanOp::Intersect => (fa.intersect(fb), ba.intersection(&bb)),
    })
}

/// An operand as (field, surface bounds).
fn field_of(body: &HybridBody) -> CoreResult<(Shape, BoundingBox3)> {
    match body {
        HybridBody::Frep { shape, bounds } => {
            check_bounds(bounds)?;
            Ok((shape.clone(), *bounds))
        }
        HybridBody::Brep { store, geo, body } => {
            let mesh = tessellate_body(store, geo, *body, &TessellationOptions::default())?;
            let bounds = mesh.bounding_box().ok_or_else(|| CoreError::Degenerate {
                context: "hybrid::boolean",
                reason: "B-Rep operand tessellated to an empty mesh".into(),
            })?;
            Ok((Shape::new(MeshSdf::new(&mesh)?), bounds))
        }
    }
}

/// Edge length of one dual-contouring grid cell for a body of the given
/// bounds at the given resolution: the geometric error the F-Rep fallback
/// itself commits, and hence the exact path's mesh-quality bar.
fn cell_size(bounds: &BoundingBox3, resolution: usize) -> f64 {
    let e = bounds.extents();
    e.x.max(e.y).max(e.z) / resolution as f64
}

/// Cubic sampling box around `bounds` with a margin of a few cells: dual
/// contouring stitches unreliably on strongly anisotropic cells (same
/// rationale as [`Part::mesh`]), and the surface must lie strictly inside
/// the box.
fn sampling_cube(bounds: &BoundingBox3, resolution: usize) -> BoundingBox3 {
    let e = bounds.extents();
    let extent = e.x.max(e.y).max(e.z);
    let margin = extent * 4.0 / resolution as f64;
    let half = Vector3::repeat(extent / 2.0 + margin);
    let center = bounds.center();
    BoundingBox3::new(center - half, center + half)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::builder::shape;
    use crate::massprops::mass_properties;
    use opensolid_brep::{primitives, translate_body};
    use std::f64::consts::PI;

    fn opts() -> HybridOptions {
        HybridOptions::default()
    }

    fn volume(mesh: &TriangleMesh) -> f64 {
        mass_properties(mesh).expect("closed manifold mesh").volume
    }

    fn assert_volume(mesh: &TriangleMesh, exact: f64, context: &str) {
        let got = volume(mesh);
        assert!(
            (got - exact).abs() / exact < 0.03,
            "{context}: volume {got} not within 3% of analytic {exact}"
        );
    }

    /// A block moved so it covers `[0, x] × [0, y] × [0, z]` — primitives
    /// are origin-centered, several tests want a corner-based one.
    fn corner_block(
        store: &mut TopologyStore,
        geo: &mut GeometryStore,
        x: f64,
        y: f64,
        z: f64,
    ) -> EntityId<Body> {
        let body = primitives::block(store, geo, x, y, z).unwrap();
        translate_body(store, geo, body, Vector3::new(x / 2.0, y / 2.0, z / 2.0)).unwrap();
        body
    }

    #[test]
    fn planar_subtract_takes_exact_path() {
        // All-planar inputs tessellate chord-exactly (zero deviation), so
        // the exact pipeline passes the quality gate and keeps its
        // topology.
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let block = primitives::block(&mut store, &mut geo, 4.0, 4.0, 2.0).unwrap();
        let tool = primitives::block(&mut store, &mut geo, 2.0, 2.0, 4.0).unwrap();
        let out = boolean(
            BooleanOp::Subtract,
            &HybridBody::brep(&store, &geo, block),
            &HybridBody::brep(&store, &geo, tool),
            &opts(),
        )
        .unwrap();
        assert!(
            matches!(out.path, HybridPath::Brep(_)),
            "transversal planar B-Rep inputs must take the exact path"
        );
        assert!(
            out.diagnostic.is_none(),
            "a kept exact result carries no validation diagnostic"
        );
        assert!(out.mesh.is_closed_manifold());
        assert_volume(
            &out.mesh,
            4.0 * 4.0 * 2.0 - 2.0 * 2.0 * 2.0,
            "block minus through-block",
        );
    }

    #[test]
    fn block_minus_cylinder_is_closed_and_accurate() {
        // The trimmed full-wrap cylinder band now refines its ear-clip
        // chords onto the true surface (of-ipt.4), so the exact pipeline
        // passes the quality gate — no F-Rep diversion — and the mesh must
        // be watertight and volume-accurate.
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let block = primitives::block(&mut store, &mut geo, 4.0, 4.0, 2.0).unwrap();
        let tool = primitives::cylinder(&mut store, &mut geo, 1.0, 4.0).unwrap();
        let out = boolean(
            BooleanOp::Subtract,
            &HybridBody::brep(&store, &geo, block),
            &HybridBody::brep(&store, &geo, tool),
            &opts(),
        )
        .unwrap();
        assert!(
            matches!(out.path, HybridPath::Brep(_)),
            "accurate exact tessellation must keep the B-Rep fast path"
        );
        assert!(out.mesh.is_closed_manifold());
        assert_volume(
            &out.mesh,
            4.0 * 4.0 * 2.0 - PI * 2.0,
            "block minus cylinder",
        );
    }

    #[test]
    fn sphere_field_minus_brep_block_takes_frep_path() {
        // Mixed representations: an analytic SDF sphere minus a B-Rep
        // block covering the (+,+,+) octant.
        let ball: HybridBody = shape::sphere(1.0).unwrap().into();
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let block = corner_block(&mut store, &mut geo, 2.0, 2.0, 2.0);
        let out = boolean(
            BooleanOp::Subtract,
            &ball,
            &HybridBody::brep(&store, &geo, block),
            &opts(),
        )
        .unwrap();
        assert!(
            matches!(out.path, HybridPath::Frep { .. }),
            "a mixed-representation boolean must take the F-Rep path"
        );
        assert!(out.mesh.is_closed_manifold());
        assert_volume(
            &out.mesh,
            (7.0 / 8.0) * (4.0 / 3.0) * PI,
            "sphere minus octant block",
        );
    }

    #[test]
    fn coincident_faces_fall_back_to_frep() {
        // B overlaps A with four coplanar side faces: the exact pipeline
        // rejects coincident contacts, so the hybrid path must rescue it.
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let a = primitives::block(&mut store, &mut geo, 1.0, 1.0, 1.0).unwrap();
        let b = primitives::block(&mut store, &mut geo, 1.2, 1.0, 1.0).unwrap();
        translate_body(&mut store, &mut geo, b, Vector3::new(0.35, 0.0, 0.0)).unwrap();
        assert!(
            brep_unite(&store, &geo, a, b, &opts().tol).is_err(),
            "precondition: the exact pipeline refuses coincident faces"
        );
        let out = boolean(
            BooleanOp::Unite,
            &HybridBody::brep(&store, &geo, a),
            &HybridBody::brep(&store, &geo, b),
            &opts(),
        )
        .unwrap();
        assert!(matches!(out.path, HybridPath::Frep { .. }));
        assert!(out.mesh.is_closed_manifold());
        assert_volume(&out.mesh, 1.45, "union with coincident faces");
    }

    #[test]
    fn brep_operands_in_separate_stores_fall_back() {
        // Valid B-Rep operands that don't share a store pair can't run the
        // exact pipeline; the fallback must still combine them.
        let mut store_a = TopologyStore::new();
        let mut geo_a = GeometryStore::new();
        let a = primitives::block(&mut store_a, &mut geo_a, 2.0, 2.0, 2.0).unwrap();
        let mut store_b = TopologyStore::new();
        let mut geo_b = GeometryStore::new();
        let b = primitives::block(&mut store_b, &mut geo_b, 2.0, 2.0, 2.0).unwrap();
        translate_body(&mut store_b, &mut geo_b, b, Vector3::new(1.0, 1.0, 1.0)).unwrap();
        let out = boolean(
            BooleanOp::Unite,
            &HybridBody::brep(&store_a, &geo_a, a),
            &HybridBody::brep(&store_b, &geo_b, b),
            &opts(),
        )
        .unwrap();
        assert!(matches!(out.path, HybridPath::Frep { .. }));
        assert!(out.mesh.is_closed_manifold());
        assert_volume(&out.mesh, 8.0 + 8.0 - 1.0, "cross-store union");
    }

    #[test]
    fn frep_result_recovers_faceted_brep() {
        let a: HybridBody = shape::box3(2.0, 2.0, 2.0).unwrap().into();
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let b = corner_block(&mut store, &mut geo, 2.0, 2.0, 2.0);
        let out = boolean(
            BooleanOp::Subtract,
            &a,
            &HybridBody::brep(&store, &geo, b),
            &opts(),
        )
        .unwrap();
        assert!(matches!(out.path, HybridPath::Frep { .. }));

        let mut out_store = TopologyStore::new();
        let mut out_geo = GeometryStore::new();
        let body = out.faceted_brep(&mut out_store, &mut out_geo, 5).unwrap();
        assert!(
            out_store.check(body).is_empty(),
            "recovered faceted body must pass the checker"
        );
    }

    #[test]
    fn exact_result_refuses_faceted_downgrade() {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let a = primitives::block(&mut store, &mut geo, 2.0, 2.0, 2.0).unwrap();
        let b = primitives::block(&mut store, &mut geo, 2.0, 2.0, 2.0).unwrap();
        translate_body(&mut store, &mut geo, b, Vector3::new(1.0, 1.0, 1.0)).unwrap();
        let out = boolean(
            BooleanOp::Unite,
            &HybridBody::brep(&store, &geo, a),
            &HybridBody::brep(&store, &geo, b),
            &opts(),
        )
        .unwrap();
        assert!(matches!(out.path, HybridPath::Brep(_)));
        let mut out_store = TopologyStore::new();
        let mut out_geo = GeometryStore::new();
        assert!(out.faceted_brep(&mut out_store, &mut out_geo, 5).is_err());
    }

    #[test]
    fn disjoint_intersection_is_empty() {
        let ball: HybridBody = shape::sphere(1.0).unwrap().into();
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let far = primitives::block(&mut store, &mut geo, 1.0, 1.0, 1.0).unwrap();
        translate_body(&mut store, &mut geo, far, Vector3::new(5.5, 5.5, 5.5)).unwrap();
        let out = boolean(
            BooleanOp::Intersect,
            &ball,
            &HybridBody::brep(&store, &geo, far),
            &opts(),
        )
        .unwrap();
        assert!(out.mesh.is_empty());
        match out.path {
            HybridPath::Frep { bounds, .. } => assert!(bounds.is_empty()),
            HybridPath::Brep(_) => panic!("mixed operands cannot take the exact path"),
        }
    }

    #[test]
    fn union_of_disjoint_mixed_bodies_keeps_both() {
        let ball: HybridBody = shape::sphere(1.0).unwrap().into();
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let far = primitives::block(&mut store, &mut geo, 1.0, 1.0, 1.0).unwrap();
        translate_body(&mut store, &mut geo, far, Vector3::new(4.5, 4.5, 4.5)).unwrap();
        let out = boolean(
            BooleanOp::Unite,
            &ball,
            &HybridBody::brep(&store, &geo, far),
            &opts(),
        )
        .unwrap();
        assert!(out.mesh.is_closed_manifold());
        assert_volume(&out.mesh, (4.0 / 3.0) * PI + 1.0, "disjoint mixed union");
    }

    #[test]
    fn coarse_resolution_is_rejected() {
        let ball: HybridBody = shape::sphere(1.0).unwrap().into();
        let other: HybridBody = shape::sphere(1.0).unwrap().into();
        let bad = HybridOptions {
            resolution: 4,
            ..Default::default()
        };
        assert!(boolean(BooleanOp::Unite, &ball, &other, &bad).is_err());
    }

    #[test]
    fn grid_volume_estimates_known_volumes() {
        let ball = shape::sphere(1.0).unwrap();
        let est = grid_volume(ball.shape(), &ball.bounds(), 32);
        let exact = (4.0 / 3.0) * PI;
        assert!(
            (est - exact).abs() / exact < 0.02,
            "sphere estimate {est} not within 2% of {exact}"
        );

        let block = shape::box3(2.0, 3.0, 1.0).unwrap();
        let est = grid_volume(block.shape(), &block.bounds(), 32);
        assert!(
            (est - 6.0).abs() / 6.0 < 0.02,
            "box estimate {est} not within 2% of 6"
        );
    }

    #[test]
    fn grid_volume_of_empty_bounds_is_zero() {
        let ball = shape::sphere(1.0).unwrap();
        assert_eq!(grid_volume(ball.shape(), &BoundingBox3::EMPTY, 32), 0.0);
    }

    #[test]
    fn volume_gate_catches_ipt4_silent_wrongness() {
        // of-ipt.4: the exact subtract on the canonical through-hole
        // config returns Ok with a closed-manifold mesh whose geometry is
        // wrong (bottom-face hole never cut; removed volume off ~12×).
        // The volume cross-check must flag it independently of the
        // chordal-deviation gate: the F-Rep estimate lands near the true
        // volume, far from the mesh's.
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let block = primitives::block(&mut store, &mut geo, 4.0, 4.0, 2.0).unwrap();
        let tool = primitives::cylinder(&mut store, &mut geo, 1.0, 4.0).unwrap();
        let out = brep_subtract(&store, &geo, block, tool, &opts().tol).unwrap();
        let (mesh, _) = out.tessellate_measured().unwrap();
        assert!(
            mesh.is_closed_manifold(),
            "of-ipt.4 mesh is combinatorially fine"
        );
        let brep_volume = volume(&mesh);

        let a = HybridBody::brep(&store, &geo, block);
        let b = HybridBody::brep(&store, &geo, tool);
        let (shape, bounds) = combined_field(BooleanOp::Subtract, &a, &b).unwrap();
        let estimated = grid_volume(&shape, &bounds, ValidationOptions::default().resolution);

        let exact = 4.0 * 4.0 * 2.0 - 2.0 * PI;
        assert!(
            (estimated - exact).abs() / exact < 0.05,
            "estimate {estimated} should be near the true volume {exact}"
        );
        let divergence = (brep_volume - estimated).abs() / brep_volume.max(estimated);
        assert!(
            divergence > ValidationOptions::default().max_volume_divergence,
            "divergence {divergence} must exceed the default gate \
             (brep {brep_volume} vs estimate {estimated})"
        );
    }

    #[test]
    fn discarded_exact_result_diverts_to_fallback_with_diagnostic() {
        // Wiring: with an impossibly strict divergence tolerance even a
        // correct exact result is discarded — the fallback must engage,
        // record the diagnostic, and still produce the right volume.
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let block = primitives::block(&mut store, &mut geo, 4.0, 4.0, 2.0).unwrap();
        let tool = primitives::block(&mut store, &mut geo, 2.0, 2.0, 4.0).unwrap();
        let strict = HybridOptions {
            validation: Some(ValidationOptions {
                max_volume_divergence: 0.0,
                ..Default::default()
            }),
            ..Default::default()
        };
        let out = boolean(
            BooleanOp::Subtract,
            &HybridBody::brep(&store, &geo, block),
            &HybridBody::brep(&store, &geo, tool),
            &strict,
        )
        .unwrap();
        assert!(
            matches!(out.path, HybridPath::Frep { .. }),
            "a discarded exact result must divert to the F-Rep fallback"
        );
        assert!(
            matches!(
                out.diagnostic,
                Some(ValidationDiagnostic::VolumeDivergence { .. })
            ),
            "the discard reason must be recorded, got {:?}",
            out.diagnostic
        );
        assert!(out.mesh.is_closed_manifold());
        assert_volume(
            &out.mesh,
            4.0 * 4.0 * 2.0 - 2.0 * 2.0 * 2.0,
            "fallback after validation discard",
        );
    }

    #[test]
    fn disabled_validation_keeps_exact_path() {
        // Pure-B-Rep benchmarking mode: no check(), no cross-check, the
        // mesh-quality bar alone decides.
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let block = primitives::block(&mut store, &mut geo, 4.0, 4.0, 2.0).unwrap();
        let tool = primitives::block(&mut store, &mut geo, 2.0, 2.0, 4.0).unwrap();
        let bench = HybridOptions {
            validation: None,
            ..Default::default()
        };
        let out = boolean(
            BooleanOp::Subtract,
            &HybridBody::brep(&store, &geo, block),
            &HybridBody::brep(&store, &geo, tool),
            &bench,
        )
        .unwrap();
        assert!(matches!(out.path, HybridPath::Brep(_)));
        assert!(out.diagnostic.is_none());
    }

    #[test]
    fn coarse_validation_resolution_is_rejected() {
        let ball: HybridBody = shape::sphere(1.0).unwrap().into();
        let other: HybridBody = shape::sphere(1.0).unwrap().into();
        let bad = HybridOptions {
            validation: Some(ValidationOptions {
                resolution: 4,
                ..Default::default()
            }),
            ..Default::default()
        };
        assert!(boolean(BooleanOp::Unite, &ball, &other, &bad).is_err());
    }

    #[test]
    fn frep_constructor_validates_bounds() {
        let part = shape::sphere(1.0).unwrap();
        assert!(HybridBody::frep(part.shape().clone(), BoundingBox3::EMPTY).is_err());
        let ok = HybridBody::frep(part.shape().clone(), part.bounds()).unwrap();
        assert!(matches!(ok, HybridBody::Frep { .. }));
    }
}
