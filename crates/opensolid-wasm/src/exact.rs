//! Exact B-Rep companion representation for playground shapes (of-4eh.17).
//!
//! When the exact-boolean toggle is on, shapes that stay within the
//! kernel's exact coverage carry a second representation next to their
//! SDF: either a *spec* (a rigid-transformed, uniformly-scaled analytic
//! primitive, cheap to rebuild into any store) or a *boolean result* (an
//! owned [`BooleanOutput`] plus its validated tessellation). Booleans run
//! through [`hybrid::boolean`] — exact-first with the full gate ladder,
//! F-Rep fallback otherwise — so a shape only ever carries an exact mesh
//! that passed manifoldness, deviation, and volume validation.
//!
//! Ownership model (the arena-lifetime answer for JS): [`HybridBody::Brep`]
//! borrows its stores, so nothing borrowed crosses the wasm boundary.
//! Instead each boolean result *owns* its `TopologyStore`/`GeometryStore`
//! (they arrive owned inside [`BooleanOutput`]), shapes share results via
//! `Rc`, and borrowed `HybridBody`s are constructed transiently inside a
//! single call. Chained booleans append the primitive operand into the
//! result's own store (arenas are append-only, so sibling shapes holding
//! the same `Rc` keep valid [`EntityId`] handles) — hence the `RefCell`.
//! WASM is single-threaded; `Rc`/`RefCell`/the mode flag need no `Send`.

use std::cell::RefCell;
use std::rc::Rc;
use std::sync::atomic::{AtomicBool, Ordering};

use opensolid_core::EntityId;
use opensolid_core::error::CoreResult;
use opensolid_core::mesh::TriangleMesh;
use opensolid_core::types::{Transform3, Vector3};
use opensolid_kernel::brep::topology::Body;
use opensolid_kernel::brep::transform::transform_body;
use opensolid_kernel::brep::{BooleanOp, BooleanOutput, GeometryStore, TopologyStore, primitives};
use opensolid_kernel::hybrid::{self, HybridBody, HybridOptions, HybridPath};

/// Global exact-boolean mode, flipped by the playground toggle. Read at
/// boolean and mesh time so flipping it re-routes without rebuilding
/// shapes (the playground re-runs the script on toggle anyway).
static EXACT_MODE: AtomicBool = AtomicBool::new(false);

/// Enable or disable the exact B-Rep boolean path.
pub fn set_exact_enabled(enabled: bool) {
    EXACT_MODE.store(enabled, Ordering::Relaxed);
}

/// Whether the exact B-Rep boolean path is enabled.
pub fn exact_enabled() -> bool {
    EXACT_MODE.load(Ordering::Relaxed)
}

/// An analytic primitive with exact B-Rep support, in the playground's
/// axis conventions (cylinder/torus about **Y**; the kernel builds them
/// about +Z, reconciled by a pre-rotation at materialization).
#[derive(Debug, Clone, Copy)]
pub enum ExactPrim {
    /// Sphere of `radius` centered at the origin.
    Sphere { radius: f64 },
    /// Axis-aligned box with half-extents, centered at the origin.
    Block { hx: f64, hy: f64, hz: f64 },
    /// Cylinder along Y: radius in the xz plane, `y ∈ ±half_height`.
    Cylinder { radius: f64, half_height: f64 },
    /// Torus with its ring in the xz plane, centered at the origin.
    Torus { major: f64, minor: f64 },
}

/// A primitive plus the similarity transform accumulated from the
/// playground's `translate`/`rotate`/`uniformScale` chain. Rebuildable
/// into any store, which is what makes chained booleans possible: the
/// spec operand is materialized directly into the other operand's store
/// so both sides satisfy [`hybrid::boolean`]'s same-store requirement.
#[derive(Debug, Clone, Copy)]
pub struct ExactSpec {
    prim: ExactPrim,
    /// World placement: applied after `scale`, i.e. the shape is
    /// `iso ∘ scale ∘ primitive`.
    iso: Transform3,
    /// Uniform scale factor (positive, finite), folded into the primitive
    /// dimensions at materialization.
    scale: f64,
}

impl ExactSpec {
    /// An untransformed primitive.
    pub fn new(prim: ExactPrim) -> Self {
        Self {
            prim,
            iso: Transform3::identity(),
            scale: 1.0,
        }
    }

    /// Moved by `offset` (world space), matching `BoundedShape::translate`.
    pub fn translated(&self, offset: Vector3) -> Self {
        Self {
            iso: Transform3::translation(offset.x, offset.y, offset.z) * self.iso,
            ..*self
        }
    }

    /// Rotated about the origin by `axis_angle` (direction = axis, norm =
    /// radians), matching `BoundedShape::rotate`.
    pub fn rotated(&self, axis_angle: Vector3) -> Self {
        Self {
            iso: Transform3::rotation(axis_angle) * self.iso,
            ..*self
        }
    }

    /// Scaled uniformly about the origin, matching
    /// `BoundedShape::uniform_scale`. `None` for a non-positive or
    /// non-finite factor (the SDF path rejects those too).
    pub fn uniform_scaled(&self, factor: f64) -> Option<Self> {
        if !(factor > 0.0 && factor.is_finite()) {
            return None;
        }
        let mut iso = self.iso;
        iso.translation.vector *= factor;
        Some(Self {
            iso,
            scale: self.scale * factor,
            ..*self
        })
    }

    /// Build the primitive into `store`/`geo` and place it. The kernel's
    /// cylinder/torus stand about +Z; the playground's stand about Y, so
    /// those get a −90° pre-rotation about X (mapping +Z to +Y) folded
    /// into the placement.
    pub fn materialize(
        &self,
        store: &mut TopologyStore,
        geo: &mut GeometryStore,
    ) -> CoreResult<EntityId<Body>> {
        let s = self.scale;
        let (body, needs_axis_fix) = match self.prim {
            ExactPrim::Sphere { radius } => (primitives::sphere(store, geo, radius * s)?, false),
            ExactPrim::Block { hx, hy, hz } => (
                primitives::block(store, geo, 2.0 * hx * s, 2.0 * hy * s, 2.0 * hz * s)?,
                false,
            ),
            ExactPrim::Cylinder {
                radius,
                half_height,
            } => (
                primitives::cylinder(store, geo, radius * s, 2.0 * half_height * s)?,
                true,
            ),
            ExactPrim::Torus { major, minor } => {
                (primitives::torus(store, geo, major * s, minor * s)?, true)
            }
        };
        let mut placement = self.iso;
        if needs_axis_fix {
            let z_to_y = Transform3::rotation(Vector3::new(-std::f64::consts::FRAC_PI_2, 0.0, 0.0));
            placement *= z_to_y;
        }
        transform_body(store, geo, body, &placement)?;
        Ok(body)
    }
}

/// An exact boolean result: the store-backed body (for further chained
/// booleans) plus the tessellation that already passed the hybrid gate
/// ladder (closed manifold, chordal deviation, volume validation).
#[derive(Debug)]
pub struct ExactBoolean {
    /// Owned result topology/geometry. `RefCell` because chained booleans
    /// append their primitive operand into this store (see module docs).
    out: RefCell<BooleanOutput>,
    /// The validated exact tessellation, served directly by `mesh()`.
    pub mesh: TriangleMesh,
}

/// The exact companion of a playground shape.
#[derive(Debug, Clone)]
pub enum ExactRep {
    /// Still a (transformed) primitive — no store built yet.
    Spec(ExactSpec),
    /// A boolean result, shared by every shape derived from it.
    Boolean(Rc<ExactBoolean>),
}

impl ExactRep {
    /// The exact mesh this shape can serve instead of an SDF re-mesh, if
    /// any. Specs mesh via the SDF path (primitives dual-contour fine);
    /// only boolean results carry a superior exact tessellation.
    pub fn exact_mesh(&self) -> Option<&TriangleMesh> {
        match self {
            ExactRep::Spec(_) => None,
            ExactRep::Boolean(b) => Some(&b.mesh),
        }
    }
}

/// Try the exact B-Rep pipeline for `op` on two exact-capable operands.
/// `None` means "no exact result" — the pipeline errored, fell back to
/// F-Rep, or the operand combination is out of exact reach (two boolean
/// results own disjoint stores; importing a body across stores is future
/// work) — and the caller's SDF composition stands alone, exactly as with
/// the toggle off.
pub fn exact_boolean(op: BooleanOp, a: &ExactRep, b: &ExactRep) -> Option<ExactBoolean> {
    match (a, b) {
        (ExactRep::Spec(sa), ExactRep::Spec(sb)) => {
            let mut store = TopologyStore::new();
            let mut geo = GeometryStore::new();
            let body_a = sa.materialize(&mut store, &mut geo).ok()?;
            let body_b = sb.materialize(&mut store, &mut geo).ok()?;
            run_hybrid(op, &store, &geo, body_a, body_b)
        }
        (ExactRep::Boolean(ea), ExactRep::Spec(sb)) => {
            let mut out = ea.out.borrow_mut();
            let out = &mut *out;
            let body_b = sb.materialize(&mut out.store, &mut out.geo).ok()?;
            run_hybrid(op, &out.store, &out.geo, out.body, body_b)
        }
        (ExactRep::Spec(sa), ExactRep::Boolean(eb)) => {
            let mut out = eb.out.borrow_mut();
            let out = &mut *out;
            let body_a = sa.materialize(&mut out.store, &mut out.geo).ok()?;
            run_hybrid(op, &out.store, &out.geo, body_a, out.body)
        }
        // Two boolean results live in two different owned stores; the
        // same-store requirement can't be met without a body import.
        (ExactRep::Boolean(_), ExactRep::Boolean(_)) => None,
    }
}

/// Run [`hybrid::boolean`] on two bodies of one store pair and keep the
/// result only when the exact path won end to end.
fn run_hybrid(
    op: BooleanOp,
    store: &TopologyStore,
    geo: &GeometryStore,
    a: EntityId<Body>,
    b: EntityId<Body>,
) -> Option<ExactBoolean> {
    let ha = HybridBody::brep(store, geo, a);
    let hb = HybridBody::brep(store, geo, b);
    let result = hybrid::boolean(op, &ha, &hb, &HybridOptions::default()).ok()?;
    match result.path {
        HybridPath::Brep(out) => Some(ExactBoolean {
            out: RefCell::new(*out),
            mesh: result.mesh,
        }),
        HybridPath::Frep { .. } => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use opensolid_core::types::Point3;

    fn mesh_bounds(mesh: &TriangleMesh) -> (Point3, Point3) {
        let b = mesh.bounding_box().expect("non-empty mesh");
        (b.min, b.max)
    }

    fn assert_close(a: f64, b: f64, tol: f64, what: &str) {
        assert!((a - b).abs() <= tol, "{what}: {a} vs {b}");
    }

    /// The Y-axis convention fix: a materialized playground cylinder must
    /// stand along Y, not the kernel's +Z.
    #[test]
    fn cylinder_materializes_along_y() {
        let spec = ExactSpec::new(ExactPrim::Cylinder {
            radius: 0.5,
            half_height: 1.0,
        });
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = spec.materialize(&mut store, &mut geo).expect("cylinder");
        let mesh = opensolid_kernel::brep::tessellate::tessellate_body(
            &store,
            &geo,
            body,
            &Default::default(),
        )
        .expect("tessellate");
        let (min, max) = mesh_bounds(&mesh);
        assert_close(max.y, 1.0, 1e-9, "top cap");
        assert_close(min.y, -1.0, 1e-9, "bottom cap");
        assert_close(max.x, 0.5, 1e-9, "radius x");
        assert_close(max.z, 0.5, 1e-9, "radius z");
    }

    /// translate/rotate/uniformScale fold into the spec exactly like the
    /// SDF chain: rotate is about the origin, after any translation.
    #[test]
    fn spec_transform_folding_matches_sdf_semantics() {
        let spec = ExactSpec::new(ExactPrim::Block {
            hx: 1.0,
            hy: 0.5,
            hz: 0.25,
        })
        .translated(Vector3::new(2.0, 0.0, 0.0))
        .rotated(Vector3::new(0.0, 0.0, 1.0) * std::f64::consts::FRAC_PI_2)
        .uniform_scaled(2.0)
        .expect("valid factor");
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = spec.materialize(&mut store, &mut geo).expect("block");
        let mesh = opensolid_kernel::brep::tessellate::tessellate_body(
            &store,
            &geo,
            body,
            &Default::default(),
        )
        .expect("tessellate");
        // Quarter turn about z maps the +X offset to +Y, then ×2 scale:
        // center (0, 4, 0), rotated half-extents (0.5, 1.0, 0.25) × 2.
        let (min, max) = mesh_bounds(&mesh);
        assert_close(min.x, -1.0, 1e-9, "min x");
        assert_close(max.x, 1.0, 1e-9, "max x");
        assert_close(min.y, 2.0, 1e-9, "min y");
        assert_close(max.y, 6.0, 1e-9, "max y");
        assert_close(min.z, -0.5, 1e-9, "min z");
        assert_close(max.z, 0.5, 1e-9, "max z");
    }

    #[test]
    fn invalid_uniform_scale_is_rejected() {
        let spec = ExactSpec::new(ExactPrim::Sphere { radius: 1.0 });
        assert!(spec.uniform_scaled(0.0).is_none());
        assert!(spec.uniform_scaled(-2.0).is_none());
        assert!(spec.uniform_scaled(f64::NAN).is_none());
        assert!(spec.uniform_scaled(2.0).is_some());
    }

    /// Two primitive specs boolean exactly: block − cylinder drills a
    /// clean hole and the exact mesh is closed and manifold.
    #[test]
    fn spec_pair_takes_exact_path() {
        let block = ExactRep::Spec(ExactSpec::new(ExactPrim::Block {
            hx: 1.0,
            hy: 0.4,
            hz: 1.0,
        }));
        let hole = ExactRep::Spec(ExactSpec::new(ExactPrim::Cylinder {
            radius: 0.4,
            half_height: 1.0,
        }));
        let out = exact_boolean(BooleanOp::Subtract, &block, &hole).expect("exact path");
        assert!(out.mesh.is_closed_manifold());
        let (min, max) = mesh_bounds(&out.mesh);
        assert_close(min.y, -0.4, 1e-9, "min y");
        assert_close(max.y, 0.4, 1e-9, "max y");
    }

    /// A boolean result chains with a further primitive spec: the spec is
    /// rebuilt into the result's own store (planar-only chain, which the
    /// exact pipeline handles; curved chains may gate to F-Rep — of-6cf).
    #[test]
    fn boolean_result_chains_with_spec() {
        let base = ExactRep::Spec(ExactSpec::new(ExactPrim::Block {
            hx: 1.0,
            hy: 1.0,
            hz: 1.0,
        }));
        let bite = ExactRep::Spec(
            ExactSpec::new(ExactPrim::Block {
                hx: 0.5,
                hy: 0.5,
                hz: 0.5,
            })
            .translated(Vector3::new(1.0, 1.0, 1.0)),
        );
        let first = exact_boolean(BooleanOp::Subtract, &base, &bite).expect("first exact");
        let first = ExactRep::Boolean(Rc::new(first));

        let nibble = ExactRep::Spec(
            ExactSpec::new(ExactPrim::Block {
                hx: 0.5,
                hy: 0.5,
                hz: 0.5,
            })
            .translated(Vector3::new(-1.0, -1.0, -1.0)),
        );
        let second = exact_boolean(BooleanOp::Subtract, &first, &nibble).expect("second exact");
        assert!(second.mesh.is_closed_manifold());

        // ...and in the mirrored operand order (spec ∪ boolean-result).
        let third = exact_boolean(BooleanOp::Unite, &nibble, &first).expect("mirrored exact");
        assert!(third.mesh.is_closed_manifold());
    }

    /// Two boolean results own disjoint stores: out of exact reach.
    #[test]
    fn boolean_pair_declines() {
        let mk = |offset: f64| {
            let a = ExactRep::Spec(ExactSpec::new(ExactPrim::Block {
                hx: 1.0,
                hy: 1.0,
                hz: 1.0,
            }));
            let b = ExactRep::Spec(
                ExactSpec::new(ExactPrim::Block {
                    hx: 0.5,
                    hy: 0.5,
                    hz: 0.5,
                })
                .translated(Vector3::new(offset, offset, offset)),
            );
            ExactRep::Boolean(Rc::new(
                exact_boolean(BooleanOp::Subtract, &a, &b).expect("exact"),
            ))
        };
        assert!(exact_boolean(BooleanOp::Unite, &mk(1.0), &mk(-1.0)).is_none());
    }

    /// A spindle torus (major ≤ minor) is SDF-representable but has no
    /// exact B-Rep constructor: materialization fails, boolean declines.
    #[test]
    fn unsupported_primitive_parameters_decline() {
        let spindle = ExactRep::Spec(ExactSpec::new(ExactPrim::Torus {
            major: 0.2,
            minor: 0.5,
        }));
        let block = ExactRep::Spec(ExactSpec::new(ExactPrim::Block {
            hx: 1.0,
            hy: 1.0,
            hz: 1.0,
        }));
        assert!(exact_boolean(BooleanOp::Unite, &spindle, &block).is_none());
    }

    #[test]
    fn mode_flag_round_trips() {
        assert!(!exact_enabled());
        set_exact_enabled(true);
        assert!(exact_enabled());
        set_exact_enabled(false);
        assert!(!exact_enabled());
    }
}
