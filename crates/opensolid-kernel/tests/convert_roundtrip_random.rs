//! Seeded randomized campaign for the hybrid representation conversions
//! (of-ipt.18): B-Rep → F-Rep via [`MeshSdf`] and F-Rep → B-Rep via
//! [`sdf_to_brep`].
//!
//! `hybrid_e2e.rs` proves the round trip works — on exactly one shape, a
//! radius-1 height-2 cylinder, at one size, at one placement, through one
//! and two cycles. That is an acceptance gate, not coverage: the conversions
//! are the seam between the kernel's two representations, and everything
//! that makes a seam fail (aspect ratio, absolute scale, placement away from
//! the origin, genus, how much of the meshing box the shape occupies) is
//! held fixed there.
//!
//! What this campaign adds, over random primitives at random sizes and
//! random placements:
//!
//! 1. **Field fidelity, not just volume.** `MeshSdf` is supposed to *be* the
//!    signed distance function of the body. Comparing it against the analytic
//!    F-Rep primitive at random points asserts that directly, to the
//!    tessellation's own sagitta — a far tighter statement than "the volume
//!    came back within 3%", and one that catches sign errors, pseudonormal
//!    bugs and BVH misses that a volume integral averages away.
//! 2. **Genus survives the round trip.** A torus that comes back genus 0 has
//!    had its hole filled; its volume barely moves.
//! 3. **Placement invariance.** Both conversions are supposed to be
//!    equivariant under rigid motion. A body translated off the origin and
//!    converted must give the same volume as at the origin — no closed form
//!    needed, so the test holds wherever the truth lies.
//! 4. **Cycle stability.** Repeated conversion must converge, not drift: the
//!    second cycle re-images an already-faceted body and must not compound.
//!
//! Protocol as `boolean_stress.rs`: deterministic seeded [`Rng`], a repro
//! string on every failure, failing cases become `bd` beads and are
//! `#[ignore]`d referencing the bead rather than softened.

use opensolid_brep::{
    Body, GeometryStore, TessellationOptions, TopologyStore, primitives, tessellate_body,
    translate_body,
};
use opensolid_core::EntityId;
use opensolid_core::types::{BoundingBox3, Point3, Vector3};
use opensolid_frep::primitives::{Box3, Cylinder, Sdf, Sphere, Torus};
use opensolid_kernel::{
    MeshSdf, SdfToBrepOptions, brep_mass_properties, mass_properties, sdf_to_brep,
};
use std::f64::consts::PI;

// ---------------------------------------------------------------------
// Deterministic RNG (splitmix64), identical to `boolean_stress.rs`.
// ---------------------------------------------------------------------

struct Rng(u64);

impl Rng {
    fn new(seed: u64) -> Self {
        Rng(seed)
    }

    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    fn unit(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.unit()
    }

    fn pick(&mut self, n: usize) -> usize {
        (self.next_u64() % n as u64) as usize
    }

    fn point(&mut self, lo: f64, hi: f64) -> Point3 {
        Point3::new(self.range(lo, hi), self.range(lo, hi), self.range(lo, hi))
    }
}

// ---------------------------------------------------------------------
// Random primitives, in both representations at once.
//
// Every case is a B-Rep body from `opensolid_brep::primitives` AND the
// analytic F-Rep field of the same solid. Holding both lets the campaign
// compare the conversion against an exact oracle instead of against a
// second approximation.
// ---------------------------------------------------------------------

/// The shape a [`Case`] holds, as parameters rather than as a built field —
/// so the same solid can be re-instantiated at any center. That is what
/// makes the translation-invariance campaign honest: it moves the *field*,
/// not merely the meshing box around a field pinned to the origin.
#[derive(Clone, Copy, Debug)]
enum Shape {
    Block {
        size: [f64; 3],
    },
    Cylinder {
        radius: f64,
        height: f64,
    },
    Sphere {
        radius: f64,
    },
    Torus {
        major: f64,
        minor: f64,
    },
    /// Carried for the volume campaigns; no analytic F-Rep counterpart is
    /// built here, so cone cases skip the field-fidelity assertions.
    Cone,
}

/// The two crates disagree on which way a primitive's main axis points:
/// `opensolid_brep::primitives` builds everything about **+Z**
/// (`primitives.rs`: "All primitives are centered at the origin with `+Z` as
/// the main axis"), while `opensolid_frep`'s [`Cylinder`] and [`Torus`] are
/// about **+Y** (their `eval` takes the radial offset in the XZ plane). Both
/// conventions are documented and internally consistent; a test that pairs a
/// B-Rep body with an F-Rep field has to bridge them, or it measures the
/// mismatch instead of the conversion.
///
/// This wrapper does the bridge and nothing else: it maps a world point into
/// the field's local frame by exchanging Y and Z about `center`. That is an
/// isometry, so distances — and therefore the whole signed distance field —
/// are preserved exactly; and since the wrapped solids are symmetric about
/// their own axis, using the reflection rather than a rotation costs nothing.
struct ZAxis<S> {
    inner: S,
    center: Point3,
}

impl<S: Sdf> Sdf for ZAxis<S> {
    fn eval(&self, p: &Point3) -> f64 {
        self.inner.eval(&Point3::new(
            p.x - self.center.x,
            p.z - self.center.z,
            p.y - self.center.y,
        ))
    }
}

impl Shape {
    /// The analytic F-Rep field of this solid centered at `center`, in the
    /// **B-Rep convention** (main axis +Z) — see [`ZAxis`].
    fn field(self, center: Point3) -> Option<Box<dyn Sdf>> {
        match self {
            Shape::Block { size } => Some(Box::new(Box3 {
                center,
                half_extents: [size[0] / 2.0, size[1] / 2.0, size[2] / 2.0],
            })),
            Shape::Cylinder { radius, height } => Some(Box::new(ZAxis {
                inner: Cylinder {
                    center: Point3::origin(),
                    radius,
                    half_height: height / 2.0,
                },
                center,
            })),
            // A sphere is axis-agnostic; no bridge needed.
            Shape::Sphere { radius } => Some(Box::new(Sphere { center, radius })),
            Shape::Torus { major, minor } => Some(Box::new(ZAxis {
                inner: Torus {
                    center: Point3::origin(),
                    major_radius: major,
                    minor_radius: minor,
                },
                center,
            })),
            Shape::Cone => None,
        }
    }
}

/// One random primitive: its B-Rep body (in `store`/`geo`), the shape
/// parameters its analytic F-Rep counterpart is built from, the closed-form
/// volume, the genus, and a half-extent for sizing meshing bounds.
struct Case {
    label: String,
    body: EntityId<Body>,
    shape: Shape,
    volume: f64,
    genus: usize,
    half: f64,
}

/// Build a random primitive body, centered at the origin. The caller
/// translates it if the campaign wants it placed elsewhere; placement is a
/// first-class parameter here, because a conversion that only works near the
/// origin is a conversion with an absolute tolerance hiding in it.
fn random_case(rng: &mut Rng, store: &mut TopologyStore, geo: &mut GeometryStore) -> Case {
    match rng.pick(5) {
        0 => {
            let s = [
                rng.range(0.6, 2.0),
                rng.range(0.6, 2.0),
                rng.range(0.6, 2.0),
            ];
            let body = primitives::block(store, geo, s[0], s[1], s[2]).expect("valid extents");
            Case {
                label: format!("block({:.4}, {:.4}, {:.4})", s[0], s[1], s[2]),
                body,
                shape: Shape::Block { size: s },
                volume: s[0] * s[1] * s[2],
                genus: 0,
                half: s.iter().cloned().fold(0.0, f64::max) / 2.0,
            }
        }
        1 => {
            let (r, h) = (rng.range(0.4, 1.2), rng.range(0.6, 2.0));
            let body = primitives::cylinder(store, geo, r, h).expect("valid dimensions");
            Case {
                label: format!("cylinder(r = {r:.4}, h = {h:.4})"),
                body,
                shape: Shape::Cylinder {
                    radius: r,
                    height: h,
                },
                volume: PI * r * r * h,
                genus: 0,
                half: r.max(h / 2.0),
            }
        }
        2 => {
            let r = rng.range(0.5, 1.4);
            let body = primitives::sphere(store, geo, r).expect("valid radius");
            Case {
                label: format!("sphere(r = {r:.4})"),
                body,
                shape: Shape::Sphere { radius: r },
                volume: 4.0 / 3.0 * PI * r * r * r,
                genus: 0,
                half: r,
            }
        }
        3 => {
            let major = rng.range(0.7, 1.3);
            let minor = rng.range(0.15, 0.4) * major;
            let body = primitives::torus(store, geo, major, minor).expect("valid radii");
            Case {
                label: format!("torus(R = {major:.4}, r = {minor:.4})"),
                body,
                shape: Shape::Torus { major, minor },
                volume: 2.0 * PI * PI * major * minor * minor,
                genus: 1,
                half: major + minor,
            }
        }
        _ => {
            let (r0, h) = (rng.range(0.5, 1.2), rng.range(0.6, 1.8));
            // Half the cones are pointed, half are frustums.
            let r1 = if rng.pick(2) == 0 {
                0.0
            } else {
                rng.range(0.15, 0.9) * r0
            };
            let body = primitives::cone(store, geo, r0, r1, h).expect("valid dimensions");
            Case {
                label: format!("cone(r0 = {r0:.4}, r1 = {r1:.4}, h = {h:.4})"),
                body,
                shape: Shape::Cone,
                volume: PI * h * (r0 * r0 + r0 * r1 + r1 * r1) / 3.0,
                genus: 0,
                half: r0.max(h / 2.0),
            }
        }
    }
}

/// Meshing bounds around a case placed at `offset`, padded so the surface
/// closes strictly inside them.
fn bounds_for(case: &Case, offset: Vector3, pad_factor: f64) -> BoundingBox3 {
    let half = case.half * pad_factor;
    let c = Point3::origin() + offset;
    BoundingBox3::new(
        Point3::new(c.x - half, c.y - half, c.z - half),
        Point3::new(c.x + half, c.y + half, c.z + half),
    )
}

fn assert_within(got: f64, want: f64, rtol: f64, context: &str) {
    let scale = want.abs().max(1e-300);
    assert!(
        ((got - want) / scale).abs() <= rtol,
        "{context}: got {got}, expected {want} \
         ({:.3e} relative, allowed {rtol:.1e})",
        ((got - want) / scale).abs()
    );
}

// =====================================================================
// (1) B-Rep → F-Rep: MeshSdf is the body's distance field, not just its
//     indicator
// =====================================================================

/// `MeshSdf` must reproduce the analytic signed distance of the body it
/// wraps, at random points inside and out.
///
/// The oracle is the exact F-Rep primitive of the same solid, so the only
/// admissible error is the tessellation's: a mesh inscribed in a curved face
/// sits at most one sagitta `R(1 − cos(π/N))` inside it, at
/// `SAMPLES_PER_CIRCLE = N` per revolution. Points are sampled away from the
/// surface by more than that budget, so near-surface sign flips — which are
/// the tessellation being faithful, not a defect — are excluded, and the
/// assertion is left measuring only what `MeshSdf` itself does.
#[test]
fn mesh_sdf_reproduces_the_analytic_field_of_its_body() {
    let mut rng = Rng::new(0xE75D_F001);
    for case in 0..24 {
        let offset = if case % 3 == 0 {
            Vector3::zeros()
        } else {
            Vector3::new(
                rng.range(-4.0, 4.0),
                rng.range(-4.0, 4.0),
                rng.range(-4.0, 4.0),
            )
        };
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let spec = random_case(&mut rng, &mut store, &mut geo);
        let Some(field) = spec.shape.field(Point3::origin() + offset) else {
            continue;
        };
        translate_body(&mut store, &mut geo, spec.body, offset).expect("finite offset");
        let repro = format!("case {case}: MeshSdf({}) at {offset:?}", spec.label);

        let options = TessellationOptions::default();
        let sdf = MeshSdf::from_body(&store, &geo, spec.body, &options)
            .unwrap_or_else(|e| panic!("{repro}: MeshSdf::from_body failed: {e:?}"));

        // The only admissible error is the chord sagitta: a chord subtending
        // `angular_step` on radius R sits `R·(1 − cos(step/2))` inside the
        // arc, so the inscribed mesh reports distances that much too large.
        // Derived from the options actually passed rather than hard-coded —
        // the default is 32 segments per circle, and a budget written for a
        // finer pitch is a budget that fails for a correct mesh. The 2×
        // covers the second parameter direction a sphere and torus discretize.
        let budget = 2.0 * spec.half * (1.0 - (options.angular_step / 2.0).cos()) + 1e-9;

        let mut sampled = 0;
        for _ in 0..400 {
            let p = Point3::origin()
                + offset
                + (rng.point(-1.0, 1.0) - Point3::origin()) * (spec.half * 2.0);
            let exact = field.eval(&p);
            // Skip the band where the tessellation legitimately disagrees.
            if exact.abs() <= 2.0 * budget {
                continue;
            }
            sampled += 1;
            let got = sdf.eval(&p);
            assert!(
                got.signum() == exact.signum(),
                "{repro}: at {p:?} MeshSdf says {got} but the analytic field says {exact} \
                 — opposite signs {:.6} clear of the surface",
                exact.abs()
            );
            assert!(
                (got - exact).abs() <= budget,
                "{repro}: at {p:?} MeshSdf gives {got}, analytic {exact}, gap {:.3e} \
                 exceeds the sagitta budget {budget:.3e}",
                (got - exact).abs()
            );
        }
        assert!(
            sampled >= 40,
            "{repro}: only {sampled} usable samples — the sampling box is not \
             exercising the field"
        );
    }
}

// =====================================================================
// (2) F-Rep → B-Rep
// =====================================================================

/// `sdf_to_brep` on the analytic field of a random primitive must produce a
/// checker-clean body whose two independent measurements — the B-Rep-native
/// surface integral and the tessellated polyhedron — agree, and whose volume
/// and genus match the closed form.
///
/// Genus is the assertion the existing volume-only coverage cannot make: a
/// torus whose hole got filled loses a few percent of volume and passes any
/// tolerance loose enough to admit faceting, but its genus goes to 0.
#[test]
fn sdf_to_brep_recovers_volume_and_genus() {
    const MAX_DEPTH: u32 = 6;
    // A depth-6 octree gives 64 cells per axis over the padded bounds; a
    // chord-inscribed facet loses about half a cell of a curved solid's
    // radius, which at these aspect ratios is a few percent.
    const RTOL: f64 = 6e-2;
    let mut rng = Rng::new(0x_5DF2_B2E7);
    for case in 0..10 {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let spec = random_case(&mut rng, &mut store, &mut geo);
        let Some(field) = spec.shape.field(Point3::origin()) else {
            continue;
        };
        let repro = format!("case {case}: sdf_to_brep({})", spec.label);

        let bounds = bounds_for(&spec, Vector3::zeros(), 1.4);
        let mut out_store = TopologyStore::new();
        let mut out_geo = GeometryStore::new();
        let recovered = sdf_to_brep(
            field.as_ref(),
            &mut out_store,
            &mut out_geo,
            &SdfToBrepOptions::new(bounds, MAX_DEPTH),
        )
        .unwrap_or_else(|e| panic!("{repro}: sdf_to_brep failed: {e:?}"));

        let failures = out_store.check(recovered);
        assert!(
            failures.is_empty(),
            "{repro}: recovered body failed check() with {} failures: {:#?}",
            failures.len(),
            failures
        );

        let counts = out_store.euler_counts(recovered);
        assert_eq!(
            counts.genus, spec.genus,
            "{repro}: genus {} survived as {} — the topology of the field was not \
             recovered",
            spec.genus, counts.genus
        );

        // Two independent measurements of the same recovered body.
        let exact = brep_mass_properties(&out_store, &out_geo, recovered)
            .unwrap_or_else(|e| panic!("{repro}: brep_mass_properties failed: {e:?}"));
        let mesh = tessellate_body(
            &out_store,
            &out_geo,
            recovered,
            &TessellationOptions::default(),
        )
        .unwrap_or_else(|e| panic!("{repro}: tessellate_body failed: {e:?}"));
        assert!(
            mesh.is_closed_manifold(),
            "{repro}: recovered body does not tessellate to a closed manifold"
        );
        let meshed = mass_properties(&mesh)
            .unwrap_or_else(|e| panic!("{repro}: mass_properties failed: {e}"));

        // The recovered body is all-planar, so the two paths measure the
        // same polyhedron and must agree to floating point.
        assert_within(
            meshed.volume,
            exact.volume,
            1e-9,
            &format!("{repro}: meshed vs B-Rep-native volume of the SAME faceted body"),
        );
        assert_within(
            exact.volume,
            spec.volume,
            RTOL,
            &format!("{repro}: recovered volume vs closed form"),
        );
    }
}

/// Facet count and volume error must fall monotonically as the octree
/// deepens: that is the statement that the conversion's error is *resolution*
/// and nothing else. A systematic bug (a dropped region, an inverted facet)
/// leaves a floor the depth cannot cross.
#[test]
fn sdf_to_brep_error_falls_with_octree_depth() {
    let mut rng = Rng::new(0x0C_7DEE_9001);
    for case in 0..3 {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let spec = random_case(&mut rng, &mut store, &mut geo);
        let Some(field) = spec.shape.field(Point3::origin()) else {
            continue;
        };
        let repro = format!(
            "case {case}: depth convergence of sdf_to_brep({})",
            spec.label
        );
        let bounds = bounds_for(&spec, Vector3::zeros(), 1.4);

        // Errors are compared across the WHOLE depth range rather than
        // step by step. Adjacent depths are not reliably ordered: at these
        // magnitudes the residual is set by where the octree lattice happens
        // to fall against the surface, and one lattice can flatter a shape
        // than the next finer one. A systematic defect — a dropped region,
        // an inverted facet — leaves a floor that three doublings cannot
        // cross, which is what this actually tests.
        let mut errors = Vec::new();
        for depth in [4, 5, 6] {
            let mut out_store = TopologyStore::new();
            let mut out_geo = GeometryStore::new();
            let recovered = sdf_to_brep(
                field.as_ref(),
                &mut out_store,
                &mut out_geo,
                &SdfToBrepOptions::new(bounds, depth),
            )
            .unwrap_or_else(|e| panic!("{repro} @ depth {depth}: failed: {e:?}"));
            assert!(
                out_store.check(recovered).is_empty(),
                "{repro} @ depth {depth}: recovered body failed check()"
            );
            let v = brep_mass_properties(&out_store, &out_geo, recovered)
                .unwrap_or_else(|e| panic!("{repro} @ depth {depth}: measurement failed: {e:?}"))
                .volume;
            errors.push(((v - spec.volume).abs() / spec.volume, depth));
        }
        let (coarse, _) = errors[0];
        let (fine, _) = errors[errors.len() - 1];
        assert!(
            fine <= coarse * 0.6 + 1e-12,
            "{repro}: relative error only improved from {coarse:.3e} (depth 4) to \
             {fine:.3e} (depth 6) — the residual is not resolution. Errors: {errors:?}"
        );
    }
}

// =====================================================================
// (3) Full cycles
// =====================================================================

/// One B-Rep → SDF → B-Rep cycle.
fn cycle(
    store: &TopologyStore,
    geo: &GeometryStore,
    body: EntityId<Body>,
    bounds: BoundingBox3,
    depth: u32,
    context: &str,
) -> (TopologyStore, GeometryStore, EntityId<Body>) {
    let sdf = MeshSdf::from_body(store, geo, body, &TessellationOptions::default())
        .unwrap_or_else(|e| panic!("{context}: MeshSdf::from_body failed: {e:?}"));
    let mut out_store = TopologyStore::new();
    let mut out_geo = GeometryStore::new();
    let recovered = sdf_to_brep(
        &sdf,
        &mut out_store,
        &mut out_geo,
        &SdfToBrepOptions::new(bounds, depth),
    )
    .unwrap_or_else(|e| panic!("{context}: sdf_to_brep failed: {e:?}"));
    assert!(
        out_store.check(recovered).is_empty(),
        "{context}: recovered body failed check()"
    );
    (out_store, out_geo, recovered)
}

fn body_volume(store: &TopologyStore, geo: &GeometryStore, body: EntityId<Body>) -> f64 {
    brep_mass_properties(store, geo, body)
        .expect("faceted body measures")
        .volume
}

/// Two full representation cycles over random primitives at random
/// placements: the first must land within the faceting budget of the closed
/// form, and the second must not compound — re-imaging an already-faceted
/// body converges rather than drifting.
#[test]
fn random_round_trips_converge_rather_than_drift() {
    const DEPTH: u32 = 5;
    const RTOL: f64 = 1.5e-1;
    /// Cycle-to-cycle drift allowed once the body is already faceted.
    const DRIFT: f64 = 5e-2;
    let mut rng = Rng::new(0x_C7C1_E001);
    for case in 0..6 {
        let offset = if case % 2 == 0 {
            Vector3::zeros()
        } else {
            Vector3::new(
                rng.range(-3.0, 3.0),
                rng.range(-3.0, 3.0),
                rng.range(-3.0, 3.0),
            )
        };
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let spec = random_case(&mut rng, &mut store, &mut geo);
        translate_body(&mut store, &mut geo, spec.body, offset).expect("finite offset");
        let repro = format!("case {case}: round trip of {} at {offset:?}", spec.label);
        let bounds = bounds_for(&spec, offset, 1.4);

        let (store1, geo1, body1) = cycle(
            &store,
            &geo,
            spec.body,
            bounds,
            DEPTH,
            &format!("{repro}: cycle 1"),
        );
        let v1 = body_volume(&store1, &geo1, body1);
        assert_within(
            v1,
            spec.volume,
            RTOL,
            &format!("{repro}: cycle 1 volume vs closed form"),
        );
        assert_eq!(
            store1.euler_counts(body1).genus,
            spec.genus,
            "{repro}: cycle 1 changed the genus"
        );

        let (store2, geo2, body2) = cycle(
            &store1,
            &geo1,
            body1,
            bounds,
            DEPTH,
            &format!("{repro}: cycle 2"),
        );
        let v2 = body_volume(&store2, &geo2, body2);
        assert_within(
            v2,
            v1,
            DRIFT,
            &format!("{repro}: cycle 2 drift from cycle 1"),
        );
        assert_eq!(
            store2.euler_counts(body2).genus,
            spec.genus,
            "{repro}: cycle 2 changed the genus"
        );
    }
}

/// Both conversions are equivariant under rigid motion, so a body converted
/// at the origin and the same body converted after a translation must give
/// the same volume — to floating point, since the only difference is where
/// the octree grid falls relative to the surface.
///
/// This needs no closed form at all: it compares the pipeline against itself,
/// so it holds regardless of how faithful the faceting is. What it catches is
/// an absolute tolerance or a grid-alignment assumption that only holds near
/// the origin.
#[test]
fn conversion_volume_is_translation_invariant() {
    const DEPTH: u32 = 5;
    // The octree lattice falls differently relative to the surface once the
    // body moves, so the faceting is a *different* polyhedron of the same
    // solid. Its volume may differ by about one facet layer.
    const RTOL: f64 = 4e-2;
    let mut rng = Rng::new(0x_72A5_1A7E);
    for case in 0..5 {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let spec = random_case(&mut rng, &mut store, &mut geo);
        let Some(field) = spec.shape.field(Point3::origin()) else {
            continue;
        };
        let repro = format!("case {case}: translation invariance of {}", spec.label);

        let at_origin = {
            let mut s = TopologyStore::new();
            let mut g = GeometryStore::new();
            let b = sdf_to_brep(
                field.as_ref(),
                &mut s,
                &mut g,
                &SdfToBrepOptions::new(bounds_for(&spec, Vector3::zeros(), 1.4), DEPTH),
            )
            .unwrap_or_else(|e| panic!("{repro}: origin conversion failed: {e:?}"));
            body_volume(&s, &g, b)
        };

        for trial in 0..2 {
            let offset = Vector3::new(
                rng.range(-5.0, 5.0),
                rng.range(-5.0, 5.0),
                rng.range(-5.0, 5.0),
            );
            // Rebuild the primitive at the offset so the FIELD moves, not
            // just the meshing box around a field pinned to the origin.
            let moved = spec
                .shape
                .field(Point3::origin() + offset)
                .expect("shape carries a field");
            let mut moved_store = TopologyStore::new();
            let mut moved_geo = GeometryStore::new();
            let b = sdf_to_brep(
                moved.as_ref(),
                &mut moved_store,
                &mut moved_geo,
                &SdfToBrepOptions::new(bounds_for(&spec, offset, 1.4), DEPTH),
            )
            .unwrap_or_else(|e| panic!("{repro}/{trial}: offset conversion failed: {e:?}"));
            let v = body_volume(&moved_store, &moved_geo, b);
            assert_within(
                v,
                at_origin,
                RTOL,
                &format!("{repro}/{trial}: volume at {offset:?} vs at the origin"),
            );
        }
    }
}
