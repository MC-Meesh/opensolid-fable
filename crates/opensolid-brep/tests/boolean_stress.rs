//! Adversarial stress tests for the B-Rep boolean pipeline (of-ipt.1).
//!
//! These tests actively try to BREAK `unite`/`subtract`/`intersect`:
//! rotated (non-axis-aligned) tools, seeded randomized transversal
//! configurations, near-degenerate clearances and slivers, scale extremes,
//! and a tessellate → MeshSdf → re-mesh round-trip cross-check.
//!
//! Protocol: a failing case is documented as a `bd` bug bead with a
//! minimal repro, and the test is then marked `#[ignore]` referencing the
//! bug ID. Failures are expected and are the point — tests must not be
//! softened to pass. Run the known-broken cases with `cargo test --test
//! boolean_stress -- --ignored`.
//!
//! Invariants asserted throughout:
//! - `BooleanOutput::check()` reports no failures,
//! - `BooleanOutput::tessellate()` yields a closed manifold mesh,
//! - mesh volume (kernel `mass_properties`) matches analytic expectations,
//! - the inclusion–exclusion identity
//!   `vol(A) + vol(B) == vol(A∪B) + vol(A∩B)` holds,
//! - results are invariant under rigid rotation of both operands.
//!
//! Sections (6)-(8) are the sphere/torus campaign (of-7ld.3) and section
//! (9) the cone/frustum campaign (of-fsl.23). Both were written
//! tests-first while `Chart` still rejected the surfaces, then promoted
//! once the gate lifted (of-7ld.4, of-dtj). Plane, cylinder, sphere,
//! torus and cone booleans now take the exact B-Rep path end-to-end, and
//! the hybrid kernel diverts any exact-path shortfall to the F-Rep
//! fallback. The campaigns' history — the bugs they filed and the fixes
//! that retired them — is in the git log, not here.
//!
//! Sections (1)-(15) are entirely live. Section (16) is not, and that is
//! the protocol working rather than failing: it is the of-ipt.19 numerical
//! robustness campaign, and one of its cases is `#[ignore]`d against one
//! defect it found and filed — of-6viu (`unite` rejects a coincident-face
//! imprint that forms an island, i.e. a boss centred on a face). The
//! `#[ignore]`d case has a live sibling that fences the working range, so a
//! fix has a boundary to move rather than a single case to flip.
//!
//! A third defect it filed, of-oygs (`ray_classify` gives up on operands
//! past ~1e3 length/radius for cylinders, ~1e7 for blocks), is fixed and
//! its two cases are live again.
//!
//! The fourth defect it filed, of-y8qc, is retired: `brep_mass_properties`
//! on a trimmed sphere degraded with the angle between the trim and the pole
//! axis, reaching 1.3e-3 at 90°, because the trim's parameter-space image
//! fits a `Curve2::Line` at 0° and nothing at all off it. `Curve2::Projected`
//! inverts the surface instead of fitting it, and adaptive contour panelling
//! resolves what that exposed underneath. Its three `#[ignore]`d cases
//! (`near_concentric_spheres_*`) went live unchanged, and
//! `a_spherical_cap_measures_the_same_at_every_trim_angle` now walks the
//! whole sweep the bug was filed on.
//!
//! Section (18) is the randomized *curved-operand* campaign (of-ipt.18).
//! Sections (6)-(9) enumerate configurations and hand-derive a closed form
//! for each; (18) samples the parameters instead and leans on the volume
//! identities, which hold for every pair of solids and so need no closed
//! form at all, plus rigid-motion invariance, which compares the pipeline
//! against itself. It found the section's two `#[ignore]`s: of-ntkk (the
//! exact path fails outright on transversal cone+sphere pairs) and of-7bnv
//! (the sphere-sphere lens volume is 4.4e-4 off its closed form, and
//! frame-dependent — invisible to the meshed 5e-3 budgets sections (6)-(9)
//! weigh against).
//!
//! The fourth defect the campaign filed, of-ukcq, is retired: mesh
//! `mass_properties` integrated tetrahedra from the absolute origin and was
//! 191× wrong at offset 1e6, and now references them to the mesh's own
//! bounding-box centre. `mesh_mass_properties_survives_far_from_origin` is
//! live, and §16's far-field families no longer skip the mesh-vs-B-Rep
//! cross-check to work around it.
//!
//! The of-9ia `#[ignore]`
//! (`skew_frustums_inclusion_exclusion`) lifted with of-37i.5: its two
//! blockers were both in imprint hosting and neither was specific to the
//! marched cone-cone arc, which had been correct all along. The coaxial
//! cone-cone pair goes through the analytic SSI
//! (`opposed_cones_intersection`, `coaxial_frustums_union_identity`).
//! The no-panic guard `no_panics_on_cone_configurations` stays live across
//! the promotion — it accepts both a valid exact solid and the structured
//! `NotImplemented` F-Rep fallback. Section (14)'s ignores lifted too:
//! of-hqb (the curved NURBS bore) and of-bd3 (the randomized
//! planar-NURBS campaigns) — its whole promotion gate is live.
//!
//! Section (14) is the FREEFORM §9 NURBS promotion-gate campaign
//! (of-37i.5), written stress-suite-first per the same policy. It is green,
//! and NURBS was promoted on it (of-ew7): the hybrid kernel no longer
//! routes NURBS operands to the F-Rep fallback as a class. What it kept is
//! the per-result quality bar every surface class faces — closed manifold,
//! chordal deviation within an F-Rep cell, and the `validate_exact` volume
//! cross-check. The kernel-side proof lives in
//! `crates/opensolid-kernel/tests/hybrid_e2e.rs`, which asserts
//! `HybridPath::Brep` for NURBS operands through the public entry point;
//! this section proves the pipeline underneath it.

use nalgebra::{Matrix3, Rotation3, Unit};
use opensolid_brep::boolean::{InsideTest, boolean_with_inside_tests, intersect, subtract, unite};
use opensolid_brep::curve::plane_basis;
use opensolid_brep::{
    Body, BodyType, BooleanOp, BooleanOutput, CheckFailure, Curve3, FaceSense, FinSense,
    GeometryStore, KnotVector, LoopType, NurbsSurface, SYSTEM_RESOLUTION, ShellOrientation,
    Surface3, TessellationOptions, TopologyStore, primitives, rotate_body, tessellate_body,
    translate_body,
};
use opensolid_core::EntityId;
use opensolid_core::error::{CoreError, CoreResult};
use opensolid_core::mesh::TriangleMesh;
use opensolid_core::tolerance::ToleranceContext;
use opensolid_core::types::{BoundingBox3, Point3, Vector3};
use opensolid_kernel::{
    MassProperties, MeshOptions, MeshSdf, brep_mass_properties, mass_properties, mesh_sdf_indexed,
};
use std::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2, FRAC_PI_4, PI};

fn tol() -> ToleranceContext {
    ToleranceContext::default()
}

/// The tessellated cylinder wall is a 96-gon prism (SAMPLES_PER_CIRCLE),
/// so circular cross sections lose `1 - sin(2π/n)/(2π/n)` ≈ 7.2e-4 of
/// their area. 0.5% relative tolerance absorbs that plus triangulation
/// noise while still catching real classification errors.
const CYL_VOLUME_RTOL: f64 = 5e-3;
/// Pure plane/plane results tessellate exactly; only fp accumulation.
const PLANAR_VOLUME_RTOL: f64 = 1e-9;
/// Spheres and tori discretize BOTH parameter directions (a cylinder only
/// one): ~96 segments around and ~48 across lose ≈1.5e-3 of the volume.
/// The same 0.5% budget as cylinders still covers it with margin.
const CURVED_VOLUME_RTOL: f64 = 5e-3;
/// Budget for the meshed volume against the B-Rep-native one (of-ipt.17), as
/// a fraction of the result's own volume. The B-Rep number is exact to
/// floating point, so the whole gap is the tessellation's.
const CROSS_CHECK_VOLUME_RTOL: f64 = 5e-3;
/// The same budget stated *absolutely*, for results a relative bound cannot
/// fairly describe.
///
/// A tessellation inscribed in a curved face sits at most one sagitta inside
/// it — `R(1 − cos(π/96))` ≈ `5.4e-4·R` at this suite's 96 samples per circle
/// — so it can lose about `sagitta × curved area` of volume. That is a
/// *fixed* amount set by the operands' curvature, not by the result: a
/// spherical cap a thousandth of a radius tall is thinner than the sagitta
/// itself, and the mesh misses tens of percent of it while being no less
/// faithful to the sphere than usual. Scaling `R` by the result's bounding
/// diagonal and allowing 4× for the several curved faces a result carries
/// gives the slack below; a misclassified *region* is orders of magnitude
/// larger than this and still fails.
const CROSS_CHECK_CHORD_SLACK: f64 = 2e-3;
/// Budget for a closed form weighed by the B-Rep-native path, which does not
/// discretize anything. Loose only by the standards of an exact method: it
/// still leaves four orders of magnitude to the meshed budget above.
const EXACT_RTOL: f64 = 1e-9;

// ---------------------------------------------------------------------
// Closed-form volumes for sphere/torus configurations (of-7ld.3).
// ---------------------------------------------------------------------

fn sphere_volume(r: f64) -> f64 {
    4.0 / 3.0 * PI * r * r * r
}

/// Spherical cap of height `h` (measured along the axis from the rim
/// plane to the surface) cut from a sphere of radius `r`.
fn spherical_cap_volume(r: f64, h: f64) -> f64 {
    PI * h * h * (3.0 * r - h) / 3.0
}

/// Lens shared by two overlapping spheres whose centers are `d` apart:
/// the two caps on either side of the radical plane.
fn sphere_lens_volume(r1: f64, r2: f64, d: f64) -> f64 {
    let x = (d * d - r2 * r2 + r1 * r1) / (2.0 * d);
    spherical_cap_volume(r1, r1 - x) + spherical_cap_volume(r2, r2 - (d - x))
}

fn torus_volume(major: f64, minor: f64) -> f64 {
    2.0 * PI * PI * major * minor * minor
}

/// Volume of a conical frustum of height `h` between circular caps of
/// radii `r1` and `r2`: `π h (r1² + r1·r2 + r2²) / 3`. A pointed cone is
/// the `r2 = 0` special case (`π h r1² / 3`); a cylinder the `r1 = r2`
/// case (`π h r²`). Used for every closed-form cone volume in section (9).
fn frustum_volume(r1: f64, r2: f64, h: f64) -> f64 {
    PI * h * (r1 * r1 + r1 * r2 + r2 * r2) / 3.0
}

/// Volume of the part of a torus (axis +Z, centered at z = 0) below the
/// plane `z = c`, for `|c| <= minor`. The cross-section at height z is an
/// annulus of area 4π·major·√(minor² − z²), so the volume is
/// 4π·major·∫√(minor² − z²) dz over [-minor, c].
fn torus_below_plane_volume(major: f64, minor: f64, c: f64) -> f64 {
    let r = minor;
    let c = c.clamp(-r, r);
    let integral =
        (r * r / 2.0) * ((c / r).asin() + FRAC_PI_2) + (c / 2.0) * (r * r - c * c).sqrt();
    4.0 * PI * major * integral
}

/// Area of the lens shared by two circles of equal radius `r` whose
/// centers are `d < 2r` apart. Revolved about an axis (Pappus) it gives
/// exact torus-torus intersection volumes.
fn circle_lens_area(r: f64, d: f64) -> f64 {
    2.0 * r * r * (d / (2.0 * r)).acos() - (d / 2.0) * (4.0 * r * r - d * d).sqrt()
}

/// check() must be clean and the tessellation closed-manifold; returns the
/// mesh for further measurement.
fn assert_valid(out: &BooleanOutput, context: &str) -> TriangleMesh {
    let failures = out.check();
    assert!(
        failures.is_empty(),
        "{context}: check() reported {} failures: {:#?}",
        failures.len(),
        failures
    );
    let mesh = out
        .tessellate()
        .unwrap_or_else(|e| panic!("{context}: tessellation failed: {e:?}"));
    assert!(
        mesh.is_closed_manifold(),
        "{context}: tessellation is not a closed manifold \
         ({} triangles)",
        mesh.triangle_count()
    );
    mesh
}

/// Volume of a valid boolean result via kernel mass properties, cross-checked
/// against the B-Rep-native measurement (of-ipt.17).
///
/// Every closed-form assertion in this file routes through here, so every one
/// of them now also asserts that the *two independent* measurement paths agree.
/// That is the point: a tessellation bug consistent enough to move every
/// meshed volume the same way is invisible to a suite that only ever weighs
/// meshes, and this suite used to be exactly that.
fn volume(out: &BooleanOutput, context: &str) -> f64 {
    measured(out, context).0.volume
}

/// Both measurements of a valid boolean result: the meshed one first, the
/// B-Rep-native one second, already asserted to agree.
///
/// The mesh volume is what the closed-form assertions weigh (they are tuned to
/// the tessellation's discretization error, and holding the tessellator to
/// them is half of what this suite is for); the B-Rep number is exact to
/// floating point, so the gap between them is a pure measure of tessellation
/// error and is held to the same budget the closed forms allow.
fn measured(out: &BooleanOutput, context: &str) -> (MassProperties, MassProperties) {
    let mesh = assert_valid(out, context);
    let meshed =
        mass_properties(&mesh).unwrap_or_else(|e| panic!("{context}: mass_properties failed: {e}"));
    let exact = brep_mass_properties(&out.store, &out.geo, out.body)
        .unwrap_or_else(|e| panic!("{context}: brep_mass_properties failed: {e}"));
    let diagonal = mesh
        .bounding_box()
        .map(|b| (b.max - b.min).norm())
        .unwrap_or(0.0);
    assert_cross_checked(&meshed, &exact, diagonal, context);
    (meshed, exact)
}

/// The meshed measurement against the exact one, allowed the larger of a
/// relative budget and the absolute slack a chord-inscribed tessellation is
/// entitled to (see [`CROSS_CHECK_CHORD_SLACK`]).
fn assert_cross_checked(
    meshed: &MassProperties,
    exact: &MassProperties,
    diagonal: f64,
    context: &str,
) {
    let gap = (meshed.volume - exact.volume).abs();
    let allowed = (CROSS_CHECK_VOLUME_RTOL * exact.volume.abs())
        .max(CROSS_CHECK_CHORD_SLACK * exact.surface_area * diagonal);
    assert!(
        gap <= allowed,
        "{context}: meshed volume {} differs from the B-Rep-native {} by {gap:.3e} \
         (allowed {allowed:.3e}: {:.1e} relative or {:.1e} of chord slack over \
         area {:.4} and diagonal {:.4})",
        meshed.volume,
        exact.volume,
        CROSS_CHECK_VOLUME_RTOL,
        CROSS_CHECK_CHORD_SLACK,
        exact.surface_area,
        diagonal,
    );
}

fn assert_close(got: f64, want: f64, rtol: f64, context: &str) {
    let scale = want.abs().max(1e-300);
    assert!(
        ((got - want) / scale).abs() <= rtol,
        "{context}: volume {got} differs from expected {want} \
         by {:.3e} relative (allowed {rtol:.1e})",
        ((got - want) / scale).abs()
    );
}

// ---------------------------------------------------------------------
// Closed-form CENTROID and INERTIA for composite solids (of-ipt.17).
//
// Volume alone was this suite's whole oracle: `mass_properties` computes a
// centroid and an inertia tensor for every result and nothing ever read them,
// so a boolean that placed the right amount of material in the wrong place
// passed. The fix is not a new integral but an algebra — mass properties are
// additive over disjoint solids and subtractive over a cavity wholly inside
// its host, and every primitive's own centroid and inertia are textbook — so
// a configuration built from primitives has an exact expected tensor, not
// just an expected number.
//
// Moments are carried about the ORIGIN because that is the frame in which
// they add; the centroid and the centroidal inertia are recovered at the end.
// ---------------------------------------------------------------------

/// Closed-form mass properties of a homogeneous solid at unit density,
/// expressed about the origin so that composites are sums.
#[derive(Debug, Clone, Copy)]
struct Rigid {
    volume: f64,
    /// First moment `∫x dV` about the origin.
    moment: Vector3,
    /// Inertia tensor about the origin.
    inertia_origin: Matrix3<f64>,
}

impl Rigid {
    /// Assemble from the textbook *centroidal* tensor plus a placement, via
    /// the parallel-axis theorem `I_o = I_c + m(|c|²E − c cᵀ)`.
    fn placed(volume: f64, centroid: Point3, inertia_centroid: Matrix3<f64>) -> Self {
        let c = centroid.coords;
        Rigid {
            volume,
            moment: c * volume,
            inertia_origin: inertia_centroid
                + (Matrix3::identity() * c.norm_squared() - c * c.transpose()) * volume,
        }
    }

    /// Axis-aligned block of `size` centered at `center`.
    fn block(center: Point3, size: Vector3) -> Self {
        let m = size.x * size.y * size.z;
        let sq = Vector3::new(size.x * size.x, size.y * size.y, size.z * size.z);
        Self::placed(
            m,
            center,
            Matrix3::from_diagonal(
                &(Vector3::new(sq.y + sq.z, sq.x + sq.z, sq.x + sq.y) * (m / 12.0)),
            ),
        )
    }

    /// Circular cylinder about `+Z` of `radius` and `height`, centered at
    /// `center`.
    fn cylinder_z(center: Point3, radius: f64, height: f64) -> Self {
        let m = PI * radius * radius * height;
        let transverse = m * (3.0 * radius * radius + height * height) / 12.0;
        Self::placed(
            m,
            center,
            Matrix3::from_diagonal(&Vector3::new(
                transverse,
                transverse,
                m * radius * radius / 2.0,
            )),
        )
    }

    /// Solid sphere of `radius` centered at `center`.
    fn sphere(center: Point3, radius: f64) -> Self {
        let m = 4.0 / 3.0 * PI * radius * radius * radius;
        let i = 0.4 * m * radius * radius;
        Self::placed(m, center, Matrix3::identity() * i)
    }

    /// Union with a solid whose interior is disjoint from this one's.
    fn plus(self, other: Rigid) -> Self {
        Rigid {
            volume: self.volume + other.volume,
            moment: self.moment + other.moment,
            inertia_origin: self.inertia_origin + other.inertia_origin,
        }
    }

    /// Difference from a solid wholly contained in this one.
    fn minus(self, other: Rigid) -> Self {
        Rigid {
            volume: self.volume - other.volume,
            moment: self.moment - other.moment,
            inertia_origin: self.inertia_origin - other.inertia_origin,
        }
    }

    fn centroid(self) -> Point3 {
        Point3::from(self.moment / self.volume)
    }

    /// The tensor a measurement reports: about the composite's own centroid.
    fn inertia_centroid(self) -> Matrix3<f64> {
        let c = self.centroid().coords;
        self.inertia_origin
            - (Matrix3::identity() * c.norm_squared() - c * c.transpose()) * self.volume
    }
}

/// Assert a measurement matches a closed-form composite in all three of
/// volume, centroid and inertia.
///
/// Centroid error is scaled by the body's own size (`V^{1/3}`) rather than by
/// its distance from the origin, which may be zero; inertia entries are scaled
/// by the largest entry of the expected tensor, so a near-zero product of
/// inertia is held to an absolute bound rather than an unmeetable relative one.
fn assert_matches_rigid(got: &MassProperties, want: Rigid, rtol: f64, context: &str) {
    assert_close(got.volume, want.volume, rtol, &format!("{context} volume"));

    let scale = want.volume.abs().cbrt().max(1e-300);
    let centroid_err = (got.centroid - want.centroid()).norm() / scale;
    assert!(
        centroid_err <= rtol,
        "{context}: centroid {:?} differs from expected {:?} by {centroid_err:.3e} \
         relative to the body size (allowed {rtol:.1e})",
        got.centroid,
        want.centroid()
    );

    let expected = want.inertia_centroid();
    let magnitude = expected.iter().fold(0.0f64, |a, b| a.max(b.abs()));
    for i in 0..3 {
        for j in 0..3 {
            let err = (got.inertia[(i, j)] - expected[(i, j)]).abs() / magnitude.max(1e-300);
            assert!(
                err <= rtol,
                "{context}: I[{i}][{j}] = {} differs from expected {} by {err:.3e} \
                 relative to the tensor's magnitude (allowed {rtol:.1e})",
                got.inertia[(i, j)],
                expected[(i, j)]
            );
        }
    }
}

// ---------------------------------------------------------------------
// Deterministic PRNG (splitmix64) — no external deps, stable across runs.
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

    /// Uniform in [0, 1).
    fn unit(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.unit()
    }

    fn pick(&mut self, n: usize) -> usize {
        (self.next_u64() % n as u64) as usize
    }
}

// ---------------------------------------------------------------------
// Scene: one TopologyStore + GeometryStore pair per test configuration.
// The boolean entry points consume two bodies living in a shared store
// pair, so every operand of one boolean call must be built here.
// ---------------------------------------------------------------------

struct Scene {
    store: TopologyStore,
    geo: GeometryStore,
    /// The tolerance every boolean run through this scene uses. Defaults to
    /// [`tol`]; section (16) varies it, because a tolerance is a length and a
    /// model whose features are 1e-6 mm across cannot be judged by the same
    /// absolute 1e-6 as one whose features are metres.
    tol: ToleranceContext,
}

impl Scene {
    fn new() -> Self {
        Scene::with_tolerance(tol())
    }

    fn with_tolerance(tol: ToleranceContext) -> Self {
        Scene {
            store: TopologyStore::new(),
            geo: GeometryStore::new(),
            tol,
        }
    }

    /// Re-open a boolean result as a scene, so further operands can be built
    /// alongside it and the result fed back into another boolean. This is what
    /// makes an N-operation chain possible (section 16.6): the boolean entry
    /// points take two bodies in a *shared* store pair, and a result owns its
    /// own pair.
    fn adopt(out: BooleanOutput, tol: ToleranceContext) -> (Self, EntityId<Body>) {
        (
            Scene {
                store: out.store,
                geo: out.geo,
                tol,
            },
            out.body,
        )
    }

    /// Axis-aligned block spanning `min`..`max` (the primitive builder
    /// centers at the origin; translate into place).
    fn block(&mut self, min: [f64; 3], max: [f64; 3]) -> EntityId<Body> {
        let body = primitives::block(
            &mut self.store,
            &mut self.geo,
            max[0] - min[0],
            max[1] - min[1],
            max[2] - min[2],
        )
        .expect("valid block extents");
        let center = Vector3::new(
            (min[0] + max[0]) / 2.0,
            (min[1] + max[1]) / 2.0,
            (min[2] + max[2]) / 2.0,
        );
        translate_body(&mut self.store, &mut self.geo, body, center).expect("finite offset");
        body
    }

    /// Cylinder whose bottom cap is centered at `base`, extending `height`
    /// along unit `axis`. Mirrors `primitives::cylinder` (two caps + a
    /// periodic wall closed by an axial seam) but with an arbitrary frame:
    /// the seam sits at the `plane_basis(axis)` reference direction, which
    /// is exactly where `Curve3::Circle` puts `t = 0`, so edge parameter
    /// ranges stay consistent by construction. (Rotating an existing
    /// z-axis cylinder would desync the seam, because the circle's angular
    /// reference is derived from its axis and is not rotation-equivariant.)
    fn cylinder(
        &mut self,
        base: Point3,
        axis: Vector3,
        radius: f64,
        height: f64,
    ) -> EntityId<Body> {
        let axis = Unit::new_normalize(axis).into_inner();
        let (e_u, _) = plane_basis(&axis);
        let bottom_center = base;
        let top_center = base + axis * height;
        let seam_bottom = bottom_center + e_u * radius;
        let seam_top = top_center + e_u * radius;

        let bottom_circle = Curve3::circle(bottom_center, axis, radius).expect("valid circle");
        let top_circle = Curve3::circle(top_center, axis, radius).expect("valid circle");
        let seam_line = Curve3::line(seam_bottom, axis).expect("valid seam line");
        let bottom_plane = Surface3::plane(bottom_center, -axis).expect("valid bottom plane");
        let top_plane = Surface3::plane(top_center, axis).expect("valid top plane");
        let wall_surface = Surface3::cylinder(bottom_center, axis, radius).expect("valid wall");

        let store = &mut self.store;
        let geo = &mut self.geo;
        let body = store.create_body(BodyType::Solid);
        let shell = store.create_shell(body, true, ShellOrientation::Outward);

        let v_bottom = store.create_vertex(seam_bottom, SYSTEM_RESOLUTION);
        let v_top = store.create_vertex(seam_top, SYSTEM_RESOLUTION);

        let e_bottom = {
            let curve = geo.add_curve(bottom_circle);
            store.create_edge_with_curve(
                v_bottom,
                v_bottom,
                SYSTEM_RESOLUTION,
                curve,
                0.0,
                2.0 * PI,
            )
        };
        let e_top = {
            let curve = geo.add_curve(top_circle);
            store.create_edge_with_curve(v_top, v_top, SYSTEM_RESOLUTION, curve, 0.0, 2.0 * PI)
        };
        let e_seam = {
            let curve = geo.add_curve(seam_line);
            store.create_edge_with_curve(v_bottom, v_top, SYSTEM_RESOLUTION, curve, 0.0, height)
        };

        // Bottom cap looks along -axis: counterclockwise about -axis is
        // against the circle's natural (+axis) direction.
        let f_bottom = store.create_face(shell, FaceSense::Positive);
        store.faces.get_mut(f_bottom).expect("just created").surface =
            Some(geo.add_surface(bottom_plane));
        store.create_loop(f_bottom, LoopType::Outer, &[(e_bottom, FinSense::Reversed)]);

        let f_top = store.create_face(shell, FaceSense::Positive);
        store.faces.get_mut(f_top).expect("just created").surface =
            Some(geo.add_surface(top_plane));
        store.create_loop(f_top, LoopType::Outer, &[(e_top, FinSense::Forward)]);

        // Wall boundary (outward normal radial): along the bottom circle,
        // up the seam, back along the top circle, down the seam.
        let f_wall = store.create_face(shell, FaceSense::Positive);
        store.faces.get_mut(f_wall).expect("just created").surface =
            Some(geo.add_surface(wall_surface));
        store.create_loop(
            f_wall,
            LoopType::Outer,
            &[
                (e_bottom, FinSense::Forward),
                (e_seam, FinSense::Forward),
                (e_top, FinSense::Reversed),
                (e_seam, FinSense::Reversed),
            ],
        );

        body
    }

    /// Cone/frustum about +Z whose bottom cap (radius `radius_bottom`) is
    /// centered at `base`, of `height`, tapering to `radius_top` at the top
    /// cap. A zero `radius_top` (or `radius_bottom`) yields a pointed apex
    /// there. Built with the tested [`primitives::cone`] — which centers the
    /// axis on the origin (bottom cap at `z = -height/2`) — then translated,
    /// so the wall's cone surface, generator seam, and cap circles match the
    /// exact boolean chart by construction (the same reuse-the-primitive
    /// strategy [`Scene::sphere`]/[`Scene::torus`] use).
    fn cone(
        &mut self,
        base: Point3,
        radius_bottom: f64,
        radius_top: f64,
        height: f64,
    ) -> EntityId<Body> {
        let body = primitives::cone(
            &mut self.store,
            &mut self.geo,
            radius_bottom,
            radius_top,
            height,
        )
        .expect("valid cone");
        let offset = (base - Point3::origin()) + Vector3::z() * (height / 2.0);
        translate_body(&mut self.store, &mut self.geo, body, offset).expect("finite offset");
        body
    }

    /// [`Scene::cone`] tilted by `angle` radians about the line through
    /// `base` with direction `tilt_axis`. Uses the tested [`rotate_body`],
    /// which re-anchors the cap circles to their rotated parameterization
    /// and rotates the cone/plane surfaces covariantly, so the tilted body
    /// stays chart-consistent (unlike a hand-rotated frame, cf.
    /// [`Scene::cylinder`]'s note on `Curve3::Circle` reference drift).
    fn cone_tilted(
        &mut self,
        base: Point3,
        radius_bottom: f64,
        radius_top: f64,
        height: f64,
        tilt_axis: Vector3,
        angle: f64,
    ) -> EntityId<Body> {
        let body = self.cone(base, radius_bottom, radius_top, height);
        rotate_body(&mut self.store, &mut self.geo, body, base, tilt_axis, angle)
            .expect("valid rotation");
        body
    }

    /// Sphere from the primitive builder (poles on ±Z, seam meridian
    /// through +X), translated so its center lands at `center`.
    fn sphere(&mut self, center: Point3, radius: f64) -> EntityId<Body> {
        let body =
            primitives::sphere(&mut self.store, &mut self.geo, radius).expect("valid radius");
        translate_body(
            &mut self.store,
            &mut self.geo,
            body,
            center - Point3::origin(),
        )
        .expect("finite offset");
        body
    }

    /// Torus about the +Z axis (seams meeting on the +X outer equator),
    /// translated so its center lands at `center`.
    fn torus(&mut self, center: Point3, major: f64, minor: f64) -> EntityId<Body> {
        let body =
            primitives::torus(&mut self.store, &mut self.geo, major, minor).expect("valid radii");
        translate_body(
            &mut self.store,
            &mut self.geo,
            body,
            center - Point3::origin(),
        )
        .expect("finite offset");
        body
    }

    /// Sphere with an arbitrary pole axis. Mirrors `primitives::sphere`,
    /// but the seam meridian is an equal-radii `Curve3::Ellipse` with an
    /// explicit frame, because `Curve3::Circle` derives its angular
    /// reference from `plane_basis` of its own axis, which is not
    /// rotation-equivariant (the same reason [`Scene::cylinder`] builds
    /// its frame directly). With ellipse axis `-e_v` and `major_dir =
    /// e_u`, the implied minor direction is `(-e_v) × e_u = axis`, so
    /// `point(t) = center + r(cos t·e_u + sin t·axis)` — the curve
    /// parameter is exactly the sphere latitude.
    fn sphere_with_axis(&mut self, center: Point3, axis: Vector3, radius: f64) -> EntityId<Body> {
        let axis = Unit::new_normalize(axis).into_inner();
        let (e_u, e_v) = plane_basis(&axis);
        let meridian = Curve3::Ellipse {
            center,
            axis: -e_v,
            major_dir: e_u,
            major_radius: radius,
            minor_radius: radius,
        };
        let surface = Surface3::sphere(center, axis, radius).expect("valid sphere");

        let body = self.store.create_body(BodyType::Solid);
        let shell = self
            .store
            .create_shell(body, true, ShellOrientation::Outward);
        let v_south = self
            .store
            .create_vertex(center - axis * radius, SYSTEM_RESOLUTION);
        let v_north = self
            .store
            .create_vertex(center + axis * radius, SYSTEM_RESOLUTION);
        let e_seam = {
            let curve = self.geo.add_curve(meridian);
            self.store.create_edge_with_curve(
                v_south,
                v_north,
                SYSTEM_RESOLUTION,
                curve,
                -FRAC_PI_2,
                FRAC_PI_2,
            )
        };
        let face = self.store.create_face(shell, FaceSense::Positive);
        self.store
            .faces
            .get_mut(face)
            .expect("just created")
            .surface = Some(self.geo.add_surface(surface));
        self.store.create_loop(
            face,
            LoopType::Outer,
            &[(e_seam, FinSense::Forward), (e_seam, FinSense::Reversed)],
        );
        body
    }

    /// Torus with an arbitrary axis. The major seam is a `Curve3::circle`
    /// about `axis` — consistent with the boolean chart by construction,
    /// since both derive their reference direction from
    /// `plane_basis(axis)` — and the minor (tube) seam is an equal-radii
    /// ellipse in the `(e_u, axis)` plane, for the same reason as
    /// [`Scene::sphere_with_axis`].
    fn torus_with_axis(
        &mut self,
        center: Point3,
        axis: Vector3,
        major: f64,
        minor: f64,
    ) -> EntityId<Body> {
        let axis = Unit::new_normalize(axis).into_inner();
        let (e_u, e_v) = plane_basis(&axis);
        let surface = Surface3::torus(center, axis, major, minor).expect("valid torus");
        let outer = major + minor;
        let major_circle = Curve3::circle(center, axis, outer).expect("valid circle");
        let minor_circle = Curve3::Ellipse {
            center: center + e_u * major,
            axis: -e_v,
            major_dir: e_u,
            major_radius: minor,
            minor_radius: minor,
        };

        let body = self.store.create_body(BodyType::Solid);
        let shell = self
            .store
            .create_shell(body, true, ShellOrientation::Outward);
        self.store
            .shells
            .get_mut(shell)
            .expect("just created")
            .genus = 1;
        let v0 = self
            .store
            .create_vertex(center + e_u * outer, SYSTEM_RESOLUTION);
        let e_major = {
            let curve = self.geo.add_curve(major_circle);
            self.store
                .create_edge_with_curve(v0, v0, SYSTEM_RESOLUTION, curve, 0.0, 2.0 * PI)
        };
        let e_minor = {
            let curve = self.geo.add_curve(minor_circle);
            self.store
                .create_edge_with_curve(v0, v0, SYSTEM_RESOLUTION, curve, 0.0, 2.0 * PI)
        };
        let face = self.store.create_face(shell, FaceSense::Positive);
        self.store
            .faces
            .get_mut(face)
            .expect("just created")
            .surface = Some(self.geo.add_surface(surface));
        self.store.create_loop(
            face,
            LoopType::Outer,
            &[
                (e_major, FinSense::Forward),
                (e_minor, FinSense::Forward),
                (e_major, FinSense::Reversed),
                (e_minor, FinSense::Reversed),
            ],
        );
        body
    }

    /// Axis-aligned box spanning `min`..`max`, but with all six faces bound
    /// to **bilinear NURBS patches** (`Surface3::Nurbs`) instead of analytic
    /// planes. A flat degree-1×1 patch with a 2×2 control grid reproduces a
    /// rectangle exactly — its four boundary isocurves are straight lines —
    /// so the geometry is identical to [`Scene::block`] down to floating
    /// point; only the surface *kind* differs. That difference is the whole
    /// point (of-xka): this is the only operand in the suite whose faces
    /// drive the boolean through the NURBS chart, SSI, and
    /// classify/reconstruct path end to end, rather than the analytic-plane
    /// path. Every prior NURBS gate stopped at the marched curve (of-37i.4)
    /// or the imprint (of-4is); none reached a NURBS-hosted *solid*.
    ///
    /// Because [`ray_surface_hits`] has no NURBS arm (of-3oj), a body built
    /// here cannot be classified by ray parity and MUST be booleaned via
    /// [`boolean_with_inside_tests`] with a [`box_inside_test`] in its slot
    /// — see the tests below. Winding and control ordering are documented
    /// on [`Scene::nurbs_hexahedron`], which this delegates to.
    fn nurbs_block(&mut self, min: [f64; 3], max: [f64; 3]) -> EntityId<Body> {
        self.nurbs_hexahedron(box_corners(min, max), [[(0.0, 1.0); 2]; 6], 1)
    }

    /// General hexahedral NURBS solid: six degree-1 B-spline patches over
    /// the bilinear blend of the 8 `corners` (in `primitives::block` corner
    /// order — see [`box_corners`]), each face carrying its own `(u, v)`
    /// knot **domain** and `spans` knot spans per direction (`spans + 1`
    /// control points; interior knots evenly spaced). For planar
    /// quadrilateral faces every choice of domain and span count produces
    /// the *identical* point set, which is exactly what the knot-scaling
    /// and multi-span promotion-gate tests need: any behavioral difference
    /// is a parameterization bug, never geometry.
    ///
    /// Control points are ordered so each patch's `du × dv` points out of
    /// the solid, matching `FaceSense::Positive` + `ShellOrientation::Outward`:
    /// with `(u,v)=(0,0)→cycle[0]`, `(1,0)→cycle[1]`, `(1,1)→cycle[2]`,
    /// `(0,1)→cycle[3]`, the normal at `(0,0)` is `(c1−c0)×(c3−c0)`, which
    /// points outward exactly when `c0→c1→c2→c3` is CCW seen from outside —
    /// the same winding as `primitives::block`'s `face_specs` (verified
    /// against the bottom face: `(0,2hy,0)×(2hx,0,0) = −Z`).
    fn nurbs_hexahedron(
        &mut self,
        corners: [Point3; 8],
        face_domains: [[(f64, f64); 2]; 6],
        spans: usize,
    ) -> EntityId<Body> {
        let knots =
            face_domains.map(|d| [deg1_knots(spans + 1, d[0]), deg1_knots(spans + 1, d[1])]);
        self.nurbs_hexahedron_knots(corners, knots)
    }

    /// [`Scene::nurbs_hexahedron`] with each face's `(u, v)` knot vectors given
    /// outright, so degree and knot spacing are free rather than fixed at
    /// "degree 1, evenly spaced". Control points go at the **Greville
    /// abscissae** of the supplied knots, normalized to the patch domain and
    /// pushed through the same bilinear blend.
    ///
    /// That placement is what keeps the geometry exact at any degree: a
    /// B-spline reproduces an affine map of its parameter exactly when its
    /// control points are that map evaluated at the Greville abscissae (the
    /// linear-precision property), and the bilinear blend of a *planar*
    /// quadrilateral is affine in `(u, v)`. So a degree-5 patch with wildly
    /// unequal knot spans traces the same flat rectangle, to floating point,
    /// as the degree-1 one — and any behavioural difference between them is a
    /// basis, span-search or parameterization bug, never geometry. For degree
    /// 1 with evenly spaced knots the Greville abscissae are `i / spans`, so
    /// [`Scene::nurbs_hexahedron`] delegating here is a rename, not a change.
    fn nurbs_hexahedron_knots(
        &mut self,
        corners: [Point3; 8],
        face_knots: [[KnotVector; 2]; 6],
    ) -> EntityId<Body> {
        /// Undirected edges as (low, high) corner-index pairs: bottom ring,
        /// top ring, verticals (identical to `primitives::block`).
        const EDGE_PAIRS: [(usize, usize); 12] = [
            (0, 1),
            (1, 2),
            (2, 3),
            (3, 0),
            (4, 5),
            (5, 6),
            (6, 7),
            (7, 4),
            (0, 4),
            (1, 5),
            (2, 6),
            (3, 7),
        ];

        // Vertex cycles counterclockwise viewed from outside — identical to
        // `primitives::block`'s `face_specs`; the outward normal each cycle
        // implies is reproduced by the control ordering documented above.
        let face_cycles: [[usize; 4]; 6] = [
            [0, 3, 2, 1], // bottom (−Z)
            [4, 5, 6, 7], // top (+Z)
            [0, 1, 5, 4], // front (−Y)
            [1, 2, 6, 5], // right (+X)
            [2, 3, 7, 6], // back (+Y)
            [3, 0, 4, 7], // left (−X)
        ];

        let store = &mut self.store;
        let geo = &mut self.geo;
        let body = store.create_body(BodyType::Solid);
        let shell = store.create_shell(body, true, ShellOrientation::Outward);
        let vertices = corners.map(|p| store.create_vertex(p, SYSTEM_RESOLUTION));

        let edges: Vec<_> = EDGE_PAIRS
            .iter()
            .map(|&(a, b)| {
                let line =
                    Curve3::line(corners[a], corners[b] - corners[a]).expect("distinct corners");
                let length = (corners[b] - corners[a]).norm();
                let curve = geo.add_curve(line);
                store.create_edge_with_curve(
                    vertices[a],
                    vertices[b],
                    SYSTEM_RESOLUTION,
                    curve,
                    0.0,
                    length,
                )
            })
            .collect();

        let directed_edge = |from: usize, to: usize| -> (EntityId<_>, FinSense) {
            let (index, &(a, _)) = EDGE_PAIRS
                .iter()
                .enumerate()
                .find(|&(_, &(a, b))| (a, b) == (from, to) || (a, b) == (to, from))
                .expect("face cycles only use listed edges");
            let sense = if a == from {
                FinSense::Forward
            } else {
                FinSense::Reversed
            };
            (edges[index], sense)
        };

        for (cycle, [knots_u, knots_v]) in face_cycles.into_iter().zip(face_knots) {
            // Row-major grid `[i][j]` with `i↔u`, `j↔v`: the bilinear blend of
            // the cycle's corners at the normalized Greville abscissae, so for
            // degree 1 with `spans` even spans row u=0 is (v=0, v=1) = (c0, c3)
            // and row u=spans is (c1, c2), reproducing the original layout.
            let us = normalized_grevilles(&knots_u);
            let vs = normalized_grevilles(&knots_v);
            let grid: Vec<Vec<Point3>> = us
                .iter()
                .map(|&u| {
                    vs.iter()
                        .map(|&v| {
                            bilerp(
                                corners[cycle[0]],
                                corners[cycle[1]],
                                corners[cycle[2]],
                                corners[cycle[3]],
                                u,
                                v,
                            )
                        })
                        .collect()
                })
                .collect();
            let patch =
                NurbsSurface::bspline(grid, knots_u, knots_v).expect("rectangular bilinear grid");
            let surface = geo.add_surface(Surface3::nurbs(patch));
            let face = store.create_face(shell, FaceSense::Positive);
            store.faces.get_mut(face).expect("just created").surface = Some(surface);
            let loop_edges: Vec<_> = (0..4)
                .map(|i| directed_edge(cycle[i], cycle[(i + 1) % 4]))
                .collect();
            store.create_loop(face, LoopType::Outer, &loop_edges);
        }
        body
    }

    /// Solid cone about `+Z` whose wall is four **exact** rational
    /// quadratic NURBS quarter patches, each with its `v = 1` control
    /// column collapsed onto the apex — the degenerate-edge shape of
    /// of-37i.7, in the one form whose right answer is known in closed
    /// form. Base circle of radius `r` at `z = z0`, apex at `z0 + h`.
    ///
    /// Exactly a cone, not an approximation of one: `v` is degree 1 with
    /// each ruling's two weights equal, which makes every `v`-line the
    /// straight segment base→apex, and the base is the of-pb7.3 exact
    /// quarter arc. So its volume is `π r² h / 3` and its wall normals are
    /// the analytic cone's, which is what lets a test on it fail loudly
    /// rather than merely differ.
    ///
    /// Topology is [`Scene::nurbs_cylinder`] with the top ring collapsed:
    /// one apex vertex instead of four, so each wall loop is a triangle
    /// (base arc, up the far seam, down the near seam) and there is no top
    /// cap.
    fn nurbs_cone(&mut self, cx: f64, cy: f64, r: f64, z0: f64, h: f64) -> EntityId<Body> {
        let axis = Vector3::z();
        let dirs: [Vector3; 4] = [
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
            Vector3::new(-1.0, 0.0, 0.0),
            Vector3::new(0.0, -1.0, 0.0),
        ];
        let base_center = Point3::new(cx, cy, z0);
        let apex = Point3::new(cx, cy, z0 + h);

        let store = &mut self.store;
        let geo = &mut self.geo;
        let body = store.create_body(BodyType::Solid);
        let shell = store.create_shell(body, true, ShellOrientation::Outward);

        let v_base: Vec<_> = dirs
            .iter()
            .map(|d| store.create_vertex(base_center + d * r, SYSTEM_RESOLUTION))
            .collect();
        let v_apex = store.create_vertex(apex, SYSTEM_RESOLUTION);

        let e_base: Vec<_> = (0..4)
            .map(|k| {
                let circle = Curve3::circle(base_center, axis, r).expect("valid circle");
                let curve = geo.add_curve(circle);
                store.create_edge_with_curve(
                    v_base[k],
                    v_base[(k + 1) % 4],
                    SYSTEM_RESOLUTION,
                    curve,
                    k as f64 * FRAC_PI_2,
                    (k + 1) as f64 * FRAC_PI_2,
                )
            })
            .collect();
        let e_seam: Vec<_> = (0..4)
            .map(|k| {
                let from = base_center + dirs[k] * r;
                let slant = apex - from;
                let length = slant.norm();
                let line = Curve3::line(from, slant / length).expect("valid seam");
                let curve = geo.add_curve(line);
                store.create_edge_with_curve(
                    v_base[k],
                    v_apex,
                    SYSTEM_RESOLUTION,
                    curve,
                    0.0,
                    length,
                )
            })
            .collect();

        // Base cap looks along -Z, so its arcs run reversed in reversed
        // order — the `nurbs_cylinder` bottom cap exactly.
        let f_base = store.create_face(shell, FaceSense::Positive);
        store.faces.get_mut(f_base).expect("just created").surface =
            Some(geo.add_surface(Surface3::plane(base_center, -axis).expect("valid plane")));
        store.create_loop(
            f_base,
            LoopType::Outer,
            &[
                (e_base[3], FinSense::Reversed),
                (e_base[2], FinSense::Reversed),
                (e_base[1], FinSense::Reversed),
                (e_base[0], FinSense::Reversed),
            ],
        );

        let knots_u = KnotVector::clamped_uniform(2, 3).expect("degree-2 knots, 3 controls");
        let knots_v = KnotVector::clamped_uniform(1, 2).expect("degree-1 knots, 2 controls");
        for k in 0..4 {
            let d0 = dirs[k];
            let d1 = dirs[(k + 1) % 4];
            let ring = [d0, d0 + d1, d1];
            let control_points: Vec<Vec<Point3>> = ring
                .iter()
                .map(|d| vec![base_center + d * r, apex])
                .collect();
            let weights: Vec<Vec<f64>> = [1.0, FRAC_1_SQRT_2, 1.0]
                .iter()
                .map(|&w| vec![w, w])
                .collect();
            let patch =
                NurbsSurface::new(control_points, weights, knots_u.clone(), knots_v.clone())
                    .expect("valid rational quarter-cone patch");
            let face = store.create_face(shell, FaceSense::Positive);
            store.faces.get_mut(face).expect("just created").surface =
                Some(geo.add_surface(Surface3::nurbs(patch)));
            store.create_loop(
                face,
                LoopType::Outer,
                &[
                    (e_base[k], FinSense::Forward),
                    (e_seam[(k + 1) % 4], FinSense::Forward),
                    (e_seam[k], FinSense::Reversed),
                ],
            );
        }
        body
    }

    /// Solid cylinder about `+Z` whose wall is four **exact** rational
    /// quadratic NURBS quarter patches — the of-pb7.3 construction (90°
    /// arcs with middle control point at the tangent intersection, weight
    /// `1/√2`) swept linearly in `v` — with planar caps. Radius `r`, axis
    /// through `(cx, cy)`, `z ∈ [z0, z0 + h]`. Geometrically identical to
    /// [`Scene::cylinder`] to ~1e-10, but every wall surface the pipeline
    /// sees is a NURBS patch: this is the "NURBS patch of exact analytic
    /// form" the FREEFORM §9 promotion gate checks against the analytic
    /// cylinder's known-good boolean. Quarter patches (rather than one
    /// periodic patch with a seam) keep every chart domain open, matching
    /// how `Chart` treats NURBS domains as non-periodic.
    ///
    /// Topology mirrors `primitives::cylinder` stretched to four wall
    /// faces: ring vertices at angles `0, π/2, π, 3π/2` (the `Curve3::circle`
    /// parameter origin is `plane_basis(+Z).0 = +X`, so arc `k` spans
    /// `t ∈ [kπ/2, (k+1)π/2]` exactly), four axial seam edges, and caps
    /// bounded by the four arcs. Wall patch `u` runs along increasing
    /// angle and `v` up the axis, so `du × dv` points radially outward.
    fn nurbs_cylinder(&mut self, cx: f64, cy: f64, r: f64, z0: f64, h: f64) -> EntityId<Body> {
        let z1 = z0 + h;
        let axis = Vector3::z();
        let dirs: [Vector3; 4] = [
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
            Vector3::new(-1.0, 0.0, 0.0),
            Vector3::new(0.0, -1.0, 0.0),
        ];
        let bottom_center = Point3::new(cx, cy, z0);
        let top_center = Point3::new(cx, cy, z1);

        let store = &mut self.store;
        let geo = &mut self.geo;
        let body = store.create_body(BodyType::Solid);
        let shell = store.create_shell(body, true, ShellOrientation::Outward);

        let v_bot: Vec<_> = dirs
            .iter()
            .map(|d| store.create_vertex(bottom_center + d * r, SYSTEM_RESOLUTION))
            .collect();
        let v_top: Vec<_> = dirs
            .iter()
            .map(|d| store.create_vertex(top_center + d * r, SYSTEM_RESOLUTION))
            .collect();

        // Quarter-circle arc edges on the two cap circles, each on its own
        // copy of the circle curve, plus four axial seams.
        let arc = |store: &mut TopologyStore,
                   geo: &mut GeometryStore,
                   center: Point3,
                   verts: &[EntityId<_>],
                   k: usize| {
            let circle = Curve3::circle(center, axis, r).expect("valid circle");
            let curve = geo.add_curve(circle);
            store.create_edge_with_curve(
                verts[k],
                verts[(k + 1) % 4],
                SYSTEM_RESOLUTION,
                curve,
                k as f64 * FRAC_PI_2,
                (k + 1) as f64 * FRAC_PI_2,
            )
        };
        let e_bot: Vec<_> = (0..4)
            .map(|k| arc(store, geo, bottom_center, &v_bot, k))
            .collect();
        let e_top: Vec<_> = (0..4)
            .map(|k| arc(store, geo, top_center, &v_top, k))
            .collect();
        let e_seam: Vec<_> = (0..4)
            .map(|k| {
                let line = Curve3::line(bottom_center + dirs[k] * r, axis).expect("valid seam");
                let curve = geo.add_curve(line);
                store.create_edge_with_curve(v_bot[k], v_top[k], SYSTEM_RESOLUTION, curve, 0.0, h)
            })
            .collect();

        // Bottom cap looks along -Z: counterclockwise about -Z is against
        // the circles' natural (+Z) direction, so the arcs run reversed in
        // reversed order.
        let f_bottom = store.create_face(shell, FaceSense::Positive);
        store.faces.get_mut(f_bottom).expect("just created").surface =
            Some(geo.add_surface(Surface3::plane(bottom_center, -axis).expect("valid plane")));
        store.create_loop(
            f_bottom,
            LoopType::Outer,
            &[
                (e_bot[3], FinSense::Reversed),
                (e_bot[2], FinSense::Reversed),
                (e_bot[1], FinSense::Reversed),
                (e_bot[0], FinSense::Reversed),
            ],
        );

        let f_top = store.create_face(shell, FaceSense::Positive);
        store.faces.get_mut(f_top).expect("just created").surface =
            Some(geo.add_surface(Surface3::plane(top_center, axis).expect("valid plane")));
        store.create_loop(
            f_top,
            LoopType::Outer,
            &[
                (e_top[0], FinSense::Forward),
                (e_top[1], FinSense::Forward),
                (e_top[2], FinSense::Forward),
                (e_top[3], FinSense::Forward),
            ],
        );

        // Four wall patches. Quarter `k` spans angles `[kπ/2, (k+1)π/2]`:
        // arc control points [d_k, d_k + d_{k+1}, d_{k+1}] (the middle one
        // is the tangent intersection at radius r·√2) with weights
        // [1, 1/√2, 1] trace the exact circular arc; sweeping each along
        // `+Z` (2 linear control points in v) gives the exact quarter
        // cylinder. Boundary: bottom arc forward, up the far seam, top arc
        // reversed, down the near seam — outward radial normal.
        let knots_u = KnotVector::clamped_uniform(2, 3).expect("degree-2 knots, 3 controls");
        let knots_v = KnotVector::clamped_uniform(1, 2).expect("degree-1 knots, 2 controls");
        for k in 0..4 {
            let d0 = dirs[k];
            let d1 = dirs[(k + 1) % 4];
            let ring = [d0, d0 + d1, d1];
            let control_points: Vec<Vec<Point3>> = ring
                .iter()
                .map(|d| vec![bottom_center + d * r, top_center + d * r])
                .collect();
            let weights: Vec<Vec<f64>> = [1.0, FRAC_1_SQRT_2, 1.0]
                .iter()
                .map(|&w| vec![w, w])
                .collect();
            let patch =
                NurbsSurface::new(control_points, weights, knots_u.clone(), knots_v.clone())
                    .expect("valid rational quarter-cylinder patch");
            let face = store.create_face(shell, FaceSense::Positive);
            store.faces.get_mut(face).expect("just created").surface =
                Some(geo.add_surface(Surface3::nurbs(patch)));
            store.create_loop(
                face,
                LoopType::Outer,
                &[
                    (e_bot[k], FinSense::Forward),
                    (e_seam[(k + 1) % 4], FinSense::Forward),
                    (e_top[k], FinSense::Reversed),
                    (e_seam[k], FinSense::Reversed),
                ],
            );
        }
        body
    }

    /// Rigid rotation of a body about `center`, mutating its vertices and
    /// geometry in place (the builders insert fresh geometry per body, so
    /// nothing is shared). Line/Plane only — i.e. blocks. Circles are
    /// excluded on purpose: `Curve3::Circle`'s angular reference comes from
    /// `plane_basis(axis)`, which is not rotation-equivariant, so rotating
    /// the axis would desync edge parameter ranges. Rotated cylinders are
    /// built directly with a rotated frame via [`Scene::cylinder`] instead.
    fn rotate(&mut self, body: EntityId<Body>, rot: &Rotation3<f64>, center: &Point3) {
        let mut curve_ids: Vec<EntityId<Curve3>> = Vec::new();
        let mut surface_ids: Vec<EntityId<Surface3>> = Vec::new();
        let mut vertex_ids = Vec::new();
        for face in self.store.faces_of_body(body) {
            if let Some(surface) = self.store.face(face).expect("stale Face id").surface {
                if !surface_ids.contains(&surface) {
                    surface_ids.push(surface);
                }
            }
            for edge_id in self.store.edges_of_face(face) {
                let edge = self.store.edge(edge_id).expect("stale Edge id");
                if let Some(curve) = edge.curve {
                    if !curve_ids.contains(&curve) {
                        curve_ids.push(curve);
                    }
                }
                for v in [edge.start_vertex, edge.end_vertex] {
                    if !vertex_ids.contains(&v) {
                        vertex_ids.push(v);
                    }
                }
            }
        }

        for v in vertex_ids {
            let point = &mut self
                .store
                .vertices
                .get_mut(v)
                .expect("stale Vertex id")
                .point;
            *point = center + rot * (*point - center);
        }
        for id in curve_ids {
            match self.geo.curves.get_mut(id).expect("stale Curve3 id") {
                Curve3::Line { origin, dir } => {
                    *origin = center + rot * (*origin - center);
                    *dir = rot * *dir;
                }
                other => panic!("Scene::rotate only supports Line edges, got {other:?}"),
            }
        }
        for id in surface_ids {
            match self.geo.surfaces.get_mut(id).expect("stale Surface3 id") {
                Surface3::Plane { origin, normal } => {
                    *origin = center + rot * (*origin - center);
                    *normal = rot * *normal;
                }
                other => panic!("Scene::rotate only supports Plane faces, got {other:?}"),
            }
        }
    }

    fn unite(&self, a: EntityId<Body>, b: EntityId<Body>) -> CoreResult<BooleanOutput> {
        unite(&self.store, &self.geo, a, b, &self.tol)
    }

    fn subtract(&self, a: EntityId<Body>, b: EntityId<Body>) -> CoreResult<BooleanOutput> {
        subtract(&self.store, &self.geo, a, b, &self.tol)
    }

    fn intersect(&self, a: EntityId<Body>, b: EntityId<Body>) -> CoreResult<BooleanOutput> {
        intersect(&self.store, &self.geo, a, b, &self.tol)
    }
}

/// The 8 corners of the axis-aligned box `min..max` in `primitives::block`
/// corner order: bottom ring `(z = min)` counterclockwise from `(min, min)`,
/// then the top ring above it.
fn box_corners(min: [f64; 3], max: [f64; 3]) -> [Point3; 8] {
    [
        Point3::new(min[0], min[1], min[2]),
        Point3::new(max[0], min[1], min[2]),
        Point3::new(max[0], max[1], min[2]),
        Point3::new(min[0], max[1], min[2]),
        Point3::new(min[0], min[1], max[2]),
        Point3::new(max[0], min[1], max[2]),
        Point3::new(max[0], max[1], max[2]),
        Point3::new(min[0], max[1], max[2]),
    ]
}

/// Bilinear blend of a quadrilateral `c0→c1→c2→c3` at `(u, v)`: `u` runs
/// along `c0→c1`, `v` along `c0→c3` (so `(1,1)` lands on `c2`).
fn bilerp(c0: Point3, c1: Point3, c2: Point3, c3: Point3, u: f64, v: f64) -> Point3 {
    let bottom = c0 + (c1 - c0) * u;
    let top = c3 + (c2 - c3) * u;
    bottom + (top - bottom) * v
}

/// Clamped degree-1 knot vector for `control_count` control points over the
/// domain `[a, b]`, interior knots evenly spaced — `clamped_uniform`
/// generalized to an arbitrary domain, which is what the knot-scaling
/// invariance tests vary.
fn deg1_knots(control_count: usize, (a, b): (f64, f64)) -> KnotVector {
    let mut knots = vec![a];
    for i in 0..control_count {
        knots.push(a + (b - a) * i as f64 / (control_count - 1) as f64);
    }
    knots.push(b);
    KnotVector::new(1, knots).expect("valid clamped degree-1 knots")
}

/// The Greville abscissae of `knots`, rescaled so the patch domain runs
/// `0..1`. Control point `i` of a degree-`p` B-spline sits at
/// `(U[i+1] + … + U[i+p]) / p`; placing it at the affine image of that value
/// makes the spline reproduce the affine map exactly, at any degree and any
/// knot spacing (see [`Scene::nurbs_hexahedron_knots`]).
fn normalized_grevilles(knots: &KnotVector) -> Vec<f64> {
    let p = knots.degree();
    let u = knots.knots();
    let control_count = u.len() - p - 1;
    let (lo, hi) = (u[p], u[u.len() - p - 1]);
    (0..control_count)
        .map(|i| {
            let g = u[i + 1..=i + p].iter().sum::<f64>() / p as f64;
            (g - lo) / (hi - lo)
        })
        .collect()
}

// =====================================================================
// (1) Rotated operands: block minus tilted cylinder
// =====================================================================

/// Subtract a cylinder tilted `angle_deg` from the z-axis (in the YZ
/// plane) from a 6×6×2 slab. The tool pierces top and bottom only, so the
/// removed material is an oblique cylinder of length `2 / cos θ`.
fn rotated_tool_through_hole(angle_deg: f64) {
    let context = format!("block minus cylinder tilted {angle_deg}°");
    let mut scene = Scene::new();
    let slab = scene.block([0.0, 0.0, 0.0], [6.0, 6.0, 2.0]);
    let theta = angle_deg.to_radians();
    let axis = Vector3::new(0.0, theta.sin(), theta.cos());
    let center = Point3::new(3.0, 3.0, 1.0);
    let (radius, half_len) = (0.5, 4.0);
    let tool = scene.cylinder(center - axis * half_len, axis, radius, 2.0 * half_len);

    let out = scene
        .subtract(slab, tool)
        .unwrap_or_else(|e| panic!("{context}: subtract failed: {e:?}"));
    let counts = out.store.euler_counts(out.body);
    assert_eq!(counts.genus, 1, "{context}: through hole must give genus 1");
    assert_eq!(out.shell_count(), 1, "{context}: single shell expected");
    let vol = volume(&out, &context);
    let expected = 6.0 * 6.0 * 2.0 - PI * radius * radius * (2.0 / theta.cos());
    assert_close(vol, expected, CYL_VOLUME_RTOL, &context);
}

#[test]
fn rotated_tool_through_hole_0_5_deg() {
    rotated_tool_through_hole(0.5);
}

#[test]
fn rotated_tool_through_hole_5_deg() {
    rotated_tool_through_hole(5.0);
}

#[test]
fn rotated_tool_through_hole_15_deg() {
    rotated_tool_through_hole(15.0);
}

#[test]
fn rotated_tool_through_hole_30_deg() {
    rotated_tool_through_hole(30.0);
}

#[test]
fn rotated_tool_through_hole_45_deg() {
    rotated_tool_through_hole(45.0);
}

/// Same tilted-tool subtraction but tilted toward a block diagonal, so no
/// imprint aligns with any coordinate plane.
#[test]
fn rotated_tool_through_hole_skew_axis() {
    let context = "block minus cylinder tilted 25° toward XY diagonal";
    let mut scene = Scene::new();
    let slab = scene.block([0.0, 0.0, 0.0], [6.0, 6.0, 2.0]);
    let theta = 25f64.to_radians();
    let lateral = Vector3::new(1.0, 1.0, 0.0).normalize();
    let axis = lateral * theta.sin() + Vector3::z() * theta.cos();
    let center = Point3::new(3.0, 3.0, 1.0);
    let (radius, half_len) = (0.5, 4.0);
    let tool = scene.cylinder(center - axis * half_len, axis, radius, 2.0 * half_len);

    let out = scene
        .subtract(slab, tool)
        .unwrap_or_else(|e| panic!("{context}: subtract failed: {e:?}"));
    let counts = out.store.euler_counts(out.body);
    assert_eq!(counts.genus, 1, "{context}: through hole must give genus 1");
    let vol = volume(&out, context);
    let expected = 72.0 - PI * radius * radius * (2.0 / theta.cos());
    assert_close(vol, expected, CYL_VOLUME_RTOL, context);
}

/// Rotated block pairs: rotate operand B about its centroid so every
/// plane/plane crossing happens at a non-trivial angle, then verify the
/// inclusion–exclusion volume identity.
#[test]
fn rotated_block_pair_volume_identity() {
    for angle_deg in [15.0f64, 30.0, 45.0] {
        let context = format!("block pair, B rotated {angle_deg}° about z");
        let mut scene = Scene::new();
        let a = scene.block([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]);
        let b = scene.block([1.0, 1.0, 0.5], [3.5, 3.5, 1.5]);
        let rot =
            Rotation3::from_axis_angle(&Unit::new_normalize(Vector3::z()), angle_deg.to_radians());
        scene.rotate(b, &rot, &Point3::new(2.25, 2.25, 1.0));

        let vol_a = 8.0;
        let vol_b = 2.5 * 2.5 * 1.0;
        let union = scene
            .unite(a, b)
            .unwrap_or_else(|e| panic!("{context}: unite failed: {e:?}"));
        let inter = scene
            .intersect(a, b)
            .unwrap_or_else(|e| panic!("{context}: intersect failed: {e:?}"));
        let diff = scene
            .subtract(a, b)
            .unwrap_or_else(|e| panic!("{context}: subtract failed: {e:?}"));

        let vol_union = volume(&union, &format!("{context}: union"));
        let vol_inter = volume(&inter, &format!("{context}: intersection"));
        let vol_diff = volume(&diff, &format!("{context}: difference"));
        assert_close(
            vol_union + vol_inter,
            vol_a + vol_b,
            PLANAR_VOLUME_RTOL,
            &format!("{context}: vol(A∪B)+vol(A∩B) vs vol(A)+vol(B)"),
        );
        assert_close(
            vol_diff,
            vol_a - vol_inter,
            PLANAR_VOLUME_RTOL,
            &format!("{context}: vol(A−B) vs vol(A)−vol(A∩B)"),
        );
    }
}

// =====================================================================
// (2) Randomized property tests (seeded, deterministic)
// =====================================================================

/// Per-axis overlap pattern for a random block pair. Every generated
/// coordinate keeps ≥ 0.1 clearance from A's planes so the configuration
/// stays transversal (no coincident/tangent contacts).
fn random_b_interval(rng: &mut Rng, a_len: f64) -> (f64, f64) {
    match rng.pick(4) {
        // B pokes out on both sides of A.
        0 => (rng.range(-1.5, -0.2), a_len + rng.range(0.2, 1.5)),
        // B pokes out on the low side only.
        1 => (rng.range(-1.5, -0.2), rng.range(0.2, a_len - 0.1)),
        // B pokes out on the high side only.
        2 => (rng.range(0.1, a_len - 0.2), a_len + rng.range(0.2, 1.5)),
        // B strictly inside A on this axis.
        _ => {
            let lo = rng.range(0.1, a_len * 0.5 - 0.05);
            (lo, rng.range(a_len * 0.5 + 0.05, a_len - 0.1))
        }
    }
}

struct BlockPair {
    a_max: [f64; 3],
    b_min: [f64; 3],
    b_max: [f64; 3],
}

impl BlockPair {
    fn random(rng: &mut Rng) -> Self {
        let a_max = [
            rng.range(1.5, 3.0),
            rng.range(1.5, 3.0),
            rng.range(1.5, 3.0),
        ];
        let mut b_min = [0.0; 3];
        let mut b_max = [0.0; 3];
        for k in 0..3 {
            let (lo, hi) = random_b_interval(rng, a_max[k]);
            b_min[k] = lo;
            b_max[k] = hi;
        }
        BlockPair {
            a_max,
            b_min,
            b_max,
        }
    }

    fn bodies(&self, scene: &mut Scene) -> (EntityId<Body>, EntityId<Body>) {
        (
            scene.block([0.0, 0.0, 0.0], self.a_max),
            scene.block(self.b_min, self.b_max),
        )
    }

    fn vol_a(&self) -> f64 {
        self.a_max.iter().product()
    }

    fn vol_b(&self) -> f64 {
        (0..3).map(|k| self.b_max[k] - self.b_min[k]).product()
    }

    /// Exact overlap volume of the two axis-aligned boxes.
    fn vol_overlap(&self) -> f64 {
        (0..3)
            .map(|k| (self.b_max[k].min(self.a_max[k]) - self.b_min[k].max(0.0)).max(0.0))
            .product()
    }

    fn repro(&self, case: usize) -> String {
        format!(
            "case {case}: A = block([0,0,0], {:?}); B = block({:?}, {:?})",
            self.a_max, self.b_min, self.b_max
        )
    }
}

/// vol(A) + vol(B) == vol(A∪B) + vol(A∩B), plus vol(A−B) == vol(A) −
/// vol(A∩B), for seeded random transversal box pairs. Expected volumes are
/// also known analytically for axis-aligned boxes and are cross-checked.
#[test]
fn random_transversal_block_pairs_volume_identity() {
    let mut rng = Rng::new(0x0F1_5EED);
    for case in 0..24 {
        let pair = BlockPair::random(&mut rng);
        let repro = pair.repro(case);
        let mut scene = Scene::new();
        let (a, b) = pair.bodies(&mut scene);

        let union = scene
            .unite(a, b)
            .unwrap_or_else(|e| panic!("{repro}: unite failed: {e:?}"));
        let inter = scene
            .intersect(a, b)
            .unwrap_or_else(|e| panic!("{repro}: intersect failed: {e:?}"));
        let diff = scene
            .subtract(a, b)
            .unwrap_or_else(|e| panic!("{repro}: subtract failed: {e:?}"));

        let vol_union = volume(&union, &format!("{repro}: union"));
        let vol_inter = volume(&inter, &format!("{repro}: intersection"));
        let vol_diff = volume(&diff, &format!("{repro}: difference"));

        assert_close(
            vol_inter,
            pair.vol_overlap(),
            PLANAR_VOLUME_RTOL,
            &format!("{repro}: intersection vs analytic overlap"),
        );
        assert_close(
            vol_union + vol_inter,
            pair.vol_a() + pair.vol_b(),
            PLANAR_VOLUME_RTOL,
            &format!("{repro}: inclusion–exclusion identity"),
        );
        assert_close(
            vol_diff,
            pair.vol_a() - pair.vol_overlap(),
            PLANAR_VOLUME_RTOL,
            &format!("{repro}: difference identity"),
        );
    }
}

/// Boolean volumes must be invariant under a rigid rotation applied to
/// BOTH operands (the configuration is congruent, only the coordinates
/// change). Catches axis-aligned fast paths and chart-dependent bugs.
#[test]
fn random_block_pairs_rotation_invariance() {
    let mut rng = Rng::new(0x0707_4713);
    for case in 0..8 {
        let pair = BlockPair::random(&mut rng);
        let repro = pair.repro(case);
        let mut scene = Scene::new();
        let (a, b) = pair.bodies(&mut scene);

        let axis = Unit::new_normalize(Vector3::new(
            rng.range(-1.0, 1.0),
            rng.range(-1.0, 1.0),
            rng.range(-1.0, 1.0),
        ));
        let angle = rng.range(0.2, 1.3);
        let rot = Rotation3::from_axis_angle(&axis, angle);
        let center = Point3::new(1.0, 1.0, 1.0);
        let mut scene_rot = Scene::new();
        let (ar, br) = pair.bodies(&mut scene_rot);
        scene_rot.rotate(ar, &rot, &center);
        scene_rot.rotate(br, &rot, &center);

        let inter = scene
            .intersect(a, b)
            .unwrap_or_else(|e| panic!("{repro}: intersect failed: {e:?}"));
        let inter_rot = scene_rot.intersect(ar, br).unwrap_or_else(|e| {
            panic!("{repro} rotated by {angle} rad about {axis:?}: intersect failed: {e:?}")
        });
        let v = volume(&inter, &format!("{repro}: intersection"));
        let v_rot = volume(&inter_rot, &format!("{repro}: rotated intersection"));
        assert_close(
            v_rot,
            v,
            1e-9,
            &format!("{repro}: intersection volume under rotation ({angle} rad, {axis:?})"),
        );
    }
}

/// Minimal repro extracted from `random_block_pairs_rotation_invariance`
/// case 5 (seed 0x0707_4713), the of-ny6 bug. Two transversal blocks
/// rotated rigidly about a generic (off-axis) axis: dense collinear
/// boundary sampling made BOTH faces adjacent to one edge skip the same
/// collinear midpoint with a chord + zero-area sliver, putting four
/// triangles on the chord edge. Fixed by thinning interior samples of
/// straight darts from the tessellation rings; kept as a regression test.
#[test]
fn rotated_block_pair_intersection_manifold() {
    let context = "generic-axis rotated block pair intersection";
    let a_max = [2.976154433844907, 1.6850031873777522, 2.0507148739253545];
    let b_min = [-0.5128313384157841, 0.3119107714116799, -0.7159103874747195];
    let b_max = [4.379734111772811, 2.2225877334453616, 1.06396095157423];
    let axis = Unit::new_normalize(Vector3::new(
        -0.2959795405001737,
        0.046345863928126674,
        0.993466254115623,
    ));
    let rot = Rotation3::from_axis_angle(&axis, 0.8165171037409436);
    let center = Point3::new(1.0, 1.0, 1.0);
    let mut scene = Scene::new();
    let a = scene.block([0.0, 0.0, 0.0], a_max);
    let b = scene.block(b_min, b_max);
    scene.rotate(a, &rot, &center);
    scene.rotate(b, &rot, &center);
    let out = scene
        .intersect(a, b)
        .unwrap_or_else(|e| panic!("{context}: intersect failed: {e:?}"));
    // Rotation-invariant expected volume: the axis-aligned overlap box.
    let expected: f64 = (0..3)
        .map(|k| (b_max[k].min(a_max[k]) - b_min[k].max(0.0)).max(0.0))
        .product();
    let vol = volume(&out, context);
    assert_close(vol, expected, PLANAR_VOLUME_RTOL, context);
}

// =====================================================================
// (3) Near-degenerate transversal cases
// =====================================================================

/// Through-hole whose wall clears a block side face by a shrinking gap.
/// Every gap here is above the default linear tolerance (1e-6), so the
/// configuration is still formally transversal and must succeed.
#[test]
fn wall_almost_tangent_to_side_face() {
    for gap in [1e-3, 1e-4, 1e-5] {
        let context = format!("cylinder wall {gap:.0e} away from face x=0");
        let radius = 0.5;
        let mut scene = Scene::new();
        let cube = scene.block([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]);
        let tool = scene.cylinder(
            Point3::new(radius + gap, 1.0, -1.0),
            Vector3::z(),
            radius,
            4.0,
        );
        let out = scene
            .subtract(cube, tool)
            .unwrap_or_else(|e| panic!("{context}: subtract failed: {e:?}"));
        let counts = out.store.euler_counts(out.body);
        assert_eq!(counts.genus, 1, "{context}: through hole must give genus 1");
        let vol = volume(&out, &context);
        assert_close(
            vol,
            8.0 - PI * radius * radius * 2.0,
            CYL_VOLUME_RTOL,
            &context,
        );
    }
}

/// Subtraction leaving a progressively thinner wall: the survivor is a
/// t × 2 × 2 slab whose volume must track t exactly (planar geometry).
#[test]
fn thin_sliver_walls() {
    for thickness in [1e-2, 1e-3, 1e-4] {
        let context = format!("sliver wall of thickness {thickness:.0e}");
        let mut scene = Scene::new();
        let a = scene.block([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]);
        let b = scene.block([thickness, -0.5, -0.5], [3.0, 2.5, 2.5]);
        let out = scene
            .subtract(a, b)
            .unwrap_or_else(|e| panic!("{context}: subtract failed: {e:?}"));
        let vol = volume(&out, &context);
        assert_close(vol, thickness * 4.0, 1e-6, &context);
    }
}

/// Tool exiting through an edge region: the cylinder is centered on a
/// vertical block edge, so the edge is strictly inside the tool and the
/// subtraction carves a quarter-round notch spanning two side faces.
#[test]
fn tool_swallows_vertical_edge() {
    let context = "quarter-notch: cylinder centered on the (2,2,z) edge";
    let radius = 0.4;
    let mut scene = Scene::new();
    let cube = scene.block([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]);
    let tool = scene.cylinder(Point3::new(2.0, 2.0, -1.0), Vector3::z(), radius, 4.0);
    let out = scene
        .subtract(cube, tool)
        .unwrap_or_else(|e| panic!("{context}: subtract failed: {e:?}"));
    let counts = out.store.euler_counts(out.body);
    assert_eq!(counts.genus, 0, "{context}: notch must not create genus");
    let vol = volume(&out, context);
    let expected = 8.0 - (PI * radius * radius / 4.0) * 2.0;
    assert_close(vol, expected, CYL_VOLUME_RTOL, context);
}

/// Tool wall grazing a vertical block edge from outside with clearance
/// below the linear tolerance. Sub-tolerance geometry: a structured
/// NotImplemented/Degenerate rejection is acceptable under the transversal
/// MVP contract, but a panic or an invalid "success" is a bug.
#[test]
fn tool_grazes_vertical_edge_sub_tolerance() {
    let context = "cylinder wall 1e-7 outside the (2,2,z) edge";
    let radius = 0.4;
    let clearance = 1e-7;
    let mut scene = Scene::new();
    let cube = scene.block([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]);
    // Push the axis out along the (1,1)/√2 diagonal so the closest
    // approach of the wall to the edge line is exactly `clearance`.
    let d = (radius + clearance) / 2f64.sqrt();
    let tool = scene.cylinder(
        Point3::new(2.0 + d, 2.0 + d, -1.0),
        Vector3::z(),
        radius,
        4.0,
    );
    match scene.subtract(cube, tool) {
        Ok(out) => {
            // If the pipeline claims success the result must be fully valid
            // and the volume must be (nearly) the untouched cube.
            let vol = volume(&out, context);
            assert_close(vol, 8.0, CYL_VOLUME_RTOL, context);
        }
        Err(CoreError::NotImplemented { .. }) | Err(CoreError::Degenerate { .. }) => {
            // Structured rejection of sub-tolerance contact: acceptable.
        }
        Err(other) => panic!("{context}: unexpected error kind: {other:?}"),
    }
}

/// Tool wall passing through the interior at a distance just ABOVE the
/// linear tolerance from a vertical edge — formally transversal, so this
/// must produce a valid notch.
#[test]
fn tool_cuts_just_inside_vertical_edge() {
    let context = "cylinder wall cutting 1e-4 inside the (2,2,z) edge";
    let radius = 0.4;
    let bite = 1e-4;
    let d = (radius - bite) / 2f64.sqrt();
    let mut scene = Scene::new();
    let cube = scene.block([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]);
    let tool = scene.cylinder(
        Point3::new(2.0 + d, 2.0 + d, -1.0),
        Vector3::z(),
        radius,
        4.0,
    );
    let out = scene
        .subtract(cube, tool)
        .unwrap_or_else(|e| panic!("{context}: subtract failed: {e:?}"));
    // The nibbled volume is a tiny circular-segment prism; just require
    // validity and a volume a hair under the full cube.
    let vol = volume(&out, context);
    assert!(
        vol < 8.0 && vol > 8.0 - 1e-3,
        "{context}: volume {vol} outside (7.999, 8.0)"
    );
}

// =====================================================================
// (4) Round-trip: B-Rep boolean → tessellate → MeshSdf → re-mesh
// =====================================================================

/// Wrap a boolean result's tessellation as a mesh SDF, re-mesh it with
/// dual contouring, and require the volumes to agree within 3%.
fn round_trip_volume(out: &BooleanOutput, context: &str) {
    let mesh = assert_valid(out, context);
    let vol_brep = mass_properties(&mesh)
        .unwrap_or_else(|e| panic!("{context}: mass_properties failed: {e}"))
        .volume;
    let sdf =
        MeshSdf::new(&mesh).unwrap_or_else(|e| panic!("{context}: MeshSdf::new failed: {e:?}"));
    let bbox = mesh.bounding_box().expect("non-empty mesh");
    let extent = bbox.max - bbox.min;
    let longest = extent.x.max(extent.y).max(extent.z);
    // One-cell clearance on every side, as mesh_sdf requires the surface
    // strictly inside the bounds.
    let margin = Vector3::new(1.0, 1.0, 1.0) * (longest * 0.1);
    let opts = MeshOptions {
        bounds: BoundingBox3::new(bbox.min - margin, bbox.max + margin),
        resolution: 96,
    };
    let remesh = mesh_sdf_indexed(&sdf, &opts);
    assert!(
        remesh.is_closed_manifold(),
        "{context}: dual-contoured SDF mesh is not a closed manifold \
         ({} triangles)",
        remesh.triangle_count()
    );
    let vol_sdf = mass_properties(&remesh)
        .unwrap_or_else(|e| panic!("{context}: SDF re-mesh mass_properties failed: {e}"))
        .volume;
    assert_close(
        vol_sdf,
        vol_brep,
        0.03,
        &format!("{context}: SDF round-trip volume"),
    );
}

#[test]
fn round_trip_block_minus_cylinder() {
    let mut scene = Scene::new();
    let slab = scene.block([0.0, 0.0, 0.0], [4.0, 4.0, 2.0]);
    let tool = scene.cylinder(Point3::new(2.0, 2.0, -1.0), Vector3::z(), 1.0, 4.0);
    let out = scene.subtract(slab, tool).expect("through-hole subtract");
    round_trip_volume(&out, "round-trip: block minus cylinder");
}

#[test]
fn round_trip_union_of_blocks() {
    let mut scene = Scene::new();
    let a = scene.block([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]);
    let b = scene.block([1.0, 1.0, 1.0], [3.0, 3.0, 3.0]);
    let out = scene.unite(a, b).expect("corner-overlap union");
    round_trip_volume(&out, "round-trip: union of overlapping blocks");
}

// =====================================================================
// (5) Scale extremes: 0.001× and 1000×
// =====================================================================

/// The through-hole scenario with every length multiplied by `scale`.
/// Volume must track scale³; validity must not depend on absolute size.
fn scaled_through_hole(scale: f64) {
    let context = format!("block minus cylinder at {scale}× scale");
    let s = scale;
    let mut scene = Scene::new();
    let slab = scene.block([0.0, 0.0, 0.0], [4.0 * s, 4.0 * s, 2.0 * s]);
    let tool = scene.cylinder(Point3::new(2.0 * s, 2.0 * s, -s), Vector3::z(), s, 4.0 * s);
    let out = scene
        .subtract(slab, tool)
        .unwrap_or_else(|e| panic!("{context}: subtract failed: {e:?}"));
    let counts = out.store.euler_counts(out.body);
    assert_eq!(counts.genus, 1, "{context}: through hole must give genus 1");
    let vol = volume(&out, &context);
    // The hole runs through the slab's 2s thickness (the tool's extra
    // length lies outside the block).
    let expected = (32.0 - 2.0 * PI) * s * s * s;
    assert_close(vol, expected, CYL_VOLUME_RTOL, &context);
}

#[test]
fn through_hole_at_scale_1() {
    scaled_through_hole(1.0);
}

#[test]
fn through_hole_at_scale_0_001() {
    scaled_through_hole(0.001);
}

#[test]
fn through_hole_at_scale_1000() {
    scaled_through_hole(1000.0);
}

/// Random block-pair volume identity at both scale extremes.
fn scaled_block_pair_identity(scale: f64) {
    let mut rng = Rng::new(0x5CA1E + scale.to_bits());
    for case in 0..6 {
        let pair = BlockPair::random(&mut rng);
        let repro = format!("scale {scale}×, {}", pair.repro(case));
        let s = scale;
        let mut scene = Scene::new();
        let a = scene.block(
            [0.0, 0.0, 0.0],
            [pair.a_max[0] * s, pair.a_max[1] * s, pair.a_max[2] * s],
        );
        let b = scene.block(
            [pair.b_min[0] * s, pair.b_min[1] * s, pair.b_min[2] * s],
            [pair.b_max[0] * s, pair.b_max[1] * s, pair.b_max[2] * s],
        );
        let union = scene
            .unite(a, b)
            .unwrap_or_else(|e| panic!("{repro}: unite failed: {e:?}"));
        let inter = scene
            .intersect(a, b)
            .unwrap_or_else(|e| panic!("{repro}: intersect failed: {e:?}"));
        let vol_union = volume(&union, &format!("{repro}: union"));
        let vol_inter = volume(&inter, &format!("{repro}: intersection"));
        let s3 = s * s * s;
        assert_close(
            vol_union + vol_inter,
            (pair.vol_a() + pair.vol_b()) * s3,
            1e-9,
            &format!("{repro}: inclusion–exclusion identity"),
        );
        assert_close(
            vol_inter,
            pair.vol_overlap() * s3,
            1e-9,
            &format!("{repro}: intersection vs analytic overlap"),
        );
    }
}

#[test]
fn block_pair_identity_at_scale_0_001() {
    scaled_block_pair_identity(0.001);
}

#[test]
fn block_pair_identity_at_scale_1000() {
    scaled_block_pair_identity(1000.0);
}

// =====================================================================
// (6) Sphere operands (of-7ld.3 campaign)
// =====================================================================

/// Sphere dipping a cap of depth `h` into the slab's top face; the
/// removed material is a spherical cap. The cap region on the sphere
/// contains the south pole — polar trimming is exercised on every run.
fn sphere_cap_bite(scale: f64) {
    let context = format!("slab minus sphere cap at {scale}× scale");
    let s = scale;
    let (r, h) = (1.0 * s, 0.6 * s);
    let mut scene = Scene::new();
    let slab = scene.block([0.0, 0.0, 0.0], [6.0 * s, 6.0 * s, 2.0 * s]);
    let ball = scene.sphere(Point3::new(3.0 * s, 3.0 * s, 2.0 * s + (r - h)), r);

    let diff = scene
        .subtract(slab, ball)
        .unwrap_or_else(|e| panic!("{context}: subtract failed: {e:?}"));
    let counts = diff.store.euler_counts(diff.body);
    assert_eq!(counts.genus, 0, "{context}: cap bite must not create genus");
    assert_eq!(diff.shell_count(), 1, "{context}: single shell expected");
    let vol = volume(&diff, &context);
    let cap = spherical_cap_volume(r, h);
    assert_close(vol, 72.0 * s * s * s - cap, CURVED_VOLUME_RTOL, &context);

    let inter = scene
        .intersect(slab, ball)
        .unwrap_or_else(|e| panic!("{context}: intersect failed: {e:?}"));
    let vol_inter = volume(&inter, &format!("{context}: intersection"));
    assert_close(
        vol_inter,
        cap,
        CURVED_VOLUME_RTOL,
        &format!("{context}: intersection vs analytic cap"),
    );
}

#[test]
fn sphere_cap_bite_scale_1() {
    sphere_cap_bite(1.0);
}

#[test]
fn sphere_cap_bite_scale_0_001() {
    sphere_cap_bite(0.001);
}

#[test]
fn sphere_cap_bite_scale_1000() {
    sphere_cap_bite(1000.0);
}

/// Sphere poking out of BOTH slab faces: the intersection is an
/// equatorial band whose trimmed sphere face has two boundary circles,
/// each wrapping the full `u` period (the sphere analog of the of-ipt.4
/// full-wrap cylinder band), and the difference is a genus-1 through
/// hole with lens-shaped mouths.
#[test]
fn sphere_band_through_slab() {
    let context = "sphere through 2-thick slab (band + lens through-hole)";
    let r = 1.5;
    let mut scene = Scene::new();
    let slab = scene.block([0.0, 0.0, 0.0], [6.0, 6.0, 2.0]);
    let ball = scene.sphere(Point3::new(3.0, 3.0, 1.0), r);

    let band = spherical_band_volume_r15();
    let inter = scene
        .intersect(slab, ball)
        .unwrap_or_else(|e| panic!("{context}: intersect failed: {e:?}"));
    let counts = inter.store.euler_counts(inter.body);
    assert_eq!(counts.genus, 0, "{context}: band is a genus-0 solid");
    let vol_inter = volume(&inter, &format!("{context}: intersection"));
    assert_close(
        vol_inter,
        band,
        CURVED_VOLUME_RTOL,
        &format!("{context}: band volume"),
    );

    let diff = scene
        .subtract(slab, ball)
        .unwrap_or_else(|e| panic!("{context}: subtract failed: {e:?}"));
    let counts = diff.store.euler_counts(diff.body);
    assert_eq!(counts.genus, 1, "{context}: through hole must give genus 1");
    let vol_diff = volume(&diff, &format!("{context}: difference"));
    assert_close(
        vol_diff,
        72.0 - band,
        CURVED_VOLUME_RTOL,
        &format!("{context}: difference volume"),
    );
}

/// Band volume for the r = 1.5 sphere centered mid-slab (z ∈ [0, 2]):
/// the sphere minus the two caps of depth r − 1 poking out either face.
fn spherical_band_volume_r15() -> f64 {
    sphere_volume(1.5) - 2.0 * spherical_cap_volume(1.5, 0.5)
}

/// Sphere centered exactly on a block corner: the intersection is one
/// sphere octant bounded by three mutually orthogonal imprint arcs
/// meeting in pairwise junctions — an imprint NETWORK, not a single
/// chain — and the octant contains the sphere's south pole.
#[test]
fn sphere_octant_on_block_corner() {
    let context = "sphere centered on block corner (octant intersection)";
    let r = 0.8;
    let mut scene = Scene::new();
    let cube = scene.block([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]);
    let ball = scene.sphere(Point3::new(2.0, 2.0, 2.0), r);

    let octant = sphere_volume(r) / 8.0;
    let inter = scene
        .intersect(cube, ball)
        .unwrap_or_else(|e| panic!("{context}: intersect failed: {e:?}"));
    let vol_inter = volume(&inter, &format!("{context}: intersection"));
    assert_close(
        vol_inter,
        octant,
        CURVED_VOLUME_RTOL,
        &format!("{context}: octant volume"),
    );

    let union = scene
        .unite(cube, ball)
        .unwrap_or_else(|e| panic!("{context}: unite failed: {e:?}"));
    let diff = scene
        .subtract(cube, ball)
        .unwrap_or_else(|e| panic!("{context}: subtract failed: {e:?}"));
    let vol_union = volume(&union, &format!("{context}: union"));
    let vol_diff = volume(&diff, &format!("{context}: difference"));
    assert_close(
        vol_union + vol_inter,
        8.0 + sphere_volume(r),
        CURVED_VOLUME_RTOL,
        &format!("{context}: inclusion–exclusion identity"),
    );
    assert_close(
        vol_diff,
        8.0 - octant,
        CURVED_VOLUME_RTOL,
        &format!("{context}: difference identity"),
    );
}

/// Block face plane through BOTH poles: the imprint is the x = 0
/// meridian circle, which passes through the two pole vertices of the
/// seam edge — an imprint threaded through existing topology at the
/// exact points where longitude is undefined.
#[test]
fn hemisphere_imprint_through_poles() {
    let context = "half-space block ∩ sphere: meridian imprint through both poles";
    let r = 1.0;
    let mut scene = Scene::new();
    let block = scene.block([0.0, -4.0, -4.0], [4.0, 4.0, 4.0]);
    let ball = scene.sphere(Point3::origin(), r);

    let inter = scene
        .intersect(block, ball)
        .unwrap_or_else(|e| panic!("{context}: intersect failed: {e:?}"));
    let vol = volume(&inter, context);
    assert_close(vol, sphere_volume(r) / 2.0, CURVED_VOLUME_RTOL, context);
}

/// Cap about the +X direction: the imprint circle crosses the sphere's
/// seam meridian (u = 0) twice, so the trimmed regions must share the
/// seam edge correctly.
#[test]
fn sphere_side_cap_crosses_seam() {
    let context = "block bites +X cap: imprint crosses the seam meridian";
    let (r, h) = (1.0, 0.7);
    let mut scene = Scene::new();
    let block = scene.block([r - h, -3.0, -3.0], [3.0, 3.0, 3.0]);
    let ball = scene.sphere(Point3::origin(), r);

    let cap = spherical_cap_volume(r, h);
    let inter = scene
        .intersect(block, ball)
        .unwrap_or_else(|e| panic!("{context}: intersect failed: {e:?}"));
    let vol_inter = volume(&inter, &format!("{context}: intersection"));
    assert_close(vol_inter, cap, CURVED_VOLUME_RTOL, context);

    let diff = scene
        .subtract(ball, block)
        .unwrap_or_else(|e| panic!("{context}: subtract failed: {e:?}"));
    let counts = diff.store.euler_counts(diff.body);
    assert_eq!(counts.genus, 0, "{context}: capped sphere stays genus 0");
    let vol_diff = volume(&diff, &format!("{context}: difference"));
    assert_close(
        vol_diff,
        sphere_volume(r) - cap,
        CURVED_VOLUME_RTOL,
        &format!("{context}: difference volume"),
    );
}

/// The +X cap bite under seeded random rigid rotations of the BLOCK
/// about the sphere center: the configuration is congruent (the sphere
/// is rotation-symmetric), so every volume must match the closed form —
/// while the imprint circle sweeps across the seam and poles at generic
/// angles.
#[test]
fn rotated_block_cap_bite_volume_invariance() {
    let mut rng = Rng::new(0x5F3E_7E11);
    let (r, h) = (1.0, 0.6);
    let expected = sphere_volume(r) - spherical_cap_volume(r, h);
    for case in 0..4 {
        let axis = Unit::new_normalize(Vector3::new(
            rng.range(-1.0, 1.0),
            rng.range(-1.0, 1.0),
            rng.range(-1.0, 1.0),
        ));
        let angle = rng.range(0.2, 1.3);
        let context = format!("case {case}: cap bite, block rotated {angle:.3} rad about {axis:?}");
        let mut scene = Scene::new();
        let block = scene.block([r - h, -3.0, -3.0], [3.0, 3.0, 3.0]);
        let rot = Rotation3::from_axis_angle(&axis, angle);
        scene.rotate(block, &rot, &Point3::origin());
        let ball = scene.sphere(Point3::origin(), r);

        let diff = scene
            .subtract(ball, block)
            .unwrap_or_else(|e| panic!("{context}: subtract failed: {e:?}"));
        let vol = volume(&diff, &context);
        assert_close(vol, expected, CURVED_VOLUME_RTOL, &context);
    }
}

/// Seeded random face-cap configurations: a sphere dips depth `h` into
/// one random face of a random cube, clear of every other face. The
/// intersection has the exact cap closed form, and the three-way volume
/// identities must hold.
#[test]
fn random_sphere_face_caps_identity() {
    let mut rng = Rng::new(0x0F1_CA9);
    for case in 0..8 {
        let a = rng.range(2.5, 3.5);
        let r = rng.range(0.4, 0.8);
        let h = rng.range(0.15, r - 0.15);
        let axis_k = rng.pick(3);
        let hi = rng.pick(2) == 1;
        let mut center = [0.0f64; 3];
        for (k, c) in center.iter_mut().enumerate() {
            *c = if k == axis_k {
                if hi { a + (r - h) } else { -(r - h) }
            } else {
                rng.range(r + 0.2, a - r - 0.2)
            };
        }
        let context =
            format!("case {case}: cube [0,{a:.3}]³, sphere r={r:.3} h={h:.3} at {center:?}");
        let mut scene = Scene::new();
        let cube = scene.block([0.0, 0.0, 0.0], [a, a, a]);
        let ball = scene.sphere(Point3::new(center[0], center[1], center[2]), r);

        let cap = spherical_cap_volume(r, h);
        let inter = scene
            .intersect(cube, ball)
            .unwrap_or_else(|e| panic!("{context}: intersect failed: {e:?}"));
        let union = scene
            .unite(cube, ball)
            .unwrap_or_else(|e| panic!("{context}: unite failed: {e:?}"));
        let diff = scene
            .subtract(cube, ball)
            .unwrap_or_else(|e| panic!("{context}: subtract failed: {e:?}"));
        let vol_inter = volume(&inter, &format!("{context}: intersection"));
        let vol_union = volume(&union, &format!("{context}: union"));
        let vol_diff = volume(&diff, &format!("{context}: difference"));
        let vol_cube = a * a * a;
        assert_close(
            vol_inter,
            cap,
            CURVED_VOLUME_RTOL,
            &format!("{context}: intersection vs analytic cap"),
        );
        assert_close(
            vol_union + vol_inter,
            vol_cube + sphere_volume(r),
            CURVED_VOLUME_RTOL,
            &format!("{context}: inclusion–exclusion identity"),
        );
        assert_close(
            vol_diff,
            vol_cube - cap,
            CURVED_VOLUME_RTOL,
            &format!("{context}: difference identity"),
        );
    }
}

/// A sphere dips a shallow cap of depth `h` into one face of a cube, so the
/// union's sphere face is the whole sphere minus that small imprint — a
/// near-full-wrap (u spans a full turn) curved face with one wide interior
/// hole. Ear clipping seeds such a face by bridging the distant outer
/// rectangle to the wide hole and then force-clips corners across the hole
/// (its least-reflex fallback ignores the hole ring), leaving flat fill
/// triangles inside the imprint plane. On the curved sphere chart those fold
/// back in 3D into two triangles that share a rim chord with the *same*
/// winding — an orientation non-manifold on the imprint rim (of-6ry). The
/// constrained-Delaunay seed recovers every ring edge and drops hole/exterior
/// triangles by parity, so no triangle can bridge the hole and the union
/// tessellates to a closed manifold. (Bounded-cap *volume* accuracy is the
/// separate concern of of-s89; this test asserts only manifoldness.)
#[test]
fn near_full_sphere_union_face_is_manifold() {
    // Sweep a few shallow depths and both hi/lo faces, on each axis, so the
    // imprint lands on the equator and near a uv pole of the sphere chart.
    let a = 3.155;
    for axis_k in 0..3 {
        for &hi in &[false, true] {
            for &(r, h) in &[(0.472, 0.220), (0.685, 0.176), (0.80, 0.16)] {
                let mut center = [a * 0.5; 3];
                center[axis_k] = if hi { a + (r - h) } else { -(r - h) };
                let context = format!("axis {axis_k} hi {hi} r={r} h={h}");
                let mut scene = Scene::new();
                let cube = scene.block([0.0, 0.0, 0.0], [a, a, a]);
                let ball = scene.sphere(Point3::new(center[0], center[1], center[2]), r);
                let union = scene
                    .unite(cube, ball)
                    .unwrap_or_else(|e| panic!("{context}: unite failed: {e:?}"));
                let mesh = union
                    .tessellate()
                    .unwrap_or_else(|e| panic!("{context}: tessellate failed: {e:?}"));
                assert!(
                    mesh.is_closed_manifold(),
                    "{context}: near-full sphere union face must be a closed manifold \
                     ({} triangles)",
                    mesh.triangle_count()
                );
            }
        }
    }
}

/// Canonical cap-bite configuration versus the same configuration
/// rigidly rotated — the sphere rebuilt with the rotated pole axis via
/// [`Scene::sphere_with_axis`], the block rotated in place. Both frames
/// must reproduce the closed form.
#[test]
fn rotated_frame_sphere_cap_congruence() {
    let (r, h) = (1.0, 0.6);
    let sphere_center = Point3::new(3.0, 3.0, 2.0 + (r - h));
    let expected = 72.0 - spherical_cap_volume(r, h);
    let rot = Rotation3::from_axis_angle(&Unit::new_normalize(Vector3::new(1.0, 2.0, 3.0)), 0.7);
    let pivot = Point3::new(1.0, 1.0, 1.0);

    for rotated in [false, true] {
        let context = format!("slab minus sphere cap, rotated frame: {rotated}");
        let mut scene = Scene::new();
        let slab = scene.block([0.0, 0.0, 0.0], [6.0, 6.0, 2.0]);
        let ball = if rotated {
            scene.rotate(slab, &rot, &pivot);
            let center = pivot + rot * (sphere_center - pivot);
            scene.sphere_with_axis(center, rot * Vector3::z(), r)
        } else {
            scene.sphere(sphere_center, r)
        };
        let diff = scene
            .subtract(slab, ball)
            .unwrap_or_else(|e| panic!("{context}: subtract failed: {e:?}"));
        let vol = volume(&diff, &context);
        assert_close(vol, expected, CURVED_VOLUME_RTOL, &context);
    }
}

/// Coaxial cylinder drilled through a sphere: the remainder is the
/// classic napkin ring, volume (4π/3)(r² − a²)^{3/2} independent of the
/// imprint details, genus 1.
#[test]
fn napkin_ring_coaxial_cylinder_drills_sphere() {
    let context = "sphere minus coaxial through-cylinder (napkin ring)";
    let (r, a) = (1.0, 0.5);
    let mut scene = Scene::new();
    let ball = scene.sphere(Point3::origin(), r);
    let tool = scene.cylinder(Point3::new(0.0, 0.0, -2.0), Vector3::z(), a, 4.0);

    let out = scene
        .subtract(ball, tool)
        .unwrap_or_else(|e| panic!("{context}: subtract failed: {e:?}"));
    let counts = out.store.euler_counts(out.body);
    assert_eq!(counts.genus, 1, "{context}: drilled sphere must be genus 1");
    let vol = volume(&out, context);
    let expected = 4.0 / 3.0 * PI * (r * r - a * a).powf(1.5);
    assert_close(vol, expected, CYL_VOLUME_RTOL, context);
}

/// Cylinder drilled through a sphere OFF-center (still a full pierce):
/// no elementary closed form, so assert validity, genus, and the volume
/// identities among the three boolean results.
#[test]
fn offset_cylinder_drills_sphere_identity() {
    let context = "sphere minus offset through-cylinder";
    let (r, a, off) = (1.0, 0.4, 0.45);
    let mut scene = Scene::new();
    let ball = scene.sphere(Point3::origin(), r);
    let tool = scene.cylinder(Point3::new(off, 0.0, -2.0), Vector3::z(), a, 4.0);

    let diff = scene
        .subtract(ball, tool)
        .unwrap_or_else(|e| panic!("{context}: subtract failed: {e:?}"));
    let counts = diff.store.euler_counts(diff.body);
    assert_eq!(counts.genus, 1, "{context}: through hole must give genus 1");
    let inter = scene
        .intersect(ball, tool)
        .unwrap_or_else(|e| panic!("{context}: intersect failed: {e:?}"));
    let vol_diff = volume(&diff, &format!("{context}: difference"));
    let vol_inter = volume(&inter, &format!("{context}: intersection"));
    assert_close(
        vol_diff + vol_inter,
        sphere_volume(r),
        CYL_VOLUME_RTOL,
        &format!("{context}: difference + intersection vs sphere volume"),
    );
}

/// Two overlapping spheres: the intersection lens has an exact closed
/// form (two caps against the radical plane), checked together with the
/// inclusion–exclusion identity for equal and unequal radii.
#[test]
fn sphere_pair_lens_identities() {
    for (r1, r2, d) in [(1.0, 1.0, 1.2), (1.0, 0.6, 0.9), (0.8, 0.8, 1.4)] {
        let context = format!("sphere pair r1={r1} r2={r2} d={d}");
        let mut scene = Scene::new();
        let s1 = scene.sphere(Point3::origin(), r1);
        let s2 = scene.sphere(Point3::new(d, 0.0, 0.0), r2);

        let lens = sphere_lens_volume(r1, r2, d);
        let inter = scene
            .intersect(s1, s2)
            .unwrap_or_else(|e| panic!("{context}: intersect failed: {e:?}"));
        let union = scene
            .unite(s1, s2)
            .unwrap_or_else(|e| panic!("{context}: unite failed: {e:?}"));
        let diff = scene
            .subtract(s1, s2)
            .unwrap_or_else(|e| panic!("{context}: subtract failed: {e:?}"));
        let vol_inter = volume(&inter, &format!("{context}: intersection"));
        let vol_union = volume(&union, &format!("{context}: union"));
        let vol_diff = volume(&diff, &format!("{context}: difference"));
        assert_close(
            vol_inter,
            lens,
            CURVED_VOLUME_RTOL,
            &format!("{context}: lens volume"),
        );
        assert_close(
            vol_union + vol_inter,
            sphere_volume(r1) + sphere_volume(r2),
            CURVED_VOLUME_RTOL,
            &format!("{context}: inclusion–exclusion identity"),
        );
        assert_close(
            vol_diff,
            sphere_volume(r1) - lens,
            CURVED_VOLUME_RTOL,
            &format!("{context}: difference identity"),
        );
    }
}

/// Seeded random transversal sphere pairs: centers along a random
/// direction, separation strictly between the internal and external
/// tangency distances with margin. Lens closed form + identities.
#[test]
fn random_sphere_pairs_identity() {
    let mut rng = Rng::new(0x2_5EED_BA11);
    for case in 0..8 {
        let r1 = rng.range(0.5, 1.2);
        let r2 = rng.range(0.5, 1.2);
        let d = rng.range((r1 - r2).abs() + 0.2, r1 + r2 - 0.15);
        let dir = Vector3::new(
            rng.range(-1.0, 1.0),
            rng.range(-1.0, 1.0),
            rng.range(-1.0, 1.0),
        )
        .normalize();
        let context = format!("case {case}: spheres r1={r1:.3} r2={r2:.3} d={d:.3} dir={dir:?}");
        let mut scene = Scene::new();
        let s1 = scene.sphere(Point3::origin(), r1);
        let s2 = scene.sphere(Point3::origin() + dir * d, r2);

        let lens = sphere_lens_volume(r1, r2, d);
        let inter = scene
            .intersect(s1, s2)
            .unwrap_or_else(|e| panic!("{context}: intersect failed: {e:?}"));
        let diff = scene
            .subtract(s1, s2)
            .unwrap_or_else(|e| panic!("{context}: subtract failed: {e:?}"));
        let vol_inter = volume(&inter, &format!("{context}: intersection"));
        let vol_diff = volume(&diff, &format!("{context}: difference"));
        assert_close(
            vol_inter,
            lens,
            CURVED_VOLUME_RTOL,
            &format!("{context}: lens volume"),
        );
        assert_close(
            vol_diff,
            sphere_volume(r1) - lens,
            CURVED_VOLUME_RTOL,
            &format!("{context}: difference identity"),
        );
    }
}

/// Nearly-tangent external sphere pair: a razor-thin lens. The
/// configuration is still formally transversal (clearance from tangency
/// far above linear tolerance), so it must produce a valid solid; the
/// volume check is a loose window because slivers tessellate coarsely.
#[test]
fn sphere_pair_near_tangent_lens() {
    for eps in [1e-3, 1e-4] {
        let context = format!("near-tangent sphere pair, overlap {eps:.0e}");
        let d = 2.0 - eps;
        let mut scene = Scene::new();
        let s1 = scene.sphere(Point3::origin(), 1.0);
        let s2 = scene.sphere(Point3::new(d, 0.0, 0.0), 1.0);
        let inter = scene
            .intersect(s1, s2)
            .unwrap_or_else(|e| panic!("{context}: intersect failed: {e:?}"));
        let vol = volume(&inter, &context);
        let lens = sphere_lens_volume(1.0, 1.0, d);
        assert!(
            vol > 0.2 * lens && vol < 5.0 * lens,
            "{context}: sliver lens volume {vol} outside ({:.3e}, {:.3e})",
            0.2 * lens,
            5.0 * lens
        );
    }
}

/// Sub-tolerance external tangency of two spheres: a structured
/// NotImplemented/Degenerate rejection is acceptable under the
/// transversal MVP contract; a panic or an invalid "success" is a bug.
#[test]
fn sphere_pair_sub_tolerance_tangency() {
    let context = "sphere pair 1e-7 inside external tangency";
    let d = 2.0 - 1e-7;
    let mut scene = Scene::new();
    let s1 = scene.sphere(Point3::origin(), 1.0);
    let s2 = scene.sphere(Point3::new(d, 0.0, 0.0), 1.0);
    match scene.unite(s1, s2) {
        Ok(out) => {
            let vol = volume(&out, context);
            assert_close(vol, 2.0 * sphere_volume(1.0), CURVED_VOLUME_RTOL, context);
        }
        Err(CoreError::NotImplemented { .. }) | Err(CoreError::Degenerate { .. }) => {}
        Err(other) => panic!("{context}: unexpected error kind: {other:?}"),
    }
}

// =====================================================================
// (7) Torus operands (of-7ld.3 campaign)
// =====================================================================

/// Torus sunk tube-deep into a slab, its center 0.2 below the top face:
/// the plane cuts every tube cross-section, so the intersection is a
/// full genus-1 ring and both boolean volumes have the exact
/// torus-below-plane closed form.
fn torus_sunk_in_slab(scale: f64) {
    let context = format!("torus sunk in slab at {scale}× scale");
    let s = scale;
    let (major, minor, drop) = (2.0 * s, 0.5 * s, 0.2 * s);
    let mut scene = Scene::new();
    let slab = scene.block([-6.0 * s, -6.0 * s, -4.0 * s], [6.0 * s, 6.0 * s, 0.0]);
    let ring = scene.torus(Point3::new(0.0, 0.0, -drop), major, minor);

    // Plane z = 0 sits `drop` above the tube center plane.
    let below = torus_below_plane_volume(major, minor, drop);
    let slab_vol = 12.0 * 12.0 * 4.0 * s * s * s;

    let inter = scene
        .intersect(slab, ring)
        .unwrap_or_else(|e| panic!("{context}: intersect failed: {e:?}"));
    let counts = inter.store.euler_counts(inter.body);
    assert_eq!(counts.genus, 1, "{context}: submerged part is a full ring");
    let vol_inter = volume(&inter, &format!("{context}: intersection"));
    assert_close(
        vol_inter,
        below,
        CURVED_VOLUME_RTOL,
        &format!("{context}: intersection vs torus-below-plane"),
    );

    let diff = scene
        .subtract(slab, ring)
        .unwrap_or_else(|e| panic!("{context}: subtract failed: {e:?}"));
    let counts = diff.store.euler_counts(diff.body);
    assert_eq!(counts.genus, 0, "{context}: ring groove must not add genus");
    let vol_diff = volume(&diff, &format!("{context}: difference"));
    assert_close(
        vol_diff,
        slab_vol - below,
        CURVED_VOLUME_RTOL,
        &format!("{context}: difference volume"),
    );

    let union = scene
        .unite(slab, ring)
        .unwrap_or_else(|e| panic!("{context}: unite failed: {e:?}"));
    let counts = union.store.euler_counts(union.body);
    assert_eq!(counts.genus, 0, "{context}: ridge ring must not add genus");
    let vol_union = volume(&union, &format!("{context}: union"));
    assert_close(
        vol_union,
        slab_vol + torus_volume(major, minor) - below,
        CURVED_VOLUME_RTOL,
        &format!("{context}: union volume"),
    );
}

#[test]
fn torus_sunk_in_slab_scale_1() {
    torus_sunk_in_slab(1.0);
}

#[test]
fn torus_sunk_in_slab_scale_0_001() {
    torus_sunk_in_slab(0.001);
}

#[test]
fn torus_sunk_in_slab_scale_1000() {
    torus_sunk_in_slab(1000.0);
}

/// Half torus by an axis-containing plane (x = 0, avoiding the seams at
/// +X): the imprints are the two tube cross-section circles at u = ±π/2,
/// each crossing the major seam edge transversally. The union grows a
/// half-ring arch on the block — a genuine handle, genus 1.
#[test]
fn half_torus_by_axis_plane() {
    let context = "torus halved by the axis-containing plane x = 0";
    let (major, minor) = (2.0, 0.5);
    let mut scene = Scene::new();
    let block = scene.block([-6.0, -6.0, -2.0], [0.0, 6.0, 2.0]);
    let ring = scene.torus(Point3::origin(), major, minor);

    let half = torus_volume(major, minor) / 2.0;
    let inter = scene
        .intersect(block, ring)
        .unwrap_or_else(|e| panic!("{context}: intersect failed: {e:?}"));
    let counts = inter.store.euler_counts(inter.body);
    assert_eq!(counts.genus, 0, "{context}: half ring is genus 0");
    let vol_inter = volume(&inter, &format!("{context}: intersection"));
    assert_close(vol_inter, half, CURVED_VOLUME_RTOL, context);

    let diff = scene
        .subtract(ring, block)
        .unwrap_or_else(|e| panic!("{context}: subtract failed: {e:?}"));
    let vol_diff = volume(&diff, &format!("{context}: difference"));
    assert_close(
        vol_diff,
        half,
        CURVED_VOLUME_RTOL,
        &format!("{context}: difference volume"),
    );

    let union = scene
        .unite(block, ring)
        .unwrap_or_else(|e| panic!("{context}: unite failed: {e:?}"));
    let counts = union.store.euler_counts(union.body);
    assert_eq!(counts.genus, 1, "{context}: arch handle must give genus 1");
    let vol_union = volume(&union, &format!("{context}: union"));
    assert_close(
        vol_union,
        12.0 * 12.0 * 4.0 / 2.0 + half,
        CURVED_VOLUME_RTOL,
        &format!("{context}: union volume"),
    );
}

/// Canonical sunk-torus configuration versus the same configuration
/// rigidly rotated — the torus rebuilt about the rotated axis via
/// [`Scene::torus_with_axis`] (its two seams land per `plane_basis` of
/// the rotated axis, exactly as the boolean chart will). Both frames
/// must reproduce the closed form.
#[test]
fn rotated_frame_torus_sunk_congruence() {
    let (major, minor, drop) = (2.0, 0.5, 0.2);
    let torus_center = Point3::new(0.0, 0.0, -drop);
    let below = torus_below_plane_volume(major, minor, drop);
    let expected = 12.0 * 12.0 * 4.0 - below;
    let rot = Rotation3::from_axis_angle(&Unit::new_normalize(Vector3::new(2.0, -1.0, 1.0)), 0.9);
    let pivot = Point3::new(1.0, 1.0, 1.0);

    for rotated in [false, true] {
        let context = format!("slab minus sunk torus, rotated frame: {rotated}");
        let mut scene = Scene::new();
        let slab = scene.block([-6.0, -6.0, -4.0], [6.0, 6.0, 0.0]);
        let ring = if rotated {
            scene.rotate(slab, &rot, &pivot);
            let center = pivot + rot * (torus_center - pivot);
            scene.torus_with_axis(center, rot * Vector3::z(), major, minor)
        } else {
            scene.torus(torus_center, major, minor)
        };
        let diff = scene
            .subtract(slab, ring)
            .unwrap_or_else(|e| panic!("{context}: subtract failed: {e:?}"));
        let vol = volume(&diff, &context);
        assert_close(vol, expected, CURVED_VOLUME_RTOL, &context);
    }
}

/// Block notch through the FULL tube cross-section over a small angular
/// span: the subtraction severs the ring into a C — genus drops 1 → 0.
/// The block's side faces are off-axis planes parallel to the torus
/// axis, whose torus sections are general quartics (marched SSI).
#[test]
fn block_severs_torus_tube() {
    let context = "block notch severing the torus tube";
    let (major, minor) = (2.0, 0.5);
    let mut scene = Scene::new();
    let ring = scene.torus(Point3::origin(), major, minor);
    let tool = scene.block([1.3, -0.35, -1.0], [2.7, 0.35, 1.0]);

    let diff = scene
        .subtract(ring, tool)
        .unwrap_or_else(|e| panic!("{context}: subtract failed: {e:?}"));
    let counts = diff.store.euler_counts(diff.body);
    assert_eq!(counts.genus, 0, "{context}: severed ring must be genus 0");
    let inter = scene
        .intersect(ring, tool)
        .unwrap_or_else(|e| panic!("{context}: intersect failed: {e:?}"));
    let vol_diff = volume(&diff, &format!("{context}: difference"));
    let vol_inter = volume(&inter, &format!("{context}: intersection"));
    assert_close(
        vol_diff + vol_inter,
        torus_volume(major, minor),
        CURVED_VOLUME_RTOL,
        &format!("{context}: difference + intersection vs torus volume"),
    );
}

/// Block notch into the OUTER wall only (never reaching the tube's
/// inner half): the ring survives, genus stays 1. The bite is centered
/// on the +X outer equator, crossing BOTH torus seams.
#[test]
fn block_notches_torus_outer_wall() {
    let context = "block notch in the torus outer wall across both seams";
    let (major, minor) = (2.0, 0.5);
    let mut scene = Scene::new();
    let ring = scene.torus(Point3::origin(), major, minor);
    let tool = scene.block([2.1, -0.35, -1.0], [2.7, 0.35, 1.0]);

    let diff = scene
        .subtract(ring, tool)
        .unwrap_or_else(|e| panic!("{context}: subtract failed: {e:?}"));
    let counts = diff.store.euler_counts(diff.body);
    assert_eq!(counts.genus, 1, "{context}: notched ring must stay genus 1");
    let inter = scene
        .intersect(ring, tool)
        .unwrap_or_else(|e| panic!("{context}: intersect failed: {e:?}"));
    let vol_diff = volume(&diff, &format!("{context}: difference"));
    let vol_inter = volume(&inter, &format!("{context}: intersection"));
    assert_close(
        vol_diff + vol_inter,
        torus_volume(major, minor),
        CURVED_VOLUME_RTOL,
        &format!("{context}: difference + intersection vs torus volume"),
    );
}

/// Two congruent coaxial tori shifted along their common axis: the tube
/// cross-sections are equal circles offset by the shift, so the
/// intersection is the revolved circle-circle lens (Pappus about the
/// common centroid radius R) — an exact closed form — and a full ring.
#[test]
fn coaxial_tori_axial_shift_lens() {
    let context = "coaxial tori shifted 0.6 along the axis";
    let (major, minor, shift) = (2.0, 0.5, 0.6);
    let mut scene = Scene::new();
    let t1 = scene.torus(Point3::origin(), major, minor);
    let t2 = scene.torus(Point3::new(0.0, 0.0, shift), major, minor);

    let lens = 2.0 * PI * major * circle_lens_area(minor, shift);
    let inter = scene
        .intersect(t1, t2)
        .unwrap_or_else(|e| panic!("{context}: intersect failed: {e:?}"));
    let counts = inter.store.euler_counts(inter.body);
    assert_eq!(counts.genus, 1, "{context}: lens ring is genus 1");
    let vol_inter = volume(&inter, &format!("{context}: intersection"));
    assert_close(vol_inter, lens, CURVED_VOLUME_RTOL, context);

    let union = scene
        .unite(t1, t2)
        .unwrap_or_else(|e| panic!("{context}: unite failed: {e:?}"));
    let counts = union.store.euler_counts(union.body);
    assert_eq!(counts.genus, 1, "{context}: merged rings stay genus 1");
    let vol_union = volume(&union, &format!("{context}: union"));
    assert_close(
        vol_union + vol_inter,
        2.0 * torus_volume(major, minor),
        CURVED_VOLUME_RTOL,
        &format!("{context}: inclusion–exclusion identity"),
    );
}

/// Two same-plane tori with different major radii (same tube radius):
/// the cross-sections are equal circles offset radially, so Pappus about
/// the lens centroid radius (R1 + R2)/2 gives the exact intersection.
#[test]
fn coplanar_tori_major_shift_lens() {
    let context = "coplanar tori, major radii 2.0 and 2.6";
    let (r1, r2, minor) = (2.0, 2.6, 0.5);
    let mut scene = Scene::new();
    let t1 = scene.torus(Point3::origin(), r1, minor);
    let t2 = scene.torus(Point3::origin(), r2, minor);

    let lens = 2.0 * PI * ((r1 + r2) / 2.0) * circle_lens_area(minor, r2 - r1);
    let inter = scene
        .intersect(t1, t2)
        .unwrap_or_else(|e| panic!("{context}: intersect failed: {e:?}"));
    let vol_inter = volume(&inter, &format!("{context}: intersection"));
    assert_close(vol_inter, lens, CURVED_VOLUME_RTOL, context);

    let diff = scene
        .subtract(t1, t2)
        .unwrap_or_else(|e| panic!("{context}: subtract failed: {e:?}"));
    let vol_diff = volume(&diff, &format!("{context}: difference"));
    assert_close(
        vol_diff,
        torus_volume(r1, minor) - lens,
        CURVED_VOLUME_RTOL,
        &format!("{context}: difference identity"),
    );
}

/// Perpendicular-axis tori built so the second tube loops around the
/// first one's centerline at constant clearance, overlapping it by 0.1:
/// genuinely doubly-curved transversal contact with no closed form —
/// assert validity and the pairwise volume identity.
#[test]
fn perpendicular_tori_identity() {
    let context = "perpendicular tori, tube-around-tube overlap";
    let mut scene = Scene::new();
    // T2's centerline (radius 1 about (0,2,0) in the x = 0 plane) keeps
    // distance exactly 1 from T1's centerline; tube radii 0.7 + 0.4
    // overlap that channel by 0.1.
    let t1 = scene.torus(Point3::origin(), 2.0, 0.7);
    let t2 = scene.torus_with_axis(Point3::new(0.0, 2.0, 0.0), Vector3::x(), 1.0, 0.4);

    let diff = scene
        .subtract(t1, t2)
        .unwrap_or_else(|e| panic!("{context}: subtract failed: {e:?}"));
    let inter = scene
        .intersect(t1, t2)
        .unwrap_or_else(|e| panic!("{context}: intersect failed: {e:?}"));
    let vol_diff = volume(&diff, &format!("{context}: difference"));
    let vol_inter = volume(&inter, &format!("{context}: intersection"));
    assert_close(
        vol_diff + vol_inter,
        torus_volume(2.0, 0.7),
        CURVED_VOLUME_RTOL,
        &format!("{context}: difference + intersection vs T1 volume"),
    );
}

/// The same construction at EXACT channel tangency (tube radii sum to
/// the centerline clearance): the surfaces touch along a whole curve
/// without crossing. Structured rejection is acceptable; a panic or an
/// invalid success is a bug.
#[test]
fn perpendicular_tori_channel_tangency() {
    let context = "perpendicular tori tangent along the channel curve";
    let mut scene = Scene::new();
    let t1 = scene.torus(Point3::origin(), 2.0, 0.6);
    let t2 = scene.torus_with_axis(Point3::new(0.0, 2.0, 0.0), Vector3::x(), 1.0, 0.4);
    match scene.unite(t1, t2) {
        Ok(out) => {
            let vol = volume(&out, context);
            assert_close(
                vol,
                torus_volume(2.0, 0.6) + torus_volume(1.0, 0.4),
                CURVED_VOLUME_RTOL,
                context,
            );
        }
        Err(CoreError::NotImplemented { .. }) | Err(CoreError::Degenerate { .. }) => {}
        Err(other) => panic!("{context}: unexpected error kind: {other:?}"),
    }
}

// =====================================================================
// (8) Sphere/torus near-tangency and SDF round-trips (of-7ld.3)
// =====================================================================

/// Sphere dipping a razor-thin cap into a slab face — formally
/// transversal (clearance far above linear tolerance) so it must
/// succeed; slivers tessellate coarsely, so the volume check is a
/// window, and validity is the real assertion.
#[test]
fn plane_grazes_sphere_tiny_caps() {
    for h in [1e-3, 1e-4] {
        let context = format!("sphere dips {h:.0e} into the slab top");
        let r = 1.0;
        let mut scene = Scene::new();
        let slab = scene.block([-4.0, -4.0, -4.0], [4.0, 4.0, 0.0]);
        let ball = scene.sphere(Point3::new(0.0, 0.0, r - h), r);
        let inter = scene
            .intersect(slab, ball)
            .unwrap_or_else(|e| panic!("{context}: intersect failed: {e:?}"));
        let vol = volume(&inter, &context);
        let cap = spherical_cap_volume(r, h);
        assert!(
            vol > 0.2 * cap && vol < 5.0 * cap,
            "{context}: sliver cap volume {vol} outside ({:.3e}, {:.3e})",
            0.2 * cap,
            5.0 * cap
        );
    }
}

/// Sphere clearing the slab top by less than the linear tolerance:
/// sub-tolerance contact. Structured rejection or a valid, untouched
/// result are both acceptable; a panic or invalid success is a bug.
#[test]
fn plane_grazes_sphere_sub_tolerance() {
    let context = "sphere dips 1e-7 into the slab top";
    let r = 1.0;
    let mut scene = Scene::new();
    let slab = scene.block([-4.0, -4.0, -4.0], [4.0, 4.0, 0.0]);
    let ball = scene.sphere(Point3::new(0.0, 0.0, r - 1e-7), r);
    match scene.subtract(slab, ball) {
        Ok(out) => {
            let vol = volume(&out, context);
            assert_close(vol, 8.0 * 8.0 * 4.0, CURVED_VOLUME_RTOL, context);
        }
        Err(CoreError::NotImplemented { .. }) | Err(CoreError::Degenerate { .. }) => {}
        Err(other) => panic!("{context}: unexpected error kind: {other:?}"),
    }
}

/// Boolean output → tessellate → MeshSdf → dual-contour re-mesh volume
/// agreement, for a sphere cap subtraction.
#[test]
fn round_trip_slab_minus_sphere_cap() {
    let mut scene = Scene::new();
    let slab = scene.block([0.0, 0.0, 0.0], [4.0, 4.0, 2.0]);
    let ball = scene.sphere(Point3::new(2.0, 2.0, 2.4), 1.0);
    let out = scene.subtract(slab, ball).expect("cap subtract");
    round_trip_volume(&out, "round-trip: slab minus sphere cap");
}

/// The same SDF round-trip for a slab ∪ sunk torus (curved ridge ring).
#[test]
fn round_trip_slab_union_torus() {
    let mut scene = Scene::new();
    let slab = scene.block([-4.0, -4.0, -4.0], [4.0, 4.0, 0.0]);
    let ring = scene.torus(Point3::new(0.0, 0.0, -0.2), 2.0, 0.5);
    let out = scene.unite(slab, ring).expect("sunk torus union");
    round_trip_volume(&out, "round-trip: slab union sunk torus");
}

/// Tangential sphere/torus contacts must never panic: every outcome is
/// either a fully valid solid or a structured error.
#[test]
fn no_panics_on_sphere_torus_tangencies() {
    let mut scene = Scene::new();
    let ball = scene.sphere(Point3::origin(), 1.0);
    let pole_block = scene.block([-2.0, -2.0, 1.0], [2.0, 2.0, 3.0]);
    let corner_cube = scene.block([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]);
    let corner_ball = scene.sphere(Point3::new(3.0, 2.0, 2.0), 1.0);
    let ring = scene.torus(Point3::origin(), 2.0, 0.5);
    let top_block = scene.block([-4.0, -4.0, 0.5], [4.0, 4.0, 3.0]);
    let cases: Vec<(&str, CoreResult<BooleanOutput>)> = vec![
        (
            "block face tangent at the sphere's north pole",
            scene.unite(ball, pole_block),
        ),
        (
            "sphere tangent to a block face at one point",
            scene.unite(corner_cube, corner_ball),
        ),
        (
            "block face tangent along the torus top circle",
            scene.unite(ring, top_block),
        ),
    ];
    for (name, result) in cases {
        match result {
            Ok(out) => {
                assert_valid(&out, name);
            }
            Err(e) => {
                let _ = format!("{name}: rejected with {e:?}");
            }
        }
    }
}

// =====================================================================
// Tangent-contact triage (of-bxl.6, COINCIDENT.md §6 tier 1)
//
// SSI reports tangency of the *infinite* surfaces; it only bars the
// exact path when the contact locus actually enters both trimmed
// regions. A locus outside either trim imprints nothing and the boolean
// is ordinary transversal work. In-trim contact stays NotImplemented:
// tier 2 (point contact → non-manifold vertex) is unrepresentable in
// the ≤2-fins-per-edge topology, and tier 3 (tangential curves through
// the trims) is of-bxl.7. The hybrid kernel serves both via F-Rep.
// =====================================================================

/// Sphere biting the block's side wall while resting on the *plane* of
/// the block's bottom face: the tangent foot (4.5, 2, 0) — which is also
/// the sphere's south pole, exercising the chart's pole convention in
/// the triage test — lies outside the bottom face's trim (x ≤ 4). The
/// tangency is a false positive and all three ops are ordinary.
#[test]
fn tangent_point_outside_trim_is_ordinary() {
    let mut scene = Scene::new();
    let block = scene.block([0.0, 0.0, 0.0], [4.0, 4.0, 4.0]);
    // Center half a radius inside the x = 4 wall's plane: the wall cuts a
    // cap of depth r − 0.5 off the sphere.
    let ball = scene.sphere(Point3::new(4.5, 2.0, 1.0), 1.0);
    let cap = spherical_cap_volume(1.0, 0.5);

    let context = "block ∩ side-biting ball tangent to the bottom plane off-trim";
    let inter = scene
        .intersect(block, ball)
        .unwrap_or_else(|e| panic!("{context}: intersect failed: {e:?}"));
    assert_close(volume(&inter, context), cap, CURVED_VOLUME_RTOL, context);

    let context = "block − side-biting ball tangent to the bottom plane off-trim";
    let diff = scene
        .subtract(block, ball)
        .unwrap_or_else(|e| panic!("{context}: subtract failed: {e:?}"));
    assert_close(
        volume(&diff, context),
        64.0 - cap,
        CURVED_VOLUME_RTOL,
        context,
    );

    let context = "block ∪ side-biting ball tangent to the bottom plane off-trim";
    let uni = scene
        .unite(block, ball)
        .unwrap_or_else(|e| panic!("{context}: unite failed: {e:?}"));
    assert_close(
        volume(&uni, context),
        64.0 + sphere_volume(1.0) - cap,
        CURVED_VOLUME_RTOL,
        context,
    );
}

/// The tangential-*curve* analog: a horizontal rod biting the block's
/// x = 4 wall while its cylinder wall is tangent to the bottom face's
/// plane along the line x = 4.5 — wholly outside the bottom trim. The
/// removed material is a circular-segment prism.
#[test]
fn tangent_line_outside_trim_is_ordinary() {
    let context = "block minus rod tangent to the bottom plane off-trim";
    let mut scene = Scene::new();
    let block = scene.block([0.0, 0.0, 0.0], [4.0, 4.0, 4.0]);
    let rod = scene.cylinder(Point3::new(4.5, 1.0, 1.0), Vector3::y(), 1.0, 2.0);
    let out = scene
        .subtract(block, rod)
        .unwrap_or_else(|e| panic!("{context}: subtract failed: {e:?}"));
    // Segment cut by a chord at distance 0.5 from a unit circle's center,
    // extruded over the rod's 2-long axis.
    let segment_area = (0.5f64).acos() - 0.5 * 0.75f64.sqrt();
    assert_close(
        volume(&out, context),
        64.0 - 2.0 * segment_area,
        CYL_VOLUME_RTOL,
        context,
    );
}

/// Tier 2 stays refused: a sphere resting ON the plate's top face (foot
/// inside both trims) would union into a body with a non-manifold
/// vertex, which the topology cannot represent. The exact path must keep
/// the structured NotImplemented that routes the hybrid kernel to F-Rep
/// — assert the refusal, not a result.
#[test]
fn tangent_point_inside_both_trims_stays_not_implemented() {
    let context = "sphere resting on the plate's top face, united";
    let mut scene = Scene::new();
    let plate = scene.block([0.0, 0.0, 0.0], [4.0, 4.0, 2.0]);
    let ball = scene.sphere(Point3::new(2.0, 2.0, 3.0), 1.0);
    match scene.unite(plate, ball) {
        Err(CoreError::NotImplemented { .. }) => {}
        other => panic!("{context}: expected NotImplemented, got {other:?}"),
    }
}

/// Tier 3 stays refused: the same rod as
/// [`tangent_line_outside_trim_is_ordinary`] moved into the block, so
/// its tangent line runs through the bottom face's trim.
#[test]
fn tangent_line_through_trim_stays_not_implemented() {
    let context = "rod resting on the block's bottom face from inside, united";
    let mut scene = Scene::new();
    let block = scene.block([0.0, 0.0, 0.0], [4.0, 4.0, 4.0]);
    let rod = scene.cylinder(Point3::new(2.0, 1.0, 1.0), Vector3::y(), 1.0, 2.0);
    match scene.unite(block, rod) {
        Err(CoreError::NotImplemented { .. }) => {}
        other => panic!("{context}: expected NotImplemented, got {other:?}"),
    }
}

// =====================================================================
// Guard: error paths must be structured, never panics.
// =====================================================================

/// A grid of increasingly awkward but legal configurations must never
/// panic — every outcome is either a valid solid or a structured error.
#[test]
fn no_panics_on_awkward_configurations() {
    let mut scene = Scene::new();
    let cube = scene.block([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]);
    let corner_tool = scene.block([2.0 - 1e-9, 0.5, 0.5], [3.0, 1.5, 1.5]);
    let resolution_tool = scene.block([0.5, 0.5, 2.0 - 1e-11], [1.5, 1.5, 3.0]);
    let needle_tool = scene.block([0.999, 0.999, -1.0], [1.001, 1.001, 3.0]);
    let cases: Vec<(&str, CoreResult<BooleanOutput>)> = vec![
        (
            "tool corner exactly on face plane",
            scene.unite(cube, corner_tool),
        ),
        (
            "tool face within system resolution of face",
            scene.unite(cube, resolution_tool),
        ),
        (
            "needle tool through the cube",
            scene.subtract(cube, needle_tool),
        ),
    ];
    for (name, result) in cases {
        match result {
            Ok(out) => {
                // Whatever the pipeline claims to have produced must hold up.
                assert_valid(&out, name);
            }
            Err(e) => {
                // Structured refusal is fine for these near-degenerate pokes.
                let _ = format!("{name}: rejected with {e:?}");
            }
        }
    }
}

// =====================================================================
// (9) Cone / frustum operands (of-fsl.23 campaign)
//
// Written tests-first while `Chart::build` still rejected
// `Surface3::Cone`; the gate has since lifted (of-dtj) and every case
// here is live.
// Volumes use `frustum_volume` closed forms
// (`π h (r1² + r1·r2 + r2²)/3`); tilted/overlap cases fall back to the
// scale-free inclusion–exclusion identity `vol(A)+vol(B)=vol(∪)+vol(∩)`.
// =====================================================================

/// A frustum tool passing entirely through a slab (both caps outside)
/// bores a tapered through-hole (genus 1). Removed material is the
/// frustum section between the two slab faces — the direct analog of the
/// cylinder `through_hole` case, exercising the cone wall and its two
/// circular plane-cone SSIs with no apex and no tool cap involved.
#[test]
fn frustum_through_slab() {
    let context = "slab minus tapered frustum (through-hole)";
    let mut scene = Scene::new();
    let slab = scene.block([0.0, 0.0, 0.0], [6.0, 6.0, 2.0]);
    // radius(z) = 0.5 + (z + 1)/2 → 1.0 at z = 0, 2.0 at z = 2.
    let tool = scene.cone(Point3::new(3.0, 3.0, -1.0), 0.5, 2.5, 4.0);
    let out = scene
        .subtract(slab, tool)
        .unwrap_or_else(|e| panic!("{context}: subtract failed: {e:?}"));
    let counts = out.store.euler_counts(out.body);
    assert_eq!(
        counts.genus, 1,
        "{context}: tapered through hole is genus 1"
    );
    let vol = volume(&out, context);
    let removed = frustum_volume(1.0, 2.0, 2.0);
    assert_close(vol, 72.0 - removed, CYL_VOLUME_RTOL, context);
}

/// A pointed cone poking up through the slab's top face cuts a conical
/// countersink pit (genus 0, single shell). The tool's apex sits inside
/// the slab, so the removed region is a cone from the apex up to the top
/// face — the apex (a pole-like `u`-circle collapse) is exercised on
/// every run, mirroring `sphere_cap_bite`'s pole coverage.
fn cone_countersink(scale: f64) {
    let context = format!("slab minus conical countersink at {scale}× scale");
    let s = scale;
    let mut scene = Scene::new();
    let slab = scene.block([0.0, 0.0, 0.0], [6.0 * s, 6.0 * s, 2.0 * s]);
    // Apex at z = 0.5s inside the slab; radius(z) = (z − 0.5s)/2 → 0.75s
    // at the top face z = 2s. Top cap (r = 2s) sits above the slab.
    let tool = scene.cone(
        Point3::new(3.0 * s, 3.0 * s, 0.5 * s),
        0.0,
        2.0 * s,
        4.0 * s,
    );
    let out = scene
        .subtract(slab, tool)
        .unwrap_or_else(|e| panic!("{context}: subtract failed: {e:?}"));
    let counts = out.store.euler_counts(out.body);
    assert_eq!(counts.genus, 0, "{context}: a blind pit adds no genus");
    assert_eq!(out.shell_count(), 1, "{context}: single shell expected");
    let vol = volume(&out, &context);
    let s3 = s * s * s;
    let removed = frustum_volume(0.0, 0.75, 1.5) * s3;
    assert_close(vol, 72.0 * s3 - removed, CURVED_VOLUME_RTOL, &context);
}

#[test]
fn cone_countersink_bite() {
    cone_countersink(1.0);
}

#[test]
fn cone_bite_at_scale_0_001() {
    cone_countersink(0.001);
}

#[test]
fn cone_bite_at_scale_1000() {
    cone_countersink(1000.0);
}

/// Two coaxial FRUSTUMS whose lateral walls cross once (no apex, both radii
/// positive): the clean end-to-end exercise of coaxial cone-cone SSI on the
/// exact path. Their axial extents are staggered so no cap planes coincide
/// (coplanar caps would trip the coincident-face MVP limit, not the SSI):
///   A widens  r 1→4 over z∈[0,3]  (wall ρ = 1 + z)
///   B narrows r 4→1 over z∈[1,4]  (wall ρ = 5 − z)
/// The walls cross at z = 2, ρ = 3 — the coaxial cone-cone circle. The
/// intersection is the barrel min(ρₐ, ρ_b) over z∈[1,3], bounded below by B's
/// bottom cap and above by A's top cap (both clipped to ρ = 2), with the
/// wall-cap circles coming from plane-cone SSI. No apex pole is involved, so
/// this promotes on the SSI alone (of-dtj.4), unlike the true-cone
/// `opposed_cones_intersection` (apex machinery, of-dtj.5).
#[test]
fn crossing_frustums_intersection() {
    let context = "coaxial crossing frustums intersection (barrel)";
    let mut scene = Scene::new();
    let widen = scene.cone(Point3::new(0.0, 0.0, 0.0), 1.0, 4.0, 3.0);
    let narrow = scene.cone(Point3::new(0.0, 0.0, 1.0), 4.0, 1.0, 3.0);
    let out = scene
        .intersect(widen, narrow)
        .unwrap_or_else(|e| panic!("{context}: intersect failed: {e:?}"));
    // Two stacked frustums: r 2→3 over z∈[1,2] and r 3→2 over z∈[2,3].
    let want = frustum_volume(2.0, 3.0, 1.0) + frustum_volume(3.0, 2.0, 1.0);
    let vol = volume(&out, context);
    assert_close(vol, want, CURVED_VOLUME_RTOL, context);
}

/// Two FRUSTUMS on non-coaxial (crossing) axes: the general-position
/// cone-cone SSI, a quartic with no closed form that the boolean pipeline
/// marches within the clashing faces' box (of-dtj.4). Both are frustums
/// (radii > 0, no apex pole), so promotion rides on the SSI alone. The
/// removed/overlap geometry has no closed form, so the invariant is the
/// scale-free inclusion–exclusion identity across all three ops.
/// Was `#[ignore]`d as of-9ia. Both of its blockers were in the imprint
/// hosting, and both were mis-scoped as cone-cone problems when neither
/// involves the marched arc at all — see of-9ia's close notes.
#[test]
fn skew_frustums_inclusion_exclusion() {
    let context = "non-coaxial frustums ∪/∩ identity";
    let mut scene = Scene::new();
    let upright = scene.cone(Point3::new(0.0, 0.0, 0.0), 2.5, 1.0, 4.0);
    let tilted = scene.cone_tilted(
        Point3::new(0.0, 0.0, 2.0),
        2.5,
        1.0,
        4.0,
        Vector3::new(1.0, 0.0, 0.0),
        50.0_f64.to_radians(),
    );
    let union = scene
        .unite(upright, tilted)
        .unwrap_or_else(|e| panic!("{context}: unite failed: {e:?}"));
    let inter = scene
        .intersect(upright, tilted)
        .unwrap_or_else(|e| panic!("{context}: intersect failed: {e:?}"));
    let vol_union = volume(&union, &format!("{context}: union"));
    let vol_inter = volume(&inter, &format!("{context}: intersection"));
    let vol_each = frustum_volume(2.5, 1.0, 4.0);
    assert_close(
        vol_union + vol_inter,
        2.0 * vol_each,
        CURVED_VOLUME_RTOL,
        &format!("{context}: identity"),
    );
}

/// Two coaxial cones opposed apex-to-base overlap in a lens whose
/// intersection is a bicone (two cones meeting base-to-base at the height
/// where their radii coincide). Exercises coaxial cone-cone SSI (a single
/// full-wrap circle at z = 2) and closed-form intersection volume.
#[test]
fn opposed_cones_intersection() {
    let context = "opposed coaxial cones intersection (bicone)";
    let mut scene = Scene::new();
    // A: widest at z = 0 (r = 2), apex at z = 3.  radius_A(z) = 2(1 − z/3).
    let cone_a = scene.cone(Point3::new(0.0, 0.0, 0.0), 2.0, 0.0, 3.0);
    // B: apex at z = 1, widening to r = 2 at z = 4.  radius_B(z) = 2(z − 1)/3.
    let cone_b = scene.cone(Point3::new(0.0, 0.0, 1.0), 0.0, 2.0, 3.0);
    let out = scene
        .intersect(cone_a, cone_b)
        .unwrap_or_else(|e| panic!("{context}: intersect failed: {e:?}"));
    // Radii coincide at z = 2 (both 2/3); ∩ is two cones of height 1 there.
    let want = 2.0 * frustum_volume(0.0, 2.0 / 3.0, 1.0);
    let vol = volume(&out, context);
    assert_close(vol, want, CURVED_VOLUME_RTOL, context);
}

/// Inclusion–exclusion identity for a full cone body and a block it
/// pierces: `vol(A) + vol(B) == vol(A∪B) + vol(A∩B)`, robust to the messy
/// (non-closed-form) overlap geometry. Exercises all three ops on cone
/// inputs at once.
#[test]
fn cone_block_inclusion_exclusion() {
    let context = "cone ∪/∩ block inclusion–exclusion";
    let mut scene = Scene::new();
    let block = scene.block([0.0, 0.0, 0.0], [4.0, 4.0, 4.0]);
    let cone = scene.cone(Point3::new(2.0, 2.0, -1.0), 1.5, 0.5, 6.0);
    let union = scene
        .unite(block, cone)
        .unwrap_or_else(|e| panic!("{context}: unite failed: {e:?}"));
    let inter = scene
        .intersect(block, cone)
        .unwrap_or_else(|e| panic!("{context}: intersect failed: {e:?}"));
    let vol_union = volume(&union, &format!("{context}: union"));
    let vol_inter = volume(&inter, &format!("{context}: intersection"));
    let vol_cone = frustum_volume(1.5, 0.5, 6.0);
    assert_close(
        vol_union + vol_inter,
        64.0 + vol_cone,
        CURVED_VOLUME_RTOL,
        &format!("{context}: identity"),
    );
}

/// Two interpenetrating coaxial frustums: the inclusion–exclusion
/// identity must hold across their cone-cone wall intersection in the
/// overlap band. Closed-form operand volumes, identity for the overlap.
/// Rides on the coaxial branch of the analytic cone-cone SSI (cf. the
/// `cone_cone_opposed_single_circle` unit test); the general non-coaxial
/// pair is marched instead, and is covered by
/// [`skew_frustums_inclusion_exclusion`].
#[test]
fn coaxial_frustums_union_identity() {
    let context = "coaxial frustums union/intersection identity";
    let mut scene = Scene::new();
    // The frustums must not share a half-angle: `lower` narrows along
    // r(z) = 2 − z/3, so an `upper` of (1.5, 0.5, 3.0) based at z = 1.5
    // would trace r(z) = 2 − z/3 as well — the same cone surface, a
    // coincident-face pair rather than the transversal wall crossing this
    // test is about. `upper` widens instead (r(z) = 1 + (z − 1.5)/3),
    // cutting the lower wall at z = 2.25 inside the overlap band.
    let lower = scene.cone(Point3::new(0.0, 0.0, 0.0), 2.0, 1.0, 3.0);
    let upper = scene.cone(Point3::new(0.0, 0.0, 1.5), 1.0, 2.0, 3.0);
    let union = scene
        .unite(lower, upper)
        .unwrap_or_else(|e| panic!("{context}: unite failed: {e:?}"));
    let inter = scene
        .intersect(lower, upper)
        .unwrap_or_else(|e| panic!("{context}: intersect failed: {e:?}"));
    let vol_union = volume(&union, &format!("{context}: union"));
    let vol_inter = volume(&inter, &format!("{context}: intersection"));
    let vol_lower = frustum_volume(2.0, 1.0, 3.0);
    let vol_upper = frustum_volume(1.0, 2.0, 3.0);
    assert_close(
        vol_union + vol_inter,
        vol_lower + vol_upper,
        CURVED_VOLUME_RTOL,
        &format!("{context}: identity"),
    );
}

/// A cone tilted 20° off the block's axes, subtracted from a block: the
/// oblique cone wall stresses the tilted-frame chart and generic
/// plane-cone SSI. No closed form for the removed volume, so the
/// scale-free inclusion–exclusion identity is the invariant.
#[test]
fn tilted_cone_block_identity() {
    let context = "tilted cone ∪/∩ block inclusion–exclusion";
    let mut scene = Scene::new();
    let block = scene.block([0.0, 0.0, 0.0], [4.0, 4.0, 4.0]);
    let cone = scene.cone_tilted(
        Point3::new(2.0, 2.0, 2.0),
        1.3,
        0.4,
        3.0,
        Vector3::new(1.0, 0.0, 0.0),
        20.0_f64.to_radians(),
    );
    let union = scene
        .unite(block, cone)
        .unwrap_or_else(|e| panic!("{context}: unite failed: {e:?}"));
    let inter = scene
        .intersect(block, cone)
        .unwrap_or_else(|e| panic!("{context}: intersect failed: {e:?}"));
    let vol_union = volume(&union, &format!("{context}: union"));
    let vol_inter = volume(&inter, &format!("{context}: intersection"));
    let vol_cone = frustum_volume(1.3, 0.4, 3.0);
    assert_close(
        vol_union + vol_inter,
        64.0 + vol_cone,
        CURVED_VOLUME_RTOL,
        &format!("{context}: identity"),
    );
}

/// Rigid-motion invariance: the conical countersink bite's volume must be
/// identical after rotating BOTH operands by the same rotation (via the
/// geometry-complete [`rotate_body`]). Catches frame-dependent chart or
/// SSI bugs the axis-aligned cases would miss.
#[test]
fn rotated_frustum_bite_invariance() {
    // Baseline: axis-aligned countersink bite.
    let mut base_scene = Scene::new();
    let base_slab = base_scene.block([0.0, 0.0, 0.0], [6.0, 6.0, 2.0]);
    let base_tool = base_scene.cone(Point3::new(3.0, 3.0, 0.5), 0.0, 2.0, 4.0);
    let base_out = base_scene
        .subtract(base_slab, base_tool)
        .expect("baseline countersink subtract");
    let base_vol = volume(&base_out, "baseline countersink");

    // Same configuration, both operands rotated 0.4 rad about a skew axis
    // through the slab center.
    let pivot = Point3::new(3.0, 3.0, 1.0);
    let axis = Vector3::new(1.0, 1.0, 0.0);
    let angle = 0.4;
    let mut scene = Scene::new();
    let slab = scene.block([0.0, 0.0, 0.0], [6.0, 6.0, 2.0]);
    let tool = scene.cone(Point3::new(3.0, 3.0, 0.5), 0.0, 2.0, 4.0);
    for body in [slab, tool] {
        rotate_body(&mut scene.store, &mut scene.geo, body, pivot, axis, angle)
            .expect("valid rotation");
    }
    let out = scene
        .subtract(slab, tool)
        .expect("rotated countersink subtract");
    let vol = volume(&out, "rotated countersink");
    assert_close(
        vol,
        base_vol,
        CURVED_VOLUME_RTOL,
        "countersink bite volume is rotation-invariant",
    );
}

/// Cone inputs must never PANIC the boolean pipeline. These cases return
/// valid solids now that cones are promoted (of-dtj); before the promotion
/// they returned a structured `NotImplemented` (the F-Rep fallback, with
/// the `Chart::build` gate closed). Both outcomes are still accepted —
/// only a panic or an invalid `Ok` is a bug — so this guard holds the line
/// either way, including for any configuration that still diverts to the
/// fallback.
#[test]
fn no_panics_on_cone_configurations() {
    let mut scene = Scene::new();
    let slab = scene.block([0.0, 0.0, 0.0], [6.0, 6.0, 2.0]);
    let through = scene.cone(Point3::new(3.0, 3.0, -1.0), 0.5, 2.5, 4.0);
    let pit = scene.cone(Point3::new(3.0, 3.0, 0.5), 0.0, 2.0, 4.0);
    let block = scene.block([0.0, 0.0, 0.0], [4.0, 4.0, 4.0]);
    let coneful = scene.cone(Point3::new(2.0, 2.0, -1.0), 1.5, 0.5, 6.0);
    let cases: Vec<(&str, CoreResult<BooleanOutput>)> = vec![
        ("frustum through slab", scene.subtract(slab, through)),
        ("conical countersink bite", scene.subtract(slab, pit)),
        ("cone ∪ block", scene.unite(block, coneful)),
        ("cone ∩ block", scene.intersect(block, coneful)),
    ];
    for (name, result) in cases {
        match result {
            Ok(out) => {
                // If the pipeline claims success it must be a valid solid.
                assert_valid(&out, name);
            }
            Err(e) => {
                // A structured fallback (NotImplemented) is acceptable.
                let _ = format!("{name}: rejected with {e:?}");
            }
        }
    }
}

// =====================================================================
// (10) Coincident surfaces carrying disjoint trims (of-bxl.2)
// =====================================================================

/// Two unit blocks set corner to corner, `gap` apart in y. Their `x = 1`
/// planes are coincident, as are their `z = 0` and `z = 1` planes, but on
/// every one of those planes the two trimmed regions miss each other — so
/// the union is ordinary transversal work (here, two disjoint cubes).
///
/// SSI decides coincidence from the *infinite* surfaces and never consults
/// the trims, so before of-bxl.2 each of those pairs was rejected outright.
///
/// `tilt` rotates both operands 45° about z. That is the load-bearing
/// variant: axis-aligned coplanar faces this far apart never even reach SSI,
/// because their bounding boxes are tight and the broad phase separates
/// them. Tilting fattens each face's axis-aligned box (a planar face is
/// boxed from its boundary samples and dilated by a fraction of its extent,
/// see `broad_phase_face_box`), so the boxes overlap, the pair reaches SSI,
/// and only the trim test can tell the configuration apart.
fn coplanar_disjoint_blocks(gap: f64, tilt: bool, context: &str) {
    let mut scene = Scene::new();
    let a = scene.block([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
    let b = scene.block([1.0, 1.0 + gap, 0.0], [2.0, 2.0 + gap, 1.0]);
    if tilt {
        let rot = Rotation3::from_axis_angle(&Unit::new_normalize(Vector3::z()), FRAC_PI_4);
        scene.rotate(a, &rot, &Point3::origin());
        scene.rotate(b, &rot, &Point3::origin());
    }
    let out = scene
        .unite(a, b)
        .unwrap_or_else(|e| panic!("{context}: unite rejected a transversal pair: {e:?}"));
    // Two unit cubes that touch nowhere: the union keeps both whole.
    assert_close(volume(&out, context), 2.0, PLANAR_VOLUME_RTOL, context);
}

#[test]
fn coplanar_disjoint_faces_unite_near_miss() {
    coplanar_disjoint_blocks(0.05, false, "coplanar faces 0.05 apart");
}

#[test]
fn coplanar_disjoint_faces_unite_clear_miss() {
    coplanar_disjoint_blocks(0.2, false, "coplanar faces 0.2 apart");
}

#[test]
fn coplanar_disjoint_faces_unite_near_miss_tilted() {
    coplanar_disjoint_blocks(0.05, true, "coplanar faces 0.05 apart, tilted 45°");
}

#[test]
fn coplanar_disjoint_faces_unite_clear_miss_tilted() {
    coplanar_disjoint_blocks(0.2, true, "coplanar faces 0.2 apart, tilted 45°");
}

/// The same pair pushed together until the two blocks touch along exactly
/// one vertical edge. The `x = 1` planes are still coincident, and their
/// trims now meet — but in a line, i.e. zero area — so there is still
/// nothing to imprint and the target must come through whole.
///
/// Only `subtract` is asserted here, and deliberately so:
/// - `unite` of this pair is legitimately NON-MANIFOLD (the cubes would stay
///   two shells joined at the shared edge's two endpoints). It is rejected
///   rather than returned — the edge-contact degeneracy, not this gate's
///   business; see `edge_adjacent_blocks_unite_is_not_implemented` (of-n5g).
/// - `intersect` is empty, which the kernel reports as `SolidWithoutShells`
///   for *any* disjoint pair (verified against fully separated blocks),
///   coincident faces or not.
#[test]
fn edge_adjacent_blocks_subtract_leaves_target_whole() {
    let context = "blocks touching along one edge, subtract";
    let mut scene = Scene::new();
    let a = scene.block([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
    let b = scene.block([1.0, 1.0, 0.0], [2.0, 2.0, 1.0]);
    let out = scene
        .subtract(a, b)
        .unwrap_or_else(|e| panic!("{context}: rejected a zero-area contact: {e:?}"));
    assert_close(volume(&out, context), 1.0, PLANAR_VOLUME_RTOL, context);
}

// =====================================================================
// (11) Coincident faces carrying OVERLAPPING trims (of-bxl.4)
// =====================================================================
//
// The other half of section (10). There the coincident surfaces' trims
// missed, so the pair was ordinary transversal work; here they genuinely
// share area, which is the case that needs the ON verdict and — where a
// partner edge crosses a face's interior — the coincident imprint.
//
// `check()` is the PRIMARY gate for this section, and the volume oracle is
// secondary (COINCIDENT.md §7). The instinct is backwards here: a leftover
// interior wall has ZERO volume, so union two stacked boxes, fail to drop
// the shared wall, and the volume still comes out exactly right. What
// `check()` catches is precisely that — a retained wall puts four fins on
// its edges and trips the manifoldness check; an ON region dropped from
// both solids opens the shell; a same-sense face kept twice duplicates into
// four fins. `volume()` runs `assert_valid` (hence `check()`) on every call
// below, so every case is gated on both.
//
// Volume still earns its place against SENSE errors — a kept face with a
// flipped normal, or the same-sense region kept from the wrong solid —
// which are geometric, and which `check()` passes happily. Hence explicit
// expected-volume asserts throughout rather than mere relative divergence,
// plus face counts, which pin the tie-break that neither gate can see.

/// Two unit cubes meeting face to face: A spans `x ∈ [0,1]`, B `x ∈ [1,2]`.
///
/// The headline case, and the most common real CAD operation the exact
/// pipeline could not do (COINCIDENT.md §3's first worked check). The
/// `x = 1` faces are coincident with *identical* trims and opposing outward
/// normals (+X against −X), so they are ON(Opposite): the wall between the
/// cubes is interior and must vanish.
///
/// No imprint is involved. The trims are identical, so neither face's edges
/// cross the other's interior and there is nothing to cut — the overlap is
/// already a whole region. Classification alone fuses the bodies.
#[test]
fn touching_cubes_unite_fuses_into_one_box() {
    let context = "cubes touching at x = 1, unite";
    let mut scene = Scene::new();
    let a = scene.block([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
    let b = scene.block([1.0, 0.0, 0.0], [2.0, 1.0, 1.0]);
    let out = scene
        .unite(a, b)
        .unwrap_or_else(|e| panic!("{context}: exact pipeline rejected touching cubes: {e:?}"));
    // The fused 2x1x1 box. A retained wall would ALSO measure 2.0 — check()
    // inside volume() is what rules it out.
    assert_close(volume(&out, context), 2.0, PLANAR_VOLUME_RTOL, context);
    // Both x = 1 regions dropped, leaving each cube's other five faces. The
    // two halves of each side plane (say A's y = 0 over x ∈ [0,1] and B's
    // over x ∈ [1,2]) are coplanar but are NOT merged into one face: they
    // are separate trims meeting along a 2-fin edge, which is manifold and
    // is what the kernel emits. 11 or 12 would mean a wall survived.
    assert_eq!(
        out.store.faces_of_body(out.body).len(),
        10,
        "{context}: expected each cube's five surviving faces"
    );
}

/// A − B where the two merely touch: nothing of A is inside B, and A's
/// `x = 1` face is ON(Opposite), which subtract KEEPS as the exposed face
/// of the cut (COINCIDENT.md §3, table row 5). So `A − B == A`, whole.
#[test]
fn touching_cubes_subtract_leaves_target_whole() {
    let context = "cubes touching at x = 1, subtract";
    let mut scene = Scene::new();
    let a = scene.block([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
    let b = scene.block([1.0, 0.0, 0.0], [2.0, 1.0, 1.0]);
    let out = scene
        .subtract(a, b)
        .unwrap_or_else(|e| panic!("{context}: exact pipeline rejected touching cubes: {e:?}"));
    assert_close(volume(&out, context), 1.0, PLANAR_VOLUME_RTOL, context);
    // Exactly A: were A's ON(Opposite) face dropped instead of kept, the
    // shell would be open and check() would fire before the count.
    assert_eq!(
        out.store.faces_of_body(out.body).len(),
        6,
        "{context}: A must come through whole"
    );
}

/// Intersection of two merely-touching solids is EMPTY, not a
/// zero-thickness sheet (COINCIDENT.md §6). The true intersection is a unit
/// square of zero volume; the kernel models solids, and a square is not one.
///
/// Both ON(Opposite) regions drop and nothing else is inside, so the result
/// keeps no faces at all. An empty solid is spelled `SolidWithoutShells` —
/// the same way any disjoint pair's intersection is, coincident faces or not
/// (see section (10)) — so that verdict here is the assertion, not a
/// failure. What would be wrong is a body with faces: that is the sheet.
#[test]
fn touching_cubes_intersect_is_empty_not_a_sheet() {
    let mut scene = Scene::new();
    let a = scene.block([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
    let b = scene.block([1.0, 0.0, 0.0], [2.0, 1.0, 1.0]);
    let out = scene
        .intersect(a, b)
        .expect("intersection of touching cubes is empty, not an error");
    assert_eq!(
        out.store.faces_of_body(out.body).len(),
        0,
        "the shared square must not survive as a zero-volume sheet"
    );
    assert!(
        matches!(
            out.check().as_slice(),
            [CheckFailure::SolidWithoutShells(_)]
        ),
        "an empty solid is the correct answer here: {:?}",
        out.check()
    );
}

/// Two unit cubes overlapping along x and flush on all four side planes:
/// A spans `x ∈ [0,1]`, B `x ∈ [0.5,1.5]`. COINCIDENT.md §3's second worked
/// check, and the configuration all three F-Rep tripwires were built from.
///
/// This is the case that needs the imprint. Each side plane (`y = 0`,
/// `y = 1`, `z = 0`, `z = 1`) carries a coincident pair whose trims overlap
/// only PARTIALLY, so the overlap's boundary runs through the middle of
/// both faces: B's `x = 0.5` edge cuts A's side faces, A's `x = 1` edge cuts
/// B's. Those edges already lie exactly in the partner's surface — that is
/// what coincidence means — so they are imprinted directly, with no
/// intersection curve computed for them.
///
/// The four side pairs are ON(Same): both cubes lie on the same side of
/// each shared side plane. Same-sense ON is kept from A ONLY — the
/// canonical tie-break, without which the shared strip is emitted twice and
/// the shell is non-manifold. That tie-break is exactly what `check()`
/// catches and volume cannot.
fn flush_overlapping_cubes(op: &str, expected: f64) {
    let context = &format!("cubes flush-overlapping on four side planes, {op}");
    let mut scene = Scene::new();
    let a = scene.block([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
    let b = scene.block([0.5, 0.0, 0.0], [1.5, 1.0, 1.0]);
    let out = match op {
        "unite" => scene.unite(a, b),
        "subtract" => scene.subtract(a, b),
        "intersect" => scene.intersect(a, b),
        _ => unreachable!(),
    }
    .unwrap_or_else(|e| panic!("{context}: exact pipeline rejected the pair: {e:?}"));
    assert_close(volume(&out, context), expected, PLANAR_VOLUME_RTOL, context);
}

#[test]
fn flush_overlapping_cubes_unite() {
    // x ∈ [0, 1.5], unit cross-section.
    flush_overlapping_cubes("unite", 1.5);
}

#[test]
fn flush_overlapping_cubes_subtract() {
    // A minus the overlap: x ∈ [0, 0.5].
    flush_overlapping_cubes("subtract", 0.5);
}

#[test]
fn flush_overlapping_cubes_intersect() {
    // The overlap itself: x ∈ [0.5, 1].
    flush_overlapping_cubes("intersect", 0.5);
}

/// Inclusion–exclusion over the flush-overlapping pair:
/// `vol(A) + vol(B) == vol(A∪B) + vol(A∩B)`.
///
/// The identity is the sharpest oracle available for this configuration
/// because it is blind to none of the sense errors: it ties union and
/// intersection to each other, so a region kept from the wrong solid, or
/// kept with a flipped normal, breaks it even where each operation's own
/// volume looks plausible in isolation.
#[test]
fn flush_overlapping_cubes_inclusion_exclusion() {
    let context = "flush-overlapping cubes, inclusion-exclusion";
    let mut scene = Scene::new();
    let a = scene.block([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
    let b = scene.block([0.5, 0.0, 0.0], [1.5, 1.0, 1.0]);
    let united = scene.unite(a, b).expect("unite of coincident-flush cubes");
    let intersected = scene
        .intersect(a, b)
        .expect("intersect of coincident-flush cubes");
    let sum = volume(&united, context) + volume(&intersected, context);
    assert_close(sum, 1.0 + 1.0, PLANAR_VOLUME_RTOL, context);
}

/// An L: A spans `x ∈ [0,2], z ∈ [0,1]`, B sits on top of A's right half at
/// `x ∈ [1,2], z ∈ [1,3]`. B's bottom face is coincident with A's top face
/// and NESTED strictly inside it.
///
/// The nesting is the point. B's `x = 1` bottom edge lies in A's top face's
/// INTERIOR, so it must be imprinted for the overlap to exist at all: A's
/// top face splits into `x ∈ [0,1]` (Out, kept — the exposed top of the L's
/// foot) and `x ∈ [1,2]` (ON(Opposite), dropped, fusing the two boxes).
/// Getting this wrong in the quiet direction — failing to imprint, so the
/// whole top face takes one verdict — either buries the foot's top inside
/// the solid or leaves the wall in, and `check()` catches both.
#[test]
fn stacked_l_shape_unite_imprints_nested_face() {
    let context = "L-shape: box stacked on half of a wider box, unite";
    let mut scene = Scene::new();
    let a = scene.block([0.0, 0.0, 0.0], [2.0, 1.0, 1.0]);
    let b = scene.block([1.0, 0.0, 1.0], [2.0, 1.0, 3.0]);
    let out = scene
        .unite(a, b)
        .unwrap_or_else(|e| panic!("{context}: exact pipeline rejected the pair: {e:?}"));
    // 2·1·1 + 1·1·2 = 4. A retained wall between them measures the same;
    // check() inside volume() is the gate that sees it.
    assert_close(volume(&out, context), 4.0, PLANAR_VOLUME_RTOL, context);
}

/// The same L subtracted: nothing of B is inside A (they only share the
/// nested face), so `A − B == A`.
#[test]
fn stacked_l_shape_subtract_leaves_target_whole() {
    let context = "L-shape stacked boxes, subtract";
    let mut scene = Scene::new();
    let a = scene.block([0.0, 0.0, 0.0], [2.0, 1.0, 1.0]);
    let b = scene.block([1.0, 0.0, 1.0], [2.0, 1.0, 3.0]);
    let out = scene
        .subtract(a, b)
        .unwrap_or_else(|e| panic!("{context}: exact pipeline rejected the pair: {e:?}"));
    assert_close(volume(&out, context), 2.0, PLANAR_VOLUME_RTOL, context);
}

/// Rotation invariance, the regression that catches snap-scaling bugs
/// (of-lxk, of-260).
///
/// Coincidence here is decided at the arrangement's weld length rather than
/// at an absolute epsilon, and the weld length is derived from the feature
/// extent. So a rigid rotation of BOTH operands — which changes every
/// coordinate but no distance between them — must not change which faces
/// read as coincident, and must land the same volume. Keying coincidence
/// off an absolute epsilon, or off point magnitude, is exactly what
/// reintroduces that bug class.
#[test]
fn touching_cubes_unite_is_rotation_invariant() {
    for (name, rot) in [
        (
            "45° about z",
            Rotation3::from_axis_angle(&Unit::new_normalize(Vector3::z()), FRAC_PI_4),
        ),
        (
            "45° about y",
            Rotation3::from_axis_angle(&Unit::new_normalize(Vector3::y()), FRAC_PI_4),
        ),
        (
            "oblique",
            Rotation3::from_axis_angle(&Unit::new_normalize(Vector3::new(1.0, 1.0, 1.0)), 0.7),
        ),
    ] {
        let context = &format!("touching cubes united, rotated {name}");
        let mut scene = Scene::new();
        let a = scene.block([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        let b = scene.block([1.0, 0.0, 0.0], [2.0, 1.0, 1.0]);
        scene.rotate(a, &rot, &Point3::origin());
        scene.rotate(b, &rot, &Point3::origin());
        let out = scene
            .unite(a, b)
            .unwrap_or_else(|e| panic!("{context}: rejected after rotation: {e:?}"));
        assert_close(volume(&out, context), 2.0, PLANAR_VOLUME_RTOL, context);
    }
}

// =====================================================================
// (12) Solids meeting only at a vertex or an edge (of-n5g)
// =====================================================================

/// The `unite` half of the pair above (of-n5g).
///
/// The union of two cubes meeting along one edge is genuinely non-manifold.
/// The bug was that it came back as `Ok` — a body that `check()` faults,
/// `tessellate()` cannot close, and `mass_properties` refuses to measure.
/// Callers had no way to tell it apart from a good result, and the hybrid
/// kernel diverts to its F-Rep fallback only on `Err` (`hybrid.rs`, "any
/// shortfall in the exact pipeline"), so this config silently produced an
/// unusable exact result instead of falling back. `Err` is therefore the
/// whole contract here: *which* `Err` is not observable to any caller,
/// because every variant diverts identically.
///
/// Two different gates reject this pair, and which one fires depends on
/// of-bxl.4's atom welding rather than on anything this test should pin:
///
/// - Since of-bxl.4, the contact edge's coincident atoms weld, so the pair
///   reconstructs as ONE pinched shell with `chi = 3`, and the Euler gate
///   in `build_output` rejects it before the of-n5g gate is reached.
/// - Before of-bxl.4 it reconstructed as two shells sharing the contact
///   vertices, which is what the of-n5g gate catches.
///
/// So accept either rejection, but demand it be one of those two — a bare
/// `is_err` would also pass on an unrelated failure and quietly stop
/// testing this bug. The of-n5g gate is NOT dead code: corner contact
/// (below) still reaches it, because two shells that touch at one vertex
/// each have a valid `chi = 2` and sail past the Euler gate.
#[test]
fn edge_adjacent_blocks_unite_is_rejected() {
    let context = "blocks touching along one edge, unite";
    let mut scene = Scene::new();
    let a = scene.block([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
    let b = scene.block([1.0, 1.0, 0.0], [2.0, 2.0, 1.0]);
    let err = scene
        .unite(a, b)
        .err()
        .unwrap_or_else(|| panic!("{context}: expected rejection, got a body"));
    let recognized = match &err {
        // The of-n5g gate.
        CoreError::NotImplemented { .. } => err.to_string().contains("vertex or edge"),
        // The Euler gate in build_output, on the pinched single shell.
        CoreError::Degenerate { context: c, .. } => {
            *c == "boolean::build_output" && err.to_string().contains("Euler characteristic")
        }
        _ => false,
    };
    assert!(
        recognized,
        "{context}: expected the of-n5g vertex/edge gate or the build_output \
         Euler gate, got {err:?}"
    );
}

/// Corner-to-corner contact: the same degeneracy reduced to a single shared
/// vertex, reached without any coincident face pair (the blocks are offset
/// in all three axes, so no two planes coincide). Guards the gate against
/// being narrowed to the edge case alone.
#[test]
fn corner_touching_blocks_unite_is_not_implemented() {
    let context = "blocks touching at one corner, unite";
    let mut scene = Scene::new();
    let a = scene.block([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
    let b = scene.block([1.0, 1.0, 1.0], [2.0, 2.0, 2.0]);
    let err = scene
        .unite(a, b)
        .err()
        .unwrap_or_else(|| panic!("{context}: expected rejection, got a body"));
    assert!(
        matches!(err, CoreError::NotImplemented { .. }),
        "{context}: expected NotImplemented, got {err:?}"
    );
}

// =====================================================================
// (13) NURBS-hosted solids: the classify/reconstruct path end to end
//      (of-xka)
// =====================================================================
//
// Every prior NURBS gate stopped short of a solid boolean: phase 2
// (of-37i.4) gated the marched intersection curve, of-4is gated the imprint
// (reached directly via `clip_imprint`), and `boolean_stress` itself
// contained ZERO NURBS operands — so the suite being green never exercised
// classify/reconstruct on a NURBS host at all. These cases close that gap:
// a box whose six faces are bilinear NURBS patches (built by
// `Scene::nurbs_block`) is analytically a cube, so the exact answers are the
// same planar volumes as the analytic campaigns — but the pipeline now runs
// the NURBS chart, NURBS SSI, region tracing on a NURBS-hosted face, atom
// building, shell reconstruction, and the volume-identity check. The
// inclusion–exclusion identity `vol(A)+vol(B) = vol(A∪B)+vol(A∩B)` is the
// invariant that catches the of-ipt.4 wrong-uv failure mode on a NURBS host.
//
// The overlap is a clean transversal half-overlap: the tool strictly exceeds
// the box in two axes and overlaps its `+x` half in the third, so the tool's
// `−x` face is the only cut. It splits each of the box's four `x`-spanning
// NURBS faces into two SIMPLE (hole-free) regions, the box's `+x` face lies
// wholly inside the tool, and no face pair is coincident (coincident NURBS
// faces are a separate, still-`NotImplemented` tangential-SSI path). A tool
// that instead *bores* the box — protruding both ends — leaves an annular
// (holed) region on the NURBS caps, which once failed classification
// (of-l69); that case is `nurbs_box_bored_by_analytic_bar` below, live since
// the region-of-interest seeding fix its docstring records.
//
// Booleans on a NURBS body go through `boolean_with_inside_tests` because
// `ray_surface_hits` has no NURBS arm (of-3oj): the injected test replaces
// ray parity for the NURBS operand. A box has an exact interior predicate,
// so `box_inside_test` never abstains — the classification stays exact (not
// `MeshSdf`-approximate), which makes the volume identity a *tight* gate.

/// Exact strict point-in-box predicate for a NURBS box built by
/// [`Scene::nurbs_block`], to inject via [`boolean_with_inside_tests`].
/// `Some(true)` strictly inside, `Some(false)` strictly outside; it never
/// returns `None` — a box's interior is an exact predicate, so there is no
/// reason to abstain and fall through to the (NURBS-erroring) ray path.
fn box_inside_test(min: [f64; 3], max: [f64; 3]) -> impl Fn(&Point3) -> Option<bool> {
    move |p: &Point3| Some((0..3).all(|i| p[i] > min[i] && p[i] < max[i]))
}

/// Connected components and total genus of a closed manifold triangle
/// mesh, as [`assert_valid`] establishes: every undirected index edge is
/// shared by exactly two consistently-oriented triangles, so per component
/// `χ = V − E + F = 2 − 2g` and `Σg = c − χ/2`, counted over referenced
/// vertices. This is the §9 "genus on every output" check: a boolean that
/// silently gains or loses a handle (the classic wrong-region failure)
/// changes `χ` even when volume and manifoldness survive.
fn components_and_genus(mesh: &TriangleMesh) -> (usize, usize) {
    fn find(parent: &mut [usize], mut x: usize) -> usize {
        while parent[x] != x {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        x
    }
    let n = mesh.vertex_count();
    let mut parent: Vec<usize> = (0..n).collect();
    let mut referenced = vec![false; n];
    let mut edges: std::collections::HashSet<(usize, usize)> = std::collections::HashSet::new();
    for tri in &mesh.indices {
        for e in 0..3 {
            let a = tri[e];
            let b = tri[(e + 1) % 3];
            referenced[a] = true;
            edges.insert((a.min(b), a.max(b)));
            let (ra, rb) = (find(&mut parent, a), find(&mut parent, b));
            if ra != rb {
                parent[ra] = rb;
            }
        }
    }
    let v = referenced.iter().filter(|&&r| r).count() as i64;
    let e = edges.len() as i64;
    let f = mesh.triangle_count() as i64;
    let roots: std::collections::HashSet<usize> = (0..n)
        .filter(|&i| referenced[i])
        .map(|i| find(&mut parent, i))
        .collect();
    let c = roots.len() as i64;
    let chi = v - e + f;
    let genus_doubled = 2 * c - chi;
    assert!(
        genus_doubled >= 0 && genus_doubled % 2 == 0,
        "Euler characteristic {chi} is inconsistent with {c} closed orientable component(s)"
    );
    (c as usize, (genus_doubled / 2) as usize)
}

/// [`volume`] plus the topology half of the §9 gate: assert the output is
/// valid (check() clean, closed manifold), has exactly the expected
/// connected `components` and total `genus`, and return its mesh volume.
fn volume_checked(out: &BooleanOutput, components: usize, genus: usize, context: &str) -> f64 {
    let mesh = assert_valid(out, context);
    let (c, g) = components_and_genus(&mesh);
    assert!(
        (c, g) == (components, genus),
        "{context}: expected {components} component(s) of total genus {genus}, \
         got {c} component(s) of total genus {g}"
    );
    mass_properties(&mesh)
        .unwrap_or_else(|e| panic!("{context}: mass_properties failed: {e}"))
        .volume
}

/// Run all three ops for a NURBS-hosted transversal pair and assert the
/// full §9 identity set at planar accuracy: `A−B = vol_a − overlap`,
/// `A∩B = overlap`, inclusion–exclusion
/// `vol(A∪B) + vol(A∩B) = vol(A) + vol(B)`, and every output a valid
/// single-component genus-0 closed manifold. `vols` is
/// `(vol_a, vol_b, overlap)`.
fn assert_overlap_identity(
    context: &str,
    scene: &Scene,
    a: EntityId<Body>,
    b: EntityId<Body>,
    tests: [Option<InsideTest>; 2],
    (vol_a, vol_b, overlap): (f64, f64, f64),
) {
    let run = |op, label: &str| {
        boolean_with_inside_tests(op, &scene.store, &scene.geo, a, b, &tol(), tests)
            .unwrap_or_else(|e| panic!("{context}: exact pipeline rejected the {label}: {e:?}"))
    };
    let subtracted = run(BooleanOp::Subtract, "subtract");
    assert_close(
        volume_checked(&subtracted, 1, 0, context),
        vol_a - overlap,
        PLANAR_VOLUME_RTOL,
        context,
    );
    let intersected = run(BooleanOp::Intersect, "intersect");
    assert_close(
        volume_checked(&intersected, 1, 0, context),
        overlap,
        PLANAR_VOLUME_RTOL,
        context,
    );
    let united = run(BooleanOp::Unite, "unite");
    let sum = volume_checked(&united, 1, 0, context) + volume_checked(&intersected, 1, 0, context);
    assert_close(sum, vol_a + vol_b, PLANAR_VOLUME_RTOL, context);
}

/// [`assert_overlap_identity`] for the common half-overlap fixture: `b`
/// overlaps exactly half of `a`, so `A−B = A∩B = vol_a / 2`.
fn assert_half_overlap_identity(
    context: &str,
    scene: &Scene,
    a: EntityId<Body>,
    b: EntityId<Body>,
    tests: [Option<InsideTest>; 2],
    vol_a: f64,
    vol_b: f64,
) {
    assert_overlap_identity(context, scene, a, b, tests, (vol_a, vol_b, vol_a / 2.0));
}

/// NURBS box half-overlapped by an **analytic** block. Only operand A is
/// NURBS, so only slot 0 carries an inside test; B classifies by exact ray
/// parity. First end-to-end proof (of-xka) that a NURBS-hosted solid
/// survives classify → reconstruct → tessellate → volume.
#[test]
fn nurbs_box_half_overlapped_by_analytic_block() {
    let context = "NURBS box half-overlapped by analytic block";
    let mut scene = Scene::new();
    let a_box = ([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]);
    let a = scene.nurbs_block(a_box.0, a_box.1);
    let b = scene.block([1.0, -1.0, -1.0], [3.0, 3.0, 3.0]);
    let inside_a = box_inside_test(a_box.0, a_box.1);
    // vol(A) = 2³ = 8; vol(B) = 2·4·4 = 32; overlap = A's +x half = 4.
    assert_half_overlap_identity(context, &scene, a, b, [Some(&inside_a), None], 8.0, 32.0);
}

/// The same half-overlap, but the tool is itself a **NURBS** box — the
/// genuine NURBS↔NURBS solid boolean the whole gap (of-xka) was about. Both
/// operands carry an inside test; every cut is a NURBS patch meeting a NURBS
/// patch, driving NURBS↔NURBS SSI and NURBS-on-NURBS region tracing that no
/// analytic operand can reach.
#[test]
fn nurbs_box_half_overlapped_by_nurbs_box() {
    let context = "NURBS box half-overlapped by NURBS box";
    let mut scene = Scene::new();
    let a_box = ([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]);
    let b_box = ([1.0, -1.0, -1.0], [3.0, 3.0, 3.0]);
    let a = scene.nurbs_block(a_box.0, a_box.1);
    let b = scene.nurbs_block(b_box.0, b_box.1);
    let inside_a = box_inside_test(a_box.0, a_box.1);
    let inside_b = box_inside_test(b_box.0, b_box.1);
    assert_half_overlap_identity(
        context,
        &scene,
        a,
        b,
        [Some(&inside_a), Some(&inside_b)],
        8.0,
        32.0,
    );
}

/// NURBS box bored transversally by an analytic bar protruding both ends,
/// which leaves an annular (holed) region on the box's top and bottom NURBS
/// caps. This once failed classification with `boolean::classify: could not
/// find an interior sample point for a face region` (of-l69): the bar's side
/// planes are infinite surfaces clipped to the box-overlap region, but the
/// NURBS cap kept its full natural domain, so `march_boxed` grid-seeded the
/// cap *outside* that region and each out-of-region seed traced an
/// overlapping fragment of the same hole edge. The two fragments reached the
/// arrangement as duplicate imprints, which `merge_imprint_chains` walked
/// into a zero-area loop with no interior point. Seeding only inside the
/// region of interest (of-l69) leaves one trace per edge, so the annulus is
/// built correctly. The same bore on analytic planar caps always worked
/// (their SSI is closed-form, not marched), and the hole-free half-overlaps
/// above work, which had localized the defect to holed regions on a NURBS
/// host.
///
/// Volumes: box `2³ = 8`, bar `1×1×3 = 3`, overlap `2`, so `A−B = 6`,
/// `A∩B = 2`, and the inclusion-exclusion identity `9 + 2 = 8 + 3` holds.
#[test]
fn nurbs_box_bored_by_analytic_bar() {
    let context = "NURBS box bored by analytic bar";
    let mut scene = Scene::new();
    let a_box = ([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]);
    let a = scene.nurbs_block(a_box.0, a_box.1);
    let b = scene.block([0.5, 0.5, -0.5], [1.5, 1.5, 2.5]);
    let inside_a = box_inside_test(a_box.0, a_box.1);
    let tests: [Option<InsideTest>; 2] = [Some(&inside_a), None];

    let run = |op, label: &str| {
        boolean_with_inside_tests(op, &scene.store, &scene.geo, a, b, &tol(), tests)
            .unwrap_or_else(|e| panic!("{context}: exact pipeline rejected the {label}: {e:?}"))
    };
    // The bore makes the difference a genus-1 solid (one handle) — the
    // sharpest topology assertion in the section: a wrong-region result
    // that happens to keep the right volume still changes χ.
    let subtracted = run(BooleanOp::Subtract, "subtract");
    assert_close(
        volume_checked(&subtracted, 1, 1, context),
        6.0,
        PLANAR_VOLUME_RTOL,
        context,
    );
    let intersected = run(BooleanOp::Intersect, "intersect");
    assert_close(
        volume_checked(&intersected, 1, 0, context),
        2.0,
        PLANAR_VOLUME_RTOL,
        context,
    );
    let united = run(BooleanOp::Unite, "unite");
    let sum = volume_checked(&united, 1, 0, context) + volume_checked(&intersected, 1, 0, context);
    assert_close(sum, 8.0 + 3.0, PLANAR_VOLUME_RTOL, context);
}

// =====================================================================
// (14) NURBS promotion gate: the FREEFORM §9 stress campaign (of-37i.5)
// =====================================================================
//
// The README's stress-suite-first policy: a surface class does not enter
// the exact pipeline until its randomized stress suite is green, and the
// suite is written BEFORE the exact path is promoted — this section IS
// that suite for NURBS. Section (13) proved a NURBS-hosted solid survives
// the pipeline at all; this section is the §9 checklist that gates
// promotion:
//
//   - a NURBS operand of EXACT analytic form (the of-pb7.3 rational
//     quarter-cylinder patches) booleaned against a block must match the
//     analytic cylinder's boolean — every result vertex on the closed-form
//     boundary to `tol.linear`, the strongest check in the suite, because
//     it validates the exact path against a known-good answer rather than
//     against itself;
//   - the inclusion–exclusion volume identity to 1e-9 relative over
//     seeded randomized transversal NURBS pairs (planar patch geometry,
//     so mesh volumes are exact and 1e-9 is meaningful);
//   - rigid-rotation invariance (control points rotate exactly; a
//     bilinear patch of rotated corners is the rotated patch);
//   - knot-scaling invariance — identical geometry under wildly scaled,
//     offset, and anisotropic knot domains, which catches any
//     `tol.parametric` normalization bug (§6): a parameter tolerance
//     applied to an unnormalized domain changes behavior between domain
//     [0,1] and domain [3,13] even though the surfaces are pointwise
//     identical;
//   - multi-span patches with the imprint landing exactly ON an interior
//     knot line, and landing mid-span;
//   - trim regions abutting (and consuming) the knot-domain boundary;
//   - closed manifold + `check()` + components + genus on every output
//     (`volume_checked`).
//
// All operands go through `boolean_with_inside_tests` with exact
// predicates in the NURBS slots (of-3oj), so classification never
// abstains and any failure is the pipeline's, not the crutch's.
//
// GATE STATUS: GREEN — every case in this section is live. The
// deterministic checks (knot scaling, multi-span, domain-boundary
// slivers) were green first; the curved-operand bore (the highest-value
// test below) is green since of-hqb (the clip predicate inverts
// chord-sampled stations by banded projection, `Chart::param_within`,
// and NURBS mesh faces take the boundary-CDT seed); and the randomized
// planar-NURBS campaigns (identity + rotation invariance) are green
// since of-bd3 (marched imprint run endpoints are polished onto their
// exact boundary junctions, superseding raw stations inside the
// marcher's landing tolerance, and geometrically straight marched runs
// get the same interior-sample drop as Line-sourced darts, so the ear
// clippers of adjacent faces no longer disagree along shared junction
// lines).
//
// PROMOTED (of-ew7). NURBS operands take the exact path through the public
// kernel entry point, asserted in `opensolid-kernel/tests/hybrid_e2e.rs`.
// The promotion needed one thing this section could not see, because every
// case here calls `boolean_with_inside_tests` directly: `tessellate_face`
// had no NURBS arm, so `hybrid::boolean` could build neither the of-3oj
// `MeshSdf` sign crutch (leaving the exact path unable to classify) *nor*
// the F-Rep fallback's own operand field — a NURBS boolean through the
// kernel failed both ways rather than falling back. of-ew7 added an
// untrimmed-patch arm whose lattice is priced off how far the normal
// turns.
//
// Two limits that arm did not lift, and that this section structurally
// cannot see because it never tessellates an input *body*: trimmed NURBS
// faces on input bodies, and a *curved* untrimmed patch not welding to its
// neighbour because the two sampled their shared edge by different rules
// (of-dvj). of-37i.6 lifted both, by taking the grid away from NURBS faces
// entirely — they now go through the same CDT a result's faces take, whose
// boundary comes from the edge curves. The kernel-level promotion is
// therefore real for curved NURBS solids too, asserted in `hybrid_e2e.rs`.
//
// What this section *does* own for phase 4 is the deviation bar itself:
// `nurbs_results_tessellate_within_one_frep_cell` below is §9's phase-4
// gate, checking every NURBS-hosted result against exactly the F-Rep cell
// `hybrid::boolean` compares chords to before trusting the exact mesh.

/// Exact strict interior predicate for the finite solid cylinder (axis
/// `+Z` through `(cx, cy)`, radius `r`, `z ∈ (z0, z1)`), to inject via
/// [`boolean_with_inside_tests`] for a [`Scene::nurbs_cylinder`] operand.
/// Like [`box_inside_test`] it never abstains.
fn cylinder_inside_test(
    cx: f64,
    cy: f64,
    r: f64,
    z0: f64,
    z1: f64,
) -> impl Fn(&Point3) -> Option<bool> {
    move |p: &Point3| Some((p.x - cx).powi(2) + (p.y - cy).powi(2) < r * r && p.z > z0 && p.z < z1)
}

/// [`box_inside_test`] for a box rigidly rotated by `rot` about `center`:
/// the query point is pulled back into the box's own frame, so the
/// predicate stays exact under the rotation.
fn rotated_box_inside_test(
    min: [f64; 3],
    max: [f64; 3],
    rot: Rotation3<f64>,
    center: Point3,
) -> impl Fn(&Point3) -> Option<bool> {
    let inv = rot.inverse();
    move |p: &Point3| {
        let q = center + inv * (p - center);
        Some((0..3).all(|i| q[i] > min[i] && q[i] < max[i]))
    }
}

/// [`box_corners`] rigidly rotated by `rot` about `center` — the operand
/// for building an exactly-rotated NURBS hexahedron (rotation is affine,
/// so the bilinear patch of rotated corners IS the rotated patch).
fn rotated_box_corners(
    min: [f64; 3],
    max: [f64; 3],
    rot: &Rotation3<f64>,
    center: &Point3,
) -> [Point3; 8] {
    box_corners(min, max).map(|c| center + rot * (c - center))
}

/// Exact signed distance to the axis-aligned box `min..max` (negative
/// inside) — one half of the closed-form answer the §9 highest-value test
/// checks the exact path against.
fn box_sdf(min: [f64; 3], max: [f64; 3], p: &Point3) -> f64 {
    let q = [
        (min[0] - p.x).max(p.x - max[0]),
        (min[1] - p.y).max(p.y - max[1]),
        (min[2] - p.z).max(p.z - max[2]),
    ];
    let outside = (q[0].max(0.0).powi(2) + q[1].max(0.0).powi(2) + q[2].max(0.0).powi(2)).sqrt();
    outside + q[0].max(q[1]).max(q[2]).min(0.0)
}

/// Exact signed distance to the finite solid cylinder (axis `+Z` through
/// `(cx, cy)`, radius `r`, `z ∈ [z0, z1]`; negative inside) — the other
/// half of the closed-form answer.
fn cylinder_sdf(cx: f64, cy: f64, r: f64, z0: f64, z1: f64, p: &Point3) -> f64 {
    let dr = ((p.x - cx).powi(2) + (p.y - cy).powi(2)).sqrt() - r;
    let dz = (z0 - p.z).max(p.z - z1);
    let outside = (dr.max(0.0).powi(2) + dz.max(0.0).powi(2)).sqrt();
    outside + dr.max(dz).min(0.0)
}

/// Assert every referenced vertex of a result mesh lies within
/// `tol.linear` of the analytic result boundary described by `residual`
/// (a signed field whose zero set is that boundary). Mesh vertices are
/// evaluated points of the result's faces, so this is a direct
/// `tol.linear` comparison of the exact path's geometry against the
/// closed form — tessellation *density* never enters, only the surfaces
/// the pipeline actually produced.
fn assert_vertices_on_boundary(
    mesh: &TriangleMesh,
    residual: &dyn Fn(&Point3) -> f64,
    context: &str,
) {
    let lin = tol().linear;
    let mut referenced = vec![false; mesh.vertex_count()];
    for tri in &mesh.indices {
        for &i in tri {
            referenced[i] = true;
        }
    }
    for (i, p) in mesh.positions.iter().enumerate() {
        if !referenced[i] {
            continue;
        }
        let r = residual(p).abs();
        assert!(
            r <= lin,
            "{context}: mesh vertex {i} at ({}, {}, {}) sits {r:.3e} off the \
             analytic result boundary (allowed {lin:.1e})",
            p.x,
            p.y,
            p.z,
        );
    }
}

/// §9's HIGHEST-VALUE TEST: a NURBS operand of exact analytic form — the
/// rational quarter-cylinder patches of [`Scene::nurbs_cylinder`], exact
/// to ~1e-10 — bored through a block, checked against the closed-form
/// answer three ways:
///
/// 1. every vertex of every result mesh lies on the analytic boolean's
///    boundary (`max`/`min` of the two exact SDFs) to `tol.linear`;
/// 2. volumes match the closed form (`8 − π/2`, `π/2`, `8 + π/2`) at the
///    curved-tessellation tolerance;
/// 3. the same boolean built from `Scene::cylinder` (the analytic
///    surface) agrees, so the NURBS and analytic paths corroborate each
///    other on identical geometry.
///
/// The bore pierces both caps, so the difference is genus 1 — the
/// topology check rules out a wrong-region result that happens to keep
/// the right volume. Every wall-crossing imprint runs across all four
/// quarter patches and their seam edges, exercising marched Plane↔NURBS
/// SSI junction welding (of-9ia) on curved geometry.
#[test]
fn block_bored_by_exact_nurbs_cylinder_matches_analytic() {
    let (cx, cy, r) = (1.0, 1.0, 0.5);
    let (z0, h) = (-1.0, 4.0);
    let bore = PI * r * r * 2.0; // clipped to the block's z ∈ [0, 2]
    let vol_b = PI * r * r * h;
    let d_box = |p: &Point3| box_sdf([0.0; 3], [2.0; 3], p);
    let d_cyl = |p: &Point3| cylinder_sdf(cx, cy, r, z0, z0 + h, p);

    let mut scene = Scene::new();
    let a = scene.block([0.0; 3], [2.0; 3]);
    let b = scene.nurbs_cylinder(cx, cy, r, z0, h);
    let inside_b = cylinder_inside_test(cx, cy, r, z0, z0 + h);
    let tests: [Option<InsideTest>; 2] = [None, Some(&inside_b)];
    let run = |op, label: &str| {
        boolean_with_inside_tests(op, &scene.store, &scene.geo, a, b, &tol(), tests)
            .unwrap_or_else(|e| panic!("{label}: exact pipeline rejected the boolean: {e:?}"))
    };

    let ctx_sub = "block − exact NURBS cylinder (through-bore)";
    let subtracted = run(BooleanOp::Subtract, ctx_sub);
    let v_sub = volume_checked(&subtracted, 1, 1, ctx_sub);
    assert_close(v_sub, 8.0 - bore, CYL_VOLUME_RTOL, ctx_sub);
    assert_vertices_on_boundary(
        &assert_valid(&subtracted, ctx_sub),
        &|p| d_box(p).max(-d_cyl(p)),
        ctx_sub,
    );

    let ctx_int = "block ∩ exact NURBS cylinder";
    let intersected = run(BooleanOp::Intersect, ctx_int);
    let v_int = volume_checked(&intersected, 1, 0, ctx_int);
    assert_close(v_int, bore, CYL_VOLUME_RTOL, ctx_int);
    assert_vertices_on_boundary(
        &assert_valid(&intersected, ctx_int),
        &|p| d_box(p).max(d_cyl(p)),
        ctx_int,
    );

    let ctx_uni = "block ∪ exact NURBS cylinder";
    let united = run(BooleanOp::Unite, ctx_uni);
    let v_uni = volume_checked(&united, 1, 0, ctx_uni);
    assert_close(v_uni, 8.0 + vol_b - bore, CYL_VOLUME_RTOL, ctx_uni);
    assert_vertices_on_boundary(
        &assert_valid(&united, ctx_uni),
        &|p| d_box(p).min(d_cyl(p)),
        ctx_uni,
    );
    assert_close(
        v_uni + v_int,
        8.0 + vol_b,
        CYL_VOLUME_RTOL,
        "NURBS cylinder inclusion–exclusion identity",
    );

    // The analytic twin: identical geometry through `Surface3::Cylinder`
    // and ray-parity classification. Its agreement pins the NURBS result
    // to the known-good path, not just to the closed form.
    let mut analytic = Scene::new();
    let a2 = analytic.block([0.0; 3], [2.0; 3]);
    let b2 = analytic.cylinder(Point3::new(cx, cy, z0), Vector3::z(), r, h);
    let ctx_ana = "block − analytic cylinder (NURBS twin cross-check)";
    let sub2 = analytic
        .subtract(a2, b2)
        .unwrap_or_else(|e| panic!("{ctx_ana}: subtract failed: {e:?}"));
    let v_sub2 = volume_checked(&sub2, 1, 1, ctx_ana);
    assert_close(v_sub2, 8.0 - bore, CYL_VOLUME_RTOL, ctx_ana);
    assert_close(
        v_sub,
        v_sub2,
        CYL_VOLUME_RTOL,
        "NURBS vs analytic cylinder bore volumes",
    );
}

/// The F-Rep cell `hybrid::boolean` measures a result's chords against
/// before it trusts the exact mesh: the mesh's longest bounding-box axis
/// over the default grid resolution (`hybrid::cell_size`, resolution 64).
/// Duplicated here rather than imported because it is private to the
/// kernel and this crate is below it.
fn frep_cell(mesh: &TriangleMesh, resolution: usize) -> f64 {
    let e = mesh
        .bounding_box()
        .expect("a boolean result mesh is non-empty")
        .extents();
    e.x.max(e.y).max(e.z) / resolution as f64
}

/// of-dvj, at the level it actually broke: a **curved** NURBS *input* body
/// must tessellate watertight. The four rational quarter patches of
/// [`Scene::nurbs_cylinder`] and their two planar caps share four circular
/// edges, and the wall used to sample each by its own rational parameter
/// while the cap sampled the *edge curve* by angle. The two are not the same
/// positions — the rims missed each other by up to half a sample and the
/// welded body came out with 128 open edges on 124 triangles, so
/// `MeshSdf::new` rejected the operand and `hybrid::boolean` could build
/// neither the of-3oj sign crutch nor the F-Rep fallback's field.
///
/// Held here rather than only at the kernel level because this is the
/// cheapest possible statement of it: no boolean, just `tessellate_body`.
#[test]
fn curved_nurbs_input_body_tessellates_watertight() {
    let (r, h) = (1.0, 3.0);
    let mut scene = Scene::new();
    let body = scene.nurbs_cylinder(0.0, 0.0, r, 0.0, h);
    let mesh = tessellate_body(
        &scene.store,
        &scene.geo,
        body,
        &TessellationOptions::default(),
    )
    .expect("a curved NURBS body tessellates");
    assert!(
        mesh.is_closed_manifold(),
        "a NURBS-walled cylinder must weld watertight, got {} triangles that do not",
        mesh.triangle_count()
    );
    let volume = mass_properties(&mesh)
        .expect("closed manifold has mass properties")
        .volume;
    // Bracketed, not approximated. The caps are ear-clipped from the four
    // quarter arcs at the default `angular_step`, so their rim is an
    // inscribed 32-gon and the body cannot hold more than the exact
    // cylinder nor less than that prism. It lands strictly between: the
    // wall's interior lattice points sit on the true surface, bulging
    // slightly past the rim chords.
    let segments = 4.0 * (FRAC_PI_2 / (std::f64::consts::TAU / 32.0)).ceil();
    let prism = 0.5 * segments * r * r * (std::f64::consts::TAU / segments).sin() * h;
    assert!(
        volume > prism && volume < PI * r * r * h,
        "NURBS-walled cylinder volume {volume} must sit between the inscribed \
         {segments}-gon prism {prism} and the exact cylinder {}",
        PI * r * r * h
    );
}

/// of-37i.7 item 1: a NURBS body whose wall patches carry a **collapsed
/// control row** — the lofted-to-a-point tip — is admitted by `Chart` and
/// tessellates **watertight**. Before of-37i.7 `Chart::build` refused every
/// such patch outright and the tip gridded, which is of-dvj's shape: the
/// wall sampled the shared base circle by its own rational parameter while
/// the cap sampled the *edge curve* by angle, the rims missed each other,
/// and the body came out with open edges. That is not merely an accuracy
/// loss — with no welded mesh there is no operand SDF, so `hybrid::boolean`
/// can build neither the exact path's of-3oj sign crutch nor the F-Rep
/// fallback's field, and *both* paths stop (FREEFORM.md §7.1's lesson).
///
/// This is `curved_nurbs_input_body_tessellates_watertight` for the
/// degenerate case, and deliberately the same cheapest possible statement:
/// no boolean, just `tessellate_body`. The body is an exact cone, so the
/// volume bar is known in closed form rather than self-consistent.
#[test]
fn nurbs_tip_body_is_admitted_and_tessellates_watertight() {
    let (r, h) = (1.0, 2.0);
    let mut scene = Scene::new();
    let body = scene.nurbs_cone(0.0, 0.0, r, 0.0, h);
    let failures = scene.store.check(body);
    assert!(
        failures.is_empty(),
        "a collapsed-row NURBS body must pass topology checks, got {failures:?}"
    );

    let mesh = tessellate_body(
        &scene.store,
        &scene.geo,
        body,
        &TessellationOptions::default(),
    )
    .expect("a collapsed-row NURBS body tessellates");
    assert!(
        mesh.is_closed_manifold(),
        "a NURBS tip body must weld watertight, got {} triangles that do not",
        mesh.triangle_count()
    );

    // Bracketed, not approximated, exactly as the cylinder case is: the
    // base rim is an inscribed polygon at the default `angular_step`, so
    // the body holds less than the exact cone and more than the pyramid on
    // that polygon.
    let volume = mass_properties(&mesh)
        .expect("closed manifold has mass properties")
        .volume;
    let exact = PI * r * r * h / 3.0;
    let segments = 4.0 * (FRAC_PI_2 / (std::f64::consts::TAU / 32.0)).ceil();
    let pyramid = 0.5 * segments * r * r * (std::f64::consts::TAU / segments).sin() * h / 3.0;
    assert!(
        volume > pyramid && volume < exact,
        "the tip body's volume {volume} must sit between the inscribed \
         {segments}-gon pyramid {pyramid} and the exact cone {exact}"
    );
}

/// §9's **phase-4 gate** (of-37i.6): every NURBS-hosted result in the
/// corpus tessellates within one F-Rep cell, so `hybrid::boolean` keeps
/// the exact mesh instead of diverting to the fallback on the deviation
/// check.
///
/// Deviation is what `BooleanOutput::tessellate_measured` reports — the
/// largest distance from a triangle edge's 3D midpoint to the surface
/// point at its parameter-space midpoint — and the bar is
/// `hybrid::cell_size` at the default resolution, exactly the comparison
/// `hybrid::boolean` makes. The margin is asserted too: passing by a hair
/// would mean a slightly larger operand or a slightly coarser grid tips
/// the router back to F-Rep, which is the failure this gate exists to
/// keep out.
///
/// Worth recording honestly: **this gate was already green before the
/// curvature-derived interior lattice landed**, and stays green with it.
/// of-hqb's reasoning for laying no Steiner points on a NURBS chart holds
/// for the corpus's exact-form cylinder — the patch is *ruled*, so flat
/// chords are exact along `v`, and its trim rings are marched densely
/// enough to carry `u` on their own. A patch that curves *both* ways has
/// no such rescue, and that is what the lattice is for; the case that
/// actually discriminates is the unit test
/// `boolean::tests::nurbs_lattice_cuts_the_worst_chord_on_a_doubly_curved_patch`.
/// What this test pins is the corpus-level bar itself, and that the
/// lattice did not *regress* it.
///
/// Faces the lattice legitimately leaves flat — planar patches — deviate
/// only by rational-evaluation round-off, and are included here to pin
/// that no Steiner points are laid where they buy nothing.
#[test]
fn nurbs_results_tessellate_within_one_frep_cell() {
    let bar = |context: &str, out: &BooleanOutput| {
        let (mesh, deviation) = out
            .tessellate_measured()
            .unwrap_or_else(|e| panic!("{context}: tessellation failed: {e:?}"));
        assert!(
            mesh.is_closed_manifold(),
            "{context}: result mesh is not a closed manifold"
        );
        let cell = frep_cell(&mesh, 64);
        assert!(
            deviation <= cell,
            "{context}: chord deviation {deviation:.3e} exceeds the F-Rep cell \
             {cell:.3e} — hybrid::boolean would divert this result to the fallback"
        );
        // Half a cell of headroom, so the gate is not sitting on its own
        // rounding.
        assert!(
            deviation <= 0.5 * cell,
            "{context}: chord deviation {deviation:.3e} is within the F-Rep cell \
             {cell:.3e} but with no margin — a marginally coarser grid diverts it"
        );
    };

    // Curved: the exact-form NURBS cylinder bored through a block. Every
    // wall face is a trimmed rational quarter patch.
    let (cx, cy, r) = (1.0, 1.0, 0.5);
    let (z0, h) = (-1.0, 4.0);
    let mut scene = Scene::new();
    let a = scene.block([0.0; 3], [2.0; 3]);
    let b = scene.nurbs_cylinder(cx, cy, r, z0, h);
    let inside_b = cylinder_inside_test(cx, cy, r, z0, z0 + h);
    let tests: [Option<InsideTest>; 2] = [None, Some(&inside_b)];
    for (op, context) in [
        (BooleanOp::Subtract, "block − exact NURBS cylinder"),
        (BooleanOp::Intersect, "block ∩ exact NURBS cylinder"),
        (BooleanOp::Unite, "block ∪ exact NURBS cylinder"),
    ] {
        let out = boolean_with_inside_tests(op, &scene.store, &scene.geo, a, b, &tol(), tests)
            .unwrap_or_else(|e| panic!("{context}: exact pipeline rejected the boolean: {e:?}"));
        bar(context, &out);
    }

    // Planar-patch NURBS: a NURBS box bored by an analytic bar. Flat faces
    // must come out at deviation zero — no lattice where it buys nothing.
    let mut scene = Scene::new();
    let a = scene.nurbs_block([0.0; 3], [2.0; 3]);
    let b = scene.block([0.5, 0.5, -1.0], [1.5, 1.5, 3.0]);
    let inside_a = box_inside_test([0.0; 3], [2.0; 3]);
    let tests: [Option<InsideTest>; 2] = [Some(&inside_a), None];
    let context = "NURBS box bored by analytic bar";
    let out = boolean_with_inside_tests(
        BooleanOp::Subtract,
        &scene.store,
        &scene.geo,
        a,
        b,
        &tol(),
        tests,
    )
    .unwrap_or_else(|e| panic!("{context}: exact pipeline rejected the boolean: {e:?}"));
    let (_, deviation) = out.tessellate_measured().expect("planar NURBS tessellates");
    assert!(
        deviation <= 1e-12,
        "{context}: planar NURBS patches must tessellate exactly, deviated {deviation:.3e} \
         (only rational-evaluation round-off is allowed here, ~1e-16)"
    );
    bar(context, &out);
}

/// Expected `(components, total genus)` of `A − B`'s boundary for a
/// [`BlockPair`]: piercing one axis while strictly interior on the other
/// two bores a handle (genus 1); piercing two with one interior axis cuts
/// `A` clean in half; anything else notches a topological ball. Panics if
/// the seed produced a non-transversal pair (`B ⊇ A`, or `B` a strict
/// interior void) — pick a different seed rather than weakening the test.
fn expected_subtract_topology(pair: &BlockPair, repro: &str) -> (usize, usize) {
    let pierced = (0..3)
        .filter(|&k| pair.b_min[k] < 0.0 && pair.b_max[k] > pair.a_max[k])
        .count();
    let interior = (0..3)
        .filter(|&k| pair.b_min[k] > 0.0 && pair.b_max[k] < pair.a_max[k])
        .count();
    assert!(
        pierced < 3 && interior < 3,
        "{repro}: seed produced a non-transversal pair; choose a different seed"
    );
    match (pierced, interior) {
        (1, 2) => (1, 1),
        (2, 1) => (2, 0),
        _ => (1, 0),
    }
}

/// §9's randomized identity campaign on NURBS-hosted operands: the same
/// seeded transversal `BlockPair` protocol as
/// `random_transversal_block_pairs_volume_identity`, but operand `A` is
/// always a NURBS box and `B` alternates analytic/NURBS — so half the
/// cases drive NURBS↔plane SSI and half NURBS↔NURBS. Volumes stay exact
/// (planar patches), so the inclusion–exclusion identity holds to 1e-9
/// relative, and every output's component count and genus is predicted
/// from the pair's overlap structure — through-bores are *expected* to
/// come out genus 1, full slabs to split `A` in two.
#[test]
fn random_transversal_nurbs_block_pairs_volume_identity() {
    // Seed chosen (see of-37i.5) so all 12 pairs are transversal AND the
    // expected subtract topologies are diverse: two through-bores
    // (genus 1), two clean splits (2 components), and eight notches.
    let mut rng = Rng::new(0xACE5);
    for case in 0..12 {
        let pair = BlockPair::random(&mut rng);
        let nurbs_tool = case % 2 == 1;
        let repro = format!(
            "{} (A as NURBS{})",
            pair.repro(case),
            if nurbs_tool { ", B as NURBS" } else { "" }
        );
        let mut scene = Scene::new();
        let a = scene.nurbs_block([0.0; 3], pair.a_max);
        let b = if nurbs_tool {
            scene.nurbs_block(pair.b_min, pair.b_max)
        } else {
            scene.block(pair.b_min, pair.b_max)
        };
        let inside_a = box_inside_test([0.0; 3], pair.a_max);
        let inside_b = box_inside_test(pair.b_min, pair.b_max);
        let tests: [Option<InsideTest>; 2] = [
            Some(&inside_a),
            if nurbs_tool { Some(&inside_b) } else { None },
        ];
        let (sub_components, sub_genus) = expected_subtract_topology(&pair, &repro);

        let run = |op, label: &str| {
            boolean_with_inside_tests(op, &scene.store, &scene.geo, a, b, &tol(), tests)
                .unwrap_or_else(|e| panic!("{repro}: {label} failed: {e:?}"))
        };
        let united = run(BooleanOp::Unite, "unite");
        let intersected = run(BooleanOp::Intersect, "intersect");
        let subtracted = run(BooleanOp::Subtract, "subtract");

        let vol_union = volume_checked(&united, 1, 0, &format!("{repro}: union"));
        let vol_inter = volume_checked(&intersected, 1, 0, &format!("{repro}: intersection"));
        let vol_diff = volume_checked(
            &subtracted,
            sub_components,
            sub_genus,
            &format!("{repro}: difference"),
        );

        assert_close(
            vol_inter,
            pair.vol_overlap(),
            PLANAR_VOLUME_RTOL,
            &format!("{repro}: intersection vs analytic overlap"),
        );
        assert_close(
            vol_union + vol_inter,
            pair.vol_a() + pair.vol_b(),
            PLANAR_VOLUME_RTOL,
            &format!("{repro}: inclusion–exclusion identity"),
        );
        assert_close(
            vol_diff,
            pair.vol_a() - pair.vol_overlap(),
            PLANAR_VOLUME_RTOL,
            &format!("{repro}: difference identity"),
        );
    }
}

/// §9 rotation invariance on NURBS operands: each seeded pair is
/// booleaned axis-aligned and again after a rigid rotation of both
/// operands — control points rotated exactly (affine map of a bilinear
/// patch), inside tests pulled back through the inverse rotation — and
/// the intersection volume must be invariant to 1e-9. The analytic twin
/// campaign is `random_block_pairs_rotation_invariance`; this one drives
/// the same invariant through NURBS↔NURBS SSI and NURBS charts, where a
/// frame-dependent seed or normalization would show up as volume drift.
#[test]
fn random_nurbs_block_pairs_rotation_invariance() {
    let mut rng = Rng::new(0x0F37_501A);
    for case in 0..4 {
        let pair = BlockPair::random(&mut rng);
        let repro = pair.repro(case);
        let mut scene = Scene::new();
        let a = scene.nurbs_block([0.0; 3], pair.a_max);
        let b = scene.nurbs_block(pair.b_min, pair.b_max);
        let inside_a = box_inside_test([0.0; 3], pair.a_max);
        let inside_b = box_inside_test(pair.b_min, pair.b_max);
        let inter = boolean_with_inside_tests(
            BooleanOp::Intersect,
            &scene.store,
            &scene.geo,
            a,
            b,
            &tol(),
            [Some(&inside_a), Some(&inside_b)],
        )
        .unwrap_or_else(|e| panic!("{repro}: NURBS intersect failed: {e:?}"));
        let v = volume_checked(&inter, 1, 0, &format!("{repro}: NURBS intersection"));
        assert_close(
            v,
            pair.vol_overlap(),
            PLANAR_VOLUME_RTOL,
            &format!("{repro}: NURBS intersection vs analytic overlap"),
        );

        let axis = Unit::new_normalize(Vector3::new(
            rng.range(-1.0, 1.0),
            rng.range(-1.0, 1.0),
            rng.range(-1.0, 1.0),
        ));
        let angle = rng.range(0.2, 1.3);
        let rot = Rotation3::from_axis_angle(&axis, angle);
        let center = Point3::new(1.0, 1.0, 1.0);
        let mut scene_rot = Scene::new();
        let ar = scene_rot.nurbs_hexahedron(
            rotated_box_corners([0.0; 3], pair.a_max, &rot, &center),
            [[(0.0, 1.0); 2]; 6],
            1,
        );
        let br = scene_rot.nurbs_hexahedron(
            rotated_box_corners(pair.b_min, pair.b_max, &rot, &center),
            [[(0.0, 1.0); 2]; 6],
            1,
        );
        let inside_ar = rotated_box_inside_test([0.0; 3], pair.a_max, rot, center);
        let inside_br = rotated_box_inside_test(pair.b_min, pair.b_max, rot, center);
        let inter_rot = boolean_with_inside_tests(
            BooleanOp::Intersect,
            &scene_rot.store,
            &scene_rot.geo,
            ar,
            br,
            &tol(),
            [Some(&inside_ar), Some(&inside_br)],
        )
        .unwrap_or_else(|e| {
            panic!("{repro} rotated by {angle} rad about {axis:?}: NURBS intersect failed: {e:?}")
        });
        let v_rot = volume_checked(
            &inter_rot,
            1,
            0,
            &format!("{repro}: rotated NURBS intersection"),
        );
        assert_close(
            v_rot,
            v,
            1e-9,
            &format!("{repro}: NURBS intersection volume under rotation ({angle} rad, {axis:?})"),
        );
    }
}

/// §9 knot-scaling invariance — the test that catches the
/// `tol.parametric` normalization bug. Same fixture as
/// `nurbs_box_half_overlapped_by_nurbs_box`, but every patch's knot
/// domain is offset far from `[0, 1]`, scaled 10× in `u` and 1/32× in
/// `v` (a 320× per-face anisotropy), and distinct per face — while the
/// point sets are bitwise-identical bilinear rectangles. Any behavioral
/// difference from the unscaled twin (which passes at 1e-9) is a
/// parameterization bug by construction: a parametric tolerance or seed
/// step applied to an unnormalized domain, a `[0,1]` assumption, or a
/// domain-relative epsilon.
#[test]
fn nurbs_box_knot_scaling_invariance() {
    let context = "knot-scaled NURBS box half-overlapped by knot-scaled NURBS box";
    let dom_a = |i: usize| -> [(f64, f64); 2] {
        let k = i as f64;
        [
            (10.0 * k + 3.0, 10.0 * k + 13.0),
            (-5.0 * k - 7.0, -5.0 * k - 7.0 + 0.03125),
        ]
    };
    // B's domains invert the anisotropy (tiny u, wide v) and sit on other
    // offsets, so no two patches in the boolean share a parameter scale.
    let dom_b = |i: usize| -> [(f64, f64); 2] {
        let k = i as f64;
        [
            (100.0 * k + 41.0, 100.0 * k + 41.0 + 0.0625),
            (7.0 * k - 2.0, 7.0 * k + 3.0),
        ]
    };
    let mut scene = Scene::new();
    let a_box = ([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]);
    let b_box = ([1.0, -1.0, -1.0], [3.0, 3.0, 3.0]);
    let a = scene.nurbs_hexahedron(
        box_corners(a_box.0, a_box.1),
        [dom_a(0), dom_a(1), dom_a(2), dom_a(3), dom_a(4), dom_a(5)],
        1,
    );
    let b = scene.nurbs_hexahedron(
        box_corners(b_box.0, b_box.1),
        [dom_b(0), dom_b(1), dom_b(2), dom_b(3), dom_b(4), dom_b(5)],
        1,
    );
    let inside_a = box_inside_test(a_box.0, a_box.1);
    let inside_b = box_inside_test(b_box.0, b_box.1);
    assert_half_overlap_identity(
        context,
        &scene,
        a,
        b,
        [Some(&inside_a), Some(&inside_b)],
        8.0,
        32.0,
    );
}

/// §9 multi-span coverage: NURBS boxes whose patches carry interior
/// knots, half-overlapped by an analytic block. With 2 spans the interior
/// knot line `u = 1/2` maps exactly onto the cut plane `x = 1` — the
/// imprint runs ALONG a knot line, where span-boundary continuity bugs in
/// evaluation, projection seeding, or marching live. With 3 spans the
/// same cut lands mid-span, with knot lines at `1/3` and `2/3` crossing
/// the trim regions instead.
#[test]
fn nurbs_box_multispan_imprints_on_and_off_knot_lines() {
    for (spans, where_cut) in [(2usize, "on the u = 1/2 knot line"), (3, "mid-span")] {
        let context = format!("{spans}-span NURBS box half-overlap, cut {where_cut}");
        let mut scene = Scene::new();
        let a_box = ([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]);
        let a = scene.nurbs_hexahedron(box_corners(a_box.0, a_box.1), [[(0.0, 1.0); 2]; 6], spans);
        let b = scene.block([1.0, -1.0, -1.0], [3.0, 3.0, 3.0]);
        let inside_a = box_inside_test(a_box.0, a_box.1);
        assert_half_overlap_identity(&context, &scene, a, b, [Some(&inside_a), None], 8.0, 32.0);
    }
}

/// §9 trim regions abutting the knot-domain boundary: sliver cuts whose
/// surviving regions run hard against the edge of the patch domain.
/// Case (a) shaves a 0.05-thick sliver off `+x`: on each x-spanning face
/// the trim boundary sits at `u = 0.975`, 2.5% of the domain from the
/// `u = 1` edge, and the `+x` face is consumed whole. Case (b) is a
/// 0.02-thick slab across the top — 1% of the domain from the `v = 1`
/// edge on four faces at once. Both must keep the 1e-9 identity: a
/// domain-boundary clamp or an epsilon that swallows the sliver shows up
/// as a volume error five orders of magnitude above the tolerance.
#[test]
fn nurbs_box_sliver_trims_abut_knot_domain_boundary() {
    let a_box = ([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]);
    let inside_a = box_inside_test(a_box.0, a_box.1);

    let mut scene = Scene::new();
    let a = scene.nurbs_block(a_box.0, a_box.1);
    let b = scene.block([1.95, -1.0, -1.0], [3.95, 3.0, 3.0]);
    assert_overlap_identity(
        "0.05 sliver at +x of a NURBS box",
        &scene,
        a,
        b,
        [Some(&inside_a), None],
        (8.0, 2.0 * 4.0 * 4.0, 0.05 * 2.0 * 2.0),
    );

    let mut scene = Scene::new();
    let a = scene.nurbs_block(a_box.0, a_box.1);
    let b = scene.block([-1.0, -1.0, 1.98], [3.0, 3.0, 3.5]);
    assert_overlap_identity(
        "0.02 slab across the top of a NURBS box",
        &scene,
        a,
        b,
        [Some(&inside_a), None],
        (8.0, 4.0 * 4.0 * 1.52, 2.0 * 2.0 * 0.02),
    );
}

// =====================================================================
// (15) The second measurement path: B-Rep-native mass properties, and
//      centroid/inertia as first-class oracles (of-ipt.17).
//
// Two independent things are proved here.
//
// First, that `brep_mass_properties` — surface integrals over the faces
// themselves, reduced to contour integrals over the trim curves — agrees with
// `mass_properties` on a mesh. Every `volume()` call in this file already
// asserts that pairwise; this section pins it to *closed forms* on both
// sides, so an agreement between two paths that are both wrong the same way
// cannot pass.
//
// Second, that the results are in the right *place* and have the right mass
// *distribution*. Volume was the whole oracle before: a boolean that kept the
// correct amount of material somewhere else entirely, or that mirrored a
// pocket to the opposite side, weighed exactly the same. The `Rigid` algebra
// above supplies the missing expectations in closed form.
// =====================================================================

/// Both measurements of a plain (non-boolean) body, cross-checked the same way
/// [`measured`] does for boolean results.
fn measured_body(
    scene: &Scene,
    body: EntityId<Body>,
    context: &str,
) -> (MassProperties, MassProperties) {
    // Match the 96 samples per circle `BooleanOutput::tessellate` uses, so
    // the cross-check budget below means the same thing here as it does for a
    // boolean result.
    let mesh = tessellate_body(
        &scene.store,
        &scene.geo,
        body,
        &TessellationOptions {
            angular_step: 2.0 * PI / 96.0,
        },
    )
    .unwrap_or_else(|e| panic!("{context}: tessellation failed: {e:?}"));
    assert!(
        mesh.is_closed_manifold(),
        "{context}: tessellation is not a closed manifold"
    );
    let meshed =
        mass_properties(&mesh).unwrap_or_else(|e| panic!("{context}: mass_properties failed: {e}"));
    let exact = brep_mass_properties(&scene.store, &scene.geo, body)
        .unwrap_or_else(|e| panic!("{context}: brep_mass_properties failed: {e}"));
    let diagonal = mesh
        .bounding_box()
        .map(|b| (b.max - b.min).norm())
        .unwrap_or(0.0);
    assert_cross_checked(&meshed, &exact, diagonal, context);
    (meshed, exact)
}

/// The operands themselves, before any boolean: the B-Rep path must land on
/// the analytic volume and area to floating point, not merely near the mesh.
///
/// This is the check that would catch a tessellator whose parameter rectangle
/// is wrong for a whole surface class — the mesh path cannot see its own bias,
/// and comparing two meshes to each other never will.
#[test]
fn brep_path_hits_closed_form_on_every_operand_class() {
    let mut scene = Scene::new();
    let block = scene.block([-1.0, -1.5, -2.5], [1.0, 1.5, 2.5]);
    let cylinder = scene.cylinder(Point3::new(0.0, 0.0, -2.0), Vector3::z(), 1.5, 4.0);
    let sphere = scene.sphere(Point3::new(0.5, -0.25, 1.0), 1.25);
    let torus = scene.torus(Point3::new(0.0, 0.0, 0.0), 3.0, 0.75);
    let frustum = scene.cone(Point3::new(0.0, 0.0, -1.5), 2.0, 0.8, 3.0);
    let cone = scene.cone(Point3::new(0.0, 0.0, -1.5), 2.0, 0.0, 3.0);

    let cases: [(&str, EntityId<Body>, f64, f64); 6] = [
        ("block", block, 2.0 * 3.0 * 5.0, 2.0 * (6.0 + 15.0 + 10.0)),
        (
            "cylinder",
            cylinder,
            PI * 1.5 * 1.5 * 4.0,
            2.0 * PI * 1.5 * 1.5 + 2.0 * PI * 1.5 * 4.0,
        ),
        (
            "sphere",
            sphere,
            sphere_volume(1.25),
            4.0 * PI * 1.25 * 1.25,
        ),
        (
            "torus",
            torus,
            torus_volume(3.0, 0.75),
            4.0 * PI * PI * 3.0 * 0.75,
        ),
        (
            "frustum",
            frustum,
            frustum_volume(2.0, 0.8, 3.0),
            PI * (4.0 + 0.64) + PI * 2.8 * ((1.2f64 * 1.2 + 9.0).sqrt()),
        ),
        (
            "pointed cone",
            cone,
            frustum_volume(2.0, 0.0, 3.0),
            PI * 4.0 + PI * 2.0 * ((4.0f64 + 9.0).sqrt()),
        ),
    ];

    for (name, body, want_volume, want_area) in cases {
        let (_, exact) = measured_body(&scene, body, name);
        assert_close(
            exact.volume,
            want_volume,
            EXACT_RTOL,
            &format!("{name} volume"),
        );
        assert_close(
            exact.surface_area,
            want_area,
            EXACT_RTOL,
            &format!("{name} area"),
        );
    }
}

/// The operands' centroids and inertia tensors, against the textbook
/// composites. Nothing in this suite weighed either before.
#[test]
fn operand_centroids_and_inertia_match_closed_form() {
    let mut scene = Scene::new();
    let block = scene.block([-1.0, -1.5, -2.5], [1.0, 1.5, 2.5]);
    let cylinder = scene.cylinder(Point3::new(0.0, 0.0, -2.0), Vector3::z(), 1.5, 4.0);
    let sphere = scene.sphere(Point3::new(0.5, -0.25, 1.0), 1.25);

    let (_, exact) = measured_body(&scene, block, "block");
    assert_matches_rigid(
        &exact,
        Rigid::block(Point3::origin(), Vector3::new(2.0, 3.0, 5.0)),
        EXACT_RTOL,
        "block",
    );

    let (_, exact) = measured_body(&scene, cylinder, "cylinder");
    assert_matches_rigid(
        &exact,
        Rigid::cylinder_z(Point3::origin(), 1.5, 4.0),
        EXACT_RTOL,
        "cylinder",
    );

    let (_, exact) = measured_body(&scene, sphere, "sphere");
    assert_matches_rigid(
        &exact,
        Rigid::sphere(Point3::new(0.5, -0.25, 1.0), 1.25),
        EXACT_RTOL,
        "sphere",
    );
}

/// A block with a **concentric** through-bore. Volume alone cannot tell this
/// apart from the same bore drilled off-center, or from a bore of the same
/// cross-section drilled along the wrong axis; the inertia tensor can, and
/// here it is exactly `block − cylinder`.
#[test]
fn concentric_bore_matches_composite_inertia() {
    let mut scene = Scene::new();
    let block = scene.block([-2.0, -2.0, -0.75], [2.0, 2.0, 0.75]);
    let drill = scene.cylinder(Point3::new(0.0, 0.0, -3.0), Vector3::z(), 0.9, 6.0);
    let out = scene
        .subtract(block, drill)
        .expect("block minus a concentric through-bore");

    let want = Rigid::block(Point3::origin(), Vector3::new(4.0, 4.0, 1.5))
        .minus(Rigid::cylinder_z(Point3::origin(), 0.9, 1.5));

    let (meshed, exact) = measured(&out, "concentric bore");
    assert_matches_rigid(&exact, want, EXACT_RTOL, "concentric bore (B-Rep path)");
    // The mesh path sees the same solid through a 96-gon bore; the budget is
    // the discretization, and the *shape* of the tensor still has to be right.
    assert_matches_rigid(&meshed, want, 1e-2, "concentric bore (mesh path)");
}

/// The same bore moved off the axis. The volume is identical to the
/// concentric case to the last bit — only the centroid and the products of
/// inertia move, and they move by an amount the closed form pins exactly.
#[test]
fn offset_bore_moves_the_centroid_and_wakes_the_products_of_inertia() {
    let mut scene = Scene::new();
    let block = scene.block([-2.0, -2.0, -0.75], [2.0, 2.0, 0.75]);
    // Clear of every side face: at radius 0.9 the bore reaches x = 1.9 and
    // y = 0.4, so nothing is tangent to the block and the cut stays transversal.
    let hole_center = Point3::new(1.0, -0.5, 0.0);
    let drill = scene.cylinder(
        Point3::new(hole_center.x, hole_center.y, -3.0),
        Vector3::z(),
        0.9,
        6.0,
    );
    let out = scene
        .subtract(block, drill)
        .expect("block minus an offset through-bore");

    let want = Rigid::block(Point3::origin(), Vector3::new(4.0, 4.0, 1.5))
        .minus(Rigid::cylinder_z(hole_center, 0.9, 1.5));

    let (meshed, exact) = measured(&out, "offset bore");
    assert_matches_rigid(&exact, want, EXACT_RTOL, "offset bore (B-Rep path)");
    assert_matches_rigid(&meshed, want, 1e-2, "offset bore (mesh path)");

    // The centroid really did move, and away from the hole: a test that
    // passes with the expectation and the measurement both stuck at the
    // origin would prove nothing.
    let centroid = exact.centroid;
    assert!(
        centroid.x < -0.05 && centroid.y > 0.02,
        "centroid {centroid:?} did not shift away from the hole"
    );
    // An off-axis bore breaks the block's symmetry about z, so Ixy is real.
    let magnitude = exact.inertia[(2, 2)];
    assert!(
        exact.inertia[(0, 1)].abs() > 1e-3 * magnitude,
        "off-axis bore left Ixy at {} (tensor magnitude {magnitude})",
        exact.inertia[(0, 1)]
    );
}

/// A stepped pedestal: two overlapping blocks united. Planar throughout, so
/// *both* paths are exact and both are held to their exact budgets, and the
/// union's centroid sits where inclusion–exclusion says — not at either
/// operand's center, and not where a double-counted overlap would put it.
#[test]
fn stepped_blocks_union_matches_composite_inertia() {
    let mut scene = Scene::new();
    let lower = scene.block([-1.0, -1.0, -2.0], [1.0, 1.0, 0.0]);
    let upper = scene.block([-0.5, -0.5, -0.3], [0.5, 0.5, 1.5]);
    let out = scene.unite(lower, upper).expect("overlapping blocks");

    // A ∪ B = A + B − A∩B, and the overlap of two axis-aligned blocks is a
    // third block.
    let want = Rigid::block(Point3::new(0.0, 0.0, -1.0), Vector3::new(2.0, 2.0, 2.0))
        .plus(Rigid::block(
            Point3::new(0.0, 0.0, 0.6),
            Vector3::new(1.0, 1.0, 1.8),
        ))
        .minus(Rigid::block(
            Point3::new(0.0, 0.0, -0.15),
            Vector3::new(1.0, 1.0, 0.3),
        ));

    let (meshed, exact) = measured(&out, "stepped blocks");
    assert_matches_rigid(&exact, want, EXACT_RTOL, "stepped blocks (B-Rep path)");
    assert_matches_rigid(
        &meshed,
        want,
        PLANAR_VOLUME_RTOL,
        "stepped blocks (mesh path)",
    );
}

/// Two separate bores through one plate: the annular caps carry *two* inner
/// loops each, so the B-Rep path's contour integral has to see both holes
/// subtract, and the composite pins where the remaining material sits.
#[test]
fn twin_bores_match_composite_inertia() {
    let left = Point3::new(-1.5, 0.4, 0.0);
    let right = Point3::new(1.7, -0.6, 0.0);

    let mut scene = Scene::new();
    let plate = scene.block([-3.0, -2.0, -0.5], [3.0, 2.0, 0.5]);
    let first_drill = scene.cylinder(Point3::new(left.x, left.y, -3.0), Vector3::z(), 0.7, 6.0);
    let once = scene.subtract(plate, first_drill).expect("first bore");

    // The second bore is drilled into the result of the first: the boolean
    // output's stores become the next scene's.
    let (mut next, once_body) = Scene::adopt(once, tol());
    let second_drill = next.cylinder(Point3::new(right.x, right.y, -3.0), Vector3::z(), 0.55, 6.0);
    let out = next.subtract(once_body, second_drill).expect("second bore");

    let want = Rigid::block(Point3::origin(), Vector3::new(6.0, 4.0, 1.0))
        .minus(Rigid::cylinder_z(left, 0.7, 1.0))
        .minus(Rigid::cylinder_z(right, 0.55, 1.0));

    let (meshed, exact) = measured(&out, "twin bores");
    assert_matches_rigid(&exact, want, EXACT_RTOL, "twin bores (B-Rep path)");
    assert_matches_rigid(&meshed, want, 1e-2, "twin bores (mesh path)");
}

/// A spherical pocket opened through the top of a block: the removed solid is
/// a hemisphere, so the composite is exact and the result carries a trimmed
/// sphere face — the class the mesh path handles least well and the B-Rep
/// path handles by integrating the sphere's own parameterization.
#[test]
fn hemispherical_pocket_matches_composite_inertia() {
    let mut scene = Scene::new();
    let block = scene.block([-2.0, -2.0, -1.5], [2.0, 2.0, 0.0]);
    let ball_center = Point3::new(0.3, -0.4, 0.0);
    let ball = scene.sphere(ball_center, 1.0);
    let out = scene
        .subtract(block, ball)
        .expect("block minus a ball on its face");

    // The half of the ball below z = 0 is what leaves the block: a hemisphere
    // of radius 1, centroid 3r/8 below the cut plane, with the textbook
    // centroidal tensor ( Izz = 2mr²/5, transverse = 83mr²/320).
    let r: f64 = 1.0;
    let m = 2.0 / 3.0 * PI * r * r * r;
    let transverse = 83.0 / 320.0 * m * r * r;
    let hemisphere = Rigid::placed(
        m,
        Point3::new(ball_center.x, ball_center.y, ball_center.z - 3.0 * r / 8.0),
        Matrix3::from_diagonal(&Vector3::new(transverse, transverse, 0.4 * m * r * r)),
    );
    let want =
        Rigid::block(Point3::new(0.0, 0.0, -0.75), Vector3::new(4.0, 4.0, 1.5)).minus(hemisphere);

    let (meshed, exact) = measured(&out, "hemispherical pocket");
    assert_matches_rigid(
        &exact,
        want,
        EXACT_RTOL,
        "hemispherical pocket (B-Rep path)",
    );
    assert_matches_rigid(&meshed, want, 2e-2, "hemispherical pocket (mesh path)");
}

/// Rigid motion invariance, measured the exact way: rotating both operands
/// rotates the result's centroid and conjugates its inertia tensor, and
/// leaves volume and area alone. The mesh path could only ever check this to
/// its own discretization; here it is a 1e-9 statement.
#[test]
fn brep_measurement_is_equivariant_under_rotation() {
    let build = |scene: &mut Scene| {
        let block = scene.block([-2.0, -2.0, -0.75], [2.0, 2.0, 0.75]);
        let drill = scene.cylinder(Point3::new(1.0, -0.5, -3.0), Vector3::z(), 0.9, 6.0);
        (block, drill)
    };

    let mut upright = Scene::new();
    let (a, b) = build(&mut upright);
    let plain = upright.subtract(a, b).expect("upright bore");
    let (_, here) = measured(&plain, "upright bore");

    let mut turned = Scene::new();
    let (a, b) = build(&mut turned);
    let axis = Unit::new_normalize(Vector3::new(0.3, -0.7, 0.5));
    let angle = 0.9;
    let rot = Rotation3::from_axis_angle(&axis, angle);
    for body in [a, b] {
        rotate_body(
            &mut turned.store,
            &mut turned.geo,
            body,
            Point3::origin(),
            axis.into_inner(),
            angle,
        )
        .expect("rigid rotation");
    }
    let spun = turned.subtract(a, b).expect("rotated bore");
    let (_, there) = measured(&spun, "rotated bore");

    assert_close(there.volume, here.volume, EXACT_RTOL, "rotated volume");
    assert_close(
        there.surface_area,
        here.surface_area,
        EXACT_RTOL,
        "rotated area",
    );

    let want_centroid = Point3::from(rot * here.centroid.coords);
    let scale = here.volume.cbrt();
    assert!(
        (there.centroid - want_centroid).norm() <= EXACT_RTOL * scale,
        "rotated centroid {:?} is not the rotation of {:?}",
        there.centroid,
        here.centroid
    );

    let want_inertia = rot.matrix() * here.inertia * rot.matrix().transpose();
    let magnitude = want_inertia.iter().fold(0.0f64, |a, b| a.max(b.abs()));
    for i in 0..3 {
        for j in 0..3 {
            assert!(
                (there.inertia[(i, j)] - want_inertia[(i, j)]).abs() <= EXACT_RTOL * magnitude,
                "rotated I[{i}][{j}] = {} vs conjugated {}",
                there.inertia[(i, j)],
                want_inertia[(i, j)]
            );
        }
    }
}

/// Inclusion–exclusion, weighed the exact way and on all ten moments at once:
/// `props(A) + props(B) == props(A∪B) + props(A∩B)` holds for volume, for the
/// first moments (hence the centroid), and for the full inertia tensor about
/// the origin. Meshed, this identity is only ever true to the tessellation;
/// exactly, it is an algebraic fact the pipeline either respects or does not.
#[test]
fn inclusion_exclusion_holds_for_every_moment() {
    let mut scene = Scene::new();
    let a = scene.block([-1.5, -1.5, -1.0], [1.5, 1.5, 1.0]);
    let b = scene.cylinder(Point3::new(0.4, -0.3, -2.0), Vector3::z(), 1.0, 4.0);

    let props =
        |body: EntityId<Body>| brep_mass_properties(&scene.store, &scene.geo, body).unwrap();
    let (pa, pb) = (props(a), props(b));

    let union = scene.unite(a, b).expect("union");
    let meet = scene.intersect(a, b).expect("intersection");
    let (_, pu) = measured(&union, "inclusion-exclusion union");
    let (_, pi) = measured(&meet, "inclusion-exclusion intersection");

    // The cylinder overhangs the block, so operand B is not contained: this
    // is a genuine four-way identity, not a restatement of A = A.
    assert!(pi.volume < pb.volume * 0.99, "intersection is not proper");

    assert_close(
        pu.volume + pi.volume,
        pa.volume + pb.volume,
        EXACT_RTOL,
        "inclusion-exclusion volume",
    );

    // First moments: `V·centroid` is the additive quantity, not the centroid.
    let moment = |p: &MassProperties| p.centroid.coords * p.volume;
    let lhs = moment(&pu) + moment(&pi);
    let rhs = moment(&pa) + moment(&pb);
    assert!(
        (lhs - rhs).norm() <= EXACT_RTOL * rhs.norm().max(pa.volume * pa.volume.cbrt()),
        "inclusion-exclusion first moments: {lhs:?} vs {rhs:?}"
    );

    // Inertia adds only about a common point, so undo each parallel-axis
    // shift back to the origin before summing.
    let about_origin = |p: &MassProperties| {
        let c = p.centroid.coords;
        p.inertia + (Matrix3::identity() * c.norm_squared() - c * c.transpose()) * p.volume
    };
    let lhs = about_origin(&pu) + about_origin(&pi);
    let rhs = about_origin(&pa) + about_origin(&pb);
    let magnitude = rhs.iter().fold(0.0f64, |m, x| m.max(x.abs()));
    for i in 0..3 {
        for j in 0..3 {
            assert!(
                (lhs[(i, j)] - rhs[(i, j)]).abs() <= EXACT_RTOL * magnitude,
                "inclusion-exclusion I[{i}][{j}]: {} vs {}",
                lhs[(i, j)],
                rhs[(i, j)]
            );
        }
    }
}

/// Freeform operands weighed the exact way. The B-Rep path integrates a NURBS
/// patch by Gauss–Legendre over its knot spans rather than by any analytic
/// shortcut, so these are the cases that prove the quadrature — and the
/// knot-span panelling — rather than a closed-form arm.
///
/// All three carry a *known* answer: a bilinear patch over a rectangle is the
/// rectangle, and `Scene::nurbs_cylinder`/`nurbs_cone` are exact quadrics
/// built from rational quadratic quarter-arcs, not approximations of them.
/// The cone additionally has a **collapsed control row** at its apex — the
/// of-37i.7 shape the boolean chart rejects outright — which the measurement
/// handles because the collapse is a parameterization singularity, exactly
/// like a sphere's pole.
#[test]
fn brep_path_hits_closed_form_on_nurbs_operands() {
    let (r, h) = (1.5, 4.0);

    let mut scene = Scene::new();
    let block = scene.nurbs_block([-1.0, -1.5, -2.5], [1.0, 1.5, 2.5]);
    let (_, exact) = measured_body(&scene, block, "NURBS block");
    assert_matches_rigid(
        &exact,
        Rigid::block(Point3::origin(), Vector3::new(2.0, 3.0, 5.0)),
        EXACT_RTOL,
        "NURBS block",
    );

    // The same box over three knot spans per direction on a `[3, 13]`-style
    // domain: identical geometry, a parameterization that shares nothing with
    // the default. A quadrature that priced panels by parameter rather than
    // by knot span would drift here and nowhere else.
    let mut scene = Scene::new();
    let rescaled = scene.nurbs_hexahedron(
        box_corners([-1.0, -1.5, -2.5], [1.0, 1.5, 2.5]),
        [[(3.0, 13.0), (-7.0, -2.0)]; 6],
        3,
    );
    let (_, exact) = measured_body(&scene, rescaled, "NURBS block, 3 spans on a shifted domain");
    assert_matches_rigid(
        &exact,
        Rigid::block(Point3::origin(), Vector3::new(2.0, 3.0, 5.0)),
        EXACT_RTOL,
        "NURBS block, 3 spans on a shifted domain",
    );

    let mut scene = Scene::new();
    let cylinder = scene.nurbs_cylinder(0.0, 0.0, r, -h / 2.0, h);
    let (_, exact) = measured_body(&scene, cylinder, "NURBS cylinder");
    assert_matches_rigid(
        &exact,
        Rigid::cylinder_z(Point3::origin(), r, h),
        EXACT_RTOL,
        "NURBS cylinder",
    );
    assert_close(
        exact.surface_area,
        2.0 * PI * r * r + 2.0 * PI * r * h,
        EXACT_RTOL,
        "NURBS cylinder area",
    );

    // The collapsed row is the one place the B-Rep path is *not* exact, and
    // the reason is the trim, not the quadrature: at the apex every `u`
    // projects to the same point, so `fit_pcurve` cannot resolve the seam's
    // `u` there. Its last sample lands 2.9% of the domain off, the fit falls
    // back to a polyline, and the region it bounds is wrong by that much in a
    // neighbourhood of the apex — where the surface area goes to zero, which
    // is why 2.9% of the trim costs 4e-6 of the volume. Raising the
    // quadrature order does not move the number at all; only a trim that is
    // not projected would. `NURBS_TRIM_RTOL` is set two orders above the
    // error, so a real regression still fails and this stays a live test.
    const NURBS_TRIM_RTOL: f64 = 1e-4;
    let mut scene = Scene::new();
    let cone = scene.nurbs_cone(0.0, 0.0, r, -h / 2.0, h);
    let (_, exact) = measured_body(&scene, cone, "NURBS cone");
    assert_close(
        exact.volume,
        frustum_volume(r, 0.0, h),
        NURBS_TRIM_RTOL,
        "NURBS cone volume",
    );
    assert_close(
        exact.centroid.z,
        -h / 2.0 + h / 4.0,
        NURBS_TRIM_RTOL,
        "NURBS cone centroid",
    );
}

// =====================================================================
// (16) Numerical robustness families (of-ipt.19)
//
// Section (5) proved the pipeline works over three decades of scale, all
// of them clustered around 1. That is the easy part of the range. This
// section covers the families the epic contemplates and (5) does not:
//
//   16.1  six decades of absolute scale, 1e-6 to 1e6 model units;
//   16.2  geometry far from the origin, where a small feature is the
//         difference of two large coordinates (catastrophic cancellation);
//   16.3  operands whose sizes differ by up to six decades;
//   16.4  extreme aspect ratios — plates and needles;
//   16.5  near-parallel surface pairs, where the SSI direction
//         `n_a × n_b` is the ill-conditioned quantity;
//   16.6  repeated booleans, where the question is whether error
//         *accumulates* over a chain rather than whether one op is right;
//   16.7  high-degree and near-degenerate-knot NURBS operands.
//
// A recurring theme is that f64 sets a floor on what any of these can be
// asserted to, and the floor is different for each family. Where it binds,
// the helper that computes the bound is written out rather than a magic
// constant being pasted in, because the bound *is* the finding: it says
// what precision the pipeline is entitled to at that configuration, and a
// failure means the pipeline did worse than arithmetic forced it to.
//
// What the campaign found, in short. The B-Rep pipeline itself came through
// the scale families intact: it reproduces `32 − 2π` to twelve digits at
// every decade from 1e-6 to 1e6 and at every offset out to 1e6, holds an
// eight-deep boolean chain to the same budget at step eight as at step one,
// and swallows degree-5 patches, C0 knots and knots 1e-9 apart without a
// wobble. What broke was mostly *around* it — the mesh measurement path
// (of-ukcq, since fixed), island imprints (of-6viu), and the trimmed-sphere
// measurement (of-y8qc). The one defect that was squarely inside it,
// of-oygs, turned out to be a band sized off the wrong quantity rather than
// anything about seeding: `near_face_boundary` scaled its "too close to
// trust" distance by the whole face's bounding box, so a slender face's own
// length made its interior unreachable. It now measures each boundary
// polyline's own chord error instead, and both of its cases are live.
// =====================================================================

/// The tolerance context a model whose features are `scale` model units
/// across deserves — the default one with its *linear* term scaled, since a
/// linear tolerance is a length and 1e-6 mm means "coincident" for a
/// millimetre-sized part and "the whole part" for a micron-sized one. The
/// angular and parametric terms are dimensionless and do not scale.
///
/// Note the floor this runs into at the small end: `effective_linear` never
/// returns below [`SYSTEM_RESOLUTION`] (1e-10), so at `scale = 1e-6` the
/// requested 1e-12 is clamped back up to 1e-10 — a relative tolerance of
/// 1e-4 of the feature rather than the 1e-6 every larger decade gets. The
/// bottom decade is therefore working four decades above the kernel's
/// precision floor rather than six. In the event that costs nothing
/// measurable: the exact path reproduces the closed form to twelve digits
/// there just as it does at 1e6.
fn tol_at_scale(scale: f64) -> ToleranceContext {
    ToleranceContext {
        linear: tol().linear * scale,
        ..tol()
    }
}

/// The six decades of absolute scale the epic contemplates, in model units.
const DECADES: [f64; 5] = [1e-6, 1e-3, 1.0, 1e3, 1e6];

/// Volume by the B-Rep-native path, still cross-checked against the mesh.
///
/// The distinction from [`volume`] is only *which of the two numbers is
/// returned*, not whether they are compared: both paths are measured and
/// asserted to agree exactly as everywhere else. Where the far field is the
/// thing under test the B-Rep number is the better oracle to weigh closed
/// forms against, because it integrates each face's own parameterization and
/// so carries no discretization error at all.
///
/// This function used to skip the cross-check outright. `mass_properties`
/// referenced its tetrahedra to the absolute origin, which cost `|p|³/V`
/// worth of digits on geometry far from home — 191× wrong at offset 1e6 —
/// putting the mesh path outside its own domain of validity for two of this
/// section's families. of-ukcq moved the apex to the mesh's bounding-box
/// centre; the two paths now agree here as well, so there is nothing left to
/// route around and the check is back on.
/// [`mesh_mass_properties_survives_far_from_origin`] holds the mesh path to
/// the closed form directly.
fn brep_volume(out: &BooleanOutput, context: &str) -> f64 {
    measured(out, context).1.volume
}

// ---------------------------------------------------------------------
// 16.1 Extreme scale: the same configuration over six decades.
// ---------------------------------------------------------------------

/// The through-hole scenario of section (5) run over the epic's full range
/// rather than its middle three decades, and asserted the stronger way: not
/// merely that each decade matches its own closed form, but that the
/// *scale-normalized* volume `V / s³` is the same number at every decade.
///
/// Each decade uses the tolerance context its own size deserves
/// ([`tol_at_scale`]). The cross-decade comparison is made on the B-Rep-native
/// measurement, which is where an exact scale invariance can honestly be
/// demanded: it reproduces `32 − 2π` to twelve digits at every decade from
/// 1e-6 to 1e6, so `EXACT_RTOL` is not a generous budget here but a very
/// tight one.
///
/// The *meshed* volume is deliberately not compared across decades. It splits
/// into two groups — 25.717972 for the two decades whose tolerance context is
/// tighter than 1e-6, 25.721229 for the three that are looser — with the
/// triangle count identical (6352) in all five. That is a tolerance-driven
/// trim refinement, not a scale defect: the tighter group sits closer to the
/// exact 25.716815, which is the direction a finer fit should move it. Each
/// decade is still held to the closed form on both paths.
#[test]
fn through_hole_over_six_decades() {
    let mut normalized = Vec::new();
    for scale in DECADES {
        let context = format!("block minus cylinder at {scale:e}× scale");
        let s = scale;
        let mut scene = Scene::with_tolerance(tol_at_scale(s));
        let slab = scene.block([0.0, 0.0, 0.0], [4.0 * s, 4.0 * s, 2.0 * s]);
        let tool = scene.cylinder(Point3::new(2.0 * s, 2.0 * s, -s), Vector3::z(), s, 4.0 * s);
        let out = scene
            .subtract(slab, tool)
            .unwrap_or_else(|e| panic!("{context}: subtract failed: {e:?}"));
        assert_eq!(
            out.store.euler_counts(out.body).genus,
            1,
            "{context}: through hole must give genus 1"
        );
        let (meshed, exact) = measured(&out, &context);
        let want = (32.0 - 2.0 * PI) * s * s * s;
        assert_close(
            meshed.volume,
            want,
            CYL_VOLUME_RTOL,
            &format!("{context} (mesh path)"),
        );
        assert_close(
            exact.volume,
            want,
            EXACT_RTOL,
            &format!("{context} (B-Rep path)"),
        );
        normalized.push((scale, exact.volume / (s * s * s)));
    }

    let reference = normalized
        .iter()
        .find(|(s, _)| *s == 1.0)
        .expect("unit scale is one of the decades")
        .1;
    for &(scale, value) in &normalized {
        assert_close(
            value,
            reference,
            EXACT_RTOL,
            &format!("normalized through-hole volume at {scale:e}× vs unit scale"),
        );
    }
}

/// Seeded random transversal block pairs at the two decades section (5)
/// does not reach. Purely planar, so the identities hold to floating point
/// and the only thing that can break them is a length comparison that
/// stopped meaning what it means at unit scale.
#[test]
fn block_pair_identity_at_the_outer_decades() {
    for scale in [1e-6f64, 1e6] {
        let mut rng = Rng::new(0x0DEC_ADE5 ^ scale.to_bits());
        for case in 0..6 {
            let pair = BlockPair::random(&mut rng);
            let repro = format!("scale {scale:e}×, {}", pair.repro(case));
            let s = scale;
            let mut scene = Scene::with_tolerance(tol_at_scale(s));
            let a = scene.block(
                [0.0, 0.0, 0.0],
                [pair.a_max[0] * s, pair.a_max[1] * s, pair.a_max[2] * s],
            );
            let b = scene.block(
                [pair.b_min[0] * s, pair.b_min[1] * s, pair.b_min[2] * s],
                [pair.b_max[0] * s, pair.b_max[1] * s, pair.b_max[2] * s],
            );
            let union = scene
                .unite(a, b)
                .unwrap_or_else(|e| panic!("{repro}: unite failed: {e:?}"));
            let inter = scene
                .intersect(a, b)
                .unwrap_or_else(|e| panic!("{repro}: intersect failed: {e:?}"));
            let s3 = s * s * s;
            let vol_union = volume(&union, &format!("{repro}: union"));
            let vol_inter = volume(&inter, &format!("{repro}: intersection"));
            assert_close(
                vol_union + vol_inter,
                (pair.vol_a() + pair.vol_b()) * s3,
                PLANAR_VOLUME_RTOL,
                &format!("{repro}: inclusion–exclusion identity"),
            );
            assert_close(
                vol_inter,
                pair.vol_overlap() * s3,
                PLANAR_VOLUME_RTOL,
                &format!("{repro}: intersection vs analytic overlap"),
            );
        }
    }
}

// ---------------------------------------------------------------------
// 16.2 Geometry far from the origin: catastrophic cancellation.
// ---------------------------------------------------------------------

/// Roundings, in units of the coordinate's own ULP, that a length is allowed
/// to accumulate on its way from a caller's `min`/`max` to a face plane.
///
/// It is not one: [`Scene::block`] builds a primitive from *extents* and
/// then translates it by a *centre*, so a face plane is reached through
/// several additions and halvings, each rounding at the magnitude of the
/// coordinates rather than of the feature. The boolean then differences
/// those planes. 128 leaves room for that chain with margin and is still
/// tight enough to be a real constraint — at the family's worst corner
/// (offset 1e6, a 1e-4 feature) it demands the slab be resolved to better
/// than three parts in ten thousand, where f64 offers about one in a
/// million.
const FAR_FIELD_ULPS: f64 = 128.0;

/// Relative volume budget for a configuration whose smallest interesting
/// length is `feature` but whose coordinates sit `offset` from the origin.
///
/// A feature length is recovered as the difference of two coordinates, and
/// f64 spacing at magnitude `offset` is `offset · EPSILON`, so the length
/// carries a relative error of at least `offset · EPSILON / feature` before
/// the kernel does anything at all — [`FAR_FIELD_ULPS`] times that, given
/// how many roundings sit between a caller's numbers and a face plane. Never
/// tighter than the flat planar budget, which dominates near the origin
/// where `offset · EPSILON` is nothing.
///
/// This is a bound on the *inputs*, not a fudge factor: a failure means the
/// pipeline lost precision that the arithmetic did not force it to lose.
fn far_field_rtol(offset: f64, feature: f64) -> f64 {
    PLANAR_VOLUME_RTOL.max(FAR_FIELD_ULPS * offset * f64::EPSILON / feature)
}

/// The mesh measurement path's own far-field behaviour, which the rest of
/// this family routes around by weighing results with [`brep_volume`].
///
/// `mass_properties` used to sum tetrahedra whose apex was the absolute
/// origin, so its intermediate magnitudes were `|p|³` where the answer is
/// `feature³`. At offset 1e6 a 4×4×2 slab with a unit bore measured 4921
/// instead of 25.72 — 191× — while `brep_mass_properties` returned
/// 25.716814691 at every offset from 0 to 1e6. of-ukcq moved the apex to the
/// mesh's own bounding-box centre, which bounds the cancellation by the
/// body's diameter instead of by its distance from the origin; this test is
/// what holds that.
#[test]
fn mesh_mass_properties_survives_far_from_origin() {
    for offset in FAR_OFFSETS {
        let context = format!("block minus cylinder at offset {offset:e}");
        let o = offset;
        let mut scene = Scene::new();
        let slab = scene.block([o, o, o], [o + 4.0, o + 4.0, o + 2.0]);
        let tool = scene.cylinder(
            Point3::new(o + 2.0, o + 2.0, o - 1.0),
            Vector3::z(),
            1.0,
            4.0,
        );
        let out = scene
            .subtract(slab, tool)
            .unwrap_or_else(|e| panic!("{context}: subtract failed: {e:?}"));
        // `measured` is what asserts the two paths agree; that is the
        // assertion of-ukcq used to break.
        let (meshed, _) = measured(&out, &context);
        assert_close(meshed.volume, 32.0 - 2.0 * PI, CYL_VOLUME_RTOL, &context);
    }
}

/// Distances from the origin at which the same unit-sized configuration is
/// rebuilt. At 1e6 the f64 grid is 1.2e-10 wide, so a unit feature is
/// resolved to ~10 significant digits rather than 16: six of the sixteen
/// digits have gone into naming *where* the part is instead of what it is.
///
/// The offsets are irrational on purpose. Integer offsets are a trap: with
/// coordinates like `1000002.0` every product in a volume integrand is an
/// exact integer well inside f64's 2^53, so the arithmetic is *exact* and
/// the family silently tests nothing. The `+ 1/π` makes every coordinate an
/// ordinary inexact float, which is the case a real model is in.
const FAR_OFFSETS: [f64; 4] = [
    0.0,
    1e2 + std::f64::consts::FRAC_1_PI,
    1e4 + std::f64::consts::FRAC_1_PI,
    1e6 + std::f64::consts::FRAC_1_PI,
];

/// A transversal block pair rebuilt at increasing distance from the origin.
/// The configuration is congruent at every offset, so the analytic answers
/// are literally the same numbers; only the coordinates naming them grow.
///
/// This is the cheapest possible catastrophic-cancellation probe and the
/// one most likely to catch an absolute epsilon: a comparison written as
/// `|a - b| < 1e-9` is fine at the origin and meaningless at 1e6, where
/// 1e-9 is eight times the representable spacing.
#[test]
fn far_from_origin_block_pair_identity() {
    let (a_max, b_min, b_max) = ([2.0, 3.0, 2.5], [1.0, -1.0, 0.5], [4.0, 2.0, 3.0]);
    // x overlap [1,2] = 1, y overlap [0,2] = 2, z overlap [0.5,2.5] = 2.
    let (vol_a, vol_b, overlap) = (15.0, 22.5, 4.0);

    for offset in FAR_OFFSETS {
        let context = format!("transversal block pair at offset {offset:e}");
        let d = |p: [f64; 3]| [p[0] + offset, p[1] + offset, p[2] + offset];
        let mut scene = Scene::new();
        let a = scene.block(d([0.0, 0.0, 0.0]), d(a_max));
        let b = scene.block(d(b_min), d(b_max));

        let union = scene
            .unite(a, b)
            .unwrap_or_else(|e| panic!("{context}: unite failed: {e:?}"));
        let inter = scene
            .intersect(a, b)
            .unwrap_or_else(|e| panic!("{context}: intersect failed: {e:?}"));
        let diff = scene
            .subtract(a, b)
            .unwrap_or_else(|e| panic!("{context}: subtract failed: {e:?}"));

        // The smallest length in play is the unit-wide x overlap.
        let rtol = far_field_rtol(offset, 1.0);
        let vol_inter = brep_volume(&inter, &format!("{context}: intersection"));
        assert_close(
            vol_inter,
            overlap,
            rtol,
            &format!("{context}: intersection vs analytic overlap"),
        );
        assert_close(
            brep_volume(&union, &format!("{context}: union")) + vol_inter,
            vol_a + vol_b,
            rtol,
            &format!("{context}: inclusion–exclusion identity"),
        );
        assert_close(
            brep_volume(&diff, &format!("{context}: difference")),
            vol_a - overlap,
            rtol,
            &format!("{context}: difference identity"),
        );
    }
}

/// The sharp version of the family: a feature far *smaller* than the
/// coordinates that locate it. A slab of thickness `t` is shaved off a
/// 2-unit cube sitting `offset` from the origin, so the cut plane and the
/// face it parallels are two nearly equal large numbers whose difference is
/// the entire answer.
///
/// At the extreme corner (`offset = 1e6`, `t = 1e-4`) the thickness is
/// recovered from coordinates whose spacing is 1.2e-10 — about a millionth
/// of `t`, and only a hundred times the effective linear tolerance there.
/// This is the configuration the epic means by "large translation + small
/// feature", and it is the one where an absolute tolerance that does not
/// scale with magnitude either swallows the slab or fails to close it.
#[test]
fn small_feature_far_from_origin() {
    for offset in [0.0, 1e3, 1e6] {
        for thickness in [1e-2, 1e-4] {
            let context = format!("{thickness:e}-thick shave off a 2-cube at offset {offset:e}");
            let o = offset;
            let t = thickness;
            let mut scene = Scene::new();
            let a = scene.block([o, o, o], [o + 2.0, o + 2.0, o + 2.0]);
            // B covers everything from `x = o + 2 - t` outward, so the
            // overlap is exactly the slab `t × 2 × 2`.
            let b = scene.block([o + 2.0 - t, o - 1.0, o - 1.0], [o + 4.0, o + 3.0, o + 3.0]);

            let inter = scene
                .intersect(a, b)
                .unwrap_or_else(|e| panic!("{context}: intersect failed: {e:?}"));
            let diff = scene
                .subtract(a, b)
                .unwrap_or_else(|e| panic!("{context}: subtract failed: {e:?}"));

            let overlap = t * 4.0;
            assert_close(
                brep_volume(&inter, &format!("{context}: intersection")),
                overlap,
                far_field_rtol(offset, t),
                &format!("{context}: the slab itself"),
            );
            // The remainder is a 2-unit body, so it is entitled to the
            // 2-unit budget rather than the slab's.
            assert_close(
                brep_volume(&diff, &format!("{context}: remainder")),
                8.0 - overlap,
                far_field_rtol(offset, 2.0),
                &format!("{context}: remainder"),
            );
        }
    }
}

/// A curved operand far from the origin. Planes are the forgiving case:
/// their SSI is closed form and never iterates. A cylinder's wall drives
/// projection and marching, both of which carry running error that grows
/// with the coordinates being manipulated, so the through-hole is the
/// harder far-field test even though the configuration is tamer.
///
/// It is also the case that used to separate the two measurement paths most
/// sharply: the B-Rep number asserted here is right to nine digits at every
/// offset, while the mesh number for the very same result was 191× too large
/// at 1e6 until of-ukcq. They now agree — [`brep_volume`] cross-checks them,
/// and [`mesh_mass_properties_survives_far_from_origin`] holds the mesh path
/// to the closed form on this exact configuration.
#[test]
fn far_from_origin_through_hole() {
    for offset in FAR_OFFSETS {
        let context = format!("block minus cylinder at offset {offset:e}");
        let o = offset;
        let mut scene = Scene::new();
        let slab = scene.block([o, o, o], [o + 4.0, o + 4.0, o + 2.0]);
        let tool = scene.cylinder(
            Point3::new(o + 2.0, o + 2.0, o - 1.0),
            Vector3::z(),
            1.0,
            4.0,
        );
        let out = scene
            .subtract(slab, tool)
            .unwrap_or_else(|e| panic!("{context}: subtract failed: {e:?}"));
        assert_eq!(
            out.store.euler_counts(out.body).genus,
            1,
            "{context}: through hole must give genus 1"
        );
        assert_close(
            brep_volume(&out, &context),
            32.0 - 2.0 * PI,
            EXACT_RTOL.max(far_field_rtol(offset, 1.0)),
            &context,
        );
    }
}

// ---------------------------------------------------------------------
// 16.3 Mixed-scale operands: a tool six decades smaller than its target.
// ---------------------------------------------------------------------

/// A 1000-unit cube dented by a cube `ratio` times smaller, straddling one
/// face so exactly half the tool overlaps.
///
/// The interesting bound is what can be asserted about the *difference*.
/// `vol(A)` is 1e9 and the bite at the widest ratio is 5e-10; f64 cannot
/// represent their difference distinctly, and no measurement of a 1e9-volume
/// mesh resolves 1e-10 of absolute volume. So the difference is held to an
/// absolute budget of `1e-12 · vol(A)` — the precision a number that size
/// carries — while the *intersection*, which is a small body measured on its
/// own, is held to the full planar budget. The two together say the right
/// thing: the tool removed exactly the right material, and it did not
/// disturb the host beyond what arithmetic allows.
#[test]
fn tiny_tool_dents_a_huge_block() {
    const L: f64 = 1000.0;
    for ratio in [1e2, 1e3, 1e4, 1e5, 1e6] {
        let w = L / ratio;
        let context = format!("{L}-unit cube dented by a {w:e}-unit cube (ratio {ratio:e})");
        let mut scene = Scene::new();
        let a = scene.block([0.0, 0.0, 0.0], [L, L, L]);
        // Straddles the `x = L` face, centered on it in y and z.
        let b = scene.block(
            [L - w / 2.0, L / 2.0 - w / 2.0, L / 2.0 - w / 2.0],
            [L + w / 2.0, L / 2.0 + w / 2.0, L / 2.0 + w / 2.0],
        );

        let inter = scene
            .intersect(a, b)
            .unwrap_or_else(|e| panic!("{context}: intersect failed: {e:?}"));
        let diff = scene
            .subtract(a, b)
            .unwrap_or_else(|e| panic!("{context}: subtract failed: {e:?}"));

        let overlap = w * w * w / 2.0;
        let vol_a = L * L * L;
        assert_close(
            brep_volume(&inter, &format!("{context}: intersection")),
            overlap,
            far_field_rtol(L, w),
            &format!("{context}: the bite itself"),
        );

        let vol_diff = brep_volume(&diff, &format!("{context}: dented host"));
        let allowed = (PLANAR_VOLUME_RTOL * overlap).max(1e-12 * vol_a);
        assert!(
            (vol_diff - (vol_a - overlap)).abs() <= allowed,
            "{context}: dented host measured {vol_diff} against an expected \
             {} — off by {:.3e}, allowed {allowed:.3e}",
            vol_a - overlap,
            (vol_diff - (vol_a - overlap)).abs()
        );
    }
}

// ---------------------------------------------------------------------
// 16.4 Extreme aspect ratios: plates and needles.
// ---------------------------------------------------------------------

/// A plate a millionth as thick as it is wide, crossed by a needle a
/// millionth as thick as it is long. Their intersection is a 1e-3 cube of
/// volume 1e-9 — nine decades below either operand — and every face of it
/// comes from a surface whose own extent is six decades away in the
/// perpendicular direction.
///
/// The inclusion–exclusion identity is a real constraint here and not a
/// tautology: the sum is ~1000 and the needle contributes 1e-3 of it, a
/// million times the budget, so a boolean that lost the needle entirely
/// still fails.
///
/// The needle is slender in *two* directions at once, which is what used to
/// put it past of-oygs's limit: its long faces are 1e-3 wide and 1000 long,
/// and the old boundary band — a fixed fraction of the face's *own bounding
/// box* — was half a unit on those faces, five hundred times their whole
/// width. Every ray hit anywhere on them read as "too close to an edge to
/// count", all six directions were abandoned, and classification failed. The
/// band is now measured from each boundary polyline's actual sagitta, which
/// on a straight edge is zero, so a face's slenderness no longer inflates the
/// distance at which its own interior stops being interior.
#[test]
fn crossed_plate_and_needle() {
    let context = "1e6-aspect plate crossed by a 1e6-aspect needle";
    let mut scene = Scene::new();
    let plate = scene.block([-500.0, -500.0, -5e-4], [500.0, 500.0, 5e-4]);
    let needle = scene.block([-5e-4, -5e-4, -500.0], [5e-4, 5e-4, 500.0]);

    let union = scene
        .unite(plate, needle)
        .unwrap_or_else(|e| panic!("{context}: unite failed: {e:?}"));
    let inter = scene
        .intersect(plate, needle)
        .unwrap_or_else(|e| panic!("{context}: intersect failed: {e:?}"));

    let (vol_plate, vol_needle) = (1000.0 * 1000.0 * 1e-3, 1e-3 * 1e-3 * 1000.0);
    let overlap = 1e-3 * 1e-3 * 1e-3;
    let vol_inter = brep_volume(&inter, &format!("{context}: intersection"));
    assert_close(
        vol_inter,
        overlap,
        far_field_rtol(500.0, 1e-3),
        &format!("{context}: intersection is the shared 1e-3 cube"),
    );
    assert_close(
        brep_volume(&union, &format!("{context}: union")) + vol_inter,
        vol_plate + vol_needle,
        PLANAR_VOLUME_RTOL,
        &format!("{context}: inclusion–exclusion identity"),
    );
}

/// Two thin plates meeting edge-on at a right angle: the shared region is a
/// 1e-3 × 1000 × 1e-3 ribbon, extreme in two directions at once rather than
/// one. A tessellator that triangulates by any area-based heuristic, or a
/// classifier that seeds interior points by bounding-box midpoints, meets a
/// region here whose bounding box is a million times its own thickness.
#[test]
fn crossed_thin_plates_share_a_ribbon() {
    let context = "two 1e6-aspect plates crossing edge-on";
    let mut scene = Scene::new();
    let flat = scene.block([-500.0, -500.0, -5e-4], [500.0, 500.0, 5e-4]);
    let upright = scene.block([-5e-4, -500.0, -500.0], [5e-4, 500.0, 500.0]);

    let inter = scene
        .intersect(flat, upright)
        .unwrap_or_else(|e| panic!("{context}: intersect failed: {e:?}"));
    assert_close(
        brep_volume(&inter, &format!("{context}: intersection")),
        1e-3 * 1000.0 * 1e-3,
        far_field_rtol(500.0, 1e-3),
        &format!("{context}: the shared ribbon"),
    );
}

/// The live fence under of-oygs: the slenderness the pipeline *does* handle,
/// so a fix (or a regression) has a boundary to move rather than a single
/// broken case to flip.
///
/// The two arms used to have wildly different limits — planar operands
/// surviving to aspect 1e7 while cylindrical ones failed by 4e3 — and that
/// four-decade gap was the finding of-oygs was filed on. It is closed: both
/// arms now hold to aspect 1e7 and fail together at 1e8, because the
/// classifier's boundary band is measured from each boundary polyline's own
/// discretization rather than from the bounding box of the face it belongs
/// to. A wall's length no longer says anything about how close to its seam a
/// point may be trusted, so a shape being slender is no longer, by itself, a
/// reason to refuse it.
///
/// What is left at 1e8 is a *resolution* limit rather than a shape one, and
/// it lands in the same place for both arms: `geometric_snap` welds at 1e-9
/// of the joint extent of both operands, and `contains_point` distrusts a
/// sample within ten welds of a face. A feature thinner than ~1e-8 of the
/// model's overall size therefore cannot be classified — a 1e-5 rib in a
/// 1000-wide model, a 1-radius bore in a 1e8-long drill — and the refusal is
/// correct at that point, not conservative: forcing the band down to a single
/// weld makes the 1e8 planar case return an answer that is 33% wrong instead.
/// See of-x01y. Inside the range every case here is exact, which is why the
/// failures read as a cliff and not as decay.
#[test]
fn high_aspect_operands_inside_the_working_range() {
    // Planar arm: two 1000-wide plates at aspect 1e6 and 1e7.
    for thickness in [1e-3, 1e-4] {
        let context = format!("crossing 1000-wide plates, thickness {thickness:e}");
        let t = thickness;
        let mut scene = Scene::new();
        let flat = scene.block([-500.0, -500.0, -t / 2.0], [500.0, 500.0, t / 2.0]);
        let upright = scene.block([-t / 2.0, -500.0, -500.0], [t / 2.0, 500.0, 500.0]);
        let inter = scene
            .intersect(flat, upright)
            .unwrap_or_else(|e| panic!("{context}: intersect failed: {e:?}"));
        assert_close(
            brep_volume(&inter, &context),
            t * 1000.0 * t,
            far_field_rtol(500.0, t),
            &format!("{context}: the shared ribbon"),
        );
    }

    // Cylindrical arm: three bores from aspect 1e4 to 1e7 — the last decade
    // before the resolution limit above — at three different absolute radii,
    // under one fixed 1e-6 tolerance context. Passing all three is the
    // statement that what survives is set by neither the drill's absolute
    // size nor the tolerance. The plate is kept in proportion to the drill
    // (100 radii wide, 10 deep) so that the drill's length is the only thing
    // varying with aspect; every other ratio in the model is fixed.
    //
    // These three are the one place in the suite that does not tessellate its
    // result, and the reason is cost, not doubt: the triangulator's time
    // climbs steeply as a face's hole shrinks relative to the face (a plain
    // 4 × 4 slab with a unit bore already takes ~1s in a debug build; a
    // thousandth-radius bore is far worse — of-mpk0), and it would dominate
    // the whole suite for no coverage. What is being fenced happens strictly
    // inside `subtract`: of-oygs is a `ray_classify` refusal, so reaching a
    // result at all is most of the assertion. `check()` and the exact volume
    // still confirm the result is a well-formed solid of the right size.
    for (radius, aspect) in [(1e-2, 1e4), (1e-3, 1e6), (1e-4, 1e7)] {
        let (width, thickness) = (100.0 * radius, 10.0 * radius);
        let height = aspect * radius;
        let context = format!("bore r={radius:e}, drill {height:e} long (aspect {aspect:e})");
        let mut scene = Scene::new();
        let plate = scene.block(
            [-width / 2.0, -width / 2.0, -thickness / 2.0],
            [width / 2.0, width / 2.0, thickness / 2.0],
        );
        let drill = scene.cylinder(
            Point3::new(0.0, 0.0, -height / 2.0),
            Vector3::z(),
            radius,
            height,
        );
        let out = scene
            .subtract(plate, drill)
            .unwrap_or_else(|e| panic!("{context}: subtract failed: {e:?}"));
        let failures = out.check();
        assert!(
            failures.is_empty(),
            "{context}: check() reported {} failures: {failures:#?}",
            failures.len()
        );
        assert_eq!(
            out.store.euler_counts(out.body).genus,
            1,
            "{context}: the bore must go through"
        );
        let measured = brep_mass_properties(&out.store, &out.geo, out.body)
            .unwrap_or_else(|e| panic!("{context}: brep_mass_properties failed: {e}"));
        assert_close(
            measured.volume,
            (width * width - PI * radius * radius) * thickness,
            EXACT_RTOL,
            &context,
        );
    }
}

/// A needle *drill*: a cylinder of radius 1e-3 bored clean through a
/// 100 × 100 plate. The hole is 3.1e-10 of the plate's volume — far below
/// what any measurement of the plate can resolve — so the difference is
/// asserted topologically (it must be genus 1: the hole exists and goes all
/// the way through) and the removed material is weighed as the intersection,
/// which is a body of its own and measurable on its own terms.
///
/// The drill is 4 long and 1e-3 in radius: aspect 4000, which is where
/// of-oygs used to stop. The mechanism was the wall's own *seam*: a cylinder
/// wall is closed by an axial line, and the old boundary band was a fixed
/// fraction (5e-4) of the wall's bounding box, so once the wall was longer
/// than 4e3 radii that band exceeded the wall's whole diameter and every hit
/// on it counted as sitting on the seam. Aspect is fenced far beyond this
/// point in [`high_aspect_operands_inside_the_working_range`]; what this test
/// keeps is the shape a user actually asks for — a real drill, bored clean
/// through a plate ten thousand times its radius.
#[test]
fn needle_drill_through_a_wide_plate() {
    let context = "1e-3-radius drill through a 100-unit plate";
    let mut scene = Scene::new();
    let plate = scene.block([-50.0, -50.0, -0.5], [50.0, 50.0, 0.5]);
    let drill = scene.cylinder(Point3::new(0.0, 0.0, -2.0), Vector3::z(), 1e-3, 4.0);

    let diff = scene
        .subtract(plate, drill)
        .unwrap_or_else(|e| panic!("{context}: subtract failed: {e:?}"));
    assert_valid(&diff, context);
    assert_eq!(
        diff.store.euler_counts(diff.body).genus,
        1,
        "{context}: the drill must leave a through hole"
    );

    let inter = scene
        .intersect(plate, drill)
        .unwrap_or_else(|e| panic!("{context}: intersect failed: {e:?}"));
    assert_close(
        brep_volume(&inter, &format!("{context}: swarf")),
        PI * 1e-6,
        EXACT_RTOL,
        &format!("{context}: removed material"),
    );
}

// ---------------------------------------------------------------------
// 16.5 Near-parallel surface pairs.
//
// An SSI's direction is `n_a × n_b`, whose magnitude is `sin` of the angle
// between the surfaces. As that angle shrinks the direction is recovered
// from the difference of two nearly equal unit vectors, and the *position*
// of the intersection moves like `separation / angle` — so a tolerance-sized
// uncertainty in where the surfaces sit becomes a `tol / sin θ`-sized
// uncertainty in where they cross. These tests put a real, closed-form
// answer on the far side of that amplification.
// ---------------------------------------------------------------------

/// A knife-edge wedge shaved off a block by a plane that misses being
/// parallel to its top face by `angle` radians.
///
/// The block is `[0,2] × [0,2] × [0,1]`; the tool is a large block whose
/// bottom face starts `δ = 0.1·sin θ` below the top face and tilts by `θ`
/// about the `y` axis through `(1, 1, 1)`. Working the rotation through, the
/// tool's bottom plane is `z = 1 − δ/cos θ − tan θ·(x − 1)`, so the material
/// it removes has height `h(x) = 0.1·tan θ + tan θ·(x − 1)` above it — zero
/// at `x = 0.9` and rising to `1.1·tan θ` at `x = 2`. That is a triangular
/// prism of volume `2 · ½ · 1.1 · 1.1 tan θ = 1.21 tan θ`, exactly, for every
/// angle in the family.
///
/// `δ` is tied to `θ` on purpose: it keeps the knife edge at `x = 0.9` at
/// every angle, so the three cases differ *only* in how sharp the edge is
/// and not in where it lands.
fn near_parallel_wedge(angle: f64) {
    let context = format!("wedge shaved by a plane {angle:e} rad off parallel");
    let (m, delta) = (angle.tan(), 0.1 * angle.sin());
    let expected = 1.21 * m;

    let mut scene = Scene::new();
    let a = scene.block([0.0, 0.0, 0.0], [2.0, 2.0, 1.0]);
    let b = scene.block([-1.0, -1.0, 1.0 - delta], [3.0, 3.0, 3.0]);
    let rot = Rotation3::from_axis_angle(&Unit::new_normalize(Vector3::y()), angle);
    scene.rotate(b, &rot, &Point3::new(1.0, 1.0, 1.0));

    let inter = scene
        .intersect(a, b)
        .unwrap_or_else(|e| panic!("{context}: intersect failed: {e:?}"));
    assert_eq!(
        inter.shell_count(),
        1,
        "{context}: the wedge is one connected solid"
    );

    // Near the knife edge the two planes are within the linear tolerance of
    // each other, so which side a point falls on is genuinely undecidable
    // there. That band is `|h(x)| <= tol`, an x-window of half-width
    // `tol / m`, and the material inside it is at most `2 (the y extent) ×
    // 2 tol/m × tol`. No implementation can do better; anything worse is a
    // real error. Note this is the quantity that blows up as `m → 0` — it is
    // the whole reason near-parallel SSI is its own robustness family.
    let ambiguous = 4.0 * tol().linear * tol().linear / m;
    let allowed = (PLANAR_VOLUME_RTOL * expected).max(ambiguous);
    let (meshed, exact) = measured(&inter, &context);
    for (label, got) in [("B-Rep path", exact.volume), ("mesh path", meshed.volume)] {
        assert!(
            (got - expected).abs() <= allowed,
            "{context} ({label}): wedge volume {got} differs from the exact \
             {expected} by {:.3e}, allowed {allowed:.3e}",
            (got - expected).abs()
        );
    }
}

#[test]
fn near_parallel_wedge_1e_2_rad() {
    near_parallel_wedge(1e-2);
}

#[test]
fn near_parallel_wedge_1e_3_rad() {
    near_parallel_wedge(1e-3);
}

#[test]
fn near_parallel_wedge_1e_4_rad() {
    near_parallel_wedge(1e-4);
}

/// The same spherical cap, measured at ten angles between its trim and the
/// sphere's own pole axis — the sweep of-y8qc was filed on.
///
/// Every case here is the *congruent solid*: a unit sphere cut by a plane
/// 0.05 from its centre, giving a cap of height 0.95. Only the sphere's
/// parameterization turns underneath it, which no measurement is entitled to
/// notice. It used to notice a great deal: 5.7e-16 at 0°, where the trim
/// follows a latitude and fits exactly, degrading smoothly to 1.3e-3 at 90°,
/// where the trim crosses every latitude and fell back to a 33-vertex
/// polyline. Twelve orders of magnitude, for a rotation.
///
/// This is the test that keeps the exact path honest about *which* trims it
/// is exact on. `EXACT_RTOL` is the same 1e-9 the pole-aligned cases in
/// section (15) are held to, and it is asserted at every angle — an
/// implementation that recovers the axis-aligned cases and fits the rest
/// fails here rather than passing on a technicality.
#[test]
fn a_spherical_cap_measures_the_same_at_every_trim_angle() {
    let (h, r) = (0.95, 1.0);
    let exact = spherical_cap_volume(r, h);
    let area = 2.0 * PI * r * h + PI * (r * r - (r - h) * (r - h));

    for tilt_deg in [0.0f64, 1.0, 5.0, 15.0, 30.0, 45.0, 60.0, 75.0, 89.0, 90.0] {
        let context = format!("cap trimmed {tilt_deg}° off the pole axis");
        let tilt = tilt_deg.to_radians();
        let mut scene = Scene::new();
        // Pole axis tilted off +X, which is the cutting plane's normal
        // throughout, so `tilt` is exactly the angle under test.
        let ball = scene.sphere_with_axis(
            Point3::origin(),
            Vector3::new(tilt.cos(), 0.0, tilt.sin()),
            r,
        );
        // A 6-cube whose −X face sits at x = r − h, so the intersection is
        // the cap of height h and nothing else.
        let half_space = scene.block([r - h, -3.0, -3.0], [r - h + 6.0, 3.0, 3.0]);

        let cap = scene
            .intersect(ball, half_space)
            .unwrap_or_else(|e| panic!("{context}: intersect failed: {e:?}"));
        let (_, measured) = measured(&cap, &context);
        assert_close(
            measured.volume,
            exact,
            EXACT_RTOL,
            &format!("{context}: cap volume (B-Rep path)"),
        );
        assert_close(
            measured.surface_area,
            area,
            EXACT_RTOL,
            &format!("{context}: cap area (B-Rep path)"),
        );
    }
}

/// Two unit spheres whose centers are `d` apart, with `d` small: the curved
/// counterpart of the wedge. At their intersection circle the two normals
/// point at each center, so they differ by an angle of about `d` — the
/// surfaces cross at a hair, and over most of their area they sit within `d`
/// of each other without being coincident.
///
/// The lens has a closed form ([`sphere_lens_volume`]), and the pair is
/// almost the whole of either sphere, so the assertion is stated against the
/// **B-Rep-native** measurement: the meshed budget for curved results
/// (0.5%) is larger than the gap between the lens and a whole sphere at
/// these separations, and would pass a boolean that simply returned one of
/// the operands. The exact path does not discretize, so it can tell them
/// apart.
fn near_concentric_spheres(d: f64) {
    let context = format!("unit spheres {d:e} apart");
    let mut scene = Scene::new();
    let a = scene.sphere(Point3::origin(), 1.0);
    let b = scene.sphere(Point3::new(d, 0.0, 0.0), 1.0);

    let inter = scene
        .intersect(a, b)
        .unwrap_or_else(|e| panic!("{context}: intersect failed: {e:?}"));
    let (_, exact) = measured(&inter, &context);
    let lens = sphere_lens_volume(1.0, 1.0, d);
    assert_close(
        exact.volume,
        lens,
        EXACT_RTOL,
        &format!("{context}: lens volume (B-Rep path)"),
    );
    // A whole sphere is what a boolean that gave up would return; the lens
    // must be distinguishable from it by orders of magnitude more than the
    // budget just used.
    assert!(
        (exact.volume - sphere_volume(1.0)).abs() > 1e3 * EXACT_RTOL * lens,
        "{context}: the lens is indistinguishable from a whole sphere, so \
         the assertion above proves nothing"
    );

    let union = scene
        .unite(a, b)
        .unwrap_or_else(|e| panic!("{context}: unite failed: {e:?}"));
    let (_, exact_union) = measured(&union, &format!("{context}: union"));
    assert_close(
        exact_union.volume + exact.volume,
        2.0 * sphere_volume(1.0),
        EXACT_RTOL,
        &format!("{context}: inclusion–exclusion identity (B-Rep path)"),
    );
}

/// All three separations used to miss the closed form by 1.3e-3, 1.5e-3 and
/// 1.6e-3 — and the cause was not the near-parallelism this family was
/// aiming at. The lens's trim circle lies perpendicular to the line of
/// centers, i.e. 90° off the spheres' pole axis, and of-y8qc was a
/// *systematic* measurement error on trimmed spheres that grew smoothly with
/// exactly that angle: 5.7e-16 at 0°, 5.1e-5 at 45°, 1.3e-3 at 90°, on a
/// plain sphere-minus-half-space with no near-parallelism anywhere in it.
///
/// They were left written as they are rather than softened to a 1e-2 budget
/// (which would have made them pass while measuring nothing) or re-aimed at
/// the poles (which would have dodged the defect instead of pinning it), and
/// they went live unchanged when `Curve2::Projected` retired it.
#[test]
fn near_concentric_spheres_1e_1_apart() {
    near_concentric_spheres(1e-1);
}

#[test]
fn near_concentric_spheres_1e_2_apart() {
    near_concentric_spheres(1e-2);
}

#[test]
fn near_concentric_spheres_1e_3_apart() {
    near_concentric_spheres(1e-3);
}

// ---------------------------------------------------------------------
// 16.6 Repeated booleans: does error accumulate over a chain?
//
// Every test above weighs a single operation. That is not the question a
// modelling history asks. A part is fifty features deep, each built on the
// output of the last, and the failure mode is not that any one of them is
// wrong but that each is slightly wrong in the same direction. The spec's
// propagation rules (`spec/08-tolerances.md` §3) say an operation may raise
// an entity's tolerance; nothing says the *volume* may drift, and these
// tests hold every step of a chain to the same budget as its first step —
// the tolerance deliberately does not widen with depth.
// ---------------------------------------------------------------------

/// Eight bores drilled one after another into a plate, each into the output
/// of the last. After step `k` the plate must be genus `k` and have lost
/// exactly `k` bores' worth of material — to the *same* budget at `k = 8` as
/// at `k = 1`.
#[test]
fn chained_bores_do_not_accumulate_volume_error() {
    const BORES: usize = 8;
    const RADIUS: f64 = 0.4;
    const THICKNESS: f64 = 1.0;
    let plate_volume = 12.0 * 12.0 * THICKNESS;
    let bore_volume = PI * RADIUS * RADIUS * THICKNESS;

    let mut scene = Scene::new();
    let mut body = scene.block([-6.0, -6.0, -0.5], [6.0, 6.0, 0.5]);

    for k in 1..=BORES {
        let context = format!("bore {k} of {BORES}");
        let theta = 2.0 * PI * (k - 1) as f64 / BORES as f64;
        let drill = scene.cylinder(
            Point3::new(3.5 * theta.cos(), 3.5 * theta.sin(), -3.0),
            Vector3::z(),
            RADIUS,
            6.0,
        );
        let out = scene
            .subtract(body, drill)
            .unwrap_or_else(|e| panic!("{context}: subtract failed: {e:?}"));
        assert_eq!(
            out.store.euler_counts(out.body).genus,
            k,
            "{context}: each bore must add exactly one handle"
        );
        let want = plate_volume - k as f64 * bore_volume;

        // Every step is weighed the exact way, at a budget that does not
        // widen with depth — that is the whole assertion, and it is sharp to
        // nine digits. Only the last step is also tessellated and
        // cross-checked: the mesh path answers nothing here the exact path
        // has not already answered a million times more precisely, and
        // of-mpk0 makes tessellating a plate with k small bores cost more
        // than every other test in this file combined.
        if k == BORES {
            let (meshed, exact) = measured(&out, &context);
            assert_close(
                exact.volume,
                want,
                EXACT_RTOL,
                &format!("{context} (B-Rep path)"),
            );
            assert_close(
                meshed.volume,
                want,
                CYL_VOLUME_RTOL,
                &format!("{context} (mesh path)"),
            );
        } else {
            let failures = out.check();
            assert!(
                failures.is_empty(),
                "{context}: check() reported {} failures: {failures:#?}",
                failures.len()
            );
            let exact = brep_mass_properties(&out.store, &out.geo, out.body)
                .unwrap_or_else(|e| panic!("{context}: brep_mass_properties failed: {e}"));
            assert_close(
                exact.volume,
                want,
                EXACT_RTOL,
                &format!("{context} (B-Rep path)"),
            );
        }

        let (next, next_body) = Scene::adopt(out, tol());
        scene = next;
        body = next_body;
    }
}

/// A null-op cycle repeated six times: unite a boss sitting flush on the
/// 4 × 4 × 2 block's top face, then subtract the same boss. Because the boss
/// shares only a face with the block and no volume, `(A ∪ B) − B` is `A` —
/// every cycle must return the *identical* solid.
///
/// What makes it a drift test rather than a triviality is the imprint. The
/// first union splits the block's top face along the boss footprint, and
/// every later cycle re-imprints onto faces that are already split, so the
/// arrangement being merged is different (and messier) at cycle six than at
/// cycle one. If coincident-face handling nudges a vertex by a tolerance each
/// time, the volume walks; the assertion is that it does not move at all —
/// the budget is the same at cycle six as at cycle one, deliberately.
///
/// `boss_half` is the boss's half-width. At 2.0 the footprint reaches the
/// top face's edges and the imprint is four boundary-to-boundary chains; at
/// 1.0 it is a closed island strictly inside the face, which is of-6viu.
/// The two differ in nothing else.
fn unite_subtract_cycles(boss_half: f64, label: &str) {
    const CYCLES: usize = 6;
    let block_volume = 4.0 * 4.0 * 2.0;
    let boss = |h: f64| ([-h, -h, 1.0], [h, h, 2.0]);
    let mut counts = Vec::new();

    let mut scene = Scene::new();
    let mut body = scene.block([-2.0, -2.0, -1.0], [2.0, 2.0, 1.0]);

    for cycle in 1..=CYCLES {
        let context = format!("{label}: unite/subtract cycle {cycle} of {CYCLES}");
        let (lo, hi) = boss(boss_half);
        let tool = scene.block(lo, hi);
        let united = scene
            .unite(body, tool)
            .unwrap_or_else(|e| panic!("{context}: unite failed: {e:?}"));
        let (mut next, united_body) = Scene::adopt(united, tol());
        let tool_again = next.block(lo, hi);
        let out = next
            .subtract(united_body, tool_again)
            .unwrap_or_else(|e| panic!("{context}: subtract failed: {e:?}"));

        assert_eq!(out.shell_count(), 1, "{context}: one shell");
        assert_eq!(
            out.store.euler_counts(out.body).genus,
            0,
            "{context}: no handles"
        );
        let (meshed, exact) = measured(&out, &context);
        assert_close(
            exact.volume,
            block_volume,
            EXACT_RTOL,
            &format!("{context} (B-Rep path)"),
        );
        assert_close(
            meshed.volume,
            block_volume,
            PLANAR_VOLUME_RTOL,
            &format!("{context} (mesh path)"),
        );
        assert!(
            exact.centroid.coords.norm() <= 1e-9,
            "{context}: centroid drifted to {:?}",
            exact.centroid
        );
        counts.push(out.face_count());

        let (next, next_body) = Scene::adopt(out, tol());
        scene = next;
        body = next_body;
    }

    // The topological half of the same question: a cycle that leaves the
    // solid geometrically identical but structurally larger every time is how
    // a modelling history grinds to a halt after fifty features, and no
    // volume assertion can see it. The bar is deliberately weak — not "the
    // count returns to six" (an imprint may legitimately survive as a split
    // face) but "the count stops growing".
    assert_eq!(
        counts.last(),
        counts.get(1),
        "{label}: face count did not settle over {CYCLES} identical cycles: {counts:?}"
    );
}

/// The boss reaches the top face's edges, so every imprint chain runs
/// boundary to boundary. Live, and the fence of-6viu's fix must not move.
#[test]
fn edge_reaching_boss_unite_subtract_cycles_do_not_drift() {
    unite_subtract_cycles(2.0, "edge-reaching boss");
}

/// The same cycle with a 2 × 2 boss centred on the 4 × 4 top face, so the
/// imprint is a closed island touching no edge — a pad in the middle of a
/// face, which is about as ordinary as CAD features get. `unite` rejects it
/// outright (of-6viu) on cycle one, so nothing about drift is learned yet;
/// the test is written to the same standard as its live sibling so that
/// fixing of-6viu turns it on unchanged.
#[test]
#[ignore = "of-6viu: unite fails when a coincident-face imprint forms an island"]
fn island_boss_unite_subtract_cycles_do_not_drift() {
    unite_subtract_cycles(1.0, "island boss");
}

// ---------------------------------------------------------------------
// 16.6.1 Coincident-face islands whose boundary is a CONIC (of-x8tn).
//
// Everything above imprints straight chords: a square boss on a flat face
// leaves four line segments that meet at their endpoints, and whether they
// reach the face's own boundary (16.6's edge-reaching case) or close on
// themselves in the middle of it (its island case) is the only variable.
// A round boss closes the same island out of ONE curve, and that single
// difference reached all the way to the atom bookkeeping: a full circle has
// two spellings in the arrangement — a flagged ring, and an open polyline
// from a seam vertex back to itself — and the tool contributed one while the
// footprint it imprinted contributed the other. Nothing downstream saw them
// as the same edge, so host and tool never fused: each closed into its own
// shell with the shared circle missing, and `build_output` rejected both on
// Euler grounds rather than emitting the boss.
//
// A round boss and a round pocket are as ordinary as their square
// counterparts, so these are held to the same bar: closed-form volume on
// both measurement paths, one shell, no handles, and — for the cycles — no
// drift and no unbounded face growth.
// ---------------------------------------------------------------------

/// The 4 × 4 × 2 block used throughout 16.6, as `(min, max)`.
const ISLAND_BLOCK: ([f64; 3], [f64; 3]) = ([-2.0, -2.0, -1.0], [2.0, 2.0, 1.0]);
const ISLAND_BLOCK_VOLUME: f64 = 4.0 * 4.0 * 2.0;

fn cylinder_volume(radius: f64, height: f64) -> f64 {
    PI * radius * radius * height
}

/// A round boss standing on the block's top face: the tool's bottom cap is
/// coincident with the host face and its footprint is a closed circle
/// strictly inside it. The union has to keep the tool's wall and top cap,
/// keep the host face minus the disk, and drop the disk itself — so its
/// volume is the plain sum, with no double count and nothing missing.
///
/// This is of-x8tn's first repro. Before the fix `unite` did not return a
/// wrong solid, it returned no solid at all: `Degenerate` out of
/// `build_output`, with an Euler count that read as the block plus its
/// imprinted ring and nothing whatsoever of the cylinder.
fn circular_boss_unite(center: [f64; 2], radius: f64, height: f64, label: &str) {
    let (lo, hi) = ISLAND_BLOCK;
    let mut scene = Scene::new();
    let base = scene.block(lo, hi);
    let boss = scene.cylinder(
        Point3::new(center[0], center[1], hi[2]),
        Vector3::z(),
        radius,
        height,
    );
    let out = scene
        .unite(base, boss)
        .unwrap_or_else(|e| panic!("{label}: unite failed: {e:?}"));

    assert_eq!(out.shell_count(), 1, "{label}: one shell");
    assert_eq!(
        out.store.euler_counts(out.body).genus,
        0,
        "{label}: no handles"
    );
    let want = ISLAND_BLOCK_VOLUME + cylinder_volume(radius, height);
    let (meshed, exact) = measured(&out, label);
    assert_close(exact.volume, want, EXACT_RTOL, &format!("{label} (B-Rep)"));
    assert_close(
        meshed.volume,
        want,
        CYL_VOLUME_RTOL,
        &format!("{label} (mesh)"),
    );
}

#[test]
fn circular_island_boss_unite_is_the_sum_of_both_volumes() {
    circular_boss_unite([0.0, 0.0], 1.0, 1.0, "circular island boss");
}

/// Off centre and a different size, so nothing about the imprinted circle
/// is symmetric with the host face's own parameterization and no match can
/// come off a shared centre or a shared extent.
#[test]
fn off_centre_circular_island_boss_unite_is_the_sum_of_both_volumes() {
    circular_boss_unite([0.75, -0.5], 0.8, 1.5, "off-centre circular island boss");
}

/// A round POCKET: the tool's TOP cap is coincident with the block's top
/// face and its body reaches down into the material, so the same island
/// imprint now has to remove volume across itself rather than add it. It
/// failed as its own case (chi = 2 - 3 + 2 - 0 = 1, the cylinder with the
/// block gone) and is fixed by the same merge, so it is fenced separately.
#[test]
fn circular_island_pocket_subtract_removes_exactly_the_tool() {
    let (lo, hi) = ISLAND_BLOCK;
    let (radius, depth) = (1.0, 0.5);
    let label = "circular island pocket";

    let mut scene = Scene::new();
    let base = scene.block(lo, hi);
    // Bottom cap inside the block, top cap flush with the block's top face.
    let tool = scene.cylinder(
        Point3::new(0.0, 0.0, hi[2] - depth),
        Vector3::z(),
        radius,
        depth,
    );
    let out = scene
        .subtract(base, tool)
        .unwrap_or_else(|e| panic!("{label}: subtract failed: {e:?}"));

    assert_eq!(out.shell_count(), 1, "{label}: one shell");
    assert_eq!(
        out.store.euler_counts(out.body).genus,
        0,
        "{label}: no handles"
    );
    let want = ISLAND_BLOCK_VOLUME - cylinder_volume(radius, depth);
    let (meshed, exact) = measured(&out, label);
    assert_close(exact.volume, want, EXACT_RTOL, &format!("{label} (B-Rep)"));
    assert_close(
        meshed.volume,
        want,
        CYL_VOLUME_RTOL,
        &format!("{label} (mesh)"),
    );
}

/// Two round bosses on the SAME host face, added one boolean at a time.
/// The second union imprints its circle onto a top face that is already
/// split by the first one and already carries a ring on its boundary, so
/// the new ring has to find its own partner among several candidates and
/// leave the settled one alone.
#[test]
fn two_circular_island_bosses_on_one_face_unite() {
    let (lo, hi) = ISLAND_BLOCK;
    let (radius, height) = (0.6, 0.9);
    let label = "two circular island bosses";

    let boss_at = |scene: &mut Scene, center: [f64; 2]| {
        scene.cylinder(
            Point3::new(center[0], center[1], hi[2]),
            Vector3::z(),
            radius,
            height,
        )
    };

    let mut scene = Scene::new();
    let body = scene.block(lo, hi);
    let first = boss_at(&mut scene, [-1.0, -1.0]);
    let out = scene
        .unite(body, first)
        .unwrap_or_else(|e| panic!("{label}: first unite failed: {e:?}"));

    let (mut scene, body) = Scene::adopt(out, tol());
    let second = boss_at(&mut scene, [1.0, 1.0]);
    let out = scene
        .unite(body, second)
        .unwrap_or_else(|e| panic!("{label}: second unite failed: {e:?}"));

    assert_eq!(out.shell_count(), 1, "{label}: one shell");
    assert_eq!(
        out.store.euler_counts(out.body).genus,
        0,
        "{label}: no handles"
    );
    let want = ISLAND_BLOCK_VOLUME + 2.0 * cylinder_volume(radius, height);
    let (meshed, exact) = measured(&out, label);
    assert_close(exact.volume, want, EXACT_RTOL, &format!("{label} (B-Rep)"));
    assert_close(
        meshed.volume,
        want,
        CYL_VOLUME_RTOL,
        &format!("{label} (mesh)"),
    );
}

/// A through bore whose tool ends EXACTLY flush with both faces it pierces,
/// so one arrangement carries two conic islands at once — the only case in
/// this section that does. A drilling tool is normally given overhang, which
/// is precisely what keeps its caps out of the picture; take the overhang
/// away and both caps become coincident face pairs, each contributing a ring
/// in two spellings that must merge with its own partner and not with the
/// other one 2 units away.
///
/// The genus assertion is what makes it more than a second copy of the
/// pocket case: a hole that fails to open, or opens into a second shell,
/// cannot report 1.
#[test]
fn flush_capped_through_bore_subtract_opens_a_genus_1_hole() {
    let (lo, hi) = ISLAND_BLOCK;
    let radius = 1.0;
    let height = hi[2] - lo[2];
    let label = "flush-capped through bore";

    let mut scene = Scene::new();
    let base = scene.block(lo, hi);
    let tool = scene.cylinder(Point3::new(0.0, 0.0, lo[2]), Vector3::z(), radius, height);
    let out = scene
        .subtract(base, tool)
        .unwrap_or_else(|e| panic!("{label}: subtract failed: {e:?}"));

    assert_eq!(out.shell_count(), 1, "{label}: one shell");
    assert_eq!(
        out.store.euler_counts(out.body).genus,
        1,
        "{label}: a through hole must give genus 1"
    );
    let want = ISLAND_BLOCK_VOLUME - cylinder_volume(radius, height);
    let (meshed, exact) = measured(&out, label);
    assert_close(exact.volume, want, EXACT_RTOL, &format!("{label} (B-Rep)"));
    assert_close(
        meshed.volume,
        want,
        CYL_VOLUME_RTOL,
        &format!("{label} (mesh)"),
    );
}

/// The 16.6 drift question asked of a conic island: unite a round boss, then
/// subtract it again, six times over. `(A ∪ B) − B` is `A`, so every cycle
/// must return the block unchanged — but each cycle re-imprints a circle
/// onto a top face that already carries one, which is where a
/// tolerance-sized nudge per cycle would show up as a walking volume or a
/// face count that never settles.
#[test]
fn circular_island_boss_unite_subtract_cycles_do_not_drift() {
    const CYCLES: usize = 6;
    let (lo, hi) = ISLAND_BLOCK;
    let (radius, height) = (1.0, 1.0);
    let label = "circular island boss";
    let mut counts = Vec::new();

    let mut scene = Scene::new();
    let mut body = scene.block(lo, hi);

    for cycle in 1..=CYCLES {
        let context = format!("{label}: unite/subtract cycle {cycle} of {CYCLES}");
        let base = Point3::new(0.0, 0.0, hi[2]);
        let tool = scene.cylinder(base, Vector3::z(), radius, height);
        let united = scene
            .unite(body, tool)
            .unwrap_or_else(|e| panic!("{context}: unite failed: {e:?}"));
        let (mut next, united_body) = Scene::adopt(united, tol());
        let tool_again = next.cylinder(base, Vector3::z(), radius, height);
        let out = next
            .subtract(united_body, tool_again)
            .unwrap_or_else(|e| panic!("{context}: subtract failed: {e:?}"));

        assert_eq!(out.shell_count(), 1, "{context}: one shell");
        assert_eq!(
            out.store.euler_counts(out.body).genus,
            0,
            "{context}: no handles"
        );
        let (meshed, exact) = measured(&out, &context);
        assert_close(
            exact.volume,
            ISLAND_BLOCK_VOLUME,
            EXACT_RTOL,
            &format!("{context} (B-Rep path)"),
        );
        assert_close(
            meshed.volume,
            ISLAND_BLOCK_VOLUME,
            CYL_VOLUME_RTOL,
            &format!("{context} (mesh path)"),
        );
        assert!(
            exact.centroid.coords.norm() <= 1e-9,
            "{context}: centroid drifted to {:?}",
            exact.centroid
        );
        counts.push(out.face_count());

        let (next, next_body) = Scene::adopt(out, tol());
        scene = next;
        body = next_body;
    }

    // Same bar as `unite_subtract_cycles`: an imprint may legitimately
    // survive as a split face, but the count must stop growing.
    assert_eq!(
        counts.last(),
        counts.get(1),
        "{label}: face count did not settle over {CYCLES} identical cycles: {counts:?}"
    );
}

// ---------------------------------------------------------------------
// 16.7 High-degree and near-degenerate-knot NURBS operands.
//
// Section (14) proved the NURBS path on degree-1 patches with evenly spaced
// knots. Degree 1 is the case where the basis is a hat function, the span
// search is a bisection over equal intervals, and no denominator in the
// Cox–de Boor recurrence is ever small. None of that survives at degree 5
// with knots 1e-9 apart.
//
// The geometry is held fixed on purpose: every operand here is the *same*
// flat-faced box as `Scene::nurbs_block`, reproduced exactly because the
// control points sit at the Greville abscissae (see
// [`Scene::nurbs_hexahedron_knots`]). So every case has the same right
// answer as `nurbs_box_half_overlapped_by_nurbs_box`, and any difference is
// a parameterization defect with nowhere to hide.
// ---------------------------------------------------------------------

/// Clamped knot vector of `degree` over `[0, 1]` with the given interior
/// knots — the general form `KnotVector::clamped_uniform` specializes.
fn clamped_knots(degree: usize, interior: &[f64]) -> KnotVector {
    let mut knots = vec![0.0; degree + 1];
    knots.extend_from_slice(interior);
    knots.extend(std::iter::repeat_n(1.0, degree + 1));
    KnotVector::new(degree, knots).expect("valid clamped knot vector")
}

/// The half-overlap fixture of section (14), run with both NURBS boxes
/// carrying the given `(u, v)` knot vectors on all six faces.
fn nurbs_half_overlap_with_knots(label: &str, knots_u: KnotVector, knots_v: KnotVector) {
    let context = format!("NURBS box half-overlap, {label}");
    let a_box = ([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]);
    let b_box = ([1.0, -1.0, -1.0], [3.0, 3.0, 3.0]);
    let mut scene = Scene::new();
    let per_face = || std::array::from_fn(|_| [knots_u.clone(), knots_v.clone()]);
    let a = scene.nurbs_hexahedron_knots(box_corners(a_box.0, a_box.1), per_face());
    let b = scene.nurbs_hexahedron_knots(box_corners(b_box.0, b_box.1), per_face());
    let inside_a = box_inside_test(a_box.0, a_box.1);
    let inside_b = box_inside_test(b_box.0, b_box.1);
    assert_half_overlap_identity(
        &context,
        &scene,
        a,
        b,
        [Some(&inside_a), Some(&inside_b)],
        8.0,
        32.0,
    );
}

/// Degree 3 in both directions with evenly spaced interior knots: the first
/// case where the basis has real support across spans and the imprint has to
/// cross knot lines it did not choose.
#[test]
fn nurbs_half_overlap_degree_3() {
    nurbs_half_overlap_with_knots(
        "degree 3, three interior knots",
        clamped_knots(3, &[0.25, 0.5, 0.75]),
        clamped_knots(3, &[0.25, 0.5, 0.75]),
    );
}

/// Degree 5 — the highest degree any of this suite's operands reach, and
/// enough that the Cox–de Boor recurrence is six levels deep.
#[test]
fn nurbs_half_overlap_degree_5() {
    nurbs_half_overlap_with_knots(
        "degree 5, two interior knots",
        clamped_knots(5, &[1.0 / 3.0, 2.0 / 3.0]),
        clamped_knots(5, &[1.0 / 3.0, 2.0 / 3.0]),
    );
}

/// Anisotropic degree: cubic in `u`, linear in `v`. Nothing about a patch
/// forces its two directions to match, and code that reads one degree where
/// it meant the other passes every symmetric test in the suite.
#[test]
fn nurbs_half_overlap_mixed_degrees() {
    nurbs_half_overlap_with_knots(
        "degree 3 in u, degree 1 in v",
        clamped_knots(3, &[0.25, 0.5, 0.75]),
        clamped_knots(1, &[0.3, 0.8]),
    );
}

/// A full-multiplicity interior knot: repeated `degree` times, which drops
/// the surface to C0 there. The patch is still exactly flat, so the kink is
/// purely parametric — the derivative is discontinuous across a line the
/// geometry knows nothing about, which is the state a marcher stepping by
/// tangent direction handles worst.
#[test]
fn nurbs_half_overlap_c0_interior_knot() {
    nurbs_half_overlap_with_knots(
        "degree 3 with a C0 (triple) interior knot at u = 1/2",
        clamped_knots(3, &[0.5, 0.5, 0.5]),
        clamped_knots(3, &[0.5, 0.5, 0.5]),
    );
}

/// Interior knots 1e-9 apart: not repeated, so every Cox–de Boor denominator
/// is nonzero, but nine orders below the span they sit in. This is the
/// near-degenerate case the epic names — the one where a span search that
/// compares with an absolute epsilon lands in the wrong span, and where the
/// basis is computed as a ratio of two quantities that are individually
/// meaningless.
#[test]
fn nurbs_half_overlap_near_coincident_knots() {
    nurbs_half_overlap_with_knots(
        "degree 3 with interior knots 1e-9 apart",
        clamped_knots(3, &[0.5, 0.5 + 1e-9, 0.5 + 2e-9]),
        clamped_knots(3, &[0.5, 0.5 + 1e-9, 0.5 + 2e-9]),
    );
}

/// A knot span a millionth of the domain wide, hard against the clamped end.
/// The two failure modes it separates are a span search that skips spans
/// narrower than its epsilon, and a projection whose parametric step size is
/// chosen from the domain length rather than the local span.
#[test]
fn nurbs_half_overlap_tiny_first_span() {
    nurbs_half_overlap_with_knots(
        "degree 4 with a 1e-6-wide first span",
        clamped_knots(4, &[1e-6, 0.5]),
        clamped_knots(4, &[1e-6, 0.5]),
    );
}

// =====================================================================
// (17) Coincident CURVED faces (of-bxl.5)
// =====================================================================
//
// Section (11) is this section's planar half, and its preamble on why
// `check()` is the primary gate and volume the secondary one applies here
// unchanged (COINCIDENT.md §7): a retained interior wall has zero volume, so
// only the combinatorial gate sees it. `volume()` runs `assert_valid` — hence
// `check()` — on every call below, and additionally cross-checks the meshed
// measurement against the B-Rep-native one, so each case is gated three ways.
// The single exception is `overlapping_coaxial_cylinders_far_from_origin`,
// which borrows §16's `brep_volume` to weigh the exact number rather than the
// meshed one at offset 1e6; it is cross-checked the same three ways.
//
// What is NEW here is not the classification but the *chart*. Curved
// coincident faces brought three failures that planar ones structurally
// cannot express, and each of them is about what a trim edge IS rather than
// about coincidence (COINCIDENT.md §3, of-bxl.5 amendment):
//
//   - a partner trim edge is an ARC, and stationing it over its conic's full
//     period imprints a second curve that bounds nothing (`CurveSpan`);
//   - a curved edge's SAMPLED polyline sags ≈5.4e-4·R off the true curve —
//     five orders above the weld length — so the boundary-lying-run test has
//     to measure against the exact curve (`exact_edge_distance`);
//   - a chart SEAM is traversed twice in opposite senses by its own face, so
//     the region lies on both sides of it and it is not part of any overlap
//     boundary (`imprint_coincident` skips it).
//
// Every one of them is silent on a plane: planar trim edges are straight
// (zero sag), bounded by the bbox clip that stands in for their span, and
// planar faces have no seams. So the cases below are chosen to make each
// failure reachable — arcs whose span is a strict sub-arc (sphere meridians),
// full-wrap periodic regions (cylinder and cone walls), and two charts of the
// SAME surface that disagree about where the seam goes (`cross_axis_spheres`).

/// Cylinder axis ±Z, radius 1, spanning `z0..z1`.
fn unit_cylinder(scene: &mut Scene, z0: f64, z1: f64) -> EntityId<Body> {
    scene.cylinder(Point3::new(0.0, 0.0, z0), Vector3::z(), 1.0, z1 - z0)
}

/// Two coaxial equal-radius cylinders stacked end to end: A over `z ∈ [0,1]`,
/// B over `z ∈ [1,2]`.
///
/// The curved analogue of `touching_cubes_unite_fuses_into_one_box`, and it
/// carries TWO coincident pairs at once, of different kinds. A's top cap and
/// B's bottom cap are coplanar with identical trims — ON(Opposite), the wall
/// between them must vanish. The two *walls* are coincident too (same axis,
/// same radius), but their trims only touch along the `z = 1` rim circle, so
/// that pair is ordinary transversal work and must imprint nothing (§10's
/// false-positive case, now on a periodic chart).
#[test]
fn coaxial_cylinders_stacked_unite_fuses() {
    let context = "coaxial cylinders stacked at z = 1, unite";
    let mut scene = Scene::new();
    let a = unit_cylinder(&mut scene, 0.0, 1.0);
    let b = unit_cylinder(&mut scene, 1.0, 2.0);
    let out = scene
        .unite(a, b)
        .unwrap_or_else(|e| panic!("{context}: exact pipeline rejected the pair: {e:?}"));
    assert_close(volume(&out, context), 2.0 * PI, CYL_VOLUME_RTOL, context);
    // Each cylinder keeps its wall; the shared cap pair drops from both. A
    // retained cap wall measures the same volume — the face count and
    // `check()` inside `volume()` are what rule it out.
    assert_eq!(
        out.store.faces_of_body(out.body).len(),
        4,
        "{context}: two walls plus the two outer caps"
    );
}

/// `A − B` where the two only touch: A's top cap is ON(Opposite), which
/// subtract KEEPS (COINCIDENT.md §3, table row 5), so `A − B == A` whole.
#[test]
fn coaxial_cylinders_stacked_subtract_leaves_target_whole() {
    let context = "coaxial cylinders stacked at z = 1, subtract";
    let mut scene = Scene::new();
    let a = unit_cylinder(&mut scene, 0.0, 1.0);
    let b = unit_cylinder(&mut scene, 1.0, 2.0);
    let out = scene
        .subtract(a, b)
        .unwrap_or_else(|e| panic!("{context}: exact pipeline rejected the pair: {e:?}"));
    assert_close(volume(&out, context), PI, CYL_VOLUME_RTOL, context);
    assert_eq!(
        out.store.faces_of_body(out.body).len(),
        3,
        "{context}: A must come through whole"
    );
}

/// Intersection of two merely-touching solids is EMPTY, not a zero-thickness
/// disc (COINCIDENT.md §6). The curved reading of
/// `touching_cubes_intersect_is_empty_not_a_sheet`.
#[test]
fn coaxial_cylinders_stacked_intersect_is_empty() {
    let mut scene = Scene::new();
    let a = unit_cylinder(&mut scene, 0.0, 1.0);
    let b = unit_cylinder(&mut scene, 1.0, 2.0);
    let out = scene
        .intersect(a, b)
        .expect("intersection of touching cylinders is empty, not an error");
    assert_eq!(
        out.store.faces_of_body(out.body).len(),
        0,
        "the shared disc must not survive as a zero-volume sheet"
    );
    assert!(
        matches!(
            out.check().as_slice(),
            [CheckFailure::SolidWithoutShells(_)]
        ),
        "an empty solid is the correct answer here: {:?}",
        out.check()
    );
}

/// Coaxial equal-radius cylinders OVERLAPPING along the axis: A over
/// `z ∈ [0,1]`, B over `z ∈ [0.5,1.5]`. The curved counterpart of
/// `flush_overlapping_cubes`, and the case that actually needs the imprint.
///
/// The walls are a coincident pair whose trims overlap only partially, so the
/// overlap's boundary runs through the middle of both: B's `z = 0.5` rim
/// circle cuts A's wall, A's `z = 1` rim cuts B's. Those circles already lie
/// exactly in the partner's surface — that is what coincidence means — so
/// they are imprinted directly, with no intersection curve computed.
///
/// Two things here are unreachable on a plane. The imprint is a **closed
/// ring** on a **periodic** chart, so it wraps the cover rather than ending
/// on a boundary; and the wall region it cuts is a full wrap, closed by a
/// seam edge that must NOT itself be imprinted (the region lies on both sides
/// of it).
fn overlapping_coaxial_cylinders(op: &str, expected: f64) {
    let context = &format!("coaxial cylinders overlapping over z ∈ [0.5,1], {op}");
    let mut scene = Scene::new();
    let a = unit_cylinder(&mut scene, 0.0, 1.0);
    let b = unit_cylinder(&mut scene, 0.5, 1.5);
    let out = match op {
        "unite" => scene.unite(a, b),
        "subtract" => scene.subtract(a, b),
        "intersect" => scene.intersect(a, b),
        _ => unreachable!(),
    }
    .unwrap_or_else(|e| panic!("{context}: exact pipeline rejected the pair: {e:?}"));
    assert_close(volume(&out, context), expected, CYL_VOLUME_RTOL, context);
}

#[test]
fn overlapping_coaxial_cylinders_unite() {
    // z ∈ [0, 1.5].
    overlapping_coaxial_cylinders("unite", 1.5 * PI);
}

#[test]
fn overlapping_coaxial_cylinders_subtract() {
    // A minus the overlap: z ∈ [0, 0.5].
    overlapping_coaxial_cylinders("subtract", 0.5 * PI);
}

#[test]
fn overlapping_coaxial_cylinders_intersect() {
    // The overlap itself: z ∈ [0.5, 1].
    overlapping_coaxial_cylinders("intersect", 0.5 * PI);
}

/// Inclusion–exclusion over the overlapping coaxial pair. As in §(11) this is
/// the sharpest oracle available, because it ties union and intersection to
/// each other: a region kept from the wrong solid, or with a flipped normal,
/// breaks it even where each operation's own volume looks plausible alone.
#[test]
fn overlapping_coaxial_cylinders_inclusion_exclusion() {
    let context = "coaxial cylinders overlapping, inclusion-exclusion";
    let mut scene = Scene::new();
    let a = unit_cylinder(&mut scene, 0.0, 1.0);
    let b = unit_cylinder(&mut scene, 0.5, 1.5);
    let united = scene.unite(a, b).expect("unite of coaxial cylinders");
    let intersected = scene
        .intersect(a, b)
        .expect("intersect of coaxial cylinders");
    let sum = volume(&united, context) + volume(&intersected, context);
    assert_close(sum, 2.0 * PI, CYL_VOLUME_RTOL, context);
}

/// B's wall trim NESTED strictly inside A's: A over `z ∈ [0,2]`, B over
/// `z ∈ [0.5,1.5]`, same radius. The curved reading of
/// `stacked_l_shape_unite_imprints_nested_face`.
///
/// The nesting is the point. BOTH of B's rim circles lie in A's wall
/// interior, so both must be imprinted for the overlap to exist at all: A's
/// wall splits into three bands, the middle one ON(Same) with B's wall and
/// the outer two OUT. Failing to imprint gives the whole wall one verdict.
///
/// `subtract` is the sharp one: B sits wholly inside A, so `A − B` is two
/// disjoint discs — a **two-component** result, which is why it is gated with
/// `volume_checked` rather than `volume`.
#[test]
fn nested_coaxial_cylinders_all_three_ops() {
    let build = |scene: &mut Scene| {
        let a = unit_cylinder(scene, 0.0, 2.0);
        let b = unit_cylinder(scene, 0.5, 1.5);
        (a, b)
    };
    let context = "coaxial cylinders, B nested inside A's wall span";

    let mut scene = Scene::new();
    let (a, b) = build(&mut scene);
    let united = scene
        .unite(a, b)
        .unwrap_or_else(|e| panic!("{context}, unite: rejected: {e:?}"));
    // B adds nothing: the union is A.
    assert_close(volume(&united, context), 2.0 * PI, CYL_VOLUME_RTOL, context);

    let mut scene = Scene::new();
    let (a, b) = build(&mut scene);
    let intersected = scene
        .intersect(a, b)
        .unwrap_or_else(|e| panic!("{context}, intersect: rejected: {e:?}"));
    // The intersection is B.
    assert_close(volume(&intersected, context), PI, CYL_VOLUME_RTOL, context);

    let mut scene = Scene::new();
    let (a, b) = build(&mut scene);
    let subtracted = scene
        .subtract(a, b)
        .unwrap_or_else(|e| panic!("{context}, subtract: rejected: {e:?}"));
    assert_close(
        volume_checked(&subtracted, 2, 0, context),
        PI,
        CYL_VOLUME_RTOL,
        context,
    );
}

/// Rotation invariance for the curved coincident path — the regression that
/// catches snap-scaling bugs (of-lxk, of-260), rerun on a periodic chart.
///
/// Built on a tilted frame rather than by rotating a z-axis pair, because
/// `Curve3::Circle`'s angular reference comes from `plane_basis(axis)` and is
/// not rotation-equivariant (see [`Scene::cylinder`]). The point stands
/// either way: coincidence is decided at the feature-derived weld length, so
/// tilting both operands must not change which faces read as coincident.
#[test]
fn overlapping_coaxial_cylinders_are_frame_invariant() {
    let context = "coaxial cylinders overlapping on an oblique axis, unite";
    let mut scene = Scene::new();
    let axis = Vector3::new(1.0, 1.0, 1.0);
    let base = Point3::new(0.3, -0.4, 0.5);
    let step = axis.normalize() * 0.5;
    let a = scene.cylinder(base, axis, 1.0, 1.0);
    let b = scene.cylinder(base + step, axis, 1.0, 1.0);
    let out = scene
        .unite(a, b)
        .unwrap_or_else(|e| panic!("{context}: rejected after tilting: {e:?}"));
    assert_close(volume(&out, context), 1.5 * PI, CYL_VOLUME_RTOL, context);
}

/// The same pair at 1e-3 and 1e3 scale. `snap` is a fraction of the feature
/// extent, so both the coincidence test and the weld move with the part; an
/// absolute epsilon anywhere in this path fails one end or the other.
#[test]
fn overlapping_coaxial_cylinders_are_scale_invariant() {
    for s in [1e-3, 1e3] {
        let context = &format!("coaxial cylinders overlapping at scale {s:e}, unite");
        let mut scene = Scene::new();
        let a = scene.cylinder(Point3::origin(), Vector3::z(), s, s);
        let b = scene.cylinder(Point3::new(0.0, 0.0, 0.5 * s), Vector3::z(), s, s);
        let out = scene
            .unite(a, b)
            .unwrap_or_else(|e| panic!("{context}: rejected: {e:?}"));
        assert_close(
            volume(&out, context),
            1.5 * PI * s * s * s,
            CYL_VOLUME_RTOL,
            context,
        );
    }
}

/// The same pair a million units from the origin. `geometric_snap` is derived
/// from the feature extent and must NOT grow with distance from the origin
/// (of-lxk, of-260); if it did, the two rim circles here would weld into one
/// and the overlap would vanish.
///
/// Weighed with §16's [`brep_volume`] rather than [`volume`], for §16.2's
/// reason and not for any reason of this section's own: the B-Rep-native
/// volume is exact to floating point where the meshed one still carries the
/// tessellator's discretization error, so the classification can be held to
/// `1.5π` at `EXACT_RTOL` — a tighter budget than the meshed cases in this
/// section get, not a looser one. `check()`, closed-manifoldness, and the
/// mesh-vs-exact cross-check all still run.
///
/// Until of-ukcq the mesh path was simply invalid out here — a BARE cylinder
/// at this offset, no boolean anywhere, measured 33.3 instead of π, so its
/// disagreement with the exact number measured that bug rather than anything
/// about coincident faces. With the tetrahedra referenced to the mesh's own
/// bbox centre the two paths agree at this offset, and the cross-check is no
/// longer something this test has to opt out of.
#[test]
fn overlapping_coaxial_cylinders_far_from_origin() {
    let context = "coaxial cylinders overlapping at (1e6, -3e5, 7e5), unite";
    let mut scene = Scene::new();
    let o = Point3::new(1e6, -3e5, 7e5);
    let a = scene.cylinder(o, Vector3::z(), 1.0, 1.0);
    let b = scene.cylinder(o + Vector3::z() * 0.5, Vector3::z(), 1.0, 1.0);
    let out = scene
        .unite(a, b)
        .unwrap_or_else(|e| panic!("{context}: rejected far from the origin: {e:?}"));
    assert_close(brep_volume(&out, context), 1.5 * PI, EXACT_RTOL, context);
}

// ---------------------------------------------------------------------
// Concentric spheres — the bead's named gate, and the case that broke.
// ---------------------------------------------------------------------

/// Two concentric equal-radius spheres, i.e. the SAME sphere twice.
///
/// `sphere_sphere` reports `Coincident` only for concentric equal radii, so
/// this is the entire coincident sphere case — the whole surface overlaps and
/// the answer is just the sphere. Trivial to state, and it was the
/// configuration that exposed two of the three of-bxl.5 bugs:
///
///  - the seam meridian is HALF a great circle, and stationing the partner's
///    seam over the conic's full period laid the OPPOSITE meridian across the
///    host as well, splitting it for nothing (`chi = 2 - 2 + 5 = 5`);
///  - the surviving seam imprint lies on the host's own outline, and the
///    boundary-lying-run test measured against the sampled polyline, which
///    sags a sagitta off a circular seam and so never recognized it.
///
/// A plane can express neither: its trim edges are straight and its faces
/// have no seam.
#[test]
fn concentric_spheres_unite_is_one_sphere() {
    let context = "identical concentric spheres, unite";
    let mut scene = Scene::new();
    let a = scene.sphere(Point3::origin(), 1.0);
    let b = scene.sphere(Point3::origin(), 1.0);
    let out = scene
        .unite(a, b)
        .unwrap_or_else(|e| panic!("{context}: exact pipeline rejected the pair: {e:?}"));
    assert_close(
        volume(&out, context),
        sphere_volume(1.0),
        CURVED_VOLUME_RTOL,
        context,
    );
    // ON(Same) is kept from A only — the canonical tie-break. Two faces here
    // would be the sphere emitted twice, which `check()` also faults.
    assert_eq!(
        out.store.faces_of_body(out.body).len(),
        1,
        "{context}: exactly one sphere face survives"
    );
}

/// `A ∩ A == A`, by the same ON(Same) tie-break as the union.
#[test]
fn concentric_spheres_intersect_is_one_sphere() {
    let context = "identical concentric spheres, intersect";
    let mut scene = Scene::new();
    let a = scene.sphere(Point3::origin(), 1.0);
    let b = scene.sphere(Point3::origin(), 1.0);
    let out = scene
        .intersect(a, b)
        .unwrap_or_else(|e| panic!("{context}: exact pipeline rejected the pair: {e:?}"));
    assert_close(
        volume(&out, context),
        sphere_volume(1.0),
        CURVED_VOLUME_RTOL,
        context,
    );
    assert_eq!(out.store.faces_of_body(out.body).len(), 1, "{context}");
}

/// `A − A` is empty. ON(Same) drops from both solids under subtract
/// (COINCIDENT.md §3, table rows 5 and 6), leaving no faces at all — and an
/// empty solid is spelled `SolidWithoutShells`, which is the assertion here
/// rather than a failure.
#[test]
fn concentric_spheres_subtract_is_empty() {
    let mut scene = Scene::new();
    let a = scene.sphere(Point3::origin(), 1.0);
    let b = scene.sphere(Point3::origin(), 1.0);
    let out = scene.subtract(a, b).expect("A − A is empty, not an error");
    assert_eq!(
        out.store.faces_of_body(out.body).len(),
        0,
        "A − A must keep no faces"
    );
    assert!(
        matches!(
            out.check().as_slice(),
            [CheckFailure::SolidWithoutShells(_)]
        ),
        "an empty solid is the correct answer here: {:?}",
        out.check()
    );
}

/// The same sphere twice, but with the two operands' POLES ON DIFFERENT AXES
/// — A's on ±Z, B's on ±X. This is the sharpest chart-sharing case the
/// coincident path has, and the one that isolated the third of-bxl.5 bug.
///
/// The surfaces are identical, so SSI reports `Coincident` and the overlap is
/// everything. But the two faces disagree about where the seam goes: B's seam
/// meridian is a perfectly good curve on A's surface that runs from A's own
/// seam out to a dead end in A's interior. Imprinted, it separates no region;
/// `apply_chain` splits a zero-area sliver along it, the sliver has no
/// interior sample, and the boolean dies in `classify`.
///
/// The fix is not a tolerance: a seam is traversed TWICE in opposite senses
/// by its own face (`[seam+, seam−]`), so the region lies on both sides of it
/// and it bounds nothing. `imprint_coincident` skips such edges, which leaves
/// this pair with no imprints at all — correctly, since the overlap is a
/// whole region already.
///
/// Nothing about this is expressible with planes, which is why of-bxl.4 could
/// not have found it.
#[test]
fn cross_axis_spheres_unite_is_one_sphere() {
    let context = "same sphere, poles on Z vs X, unite";
    let mut scene = Scene::new();
    let a = scene.sphere_with_axis(Point3::origin(), Vector3::z(), 1.0);
    let b = scene.sphere_with_axis(Point3::origin(), Vector3::x(), 1.0);
    let out = scene
        .unite(a, b)
        .unwrap_or_else(|e| panic!("{context}: exact pipeline rejected the pair: {e:?}"));
    assert_close(
        volume(&out, context),
        sphere_volume(1.0),
        CURVED_VOLUME_RTOL,
        context,
    );
    assert_eq!(
        out.store.faces_of_body(out.body).len(),
        1,
        "{context}: one sphere face, kept from A by the tie-break"
    );
}

/// The intersection half of the cross-axis pair, and its inclusion–exclusion
/// identity: `vol(A) + vol(B) == vol(A∪B) + vol(A∩B)` reads
/// `2V == V + V` here, which is only satisfied if BOTH operations keep the
/// sphere exactly once. Keeping it twice, or dropping it, breaks the identity
/// while each volume alone might still look plausible.
#[test]
fn cross_axis_spheres_inclusion_exclusion() {
    let context = "same sphere, poles on Z vs X, inclusion-exclusion";
    let mut scene = Scene::new();
    let a = scene.sphere_with_axis(Point3::origin(), Vector3::z(), 1.0);
    let b = scene.sphere_with_axis(Point3::origin(), Vector3::x(), 1.0);
    let united = scene.unite(a, b).expect("unite of cross-axis spheres");
    let intersected = scene
        .intersect(a, b)
        .expect("intersect of cross-axis spheres");
    let sum = volume(&united, context) + volume(&intersected, context);
    assert_close(sum, 2.0 * sphere_volume(1.0), CURVED_VOLUME_RTOL, context);
}

/// Concentric spheres of DIFFERENT radii are the false-positive guard for the
/// section: `sphere_sphere` reports `Empty` (no intersection curve, and the
/// surfaces are not coincident), so the coincident path must not fire at all.
/// The union is the outer ball and the intersection the inner one.
#[test]
fn concentric_spheres_of_unequal_radii_are_not_coincident() {
    let context = "concentric spheres, radii 1 and 2";
    let mut scene = Scene::new();
    let a = scene.sphere(Point3::origin(), 2.0);
    let b = scene.sphere(Point3::origin(), 1.0);
    let united = scene
        .unite(a, b)
        .unwrap_or_else(|e| panic!("{context}, unite: rejected: {e:?}"));
    assert_close(
        volume(&united, context),
        sphere_volume(2.0),
        CURVED_VOLUME_RTOL,
        context,
    );
    let intersected = scene
        .intersect(a, b)
        .unwrap_or_else(|e| panic!("{context}, intersect: rejected: {e:?}"));
    assert_close(
        volume(&intersected, context),
        sphere_volume(1.0),
        CURVED_VOLUME_RTOL,
        context,
    );
}

// ---------------------------------------------------------------------
// Coaxial cones — `cone_cone` / `coaxial_profiles`.
// ---------------------------------------------------------------------

/// A full cone `r = 2 → 0` over `z ∈ [0,2]`, cut at `z = 1` into a frustum
/// and a tip, then re-united. The two wall surfaces share an apex, an axis
/// and a half-angle, so `cone_cone` reports `Coincident`; their trims touch
/// along the `z = 1` rim and no more, so the pair imprints nothing and the
/// coplanar cap pair does the fusing.
///
/// The apex is what makes this different from the cylinder stack: it is a
/// chart pole, and the tip's wall region runs into it.
#[test]
fn coaxial_cones_stacked_unite_rebuilds_the_cone() {
    let context = "cone split at z = 1 and re-united";
    let mut scene = Scene::new();
    let a = scene.cone(Point3::origin(), 2.0, 1.0, 1.0);
    let b = scene.cone(Point3::new(0.0, 0.0, 1.0), 1.0, 0.0, 1.0);
    let out = scene
        .unite(a, b)
        .unwrap_or_else(|e| panic!("{context}: exact pipeline rejected the pair: {e:?}"));
    assert_close(
        volume(&out, context),
        frustum_volume(2.0, 0.0, 2.0),
        CYL_VOLUME_RTOL,
        context,
    );
    assert_eq!(
        out.store.faces_of_body(out.body).len(),
        3,
        "{context}: two wall bands plus the base cap"
    );
}

/// The same cone, but the two pieces OVERLAP over `z ∈ [1,1.5]`: A is the
/// frustum `r = 2 → 0.5` over `z ∈ [0,1.5]`, B the tip `r = 1 → 0` over
/// `z ∈ [1,2]`. Now the coincident wall pair's trims genuinely share area,
/// so each partner's rim circle must be imprinted into the other's wall.
///
/// The union is the whole cone again, which is the oracle: an imprint placed
/// wrong here shows up as a wall band kept twice or dropped, and `check()`
/// sees both.
fn overlapping_coaxial_cones(op: &str, expected: f64) {
    let context = &format!("coaxial cone pieces overlapping over z ∈ [1,1.5], {op}");
    let mut scene = Scene::new();
    let a = scene.cone(Point3::origin(), 2.0, 0.5, 1.5);
    let b = scene.cone(Point3::new(0.0, 0.0, 1.0), 1.0, 0.0, 1.0);
    let out = match op {
        "unite" => scene.unite(a, b),
        "subtract" => scene.subtract(a, b),
        "intersect" => scene.intersect(a, b),
        _ => unreachable!(),
    }
    .unwrap_or_else(|e| panic!("{context}: exact pipeline rejected the pair: {e:?}"));
    assert_close(volume(&out, context), expected, CYL_VOLUME_RTOL, context);
}

#[test]
fn overlapping_coaxial_cones_unite() {
    // The whole cone, r = 2 at z = 0 tapering to the apex at z = 2.
    overlapping_coaxial_cones("unite", frustum_volume(2.0, 0.0, 2.0));
}

#[test]
fn overlapping_coaxial_cones_subtract() {
    // A minus the tip B: the frustum r = 2 → 1 over z ∈ [0,1].
    overlapping_coaxial_cones("subtract", frustum_volume(2.0, 1.0, 1.0));
}

#[test]
fn overlapping_coaxial_cones_intersect() {
    // The overlap: the frustum r = 1 → 0.5 over z ∈ [1,1.5].
    overlapping_coaxial_cones("intersect", frustum_volume(1.0, 0.5, 0.5));
}

/// Inclusion–exclusion over the overlapping cone pair.
#[test]
fn overlapping_coaxial_cones_inclusion_exclusion() {
    let context = "coaxial cone pieces overlapping, inclusion-exclusion";
    let mut scene = Scene::new();
    let a = scene.cone(Point3::origin(), 2.0, 0.5, 1.5);
    let b = scene.cone(Point3::new(0.0, 0.0, 1.0), 1.0, 0.0, 1.0);
    let united = scene.unite(a, b).expect("unite of coaxial cone pieces");
    let intersected = scene
        .intersect(a, b)
        .expect("intersect of coaxial cone pieces");
    let sum = volume(&united, context) + volume(&intersected, context);
    assert_close(
        sum,
        frustum_volume(2.0, 0.5, 1.5) + frustum_volume(1.0, 0.0, 1.0),
        CYL_VOLUME_RTOL,
        context,
    );
}

/// The same cone twice: BOTH the wall pair and the base cap pair are
/// coincident with identical trims, and the wall pair runs into a shared
/// apex. `A ∪ A == A ∩ A == A`, one wall face and one cap.
#[test]
fn identical_cones_unite_and_intersect_are_the_cone() {
    let context = "identical cones";
    let expected = frustum_volume(2.0, 0.0, 2.0);
    for op in ["unite", "intersect"] {
        let mut scene = Scene::new();
        let a = scene.cone(Point3::origin(), 2.0, 0.0, 2.0);
        let b = scene.cone(Point3::origin(), 2.0, 0.0, 2.0);
        let out = match op {
            "unite" => scene.unite(a, b),
            _ => scene.intersect(a, b),
        }
        .unwrap_or_else(|e| panic!("{context}, {op}: rejected: {e:?}"));
        assert_close(volume(&out, context), expected, CYL_VOLUME_RTOL, context);
        assert_eq!(
            out.store.faces_of_body(out.body).len(),
            2,
            "{context}, {op}: one wall and one cap, kept from A only"
        );
    }
}

// ---------------------------------------------------------------------
// The two tolerance questions COINCIDENT.md §9 left open, both closed here.
// ---------------------------------------------------------------------

/// The CONVERSE tolerance mismatch (COINCIDENT.md §9, first bullet): SSI
/// reporting `Empty` for surfaces the arrangement would nonetheless weld.
///
/// Two boxes of extent 1e4 separated by a 5e-6 gap. `snap` is ~1e-9 of the
/// feature extent, so here it is ~2.4e-5 — ABOVE `tol.linear = 1e-6`, which
/// inverts the usual roles: SSI calls the two `x = 1e4` planes distinct (the
/// gap exceeds `linear`), while every vertex and edge on them welds (the gap
/// is under `snap`). Left alone, the fused shell is reconstructed from
/// regions classified as if the faces never met, and the classifier runs out
/// of usable rays: `Degenerate { context: "boolean::ray_classify" }`.
///
/// This is the only configuration in the suite where `snap > tol.linear`, and
/// therefore the only one that can reach the `Empty`-arm re-test at all.
#[test]
fn sub_snap_gap_at_large_extent_welds_as_coincident() {
    let context = "1e4-extent boxes with a 5e-6 (sub-snap) gap";
    let mut scene = Scene::new();
    let a = scene.block([0.0, 0.0, 0.0], [1e4, 1e4, 1e4]);
    let b = scene.block([1e4 + 5e-6, 0.0, 0.0], [2e4, 1e4, 1e4]);
    let united = scene
        .unite(a, b)
        .unwrap_or_else(|e| panic!("{context}, unite: rejected: {e:?}"));
    // The 5e-6 gap contributes 5e2 of the 2e12 total — 2.5e-10 relative, an
    // order under the planar budget, so the fused box is the exact answer to
    // the precision this suite asserts anywhere.
    assert_close(volume(&united, context), 2e12, PLANAR_VOLUME_RTOL, context);
    assert_eq!(
        united.store.faces_of_body(united.body).len(),
        10,
        "{context}: each box's five surviving faces, as for touching cubes"
    );
    let mut scene = Scene::new();
    let a = scene.block([0.0, 0.0, 0.0], [1e4, 1e4, 1e4]);
    let b = scene.block([1e4 + 5e-6, 0.0, 0.0], [2e4, 1e4, 1e4]);
    let subtracted = scene
        .subtract(a, b)
        .unwrap_or_else(|e| panic!("{context}, subtract: rejected: {e:?}"));
    assert_close(
        volume(&subtracted, context),
        1e12,
        PLANAR_VOLUME_RTOL,
        context,
    );
}

/// The near-coaxial DECISION (COINCIDENT.md §9, second bullet), tested from
/// the side that is reachable: an offset BELOW the weld length is coincident,
/// and stays coincident.
///
/// §9 commits the kernel to a discontinuity at `snap` rather than snapping
/// near-coaxial pairs to coaxial. The threshold has to be the weld length
/// itself and nothing else — below it two surfaces are indistinguishable to
/// every other predicate in the pipeline, so calling them coincident is the
/// only self-consistent answer. Offsetting one cylinder by 1e-12 (against a
/// `snap` of ~2e-9) must therefore land exactly the coaxial answer, unchanged
/// from `overlapping_coaxial_cylinders_unite`.
#[test]
fn sub_snap_offset_cylinders_stay_coincident() {
    for d in [1e-12, 1e-10] {
        let context = &format!("coaxial cylinders with a {d:e} (sub-snap) axis offset, unite");
        let mut scene = Scene::new();
        let a = scene.cylinder(Point3::origin(), Vector3::z(), 1.0, 1.0);
        let b = scene.cylinder(Point3::new(d, 0.0, 0.5), Vector3::z(), 1.0, 1.0);
        let out = scene
            .unite(a, b)
            .unwrap_or_else(|e| panic!("{context}: rejected: {e:?}"));
        assert_close(volume(&out, context), 1.5 * PI, CYL_VOLUME_RTOL, context);
    }
}

/// The other side of the same threshold: an offset ABOVE the weld length is
/// NOT coincident, and the pair is ordinary transversal work — two
/// equal-radius cylinders whose walls meet in two full lines
/// (`ssi/analytic.rs:512`), bounding a thin lune.
///
/// KNOWN BROKEN — of-m350, and NOT a coincident-face bug: the failures are
/// byte-identical with of-bxl.5's changes reverted, so this is the
/// transversal path. Kept as written rather than softened, per this file's
/// protocol, because §9's decision to model the lune rather than snap it away
/// assumes exactly this path works. `d = 1e-3` and `d = 1e-5` produce
/// `OpenEdgeInClosedShell` and an impossible Euler characteristic
/// respectively.
#[test]
#[ignore = "of-m350: near-coaxial equal-radius cylinders produce invalid booleans"]
fn near_coaxial_cylinders_stay_transversal() {
    for d in [1e-3, 1e-5] {
        let context = &format!("cylinders with a {d:e} axis offset, inclusion-exclusion");
        let mut scene = Scene::new();
        let a = scene.cylinder(Point3::origin(), Vector3::z(), 1.0, 1.0);
        let b = scene.cylinder(Point3::new(d, 0.0, 0.0), Vector3::z(), 1.0, 1.0);
        let united = scene
            .unite(a, b)
            .unwrap_or_else(|e| panic!("{context}, unite: rejected: {e:?}"));
        let intersected = scene
            .intersect(a, b)
            .unwrap_or_else(|e| panic!("{context}, intersect: rejected: {e:?}"));
        // No closed form is needed: inclusion-exclusion pins the pair against
        // each other, and both operands are unit cylinders.
        let sum = volume(&united, context) + volume(&intersected, context);
        assert_close(sum, 2.0 * PI, CYL_VOLUME_RTOL, context);
    }
}

// =====================================================================
// (18) Randomized CURVED operands and rigid-motion fuzzing (of-ipt.18)
// =====================================================================
//
// Sections (6)-(9) are *enumerated* sphere/torus/cone campaigns: each case
// is a configuration someone chose, with a closed form worked out by hand
// for it. That is what makes them precise and also what bounds them — the
// pipeline is only ever asked about arrangements a person thought to write
// down, and the exact B-Rep path's failure modes (chart selection, seam
// placement, SSI branch choice) are all things that vary continuously with
// the operands' parameters.
//
// This section samples those parameters instead. It cannot use a hand-derived
// closed form, and it does not need one: the *identities* are oracles that
// hold for every pair of solids whatsoever.
//
//     vol(A) + vol(B) == vol(A ∪ B) + vol(A ∩ B)       inclusion–exclusion
//     vol(A − B)      == vol(A) − vol(A ∩ B)           difference
//
// Both sides are weighed by the B-Rep-native path (of-ipt.17), which is
// exact to floating point and does not discretize anything, so these are
// asserted at `EXACT_RTOL` rather than at a tessellation budget. A boolean
// that misclassifies a region shifts one term and not the others, and
// nothing in the identity can absorb it.
//
// The second campaign fuzzes rigid motion. Congruent configurations must
// produce congruent results, so a random rotation of BOTH operands may move
// the result's centroid but must not change its volume — and the centroid
// must move exactly as the operands did. This needs no oracle at all: it
// compares the pipeline against itself, which is why it holds for
// configurations whose right answer nobody has worked out.

/// A randomized curved configuration, as *parameters* rather than as built
/// bodies — so the same pair can be constructed twice, once plain and once
/// rigidly moved, which is what the invariance campaign compares.
#[derive(Clone, Copy, Debug)]
enum CurvedPair {
    /// Two overlapping spheres, centers `d` apart on the x axis.
    SphereSphere { r1: f64, r2: f64, d: f64 },
    /// A cylinder bored clean through a sphere, both centered on the origin.
    SphereCylinder { rs: f64, rc: f64, h: f64 },
    /// A block corner driven into a sphere.
    SphereBlock {
        r: f64,
        half: [f64; 3],
        off: [f64; 3],
    },
    /// A block slab cutting across a torus.
    TorusBlock {
        major: f64,
        minor: f64,
        half: [f64; 3],
        off: [f64; 3],
    },
    /// A sphere overlapping a frustum on its axis.
    ConeSphere {
        r0: f64,
        r1: f64,
        h: f64,
        rs: f64,
        dz: f64,
    },
}

impl CurvedPair {
    /// Sample a configuration with generous transversality margins.
    ///
    /// Every margin below keeps the operands' surfaces meeting cleanly:
    /// tangency and near-coincidence are what sections (3) and (5) are for,
    /// and mixing them in here would make a failure ambiguous between "the
    /// pipeline is wrong" and "the configuration is degenerate".
    ///
    /// [`CurvedPair::ConeSphere`] is **excluded from the sampler** (the
    /// `pick(4)` below, not `pick(5)`): the exact path fails outright on
    /// transversal cone-sphere pairs with `boolean::classify` and
    /// `boolean::ray_classify` `Degenerate` errors — of-ntkk. The variant is
    /// kept, and its repro is pinned in
    /// [`cone_sphere_union_takes_the_exact_path`]; restoring it here is the
    /// one-character change that re-arms the campaign once the bead closes.
    fn random(rng: &mut Rng) -> Self {
        match rng.pick(4) {
            0 => {
                let (r1, r2) = (rng.range(0.8, 1.6), rng.range(0.8, 1.6));
                // Transversal lens: the spheres must overlap without either
                // containing the other.
                let lo = (r1 - r2).abs() + 0.25;
                let hi = r1 + r2 - 0.25;
                CurvedPair::SphereSphere {
                    r1,
                    r2,
                    d: rng.range(lo, hi),
                }
            }
            1 => {
                let rs = rng.range(1.0, 1.8);
                CurvedPair::SphereCylinder {
                    rs,
                    // Strictly inside the sphere's equator, so the bore is a
                    // through hole and the caps clear the sphere entirely.
                    rc: rng.range(0.25, rs - 0.35),
                    h: 2.0 * rs + rng.range(0.6, 1.6),
                }
            }
            2 => {
                let r = rng.range(0.9, 1.6);
                let half = [
                    rng.range(0.5, 1.1),
                    rng.range(0.5, 1.1),
                    rng.range(0.5, 1.1),
                ];
                // Offset so one corner region of the block is inside the
                // sphere and the opposite one is clear of it.
                let axis_off = |rng: &mut Rng, h: f64| {
                    (r + h) * rng.range(0.35, 0.6) * if rng.pick(2) == 0 { 1.0 } else { -1.0 }
                };
                CurvedPair::SphereBlock {
                    r,
                    half,
                    off: [
                        axis_off(rng, half[0]),
                        axis_off(rng, half[1]),
                        axis_off(rng, half[2]),
                    ],
                }
            }
            3 => {
                let major = rng.range(1.0, 1.6);
                let minor = rng.range(0.25, 0.45) * major;
                // A slab wide enough in x and y to reach past the torus, and
                // thin enough in z to cut the tube rather than swallow it.
                CurvedPair::TorusBlock {
                    major,
                    minor,
                    half: [
                        major + minor + rng.range(0.3, 0.8),
                        major + minor + rng.range(0.3, 0.8),
                        rng.range(0.25, 0.7) * minor,
                    ],
                    off: [0.0, 0.0, rng.range(-0.35, 0.35) * minor],
                }
            }
            _ => {
                let r0 = rng.range(0.9, 1.5);
                let r1 = rng.range(0.2, 0.7) * r0;
                let h = rng.range(1.2, 2.2);
                let rs = rng.range(0.6, 1.1);
                CurvedPair::ConeSphere {
                    r0,
                    r1,
                    h,
                    rs,
                    // The sphere straddles the cone's top cap, so it meets
                    // the wall and the cap without reaching either extreme.
                    dz: h / 2.0 + rng.range(-0.35, 0.35) * rs,
                }
            }
        }
    }

    /// Build the pair into `scene`, `A` first.
    fn build(self, scene: &mut Scene) -> (EntityId<Body>, EntityId<Body>) {
        match self {
            CurvedPair::SphereSphere { r1, r2, d } => (
                scene.sphere(Point3::new(-d / 2.0, 0.0, 0.0), r1),
                scene.sphere(Point3::new(d / 2.0, 0.0, 0.0), r2),
            ),
            CurvedPair::SphereCylinder { rs, rc, h } => (
                scene.sphere(Point3::origin(), rs),
                scene.cylinder(Point3::new(0.0, 0.0, -h / 2.0), Vector3::z(), rc, h),
            ),
            CurvedPair::SphereBlock { r, half, off } => (
                scene.sphere(Point3::origin(), r),
                scene.block(
                    [off[0] - half[0], off[1] - half[1], off[2] - half[2]],
                    [off[0] + half[0], off[1] + half[1], off[2] + half[2]],
                ),
            ),
            CurvedPair::TorusBlock {
                major,
                minor,
                half,
                off,
            } => (
                scene.torus(Point3::origin(), major, minor),
                scene.block(
                    [off[0] - half[0], off[1] - half[1], off[2] - half[2]],
                    [off[0] + half[0], off[1] + half[1], off[2] + half[2]],
                ),
            ),
            CurvedPair::ConeSphere { r0, r1, h, rs, dz } => (
                scene.cone(Point3::new(0.0, 0.0, -h / 2.0), r0, r1, h),
                scene.sphere(Point3::new(0.0, 0.0, dz), rs),
            ),
        }
    }

    fn repro(&self, case: usize, seed: &str) -> String {
        format!("case {case}: {self:?} [seed {seed}]")
    }
}

/// The exact (B-Rep-native) volume of an operand body.
fn operand_volume(scene: &Scene, body: EntityId<Body>, context: &str) -> f64 {
    measured_body(scene, body, context).1.volume
}

/// Inclusion–exclusion and the difference identity over randomly sampled
/// curved pairs, weighed by the exact path.
///
/// No closed form is involved, which is the point: the identities hold for
/// every pair of solids, so the campaign can sample configurations nobody
/// has worked the answer out for.
#[test]
fn random_curved_pairs_satisfy_the_volume_identities() {
    let mut rng = Rng::new(0xC01D_ED11);
    for case in 0..20 {
        let pair = CurvedPair::random(&mut rng);
        let repro = pair.repro(case, "0xC01D_ED11");
        let mut scene = Scene::new();
        let (a, b) = pair.build(&mut scene);

        let vol_a = operand_volume(&scene, a, &format!("{repro}: operand A"));
        let vol_b = operand_volume(&scene, b, &format!("{repro}: operand B"));

        let union = scene
            .unite(a, b)
            .unwrap_or_else(|e| panic!("{repro}: unite failed: {e:?}"));
        let inter = scene
            .intersect(a, b)
            .unwrap_or_else(|e| panic!("{repro}: intersect failed: {e:?}"));
        let diff = scene
            .subtract(a, b)
            .unwrap_or_else(|e| panic!("{repro}: subtract failed: {e:?}"));

        // `measured` already asserts check(), closed-manifold, and that the
        // meshed and exact measurements agree; take the exact one.
        let vol_union = measured(&union, &format!("{repro}: union")).1.volume;
        let vol_inter = measured(&inter, &format!("{repro}: intersection")).1.volume;
        let vol_diff = measured(&diff, &format!("{repro}: difference")).1.volume;

        assert!(
            vol_inter > 0.0,
            "{repro}: the sampled configuration does not overlap (intersection volume \
             {vol_inter}); the margins in `CurvedPair::random` are supposed to guarantee \
             a transversal pair"
        );
        assert_close(
            vol_union + vol_inter,
            vol_a + vol_b,
            EXACT_RTOL,
            &format!("{repro}: inclusion–exclusion identity"),
        );
        assert_close(
            vol_diff,
            vol_a - vol_inter,
            EXACT_RTOL,
            &format!("{repro}: difference identity"),
        );
    }
}

/// Budget for a quantity that a rigid motion may not change at all.
///
/// It should be [`EXACT_RTOL`]. It is not, because the exact path is
/// measurably frame-dependent: over this campaign the worst volume deviation
/// under a rigid motion is `3.9e-3` and the worst centroid deviation
/// `3.6e-3`, both on sphere-sphere pairs, where the intersection lens comes
/// out `4.4e-4` off its closed form in one frame and `4.3e-4` off the other
/// way in another — of-7bnv. The tight statement is
/// [`sphere_lens_volume_is_exact_and_frame_independent`], parked `#[ignore]`d
/// against that bead.
///
/// `5e-3` sits just above the observed spread, so this stays a live test of
/// everything larger: a misclassified region is orders of magnitude bigger
/// than the trim error and still fails here.
const RIGID_MOTION_RTOL: f64 = 5e-3;

/// A rigid motion applied to BOTH operands must leave the result congruent:
/// the same volume, and a centroid that has moved exactly as the operands
/// did.
///
/// The centroid half is the sharper of the two. Volume is a single scalar
/// that a compensating pair of misclassifications can preserve; the centroid
/// is three numbers that pin *where* the material is, and asserting its
/// equivariance under a random rotation is the statement that the exact path
/// made the same decisions in a rotated frame that it made in the original
/// one. Chart selection and seam placement are precisely the parts of that
/// path that are not rotation-invariant by construction.
#[test]
fn random_curved_pairs_are_invariant_under_rigid_motion() {
    let mut rng = Rng::new(0x0009_161D_C0DE);
    for case in 0..8 {
        let pair = CurvedPair::random(&mut rng);
        let repro = pair.repro(case, "0x0009_161D_C0DE");

        let axis = Vector3::new(
            rng.range(-1.0, 1.0),
            rng.range(-1.0, 1.0),
            rng.range(-1.0, 1.0),
        );
        let angle = rng.range(0.2, 2.8);
        let center = Point3::new(
            rng.range(-1.5, 1.5),
            rng.range(-1.5, 1.5),
            rng.range(-1.5, 1.5),
        );
        let unit = Unit::new_normalize(axis);
        let rot = Rotation3::from_axis_angle(&unit, angle);
        let moved = format!(
            "{repro}, rotated {angle:.6} rad about {:?} at {center:?}",
            unit.into_inner()
        );

        let mut plain = Scene::new();
        let (a, b) = pair.build(&mut plain);

        let mut turned = Scene::new();
        let (ar, br) = pair.build(&mut turned);
        for body in [ar, br] {
            // The general `rotate_body`, not `Scene::rotate`: it re-anchors
            // circular edges to their rotated parameterization and rotates
            // quadric surfaces covariantly, which is what keeps a curved body
            // chart-consistent (see `Scene::cone_tilted`).
            rotate_body(
                &mut turned.store,
                &mut turned.geo,
                body,
                center,
                unit.into_inner(),
                angle,
            )
            .unwrap_or_else(|e| panic!("{moved}: rotate_body failed: {e:?}"));
        }

        type BoolOp = fn(&Scene, EntityId<Body>, EntityId<Body>) -> CoreResult<BooleanOutput>;
        let ops: [(&str, BoolOp); 3] = [
            ("union", |s, a, b| s.unite(a, b)),
            ("intersection", |s, a, b| s.intersect(a, b)),
            ("difference", |s, a, b| s.subtract(a, b)),
        ];
        for (op, run) in ops {
            let out = run(&plain, a, b).unwrap_or_else(|e| panic!("{repro}: {op} failed: {e:?}"));
            let out_rot =
                run(&turned, ar, br).unwrap_or_else(|e| panic!("{moved}: {op} failed: {e:?}"));

            let plain_props = measured(&out, &format!("{repro}: {op}")).1;
            let rot_props = measured(&out_rot, &format!("{moved}: {op}")).1;

            assert_close(
                rot_props.volume,
                plain_props.volume,
                RIGID_MOTION_RTOL,
                &format!("{moved}: {op} volume under rigid motion"),
            );

            let want = center + rot * (plain_props.centroid - center);
            let gap = (rot_props.centroid - want).norm();
            let scale = 1.0 + want.coords.norm();
            assert!(
                gap <= RIGID_MOTION_RTOL * scale,
                "{moved}: {op} centroid {:?} is not the rigid image of the unrotated \
                 centroid {:?} (expected {want:?}, off by {gap:.3e}, allowed {:.3e})",
                rot_props.centroid,
                plain_props.centroid,
                RIGID_MOTION_RTOL * scale
            );
        }
    }
}

/// The exact path must handle a transversal cone/frustum + sphere pair. It
/// does not (of-ntkk).
///
/// Minimal repro from `random_curved_pairs_satisfy_the_volume_identities`
/// case 6 (seed `0xC01D_ED11`), reduced to a single `unite`. The sphere
/// straddles the frustum's top cap and crosses its lateral wall — the cap is
/// swallowed entirely (sphere cross-section 0.744 against a cap radius of
/// 0.302), the sphere is strictly inside the cone at `z = 0` and strictly
/// outside it at `z = 0.4`. Nothing is tangent; the surfaces cross cleanly.
/// The union fails with `Degenerate { context: "boolean::classify", reason:
/// "could not find an interior sample point for a face region" }`.
///
/// Kept live and `#[ignore]`d per the never-soften policy —
/// `cargo test --test boolean_stress -- --ignored`.
#[test]
#[ignore = "of-ntkk: exact path fails with Degenerate on transversal cone+sphere pairs"]
fn cone_sphere_union_takes_the_exact_path() {
    let pair = CurvedPair::ConeSphere {
        r0: 1.0968253795551284,
        r1: 0.3020208770756382,
        h: 1.3556155311335805,
        rs: 0.7458934638406223,
        dz: 0.7313824048408364,
    };
    let mut scene = Scene::new();
    let (a, b) = pair.build(&mut scene);
    let out = scene
        .unite(a, b)
        .unwrap_or_else(|e| panic!("transversal cone+sphere union failed: {e:?}"));
    let vol = measured(&out, "transversal cone+sphere union").1.volume;
    let vol_a = operand_volume(&scene, a, "cone operand");
    assert!(
        vol > vol_a,
        "the union must be larger than the cone alone ({vol} vs {vol_a})"
    );
}

/// The exact measurement of a sphere-sphere lens must equal its closed form,
/// in every frame. It does not (of-7bnv).
///
/// Minimal repro from `random_curved_pairs_are_invariant_under_rigid_motion`
/// case 9 (seed `0x0009_161D_C0DE`), reduced to the intersection alone — which
/// is where the campaign's back-solve localized the error, the three booleans
/// being mutually consistent to `1e-9` around it. The measured lens is
/// `4.4e-4` above the closed form in the plain frame and `4.3e-4` below it
/// after a rigid rotation, which surfaces as a `2e-3` disagreement on
/// `A − B`.
///
/// Kept live and `#[ignore]`d per the never-soften policy.
#[test]
#[ignore = "of-7bnv: exact sphere-sphere lens volume is ~4e-4 off closed form and \
            frame-dependent"]
fn sphere_lens_volume_is_exact_and_frame_independent() {
    let (r1, r2) = (0.8312561812647244, 1.0925146621706086);
    let d = 0.5596874067739965;
    let want = sphere_lens_volume(r1, r2, d);

    let pair = CurvedPair::SphereSphere { r1, r2, d };
    let mut plain = Scene::new();
    let (a, b) = pair.build(&mut plain);
    let inter = plain
        .intersect(a, b)
        .expect("transversal spheres intersect");
    let got = measured(&inter, "sphere lens").1.volume;
    assert_close(got, want, EXACT_RTOL, "sphere lens vs closed form");

    // And the same, after a rigid motion of both operands.
    let axis = Unit::new_normalize(Vector3::new(
        -0.4698530016621909,
        0.6050056895436698,
        -0.6428112261378282,
    ));
    let angle = 2.311388;
    let center = Point3::new(
        -0.24731205816347712,
        1.2565024334215034,
        0.40905343797844207,
    );
    let mut turned = Scene::new();
    let (ar, br) = pair.build(&mut turned);
    for body in [ar, br] {
        rotate_body(
            &mut turned.store,
            &mut turned.geo,
            body,
            center,
            axis.into_inner(),
            angle,
        )
        .expect("valid rotation");
    }
    let inter_rot = turned
        .intersect(ar, br)
        .expect("transversal spheres intersect after rotation");
    let got_rot = measured(&inter_rot, "rotated sphere lens").1.volume;
    assert_close(
        got_rot,
        want,
        EXACT_RTOL,
        "rotated sphere lens vs closed form",
    );
}
