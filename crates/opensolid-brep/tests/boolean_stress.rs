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
//! Bugs filed from this suite (first run, 2026-07-04):
//! - of-ipt.4: block×cylinder booleans silently wrong — hole volume off by
//!   ~12×, bottom-face hole never cut, cylinder band ~30% tessellated.
//! - of-ipt.5: ≥15° tilted cylinder tool: subtract silently returns A
//!   unchanged (imprints dropped; SSI verified correct).
//! - of-ipt.6: 0.5°–5° tilted cylinder: output fails check() and
//!   tessellates non-manifold.
//! - of-ipt.7: 25° diagonal tilt: Degenerate "interior imprint ring lies
//!   in no region of its host face".
//! - of-ipt.8: quarter-notch through a vertical edge: volume wrong
//!   (removed 0.197 vs 0.251) despite valid topology.
//! - of-ipt.9: tessellate() emits sliver triangles; MeshSdf::new rejects
//!   every boolean output tried (even pure block∪block).
//!
//! Invariants asserted throughout:
//! - `BooleanOutput::check()` reports no failures,
//! - `BooleanOutput::tessellate()` yields a closed manifold mesh,
//! - mesh volume (kernel `mass_properties`) matches analytic expectations,
//! - the inclusion–exclusion identity
//!   `vol(A) + vol(B) == vol(A∪B) + vol(A∩B)` holds,
//! - results are invariant under rigid rotation of both operands.

use nalgebra::{Rotation3, Unit};
use opensolid_brep::boolean::{intersect, subtract, unite};
use opensolid_brep::{AnalyticFace, AnalyticSolid, BooleanOutput, Curve3, SolidEdge, Surface3};
use opensolid_core::error::{CoreError, CoreResult};
use opensolid_core::mesh::TriangleMesh;
use opensolid_core::tolerance::ToleranceContext;
use opensolid_core::types::{BoundingBox3, Point3, Vector3};
use opensolid_kernel::{MeshOptions, MeshSdf, mass_properties, mesh_sdf_indexed};
use std::f64::consts::PI;

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

/// Volume of a valid boolean result via kernel mass properties.
fn volume(out: &BooleanOutput, context: &str) -> f64 {
    let mesh = assert_valid(out, context);
    mass_properties(&mesh)
        .unwrap_or_else(|e| panic!("{context}: mass_properties failed: {e}"))
        .volume
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
// Rigid rotation of plane/line solids (blocks). Circles are excluded on
// purpose: Curve3::Circle's angular reference comes from plane_basis(axis),
// which is not rotation-equivariant, so partial arcs would desync. Rotated
// cylinders are built directly with a rotated axis instead.
// ---------------------------------------------------------------------

fn rotate_solid(solid: &AnalyticSolid, rot: &Rotation3<f64>, center: &Point3) -> AnalyticSolid {
    let rp = |p: &Point3| center + rot * (p - center);
    let rv = |v: &Vector3| rot * v;
    let edges = solid
        .edges
        .iter()
        .map(|e| {
            let curve = match &e.curve {
                Curve3::Line { origin, dir } => {
                    Curve3::line(rp(origin), rv(dir)).expect("rotated unit dir")
                }
                other => panic!("rotate_solid only supports Line edges, got {other:?}"),
            };
            SolidEdge {
                curve,
                t0: e.t0,
                t1: e.t1,
                closed: e.closed,
            }
        })
        .collect();
    let faces = solid
        .faces
        .iter()
        .map(|f| {
            let surface = match &f.surface {
                Surface3::Plane { origin, normal } => {
                    Surface3::plane(rp(origin), rv(normal)).expect("rotated unit normal")
                }
                other => panic!("rotate_solid only supports Plane faces, got {other:?}"),
            };
            AnalyticFace {
                surface,
                outward_along_normal: f.outward_along_normal,
                loops: f.loops.clone(),
            }
        })
        .collect();
    AnalyticSolid { edges, faces }
}

fn block(min: [f64; 3], max: [f64; 3]) -> AnalyticSolid {
    AnalyticSolid::block(
        Point3::new(min[0], min[1], min[2]),
        Point3::new(max[0], max[1], max[2]),
    )
    .expect("valid block extents")
}

// =====================================================================
// (1) Rotated operands: block minus tilted cylinder
// =====================================================================

/// Subtract a cylinder tilted `angle_deg` from the z-axis (in the YZ
/// plane) from a 6×6×2 slab. The tool pierces top and bottom only, so the
/// removed material is an oblique cylinder of length `2 / cos θ`.
fn rotated_tool_through_hole(angle_deg: f64) {
    let context = format!("block minus cylinder tilted {angle_deg}°");
    let slab = block([0.0, 0.0, 0.0], [6.0, 6.0, 2.0]);
    let theta = angle_deg.to_radians();
    let axis = Vector3::new(0.0, theta.sin(), theta.cos());
    let center = Point3::new(3.0, 3.0, 1.0);
    let (radius, half_len) = (0.5, 4.0);
    let base = center - axis * half_len;
    let tool =
        AnalyticSolid::cylinder(base, axis, radius, 2.0 * half_len).expect("valid tilted tool");

    let out = subtract(&slab, &tool, &tol())
        .unwrap_or_else(|e| panic!("{context}: subtract failed: {e:?}"));
    let counts = out.store.euler_counts(out.body);
    assert_eq!(counts.genus, 1, "{context}: through hole must give genus 1");
    assert_eq!(out.shell_count(), 1, "{context}: single shell expected");
    let vol = volume(&out, &context);
    let expected = 6.0 * 6.0 * 2.0 - PI * radius * radius * (2.0 / theta.cos());
    assert_close(vol, expected, CYL_VOLUME_RTOL, &context);
}

#[test]
#[ignore = "of-ipt.6: 0.5°–5° tilt yields output failing check() and non-manifold tessellation"]
fn rotated_tool_through_hole_0_5_deg() {
    rotated_tool_through_hole(0.5);
}

#[test]
#[ignore = "of-ipt.6: 0.5°–5° tilt yields output failing check() and non-manifold tessellation"]
fn rotated_tool_through_hole_5_deg() {
    rotated_tool_through_hole(5.0);
}

#[test]
#[ignore = "of-ipt.5: ≥15° tilt drops all imprints — subtract silently returns A unchanged"]
fn rotated_tool_through_hole_15_deg() {
    rotated_tool_through_hole(15.0);
}

#[test]
#[ignore = "of-ipt.5: ≥15° tilt drops all imprints — subtract silently returns A unchanged"]
fn rotated_tool_through_hole_30_deg() {
    rotated_tool_through_hole(30.0);
}

#[test]
#[ignore = "of-ipt.5: ≥15° tilt drops all imprints — subtract silently returns A unchanged"]
fn rotated_tool_through_hole_45_deg() {
    rotated_tool_through_hole(45.0);
}

/// Same tilted-tool subtraction but tilted toward a block diagonal, so no
/// imprint aligns with any coordinate plane.
#[test]
#[ignore = "of-ipt.7: Degenerate 'interior imprint ring lies in no region of its host face'"]
fn rotated_tool_through_hole_skew_axis() {
    let context = "block minus cylinder tilted 25° toward XY diagonal";
    let slab = block([0.0, 0.0, 0.0], [6.0, 6.0, 2.0]);
    let theta = 25f64.to_radians();
    let lateral = Vector3::new(1.0, 1.0, 0.0).normalize();
    let axis = lateral * theta.sin() + Vector3::z() * theta.cos();
    let center = Point3::new(3.0, 3.0, 1.0);
    let (radius, half_len) = (0.5, 4.0);
    let tool = AnalyticSolid::cylinder(center - axis * half_len, axis, radius, 2.0 * half_len)
        .expect("valid skew tool");

    let out = subtract(&slab, &tool, &tol())
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
        let a = block([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]);
        let b0 = block([1.0, 1.0, 0.5], [3.5, 3.5, 1.5]);
        let rot =
            Rotation3::from_axis_angle(&Unit::new_normalize(Vector3::z()), angle_deg.to_radians());
        let b = rotate_solid(&b0, &rot, &Point3::new(2.25, 2.25, 1.0));

        let vol_a = 8.0;
        let vol_b = 2.5 * 2.5 * 1.0;
        let union =
            unite(&a, &b, &tol()).unwrap_or_else(|e| panic!("{context}: unite failed: {e:?}"));
        let inter = intersect(&a, &b, &tol())
            .unwrap_or_else(|e| panic!("{context}: intersect failed: {e:?}"));
        let diff = subtract(&a, &b, &tol())
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

    fn solids(&self) -> (AnalyticSolid, AnalyticSolid) {
        (
            block([0.0, 0.0, 0.0], self.a_max),
            block(self.b_min, self.b_max),
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
        let (a, b) = pair.solids();

        let union =
            unite(&a, &b, &tol()).unwrap_or_else(|e| panic!("{repro}: unite failed: {e:?}"));
        let inter = intersect(&a, &b, &tol())
            .unwrap_or_else(|e| panic!("{repro}: intersect failed: {e:?}"));
        let diff =
            subtract(&a, &b, &tol()).unwrap_or_else(|e| panic!("{repro}: subtract failed: {e:?}"));

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
        let (a, b) = pair.solids();

        let axis = Unit::new_normalize(Vector3::new(
            rng.range(-1.0, 1.0),
            rng.range(-1.0, 1.0),
            rng.range(-1.0, 1.0),
        ));
        let angle = rng.range(0.2, 1.3);
        let rot = Rotation3::from_axis_angle(&axis, angle);
        let center = Point3::new(1.0, 1.0, 1.0);
        let (ar, br) = (
            rotate_solid(&a, &rot, &center),
            rotate_solid(&b, &rot, &center),
        );

        let inter = intersect(&a, &b, &tol())
            .unwrap_or_else(|e| panic!("{repro}: intersect failed: {e:?}"));
        let inter_rot = intersect(&ar, &br, &tol()).unwrap_or_else(|e| {
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

// =====================================================================
// (3) Near-degenerate transversal cases
// =====================================================================

/// Through-hole whose wall clears a block side face by a shrinking gap.
/// Every gap here is above the default linear tolerance (1e-6), so the
/// configuration is still formally transversal and must succeed.
#[test]
#[ignore = "of-ipt.4: block×cylinder hole volume off by ~12× (bottom hole never cut, partial band)"]
fn wall_almost_tangent_to_side_face() {
    for gap in [1e-3, 1e-4, 1e-5] {
        let context = format!("cylinder wall {gap:.0e} away from face x=0");
        let radius = 0.5;
        let cube = block([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]);
        let tool = AnalyticSolid::cylinder(
            Point3::new(radius + gap, 1.0, -1.0),
            Vector3::z(),
            radius,
            4.0,
        )
        .expect("valid tool");
        let out = subtract(&cube, &tool, &tol())
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
        let a = block([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]);
        let b = block([thickness, -0.5, -0.5], [3.0, 2.5, 2.5]);
        let out = subtract(&a, &b, &tol())
            .unwrap_or_else(|e| panic!("{context}: subtract failed: {e:?}"));
        let vol = volume(&out, &context);
        assert_close(vol, thickness * 4.0, 1e-6, &context);
    }
}

/// Tool exiting through an edge region: the cylinder is centered on a
/// vertical block edge, so the edge is strictly inside the tool and the
/// subtraction carves a quarter-round notch spanning two side faces.
#[test]
#[ignore = "of-ipt.8: quarter-notch removes 0.197 instead of 0.251 despite valid topology"]
fn tool_swallows_vertical_edge() {
    let context = "quarter-notch: cylinder centered on the (2,2,z) edge";
    let radius = 0.4;
    let cube = block([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]);
    let tool = AnalyticSolid::cylinder(Point3::new(2.0, 2.0, -1.0), Vector3::z(), radius, 4.0)
        .expect("valid tool");
    let out = subtract(&cube, &tool, &tol())
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
    let cube = block([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]);
    // Push the axis out along the (1,1)/√2 diagonal so the closest
    // approach of the wall to the edge line is exactly `clearance`.
    let d = (radius + clearance) / 2f64.sqrt();
    let tool = AnalyticSolid::cylinder(
        Point3::new(2.0 + d, 2.0 + d, -1.0),
        Vector3::z(),
        radius,
        4.0,
    )
    .expect("valid tool");
    match subtract(&cube, &tool, &tol()) {
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
    let cube = block([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]);
    let tool = AnalyticSolid::cylinder(
        Point3::new(2.0 + d, 2.0 + d, -1.0),
        Vector3::z(),
        radius,
        4.0,
    )
    .expect("valid tool");
    let out = subtract(&cube, &tool, &tol())
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
#[ignore = "of-ipt.9: tessellate() emits sliver triangles that MeshSdf::new rejects (then of-ipt.4)"]
fn round_trip_block_minus_cylinder() {
    let slab = block([0.0, 0.0, 0.0], [4.0, 4.0, 2.0]);
    let tool = AnalyticSolid::cylinder(Point3::new(2.0, 2.0, -1.0), Vector3::z(), 1.0, 4.0)
        .expect("valid tool");
    let out = subtract(&slab, &tool, &tol()).expect("through-hole subtract");
    round_trip_volume(&out, "round-trip: block minus cylinder");
}

#[test]
#[ignore = "of-ipt.9: tessellate() emits sliver triangles that MeshSdf::new rejects"]
fn round_trip_union_of_blocks() {
    let a = block([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]);
    let b = block([1.0, 1.0, 1.0], [3.0, 3.0, 3.0]);
    let out = unite(&a, &b, &tol()).expect("corner-overlap union");
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
    let slab = block([0.0, 0.0, 0.0], [4.0 * s, 4.0 * s, 2.0 * s]);
    let tool = AnalyticSolid::cylinder(Point3::new(2.0 * s, 2.0 * s, -s), Vector3::z(), s, 4.0 * s)
        .expect("valid scaled tool");
    let out = subtract(&slab, &tool, &tol())
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
#[ignore = "of-ipt.4: block×cylinder hole volume off by ~12× (bottom hole never cut, partial band)"]
fn through_hole_at_scale_1() {
    scaled_through_hole(1.0);
}

#[test]
#[ignore = "of-ipt.4: block×cylinder hole volume off by ~12× (bottom hole never cut, partial band)"]
fn through_hole_at_scale_0_001() {
    scaled_through_hole(0.001);
}

#[test]
#[ignore = "of-ipt.4: block×cylinder hole volume off by ~12× (bottom hole never cut, partial band)"]
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
        let a = block(
            [0.0, 0.0, 0.0],
            [pair.a_max[0] * s, pair.a_max[1] * s, pair.a_max[2] * s],
        );
        let b = block(
            [pair.b_min[0] * s, pair.b_min[1] * s, pair.b_min[2] * s],
            [pair.b_max[0] * s, pair.b_max[1] * s, pair.b_max[2] * s],
        );
        let union =
            unite(&a, &b, &tol()).unwrap_or_else(|e| panic!("{repro}: unite failed: {e:?}"));
        let inter = intersect(&a, &b, &tol())
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
// Guard: error paths must be structured, never panics.
// =====================================================================

/// A grid of increasingly awkward but legal configurations must never
/// panic — every outcome is either a valid solid or a structured error.
#[test]
fn no_panics_on_awkward_configurations() {
    let cube = block([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]);
    let cases: Vec<(&str, CoreResult<BooleanOutput>)> = vec![
        (
            "tool corner exactly on face plane",
            unite(
                &cube,
                &block([2.0 - 1e-9, 0.5, 0.5], [3.0, 1.5, 1.5]),
                &tol(),
            ),
        ),
        (
            "tool face within system resolution of face",
            unite(
                &cube,
                &block([0.5, 0.5, 2.0 - 1e-11], [1.5, 1.5, 3.0]),
                &tol(),
            ),
        ),
        (
            "needle tool through the cube",
            subtract(
                &cube,
                &block([0.999, 0.999, -1.0], [1.001, 1.001, 3.0]),
                &tol(),
            ),
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
