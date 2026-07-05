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
//! - of-ny6: block∩block under a generic-axis rotation tessellates
//!   non-manifold (one edge shared by four triangles) despite clean
//!   check() and correct hexahedron topology.
//!
//! Re-verified after the of-k3u seam-refinement fix (of-ipt.12,
//! 2026-07-05): 15° tilt is still a silent no-op; 30°/45° now accept the
//! imprint but fail check() (OpenEdgeInClosedShell) like of-ipt.6; the
//! 25° skew case no longer errors — it builds correct topology (genus 1,
//! clean check) but tessellates non-manifold. of-ipt.6/8/9 and of-ny6
//! are unchanged.
//!
//! of-ipt.4 FIXED (2026-07-05): full-wrap curved-chart bands now refine
//! wide uv chords on-surface during tessellation; the block×cylinder
//! through-hole cases (all scales), the near-tangent wall case, and the
//! block−cylinder round trip are un-ignored and pass.
//!
//! of-ipt.7 FIXED (2026-07-05): the 25° skew case tessellates closed-
//! manifold after the of-ipt.4 curved-chart refinement and the of-299
//! hole-bridge validation landed; volume matches the transversal
//! prediction. Un-ignored.
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
use opensolid_brep::curve::plane_basis;
use opensolid_brep::{
    Body, BodyType, BooleanOutput, Curve3, FaceSense, FinSense, GeometryStore, LoopType,
    SYSTEM_RESOLUTION, ShellOrientation, Surface3, TopologyStore, primitives, translate_body,
};
use opensolid_core::EntityId;
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
// Scene: one TopologyStore + GeometryStore pair per test configuration.
// The boolean entry points consume two bodies living in a shared store
// pair, so every operand of one boolean call must be built here.
// ---------------------------------------------------------------------

struct Scene {
    store: TopologyStore,
    geo: GeometryStore,
}

impl Scene {
    fn new() -> Self {
        Scene {
            store: TopologyStore::new(),
            geo: GeometryStore::new(),
        }
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
        unite(&self.store, &self.geo, a, b, &tol())
    }

    fn subtract(&self, a: EntityId<Body>, b: EntityId<Body>) -> CoreResult<BooleanOutput> {
        subtract(&self.store, &self.geo, a, b, &tol())
    }

    fn intersect(&self, a: EntityId<Body>, b: EntityId<Body>) -> CoreResult<BooleanOutput> {
        intersect(&self.store, &self.geo, a, b, &tol())
    }
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
#[ignore = "of-ipt.8: quarter-notch removes 0.197 instead of 0.251 despite valid topology"]
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
#[ignore = "of-ipt.9: tessellate() emits sliver triangles that MeshSdf::new rejects"]
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
