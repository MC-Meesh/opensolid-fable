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
//! One case is `#[ignore]`d, naming its open blocker: of-9ia, where the
//! marched non-coaxial cone-cone imprint is hosted as an open chain
//! ending in a face interior — a reconstruction gap, not an SSI gap
//! (`skew_frustums_inclusion_exclusion`). The coaxial cone-cone pair goes
//! through the analytic SSI and runs live
//! (`opposed_cones_intersection`, `coaxial_frustums_union_identity`).
//! The no-panic guard `no_panics_on_cone_configurations` stays live across
//! the promotion — it accepts both a valid exact solid and the structured
//! `NotImplemented` F-Rep fallback.

use nalgebra::{Rotation3, Unit};
use opensolid_brep::boolean::{intersect, subtract, unite};
use opensolid_brep::curve::plane_basis;
use opensolid_brep::{
    Body, BodyType, BooleanOutput, CheckFailure, Curve3, FaceSense, FinSense, GeometryStore,
    LoopType, SYSTEM_RESOLUTION, ShellOrientation, Surface3, TopologyStore, primitives,
    rotate_body, translate_body,
};
use opensolid_core::EntityId;
use opensolid_core::error::{CoreError, CoreResult};
use opensolid_core::mesh::TriangleMesh;
use opensolid_core::tolerance::ToleranceContext;
use opensolid_core::types::{BoundingBox3, Point3, Vector3};
use opensolid_kernel::{MeshOptions, MeshSdf, mass_properties, mesh_sdf_indexed};
use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI};

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
// here is live except `skew_frustums_inclusion_exclusion` (of-9ia).
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
#[test]
#[ignore = "of-9ia: the non-coaxial cone-cone SSI marches correctly (of-dtj.4, \
            unit test marched_bounded_cone_cone_offset_axes), but hosting the \
            marched imprint on the two curved cone faces yields an open chain \
            that ends in the face interior (boolean::imprint Degenerate, \
            'an imprint chain ends in a face interior') — a reconstruction \
            gap, not an SSI gap"]
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
/// fallback (e.g. non-coaxial cone–cone, of-9ia).
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
/// - `unite` of this pair is legitimately NON-MANIFOLD (the cubes stay two
///   shells joined at the shared edge's two endpoints, which `check`
///   reports as `VertexSharedBetweenShells`). That it returns `Ok` with an
///   unusable body rather than rejecting is of-n5g — the edge-contact
///   degeneracy, not this gate's business.
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
