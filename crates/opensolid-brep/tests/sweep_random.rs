//! Seeded randomized campaign for the B-Rep sweep constructors
//! ([`extrude`], [`revolve`]) — of-ipt.18.
//!
//! `sweep.rs` had unit tests and doctests over hand-picked profiles only:
//! a unit square, a triangle, a rectangle on the axis, a semicircle. Those
//! pin the shapes someone thought to write down. What they cannot reach is
//! the class of configuration a sweep actually fails on — a profile in a
//! generic (non-axis-aligned) plane, an oblique extrusion direction, an
//! arbitrary revolution axis — because every one of those is a different
//! *frame*, and a frame bug is invisible to a suite that only ever works in
//! the XY plane along +Z.
//!
//! # The oracles are exact, not approximate
//!
//! `SweptBody::tessellate` samples `resolution` points per profile segment
//! and `resolution` steps around a revolution. For the straight-segment
//! polygon profiles used here that discretization is *closed form*, not an
//! approximation to be absorbed by a loose tolerance:
//!
//! - **Extrusion.** Extra samples along a straight segment are collinear and
//!   the cap fan is exact for a convex profile, so the tessellation is the
//!   prism itself. Volume is exactly `area × |direction · normal|` — the
//!   perpendicular height, so an oblique (sheared) direction must give the
//!   same volume as the perpendicular one of equal normal component
//!   (Cavalieri). Centroid is exactly the profile centroid plus half the
//!   direction.
//! - **Revolution.** At every height the tessellated cross-section is the
//!   regular `n`-gon inscribed in the true circle of that radius, so every
//!   cross-sectional area is scaled by the *same* constant
//!   `k(n) = n·sin(2π/n) / 2π`. The meshed volume is therefore exactly
//!   `k(n)` times the Pappus volume `2π·x̄·A`. See [`ngon_area_ratio`].
//!
//! Both are asserted at `1e-9` relative — floating-point accumulation, not
//! discretization. A tolerance that only says "about right" cannot tell a
//! frame bug from a meshing artifact; these can.
//!
//! Protocol as `boolean_stress.rs`: deterministic seeded [`Rng`], a repro
//! string on every failure, failures become `bd` beads and the case is
//! `#[ignore]`d referencing the bead rather than softened.

use nalgebra::{Rotation3, Unit};
use opensolid_brep::sweep::{Profile, ProfileSegment, SweptBody, extrude, revolve};
use opensolid_core::error::CoreError;
use opensolid_core::mesh::TriangleMesh;
use opensolid_core::types::{Point3, Vector3};
use opensolid_kernel::{MassProperties, mass_properties};
use std::f64::consts::PI;

// ---------------------------------------------------------------------
// Deterministic RNG (splitmix64), identical to `boolean_stress.rs`.
// ---------------------------------------------------------------------

/// Campaign remix (of-5rim): `OPENSOLID_CAMPAIGN_SEED=<hex>` XORs every suite
/// seed so the same properties walk fresh configurations each run. Unset (CI,
/// plain `cargo test`), the suite is byte-for-byte deterministic. A campaign
/// failure reproduces with the same variable value plus the test name — the
/// campaign driver (`tools/campaign/`) records both in the bead it files.
fn campaign_seed() -> u64 {
    static MIX: std::sync::OnceLock<u64> = std::sync::OnceLock::new();
    *MIX.get_or_init(|| match std::env::var("OPENSOLID_CAMPAIGN_SEED") {
        Ok(raw) => {
            let hex = raw.trim();
            let hex = hex.strip_prefix("0x").unwrap_or(hex);
            u64::from_str_radix(&hex.replace('_', ""), 16)
                .unwrap_or_else(|_| panic!("OPENSOLID_CAMPAIGN_SEED must be hex, got {raw:?}"))
        }
        Err(_) => 0,
    })
}

struct Rng(u64);

impl Rng {
    fn new(seed: u64) -> Self {
        Rng(seed ^ campaign_seed())
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
}

// ---------------------------------------------------------------------
// Random planar frames: the whole point of the campaign.
// ---------------------------------------------------------------------

/// An orthonormal frame `(u, v, n)` at `origin`. Profiles are authored in
/// 2D and placed through a frame, so the identity frame reproduces the
/// existing XY-plane tests and every other frame is new coverage.
#[derive(Clone, Copy, Debug)]
struct Frame {
    origin: Point3,
    u: Vector3,
    v: Vector3,
    n: Vector3,
}

impl Frame {
    fn identity() -> Self {
        Frame {
            origin: Point3::origin(),
            u: Vector3::x(),
            v: Vector3::y(),
            n: Vector3::z(),
        }
    }

    /// A uniformly-ish random rigid placement of the identity frame.
    fn random(rng: &mut Rng) -> Self {
        let axis = Unit::new_normalize(Vector3::new(
            rng.range(-1.0, 1.0),
            rng.range(-1.0, 1.0),
            rng.range(-1.0, 1.0),
        ));
        let rot = Rotation3::from_axis_angle(&axis, rng.range(0.0, 2.0 * PI));
        Frame {
            origin: Point3::new(
                rng.range(-2.0, 2.0),
                rng.range(-2.0, 2.0),
                rng.range(-2.0, 2.0),
            ),
            u: rot * Vector3::x(),
            v: rot * Vector3::y(),
            n: rot * Vector3::z(),
        }
    }

    fn at(&self, p: [f64; 2]) -> Point3 {
        self.origin + self.u * p[0] + self.v * p[1]
    }

    fn dir(&self, d: [f64; 3]) -> Vector3 {
        self.u * d[0] + self.v * d[1] + self.n * d[2]
    }
}

/// A closed profile from 2D vertices placed through `frame`, wound so its
/// normal is `frame.n` (the vertices must already be counterclockwise).
fn profile_from(frame: &Frame, vertices: &[[f64; 2]]) -> Profile {
    let n = vertices.len();
    let segments = (0..n)
        .map(|i| {
            ProfileSegment::line_between(frame.at(vertices[i]), frame.at(vertices[(i + 1) % n]))
                .expect("distinct consecutive vertices")
        })
        .collect();
    Profile::new(segments).expect("valid closed planar profile")
}

// ---------------------------------------------------------------------
// 2D polygon measures — the closed-form side of every oracle below.
// ---------------------------------------------------------------------

/// Signed area of a simple polygon (positive when counterclockwise).
fn polygon_area(vertices: &[[f64; 2]]) -> f64 {
    let n = vertices.len();
    (0..n)
        .map(|i| {
            let (a, b) = (vertices[i], vertices[(i + 1) % n]);
            a[0] * b[1] - b[0] * a[1]
        })
        .sum::<f64>()
        / 2.0
}

/// Area centroid of a simple polygon.
fn polygon_centroid(vertices: &[[f64; 2]]) -> [f64; 2] {
    let n = vertices.len();
    let area = polygon_area(vertices);
    let mut cx = 0.0;
    let mut cy = 0.0;
    for i in 0..n {
        let (a, b) = (vertices[i], vertices[(i + 1) % n]);
        let cross = a[0] * b[1] - b[0] * a[1];
        cx += (a[0] + b[0]) * cross;
        cy += (a[1] + b[1]) * cross;
    }
    [cx / (6.0 * area), cy / (6.0 * area)]
}

/// The product moment `∫∫ x·y dA` of a simple polygon about the origin,
/// by exact triangle-fan decomposition.
///
/// For a triangle with vertices `p₀ p₁ p₂` and signed area `A`, writing the
/// integrand in barycentric coordinates and using
/// `∫ λᵢλⱼ dA = A/12` (`i ≠ j`), `A/6` (`i = j`) gives
/// `∫ xy dA = A/12 · [(Σxᵢ)(Σyⱼ) + Σᵢ xᵢyᵢ]`. Fanning from the origin
/// (`p₀ = O`) collapses that to the expression below, and the signed areas
/// cancel the parts of the fan outside the polygon.
///
/// This is what the revolved solid's axial centroid needs: `dV = 2πx dA`
/// weights the profile by its *radius*, so the solid's centroid sits at
/// `∫xy dA / ∫x dA` — **not** at the profile's area centroid, which is only
/// the answer when the profile is symmetric about its own mid-height.
fn polygon_moment_xy(vertices: &[[f64; 2]]) -> f64 {
    let n = vertices.len();
    (0..n)
        .map(|i| {
            let (a, b) = (vertices[i], vertices[(i + 1) % n]);
            let area = (a[0] * b[1] - b[0] * a[1]) / 2.0;
            area / 12.0 * ((a[0] + b[0]) * (a[1] + b[1]) + a[0] * a[1] + b[0] * b[1])
        })
        .sum()
}

/// A random counterclockwise convex-ish polygon: one vertex per angular
/// sector at a random radius, which keeps the loop simple by construction.
fn random_polygon(rng: &mut Rng, n: usize, r_lo: f64, r_hi: f64) -> Vec<[f64; 2]> {
    let sector = 2.0 * PI / n as f64;
    (0..n)
        .map(|i| {
            let theta = i as f64 * sector + rng.range(0.15 * sector, 0.85 * sector);
            let r = rng.range(r_lo, r_hi);
            [r * theta.cos(), r * theta.sin()]
        })
        .collect()
}

/// The same polygon shifted clear of `x = 0` so it can be revolved about the
/// frame's `v` axis without crossing it.
fn shifted_off_axis(polygon: &[[f64; 2]], clearance: f64) -> Vec<[f64; 2]> {
    let min_x = polygon.iter().map(|p| p[0]).fold(f64::INFINITY, f64::min);
    polygon
        .iter()
        .map(|p| [p[0] - min_x + clearance, p[1]])
        .collect()
}

/// Area ratio of the regular `n`-gon inscribed in the unit circle:
/// `n·sin(2π/n) / 2π`. The tessellated revolution replaces every circular
/// cross-section by this n-gon, so it scales the whole volume by exactly
/// this constant (see the [module docs](self)).
fn ngon_area_ratio(n: usize) -> f64 {
    let n = n as f64;
    n * (2.0 * PI / n).sin() / (2.0 * PI)
}

// ---------------------------------------------------------------------
// Assertions
// ---------------------------------------------------------------------

fn assert_close(got: f64, want: f64, rtol: f64, context: &str) {
    let scale = want.abs().max(1e-300);
    assert!(
        ((got - want) / scale).abs() <= rtol,
        "{context}: got {got}, expected {want} \
         ({:.3e} relative, allowed {rtol:.1e})",
        ((got - want) / scale).abs()
    );
}

/// `check()` must be clean and the tessellation a closed manifold; returns
/// the mesh and its mass properties for further measurement.
fn measured(body: &SweptBody, resolution: usize, context: &str) -> (TriangleMesh, MassProperties) {
    let failures = body.check();
    assert!(
        failures.is_empty(),
        "{context}: check() reported {} failures: {:#?}",
        failures.len(),
        failures
    );
    let mesh = body
        .tessellate(resolution)
        .unwrap_or_else(|e| panic!("{context}: tessellation failed: {e:?}"));
    assert!(
        mesh.is_closed_manifold(),
        "{context}: tessellation is not a closed manifold ({} triangles)",
        mesh.triangle_count()
    );
    let props =
        mass_properties(&mesh).unwrap_or_else(|e| panic!("{context}: mass_properties failed: {e}"));
    assert!(
        props.volume > 0.0,
        "{context}: tessellation encloses a NEGATIVE volume {} — the shell is \
         inside-out",
        props.volume
    );
    (mesh, props)
}

fn assert_point_close(got: Point3, want: Point3, atol: f64, context: &str) {
    let gap = (got - want).norm();
    assert!(
        gap <= atol,
        "{context}: point {got:?} differs from expected {want:?} by {gap:.3e} \
         (allowed {atol:.1e})"
    );
}

// =====================================================================
// (1) Extrusion
// =====================================================================

/// Random convex polygons in random planes, extruded along random directions
/// oblique to those planes. Every prism must be valid, closed, correctly
/// oriented, and carry exactly the topology `extrude` documents; its volume
/// and centroid are asserted against the closed forms (see module docs).
#[test]
fn random_polygon_extrusions_are_exact_prisms() {
    const RTOL: f64 = 1e-9;
    let mut rng = Rng::new(0xE_C701_0DE0);
    for case in 0..24 {
        let frame = if case == 0 {
            // Case 0 is the axis-aligned control: whatever a random frame
            // breaks, it should not be something already broken at identity.
            Frame::identity()
        } else {
            Frame::random(&mut rng)
        };
        let n = 3 + rng.pick(6);
        let polygon = random_polygon(&mut rng, n, 0.4, 1.6);
        let area = polygon_area(&polygon);
        let centroid2d = polygon_centroid(&polygon);

        // Oblique by construction: a nonzero normal component (so the
        // extrusion is legal) plus a random in-plane shear.
        let height = rng.range(0.5, 2.5) * if rng.pick(2) == 0 { 1.0 } else { -1.0 };
        let shear = [rng.range(-1.2, 1.2), rng.range(-1.2, 1.2)];
        let direction = frame.dir([shear[0], shear[1], height]);

        let repro = format!(
            "case {case}: extrude({n}-gon area {area:.6}, height {height:.6}, \
             shear {shear:?}) [seed 0xE_C701_0DE0]"
        );

        let profile = profile_from(&frame, &polygon);
        let body = extrude(&profile, direction)
            .unwrap_or_else(|e| panic!("{repro}: extrude failed: {e:?}"));

        // `extrude` documents 2n vertices, 3n edges, n + 2 faces, genus 0.
        let counts = body.store.euler_counts(body.body);
        assert_eq!(
            (counts.vertices, counts.edges, counts.faces, counts.genus),
            (2 * n, 3 * n, n + 2, 0),
            "{repro}: topology counts"
        );

        let (_, props) = measured(&body, 8, &repro);
        // Sheared or not, the volume is base area times PERPENDICULAR
        // height — Cavalieri. A frame bug that used |direction| instead
        // shows up here as soon as the shear is nonzero.
        assert_close(props.volume, area * height.abs(), RTOL, &repro);
        assert_point_close(
            props.centroid,
            frame.at(centroid2d) + direction / 2.0,
            1e-9 * (1.0 + direction.norm()),
            &format!("{repro}: centroid"),
        );
    }
}

/// Shearing an extrusion at fixed perpendicular height must not change its
/// volume, and reversing the direction must not change it either (winding is
/// normalized internally, so both senses produce an outward shell). Asserted
/// between *pairs* of bodies, so it holds even where the closed form above
/// would not.
#[test]
fn random_extrusion_volume_is_shear_and_sense_invariant() {
    const RTOL: f64 = 1e-9;
    let mut rng = Rng::new(0x5_4EA6_0001);
    for case in 0..12 {
        let frame = Frame::random(&mut rng);
        let n = 3 + rng.pick(5);
        let polygon = random_polygon(&mut rng, n, 0.4, 1.4);
        let profile = profile_from(&frame, &polygon);
        let height = rng.range(0.6, 2.0);
        let repro = format!("case {case}: shear/sense invariance, height {height:.6}");

        let straight = extrude(&profile, frame.dir([0.0, 0.0, height]))
            .unwrap_or_else(|e| panic!("{repro}: straight extrude failed: {e:?}"));
        let (_, straight_props) = measured(&straight, 8, &format!("{repro}: straight"));

        for trial in 0..3 {
            let shear = [rng.range(-1.5, 1.5), rng.range(-1.5, 1.5)];
            let sheared = extrude(&profile, frame.dir([shear[0], shear[1], height]))
                .unwrap_or_else(|e| panic!("{repro}/{trial}: sheared extrude failed: {e:?}"));
            let (_, props) = measured(&sheared, 8, &format!("{repro}/{trial}: sheared {shear:?}"));
            assert_close(
                props.volume,
                straight_props.volume,
                RTOL,
                &format!("{repro}/{trial}: sheared by {shear:?} vs straight"),
            );
        }

        let reversed = extrude(&profile, frame.dir([0.0, 0.0, -height]))
            .unwrap_or_else(|e| panic!("{repro}: reversed extrude failed: {e:?}"));
        let (_, rev_props) = measured(&reversed, 8, &format!("{repro}: reversed"));
        assert_close(
            rev_props.volume,
            straight_props.volume,
            RTOL,
            &format!("{repro}: reversed sense vs forward"),
        );
    }
}

/// A direction lying in the profile plane cannot extrude, at any frame.
/// The rejection is a documented contract and the in-plane test is a
/// dot-product against the frame normal — exactly the kind of check that
/// silently only works for axis-aligned planes.
#[test]
fn random_in_plane_extrusion_directions_are_rejected() {
    let mut rng = Rng::new(0x1_2A2E_0001);
    for case in 0..16 {
        let frame = Frame::random(&mut rng);
        let n = 4 + rng.pick(3);
        let profile = profile_from(&frame, &random_polygon(&mut rng, n, 0.5, 1.5));
        let (a, b) = (rng.range(-2.0, 2.0), rng.range(-2.0, 2.0));
        // Skip a direction that rounds to zero; it is rejected for a
        // different (also correct) reason and would not test the plane check.
        if a.hypot(b) < 1e-6 {
            continue;
        }
        let direction = frame.dir([a, b, 0.0]);
        let repro = format!("case {case}: in-plane direction ({a:.6}, {b:.6}, 0)");
        match extrude(&profile, direction) {
            Ok(_) => panic!("{repro}: expected rejection, got a body"),
            Err(CoreError::InvalidArgument { argument, .. }) => {
                assert_eq!(argument, "direction", "{repro}: wrong argument named")
            }
            Err(e) => panic!("{repro}: expected InvalidArgument on `direction`, got {e:?}"),
        }
    }
}

// =====================================================================
// (2) Revolution
// =====================================================================

/// Random polygons clear of the axis, revolved a full turn about a random
/// in-plane axis. The Pappus volume `2π·x̄·A`, scaled by the tessellation's
/// exact n-gon ratio, is the oracle; the topology must be genus 1 (the
/// profile never touches the axis, so the solid is torus-like).
#[test]
fn random_off_axis_revolutions_match_pappus() {
    const RTOL: f64 = 1e-9;
    const RESOLUTION: usize = 48;
    let mut rng = Rng::new(0x2E_7013_0001);
    for case in 0..20 {
        let frame = if case == 0 {
            Frame::identity()
        } else {
            Frame::random(&mut rng)
        };
        let n = 3 + rng.pick(5);
        let raw = random_polygon(&mut rng, n, 0.3, 1.0);
        let polygon = shifted_off_axis(&raw, rng.range(0.3, 1.2));
        let area = polygon_area(&polygon);
        let centroid2d = polygon_centroid(&polygon);

        let repro = format!(
            "case {case}: revolve({n}-gon area {area:.6}, x̄ {:.6}) [seed 0x2E_7013_0001]",
            centroid2d[0]
        );

        let profile = profile_from(&frame, &polygon);
        // The axis is the frame's `v` line through its origin: in the
        // profile plane by construction, and not axis-aligned in world space
        // for any random frame.
        let body = revolve(&profile, frame.origin, frame.v)
            .unwrap_or_else(|e| panic!("{repro}: revolve failed: {e:?}"));

        let counts = body.store.euler_counts(body.body);
        assert_eq!(
            counts.genus, 1,
            "{repro}: a profile clear of the axis must revolve to a genus-1 solid, \
             got genus {} ({} vertices, {} edges, {} faces)",
            counts.genus, counts.vertices, counts.edges, counts.faces
        );
        // `revolve` documents it: one periodic face per profile segment, one
        // closed circular edge per profile vertex, one seam vertex on each of
        // those edges. Each periodic face closes on its own two circular
        // edges, so there is no separate seam edge to count.
        assert_eq!(
            (counts.vertices, counts.edges, counts.faces),
            (n, n, n),
            "{repro}: topology counts"
        );

        let (_, props) = measured(&body, RESOLUTION, &repro);
        let pappus = 2.0 * PI * centroid2d[0] * area;
        assert_close(
            props.volume,
            pappus * ngon_area_ratio(RESOLUTION),
            RTOL,
            &format!("{repro}: Pappus volume {pappus:.9} × n-gon ratio"),
        );
        // The centroid of a full revolution is on the axis, at the
        // RADIUS-WEIGHTED axial height `∫xy dA / ∫x dA` (see
        // `polygon_moment_xy`). The n-gon discretization scales every
        // cross-section by the same constant, so it cancels here.
        let axial = polygon_moment_xy(&polygon) / (centroid2d[0] * area);
        assert_point_close(
            props.centroid,
            frame.origin + frame.v * axial,
            1e-9 * (1.0 + frame.origin.coords.norm()),
            &format!("{repro}: centroid (radius-weighted axial height {axial:.9})"),
        );
    }
}

/// A rectangle with one side *on* the axis revolves to a cylinder: the
/// on-axis segment vanishes into the interior and the result is genus 0.
/// Randomized over radius, height, axial offset and frame.
#[test]
fn random_on_axis_rectangles_revolve_to_cylinders() {
    const RTOL: f64 = 1e-9;
    const RESOLUTION: usize = 48;
    let mut rng = Rng::new(0x0_C711_0001);
    for case in 0..12 {
        let frame = if case == 0 {
            Frame::identity()
        } else {
            Frame::random(&mut rng)
        };
        let r = rng.range(0.3, 1.5);
        let y0 = rng.range(-1.5, 0.5);
        let h = rng.range(0.5, 2.5);
        let y1 = y0 + h;
        // Counterclockwise about +n: (0,y0) → (r,y0) → (r,y1) → (0,y1).
        let polygon = [[0.0, y0], [r, y0], [r, y1], [0.0, y1]];
        let repro = format!("case {case}: revolve(rect r = {r:.6}, y ∈ [{y0:.6}, {y1:.6}])");

        let profile = profile_from(&frame, &polygon);
        let body = revolve(&profile, frame.origin, frame.v)
            .unwrap_or_else(|e| panic!("{repro}: revolve failed: {e:?}"));

        let counts = body.store.euler_counts(body.body);
        assert_eq!(
            counts.genus, 0,
            "{repro}: a rectangle with a side on the axis must revolve to a genus-0 \
             cylinder, got genus {}",
            counts.genus
        );

        let (_, props) = measured(&body, RESOLUTION, &repro);
        let exact = PI * r * r * h;
        assert_close(
            props.volume,
            exact * ngon_area_ratio(RESOLUTION),
            RTOL,
            &format!("{repro}: cylinder volume {exact:.9} × n-gon ratio"),
        );
        assert_point_close(
            props.centroid,
            frame.origin + frame.v * (y0 + h / 2.0),
            1e-9 * (1.0 + frame.origin.coords.norm()),
            &format!("{repro}: centroid"),
        );
    }
}

/// The tessellated volume must converge to the true Pappus volume as the
/// angular resolution rises, at the `n`-gon rate — which is the statement
/// that the *only* error is the documented discretization. A frame or
/// winding bug contributes a resolution-independent offset and breaks this
/// even where a single-resolution check might pass on a lucky tolerance.
#[test]
fn revolution_volume_converges_at_the_ngon_rate() {
    let mut rng = Rng::new(0xC09A_E000);
    for case in 0..6 {
        let frame = Frame::random(&mut rng);
        let polygon = shifted_off_axis(&random_polygon(&mut rng, 5, 0.3, 0.9), 0.5);
        let area = polygon_area(&polygon);
        let x_bar = polygon_centroid(&polygon)[0];
        let pappus = 2.0 * PI * x_bar * area;
        let profile = profile_from(&frame, &polygon);
        let body = revolve(&profile, frame.origin, frame.v).expect("valid revolution");
        let repro = format!("case {case}: convergence, Pappus volume {pappus:.9}");

        let mut previous_error = f64::INFINITY;
        for resolution in [12, 24, 48, 96] {
            let (_, props) = measured(&body, resolution, &format!("{repro} @ {resolution}"));
            assert_close(
                props.volume,
                pappus * ngon_area_ratio(resolution),
                1e-9,
                &format!("{repro} @ {resolution}"),
            );
            let error = (props.volume - pappus).abs() / pappus;
            assert!(
                error < previous_error,
                "{repro}: relative error {error:.3e} at resolution {resolution} did not \
                 improve on {previous_error:.3e}"
            );
            previous_error = error;
        }
        // The n-gon deficit is `1 - k(n) ≈ (2π)² / 6n²`, which at n = 96 is
        // 7.1e-4. Anything materially above that is not discretization.
        let deficit = 1.0 - ngon_area_ratio(96);
        assert!(
            previous_error <= deficit * 1.01,
            "{repro}: relative error {previous_error:.3e} at resolution 96 exceeds the \
             n-gon deficit {deficit:.3e} — the residual is not discretization"
        );
    }
}

/// An axis out of the profile plane, and a profile straddling the axis, are
/// both documented rejections. Randomized so the checks are exercised in
/// generic frames rather than only where the axis is a coordinate axis.
#[test]
fn random_invalid_revolution_axes_are_rejected() {
    let mut rng = Rng::new(0x1_2AAE_0001);
    for case in 0..12 {
        let frame = Frame::random(&mut rng);
        let polygon = shifted_off_axis(&random_polygon(&mut rng, 5, 0.3, 0.9), 0.4);
        let profile = profile_from(&frame, &polygon);

        // (a) An axis tilted out of the profile plane.
        let tilt = rng.range(0.1, 1.0);
        let tilted = frame.dir([0.0, 1.0, tilt]);
        let repro = format!("case {case}: axis tilted {tilt:.6} out of the profile plane");
        match revolve(&profile, frame.origin, tilted) {
            Ok(_) => panic!("{repro}: expected rejection, got a body"),
            Err(CoreError::InvalidArgument { argument, .. }) => {
                assert_eq!(argument, "axis_dir", "{repro}: wrong argument named")
            }
            Err(e) => panic!("{repro}: expected InvalidArgument on `axis_dir`, got {e:?}"),
        }

        // (b) An in-plane axis the profile straddles: shift the axis point
        // into the polygon's own x-range so the profile crosses it.
        let x_mid = polygon.iter().map(|p| p[0]).sum::<f64>() / polygon.len() as f64;
        let crossing = frame.at([x_mid, 0.0]);
        let repro = format!("case {case}: axis through the profile at x = {x_mid:.6}");
        match revolve(&profile, crossing, frame.v) {
            Ok(_) => panic!("{repro}: expected rejection, got a body"),
            Err(CoreError::InvalidArgument { .. }) => {}
            Err(e) => panic!("{repro}: expected InvalidArgument, got {e:?}"),
        }
    }
}

// =====================================================================
// (3) Rigid-motion invariance of the whole constructor
// =====================================================================

/// The same 2D profile and the same frame-relative direction, placed through
/// two different random frames, must produce congruent solids: identical
/// topology counts and identical volume. This is the campaign's strongest
/// single statement, because it needs no closed form at all — it compares
/// the constructor against itself under a rigid motion, so any frame-
/// dependent behaviour fails it regardless of what the right answer is.
#[test]
fn sweeps_are_invariant_under_rigid_placement() {
    const RTOL: f64 = 1e-9;
    let mut rng = Rng::new(0x0008_161D_0001);
    for case in 0..12 {
        let n = 3 + rng.pick(5);
        let polygon = random_polygon(&mut rng, n, 0.4, 1.3);
        let off_axis = shifted_off_axis(&polygon, rng.range(0.3, 0.9));
        let height = rng.range(0.6, 2.0);
        let shear = [rng.range(-1.0, 1.0), rng.range(-1.0, 1.0)];
        let repro = format!("case {case}: rigid invariance of a {n}-gon sweep");

        let frames = [
            Frame::identity(),
            Frame::random(&mut rng),
            Frame::random(&mut rng),
        ];

        let mut extruded = Vec::new();
        let mut revolved = Vec::new();
        for (i, frame) in frames.iter().enumerate() {
            let ctx = format!("{repro}, frame {i}");
            let profile = profile_from(frame, &polygon);
            let body = extrude(&profile, frame.dir([shear[0], shear[1], height]))
                .unwrap_or_else(|e| panic!("{ctx}: extrude failed: {e:?}"));
            let counts = body.store.euler_counts(body.body);
            let (_, props) = measured(&body, 8, &format!("{ctx}: extrusion"));
            extruded.push((counts.vertices, counts.edges, counts.faces, props.volume));

            let profile = profile_from(frame, &off_axis);
            let body = revolve(&profile, frame.origin, frame.v)
                .unwrap_or_else(|e| panic!("{ctx}: revolve failed: {e:?}"));
            let counts = body.store.euler_counts(body.body);
            let (_, props) = measured(&body, 32, &format!("{ctx}: revolution"));
            revolved.push((counts.vertices, counts.edges, counts.faces, props.volume));
        }

        for (label, results) in [("extrusion", &extruded), ("revolution", &revolved)] {
            let (v0, e0, f0, vol0) = results[0];
            for (i, &(v, e, f, vol)) in results.iter().enumerate().skip(1) {
                assert_eq!(
                    (v, e, f),
                    (v0, e0, f0),
                    "{repro}: {label} topology under frame {i} differs from the \
                     identity frame"
                );
                assert_close(
                    vol,
                    vol0,
                    RTOL,
                    &format!("{repro}: {label} volume under frame {i} vs identity"),
                );
            }
        }
    }
}
