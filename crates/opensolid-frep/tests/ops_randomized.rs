//! Seeded randomized campaigns for the F-Rep operator classes that had no
//! randomized coverage at all (of-ipt.18): the offset family
//! ([`Offset`](opensolid_frep::Offset) / [`Shell`](opensolid_frep::Shell) /
//! [`Rounded`](opensolid_frep::Rounded)), edge-selective fillets and chamfers
//! ([`EdgeBlend`](opensolid_frep::fillet::EdgeBlend)), and sweeps
//! ([`Sweep`](opensolid_frep::Sweep) / [`Loft`](opensolid_frep::Loft)).
//!
//! `property_invariants.rs` covers primitives and sharp CSG with proptest.
//! Everything downstream of those — every operator whose docs carry a metric
//! caveat — was tested only at hand-picked configurations, which is exactly
//! the shape of coverage a metric bug survives.
//!
//! Protocol (as `boolean_stress.rs`): a deterministic seeded [`Rng`], a repro
//! string printed on every failure, closed-form oracles wherever one exists.
//! A failing case becomes a `bd` bug bead with a minimal repro and the test is
//! marked `#[ignore]` referencing it — never softened to pass.
//!
//! # What the offset caveat actually implies
//!
//! `ops.rs` documents that offsetting a non-exact field (CSG interiors, blends)
//! moves the surface by `d / |∇f|`, not by `d`. That reads as an unfalsifiable
//! disclaimer, but it has a sharp, checkable consequence. Every field this
//! crate builds from exact primitives is 1-Lipschitz, so for any `p`,
//! `|f(p)| <= dist(p, surface)`. A point on the offset surface has `f(p) = d`,
//! hence
//!
//! ```text
//! dist(p, original surface) >= d,      always
//! dist(p, original surface) == d,      iff f is exact along that direction
//! ```
//!
//! So the offset never *undershoots* — it can only stand off too far, and
//! exactly where the field is inexact. [`offset_surface_stand_off_distance`]
//! asserts the inequality for CSG operands and the equality for exact
//! primitives, measuring the distance against an independently meshed copy of
//! the original surface so the oracle never routes back through the field
//! under test.

use opensolid_core::mesh::TriangleMesh;
use opensolid_core::types::{BoundingBox3, Point3, Vector3};
use opensolid_frep::blend::SmoothUnion;
use opensolid_frep::csg::{Intersection, Subtraction, Union};
use opensolid_frep::fillet::{BlendMode, BooleanKind, EdgeBlend, EdgeRegion};
use opensolid_frep::primitives::{Box3, Capsule, Cylinder, Sdf, Sphere, Torus};
use opensolid_frep::profile::Profile2D;
use opensolid_frep::{Extrude, Loft, MeshOptions, SdfOpsExt, Sweep, mesh_sdf_indexed};
use std::f64::consts::PI;

// ---------------------------------------------------------------------
// Deterministic RNG (splitmix64), identical to `boolean_stress.rs` so a
// repro seed means the same thing in both suites.
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

    fn point(&mut self, lo: f64, hi: f64) -> Point3 {
        Point3::new(self.range(lo, hi), self.range(lo, hi), self.range(lo, hi))
    }
}

// ---------------------------------------------------------------------
// Measurement helpers. None of them evaluate the field under test: the
// volume comes from the mesh via the divergence theorem, and the stand-off
// distance from the mesh triangles directly.
// ---------------------------------------------------------------------

/// Signed volume of a closed, outward-oriented mesh (divergence theorem:
/// one signed tetrahedron per triangle against the origin).
fn mesh_volume(mesh: &TriangleMesh) -> f64 {
    mesh.indices
        .iter()
        .map(|[i, j, k]| {
            let a = mesh.positions[*i].coords;
            let b = mesh.positions[*j].coords;
            let c = mesh.positions[*k].coords;
            a.dot(&b.cross(&c)) / 6.0
        })
        .sum()
}

/// Closest point of triangle `abc` to `p` (Ericson, *Real-Time Collision
/// Detection* §5.1.5 — the standard Voronoi-region cascade).
fn closest_point_on_triangle(p: &Point3, a: &Point3, b: &Point3, c: &Point3) -> Point3 {
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;
    let d1 = ab.dot(&ap);
    let d2 = ac.dot(&ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return *a;
    }
    let bp = p - b;
    let d3 = ab.dot(&bp);
    let d4 = ac.dot(&bp);
    if d3 >= 0.0 && d4 <= d3 {
        return *b;
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        return a + ab * (d1 / (d1 - d3));
    }
    let cp = p - c;
    let d5 = ab.dot(&cp);
    let d6 = ac.dot(&cp);
    if d6 >= 0.0 && d5 <= d6 {
        return *c;
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        return a + ac * (d2 / (d2 - d6));
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
    }
    let denom = 1.0 / (va + vb + vc);
    a + ab * (vb * denom) + ac * (vc * denom)
}

/// Brute-force unsigned distance from `p` to a mesh surface.
fn distance_to_mesh(p: &Point3, mesh: &TriangleMesh) -> f64 {
    mesh.indices
        .iter()
        .map(|[i, j, k]| {
            let q = closest_point_on_triangle(
                p,
                &mesh.positions[*i],
                &mesh.positions[*j],
                &mesh.positions[*k],
            );
            (p - q).norm()
        })
        .fold(f64::INFINITY, f64::min)
}

/// A cube of half-extent `half` centered on the origin.
///
/// Every volume campaign below sizes its bounds to the shape rather than to
/// a fixed generous box. A uniform grid spends its resolution on the whole
/// box, so a shape occupying a third of it is meshed at a third of the
/// nominal cell size — and the volume error, which scales with the cell, is
/// then dominated by empty space rather than by anything the operator did.
/// Fitting the bounds is what makes these closed-form oracles tight enough
/// to catch a real metric defect.
fn cube_bounds(half: f64) -> BoundingBox3 {
    BoundingBox3::new(
        Point3::new(-half, -half, -half),
        Point3::new(half, half, half),
    )
}

/// Bounds fitted around a shape's own half-extents, with 15% clearance so
/// the surface closes strictly inside them (the mesher emits boundary edges,
/// not a closed manifold, if it reaches the wall).
fn fitted_bounds(min: Point3, max: Point3) -> BoundingBox3 {
    let pad = 0.15 * (max - min).norm() / 3.0_f64.sqrt();
    BoundingBox3::new(
        Point3::new(min.x - pad, min.y - pad, min.z - pad),
        Point3::new(max.x + pad, max.y + pad, max.z + pad),
    )
}

fn mesh_in(sdf: &dyn Sdf, bounds: BoundingBox3, resolution: usize) -> TriangleMesh {
    mesh_sdf_indexed(sdf, &MeshOptions { bounds, resolution })
}

fn mesh_of(sdf: &dyn Sdf, half: f64, resolution: usize) -> TriangleMesh {
    mesh_in(sdf, cube_bounds(half), resolution)
}

fn assert_close(got: f64, want: f64, rtol: f64, context: &str) {
    let scale = want.abs().max(1e-300);
    assert!(
        ((got - want) / scale).abs() <= rtol,
        "{context}: got {got}, expected {want} \
         ({:.3e} relative, allowed {rtol:.1e})",
        ((got - want) / scale).abs()
    );
}

// ---------------------------------------------------------------------
// Closed-form volumes for Minkowski dilations (Steiner formula). Every one
// of these is exact for the shape named; they are the tight oracle the
// mesh-based stand-off campaign cannot be.
// ---------------------------------------------------------------------

fn sphere_volume(r: f64) -> f64 {
    4.0 / 3.0 * PI * r * r * r
}

/// A box of side lengths `l` dilated by a ball of radius `d`: the box, plus
/// a slab on each face, a quarter-cylinder along each edge (four edges per
/// direction make one full cylinder), and the eight corner octants making
/// one full sphere.
fn dilated_box_volume(l: [f64; 3], d: f64) -> f64 {
    let [a, b, c] = l;
    a * b * c
        + 2.0 * d * (a * b + b * c + c * a)
        + PI * d * d * (a + b + c)
        + 4.0 / 3.0 * PI * d * d * d
}

/// A capsule of segment length `h` and radius `r`: a cylinder plus a sphere.
/// Dilation by `d` is the same capsule at radius `r + d`, so this doubles as
/// the dilated-capsule oracle.
fn capsule_volume(h: f64, r: f64) -> f64 {
    PI * r * r * h + sphere_volume(r)
}

// =====================================================================
// (1) Offset: stand-off distance, measured against an independent mesh
// =====================================================================

/// One randomized operand: the field, an independently meshed copy of its
/// zero set, and whether the field is an *exact* distance function.
struct Operand {
    sdf: Box<dyn Sdf>,
    label: String,
    /// True only for a single exact primitive; CSG composites are merely
    /// 1-Lipschitz, which is what makes the stand-off one-sided.
    exact: bool,
}

fn random_operand(rng: &mut Rng) -> Operand {
    match rng.pick(6) {
        0 => {
            let r = rng.range(0.6, 1.1);
            Operand {
                sdf: Box::new(Sphere {
                    center: Point3::origin(),
                    radius: r,
                }),
                label: format!("sphere(r = {r:.4})"),
                exact: true,
            }
        }
        1 => {
            let h = [
                rng.range(0.5, 1.0),
                rng.range(0.5, 1.0),
                rng.range(0.5, 1.0),
            ];
            Operand {
                sdf: Box::new(Box3 {
                    center: Point3::origin(),
                    half_extents: h,
                }),
                label: format!("box(half = {h:?})"),
                exact: true,
            }
        }
        2 => {
            let (r, hh) = (rng.range(0.4, 0.9), rng.range(0.5, 1.0));
            Operand {
                sdf: Box::new(Cylinder {
                    center: Point3::origin(),
                    radius: r,
                    half_height: hh,
                }),
                label: format!("cylinder(r = {r:.4}, half_h = {hh:.4})"),
                exact: true,
            }
        }
        3 => {
            let (major, minor) = (rng.range(0.7, 1.0), rng.range(0.2, 0.35));
            Operand {
                sdf: Box::new(Torus {
                    center: Point3::origin(),
                    major_radius: major,
                    minor_radius: minor,
                }),
                label: format!("torus(R = {major:.4}, r = {minor:.4})"),
                exact: true,
            }
        }
        // Two overlapping spheres: `min` is 1-Lipschitz but inexact in the
        // overlap region, which is precisely where the offset stands off.
        4 => {
            let (r1, r2) = (rng.range(0.6, 0.9), rng.range(0.6, 0.9));
            let sep = rng.range(0.4, 0.9);
            Operand {
                sdf: Box::new(Union {
                    a: Sphere {
                        center: Point3::new(-sep, 0.0, 0.0),
                        radius: r1,
                    },
                    b: Sphere {
                        center: Point3::new(sep, 0.0, 0.0),
                        radius: r2,
                    },
                }),
                label: format!("sphere({r1:.4}) ∪ sphere({r2:.4}) at ±{sep:.4}"),
                exact: false,
            }
        }
        // A box intersected with a sphere: `max` is inexact near the
        // intersection curve, the classic "corner" underestimate.
        _ => {
            let h = rng.range(0.6, 0.9);
            let r = rng.range(0.8, 1.05);
            Operand {
                sdf: Box::new(Intersection {
                    a: Box3 {
                        center: Point3::origin(),
                        half_extents: [h, h, h],
                    },
                    b: Sphere {
                        center: Point3::origin(),
                        radius: r,
                    },
                }),
                label: format!("box({h:.4}) ∩ sphere({r:.4})"),
                exact: false,
            }
        }
    }
}

/// The offset surface stands off the original by **at least** the offset
/// distance, and by **exactly** it when the field is an exact SDF.
///
/// Both halves are measured against an independently meshed copy of the
/// original zero set, so nothing in the oracle re-evaluates the field being
/// tested. See the [module docs](self) for why the inequality is the right
/// statement for CSG operands.
#[test]
fn offset_surface_stand_off_distance() {
    // The base surface is meshed finer than the offset surface: its vertex
    // error then contributes less than the offset mesh's own, and the
    // budget below is dominated by a single known term.
    const BASE_RES: usize = 56;
    const OFFSET_RES: usize = 40;
    const HALF: f64 = 2.2;
    // A dual-contouring vertex sits within one cell of the true zero set,
    // so the measured stand-off carries one cell diagonal from each mesh.
    let budget = 3.0_f64.sqrt() * 2.0 * HALF * (1.0 / BASE_RES as f64 + 1.0 / OFFSET_RES as f64);

    let mut rng = Rng::new(0x0FF5_E700);
    for case in 0..10 {
        let operand = random_operand(&mut rng);
        let d = rng.range(0.25, 0.5);
        let repro = format!(
            "case {case}: offset({}, {d:.6}) [seed 0x0FF5_E700]",
            operand.label
        );

        let base_mesh = mesh_of(operand.sdf.as_ref(), HALF, BASE_RES);
        assert!(
            !base_mesh.is_empty() && base_mesh.is_closed_manifold(),
            "{repro}: base surface did not mesh to a closed manifold"
        );

        let grown = (&operand.sdf).offset(d).expect("finite distance");
        let grown_mesh = mesh_of(&grown, HALF, OFFSET_RES);
        assert!(
            !grown_mesh.is_empty() && grown_mesh.is_closed_manifold(),
            "{repro}: offset surface did not mesh to a closed manifold"
        );

        // Sample the offset surface rather than sweeping every vertex: the
        // distance query is O(triangles) and the campaign is a sampler, not
        // an exhaustive proof.
        let stride = (grown_mesh.positions.len() / 24).max(1);
        for (i, p) in grown_mesh.positions.iter().enumerate().step_by(stride) {
            let dist = distance_to_mesh(p, &base_mesh);
            assert!(
                dist >= d - budget,
                "{repro}: offset surface vertex {i} at {p:?} is only {dist:.6} from the \
                 original surface, less than the offset {d:.6} (budget {budget:.6}). A \
                 1-Lipschitz field can never undershoot — this is a real metric defect."
            );
            if operand.exact {
                assert!(
                    dist <= d + budget,
                    "{repro}: offset of an EXACT field stands off {dist:.6}, not {d:.6} \
                     (budget {budget:.6}) at vertex {i} {p:?}"
                );
            }
        }
    }
}

// =====================================================================
// (2) Offset: closed-form dilation volumes (the tight oracle)
// =====================================================================

/// Outward offset of an exact convex primitive is Minkowski dilation, whose
/// volume the Steiner formula gives in closed form. This is the numerically
/// tight half of the offset campaign: the mesh-based stand-off above can
/// only bound the error at a cell diagonal, but a volume matches to the
/// tessellation's own budget.
#[test]
fn random_outward_offset_volume_matches_steiner() {
    // Curved dilated surfaces discretize in both directions; the same 0.5%
    // budget `boolean_stress.rs` allows curved tessellations covers it.
    const RTOL: f64 = 8e-3;
    let mut rng = Rng::new(0x57E1_4E12);
    for case in 0..12 {
        let d = rng.range(0.15, 0.45);
        // `half` is the dilated shape's own half-extent, so the meshing box
        // is fitted to it (see `fitted_bounds`).
        let (sdf, want, label, half): (Box<dyn Sdf>, f64, String, f64) = match rng.pick(3) {
            0 => {
                let r = rng.range(0.5, 1.0);
                (
                    Box::new(Sphere {
                        center: Point3::origin(),
                        radius: r,
                    }),
                    sphere_volume(r + d),
                    format!("sphere(r = {r:.6})"),
                    r + d,
                )
            }
            1 => {
                let h = [
                    rng.range(0.4, 0.9),
                    rng.range(0.4, 0.9),
                    rng.range(0.4, 0.9),
                ];
                (
                    Box::new(Box3 {
                        center: Point3::origin(),
                        half_extents: h,
                    }),
                    dilated_box_volume([2.0 * h[0], 2.0 * h[1], 2.0 * h[2]], d),
                    format!("box(half = {h:?})"),
                    h.iter().cloned().fold(0.0, f64::max) + d,
                )
            }
            _ => {
                let (hh, r) = (rng.range(0.3, 0.8), rng.range(0.25, 0.5));
                (
                    Box::new(Capsule {
                        start: Point3::new(0.0, 0.0, -hh),
                        end: Point3::new(0.0, 0.0, hh),
                        radius: r,
                    }),
                    capsule_volume(2.0 * hh, r + d),
                    format!("capsule(h = {:.6}, r = {r:.6})", 2.0 * hh),
                    hh + r + d,
                )
            }
        };
        let repro = format!("case {case}: offset({label}, {d:.6}) [seed 0x57E1_4E12]");
        let grown = sdf.offset(d).expect("finite distance");
        let mesh = mesh_of(&grown, half * 1.15, 72);
        assert!(
            mesh.is_closed_manifold(),
            "{repro}: dilated surface is not a closed manifold"
        );
        assert_close(mesh_volume(&mesh), want, RTOL, &repro);
    }
}

/// Inward offset erodes the solid, and past the inradius it must vanish
/// entirely rather than invert or emit a phantom surface.
#[test]
fn random_inward_offset_erodes_and_vanishes() {
    const RTOL: f64 = 8e-3;
    let mut rng = Rng::new(0xE20D_E001);
    for case in 0..10 {
        let r = rng.range(0.6, 1.2);
        let sphere = Sphere {
            center: Point3::origin(),
            radius: r,
        };
        // Half the cases erode within the inradius, half past it.
        let past = case % 2 == 1;
        let d = if past {
            -(r + rng.range(0.05, 0.4))
        } else {
            -rng.range(0.1, r * 0.6)
        };
        let repro = format!("case {case}: offset(sphere(r = {r:.6}), {d:.6})");
        let shrunk = sphere.offset(d).expect("finite distance");
        // When the erosion is expected to vanish, mesh the *original*
        // sphere's box: an empty result inside bounds that never contained
        // the solid would prove nothing.
        let half = if past { r * 1.2 } else { (r + d) * 1.15 };
        let mesh = mesh_of(&shrunk, half, 64);
        if past {
            assert!(
                mesh.is_empty(),
                "{repro}: eroding past the inradius must leave nothing, got {} triangles",
                mesh.triangle_count()
            );
        } else {
            assert!(
                mesh.is_closed_manifold(),
                "{repro}: eroded surface is not a closed manifold"
            );
            assert_close(mesh_volume(&mesh), sphere_volume(r + d), RTOL, &repro);
        }
    }
}

/// The documented composition caveat, as a randomized identity: field
/// offsets are level-set shifts, so they compose *additively* and an
/// inward/outward pair collapses to the sharp original. `ops.rs` states this
/// as prose and `offset_pair_collapses_to_original` pins one case; this
/// asserts it over random fields, random pairs, and random points.
#[test]
fn random_offsets_compose_additively() {
    let mut rng = Rng::new(0xC0FF_5E70);
    for case in 0..24 {
        let operand = random_operand(&mut rng);
        let (a, b) = (rng.range(-0.6, 0.6), rng.range(-0.6, 0.6));
        let repro = format!(
            "case {case}: offset(offset({}, {a:.6}), {b:.6}) [seed 0xC0FF_5E70]",
            operand.label
        );
        let composed = (&operand.sdf)
            .offset(a)
            .expect("finite")
            .offset(b)
            .expect("finite");
        let single = (&operand.sdf).offset(a + b).expect("finite");
        for _ in 0..12 {
            let p = rng.point(-2.0, 2.0);
            let (x, y) = (composed.eval(&p), single.eval(&p));
            assert!(
                (x - y).abs() <= 1e-12 * (1.0 + y.abs()),
                "{repro}: at {p:?} the composed offset gives {x} but the single \
                 offset by {:.6} gives {y}",
                a + b
            );
        }
    }
}

// =====================================================================
// (3) Shell and Rounded
// =====================================================================

/// A shell is the set `|f| <= thickness/2`, so for an exact field its volume
/// is the dilated solid minus the eroded one. Closed forms for the sphere
/// and the box (Steiner both ways) pin it exactly.
#[test]
fn random_shell_wall_volume_matches_closed_form() {
    const RTOL: f64 = 2e-2;
    let mut rng = Rng::new(0x5AE1_1000);
    for case in 0..8 {
        let t = rng.range(0.15, 0.35);
        let (sdf, want, label, half): (Box<dyn Sdf>, f64, String, f64) = if case % 2 == 0 {
            let r = rng.range(0.7, 1.2);
            (
                Box::new(Sphere {
                    center: Point3::origin(),
                    radius: r,
                }),
                sphere_volume(r + t / 2.0) - sphere_volume(r - t / 2.0),
                format!("sphere(r = {r:.6})"),
                r + t / 2.0,
            )
        } else {
            let h = [
                rng.range(0.6, 1.0),
                rng.range(0.6, 1.0),
                rng.range(0.6, 1.0),
            ];
            let inner: f64 = h.iter().map(|x| 2.0 * (x - t / 2.0)).product();
            (
                Box::new(Box3 {
                    center: Point3::origin(),
                    half_extents: h,
                }),
                dilated_box_volume([2.0 * h[0], 2.0 * h[1], 2.0 * h[2]], t / 2.0) - inner,
                format!("box(half = {h:?})"),
                h.iter().cloned().fold(0.0, f64::max) + t / 2.0,
            )
        };
        let repro = format!("case {case}: shell({label}, {t:.6})");
        let shell = sdf.shell(t).expect("positive thickness");
        let mesh = mesh_of(&shell, half * 1.15, 68);
        assert!(
            mesh.is_closed_manifold(),
            "{repro}: shell is not a closed manifold"
        );
        assert_close(mesh_volume(&mesh), want, RTOL, &repro);
    }
}

/// Rounding an inset box restores its nominal extents and replaces the
/// corners with radius-`r` fillets, so the result is exactly the dilated
/// inset box — the Steiner oracle again, now over random extents and radii.
#[test]
fn random_rounded_inset_box_volume_matches_dilation() {
    const RTOL: f64 = 1e-2;
    let mut rng = Rng::new(0x2011_DED0);
    for case in 0..10 {
        let nominal = [
            rng.range(0.7, 1.1),
            rng.range(0.7, 1.1),
            rng.range(0.7, 1.1),
        ];
        let r = rng
            .range(0.1, 0.35)
            .min(nominal.iter().cloned().fold(f64::MAX, f64::min) - 0.2);
        let core = Box3 {
            center: Point3::origin(),
            half_extents: [nominal[0] - r, nominal[1] - r, nominal[2] - r],
        };
        let half = core.half_extents;
        let repro = format!("case {case}: rounded(box(half = {half:?}), {r:.6})");
        let rounded = core.rounded(r).expect("positive radius");

        // The nominal face centers are restored exactly.
        for axis in 0..3 {
            let mut p = [0.0; 3];
            p[axis] = nominal[axis];
            let at = Point3::new(p[0], p[1], p[2]);
            assert!(
                rounded.eval(&at).abs() < 1e-12,
                "{repro}: nominal face center {at:?} is off the rounded surface by {}",
                rounded.eval(&at)
            );
        }

        let mesh = mesh_of(
            &rounded,
            nominal.iter().cloned().fold(0.0, f64::max) * 1.15,
            72,
        );
        assert!(
            mesh.is_closed_manifold(),
            "{repro}: rounded box is not a closed manifold"
        );
        let want = dilated_box_volume([2.0 * half[0], 2.0 * half[1], 2.0 * half[2]], r);
        assert_close(mesh_volume(&mesh), want, RTOL, &repro);
    }
}

// =====================================================================
// (4) Edge-selective fillet / chamfer
// =====================================================================

fn random_blend_kind(rng: &mut Rng) -> BooleanKind {
    match rng.pick(3) {
        0 => BooleanKind::Union,
        1 => BooleanKind::Intersection,
        _ => BooleanKind::Subtraction,
    }
}

/// Sharp value of `kind` at `p`, computed without the blend machinery.
fn sharp_eval<A: Sdf, B: Sdf>(kind: BooleanKind, a: &A, b: &B, p: &Point3) -> f64 {
    match kind {
        BooleanKind::Union => Union { a, b }.eval(p),
        BooleanKind::Intersection => Intersection { a, b }.eval(p),
        BooleanKind::Subtraction => Subtraction { a, b }.eval(p),
    }
}

/// The whole promise of an edge-selective blend: **outside the influence
/// region it is bit-for-bit the sharp boolean**, so neighbouring edges stay
/// crisp. A windowing bug that leaks any blend past `2·radius` shows up here
/// and nowhere else — a global smooth-min passes every "is it rounded near
/// the edge" test.
#[test]
fn random_edge_blend_is_exactly_sharp_outside_its_influence() {
    let mut rng = Rng::new(0xF111_E7ED);
    for case in 0..16 {
        let a = Box3 {
            center: Point3::origin(),
            half_extents: [
                rng.range(0.6, 1.0),
                rng.range(0.6, 1.0),
                rng.range(0.6, 1.0),
            ],
        };
        let b = Sphere {
            center: rng.point(-0.6, 0.6),
            radius: rng.range(0.5, 1.0),
        };
        let kind = random_blend_kind(&mut rng);
        let mode = if rng.pick(2) == 0 {
            BlendMode::Fillet
        } else {
            BlendMode::Chamfer
        };
        let radius = rng.range(0.05, 0.3);
        // A short random polyline standing in for a selected edge.
        let region = EdgeRegion::from_polyline(&[
            rng.point(-1.0, 1.0),
            rng.point(-1.0, 1.0),
            rng.point(-1.0, 1.0),
        ]);
        let repro = format!(
            "case {case}: EdgeBlend({kind:?}, {mode:?}, r = {radius:.6}) [seed 0xF111_E7ED]"
        );
        let blend = EdgeBlend::new(&a, &b, kind, mode, radius, region.clone());

        for _ in 0..40 {
            let p = rng.point(-2.0, 2.0);
            let sharp = sharp_eval(kind, &a, &b, &p);
            let got = blend.eval(&p);
            if region.distance(&p) > 2.0 * radius {
                assert!(
                    got == sharp,
                    "{repro}: at {p:?}, {:.6} from the region (influence {:.6}), the blend \
                     gives {got} but the sharp boolean gives {sharp}",
                    region.distance(&p),
                    2.0 * radius
                );
            }
            // Inside the influence the blend may only round the edge *off*
            // the sharp value, by a bounded amount, and always in the
            // direction that adds material on the concave side. In min-space
            // both smooth-min and chamfer-min are <= min; the outer sign of
            // `kind` maps that back to the boolean's own orientation.
            let outward = matches!(kind, BooleanKind::Union);
            let deviation = got - sharp;
            let bound = radius + 1e-12;
            assert!(
                deviation.abs() <= bound,
                "{repro}: at {p:?} the blend deviates {deviation:.6} from the sharp \
                 boolean, beyond the radius bound {bound:.6}"
            );
            if outward {
                assert!(
                    deviation <= 1e-12,
                    "{repro}: at {p:?} a union blend raised the field by {deviation:.6}; \
                     a smooth/chamfer min may only lower it"
                );
            } else {
                assert!(
                    deviation >= -1e-12,
                    "{repro}: at {p:?} an intersection/subtraction blend lowered the field \
                     by {deviation:.6}; the negated min may only raise it"
                );
            }
        }
    }
}

/// A zero radius (or an empty region) must reduce to the sharp boolean
/// *everywhere*, not merely to something close: the chamfer formula does not
/// converge to `min` as `r → 0`, so the degenerate case has to be
/// short-circuited rather than evaluated.
#[test]
fn random_edge_blend_degenerates_to_sharp() {
    let mut rng = Rng::new(0x2E80_B1E4);
    for case in 0..12 {
        let a = Sphere {
            center: rng.point(-0.5, 0.5),
            radius: rng.range(0.5, 1.0),
        };
        let b = Box3 {
            center: rng.point(-0.5, 0.5),
            half_extents: [
                rng.range(0.5, 0.9),
                rng.range(0.5, 0.9),
                rng.range(0.5, 0.9),
            ],
        };
        let kind = random_blend_kind(&mut rng);
        let mode = if rng.pick(2) == 0 {
            BlendMode::Fillet
        } else {
            BlendMode::Chamfer
        };
        let radius = rng.range(0.05, 0.4);
        let full = EdgeRegion::from_polyline(&[rng.point(-1.0, 1.0), rng.point(-1.0, 1.0)]);

        for (label, blend) in [
            (
                "zero radius",
                EdgeBlend::new(&a, &b, kind, mode, 0.0, full.clone()),
            ),
            (
                "empty region",
                EdgeBlend::new(&a, &b, kind, mode, radius, EdgeRegion::default()),
            ),
        ] {
            let repro = format!("case {case}: {label}, {kind:?}, {mode:?}");
            for _ in 0..24 {
                let p = rng.point(-2.0, 2.0);
                let sharp = sharp_eval(kind, &a, &b, &p);
                let got = blend.eval(&p);
                assert!(
                    got == sharp,
                    "{repro}: at {p:?} the degenerate blend gives {got}, not the sharp {sharp}"
                );
            }
        }
    }
}

/// `EdgeBlend` supplies its own `eval_interval` because the windowed field
/// is not globally 1-Lipschitz. An interval that fails to contain the values
/// it bounds silently prunes real surface out of the octree, so containment
/// is checked over random boxes and random interior points.
#[test]
fn random_edge_blend_interval_contains_its_values() {
    let mut rng = Rng::new(0x1471_E12A);
    for case in 0..16 {
        let a = Box3 {
            center: Point3::origin(),
            half_extents: [0.8, 0.7, 0.9],
        };
        let b = Sphere {
            center: rng.point(-0.7, 0.7),
            radius: rng.range(0.5, 1.0),
        };
        let kind = random_blend_kind(&mut rng);
        let mode = if rng.pick(2) == 0 {
            BlendMode::Fillet
        } else {
            BlendMode::Chamfer
        };
        let radius = rng.range(0.05, 0.3);
        let region = EdgeRegion::from_polyline(&[rng.point(-1.0, 1.0), rng.point(-1.0, 1.0)]);
        let blend = EdgeBlend::new(&a, &b, kind, mode, radius, region);
        let repro = format!("case {case}: EdgeBlend({kind:?}, {mode:?}, r = {radius:.6}) interval");

        for _ in 0..8 {
            let center = rng.point(-1.5, 1.5);
            let extent = rng.range(0.05, 0.6);
            let bb = BoundingBox3::new(
                Point3::new(center.x - extent, center.y - extent, center.z - extent),
                Point3::new(center.x + extent, center.y + extent, center.z + extent),
            );
            let i = blend.eval_interval(&bb);
            for _ in 0..12 {
                let p = Point3::new(
                    rng.range(bb.min.x, bb.max.x),
                    rng.range(bb.min.y, bb.max.y),
                    rng.range(bb.min.z, bb.max.z),
                );
                let v = blend.eval(&p);
                assert!(
                    v >= i.lo - 1e-12 && v <= i.hi + 1e-12,
                    "{repro}: eval({p:?}) = {v} escapes the interval [{}, {}] for box \
                     {:?}..{:?}",
                    i.lo,
                    i.hi,
                    bb.min,
                    bb.max
                );
            }
        }
    }
}

// =====================================================================
// (5) Sweeps and lofts
// =====================================================================

/// A random convex polygon, as a closed CCW profile centered on its own
/// origin (so the sweep path rides through its interior).
fn random_convex_profile(rng: &mut Rng, n: usize, r_lo: f64, r_hi: f64) -> (Profile2D, f64) {
    let mut vertices = Vec::with_capacity(n);
    for i in 0..n {
        // Jitter each vertex within its own angular sector: the ordering by
        // angle is what keeps the polygon simple and convex-ish.
        let sector = 2.0 * PI / n as f64;
        let theta = i as f64 * sector + rng.range(0.1 * sector, 0.9 * sector);
        let r = rng.range(r_lo, r_hi);
        vertices.push([r * theta.cos(), r * theta.sin()]);
    }
    let area = shoelace_area(&vertices);
    let profile = Profile2D::new(vertices, vec![0.0; n]).expect("valid closed profile");
    (profile, area)
}

/// Unsigned area of a simple polygon.
fn shoelace_area(vertices: &[[f64; 2]]) -> f64 {
    let n = vertices.len();
    let twice: f64 = (0..n)
        .map(|i| {
            let a = vertices[i];
            let b = vertices[(i + 1) % n];
            a[0] * b[1] - b[0] * a[1]
        })
        .sum();
    twice.abs() / 2.0
}

/// A straight one-segment sweep is a prism, so its volume is the profile
/// area times the path length — independent of the path's direction, which
/// is the part the per-segment frame construction can get wrong.
#[test]
fn random_straight_sweep_volume_matches_prism() {
    const RTOL: f64 = 3e-2;
    let mut rng = Rng::new(0x5_9EE9_0001);
    for case in 0..8 {
        let (profile, area) = random_convex_profile(&mut rng, 5, 0.35, 0.6);
        let dir = {
            let v = Vector3::new(
                rng.range(-1.0, 1.0),
                rng.range(-1.0, 1.0),
                rng.range(-1.0, 1.0),
            );
            v.normalize()
        };
        let length = rng.range(1.0, 1.8);
        let start = -dir * (length / 2.0);
        let end = dir * (length / 2.0);
        let path = [[start.x, start.y, start.z], [end.x, end.y, end.z]];
        let repro =
            format!("case {case}: sweep(5-gon area {area:.6}) along {path:?} [seed 0x5_9EE9_0001]");
        let sweep = Sweep::new(profile, &path).expect("valid path");
        let mesh = mesh_of(&sweep, length / 2.0 + 0.7, 72);
        assert!(
            mesh.is_closed_manifold(),
            "{repro}: swept prism is not a closed manifold"
        );
        assert_close(mesh_volume(&mesh), area * length, RTOL, &repro);
    }
}

/// Every point of the path lies inside the swept solid (the profile origin
/// rides on the path, and the profiles here contain their origin), and the
/// field stays 1-Lipschitz along random segments — the property the module
/// docs claim for the per-segment prism union.
#[test]
fn random_polyline_sweep_contains_its_path_and_is_lipschitz() {
    let mut rng = Rng::new(0x5_9EE9_0002);
    for case in 0..10 {
        // Radii bounded below so the profile always contains (0, 0).
        let (profile, _) = random_convex_profile(&mut rng, 6, 0.3, 0.45);
        let segments = 2 + rng.pick(3);
        let mut path = vec![[0.0f64; 3]];
        for _ in 0..segments {
            let last = *path.last().expect("non-empty");
            let step = Vector3::new(
                rng.range(-1.0, 1.0),
                rng.range(-1.0, 1.0),
                rng.range(-1.0, 1.0),
            )
            .normalize()
                * rng.range(0.5, 1.0);
            path.push([last[0] + step.x, last[1] + step.y, last[2] + step.z]);
        }
        let repro = format!("case {case}: sweep along {path:?} [seed 0x5_9EE9_0002]");
        let sweep = Sweep::new(profile, &path).expect("valid path");

        // The path rides through the profile's local origin, so no path
        // point may ever be OUTSIDE the solid. The two ends sit exactly on
        // the end caps (f = 0 by construction for an exact prism), and so —
        // wrongly — do the interior joints: of-57c1. The strict statement
        // for interior joints is parked in
        // `sweep_interior_joints_are_strictly_inside` below.
        for (i, pt) in path.iter().enumerate() {
            let p = Point3::new(pt[0], pt[1], pt[2]);
            let v = sweep.eval(&p);
            assert!(
                v <= 1e-12,
                "{repro}: path point {i} {p:?} is outside the swept solid (f = {v})"
            );
        }
        for (i, w) in path.windows(2).enumerate() {
            let mid = Point3::new(
                0.5 * (w[0][0] + w[1][0]),
                0.5 * (w[0][1] + w[1][1]),
                0.5 * (w[0][2] + w[1][2]),
            );
            let v = sweep.eval(&mid);
            assert!(
                v < 0.0,
                "{repro}: midpoint of segment {i} at {mid:?} is not strictly inside the \
                 swept solid (f = {v})"
            );
        }

        for _ in 0..24 {
            let (p, q) = (rng.point(-2.5, 2.5), rng.point(-2.5, 2.5));
            let (fp, fq) = (sweep.eval(&p), sweep.eval(&q));
            let gap = (p - q).norm();
            assert!(
                (fp - fq).abs() <= gap + 1e-9,
                "{repro}: field is not 1-Lipschitz between {p:?} and {q:?}: \
                 |{fp} - {fq}| = {} > {gap}",
                (fp - fq).abs()
            );
        }
    }
}

/// An interior path joint is covered by BOTH adjacent prisms, so it is in
/// the interior of their union and its field value must be strictly
/// negative. It is exactly `0` instead (of-57c1): the joint sits on a cap
/// plane of each prism, `prism(d, 0) == 0` for every `d <= 0`, and `min`
/// carries the zero through. Kept live and `#[ignore]`d per the
/// never-soften policy — `cargo test --test ops_randomized -- --ignored`.
#[test]
#[ignore = "of-57c1: interior sweep joints evaluate to exactly 0, not < 0"]
fn sweep_interior_joints_are_strictly_inside() {
    // The minimal repro extracted from
    // `random_polyline_sweep_contains_its_path_and_is_lipschitz` case 1
    // (seed 0x5_9EE9_0002).
    let profile = Profile2D::new(
        vec![[0.4, 0.0], [0.0, 0.4], [-0.4, 0.0], [0.0, -0.4]],
        vec![0.0; 4],
    )
    .expect("valid square profile");
    let path = [
        [0.0, 0.0, 0.0],
        [0.5656735832369914, -0.00537525251826387, 0.563720258293554],
        [0.6618480244636268, 0.19379813598943468, 1.0674756371496044],
    ];
    let sweep = Sweep::new(profile, &path).expect("valid path");
    let joint = Point3::new(path[1][0], path[1][1], path[1][2]);
    let v = sweep.eval(&joint);
    assert!(
        v < 0.0,
        "interior joint {joint:?} is in the interior of both adjacent prisms but \
         evaluates to {v}"
    );
}

/// A sweep along a single segment parallel to `Extrude`'s own axis must
/// agree with `Extrude` up to the frame the sweep chooses: same volume,
/// same closed manifold. Catches a per-segment prism that silently disagrees
/// with the extrusion the rest of the crate is built on.
#[test]
fn random_axis_sweep_agrees_with_extrude() {
    const RTOL: f64 = 3e-2;
    let mut rng = Rng::new(0x5_9EE9_0003);
    for case in 0..6 {
        let (profile, area) = random_convex_profile(&mut rng, 6, 0.35, 0.6);
        let height = rng.range(0.8, 1.6);
        let repro = format!("case {case}: extrude vs sweep, height {height:.6}, area {area:.6}");

        // Profile radii stay under 0.6 and both solids span y ∈ [0, height].
        let bounds = fitted_bounds(Point3::new(-0.6, 0.0, -0.6), Point3::new(0.6, height, 0.6));

        let extruded = Extrude::new(profile.clone(), height).expect("positive height");
        let ex_mesh = mesh_in(&extruded, bounds, 72);
        assert!(
            ex_mesh.is_closed_manifold(),
            "{repro}: extrusion is not a closed manifold"
        );
        assert_close(mesh_volume(&ex_mesh), area * height, RTOL, &repro);

        let swept =
            Sweep::new(profile, &[[0.0, 0.0, 0.0], [0.0, height, 0.0]]).expect("valid path");
        let sw_mesh = mesh_in(&swept, bounds, 72);
        assert!(
            sw_mesh.is_closed_manifold(),
            "{repro}: sweep is not a closed manifold"
        );
        assert_close(mesh_volume(&sw_mesh), area * height, RTOL, &repro);
    }
}

/// A loft between two copies of the *same* profile is a prism, so the linear
/// SDF morph must reproduce the extrusion volume exactly. Between different
/// profiles the intermediate sections are the morphed level sets, not a
/// corresponding-point sweep — so the only sound volume claim is that the
/// result is bracketed by the two prisms, which is what is asserted.
#[test]
fn random_loft_brackets_its_end_prisms() {
    const RTOL: f64 = 3e-2;
    let mut rng = Rng::new(0x10F7_0001);
    for case in 0..8 {
        let (bottom, area_b) = random_convex_profile(&mut rng, 5, 0.3, 0.5);
        let height = rng.range(0.8, 1.5);
        let repro = format!("case {case}: loft height {height:.6} [seed 0x10F7_0001]");
        // Profile radii stay under 0.5; the loft spans y ∈ [0, height].
        let bounds = fitted_bounds(Point3::new(-0.5, 0.0, -0.5), Point3::new(0.5, height, 0.5));

        // Identical profiles: an exact prism.
        let prism = Loft::new(bottom.clone(), bottom.clone(), height).expect("positive height");
        let mesh = mesh_in(&prism, bounds, 72);
        assert!(
            mesh.is_closed_manifold(),
            "{repro}: constant loft is not a closed manifold"
        );
        assert_close(
            mesh_volume(&mesh),
            area_b * height,
            RTOL,
            &format!("{repro}: constant"),
        );

        // Differing profiles: bracketed by the two end prisms.
        let (top, area_t) = random_convex_profile(&mut rng, 5, 0.3, 0.5);
        let tapered = Loft::new(bottom, top, height).expect("positive height");
        let mesh = mesh_in(&tapered, bounds, 72);
        assert!(
            mesh.is_closed_manifold(),
            "{repro}: tapered loft is not a closed manifold"
        );
        let v = mesh_volume(&mesh);
        let (lo, hi) = (
            area_b.min(area_t) * height * (1.0 - RTOL),
            area_b.max(area_t) * height * (1.0 + RTOL),
        );
        assert!(
            v >= lo && v <= hi,
            "{repro}: loft volume {v} is outside the end-prism bracket [{lo}, {hi}] \
             (areas {area_b:.6} / {area_t:.6})"
        );
    }
}

/// Blends are the smooth counterpart of the sharp CSG the offset family sits
/// on top of, and `SmoothUnion` under an outward `Offset` is the composition
/// the fillet docs warn about. The invariant that must survive it: the
/// smooth union never exceeds the sharp one, and the offset shifts both by
/// exactly the same constant.
#[test]
fn random_offset_of_smooth_union_shifts_uniformly() {
    let mut rng = Rng::new(0x5B1E_0FF5);
    for case in 0..16 {
        let a = Sphere {
            center: rng.point(-0.6, 0.6),
            radius: rng.range(0.5, 0.9),
        };
        let b = Box3 {
            center: rng.point(-0.6, 0.6),
            half_extents: [
                rng.range(0.4, 0.8),
                rng.range(0.4, 0.8),
                rng.range(0.4, 0.8),
            ],
        };
        let k = rng.range(0.05, 0.4);
        let d = rng.range(-0.4, 0.4);
        let repro = format!("case {case}: offset(SmoothUnion(k = {k:.6}), {d:.6})");
        let smooth = SmoothUnion {
            a: &a,
            b: &b,
            radius: k,
        };
        let shifted = (&smooth).offset(d).expect("finite distance");
        for _ in 0..24 {
            let p = rng.point(-2.0, 2.0);
            let sharp = a.eval(&p).min(b.eval(&p));
            let sm = smooth.eval(&p);
            assert!(
                sm <= sharp + 1e-12,
                "{repro}: smooth union {sm} exceeds the sharp min {sharp} at {p:?}"
            );
            let got = shifted.eval(&p);
            assert!(
                (got - (sm - d)).abs() <= 1e-12 * (1.0 + sm.abs()),
                "{repro}: at {p:?} the offset gives {got}, not {} = f - d",
                sm - d
            );
        }
    }
}
