//! of-stjd: end-to-end pseudonormal closure for a plate with a tilted
//! through-bore — the synthetic twin of nist_ctc_02's of-aoml failure with
//! no corpus file in the loop.
//!
//! The shell is assembled exactly the way the STEP reader's mesh fallback
//! assembles one: each face triangulated by [`triangulate_bounded_face`]
//! from shared, byte-identical boundary polylines, welded at zero epsilon.
//! A bore whose axis is tilted off the plate leaves its rim samples exactly
//! in the plate's plane, so any triangle realized from rim samples alone is
//! flat there, antiparallel to the plate's facets, and cancels the rim
//! vertices' angle-weighted pseudonormals — which [`MeshSdf::new`] refuses.
//! The of-aoml lens split is what prevents that; this gate holds it to the
//! full closed-shell standard, not just the one wall's triangle list.

use std::f64::consts::TAU;

use opensolid_kernel::MeshSdf;
use opensolid_kernel::brep::{Surface3, TessellationOptions, triangulate_bounded_face};
use opensolid_kernel::core::mesh::TriangleMesh;
use opensolid_kernel::core::types::{Point3, Vector3};

/// Plate half-width, plate thickness, bore radius, rim sample count.
const W: f64 = 50.0;
const H: f64 = 20.0;
const R: f64 = 10.0;
const N: usize = 48;

/// A point of the bore-wall cylinder (axis tilted `theta` about y, through
/// the origin) at angle `u` and the height where the wall meets the plane
/// `z = h`: the rim ellipse every face involved must sample identically.
fn rim_point(theta: f64, u: f64, h: f64) -> Point3 {
    let (s, c) = (theta.sin(), theta.cos());
    let e_u = Vector3::new(c, 0.0, -s);
    let e_v = Vector3::new(0.0, 1.0, 0.0);
    let axis = Vector3::new(s, 0.0, c);
    let v = h / c + R * (s / c) * u.cos();
    Point3::origin() + e_u * (R * u.cos()) + e_v * (R * u.sin()) + axis * v
}

/// The rim ellipse at `z = h` as a closed polyline, `u` ascending.
fn rim_ring(theta: f64, h: f64) -> Vec<Point3> {
    (0..N)
        .map(|i| rim_point(theta, TAU * i as f64 / N as f64, h))
        .collect()
}

/// Assemble the plate-with-tilted-through-bore shell and weld it.
fn tilted_bore_plate(theta: f64) -> TriangleMesh {
    let options = TessellationOptions::default();
    let band = 1e-6;
    let mut mesh = TriangleMesh::new();
    let bot = rim_ring(theta, 0.0);
    let top = rim_ring(theta, H);

    // Squares counter-clockwise seen from above.
    let square = |z: f64| -> Vec<Point3> {
        vec![
            Point3::new(-W, -W, z),
            Point3::new(W, -W, z),
            Point3::new(W, W, z),
            Point3::new(-W, W, z),
        ]
    };
    let reversed = |ring: &[Point3]| -> Vec<Point3> { ring.iter().rev().copied().collect() };

    // Bottom face: outward -z, so the plane chart (normal +z) is flipped;
    // rings counter-clockwise seen from below.
    triangulate_bounded_face(
        &Surface3::Plane {
            origin: Point3::origin(),
            normal: Vector3::z(),
        },
        vec![reversed(&square(0.0)), bot.clone()],
        true,
        band,
        &options,
        &mut mesh,
    )
    .expect("bottom face triangulates");
    // Top face: outward +z; rings counter-clockwise seen from above.
    triangulate_bounded_face(
        &Surface3::Plane {
            origin: Point3::new(0.0, 0.0, H),
            normal: Vector3::z(),
        },
        vec![square(H), reversed(&top)],
        false,
        band,
        &options,
        &mut mesh,
    )
    .expect("top face triangulates");
    // The four plate sides, each a rectangle wound counter-clockwise seen
    // from its own outward normal.
    let sq0 = square(0.0);
    for k in 0..4 {
        let (a, b) = (sq0[k], sq0[(k + 1) % 4]);
        let (a_top, b_top) = (Point3::new(a.x, a.y, H), Point3::new(b.x, b.y, H));
        let outward = Vector3::new((b.y - a.y) / (2.0 * W), (a.x - b.x) / (2.0 * W), 0.0);
        triangulate_bounded_face(
            &Surface3::Plane {
                origin: a,
                normal: -outward,
            },
            vec![vec![a, b, b_top, a_top]],
            true,
            band,
            &options,
            &mut mesh,
        )
        .unwrap_or_else(|e| panic!("side face {k} triangulates: {e}"));
    }
    // Bore wall: the cylinder's radial-out normal points away from the
    // solid, so the face is flipped. One ring, the way the fallback builds
    // it: bottom rim a full turn (closing sample repeated), up the seam,
    // top rim a full turn back, and the closure runs down the same seam.
    let mut wall: Vec<Point3> = Vec::with_capacity(2 * N + 2);
    wall.extend(bot.iter().copied());
    wall.push(bot[0]);
    wall.push(top[0]);
    wall.extend(top.iter().rev().copied());
    let (s, c) = (theta.sin(), theta.cos());
    triangulate_bounded_face(
        &Surface3::Cylinder {
            origin: Point3::origin(),
            axis: Vector3::new(s, 0.0, c),
            radius: R,
        },
        vec![wall],
        true,
        band,
        &options,
        &mut mesh,
    )
    .expect("bore wall triangulates");

    mesh.weld(0.0)
}

/// The gate the of-aoml fix answers for, at the tilts it holds today.
#[test]
fn tilted_through_bore_plate_closes_and_has_pseudonormals() {
    for &deg in &[2.0f64, 5.0] {
        let shell = tilted_bore_plate(deg.to_radians());
        assert!(
            shell.is_closed_manifold(),
            "tilt {deg}°: shell does not close: {:?}",
            shell.manifold_defects().describe()
        );
        MeshSdf::new(&shell)
            .unwrap_or_else(|e| panic!("tilt {deg}°: shell rejected as an SDF: {e}"));
    }
}

/// of-8oit: at steep tilts the wall still carries flat flap triangles
/// (see the boolean.rs battery — the lens outgrows the split band's
/// quarter-cell upper bound), and end-to-end the failure is *silent*: the
/// shell closes and `MeshSdf::new` accepts it, because at these
/// proportions the rim pseudonormals come out as garbage directions
/// rather than exact zeros — nist_ctc_02's loud refusal was the lucky
/// case. The accepted field then reports bore-void points near the rim as
/// **inside the solid** (measured: 4/96 probes at 20°, 14/96 at 45°).
/// This asserts the desired behavior — every bore-void sample strictly
/// outside — and is ignored until of-8oit lands.
#[test]
#[ignore = "of-8oit: flat flaps at tilt >= 20° flip SDF signs in the bore void"]
fn steep_tilt_sdf_keeps_its_signs() {
    use opensolid_kernel::frep::primitives::Sdf;
    let mut wrong = Vec::new();
    for &deg in &[10.0f64, 20.0, 45.0] {
        let theta = deg.to_radians();
        let shell = tilted_bore_plate(theta);
        assert!(
            shell.is_closed_manifold(),
            "tilt {deg}°: shell does not close"
        );
        let sdf = MeshSdf::new(&shell)
            .unwrap_or_else(|e| panic!("tilt {deg}°: shell rejected as an SDF: {e}"));
        // Sample all around the rim, just off the plate's bottom plane:
        // strictly below the plate (dz < 0) everything is air, and pulled
        // 30% toward the bore axis above it (dz > 0) is air in the void.
        for i in 0..N {
            let u = TAU * i as f64 / N as f64;
            for &dz in &[-0.05f64, 0.05] {
                let on_rim = rim_point(theta, u, 0.0);
                let (s, c) = (theta.sin(), theta.cos());
                let center = Point3::origin() + Vector3::new(s, 0.0, c) * (on_rim.z / c);
                let p = center + (on_rim - center) * 0.7 + Vector3::new(0.0, 0.0, dz);
                let d = sdf.eval(&p);
                if d < 0.0 {
                    wrong.push(format!(
                        "tilt {deg}°: bore-void point at u = {u:.3}, dz = {dz} \
                         reports {d:.3e} (inside)"
                    ));
                }
            }
        }
    }
    assert!(wrong.is_empty(), "{}", wrong.join("\n"));
}
