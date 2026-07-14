//! End-to-end tests for the rigid-body mate solver (of-fsl.25.2).
//!
//! The solver is decoupled from part geometry: it operates on instance *poses*
//! (a [`Transform3`] plus a `fixed` flag) and abstract plane/axis/point
//! features, so these tests drive it directly via [`solve_mates`] with no
//! geometry built. Each test checks the resolved poses satisfy the mates
//! (residual → 0), that DOF accounting is right, and that conflicts surface as
//! [`SolveStatus::OverConstrained`] rather than panicking. A final integration
//! test exercises the [`Assembly`] wrapper over real part instances.

use nalgebra::{Translation3, UnitQuaternion};
use opensolid_core::types::{BoundingBox3, Point3, Transform3, Vector3};
use opensolid_kernel::assembly::{
    Feature, FeatureRef, Mate, MateKind, SolveStatus, seat_concentric_coincident, solve_mates,
};

fn pose(t: [f64; 3], axis: [f64; 3], angle: f64) -> Transform3 {
    let rot = if angle == 0.0 {
        UnitQuaternion::identity()
    } else {
        UnitQuaternion::from_scaled_axis(
            Vector3::new(axis[0], axis[1], axis[2]).normalize() * angle,
        )
    };
    Transform3::from_parts(Translation3::new(t[0], t[1], t[2]), rot)
}

fn pt(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(x, y, z)
}
fn v(x: f64, y: f64, z: f64) -> Vector3 {
    Vector3::new(x, y, z)
}

/// World-space feature under a solved pose.
fn world(feature: Feature, iso: &Transform3) -> Feature {
    match feature {
        Feature::Plane { point, normal } => Feature::Plane {
            point: iso * point,
            normal: iso.rotation * normal,
        },
        Feature::Axis { point, direction } => Feature::Axis {
            point: iso * point,
            direction: iso.rotation * direction,
        },
        Feature::Point { point } => Feature::Point { point: iso * point },
    }
}

// ---------------------------------------------------------------------------
// Single-mate solves
// ---------------------------------------------------------------------------

#[test]
fn coincident_plane_plane_flush_and_antiparallel() {
    // Instance 0: ground plane z=0, +Z (fixed). Instance 1: floating part,
    // near-seated (normal ~25° off -Z, offset above).
    let poses = [
        Transform3::identity(),
        pose([2.0, 3.0, 4.0], [1.0, 0.3, 0.0], 2.7),
    ];
    let fixed = [true, false];
    let a = Feature::plane(pt(0.0, 0.0, 0.0), v(0.0, 0.0, 1.0)).unwrap();
    let b = Feature::plane(pt(0.0, 0.0, 0.0), v(0.0, 0.0, 1.0)).unwrap();
    let mates = [Mate::coincident(FeatureRef::new(0, a), FeatureRef::new(1, b)).unwrap()];

    let res = solve_mates(&poses, &fixed, &mates);
    assert_eq!(
        res.status,
        SolveStatus::Converged,
        "residual {}",
        res.residual_norm
    );
    assert!(res.residual_norm < 1e-8);

    // Floating plane became flush (z≈0) and anti-parallel (normal ≈ -Z).
    let Feature::Plane { point, normal } = world(b, &res.transforms[1]) else {
        unreachable!()
    };
    assert!(point.z.abs() < 1e-7, "flush: {}", point.z);
    assert!(
        (normal + v(0.0, 0.0, 1.0)).norm() < 1e-6,
        "antiparallel: {normal}"
    );

    // Coincident (face–face) removes 3 DOF → 3 remain (2 slide + 1 spin).
    assert_eq!(res.free_dof, 3);
}

#[test]
fn concentric_makes_axes_collinear_from_90_degrees() {
    // Instance 0: fixed bore along +Z through (1,2,0). Instance 1: floating
    // shaft rotated 90° off, elsewhere in space.
    let poses = [
        Transform3::identity(),
        pose([5.0, -4.0, 3.0], [0.0, 1.0, 0.0], 1.4),
    ];
    let fixed = [true, false];
    let bore_axis = Feature::axis(pt(1.0, 2.0, 0.0), v(0.0, 0.0, 1.0)).unwrap();
    let shaft_axis = Feature::axis(pt(0.0, 0.0, 0.0), v(0.0, 0.0, 1.0)).unwrap();
    let mates = [Mate::concentric(
        FeatureRef::new(0, bore_axis),
        FeatureRef::new(1, shaft_axis),
    )
    .unwrap()];

    let res = solve_mates(&poses, &fixed, &mates);
    assert_eq!(
        res.status,
        SolveStatus::Converged,
        "residual {}",
        res.residual_norm
    );
    assert!(res.residual_norm < 1e-8);

    let Feature::Axis { point, direction } = world(shaft_axis, &res.transforms[1]) else {
        unreachable!()
    };
    assert!(
        direction.cross(&v(0.0, 0.0, 1.0)).norm() < 1e-6,
        "dir {direction}"
    );
    assert!(
        (point.x - 1.0).abs() < 1e-6 && (point.y - 2.0).abs() < 1e-6,
        "line {point}"
    );

    // Concentric removes 4 DOF → 2 remain (slide along axis + spin).
    assert_eq!(res.free_dof, 2);
}

#[test]
fn distance_plane_plane_holds_offset() {
    let poses = [
        Transform3::identity(),
        pose([0.0, 0.0, 1.5], [1.0, 0.0, 0.0], 3.0),
    ];
    let fixed = [true, false];
    let a = Feature::plane(pt(0.0, 0.0, 0.0), v(0.0, 0.0, 1.0)).unwrap();
    let b = Feature::plane(pt(0.0, 0.0, 0.0), v(0.0, 0.0, 1.0)).unwrap();
    // Keep the part's plane 2.5 above ground along ground's +Z normal.
    let mates = [Mate::distance(FeatureRef::new(0, a), FeatureRef::new(1, b), -2.5).unwrap()];

    let res = solve_mates(&poses, &fixed, &mates);
    assert_eq!(
        res.status,
        SolveStatus::Converged,
        "residual {}",
        res.residual_norm
    );
    assert!(res.residual_norm < 1e-8);

    let Feature::Plane { point, .. } = world(b, &res.transforms[1]) else {
        unreachable!()
    };
    assert!((point.z - 2.5).abs() < 1e-6, "offset plane z {}", point.z);
}

#[test]
fn distance_point_point_holds_separation() {
    let poses = [
        Transform3::identity(),
        pose([3.0, 0.0, 0.0], [0.0, 0.0, 1.0], 0.5),
    ];
    let fixed = [true, false];
    let a = Feature::point(pt(0.0, 0.0, 0.0));
    let b = Feature::point(pt(0.0, 0.0, 0.0));
    let mates = [Mate::distance(FeatureRef::new(0, a), FeatureRef::new(1, b), 2.0).unwrap()];

    let res = solve_mates(&poses, &fixed, &mates);
    assert_eq!(
        res.status,
        SolveStatus::Converged,
        "residual {}",
        res.residual_norm
    );

    let Feature::Point { point } = world(b, &res.transforms[1]) else {
        unreachable!()
    };
    assert!(
        (point.coords.norm() - 2.0).abs() < 1e-6,
        "separation {}",
        point.coords.norm()
    );
}

// ---------------------------------------------------------------------------
// Concentric + coincident (fastener seat) — closed form and DOF
// ---------------------------------------------------------------------------

/// The canonical bolt-in-a-counterbore. Instance 0 = fixed plate, instance 1 =
/// floating bolt located by one concentric + one coincident mate. Returns the
/// poses, fixed flags, mates, and the bolt's shaft-axis / head-plane features.
fn bolt_case(bolt_start: Transform3) -> ([Transform3; 2], [bool; 2], Vec<Mate>, Feature, Feature) {
    let poses = [Transform3::identity(), bolt_start];
    let fixed = [true, false];

    // Plate: hole axis +Z through (1,1,·); seat plane z=0, normal +Z.
    let hole_axis = Feature::axis(pt(1.0, 1.0, 0.0), v(0.0, 0.0, 1.0)).unwrap();
    let seat_plane = Feature::plane(pt(0.0, 0.0, 0.0), v(0.0, 0.0, 1.0)).unwrap();
    // Bolt (local): shaft axis +Z through origin; head-bottom plane at z=2,
    // normal +Z (parallel to the axis — the canonical fastener).
    let shaft_axis = Feature::axis(pt(0.0, 0.0, 0.0), v(0.0, 0.0, 1.0)).unwrap();
    let head_plane = Feature::plane(pt(0.0, 0.0, 2.0), v(0.0, 0.0, 1.0)).unwrap();

    let mates = vec![
        Mate::concentric(
            FeatureRef::new(0, hole_axis),
            FeatureRef::new(1, shaft_axis),
        )
        .unwrap(),
        Mate::coincident(
            FeatureRef::new(0, seat_plane),
            FeatureRef::new(1, head_plane),
        )
        .unwrap(),
    ];
    (poses, fixed, mates, shaft_axis, head_plane)
}

#[test]
fn bolt_seat_uses_closed_form_and_is_exact() {
    let (poses, fixed, mates, shaft_axis, head_plane) =
        bolt_case(pose([5.0, 5.0, 5.0], [1.0, 0.0, 0.0], 0.5));

    let res = solve_mates(&poses, &fixed, &mates);
    assert_eq!(res.status, SolveStatus::Converged);
    assert_eq!(res.iterations, 0, "expected closed-form seat");
    assert!(res.residual_norm < 1e-12, "residual {}", res.residual_norm);

    let Feature::Plane { point, normal } = world(head_plane, &res.transforms[1]) else {
        unreachable!()
    };
    assert!(point.z.abs() < 1e-9, "seated z {}", point.z);
    assert!((normal + v(0.0, 0.0, 1.0)).norm() < 1e-9, "normal {normal}");

    let Feature::Axis {
        point: ap,
        direction: ad,
    } = world(shaft_axis, &res.transforms[1])
    else {
        unreachable!()
    };
    assert!(ad.cross(&v(0.0, 0.0, 1.0)).norm() < 1e-9);
    assert!(
        (ap.x - 1.0).abs() < 1e-9 && (ap.y - 1.0).abs() < 1e-9,
        "axis {ap}"
    );

    // Bolt still free to spin about its axis: exactly 1 free DOF.
    assert_eq!(res.free_dof, 1);
    assert!(res.is_under_constrained());
}

#[test]
fn seat_closed_form_matches_iterative_solver() {
    let (poses, fixed, mates, _, head_plane) =
        bolt_case(pose([3.0, -2.0, 6.0], [0.0, 1.0, 0.0], 0.3));
    let seated = seat_concentric_coincident(&poses, &fixed, &mates).expect("pattern should match");

    // A near-start LM solve of the same problem lands on the same seat plane.
    let (poses2, fixed2, mates2, _, _) = bolt_case(pose([1.2, 0.8, 0.1], [1.0, 0.0, 0.0], 3.0));
    let res2 = solve_mates(&poses2, &fixed2, &mates2);
    assert_eq!(res2.status, SolveStatus::Converged);

    let Feature::Plane {
        point: p1,
        normal: n1,
    } = world(head_plane, &seated)
    else {
        unreachable!()
    };
    assert!(p1.z.abs() < 1e-9 && (n1 + v(0.0, 0.0, 1.0)).norm() < 1e-9);
    let Feature::Plane {
        point: p2,
        normal: n2,
    } = world(head_plane, &res2.transforms[1])
    else {
        unreachable!()
    };
    assert!(p2.z.abs() < 1e-6 && (n2 + v(0.0, 0.0, 1.0)).norm() < 1e-6);
}

// ---------------------------------------------------------------------------
// DOF accounting & conflict handling
// ---------------------------------------------------------------------------

#[test]
fn no_mates_leaves_all_dof_free() {
    let poses = [
        Transform3::identity(),
        pose([1.0, 0.0, 0.0], [0.0, 0.0, 1.0], 0.2),
        Transform3::identity(),
    ];
    let fixed = [true, false, false];
    let mates: [Mate; 0] = [];

    let res = solve_mates(&poses, &fixed, &mates);
    assert_eq!(res.status, SolveStatus::Converged);
    assert_eq!(res.free_dof, 12); // two floating instances, 6 DOF each
    assert!(res.is_under_constrained());
}

#[test]
fn conflicting_distance_mates_report_over_constrained_without_panic() {
    // The same floating plane is asked to sit 1 above ground (z=0) and 1 above
    // g2 (z=10) — i.e. z=1 and z=9. Cannot satisfy both.
    let poses = [
        Transform3::identity(),
        Transform3::from_parts(
            Translation3::new(0.0, 0.0, 10.0),
            UnitQuaternion::identity(),
        ),
        pose([0.0, 0.0, 3.0], [1.0, 0.0, 0.0], 0.1),
    ];
    let fixed = [true, true, false];
    let gp = Feature::plane(pt(0.0, 0.0, 0.0), v(0.0, 0.0, 1.0)).unwrap();
    let pp = Feature::plane(pt(0.0, 0.0, 0.0), v(0.0, 0.0, 1.0)).unwrap();
    let mates = [
        Mate::distance(FeatureRef::new(0, gp), FeatureRef::new(2, pp), -1.0).unwrap(),
        Mate::distance(FeatureRef::new(1, gp), FeatureRef::new(2, pp), -1.0).unwrap(),
    ];

    let res = solve_mates(&poses, &fixed, &mates);
    assert_eq!(res.status, SolveStatus::OverConstrained);
    assert!(res.is_over_constrained());
    assert!(res.residual_norm > 1e-3, "should retain conflict residual");
    let Feature::Plane { point, .. } = world(pp, &res.transforms[2]) else {
        unreachable!()
    };
    assert!(point.z > 1.0 && point.z < 9.0, "compromise z {}", point.z);
}

#[test]
fn redundant_consistent_mate_still_converges() {
    // Two coincident mates demanding the same thing — redundant but consistent.
    let poses = [
        Transform3::identity(),
        pose([1.0, 1.0, 3.0], [1.0, 0.2, 0.0], 2.9),
    ];
    let fixed = [true, false];
    let a = Feature::plane(pt(0.0, 0.0, 0.0), v(0.0, 0.0, 1.0)).unwrap();
    let b = Feature::plane(pt(0.0, 0.0, 0.0), v(0.0, 0.0, 1.0)).unwrap();
    let mates = [
        Mate::coincident(FeatureRef::new(0, a), FeatureRef::new(1, b)).unwrap(),
        Mate::coincident(FeatureRef::new(0, a), FeatureRef::new(1, b)).unwrap(),
    ];

    let res = solve_mates(&poses, &fixed, &mates);
    assert_eq!(
        res.status,
        SolveStatus::Converged,
        "residual {}",
        res.residual_norm
    );
    assert!(res.residual_norm < 1e-8);
}

#[test]
fn all_fixed_reports_residual_only() {
    let poses = [
        Transform3::identity(),
        Transform3::from_parts(Translation3::new(0.0, 0.0, 4.0), UnitQuaternion::identity()),
    ];
    let fixed = [true, true];
    let a = Feature::point(pt(0.0, 0.0, 0.0));
    let b = Feature::point(pt(0.0, 0.0, 0.0));
    // Demand 4 apart; they are exactly 4 apart ⇒ satisfied, nothing to move.
    let mates = [Mate::distance(FeatureRef::new(0, a), FeatureRef::new(1, b), 4.0).unwrap()];

    let res = solve_mates(&poses, &fixed, &mates);
    assert_eq!(res.status, SolveStatus::Converged);
    assert_eq!(res.free_dof, 0);
    assert_eq!(res.iterations, 0);
}

// ---------------------------------------------------------------------------
// Multi-instance chains
// ---------------------------------------------------------------------------

#[test]
fn two_floating_instances_chain_solve() {
    // ground — coincident — partA — coincident — partB, a stacked chain with
    // outward-facing normals so the parts seat by small translations + tilts
    // from a roughly-placed start — the regime the GUI drops instances in.
    let poses = [
        Transform3::identity(),
        pose([0.1, 0.1, 0.3], [1.0, 0.2, 0.0], 0.3),
        pose([0.2, -0.1, 1.4], [0.0, 1.0, 0.1], 0.3),
    ];
    let fixed = [true, false, false];

    let g_top = Feature::plane(pt(0.0, 0.0, 0.0), v(0.0, 0.0, 1.0)).unwrap();
    let a_bot = Feature::plane(pt(0.0, 0.0, 0.0), v(0.0, 0.0, -1.0)).unwrap();
    let a_top = Feature::plane(pt(0.0, 0.0, 1.0), v(0.0, 0.0, 1.0)).unwrap();
    let b_bot = Feature::plane(pt(0.0, 0.0, 0.0), v(0.0, 0.0, -1.0)).unwrap();

    let mates = [
        Mate::coincident(FeatureRef::new(0, g_top), FeatureRef::new(1, a_bot)).unwrap(),
        Mate::coincident(FeatureRef::new(1, a_top), FeatureRef::new(2, b_bot)).unwrap(),
    ];

    let res = solve_mates(&poses, &fixed, &mates);
    assert_eq!(
        res.status,
        SolveStatus::Converged,
        "residual {}",
        res.residual_norm
    );
    assert!(res.residual_norm < 1e-7);

    let Feature::Plane { point: pa, .. } = world(a_bot, &res.transforms[1]) else {
        unreachable!()
    };
    assert!(pa.z.abs() < 1e-6, "A bottom z {}", pa.z);
    let Feature::Plane { point: pat, .. } = world(a_top, &res.transforms[1]) else {
        unreachable!()
    };
    let Feature::Plane { point: pbb, .. } = world(b_bot, &res.transforms[2]) else {
        unreachable!()
    };
    assert!(
        (pat.z - pbb.z).abs() < 1e-6,
        "A-top {} vs B-bottom {}",
        pat.z,
        pbb.z
    );
}

// ---------------------------------------------------------------------------
// Constructor validation
// ---------------------------------------------------------------------------

#[test]
fn degenerate_direction_is_rejected() {
    assert!(Feature::plane(pt(0.0, 0.0, 0.0), v(0.0, 0.0, 0.0)).is_err());
    assert!(Feature::axis(pt(0.0, 0.0, 0.0), v(0.0, 0.0, 0.0)).is_err());
}

#[test]
fn mate_feature_mismatch_is_rejected() {
    let plane = Feature::plane(pt(0.0, 0.0, 0.0), v(0.0, 0.0, 1.0)).unwrap();
    let point = Feature::point(pt(0.0, 0.0, 0.0));
    assert!(Mate::concentric(FeatureRef::new(0, plane), FeatureRef::new(1, point)).is_err());
    assert!(Mate::distance(FeatureRef::new(0, plane), FeatureRef::new(1, point), 1.0).is_err());
}

#[test]
fn mate_is_valid_checks_range_and_pairing() {
    let plane = Feature::plane(pt(0.0, 0.0, 0.0), v(0.0, 0.0, 1.0)).unwrap();
    let ok = Mate::coincident(FeatureRef::new(0, plane), FeatureRef::new(1, plane)).unwrap();
    assert!(ok.is_valid(2));
    assert!(!ok.is_valid(1)); // instance index 1 out of range for 1 instance
}

#[test]
fn coincident_point_on_plane() {
    let poses = [
        Transform3::identity(),
        pose([2.0, 2.0, 5.0], [0.0, 0.0, 1.0], 0.4),
    ];
    let fixed = [true, false];
    // A point on the part must lie on ground's z=0 plane.
    let plane = Feature::plane(pt(0.0, 0.0, 0.0), v(0.0, 0.0, 1.0)).unwrap();
    let point = Feature::point(pt(0.0, 0.0, 0.0));
    let mates = [Mate::coincident(FeatureRef::new(1, point), FeatureRef::new(0, plane)).unwrap()];

    let res = solve_mates(&poses, &fixed, &mates);
    assert_eq!(
        res.status,
        SolveStatus::Converged,
        "residual {}",
        res.residual_norm
    );
    let Feature::Point { point: wp } = world(point, &res.transforms[1]) else {
        unreachable!()
    };
    assert!(wp.z.abs() < 1e-7, "point on plane z {}", wp.z);
    // Point-on-plane removes only 1 DOF ⇒ 5 remain.
    assert_eq!(res.free_dof, 5);
}

// ---------------------------------------------------------------------------
// Assembly wrapper — solve over real part instances
// ---------------------------------------------------------------------------

#[test]
fn assembly_solve_in_place_moves_real_instances() {
    use opensolid_frep::Shape;
    use opensolid_frep::primitives::Sphere;
    use opensolid_kernel::AssemblyPart;
    use opensolid_kernel::assembly::{Assembly, Instance};
    use std::sync::Arc;

    // A unit sphere part (only its transform/pose matters to the solver).
    let shape = Shape::new(Sphere {
        center: pt(0.0, 0.0, 0.0),
        radius: 1.0,
    });
    let bounds = BoundingBox3::new(pt(-1.0, -1.0, -1.0), pt(1.0, 1.0, 1.0));
    let part = Arc::new(AssemblyPart::from_sdf(shape, bounds, 24).unwrap());

    let mut asm = Assembly::new();
    let anchor = asm.insert(Instance::new(Arc::clone(&part), Transform3::identity()).fixed(true));
    let mover = asm.insert(Instance::new(
        Arc::clone(&part),
        pose([5.0, 0.0, 0.0], [0.0, 0.0, 1.0], 0.0),
    ));

    // Constrain the two part centers to sit 2 apart.
    let a = Feature::point(pt(0.0, 0.0, 0.0));
    let b = Feature::point(pt(0.0, 0.0, 0.0));
    asm.add_mate(
        Mate::distance(FeatureRef::new(anchor, a), FeatureRef::new(mover, b), 2.0).unwrap(),
    );

    let res = asm.solve_in_place();
    assert_eq!(
        res.status,
        SolveStatus::Converged,
        "residual {}",
        res.residual_norm
    );

    // The mover's instance transform was updated in place.
    let placed = asm.instances()[mover].transform;
    let center = placed * pt(0.0, 0.0, 0.0);
    assert!(
        (center.coords.norm() - 2.0).abs() < 1e-6,
        "separation {}",
        center.coords.norm()
    );
    assert_eq!(asm.mates().len(), 1);
}

// Reference MateKind so the import is always used.
const _: MateKind = MateKind::Coincident;
