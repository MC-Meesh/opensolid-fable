//! Regression for of-o0o: the hinge-leaf part meshed watertight across a
//! sweep of bore radii. Before the fold-flap repair, near-tangent bore
//! silhouettes (where the subtracted cylinder grazes horizontal) produced a
//! handful of non-manifold four-triangle edges — single-sheet folds the
//! two-sheet pinch repair could not split — so `validate` reported
//! `valid:false` / `volume:null` at knife-edge radii (r=1.6 shipped-gallery
//! example dodged this by using r=2).

use opensolid_core::types::Vector3;
use opensolid_wasm::bounded::BoundedShape;

fn rz(s: &BoundedShape) -> BoundedShape {
    s.rotate(Vector3::new(0.0, 0.0, 90.0))
}

/// The `examples/agent-gallery/hinge-leaf` part, parameterized by bore radius.
fn leaf(r: f64) -> BoundedShape {
    let plate = BoundedShape::box3(30.0, 15.0, 0.75).translate(Vector3::new(0.0, -15.75, 0.0));
    let mut s = plate;
    for x in [-24.0, 0.0, 24.0] {
        let knuckle = rz(&BoundedShape::cylinder(4.0, 6.0)).translate(Vector3::new(x, 0.0, 0.0));
        s = s.union(&knuckle);
    }
    s.subtract(&rz(&BoundedShape::cylinder(r, 40.0)))
}

#[test]
fn hinge_leaf_is_watertight_across_bore_radii() {
    for r in [1.2, 1.4, 1.6, 1.8, 2.0, 2.5, 3.0, 3.5] {
        let mesh = leaf(r).mesh_adaptive(f64::NAN, None);
        let defects = mesh.manifold_defects();
        assert!(
            defects.is_closed(),
            "r={r}: mesh is not a closed manifold — {defects:?}"
        );
    }
}
