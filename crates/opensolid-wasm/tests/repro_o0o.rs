use opensolid_core::types::Vector3;
use opensolid_frep::refine::pinched_edge_count;
use opensolid_wasm::bounded::BoundedShape;

fn rz(s: &BoundedShape) -> BoundedShape {
    s.rotate(Vector3::new(0.0, 0.0, 90.0))
}

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
fn repro() {
    for r in [1.2, 1.4, 1.6, 1.8, 2.0, 2.5, 3.0, 3.5] {
        let part = leaf(r);
        let mesh = part.mesh_adaptive(f64::NAN, None);
        let d = mesh.manifold_defects();
        println!(
            "r={r} tris={} pinched={} manifold={} defects: boundary={} misoriented={} pinched3plus={} degen={}",
            mesh.indices.len(),
            pinched_edge_count(&mesh),
            mesh.is_closed_manifold(),
            d.boundary_edges,
            d.misoriented_edges,
            d.pinched_edges,
            d.degenerate_triangles,
        );
    }
}
