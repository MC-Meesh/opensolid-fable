//! Acceptance (of-ipt.3, part 4): the playground WASM path consumes a
//! hybrid boolean result.
//!
//! The playground renders whatever [`flatten_mesh`] hands it and composes
//! further modeling on [`BoundedShape`] (the wasm-bindgen layer in
//! `lib.rs` is a thin delegating wrapper over both). So "consuming a
//! hybrid result" means two things, both covered here: the hybrid mesh
//! flattens into valid GPU buffers, and the hybrid F-Rep field slots back
//! into a `BoundedShape` for further playground CSG.

use opensolid_kernel::brep::{GeometryStore, TopologyStore, primitives, translate_body};
use opensolid_kernel::builder::shape;
use opensolid_kernel::core::types::Vector3;
use opensolid_kernel::hybrid::{self, HybridBody, HybridOptions, HybridPath};
use opensolid_wasm::bounded::{BoundedShape, FlatMesh, flatten_mesh};

fn assert_valid_gpu_buffers(flat: &FlatMesh) {
    assert!(!flat.positions.is_empty(), "no vertex data");
    assert_eq!(
        flat.positions.len(),
        flat.normals.len(),
        "positions and normals must pair up"
    );
    assert_eq!(flat.positions.len() % 3, 0, "positions must be xyz triples");
    assert_eq!(flat.indices.len() % 3, 0, "indices must form triangles");
    let vertex_count = (flat.positions.len() / 3) as u32;
    assert!(
        flat.indices.iter().all(|&i| i < vertex_count),
        "index out of range"
    );
}

/// A mixed-representation boolean (implicit sphere minus exact B-Rep
/// block) produced by the kernel's hybrid path, consumed exactly the way
/// the playground does.
#[test]
fn hybrid_boolean_result_feeds_the_playground_mesh_path() {
    let ball: HybridBody = shape::sphere(1.0).unwrap().into();
    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let block = primitives::block(&mut store, &mut geo, 2.0, 2.0, 2.0).unwrap();
    translate_body(&mut store, &mut geo, block, Vector3::new(1.0, 1.0, 1.0)).unwrap();

    let out = hybrid::subtract(
        &ball,
        &HybridBody::brep(&store, &geo, block),
        &HybridOptions::default(),
    )
    .unwrap();
    assert!(out.mesh.is_closed_manifold());

    // 1) Direct upload: the hybrid result mesh flattens into the GPU
    //    buffer layout the playground's three.js side consumes.
    assert_valid_gpu_buffers(&flatten_mesh(&out.mesh));

    // 2) Continued modeling: the fallback field plus its bounds is
    //    exactly a BoundedShape, so the playground can keep composing.
    let HybridPath::Frep {
        shape: field,
        bounds,
    } = out.path
    else {
        panic!("mixed representations must take the F-Rep path");
    };
    let carried = BoundedShape {
        shape: field,
        bounds,
    };
    let remeshed = carried.mesh(48, None);
    assert!(
        remeshed.is_closed_manifold(),
        "hybrid field must re-mesh cleanly through the playground path"
    );
    assert_valid_gpu_buffers(&flatten_mesh(&remeshed));

    // ...including further CSG against a playground-native shape.
    let extended =
        carried.union(&BoundedShape::sphere(0.4).translate(Vector3::new(0.0, 0.0, -1.1)));
    let composed = extended.mesh(48, None);
    assert!(
        composed.is_closed_manifold(),
        "hybrid field must compose with playground CSG"
    );
    assert_valid_gpu_buffers(&flatten_mesh(&composed));
}
