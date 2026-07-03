//! Build a demo part with F-Rep booleans, mesh it, and export STL + OBJ.
//!
//! Run from the repo root:
//!
//! ```sh
//! cargo run -p opensolid-kernel --example demo
//! ```

use std::fs::File;
use std::io::BufWriter;

use opensolid_kernel::core::types::{BoundingBox3, Point3};
use opensolid_kernel::frep::Shape;
use opensolid_kernel::frep::primitives::{Box3, Cylinder, Sphere};
use opensolid_kernel::{MeshOptions, mesh_sdf_indexed, write_obj, write_stl_binary};

fn main() -> std::io::Result<()> {
    // A box, organically blended with a sphere on its side, then a
    // cylindrical hole drilled vertically through the box. The sphere stays
    // clear of the hole so the drill exits through flat faces only — sharp
    // subtraction across a grazing curved surface leaves slivers thinner
    // than a grid cell, which dual contouring cannot mesh watertight.
    let body = Shape::new(Box3 {
        center: Point3::origin(),
        half_extents: [1.0, 1.0, 1.0],
    });
    let bump = Shape::new(Sphere {
        center: Point3::new(1.3, 0.0, 0.0),
        radius: 0.6,
    });
    let hole = Shape::new(Cylinder {
        center: Point3::origin(),
        radius: 0.45,
        half_height: 2.0,
    });
    let part = body.smooth_union(bump, 0.3).subtract(hole);

    // Mesh on a uniform grid; the bounds must strictly contain the surface.
    let opts = MeshOptions {
        bounds: BoundingBox3::new(Point3::new(-2.5, -2.5, -2.5), Point3::new(2.5, 2.5, 2.5)),
        resolution: 96,
    };
    let mesh = mesh_sdf_indexed(&part, &opts);
    assert!(mesh.is_closed_manifold(), "demo mesh must be watertight");

    write_stl_binary(&mesh, &mut BufWriter::new(File::create("demo.stl")?))?;
    write_obj(&mesh, &mut BufWriter::new(File::create("demo.obj")?))?;

    println!(
        "Meshed {} triangles ({} vertices), closed and manifold.",
        mesh.triangle_count(),
        mesh.vertex_count()
    );
    println!("Wrote demo.stl and demo.obj to the current directory.");
    println!("View them by dragging demo.stl into an online viewer, e.g. https://www.viewstl.com");
    Ok(())
}
