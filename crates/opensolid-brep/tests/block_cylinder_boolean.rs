//! Regression tests for of-ipt.4: block×cylinder booleans must produce
//! geometrically correct results, not just combinatorially valid ones.
//!
//! The canonical through-hole config (slab 4×4×2 minus r=1 cylinder along
//! z) used to pass `check()` and tessellate to a closed manifold while the
//! actual geometry was silently wrong: the bottom-face hole was never cut,
//! the cylinder band was only ~30% covered, and the removed volume was off
//! by up to 12×. These tests measure the mesh, not just the topology.

use opensolid_brep::GeometryStore;
use opensolid_brep::boolean::{BooleanOutput, intersect, subtract, unite};
use opensolid_brep::primitives::{block, cylinder};
use opensolid_brep::topology::TopologyStore;
use opensolid_core::mesh::TriangleMesh;
use opensolid_core::tolerance::ToleranceContext;
use opensolid_core::types::{Point3, Vector3};
use std::f64::consts::PI;

/// The tessellated cylinder wall is a 96-gon prism (SAMPLES_PER_CIRCLE),
/// so circular cross sections lose `1 - sin(2π/n)/(2π/n)` ≈ 7.2e-4 of
/// their area. 0.5% relative tolerance absorbs that plus triangulation
/// noise while still catching real classification errors.
const CYL_RTOL: f64 = 5e-3;

fn tol() -> ToleranceContext {
    ToleranceContext::default()
}

/// check() must be clean and the tessellation closed-manifold; returns the
/// mesh for measurement.
fn assert_valid(out: &BooleanOutput, context: &str) -> TriangleMesh {
    let failures = out.check();
    assert!(
        failures.is_empty(),
        "{context}: check() reported {} failures: {failures:#?}",
        failures.len()
    );
    let mesh = out
        .tessellate()
        .unwrap_or_else(|e| panic!("{context}: tessellation failed: {e:?}"));
    assert!(
        mesh.is_closed_manifold(),
        "{context}: tessellation is not a closed manifold ({} triangles)",
        mesh.triangle_count()
    );
    mesh
}

/// Signed volume of a closed mesh via the divergence theorem.
fn mesh_volume(mesh: &TriangleMesh) -> f64 {
    mesh.to_triangles()
        .iter()
        .map(|t| {
            let a = t.positions[0].coords;
            let b = t.positions[1].coords;
            let c = t.positions[2].coords;
            a.dot(&b.cross(&c)) / 6.0
        })
        .sum()
}

/// Total triangle area lying in the plane `z = plane_z` (within 1e-9),
/// i.e. the tessellated area of that planar face.
fn planar_face_area(mesh: &TriangleMesh, plane_z: f64) -> f64 {
    mesh.to_triangles()
        .iter()
        .filter(|t| t.positions.iter().all(|v| (v.z - plane_z).abs() < 1e-9))
        .map(|t| {
            let ab = t.positions[1] - t.positions[0];
            let ac = t.positions[2] - t.positions[0];
            ab.cross(&ac).norm() / 2.0
        })
        .sum()
}

fn assert_close(got: f64, want: f64, rtol: f64, context: &str) {
    let scale = want.abs().max(1e-300);
    assert!(
        ((got - want) / scale).abs() <= rtol,
        "{context}: got {got}, expected {want} \
         (off by {:.3e} relative, allowed {rtol:.1e})",
        ((got - want) / scale).abs()
    );
}

/// Slab 4×4×2 (z ∈ [-1, 1]) and a r=1 cylinder along z piercing both caps
/// (z ∈ [-2, 2]), both centered at the origin — the canonical through-hole
/// configuration from the bug report (translated to the origin).
fn slab_and_tool() -> (
    TopologyStore,
    GeometryStore,
    opensolid_core::EntityId<opensolid_brep::Body>,
    opensolid_core::EntityId<opensolid_brep::Body>,
) {
    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let slab = block(&mut store, &mut geo, 4.0, 4.0, 2.0).expect("valid slab");
    let tool = cylinder(&mut store, &mut geo, 1.0, 4.0).expect("valid tool");
    (store, geo, slab, tool)
}

#[test]
fn subtract_through_hole_volume_and_faces() {
    let (store, geo, slab, tool) = slab_and_tool();
    let out = subtract(&store, &geo, slab, tool, &tol()).expect("subtract");
    let mesh = assert_valid(&out, "slab − cylinder");

    // Both caps must have the hole cut: area 16 − π each.
    let top = planar_face_area(&mesh, 1.0);
    let bottom = planar_face_area(&mesh, -1.0);
    assert_close(top, 16.0 - PI, CYL_RTOL, "top face area");
    assert_close(bottom, 16.0 - PI, CYL_RTOL, "bottom face area");

    let vol = mesh_volume(&mesh);
    assert_close(vol, 32.0 - 2.0 * PI, CYL_RTOL, "through-hole volume");
}

#[test]
fn intersect_block_cylinder_volume() {
    let (store, geo, slab, tool) = slab_and_tool();
    let out = intersect(&store, &geo, slab, tool, &tol()).expect("intersect");
    let mesh = assert_valid(&out, "slab ∩ cylinder");
    // The intersection is the r=1 cylinder clipped to z ∈ [-1, 1]: vol 2π.
    assert_close(
        mesh_volume(&mesh),
        2.0 * PI,
        CYL_RTOL,
        "intersection volume",
    );
}

#[test]
fn unite_block_cylinder_volume() {
    let (store, geo, slab, tool) = slab_and_tool();
    let out = unite(&store, &geo, slab, tool, &tol()).expect("unite");
    let mesh = assert_valid(&out, "slab ∪ cylinder");
    // vol(A) + vol(B) − vol(A∩B) = 32 + 4π − 2π.
    assert_close(
        mesh_volume(&mesh),
        32.0 + 2.0 * PI,
        CYL_RTOL,
        "union volume",
    );
}

#[test]
fn inclusion_exclusion_identity_block_cylinder() {
    let (store, geo, slab, tool) = slab_and_tool();
    let vol = |out: &BooleanOutput, ctx: &str| mesh_volume(&assert_valid(out, ctx));
    let v_union = vol(
        &unite(&store, &geo, slab, tool, &tol()).expect("unite"),
        "union",
    );
    let v_inter = vol(
        &intersect(&store, &geo, slab, tool, &tol()).expect("intersect"),
        "intersection",
    );
    // vol(A) + vol(B) = vol(A∪B) + vol(A∩B); B is a 96-gon-tessellated
    // cylinder so compare against the analytic values with CYL_RTOL.
    assert_close(
        v_union + v_inter,
        32.0 + 4.0 * PI,
        CYL_RTOL,
        "inclusion–exclusion identity",
    );
}

/// Same through-hole at extreme scales — congruence-consistent failures at
/// 0.001× and 1000× were part of the original report.
#[test]
fn through_hole_at_scale_extremes() {
    for scale in [1e-3, 1e3] {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let slab =
            block(&mut store, &mut geo, 4.0 * scale, 4.0 * scale, 2.0 * scale).expect("valid slab");
        let tool = cylinder(&mut store, &mut geo, scale, 4.0 * scale).expect("valid tool");
        let out = subtract(&store, &geo, slab, tool, &tol()).expect("subtract");
        let context = format!("through hole at scale {scale}");
        let mesh = assert_valid(&out, &context);
        let expected = (32.0 - 2.0 * PI) * scale * scale * scale;
        assert_close(mesh_volume(&mesh), expected, CYL_RTOL, &context);
    }
}

/// A cube with a smaller through-hole (r = 0.5) — the variant from the
/// report that removed 0.1309 instead of π/2.
#[test]
fn cube_with_small_through_hole() {
    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let cube = block(&mut store, &mut geo, 2.0, 2.0, 2.0).expect("valid cube");
    let tool = cylinder(&mut store, &mut geo, 0.5, 4.0).expect("valid tool");
    let out = subtract(&store, &geo, cube, tool, &tol()).expect("subtract");
    let mesh = assert_valid(&out, "cube − r=0.5 cylinder");
    assert_close(
        mesh_volume(&mesh),
        8.0 - PI * 0.25 * 2.0,
        CYL_RTOL,
        "cube through-hole volume",
    );
}

// Keep unused-import lint honest if Point3/Vector3 end up unused.
#[allow(dead_code)]
fn _types(_: Point3, _: Vector3) {}
