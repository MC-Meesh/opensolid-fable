//! Binding-layer core: a [`Shape`] paired with a tracked bounding box, plus
//! the flat-buffer mesh conversion used by the WASM API.
//!
//! Everything here is plain Rust (no wasm-bindgen types) so the logic is
//! fully exercised by native `cargo test`; the `lib.rs` wasm layer is a thin
//! delegating wrapper.

use opensolid_core::mesh::TriangleMesh;
use opensolid_core::types::{BoundingBox3, Point3, Vector3};
use opensolid_frep::mesh::{MeshOptions, mesh_sdf_indexed};
use opensolid_frep::primitives::{Box3, Capsule, Cylinder, Sphere, Torus};
use opensolid_frep::{SdfTransformExt, Shape};

/// A runtime-composable shape that carries a conservative axis-aligned
/// bounding box of its surface, so meshing can auto-derive grid bounds.
#[derive(Clone)]
pub struct BoundedShape {
    pub shape: Shape,
    pub bounds: BoundingBox3,
}

fn symmetric_bounds(hx: f64, hy: f64, hz: f64) -> BoundingBox3 {
    BoundingBox3::new(Point3::new(-hx, -hy, -hz), Point3::new(hx, hy, hz))
}

fn bounds_intersection(a: &BoundingBox3, b: &BoundingBox3) -> BoundingBox3 {
    // Disjoint boxes would produce min > max; collapse that axis to its
    // midpoint so downstream meshing sees a valid (empty) region.
    let axis = |lo: f64, hi: f64| {
        if lo <= hi {
            (lo, hi)
        } else {
            let mid = 0.5 * (lo + hi);
            (mid, mid)
        }
    };
    let (xl, xh) = axis(a.min.x.max(b.min.x), a.max.x.min(b.max.x));
    let (yl, yh) = axis(a.min.y.max(b.min.y), a.max.y.min(b.max.y));
    let (zl, zh) = axis(a.min.z.max(b.min.z), a.max.z.min(b.max.z));
    BoundingBox3::new(Point3::new(xl, yl, zl), Point3::new(xh, yh, zh))
}

fn max_extent(b: &BoundingBox3) -> f64 {
    let size = b.max - b.min;
    size.x.max(size.y).max(size.z)
}

impl BoundedShape {
    pub fn sphere(radius: f64) -> Self {
        Self {
            shape: Shape::new(Sphere {
                center: Point3::origin(),
                radius,
            }),
            bounds: symmetric_bounds(radius, radius, radius),
        }
    }

    pub fn box3(hx: f64, hy: f64, hz: f64) -> Self {
        Self {
            shape: Shape::new(Box3 {
                center: Point3::origin(),
                half_extents: [hx, hy, hz],
            }),
            bounds: symmetric_bounds(hx, hy, hz),
        }
    }

    /// Cylinder along the y axis (matches [`Cylinder`]'s axial convention).
    pub fn cylinder(radius: f64, half_height: f64) -> Self {
        Self {
            shape: Shape::new(Cylinder {
                center: Point3::origin(),
                radius,
                half_height,
            }),
            bounds: symmetric_bounds(radius, half_height, radius),
        }
    }

    /// Torus with its ring in the xz plane (matches [`Torus`]).
    pub fn torus(major_radius: f64, minor_radius: f64) -> Self {
        let reach = major_radius + minor_radius;
        Self {
            shape: Shape::new(Torus {
                center: Point3::origin(),
                major_radius,
                minor_radius,
            }),
            bounds: symmetric_bounds(reach, minor_radius, reach),
        }
    }

    pub fn capsule(start: Point3, end: Point3, radius: f64) -> Self {
        let min = Point3::new(
            start.x.min(end.x) - radius,
            start.y.min(end.y) - radius,
            start.z.min(end.z) - radius,
        );
        let max = Point3::new(
            start.x.max(end.x) + radius,
            start.y.max(end.y) + radius,
            start.z.max(end.z) + radius,
        );
        Self {
            shape: Shape::new(Capsule { start, end, radius }),
            bounds: BoundingBox3::new(min, max),
        }
    }

    pub fn translate(&self, offset: Vector3) -> Self {
        Self {
            shape: Shape::new(self.shape.clone().translated(offset)),
            bounds: BoundingBox3::new(self.bounds.min + offset, self.bounds.max + offset),
        }
    }

    pub fn union(&self, other: &Self) -> Self {
        Self {
            shape: self.shape.clone().union(other.shape.clone()),
            bounds: self.bounds.union(&other.bounds),
        }
    }

    /// Surface of `a ∩ b` lies inside both operands, so the tracked box is
    /// the intersection of the operand boxes.
    pub fn intersect(&self, other: &Self) -> Self {
        Self {
            shape: self.shape.clone().intersect(other.shape.clone()),
            bounds: bounds_intersection(&self.bounds, &other.bounds),
        }
    }

    /// Surface of `a \ b` lies inside `a` (subtraction only removes material
    /// and exposes faces of `b` that are inside `a`), so `a`'s box is kept.
    pub fn subtract(&self, other: &Self) -> Self {
        Self {
            shape: self.shape.clone().subtract(other.shape.clone()),
            bounds: self.bounds,
        }
    }

    /// Smooth union. Without an explicit `radius` a heuristic of 10% of the
    /// combined box's largest extent is used. The polynomial blend can bulge
    /// at most `radius / 4` beyond the plain union, so the tracked box is
    /// padded by that much.
    pub fn smooth_union(&self, other: &Self, radius: Option<f64>) -> Self {
        let combined = self.bounds.union(&other.bounds);
        let radius = radius.unwrap_or_else(|| 0.1 * max_extent(&combined));
        let pad = Vector3::repeat(0.25 * radius);
        Self {
            shape: self.shape.clone().smooth_union(other.shape.clone(), radius),
            bounds: BoundingBox3::new(combined.min - pad, combined.max + pad),
        }
    }

    /// Mesh the shape on a `resolution`³ grid. With `bound` set, the grid
    /// covers the explicit cube `[-bound, bound]³`; otherwise bounds are
    /// auto-derived from the tracked bounding box (see [`Self::mesh_bounds`]).
    pub fn mesh(&self, resolution: usize, bound: Option<f64>) -> TriangleMesh {
        let bounds = match bound {
            Some(b) => symmetric_bounds(b, b, b),
            None => self.mesh_bounds(resolution),
        };
        mesh_sdf_indexed(&self.shape, &MeshOptions { bounds, resolution })
    }

    /// Auto-derived meshing bounds: the tracked box padded so the surface
    /// stays strictly interior. The mesher does not stitch crossings in the
    /// boundary cell layer, so the pad must exceed one grid cell: at least
    /// 10% of the largest extent, growing on coarse grids where a single
    /// cell is wider than that.
    pub fn mesh_bounds(&self, resolution: usize) -> BoundingBox3 {
        let extent = max_extent(&self.bounds).max(1e-9);
        let frac = if resolution == 0 {
            0.1
        } else {
            (3.0 / resolution as f64).max(0.1)
        };
        let pad = Vector3::repeat(extent * frac);
        BoundingBox3::new(self.bounds.min - pad, self.bounds.max + pad)
    }
}

/// A [`TriangleMesh`] flattened into the GPU-friendly buffers the JS side
/// consumes: xyz-interleaved `f32` positions/normals and `u32` index triples.
pub struct FlatMesh {
    pub positions: Vec<f32>,
    pub normals: Vec<f32>,
    pub indices: Vec<u32>,
}

/// Flatten an indexed mesh, preserving vertex order and triangle winding.
pub fn flatten_mesh(mesh: &TriangleMesh) -> FlatMesh {
    debug_assert_eq!(mesh.positions.len(), mesh.normals.len());
    FlatMesh {
        positions: mesh
            .positions
            .iter()
            .flat_map(|p| [p.x as f32, p.y as f32, p.z as f32])
            .collect(),
        normals: mesh
            .normals
            .iter()
            .flat_map(|n| [n.x as f32, n.y as f32, n.z as f32])
            .collect(),
        indices: mesh
            .indices
            .iter()
            .flatten()
            .map(|&i| u32::try_from(i).expect("vertex index exceeds u32"))
            .collect(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use opensolid_frep::primitives::Sdf;

    const RES: usize = 24;

    /// Meshing with auto bounds must produce a non-empty closed manifold
    /// whose vertices lie on the SDF surface — this fails if the tracked
    /// bounding box ever fails to contain the surface with enough padding.
    fn assert_meshes_cleanly(s: &BoundedShape) -> TriangleMesh {
        let mesh = assert_meshes_on_surface(s);
        assert!(mesh.is_closed_manifold(), "auto-bounds mesh not manifold");
        mesh
    }

    /// Like [`assert_meshes_cleanly`] but without the manifold check, for
    /// shapes with sharp concave creases (e.g. subtractions) where the dual
    /// contouring mesher is known to emit non-manifold edges at moderate
    /// resolutions.
    fn assert_meshes_on_surface(s: &BoundedShape) -> TriangleMesh {
        let mesh = s.mesh(RES, None);
        assert!(!mesh.is_empty(), "auto-bounds mesh is empty");
        let b = s.mesh_bounds(RES);
        let cell = max_extent(&b) / RES as f64;
        for p in &mesh.positions {
            assert!(s.shape.eval(p).abs() < cell, "vertex {p:?} off surface");
            assert!(b.contains(p), "vertex {p:?} outside mesh bounds");
        }
        mesh
    }

    #[test]
    fn primitives_mesh_within_auto_bounds() {
        assert_meshes_cleanly(&BoundedShape::sphere(1.0));
        assert_meshes_cleanly(&BoundedShape::box3(1.0, 0.5, 0.75));
        assert_meshes_cleanly(&BoundedShape::cylinder(0.5, 1.0));
        assert_meshes_cleanly(&BoundedShape::torus(1.0, 0.3));
        assert_meshes_cleanly(&BoundedShape::capsule(
            Point3::new(-0.5, -0.2, 0.1),
            Point3::new(0.5, 0.8, -0.3),
            0.4,
        ));
    }

    #[test]
    fn primitive_bounds_contain_surface_tightly() {
        // The tracked box must touch the shape: SDF at box corners is
        // positive (outside), SDF at face-center of the box is ~0 for
        // shapes whose surface reaches the box.
        let s = BoundedShape::cylinder(0.5, 1.0);
        assert!(s.shape.eval(&s.bounds.max) > 0.0);
        assert!(s.shape.eval(&Point3::new(0.5, 0.0, 0.0)).abs() < 1e-12);
        assert!(s.shape.eval(&Point3::new(0.0, 1.0, 0.0)).abs() < 1e-12);

        let t = BoundedShape::torus(1.0, 0.3);
        assert!(t.shape.eval(&t.bounds.max) > 0.0);
        assert!(t.shape.eval(&Point3::new(1.3, 0.0, 0.0)).abs() < 1e-12);
        assert!(t.shape.eval(&Point3::new(0.0, 0.3, 1.0)).abs() < 1e-12);
    }

    #[test]
    fn translate_shifts_bounds_and_surface() {
        let s = BoundedShape::sphere(1.0).translate(Vector3::new(3.0, -1.0, 2.0));
        assert_eq!(s.bounds.min, Point3::new(2.0, -2.0, 1.0));
        assert_eq!(s.bounds.max, Point3::new(4.0, 0.0, 3.0));
        assert!(s.shape.eval(&Point3::new(4.0, -1.0, 2.0)).abs() < 1e-12);
        let mesh = assert_meshes_cleanly(&s);
        // Every vertex sits near the translated sphere, not the origin.
        for p in &mesh.positions {
            assert!((p - Point3::new(3.0, -1.0, 2.0)).norm() < 1.2);
        }
    }

    #[test]
    fn csg_bounds_combine_correctly() {
        let a = BoundedShape::sphere(1.0);
        let b = BoundedShape::sphere(1.0).translate(Vector3::new(1.0, 0.0, 0.0));

        let u = a.union(&b);
        assert_eq!(u.bounds.min, Point3::new(-1.0, -1.0, -1.0));
        assert_eq!(u.bounds.max, Point3::new(2.0, 1.0, 1.0));
        assert_meshes_cleanly(&u);

        let i = a.intersect(&b);
        assert_eq!(i.bounds.min, Point3::new(0.0, -1.0, -1.0));
        assert_eq!(i.bounds.max, Point3::new(1.0, 1.0, 1.0));
        assert_meshes_cleanly(&i);

        let d = a.subtract(&b);
        assert_eq!(d.bounds.min, a.bounds.min);
        assert_eq!(d.bounds.max, a.bounds.max);
        assert_meshes_on_surface(&d);
    }

    #[test]
    fn disjoint_intersection_is_empty_not_panicking() {
        let a = BoundedShape::sphere(1.0);
        let b = BoundedShape::sphere(1.0).translate(Vector3::new(10.0, 0.0, 0.0));
        let i = a.intersect(&b);
        assert!(i.bounds.min.x <= i.bounds.max.x);
        assert!(i.mesh(16, None).is_empty());
    }

    #[test]
    fn smooth_union_defaults_and_meshes() {
        let a = BoundedShape::sphere(1.0);
        let b = BoundedShape::sphere(1.0).translate(Vector3::new(1.2, 0.0, 0.0));
        let plain = a.union(&b);

        let s = a.smooth_union(&b, Some(0.4));
        // Bounds are the plain union padded by radius/4.
        assert!((s.bounds.min.x - (plain.bounds.min.x - 0.1)).abs() < 1e-12);
        assert!((s.bounds.max.x - (plain.bounds.max.x + 0.1)).abs() < 1e-12);
        assert_meshes_cleanly(&s);

        // Default radius: 10% of the combined box's largest extent (3.2).
        let d = a.smooth_union(&b, None);
        let expected_pad = 0.25 * (0.1 * 3.2);
        assert!((d.bounds.max.x - (plain.bounds.max.x + expected_pad)).abs() < 1e-12);
        assert_meshes_cleanly(&d);
    }

    #[test]
    fn explicit_bound_overrides_auto() {
        let s = BoundedShape::sphere(1.0);
        let mesh = s.mesh(20, Some(1.6));
        assert!(!mesh.is_empty());
        assert!(mesh.is_closed_manifold());
        // A generous explicit bound still works, just with coarser cells.
        assert!(!s.mesh(20, Some(5.0)).is_empty());
    }

    #[test]
    fn zero_resolution_returns_empty_mesh() {
        assert!(BoundedShape::sphere(1.0).mesh(0, None).is_empty());
    }

    #[test]
    fn flatten_round_trips_the_indexed_mesh() {
        let s = BoundedShape::sphere(1.0);
        let mesh = s.mesh(RES, None);
        let flat = flatten_mesh(&mesh);

        assert_eq!(flat.positions.len(), mesh.vertex_count() * 3);
        assert_eq!(flat.normals.len(), mesh.vertex_count() * 3);
        assert_eq!(flat.indices.len(), mesh.triangle_count() * 3);

        // Values survive the f64 → f32 narrowing exactly as casts.
        for (v, p) in mesh.positions.iter().enumerate() {
            assert_eq!(flat.positions[v * 3], p.x as f32);
            assert_eq!(flat.positions[v * 3 + 1], p.y as f32);
            assert_eq!(flat.positions[v * 3 + 2], p.z as f32);
        }
        for (v, n) in mesh.normals.iter().enumerate() {
            assert_eq!(flat.normals[v * 3], n.x as f32);
            assert_eq!(flat.normals[v * 3 + 1], n.y as f32);
            assert_eq!(flat.normals[v * 3 + 2], n.z as f32);
        }
        // Index triples and winding are preserved and in range.
        for (t, tri) in mesh.indices.iter().enumerate() {
            for (c, &corner) in tri.iter().enumerate() {
                let idx = flat.indices[t * 3 + c];
                assert_eq!(idx as usize, corner);
                assert!((idx as usize) < mesh.vertex_count());
            }
        }
    }

    #[test]
    fn flatten_empty_mesh_is_empty() {
        let flat = flatten_mesh(&TriangleMesh::new());
        assert!(flat.positions.is_empty());
        assert!(flat.normals.is_empty());
        assert!(flat.indices.is_empty());
    }
}
