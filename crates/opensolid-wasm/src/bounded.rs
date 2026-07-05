//! Binding-layer core: a [`Shape`] paired with a tracked bounding box, plus
//! the flat-buffer mesh conversion used by the WASM API.
//!
//! Everything here is plain Rust (no wasm-bindgen types) so the logic is
//! fully exercised by native `cargo test`; the `lib.rs` wasm layer is a thin
//! delegating wrapper.

use opensolid_core::error::CoreResult;
use opensolid_core::mesh::TriangleMesh;
use opensolid_core::types::{BoundingBox3, Point3, Transform3, Vector3};
use opensolid_frep::mesh::{MeshOptions, mesh_sdf_indexed};
use opensolid_frep::primitives::{Box3, Capsule, Cylinder, RoundedBox, Sphere, Torus};
use opensolid_frep::{Extrude, Profile2D, Revolve, SdfTransformExt, Shape};

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

    /// Box with rounded edges. `hx`/`hy`/`hz` are the outer half-extents
    /// including the rounding, so the box occupies the same volume as
    /// [`Self::box3`] with the same half-extents; `radius` must not exceed
    /// the smallest half-extent (matches [`RoundedBox`]).
    pub fn rounded_box(hx: f64, hy: f64, hz: f64, radius: f64) -> Self {
        Self {
            shape: Shape::new(RoundedBox {
                center: Point3::origin(),
                half_extents: [hx, hy, hz],
                radius,
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

    /// The profile swept along +Y over `y ∈ [0, height]`; profile `(u, v)`
    /// maps to world `(x, z)`.
    ///
    /// # Errors
    /// Propagates [`Extrude::new`] validation (`height > 0` and finite).
    pub fn extrude(profile: Profile2D, height: f64) -> CoreResult<Self> {
        let (min, max) = profile.bounds();
        let bounds = BoundingBox3::new(
            Point3::new(min[0], 0.0, min[1]),
            Point3::new(max[0], height, max[1]),
        );
        Ok(Self {
            shape: Shape::new(Extrude::new(profile, height)?),
            bounds,
        })
    }

    /// The profile revolved around the Y axis through `angle` radians,
    /// sweeping from the +X half-plane towards +Z; profile `(u, v)` maps to
    /// `(radius, y)`. The tracked box is the full-turn box (conservative
    /// for partial sweeps).
    ///
    /// # Errors
    /// Propagates [`Revolve::new`] validation (`angle` in `(0, 2π]`,
    /// profile in `u >= 0`).
    pub fn revolve(profile: Profile2D, angle: f64) -> CoreResult<Self> {
        let (min, max) = profile.bounds();
        let reach = max[0].max(0.0);
        let bounds = BoundingBox3::new(
            Point3::new(-reach, min[1], -reach),
            Point3::new(reach, max[1], reach),
        );
        Ok(Self {
            shape: Shape::new(Revolve::new(profile, angle)?),
            bounds,
        })
    }

    pub fn translate(&self, offset: Vector3) -> Self {
        Self {
            shape: Shape::new(self.shape.clone().translated(offset)),
            bounds: BoundingBox3::new(self.bounds.min + offset, self.bounds.max + offset),
        }
    }

    /// Rotated about the origin. `axis_angle`'s direction is the rotation
    /// axis and its norm the angle in radians. The tracked box is the AABB
    /// of the rotated corners (conservative).
    pub fn rotate(&self, axis_angle: Vector3) -> Self {
        let rot = Transform3::rotation(axis_angle);
        let b = &self.bounds;
        let corners = (0..8).map(|i| {
            rot * Point3::new(
                if i & 1 == 0 { b.min.x } else { b.max.x },
                if i & 2 == 0 { b.min.y } else { b.max.y },
                if i & 4 == 0 { b.min.z } else { b.max.z },
            )
        });
        Self {
            shape: Shape::new(self.shape.clone().rotated(axis_angle)),
            bounds: BoundingBox3::from_points(corners),
        }
    }

    /// Scaled per-axis about the origin (each factor `> 0`). Sign-exact
    /// but not metric-exact — see [`opensolid_frep::AnisotropicScale`].
    ///
    /// # Errors
    /// Propagates factor validation (positive and finite).
    pub fn scale(&self, factors: Vector3) -> CoreResult<Self> {
        let shape = Shape::new(self.shape.clone().scaled_anisotropic(factors)?);
        Ok(Self {
            shape,
            bounds: BoundingBox3::new(
                Point3::new(
                    self.bounds.min.x * factors.x,
                    self.bounds.min.y * factors.y,
                    self.bounds.min.z * factors.z,
                ),
                Point3::new(
                    self.bounds.max.x * factors.x,
                    self.bounds.max.y * factors.y,
                    self.bounds.max.z * factors.z,
                ),
            ),
        })
    }

    /// Scaled uniformly about the origin (`factor > 0`); stays an exact
    /// distance field.
    ///
    /// # Errors
    /// Propagates factor validation (positive and finite).
    pub fn uniform_scale(&self, factor: f64) -> CoreResult<Self> {
        let shape = Shape::new(self.shape.clone().scaled(factor)?);
        Ok(Self {
            shape,
            bounds: BoundingBox3::new(
                Point3::from(self.bounds.min.coords * factor),
                Point3::from(self.bounds.max.coords * factor),
            ),
        })
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
        let mesh = s.mesh(RES, None);
        assert!(mesh.is_closed_manifold(), "auto-bounds mesh not manifold");
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
        assert_meshes_cleanly(&BoundedShape::rounded_box(1.0, 0.5, 0.75, 0.2));
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

        // Rounded box surface reaches the face centers of its tracked box.
        let r = BoundedShape::rounded_box(1.0, 0.5, 0.75, 0.2);
        assert!(r.shape.eval(&r.bounds.max) > 0.0);
        assert!(r.shape.eval(&Point3::new(1.0, 0.0, 0.0)).abs() < 1e-12);
        assert!(r.shape.eval(&Point3::new(0.0, 0.5, 0.0)).abs() < 1e-12);
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
        assert_meshes_cleanly(&d);
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

    #[test]
    fn rotate_quarter_turn_swaps_bounds_axes() {
        // Box reaching x = ±2 rotated 90° about z reaches y = ±2.
        use std::f64::consts::FRAC_PI_2;
        let s = BoundedShape::box3(2.0, 1.0, 0.5).rotate(Vector3::new(0.0, 0.0, FRAC_PI_2));
        assert!((s.bounds.max - Point3::new(1.0, 2.0, 0.5)).norm() < 1e-12);
        assert!((s.bounds.min - Point3::new(-1.0, -2.0, -0.5)).norm() < 1e-12);
        assert!(s.shape.eval(&Point3::new(0.0, 2.0, 0.0)).abs() < 1e-12);
        assert_meshes_cleanly(&s);
    }

    #[test]
    fn rotate_oblique_bounds_still_contain_surface() {
        let s = BoundedShape::box3(1.0, 0.5, 0.75)
            .rotate(Vector3::new(0.4, -0.8, 0.3))
            .translate(Vector3::new(0.5, -0.25, 1.0));
        assert_meshes_cleanly(&s);
    }

    #[test]
    fn uniform_scale_scales_bounds_and_surface() {
        let s = BoundedShape::sphere(1.0)
            .translate(Vector3::new(1.0, 0.0, 0.0))
            .uniform_scale(2.0)
            .expect("valid factor");
        assert_eq!(s.bounds.min, Point3::new(0.0, -2.0, -2.0));
        assert_eq!(s.bounds.max, Point3::new(4.0, 2.0, 2.0));
        assert!(s.shape.eval(&Point3::new(4.0, 0.0, 0.0)).abs() < 1e-12);
        assert_meshes_cleanly(&s);
    }

    #[test]
    fn anisotropic_scale_bounds_and_sign() {
        let s = BoundedShape::sphere(1.0)
            .scale(Vector3::new(2.0, 1.0, 0.5))
            .expect("valid factors");
        assert_eq!(s.bounds.min, Point3::new(-2.0, -1.0, -0.5));
        assert_eq!(s.bounds.max, Point3::new(2.0, 1.0, 0.5));
        assert!(s.shape.eval(&Point3::new(2.0, 0.0, 0.0)).abs() < 1e-12);
        assert!(s.shape.eval(&Point3::new(0.0, 0.0, 0.6)) > 0.0);
        let mesh = s.mesh(RES, None);
        assert!(!mesh.is_empty());
        assert!(mesh.is_closed_manifold());
    }

    #[test]
    fn scale_rejects_bad_factors() {
        let s = BoundedShape::sphere(1.0);
        assert!(s.scale(Vector3::new(0.0, 1.0, 1.0)).is_err());
        assert!(s.scale(Vector3::new(1.0, -1.0, 1.0)).is_err());
        assert!(s.uniform_scale(0.0).is_err());
        assert!(s.uniform_scale(f64::NAN).is_err());
    }

    fn l_profile() -> Profile2D {
        Profile2D::new(
            vec![
                [0.0, 0.0],
                [1.0, 0.0],
                [1.0, 0.4],
                [0.4, 0.4],
                [0.4, 1.0],
                [0.0, 1.0],
            ],
            vec![0.0; 6],
        )
        .expect("valid L profile")
    }

    #[test]
    fn extrude_tracks_bounds_and_meshes() {
        let s = BoundedShape::extrude(l_profile(), 0.8).expect("valid extrude");
        assert_eq!(s.bounds.min, Point3::new(0.0, 0.0, 0.0));
        assert_eq!(s.bounds.max, Point3::new(1.0, 0.8, 1.0));
        // Surface touches the tracked box faces.
        assert!(s.shape.eval(&Point3::new(0.5, 0.8, 0.2)).abs() < 1e-12);
        assert!(s.shape.eval(&Point3::new(1.0, 0.4, 0.2)).abs() < 1e-12);
        assert_meshes_cleanly(&s);
    }

    #[test]
    fn revolve_tracks_bounds_and_meshes() {
        let profile = Profile2D::new(
            vec![[0.3, -0.2], [0.8, -0.2], [0.8, 0.2], [0.3, 0.2]],
            vec![0.0; 4],
        )
        .expect("valid profile");
        let s =
            BoundedShape::revolve(profile.clone(), std::f64::consts::TAU).expect("valid revolve");
        assert_eq!(s.bounds.min, Point3::new(-0.8, -0.2, -0.8));
        assert_eq!(s.bounds.max, Point3::new(0.8, 0.2, 0.8));
        assert!(s.shape.eval(&Point3::new(0.0, 0.0, 0.8)).abs() < 1e-12);
        assert_meshes_cleanly(&s);

        // Partial sweep: still contained in the (conservative) full box.
        let partial = BoundedShape::revolve(profile, 2.0).expect("valid revolve");
        assert_meshes_cleanly(&partial);
    }

    #[test]
    fn extrude_and_revolve_reject_bad_input() {
        assert!(BoundedShape::extrude(l_profile(), 0.0).is_err());
        assert!(BoundedShape::extrude(l_profile(), -1.0).is_err());
        assert!(BoundedShape::revolve(l_profile(), 0.0).is_err());
        assert!(BoundedShape::revolve(l_profile(), 7.0).is_err());
        // Profile crossing to negative u cannot be revolved.
        let crossing = Profile2D::new(
            vec![[-0.5, 0.0], [0.5, 0.0], [0.5, 1.0], [-0.5, 1.0]],
            vec![0.0; 4],
        )
        .expect("valid profile");
        assert!(BoundedShape::revolve(crossing, std::f64::consts::TAU).is_err());
    }

    #[test]
    fn swept_solids_compose_with_csg_and_transforms() {
        // A rotated, scaled extrusion subtracted from a revolve: the whole
        // pipeline stays composable and meshable.
        let ring = BoundedShape::revolve(
            Profile2D::new(
                vec![[0.5, -0.15], [0.9, -0.15], [0.9, 0.15], [0.5, 0.15]],
                vec![0.0; 4],
            )
            .expect("valid profile"),
            std::f64::consts::TAU,
        )
        .expect("valid revolve");
        let bar = BoundedShape::extrude(l_profile(), 0.5)
            .expect("valid extrude")
            .rotate(Vector3::new(0.0, 0.3, 0.0))
            .translate(Vector3::new(-0.5, -0.25, -0.5));
        let part = ring.subtract(&bar);
        let mesh = part.mesh(RES, None);
        assert!(!mesh.is_empty());
        assert!(mesh.is_closed_manifold());
    }
}
