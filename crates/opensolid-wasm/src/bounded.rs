//! Binding-layer core: a [`Shape`] paired with a tracked bounding box, plus
//! the flat-buffer mesh conversion used by the WASM API.
//!
//! Everything here is plain Rust (no wasm-bindgen types) so the logic is
//! fully exercised by native `cargo test`; the `lib.rs` wasm layer is a thin
//! delegating wrapper.

use opensolid_core::error::CoreResult;
use opensolid_core::interval::Interval;
use opensolid_core::mesh::TriangleMesh;
use opensolid_core::types::{BoundingBox3, Point3, Transform3, Vector3};
use opensolid_frep::mesh::{MeshOptions, mesh_sdf_indexed};
use opensolid_frep::mesh_adaptive::{AdaptiveMeshOptions, mesh_sdf_adaptive_indexed};
use opensolid_frep::primitives::{
    Box3, Capsule, Cone, Cylinder, HalfSpace, RoundedBox, Sdf, Sphere, Torus,
};
use opensolid_frep::refine::{RefineOptions, refine_mesh};
use opensolid_frep::{
    BlendMode, BooleanKind, EdgeRegion, Extrude, OpenPath2D, Profile2D, Revolve, Rib, RibSide,
    SdfTransformExt, Shape,
};

/// Depth bounds for accuracy-driven adaptive meshing: the ceiling keeps the
/// finest lattice at 512³ virtual cells so interactive remeshing stays
/// within budget, the floor keeps the error estimate meaningful on a
/// non-trivial base grid.
const ADAPTIVE_MIN_DEPTH: u32 = 4;
const ADAPTIVE_MAX_DEPTH: u32 = 9;

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

    /// Cone/frustum along the y axis (matches [`Cone`]'s axial convention):
    /// `radius_bottom` at `y = -half_height`, `radius_top` at
    /// `y = +half_height`.
    pub fn cone(radius_bottom: f64, radius_top: f64, half_height: f64) -> Self {
        let reach = radius_bottom.max(radius_top);
        Self {
            shape: Shape::new(Cone {
                center: Point3::origin(),
                half_height,
                radius_bottom,
                radius_top,
            }),
            bounds: symmetric_bounds(reach, half_height, reach),
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
        Self::extrude_draft(profile, height, 0.0)
    }

    /// The profile swept along +Y with a `draft` angle (radians): the section
    /// tapers inward (positive) or flares outward (negative) with height. See
    /// [`Extrude::with_draft`].
    ///
    /// # Errors
    /// Propagates [`Extrude::with_draft`] validation (`height > 0` and finite,
    /// `|draft| < `[`opensolid_frep::MAX_DRAFT`]).
    pub fn extrude_draft(profile: Profile2D, height: f64, draft: f64) -> CoreResult<Self> {
        // Build first so an invalid draft/height errors before we touch the
        // profile bounds.
        let shape = Extrude::with_draft(profile.clone(), height, draft)?;
        let (min, max) = profile.bounds();
        // A negative draft flares the top out past the base profile by up to
        // |tan(draft)|·height; pad the lateral box so the mesher's derived
        // grid still contains the whole solid. Positive draft only shrinks.
        let pad = (-draft.tan() * height).max(0.0);
        let bounds = BoundingBox3::new(
            Point3::new(min[0] - pad, 0.0, min[1] - pad),
            Point3::new(max[0] + pad, height, max[1] + pad),
        );
        Ok(Self {
            shape: Shape::new(shape),
            bounds,
        })
    }

    /// A half-space: the closed set on the negative side of the plane through
    /// `point` with unit-ish outward `normal` (interior where
    /// `normal · (p − point) ≤ 0`). Unbounded, so its tracked box is a large
    /// finite cube — intended only as an operand of `intersect` (e.g. the
    /// "up to face" extrude terminator), where the result inherits the other
    /// operand's finite bounds.
    pub fn half_space(point: Point3, normal: Vector3) -> Self {
        let n = normal.try_normalize(1e-12).unwrap_or_else(Vector3::y);
        let offset = n.dot(&point.coords);
        const FAR: f64 = 1.0e6;
        Self {
            shape: Shape::new(HalfSpace { normal: n, offset }),
            bounds: BoundingBox3::new(Point3::new(-FAR, -FAR, -FAR), Point3::new(FAR, FAR, FAR)),
        }
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

    /// An open sketch path thickened into a support rib and swept along +Y
    /// over `y ∈ [0, height]`; path `(u, v)` maps to world `(x, z)` exactly
    /// as [`Self::extrude`] does. `side` selects which side of the path
    /// receives material ([`RibSide::Both`] is symmetric and exact).
    ///
    /// # Errors
    /// Propagates [`Rib::new`] validation (`thickness > 0`, `height > 0`,
    /// both finite).
    pub fn rib(path: OpenPath2D, thickness: f64, height: f64, side: RibSide) -> CoreResult<Self> {
        let rib = Rib::new(path, thickness, height, side)?;
        let (min, max) = rib.world_bounds();
        let bounds = BoundingBox3::new(
            Point3::new(min[0], min[1], min[2]),
            Point3::new(max[0], max[1], max[2]),
        );
        Ok(Self {
            shape: Shape::new(rib),
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

    /// Tapered (drafted) about the plane through `neutral_point` with normal
    /// `pull`, by draft `angle` in radians. Sign-exact but not metric-exact
    /// — see [`opensolid_frep::Taper`]. The tracked box is the AABB of the
    /// tracked box's forward taper image, bounded with interval arithmetic
    /// (conservative for any axis).
    ///
    /// # Errors
    /// Propagates [`Taper::new`] validation (finite non-zero `pull`, finite
    /// `neutral_point`, and `|angle| < π/2`).
    pub fn taper(&self, pull: Vector3, neutral_point: Point3, angle: f64) -> CoreResult<Self> {
        let shape = Shape::new(self.shape.clone().tapered(pull, neutral_point, angle)?);
        // Recompute the (now-validated) parameters for the forward bounds map.
        let axis = pull / pull.norm();
        let neutral = axis.dot(&neutral_point.coords);
        let rate = angle.tan();
        let pt = Interval::point;
        let b = &self.bounds;
        let bx = Interval::new(b.min.x, b.max.x);
        let by = Interval::new(b.min.y, b.max.y);
        let bz = Interval::new(b.min.z, b.max.z);
        // Forward map D(q) = k·q + (1 − k)·(axis·q)·axis, k = 1 + rate·(a − neutral).
        let a = pt(axis.x) * bx + pt(axis.y) * by + pt(axis.z) * bz;
        let k = pt(1.0) + pt(rate) * (a - pt(neutral));
        let d = |bj: Interval, nj: f64| k * bj + (pt(1.0) - k) * a * pt(nj);
        let dx = d(bx, axis.x);
        let dy = d(by, axis.y);
        let dz = d(bz, axis.z);
        Ok(Self {
            shape,
            bounds: BoundingBox3::new(
                Point3::new(dx.lo, dy.lo, dz.lo),
                Point3::new(dx.hi, dy.hi, dz.hi),
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

    /// Edge-selective fillet/chamfer: the boolean `kind` of `self` and
    /// `other`, with its sharp edge rounded (fillet) or beveled (chamfer)
    /// only within `radius`-scaled reach of the selected edge `region`.
    /// Untouched edges stay sharp. The blend perturbs the surface by at most
    /// ~`radius` near the edge, so the boolean's tracked box is padded by
    /// that much for meshing/framing headroom.
    pub fn blend_edge(
        &self,
        other: &Self,
        kind: BooleanKind,
        mode: BlendMode,
        radius: f64,
        region: EdgeRegion,
    ) -> Self {
        let base = match kind {
            BooleanKind::Union => self.bounds.union(&other.bounds),
            BooleanKind::Intersection => bounds_intersection(&self.bounds, &other.bounds),
            BooleanKind::Subtraction => self.bounds,
        };
        let pad = Vector3::repeat(0.5 * radius.max(0.0));
        Self {
            shape: self
                .shape
                .clone()
                .blend_edge(other.shape.clone(), kind, mode, radius, region),
            bounds: BoundingBox3::new(base.min - pad, base.max + pad),
        }
    }

    /// Signed distance from `point` to the surface: negative inside,
    /// positive outside. After smooth blends or anisotropic scaling the
    /// field is not an exact Euclidean distance, but its sign and zero set
    /// stay correct, so nearest-surface queries can compare magnitudes.
    pub fn distance(&self, point: Point3) -> f64 {
        self.shape.eval(&point)
    }

    /// Outward unit surface normal at `point`, the normalized field
    /// gradient (F-Rep normals are exact where the field is smooth). On a
    /// point off the surface it is still the field's ascent direction, so
    /// "sketch on a curved face" can take the normal at the picked hit
    /// point directly. Returns `+X` as a stable fallback where the gradient
    /// vanishes (a flat interior, or a non-differentiable locus).
    pub fn surface_normal(&self, point: Point3) -> Vector3 {
        let g = opensolid_frep::eval::gradient(&self.shape, &point);
        g.try_normalize(0.0).unwrap_or_else(Vector3::x)
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

    /// Mesh the shape with graded adaptive octree dual contouring to a
    /// target `accuracy`: the maximum chordal deviation of the mesh from
    /// the surface, in model units. The octree refines near curvature and
    /// CSG feature edges (kept crisp by QEF vertex placement) and stays
    /// coarse over flat regions. With `bound` set, the grid covers the
    /// explicit cube `[-bound, bound]³`; otherwise bounds are auto-derived
    /// from the tracked bounding box.
    ///
    /// The octree depth is derived from `accuracy` (finest cell roughly one
    /// accuracy unit wide) and clamped to [`ADAPTIVE_MAX_DEPTH`], so
    /// accuracies below about `extent / 2^9` degrade gracefully instead of
    /// exploding the cell budget. Non-finite or non-positive accuracies
    /// fall back to 0.5% of the meshed extent.
    ///
    /// The raw dual-contouring output is refined before returning
    /// ([`refine_mesh`]): vertices on sharp CSG edges are snapped onto the
    /// analytic intersection curves (no scalloping), and a feature-aware
    /// smoothing/flip pass regularizes the triangulation.
    pub fn mesh_adaptive(&self, accuracy: f64, bound: Option<f64>) -> TriangleMesh {
        let bounds = match bound {
            Some(b) => symmetric_bounds(b, b, b),
            // Any resolution >= 30 pads by the flat 10%, which exceeds the
            // one-cell clearance the stitch needs at every allowed depth.
            None => self.mesh_bounds(64),
        };
        let extent = max_extent(&bounds).max(1e-9);
        let accuracy = if accuracy.is_finite() && accuracy > 0.0 {
            accuracy
        } else {
            5e-3 * extent
        };
        let max_depth = (extent / accuracy)
            .log2()
            .ceil()
            .clamp(ADAPTIVE_MIN_DEPTH as f64, ADAPTIVE_MAX_DEPTH as f64)
            as u32;
        let mut mesh = mesh_sdf_adaptive_indexed(
            &self.shape,
            &AdaptiveMeshOptions {
                bounds,
                max_depth,
                accuracy: Some(accuracy),
            },
        );
        let cell = extent / (1u64 << max_depth) as f64;
        refine_mesh(&self.shape, &mut mesh, &RefineOptions::for_cell(cell));
        mesh
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

    /// The playground's default script: rounded box smooth-united with a
    /// sphere, cylinder hole subtracted.
    fn default_playground_scene() -> BoundedShape {
        let body = BoundedShape::rounded_box(1.0, 0.55, 0.8, 0.15);
        let bump = BoundedShape::sphere(0.55).translate(Vector3::new(0.0, 0.65, 0.0));
        let hole = BoundedShape::cylinder(0.28, 2.0);
        body.smooth_union(&bump, Some(0.25)).subtract(&hole)
    }

    /// Accuracy-driven adaptive meshing with auto bounds must stay
    /// watertight and within the deviation target across the primitive
    /// gallery (curved, flat, and sharp-rimmed shapes).
    #[test]
    fn mesh_adaptive_primitives_within_accuracy() {
        let acc = 0.01;
        for s in [
            BoundedShape::sphere(1.0),
            BoundedShape::box3(1.0, 0.5, 0.75),
            BoundedShape::rounded_box(1.0, 0.5, 0.75, 0.2),
            BoundedShape::cylinder(0.5, 1.0),
            BoundedShape::torus(1.0, 0.3),
        ] {
            let mesh = s.mesh_adaptive(acc, None);
            assert!(!mesh.is_empty(), "adaptive mesh is empty");
            assert!(mesh.is_closed_manifold(), "adaptive mesh not manifold");
            for p in &mesh.positions {
                // 2x: leaves at the depth cap carry no per-vertex accuracy
                // certificate (refinement was clamped, not converged).
                assert!(
                    s.shape.eval(p).abs() <= 2.0 * acc,
                    "vertex {p:?} deviates beyond target"
                );
            }
        }
    }

    /// Regression for of-54d: at the playground's default accuracy the raw
    /// graded adaptive mesh of the default scene must have zero four-triangle
    /// (pinched) edges — the crown where the hole meets the sphere is a
    /// two-sheets-per-cell band that fused into a ragged non-manifold scar
    /// before per-component cell vertices. The `repair_pinched_edges` fallback
    /// therefore does nothing here (a no-op), and the crown wireframe is
    /// locally manifold.
    #[test]
    fn mesh_adaptive_default_scene_has_no_pinched_edges() {
        let part = default_playground_scene();
        let bounds = part.mesh_bounds(64);
        let extent = max_extent(&bounds).max(1e-9);
        // The playground meshes at 0.005; check the whole clean band around it.
        for acc in [0.01, 0.005, 0.0025, 0.001] {
            let max_depth = (extent / acc)
                .log2()
                .ceil()
                .clamp(ADAPTIVE_MIN_DEPTH as f64, ADAPTIVE_MAX_DEPTH as f64)
                as u32;
            let mesh = mesh_sdf_adaptive_indexed(
                &part.shape,
                &AdaptiveMeshOptions {
                    bounds,
                    max_depth,
                    accuracy: Some(acc),
                },
            );
            assert_eq!(
                opensolid_frep::refine::pinched_edge_count(&mesh),
                0,
                "raw adaptive default scene has pinched edges at acc {acc}"
            );
            assert!(
                mesh.is_closed_manifold(),
                "raw adaptive default scene not manifold at acc {acc}"
            );
        }
    }

    /// The default playground scene (smooth blend + sharp hole rims) must
    /// mesh watertight and on-target through the adaptive path.
    #[test]
    fn mesh_adaptive_default_scene_within_accuracy() {
        let part = default_playground_scene();
        let acc = 0.01;
        let mesh = part.mesh_adaptive(acc, None);
        assert!(!mesh.is_empty());
        assert!(mesh.is_closed_manifold(), "default scene not manifold");
        for p in &mesh.positions {
            assert!(
                part.shape.eval(p).abs() <= 2.0 * acc,
                "vertex {p:?} deviates beyond target"
            );
        }
    }

    /// The bead's headline acceptance (of-5fl.10): the rim of the cylinder
    /// hole in the default playground scene must come out as a clean
    /// circle, not a sawtooth. After the refine pass, the crease vertices
    /// where the hole meets the flat bottom face sit *exactly* (1e-9) on
    /// the analytic circle x² + z² = 0.28² at y = -0.55.
    #[test]
    fn mesh_adaptive_default_scene_rim_is_exact() {
        let part = default_playground_scene();
        let mesh = part.mesh_adaptive(0.005, None);
        assert!(mesh.is_closed_manifold());
        let exact = mesh
            .positions
            .iter()
            .filter(|p| {
                let radial = (p.x * p.x + p.z * p.z).sqrt();
                (radial - 0.28).abs() < 1e-9 && (p.y + 0.55).abs() < 1e-9
            })
            .count();
        // The rim crosses hundreds of finest cells at this accuracy; a
        // healthy snap lands a vertex per crease cell.
        assert!(exact > 50, "only {exact} vertices exactly on the hole rim");
    }

    /// Garbage accuracies must not panic: they fall back to a sane default.
    #[test]
    fn mesh_adaptive_degenerate_accuracy() {
        let s = BoundedShape::sphere(1.0);
        for acc in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            let mesh = s.mesh_adaptive(acc, None);
            assert!(mesh.is_closed_manifold());
        }
        // Explicit bound works like the uniform path's.
        assert!(s.mesh_adaptive(0.01, Some(1.5)).is_closed_manifold());
    }

    /// Perf measurement for the bead notes, not a gate:
    /// `cargo test -p opensolid-wasm --release -- --ignored --nocapture`
    #[test]
    #[ignore = "perf measurement; run with --release -- --ignored --nocapture"]
    fn adaptive_remesh_perf_default_scene() {
        let part = default_playground_scene();
        for acc in [0.02, 0.01, 0.005] {
            let t0 = std::time::Instant::now();
            let mesh = part.mesh_adaptive(acc, None);
            let ms = t0.elapsed().as_secs_f64() * 1e3;
            eprintln!(
                "adaptive acc {acc}: {} tris, {} verts, {ms:.1} ms",
                mesh.triangle_count(),
                mesh.vertex_count()
            );
        }
        for res in [64usize, 128] {
            let t0 = std::time::Instant::now();
            let mesh = part.mesh(res, None);
            let ms = t0.elapsed().as_secs_f64() * 1e3;
            eprintln!(
                "uniform res {res}: {} tris, {} verts, {ms:.1} ms",
                mesh.triangle_count(),
                mesh.vertex_count()
            );
        }
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
    fn distance_is_signed_and_zero_on_surface() {
        let s = BoundedShape::sphere(1.0);
        assert!((s.distance(Point3::new(2.0, 0.0, 0.0)) - 1.0).abs() < 1e-12);
        assert!((s.distance(Point3::origin()) + 1.0).abs() < 1e-12);
        assert!(s.distance(Point3::new(0.0, 1.0, 0.0)).abs() < 1e-12);

        // Follows transforms: the moved sphere's surface is at x = 4.
        let moved = s.translate(Vector3::new(3.0, 0.0, 0.0));
        assert!(moved.distance(Point3::new(4.0, 0.0, 0.0)).abs() < 1e-12);
        assert!(moved.distance(Point3::origin()) > 1.0);

        // CSG: a point inside the subtracted hole is outside the result but
        // inside (negative for) the hole shape — magnitude comparison picks
        // the hole as the nearest feature.
        let plate = BoundedShape::box3(1.0, 0.2, 1.0);
        let hole = BoundedShape::cylinder(0.3, 1.0);
        let part = plate.subtract(&hole);
        let inside_hole = Point3::new(0.0, 0.0, 0.0);
        assert!(part.distance(inside_hole) > 0.0);
        assert!(hole.distance(inside_hole) < 0.0);
    }

    #[test]
    fn surface_normal_points_outward_and_follows_transforms() {
        // Sphere normals are radial: the outward unit normal at a surface
        // point equals that point (unit sphere).
        let s = BoundedShape::sphere(1.0);
        let n = s.surface_normal(Point3::new(0.0, 1.0, 0.0));
        assert!((n - Vector3::new(0.0, 1.0, 0.0)).norm() < 1e-4);

        // Off-surface points still report the ascent direction (radial).
        let n2 = s.surface_normal(Point3::new(3.0, 0.0, 0.0));
        assert!((n2 - Vector3::new(1.0, 0.0, 0.0)).norm() < 1e-4);

        // A moved sphere's normal at its shifted north pole still points +Y.
        let moved = s.translate(Vector3::new(5.0, 0.0, 0.0));
        let n3 = moved.surface_normal(Point3::new(5.0, 1.0, 0.0));
        assert!((n3 - Vector3::new(0.0, 1.0, 0.0)).norm() < 1e-4);

        // Result is a unit vector.
        assert!((n.norm() - 1.0).abs() < 1e-9);
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
    fn taper_flares_bounds_and_meshes() {
        // Draft a unit cube about y = 0 pulling along +Y: the +Y section
        // widens to ±(1 + tan(0.25)) ≈ ±1.255, the −Y section pinches in.
        let s = BoundedShape::box3(1.0, 1.0, 1.0)
            .taper(Vector3::new(0.0, 1.0, 0.0), Point3::origin(), 0.25)
            .expect("valid taper");
        let flare = 1.0 + 0.25_f64.tan();
        // The tracked box is a conservative (interval-arithmetic) superset of
        // the flared solid: it must contain the true drafted extents.
        assert!(s.bounds.min.x <= -flare + 1e-9 && s.bounds.max.x >= flare - 1e-9);
        assert!(s.bounds.min.z <= -flare + 1e-9 && s.bounds.max.z >= flare - 1e-9);
        assert!(s.bounds.min.y <= -1.0 + 1e-9 && s.bounds.max.y >= 1.0 - 1e-9);
        assert!(s.bounds.min.x.is_finite() && s.bounds.max.y.is_finite());
        // The flared wall is a real surface point; the pinched-in one too.
        assert!(s.shape.eval(&Point3::new(flare, 1.0, 0.0)).abs() < 1e-9);
        assert!(
            s.shape
                .eval(&Point3::new(1.0 - 0.25_f64.tan(), -1.0, 0.0))
                .abs()
                < 1e-9
        );
        assert_meshes_cleanly(&s);
    }

    #[test]
    fn taper_rejects_bad_arguments() {
        let s = BoundedShape::box3(1.0, 1.0, 1.0);
        assert!(s.taper(Vector3::zeros(), Point3::origin(), 0.1).is_err());
        assert!(
            s.taper(
                Vector3::new(0.0, 1.0, 0.0),
                Point3::origin(),
                std::f64::consts::FRAC_PI_2
            )
            .is_err()
        );
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

    fn unit_square() -> Profile2D {
        Profile2D::new(
            vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]],
            vec![0.0; 4],
        )
        .expect("valid square")
    }

    /// Signed volume enclosed by a closed mesh via the divergence theorem:
    /// (1/6) Σ v0 · (v1 × v2) over outward-oriented triangles.
    fn mesh_volume(mesh: &TriangleMesh) -> f64 {
        let mut v = 0.0;
        for [i, j, k] in &mesh.indices {
            let a = mesh.positions[*i].coords;
            let b = mesh.positions[*j].coords;
            let c = mesh.positions[*k].coords;
            v += a.dot(&b.cross(&c));
        }
        v / 6.0
    }

    #[test]
    fn draft_orders_volume_and_meshes() {
        // Positive draft narrows the section (less volume than the prism);
        // negative draft flares it (more). All three mesh watertight.
        let straight = BoundedShape::extrude(unit_square(), 1.0).expect("valid");
        let inward = BoundedShape::extrude_draft(unit_square(), 1.0, 0.2).expect("valid");
        let outward = BoundedShape::extrude_draft(unit_square(), 1.0, -0.2).expect("valid");
        let vol = |s: &BoundedShape| mesh_volume(&assert_meshes_cleanly(s)).abs();
        let (vi, vs, vo) = (vol(&inward), vol(&straight), vol(&outward));
        assert!(vi < vs, "inward draft {vi} !< straight {vs}");
        assert!(vs < vo, "straight {vs} !< outward draft {vo}");
    }

    #[test]
    fn positive_draft_matches_frustum_volume() {
        // A unit square drafted by tan(draft)=0.2 over height 1 is a square
        // frustum: base side 1, each wall inset by 0.2 → top side 0.6.
        // V = (H/3)(A_b + A_t + sqrt(A_b·A_t)).
        let draft = 0.2_f64.atan();
        let s = BoundedShape::extrude_draft(unit_square(), 1.0, draft).expect("valid");
        let mesh = s.mesh_adaptive(0.004, None);
        let (a_b, a_t) = (1.0_f64, 0.6_f64 * 0.6);
        let expected = (1.0 / 3.0) * (a_b + a_t + (a_b * a_t).sqrt());
        let got = mesh_volume(&mesh).abs();
        assert!(
            (got - expected).abs() < 0.03 * expected,
            "frustum volume {got} vs expected {expected}"
        );
    }

    #[test]
    fn half_space_clips_extrude_up_to_face() {
        // "Up to face": a through-all extrude (height 2) intersected with the
        // half-space below y = 0.5 terminates the solid at that plane. Volume
        // = base area (1) × 0.5.
        let tall = BoundedShape::extrude(unit_square(), 2.0).expect("valid");
        let stop =
            BoundedShape::half_space(Point3::new(0.0, 0.5, 0.0), Vector3::new(0.0, 1.0, 0.0));
        let clipped = tall.intersect(&stop);
        // The intersection inherits the extrude's finite bounds (not FAR).
        assert!(clipped.bounds.max.y <= 2.0 + 1e-9);
        let mesh = clipped.mesh_adaptive(0.004, None);
        assert!(mesh.is_closed_manifold(), "clipped solid not watertight");
        let got = mesh_volume(&mesh).abs();
        assert!((got - 0.5).abs() < 0.02, "clipped volume {got} vs 0.5");
        // Top cap sits on the terminating plane.
        assert!(clipped.shape.eval(&Point3::new(0.5, 0.5, 0.5)).abs() < 1e-9);
    }

    #[test]
    fn extrude_and_revolve_reject_bad_input() {
        assert!(BoundedShape::extrude(l_profile(), 0.0).is_err());
        assert!(BoundedShape::extrude(l_profile(), -1.0).is_err());
        assert!(BoundedShape::extrude_draft(l_profile(), 1.0, f64::NAN).is_err());
        assert!(BoundedShape::extrude_draft(l_profile(), 1.0, 1.5).is_err());
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
    fn rib_tracks_bounds_and_meshes() {
        // An open V-shaped path thickened symmetrically.
        let path = OpenPath2D::new(vec![[-0.6, 0.0], [0.0, 0.4], [0.6, 0.0]], vec![0.0, 0.0])
            .expect("valid path");
        let s = BoundedShape::rib(path, 0.2, 0.8, RibSide::Both).expect("valid rib");
        // Tracked box: centreline box [-0.6,0.6]×[0,0.4] grown by thickness
        // 0.2 in x/z, y ∈ [0, 0.8].
        let close = |a: Point3, b: Point3| (a - b).norm() < 1e-9;
        assert!(close(s.bounds.min, Point3::new(-0.8, 0.0, -0.2)));
        assert!(close(s.bounds.max, Point3::new(0.8, 0.8, 0.6)));
        assert_meshes_cleanly(&s);

        // One-sided ribs still mesh cleanly under auto-bounds.
        let path2 = OpenPath2D::new(vec![[-0.5, 0.0], [0.5, 0.0]], vec![0.0]).expect("valid path");
        let one_sided = BoundedShape::rib(path2, 0.25, 0.6, RibSide::First).expect("valid rib");
        assert_meshes_cleanly(&one_sided);
    }

    #[test]
    fn rib_rejects_bad_input() {
        let path = || OpenPath2D::new(vec![[0.0, 0.0], [1.0, 0.0]], vec![0.0]).expect("valid path");
        assert!(BoundedShape::rib(path(), 0.0, 1.0, RibSide::Both).is_err());
        assert!(BoundedShape::rib(path(), -0.2, 1.0, RibSide::Both).is_err());
        assert!(BoundedShape::rib(path(), 0.2, 0.0, RibSide::Both).is_err());
        assert!(BoundedShape::rib(path(), 0.2, f64::NAN, RibSide::Both).is_err());
    }

    #[test]
    fn rib_unions_with_extrude_base() {
        // A rib standing on top of an extruded base plate — the canonical
        // "support rib" composition: union, then mesh as one solid.
        let base = BoundedShape::extrude(
            Profile2D::new(
                vec![[-0.8, -0.3], [0.8, -0.3], [0.8, 0.3], [-0.8, 0.3]],
                vec![0.0; 4],
            )
            .expect("valid base"),
            0.2,
        )
        .expect("valid extrude");
        let rib = BoundedShape::rib(
            OpenPath2D::new(vec![[-0.6, 0.0], [0.6, 0.0]], vec![0.0]).expect("valid path"),
            0.15,
            0.6,
            RibSide::Both,
        )
        .expect("valid rib");
        let part = base.union(&rib);
        let mesh = part.mesh(RES, None);
        assert!(!mesh.is_empty());
        assert!(mesh.is_closed_manifold());
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

    /// An edge-selective fillet on the union of two overlapping boxes must
    /// mesh watertight and actually round the selected edge: along the picked
    /// edge the filleted surface bulges inward relative to the sharp union,
    /// while a box corner far from the edge is left untouched.
    #[test]
    fn blend_edge_fillet_localizes_and_meshes() {
        let a = BoundedShape::box3(1.0, 1.0, 1.0);
        let b = BoundedShape::box3(1.0, 1.0, 1.0).translate(Vector3::new(1.0, 0.0, 0.0));
        // The two unit cubes share the plane x = 1 over y,z ∈ [-1, 1]; the
        // convex edge picked is the vertical edge at (1, 1, z).
        let edge =
            EdgeRegion::from_polyline(&[Point3::new(1.0, 1.0, -1.0), Point3::new(1.0, 1.0, 1.0)]);
        let sharp = a.union(&b);
        let filleted = a.blend_edge(&b, BooleanKind::Union, BlendMode::Fillet, 0.3, edge);

        // On the selected edge the fillet fills the concave notch: the field
        // is strictly more negative (inside) than the sharp union.
        let on_edge = Point3::new(1.0, 1.0, 0.0);
        assert!(filleted.distance(on_edge) < sharp.distance(on_edge) - 1e-6);

        // A corner far from the edge is untouched (sharp == filleted).
        let far = Point3::new(-1.0, -1.0, -1.0);
        assert!((filleted.distance(far) - sharp.distance(far)).abs() < 1e-9);

        let mesh = filleted.mesh_adaptive(0.01, None);
        assert!(!mesh.is_empty());
        assert!(mesh.is_closed_manifold(), "filleted union not manifold");
    }
}
