use crate::blend::SmoothUnion;
use crate::csg::{Intersection, Subtraction, Union};
use crate::fillet::{BlendMode, BooleanKind, EdgeBlend, EdgeRegion};
use crate::pattern::{CircularPattern, LinearPattern, Mirror};
use crate::primitives::Sdf;
use opensolid_core::error::CoreResult;
use opensolid_core::interval::Interval;
use opensolid_core::types::{BoundingBox3, Point3, Vector3};
use std::sync::Arc;

/// Runtime-composable handle to an SDF tree. Cloning is cheap (shared
/// reference), so subtrees can be reused across multiple shapes. The
/// generic combinators (`Union<A, B>` etc.) remain the zero-cost path;
/// `Shape` trades one virtual call per node for runtime composition.
#[derive(Clone)]
pub struct Shape(Arc<dyn Sdf>);

impl Shape {
    pub fn new(sdf: impl Sdf + 'static) -> Self {
        Shape(Arc::new(sdf))
    }

    pub fn union(self, other: Shape) -> Shape {
        Shape::new(Union { a: self, b: other })
    }

    pub fn intersect(self, other: Shape) -> Shape {
        Shape::new(Intersection { a: self, b: other })
    }

    pub fn subtract(self, other: Shape) -> Shape {
        Shape::new(Subtraction { a: self, b: other })
    }

    pub fn smooth_union(self, other: Shape, radius: f64) -> Shape {
        Shape::new(SmoothUnion {
            a: self,
            b: other,
            radius,
        })
    }

    /// Boolean of `self` and `other` whose sharp edge is filleted or
    /// chamfered only within `region` (the selected feature-edge polyline).
    /// Elsewhere it is exactly the sharp boolean, so untouched edges stay
    /// crisp. `radius` is the fillet radius / chamfer setback.
    pub fn blend_edge(
        self,
        other: Shape,
        kind: BooleanKind,
        mode: BlendMode,
        radius: f64,
        region: EdgeRegion,
    ) -> Shape {
        Shape::new(EdgeBlend::new(self, other, kind, mode, radius, region))
    }

    /// Convenience: a rounded fillet on the edge produced by unioning `self`
    /// with `other`, localized to `region`.
    pub fn fillet_edge(self, other: Shape, radius: f64, region: EdgeRegion) -> Shape {
        self.blend_edge(other, BooleanKind::Union, BlendMode::Fillet, radius, region)
    }

    /// Convenience: a planar chamfer on the edge produced by unioning `self`
    /// with `other`, localized to `region`.
    pub fn chamfer_edge(self, other: Shape, radius: f64, region: EdgeRegion) -> Shape {
        self.blend_edge(
            other,
            BooleanKind::Union,
            BlendMode::Chamfer,
            radius,
            region,
        )
    }

    /// `count` copies of this shape, copy `k` translated by `k * step`.
    ///
    /// # Errors
    /// Propagates [`LinearPattern::new`] validation (`count >= 1`, finite
    /// `step`).
    pub fn linear_pattern(self, step: Vector3, count: usize) -> CoreResult<Shape> {
        Ok(Shape::new(LinearPattern::new(self, step, count)?))
    }

    /// `count` copies of this shape rotated about the axis line through
    /// `center` with direction `axis`, copy `k` turned by `k * angle` radians.
    ///
    /// # Errors
    /// Propagates [`CircularPattern::new`] validation (`count >= 1`, non-zero
    /// finite `axis`, finite `angle`).
    pub fn circular_pattern(
        self,
        center: Point3,
        axis: Vector3,
        angle: f64,
        count: usize,
    ) -> CoreResult<Shape> {
        Ok(Shape::new(CircularPattern::new(
            self, center, axis, angle, count,
        )?))
    }

    /// This shape unioned with its reflection across the plane through `point`
    /// with `normal`.
    ///
    /// # Errors
    /// Propagates [`Mirror::new`] validation (non-zero finite `normal`).
    pub fn mirror(self, point: Point3, normal: Vector3) -> CoreResult<Shape> {
        Ok(Shape::new(Mirror::new(self, point, normal)?))
    }

    /// True if both handles refer to the same underlying SDF instance.
    /// Structural equality of SDF trees is undecidable in general; identity
    /// is what sessions need to verify state restoration.
    pub fn ptr_eq(&self, other: &Shape) -> bool {
        Arc::ptr_eq(&self.0, &other.0)
    }
}

impl Sdf for Shape {
    fn eval(&self, p: &Point3) -> f64 {
        self.0.eval(p)
    }

    fn grad(&self, p: &Point3) -> Vector3 {
        self.0.grad(p)
    }

    // Must forward: falling back to the trait default would discard the
    // tree's analytic bounds (and be wrong for non-metric inner fields).
    fn eval_interval(&self, b: &BoundingBox3) -> Interval {
        self.0.eval_interval(b)
    }

    // Must forward: the default would collapse the CSG tree's branch
    // decomposition into a single kinked branch.
    fn branches(&self, p: &Point3, tol: f64, out: &mut Vec<(f64, Vector3)>) {
        self.0.branches(p, tol, out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::primitives::{Box3, Cylinder, Sphere};

    fn sample_points() -> Vec<Point3> {
        vec![
            Point3::origin(),
            Point3::new(0.5, 0.0, 0.0),
            Point3::new(-1.2, 0.7, 0.3),
            Point3::new(0.0, 1.5, -0.5),
            Point3::new(3.0, -2.0, 1.0),
        ]
    }

    fn sphere(x: f64, r: f64) -> Sphere {
        Sphere {
            center: Point3::new(x, 0.0, 0.0),
            radius: r,
        }
    }

    #[test]
    fn boxed_dyn_tree_matches_generic() {
        let generic = Subtraction {
            a: Union {
                a: sphere(-0.5, 1.0),
                b: Box3 {
                    center: Point3::origin(),
                    half_extents: [0.8, 0.8, 0.8],
                },
            },
            b: Cylinder {
                center: Point3::origin(),
                radius: 0.3,
                half_height: 2.0,
            },
        };

        // Same tree assembled at runtime from trait objects.
        let union: Box<dyn Sdf> = Box::new(Union {
            a: Box::new(sphere(-0.5, 1.0)) as Box<dyn Sdf>,
            b: Box::new(Box3 {
                center: Point3::origin(),
                half_extents: [0.8, 0.8, 0.8],
            }) as Box<dyn Sdf>,
        });
        let dynamic: Box<dyn Sdf> = Box::new(Subtraction {
            a: union,
            b: Box::new(Cylinder {
                center: Point3::origin(),
                radius: 0.3,
                half_height: 2.0,
            }) as Box<dyn Sdf>,
        });

        for p in sample_points() {
            assert_eq!(generic.eval(&p), dynamic.eval(&p), "at {p:?}");
        }
    }

    #[test]
    fn shape_builder_matches_generic() {
        let shape = Shape::new(sphere(-0.5, 1.0))
            .union(Shape::new(sphere(0.5, 1.0)))
            .subtract(Shape::new(sphere(0.0, 0.4)))
            .smooth_union(Shape::new(sphere(2.0, 0.8)), 0.3)
            .intersect(Shape::new(Box3 {
                center: Point3::origin(),
                half_extents: [3.0, 3.0, 3.0],
            }));

        let generic = Intersection {
            a: SmoothUnion {
                a: Subtraction {
                    a: Union {
                        a: sphere(-0.5, 1.0),
                        b: sphere(0.5, 1.0),
                    },
                    b: sphere(0.0, 0.4),
                },
                b: sphere(2.0, 0.8),
                radius: 0.3,
            },
            b: Box3 {
                center: Point3::origin(),
                half_extents: [3.0, 3.0, 3.0],
            },
        };

        for p in sample_points() {
            assert_eq!(shape.eval(&p), generic.eval(&p), "at {p:?}");
        }
    }

    #[test]
    fn shape_forwards_eval_interval() {
        use opensolid_core::types::BoundingBox3;
        let s = sphere(0.0, 1.0);
        let shape = Shape::new(sphere(0.0, 1.0));
        // A box where the exact sphere interval is strictly tighter than
        // the Lipschitz default, so falling back would be detected.
        let b = BoundingBox3::new(Point3::new(1.0, 1.0, 1.0), Point3::new(3.0, 2.0, 2.0));
        let exact = s.eval_interval(&b);
        assert_eq!(shape.eval_interval(&b), exact);
        let center_d = s.eval(&b.center());
        let r = 0.5 * b.extents().norm();
        assert!(exact.lo > center_d - r && exact.hi < center_d + r);
    }

    #[test]
    fn composed_shape_interval_containment() {
        let shape = Shape::new(sphere(-0.5, 1.0))
            .union(Shape::new(sphere(0.5, 1.0)))
            .subtract(Shape::new(sphere(0.0, 0.4)))
            .smooth_union(Shape::new(sphere(1.2, 0.8)), 0.3);
        crate::test_util::assert_interval_containment(&shape, 41);
    }

    #[test]
    fn shape_is_send_sync_and_cheaply_cloneable() {
        fn assert_send_sync<T: Send + Sync>(_: &T) {}

        let a = Shape::new(sphere(0.0, 1.0));
        let reused = a.clone().union(a.clone());
        assert_send_sync(&reused);
        assert_eq!(reused.eval(&Point3::origin()), a.eval(&Point3::origin()));
    }

    #[test]
    fn reference_and_arc_evaluate_like_the_value() {
        let s = sphere(0.0, 1.0);
        let p = Point3::new(0.3, -0.2, 0.9);
        let by_ref = &s;
        assert_eq!(by_ref.eval(&p), s.eval(&p));

        let arc: Arc<dyn Sdf> = Arc::new(sphere(0.0, 1.0));
        assert_eq!(arc.eval(&p), s.eval(&p));
    }
}
