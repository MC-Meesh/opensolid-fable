use crate::blend::SmoothUnion;
use crate::csg::{Intersection, Subtraction, Union};
use crate::primitives::Sdf;
use opensolid_core::types::{Point3, Vector3};
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
}

impl Sdf for Shape {
    fn eval(&self, p: &Point3) -> f64 {
        self.0.eval(p)
    }

    fn grad(&self, p: &Point3) -> Vector3 {
        self.0.grad(p)
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
