//! Surface-surface intersection (SSI).
//!
//! [`analytic`] handles exact intersections between analytic primitives;
//! [`marching`] traces NURBS-NURBS intersections numerically
//! (grid-seeded predictor-corrector marching, transversal MVP) and, via
//! [`intersect_marched`], the analytic primitive pairs whose general
//! configurations have no closed form and carry a compact partner to seed
//! the grid (cylinder-sphere, plane-torus, cylinder-torus, sphere-torus,
//! torus-torus, sphere-cone, torus-cone). [`intersect_marched_bounded`]
//! marches the unbounded plane-cone parabola/hyperbola sections within an
//! explicit region of interest (neither partner is compact).

pub mod analytic;
pub mod marching;

pub use analytic::{IntersectionCurve, IntersectionKind, SurfaceIntersection, intersect};
pub use marching::{MarchedCurve, intersect_marched, intersect_marched_bounded, intersect_nurbs};
