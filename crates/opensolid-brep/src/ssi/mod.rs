//! Surface-surface intersection (SSI).
//!
//! [`analytic`] handles exact intersections between analytic primitives;
//! [`marching`] traces NURBS-NURBS intersections numerically
//! (grid-seeded predictor-corrector marching, transversal MVP) and, via
//! [`intersect_marched`], the analytic sphere/torus pairs whose general
//! configurations have no closed form (cylinder-sphere, plane-torus,
//! cylinder-torus, sphere-torus, torus-torus).

pub mod analytic;
pub mod marching;

pub use analytic::{IntersectionCurve, IntersectionKind, SurfaceIntersection, intersect};
pub use marching::{MarchedCurve, intersect_marched, intersect_nurbs};
