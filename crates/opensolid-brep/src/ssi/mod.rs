//! Surface-surface intersection (SSI).
//!
//! [`analytic`] handles exact intersections between analytic primitives;
//! [`marching`] traces NURBS-NURBS intersections numerically
//! (grid-seeded predictor-corrector marching, transversal MVP).

pub mod analytic;
pub mod marching;

pub use analytic::{IntersectionCurve, IntersectionKind, SurfaceIntersection, intersect};
pub use marching::{MarchedCurve, intersect_nurbs};
