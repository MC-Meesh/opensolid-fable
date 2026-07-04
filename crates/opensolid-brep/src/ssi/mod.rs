//! Surface-surface intersection (SSI).
//!
//! [`analytic`] handles exact intersections between analytic primitives;
//! numeric marching for general surface pairs lands here later.

pub mod analytic;

pub use analytic::{IntersectionCurve, IntersectionKind, SurfaceIntersection, intersect};
