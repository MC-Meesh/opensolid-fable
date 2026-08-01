//! NURBS geometry: rational B-spline curves and tensor-product surfaces.

pub mod curve;
pub mod curve2;
pub mod surface;

pub use curve::{KnotVector, NurbsCurve, NurbsError};
pub use curve2::NurbsCurve2;
pub use surface::NurbsSurface;
