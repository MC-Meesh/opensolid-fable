//! NURBS geometry: rational B-spline curves and tensor-product surfaces.

pub mod curve;
pub mod surface;

pub use curve::{KnotVector, NurbsCurve, NurbsError};
pub use surface::NurbsSurface;
