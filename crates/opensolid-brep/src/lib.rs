//! B-Rep: boundary representation topology + parametric geometry.
//!
//! Current contents: analytic parametric curves ([`Curve3`]) and the curve
//! evaluation trait ([`CurveEval`]). NURBS curves/surfaces, the
//! face/edge/vertex topology graph, and tolerant modeling land here next.

pub mod curve;

pub use curve::{Curve3, CurveEval};
