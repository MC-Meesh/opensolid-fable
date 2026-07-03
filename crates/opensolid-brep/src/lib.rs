//! B-Rep: boundary representation topology + parametric geometry.
//!
//! Current contents: analytic parametric curves ([`Curve3`]), the curve
//! evaluation trait ([`CurveEval`]), NURBS curves ([`NurbsCurve`]), and the
//! topology graph ([`TopologyStore`]: Body > Shell > Face > Loop > Fin >
//! Edge > Vertex). NURBS surfaces and tolerant modeling land here next.

pub mod curve;
pub mod nurbs;
pub mod topology;

pub use curve::{Curve3, CurveEval};
pub use nurbs::{KnotVector, NurbsCurve, NurbsError};
pub use topology::{
    Body, BodyType, Curve, Edge, Face, FaceSense, Fin, FinSense, Loop, LoopType, SYSTEM_RESOLUTION,
    Shell, ShellOrientation, Surface, TopologyStore, Vertex,
};
