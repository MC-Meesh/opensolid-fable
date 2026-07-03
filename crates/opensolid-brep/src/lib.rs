//! B-Rep: boundary representation topology + parametric geometry.
//!
//! Current contents: analytic parametric curves ([`Curve3`]) and the curve
//! evaluation trait ([`CurveEval`]), plus the topology graph
//! ([`TopologyStore`]: Body > Shell > Face > Loop > Fin > Edge > Vertex).
//! NURBS curves/surfaces and tolerant modeling land here next.

pub mod curve;
pub mod topology;

pub use curve::{Curve3, CurveEval};
pub use topology::{
    Body, BodyType, Curve, Edge, Face, FaceSense, Fin, FinSense, Loop, LoopType, SYSTEM_RESOLUTION,
    Shell, ShellOrientation, Surface, TopologyStore, Vertex,
};
