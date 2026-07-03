//! B-Rep: boundary representation topology + parametric geometry.
//!
//! Current contents: analytic parametric curves ([`Curve3`]) and surfaces
//! ([`Surface3`]) with their evaluation traits ([`CurveEval`],
//! [`SurfaceEval`]), NURBS curves ([`NurbsCurve`]), and the topology graph
//! ([`TopologyStore`]: Body > Shell > Face > Loop > Fin > Edge > Vertex).
//! NURBS surfaces and tolerant modeling land here next.
//!
//! This crate follows the OpenSolid error handling policy documented at the
//! [`opensolid_core`] crate level: fallible public APIs (e.g. the [`Curve3`]
//! and [`Surface3`] constructors) return [`opensolid_core::CoreResult`]
//! instead of panicking on invalid input.

pub mod curve;
pub mod nurbs;
pub mod surface;
pub mod topology;

pub use curve::{Curve3, CurveEval};
pub use nurbs::{KnotVector, NurbsCurve, NurbsError};
pub use surface::{Surface3, SurfaceEval};
pub use topology::{
    Body, BodyType, Curve, Edge, Face, FaceSense, Fin, FinSense, Loop, LoopType, SYSTEM_RESOLUTION,
    Shell, ShellOrientation, Surface, TopologyStore, Vertex,
};
