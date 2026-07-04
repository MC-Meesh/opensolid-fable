//! B-Rep: boundary representation topology + parametric geometry.
//!
//! Current contents: analytic parametric curves ([`Curve3`]) and surfaces
//! ([`Surface3`]) with their evaluation traits ([`CurveEval`],
//! [`SurfaceEval`]), analytic surface-surface intersection ([`ssi`]),
//! NURBS curves ([`NurbsCurve`]) and tensor-product
//! surfaces ([`NurbsSurface`]), closest-point projection ([`project`]:
//! [`CurveProject`], [`SurfaceProject`]), the topology graph
//! ([`TopologyStore`]: Body > Shell > Face > Loop > Fin > Edge > Vertex),
//! the Euler operators ([`euler`]: MVFS/MEV/MEF/KEMR/KFMRH with
//! Euler-Poincaré invariant checking), body validation
//! ([`check`]: [`TopologyStore::check`] returning structured
//! [`CheckFailure`]s), and the sweep constructors ([`sweep`]: [`extrude`]
//! and [`revolve`] planar profiles into solids). Tolerant modeling lands
//! here next.
//!
//! This crate follows the OpenSolid error handling policy documented at the
//! [`opensolid_core`] crate level: fallible public APIs (e.g. the [`Curve3`]
//! and [`Surface3`] constructors) return [`opensolid_core::CoreResult`]
//! instead of panicking on invalid input.

pub mod check;
pub mod curve;
pub mod euler;
pub mod nurbs;
pub mod project;
pub mod ssi;
pub mod surface;
pub mod sweep;
pub mod topology;

pub use check::{CheckFailure, EntityRef, MAX_ALLOWED_TOLERANCE};
pub use curve::{Curve3, CurveEval};
pub use euler::{EulerCounts, EulerError};
pub use nurbs::{KnotVector, NurbsCurve, NurbsError, NurbsSurface};
pub use project::{CurveProject, CurveProjection, SurfaceProject, SurfaceProjection};
pub use ssi::{IntersectionCurve, IntersectionKind, SurfaceIntersection, intersect};
pub use surface::{Surface3, SurfaceEval};
pub use sweep::{Profile, ProfileSegment, SweptBody, extrude, revolve};
pub use topology::{
    Body, BodyType, Curve, Edge, Face, FaceSense, Fin, FinSense, Loop, LoopType, SYSTEM_RESOLUTION,
    Shell, ShellOrientation, Surface, TopologyStore, Vertex,
};
