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
//! [`CheckFailure`]s), the sweep constructors ([`sweep`]: [`extrude`]
//! and [`revolve`] planar profiles into solids), the geometry store
//! ([`GeometryStore`] backing [`Edge::curve`]/[`Face::surface`]), and
//! primitive solids ([`primitives`]: block, cylinder, sphere, torus with
//! full topology + geometry), body placement ([`transform`]:
//! [`translate_body`]), booleans over store-backed bodies ([`boolean`]:
//! unite/subtract/intersect producing geometry-bound results), and
//! tessellation ([`tessellate`]: analytic faces to
//! [`opensolid_core::TriangleMesh`]). Tolerant modeling lands here next.
//!
//! This crate follows the OpenSolid error handling policy documented at the
//! [`opensolid_core`] crate level: fallible public APIs (e.g. the [`Curve3`]
//! and [`Surface3`] constructors) return [`opensolid_core::CoreResult`]
//! instead of panicking on invalid input.

pub mod boolean;
pub mod check;
pub mod curve;
pub mod euler;
pub mod geometry;
pub mod nurbs;
pub mod primitives;
pub mod project;
pub mod ssi;
pub mod surface;
pub mod sweep;
pub mod tessellate;
pub mod topology;
pub mod transform;

pub use boolean::{BooleanOp, BooleanOutput};
pub use check::{CheckFailure, EntityRef, MAX_ALLOWED_TOLERANCE};
pub use curve::{Curve3, CurveEval};
pub use euler::{EulerCounts, EulerError};
pub use geometry::GeometryStore;
pub use nurbs::{KnotVector, NurbsCurve, NurbsError, NurbsSurface};
pub use project::{CurveProject, CurveProjection, SurfaceProject, SurfaceProjection};
pub use ssi::{
    IntersectionCurve, IntersectionKind, MarchedCurve, SurfaceIntersection, intersect,
    intersect_nurbs,
};
pub use surface::{Surface3, SurfaceEval};
pub use sweep::{Profile, ProfileSegment, SweptBody, extrude, revolve};
pub use tessellate::{TessellationOptions, tessellate_body, tessellate_face};
pub use topology::{
    Body, BodyType, Curve, Edge, Face, FaceSense, Fin, FinSense, Loop, LoopType, SYSTEM_RESOLUTION,
    Shell, ShellOrientation, TopologyStore, Vertex,
};
pub use transform::translate_body;
