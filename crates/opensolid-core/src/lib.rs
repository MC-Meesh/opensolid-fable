//! Core types for the OpenSolid kernel: points, vectors, transforms,
//! bounding boxes, triangle meshes, and the generational arena.
//!
//! # Error handling policy
//!
//! All OpenSolid crates follow these rules:
//!
//! - **Public APIs that can fail return [`CoreResult`].** Invalid caller
//!   input is reported through [`CoreError`] — it is never answered with a
//!   panic. Error messages are actionable: they name the offending argument
//!   or operation and the constraint that was violated.
//! - **Panics are reserved for internal invariant violations** — bugs in
//!   the library, not misuse by the caller. If a function can panic when a
//!   *documented precondition* is broken (e.g. parallel arrays of a
//!   [`TriangleMesh`] manipulated directly), it says so under a `# Panics`
//!   heading; a function without that heading does not panic.
//! - **Expected absences return `Option`.** Lookups whose "miss" is a normal
//!   outcome (e.g. [`Arena::get`] after removal, [`TriangleMesh::bounding_box`]
//!   on an empty mesh) use `Option` rather than an error.

pub mod arena;
pub mod error;
pub mod interval;
pub mod mesh;
pub mod tolerance;
pub mod types;

pub use arena::{Arena, ArenaSnapshot, EntityId};
pub use error::{CoreError, CoreResult};
pub use interval::Interval;
pub use mesh::{Triangle, TriangleMesh};
pub use tolerance::{ANGULAR_RESOLUTION, SYSTEM_RESOLUTION, ToleranceContext};
pub use types::{BoundingBox3, Point3, Ray3, Transform3, Vector3};
