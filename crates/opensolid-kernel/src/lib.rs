//! Unified kernel: bridges the F-Rep (`opensolid-frep`) and B-Rep
//! (`opensolid-brep`) representations into one modeling API.
//!
//! # The hybrid contract: booleans never fail
//!
//! [`hybrid::boolean`] combines two bodies given in either representation.
//! The exact analytic B-Rep pipeline runs first and its result is kept only
//! if it clears a full gate ladder — a closed, manifold, chord-faithful
//! tessellation *and* a runtime validation gate ([`hybrid::ValidationOptions`]:
//! topology [`check`](brep::BooleanOutput::check) plus a volume cross-check
//! against an independent F-Rep estimate). Anything else — a mixed-rep pair,
//! an exact-pipeline shortfall, or a silently-wrong result caught by the gate
//! — diverts to the F-Rep fallback, which tessellates both operands, wraps
//! them as signed distance fields ([`MeshSdf`]), does the trivially-robust
//! `min`/`max` CSG, and re-meshes. The result is always a watertight mesh.
//! See the [`hybrid`] module docs for the full rationale.
//!
//! # Modules
//!
//! - [`assembly`] — multi-part [`Assembly`] documents: `Transformed`-backed
//!   instancing, `max<0` interference detection, and rigid-body
//!   mass-property aggregation ([`Assembly`], [`Instance`], [`Part`]).
//! - [`hybrid`] — the never-fail boolean entry point ([`HybridBoolean`]).
//! - [`convert`] — conversion both ways: [`MeshSdf`] (B-Rep→SDF) and
//!   [`sdf_to_brep()`] (SDF→faceted B-Rep).
//! - [`builder`] — a fluent F-Rep builder ([`Part`], [`shape`]).
//! - [`mesh`] — dual-contouring meshers ([`mesh_sdf_indexed`],
//!   [`MeshOptions`]) producing the shared [`TriangleMesh`].
//! - [`massprops`] — exact polyhedral [`mass_properties`] (volume, centroid,
//!   inertia) via the divergence theorem.
//! - [`io`] — STL/OBJ writers ([`write_stl_binary`], [`write_obj`]).
//! - [`session`] — a modeling [`Session`] with copy-on-write undo/redo and an
//!   append-only journal.
//!
//! The three underlying crates are re-exported as [`core`], [`frep`], and
//! [`brep`] for direct access.
pub mod assembly;
pub mod builder;
pub mod convert;
pub mod hybrid;
pub mod io;
pub mod massprops;
pub mod mesh;
pub mod session;

pub use opensolid_brep as brep;
pub use opensolid_core as core;
pub use opensolid_frep as frep;

// The BVH moved to `opensolid-core` (of-alf) so `opensolid-brep` can use it
// for boolean clash detection; keep the old kernel paths working.
pub use opensolid_core::bvh;

pub use assembly::{
    Assembly, AssemblyError, AssemblyMassProperties, Instance, InterferenceReport,
    Part as AssemblyPart,
};
pub use builder::{Part, shape};
pub use convert::{MeshSdf, SdfToBrepOptions, sdf_to_brep};
pub use hybrid::{
    HybridBody, HybridBoolean, HybridOptions, HybridPath, ValidationDiagnostic, ValidationOptions,
};
pub use io::{write_obj, write_stl_ascii, write_stl_binary};
pub use massprops::{MassProperties, MassPropertiesError, mass_properties};
pub use mesh::{MeshOptions, Triangle, TriangleMesh, mesh_sdf, mesh_sdf_indexed};
pub use opensolid_core::bvh::Bvh;
pub use session::{JournalEntry, Model, Session, SessionError};
