// Unified kernel: bridges F-Rep and B-Rep representations.
// Both conversion directions live in `convert`.
pub mod builder;
pub mod convert;
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

pub use builder::{Part, shape};
pub use convert::{MeshSdf, SdfToBrepOptions, sdf_to_brep};
pub use io::{write_obj, write_stl_ascii, write_stl_binary};
pub use massprops::{MassProperties, MassPropertiesError, mass_properties};
pub use mesh::{MeshOptions, Triangle, TriangleMesh, mesh_sdf, mesh_sdf_indexed};
pub use opensolid_core::bvh::Bvh;
pub use session::{JournalEntry, Model, Session, SessionError};
