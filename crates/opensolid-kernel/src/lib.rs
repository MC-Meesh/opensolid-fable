// Unified kernel: bridges F-Rep and B-Rep representations
// TODO: implicit-to-boundary conversion
pub mod builder;
pub mod bvh;
pub mod io;
pub mod massprops;
pub mod mesh;
pub mod session;

pub use opensolid_brep as brep;
pub use opensolid_core as core;
pub use opensolid_frep as frep;

pub use builder::{Part, shape};
pub use bvh::Bvh;
pub use io::{write_obj, write_stl_ascii, write_stl_binary};
pub use massprops::{MassProperties, MassPropertiesError, mass_properties};
pub use mesh::{MeshOptions, Triangle, TriangleMesh, mesh_sdf, mesh_sdf_indexed};
pub use session::{JournalEntry, Model, Session, SessionError};
