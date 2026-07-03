// Unified kernel: bridges F-Rep and B-Rep representations
// TODO: implicit-to-boundary conversion, session
pub mod mesh;

pub use opensolid_brep as brep;
pub use opensolid_core as core;
pub use opensolid_frep as frep;

pub use mesh::{MeshOptions, Triangle, TriangleMesh, mesh_sdf, mesh_sdf_indexed};
