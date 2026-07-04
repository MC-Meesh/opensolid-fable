//! Conversions between the kernel's two representations.
//!
//! Currently one direction: B-Rep → F-Rep via [`MeshSdf`] (tessellate,
//! then evaluate signed distance to the mesh). The reverse direction
//! (implicit → boundary) is a later issue.

pub mod brep_to_sdf;

pub use brep_to_sdf::MeshSdf;
