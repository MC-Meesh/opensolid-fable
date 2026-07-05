//! Conversions between the kernel's two representations.
//!
//! B-Rep → F-Rep via [`MeshSdf`] (tessellate, then evaluate signed
//! distance to the mesh), and F-Rep → B-Rep via [`sdf_to_brep`] (adaptive
//! dual-contouring mesh, planar regions recovered as faces).

pub mod brep_to_sdf;
pub mod sdf_to_brep;

pub use brep_to_sdf::MeshSdf;
pub use sdf_to_brep::{SdfToBrepOptions, sdf_to_brep};
