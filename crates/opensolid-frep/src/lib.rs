//! F-Rep: functional (implicit) solid modeling on signed distance fields.
//!
//! A solid is a function `f(p)` that is negative inside, positive outside,
//! and zero on the boundary — the [`Sdf`](primitives::Sdf) trait
//! ([`primitives`]). Booleans are `min`/`max` on those fields ([`csg`]), so
//! they are trivially robust: unlike B-Rep surface surgery they cannot fail
//! on coincident faces or tangencies. The price is that the boundary is only
//! recovered approximately, at a chosen grid resolution.
//!
//! # What's here
//!
//! - **Primitives** ([`primitives`]): [`Sphere`](primitives::Sphere),
//!   [`Box3`](primitives::Box3), [`Cylinder`](primitives::Cylinder),
//!   [`Torus`](primitives::Torus), [`Cone`](primitives::Cone),
//!   [`Capsule`](primitives::Capsule), [`HalfSpace`](primitives::HalfSpace),
//!   [`RoundedBox`](primitives::RoundedBox). Each supplies an exact value,
//!   a gradient, and conservative interval bounds
//!   ([`Sdf::eval_interval`](primitives::Sdf::eval_interval)) for empty-space
//!   pruning.
//! - **CSG** ([`csg`]): sharp [`Union`](csg::Union) = `min`,
//!   [`Intersection`](csg::Intersection) = `max`,
//!   [`Subtraction`](csg::Subtraction) = `max(a, -b)`.
//! - **Smooth blending** ([`blend`]): polynomial
//!   [`SmoothUnion`](blend::SmoothUnion) /
//!   [`SmoothSubtraction`](blend::SmoothSubtraction) for organic fillets.
//! - **Offset family** ([`ops`]): [`Offset`], [`Shell`], [`Rounded`],
//!   chainable via [`SdfOpsExt`].
//! - **Transforms** ([`transform`]): rigid [`Transformed`], uniform/
//!   anisotropic scale, and [`Taper`] (draft about a neutral plane).
//! - **Profiles & sweeps** ([`profile`]): exact 2D [`Profile2D`] with
//!   [`Extrude`] and [`Revolve`] into solids.
//! - **Composition** ([`Shape`]): an `Arc<dyn Sdf>` handle for cheap runtime
//!   composition of any of the above.
//! - **Meshing**: uniform-grid dual contouring ([`mesh_sdf`],
//!   [`mesh_sdf_indexed`]) and adaptive-octree dual contouring with QEF sharp
//!   features ([`mesh_sdf_adaptive`]). Both produce watertight, manifold
//!   meshes.
//! - **Refinement** ([`refine`]): post-meshing pass ([`refine_mesh`]) that
//!   snaps feature vertices onto the analytic CSG intersection curves (via
//!   [`Sdf::branches`](primitives::Sdf::branches)) and regularizes the
//!   triangulation with feature-aware tangential smoothing, Delaunay edge
//!   flips, and sliver collapse.
//!
//! This crate is the robust fast path of the hybrid kernel; the exact B-Rep
//! side lives in `opensolid-brep`, and `opensolid-kernel` bridges the two.

pub mod blend;
pub mod csg;
pub mod eval;
pub mod fillet;
pub mod mesh;
pub mod mesh_adaptive;
pub mod ops;
pub mod primitives;
pub mod profile;
pub mod refine;
pub mod shape;
pub mod sweep;
#[cfg(test)]
pub(crate) mod test_util;
pub mod transform;

pub use fillet::{BlendMode, BooleanKind, EdgeBlend, EdgeRegion};
pub use mesh::{MeshOptions, Triangle, TriangleMesh, mesh_sdf, mesh_sdf_indexed};
pub use mesh_adaptive::{AdaptiveMeshOptions, mesh_sdf_adaptive, mesh_sdf_adaptive_indexed};
pub use ops::{Offset, Rounded, SdfOpsExt, Shell};
pub use profile::{Extrude, MAX_DRAFT, OpenPath2D, Profile2D, Revolve, Rib, RibSide};
pub use refine::{RefineOptions, refine_mesh};
pub use shape::Shape;
pub use sweep::{Loft, Sweep};
pub use transform::{AnisotropicScale, SdfTransformExt, Taper, Transformed, UniformScale};
