//! Differentiable F-Rep: parameter gradients by forward-mode AD.
//!
//! [`Sdf::grad`](crate::primitives::Sdf::grad) answers *"which way is out?"* —
//! the gradient with respect to the **sample point**, which is what meshing
//! needs. This module answers a different question: *"which way should I move
//! the design?"* — the gradient with respect to the **design parameters**
//! (`∂f/∂radius`, `∂volume/∂thickness`). That is the derivative optimisation
//! needs, and it is the seed of gradient-based part optimisation.
//!
//! # How it fits together
//!
//! - [`Scalar`] — the abstraction field code is generic over.
//! - [`Dual`] — value + `N` partials; forward-mode AD.
//! - [`Vec3`] — a 3-vector over any `Scalar`.
//! - [`field`] — every primitive and operator, written once over `T: Scalar`.
//! - [`ParamSdf`] — a shape as a function of `N` design parameters; gives
//!   exact `∂f/∂θ` and [`freeze`](ParamSdf::freeze)s back into an ordinary
//!   [`Sdf`] for meshing.
//! - [`objective`] — differentiable volume, mass, centre of gravity and
//!   clearance, computed **on the field** without meshing.
//! - [`optimize`] — projected gradient descent over box-bounded parameters.
//!
//! # Why the point is a constant
//!
//! Only the parameters are seeded as duals; the sample point is lifted as a
//! constant ([`Vec3::from_point`]). We differentiate *the design*, not *where
//! we sampled it*. This keeps `Point3` (a nalgebra alias pinned at `f64`)
//! untouched and the existing [`Sdf`](crate::primitives::Sdf) object-safe.
//!
//! # Example
//!
//! Optimise a sphere's radius to hit a target volume:
//!
//! ```
//! use opensolid_frep::diff::{
//!     objective::{Occupancy, volume},
//!     optimize::{Bounds, descend, DescentOptions},
//!     ParamSdf, Scalar, Vec3, field,
//! };
//! use opensolid_core::types::BoundingBox3;
//! use opensolid_core::types::Point3;
//!
//! struct Ball;
//! impl ParamSdf<1> for Ball {
//!     fn field<T: Scalar>(&self, p: Vec3<T>, params: &[T; 1]) -> T {
//!         field::sphere(p, Vec3::zero(), params[0])
//!     }
//! }
//!
//! let domain = BoundingBox3::new(Point3::new(-3.0, -3.0, -3.0), Point3::new(3.0, 3.0, 3.0));
//! let occ = Occupancy::new(48, 0.12);
//! let target = 14.0; // between the volumes of r=1.4 and r=1.6
//!
//! // Least-squares miss against the target volume, and its parameter gradient.
//! let loss = |p: &[f64; 1]| {
//!     let (v, g) = volume(&Ball, p, &domain, &occ);
//!     let e = v - target;
//!     (e * e, [2.0 * e * g[0]])
//! };
//!
//! let start = [1.0];
//! let result = descend(loss, start, &Bounds::new([0.2], [3.0]), &DescentOptions::default());
//! let (v_end, _) = volume(&Ball, &result.params, &domain, &occ);
//!
//! // Converged onto the target volume from a radius that badly missed it.
//! let (v_start, _) = volume(&Ball, &start, &domain, &occ);
//! assert!((v_end - target).abs() < (v_start - target).abs() * 0.1);
//! ```

mod dual;
pub mod field;
pub mod objective;
pub mod optimize;
mod param;
mod scalar;
mod vec;

pub use dual::Dual;
pub use param::{Frozen, ParamSdf};
pub use scalar::Scalar;
pub use vec::Vec3;
