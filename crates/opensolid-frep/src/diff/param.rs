//! Parametric shapes: fields that depend on a design parameter vector, and
//! the machinery to differentiate and to mesh them.

use super::dual::Dual;
use super::scalar::Scalar;
use super::vec::Vec3;
use crate::primitives::Sdf;
use opensolid_core::types::Point3;

/// A shape whose field depends on `N` design parameters.
///
/// The single required method is written **once**, generic over the scalar,
/// using the [`field`](super::field) tower. Evaluating it at `f64` gives the
/// field; evaluating it at [`Dual<N>`] gives the field *and* its derivatives
/// with respect to every parameter, from the same source.
///
/// The generic method means this trait is not object-safe — deliberately.
/// The existing [`Sdf`] stays object-safe (`Shape` is an `Arc<dyn Sdf>` and
/// the whole mesher relies on that); this parallel trait carries the scalar
/// generic instead, and [`freeze`](ParamSdf::freeze) bridges back. See
/// `docs/design/DIFFERENTIABLE.md` §2.
///
/// # Example
///
/// ```
/// use opensolid_frep::diff::{ParamSdf, Scalar, Vec3, field};
///
/// /// A sphere whose radius is the one design parameter.
/// struct Ball;
///
/// impl ParamSdf<1> for Ball {
///     fn field<T: Scalar>(&self, p: Vec3<T>, params: &[T; 1]) -> T {
///         field::sphere(p, Vec3::zero(), params[0])
///     }
/// }
///
/// use opensolid_core::types::Point3;
/// let p = Point3::new(2.0, 0.0, 0.0);
///
/// // Value: distance from the surface of a radius-1 ball.
/// assert!((Ball.eval(&p, &[1.0]) - 1.0).abs() < 1e-12);
///
/// // Derivative: growing the radius by 1 moves the surface 1 closer.
/// let (v, g) = Ball.value_and_grad(&p, &[1.0]);
/// assert!((v - 1.0).abs() < 1e-12);
/// assert!((g[0] - (-1.0)).abs() < 1e-12);
/// ```
pub trait ParamSdf<const N: usize>: Send + Sync {
    /// The field at `p` for parameters `params`, generic over the scalar.
    fn field<T: Scalar>(&self, p: Vec3<T>, params: &[T; N]) -> T;

    /// Human-readable parameter names, for diagnostics and reporting.
    ///
    /// The default is `θ0..θ{N-1}`; override to get readable optimiser logs.
    fn param_names(&self) -> [&'static str; N] {
        const NAMES: [&str; 16] = [
            "θ0", "θ1", "θ2", "θ3", "θ4", "θ5", "θ6", "θ7", "θ8", "θ9", "θ10", "θ11", "θ12", "θ13",
            "θ14", "θ15",
        ];
        std::array::from_fn(|i| if i < NAMES.len() { NAMES[i] } else { "θ?" })
    }

    /// The field value at `p` — the tower instantiated at `f64`.
    fn eval(&self, p: &Point3, params: &[f64; N]) -> f64 {
        self.field(Vec3::from_point(p), params)
    }

    /// The field value at `p` **and** `∂f/∂θᵢ` for every parameter, from a
    /// single forward pass with `N`-wide dual arithmetic.
    ///
    /// This is exact to machine precision: no step size, no truncation
    /// error. Contrast [`grad_fd`](ParamSdf::grad_fd).
    fn value_and_grad(&self, p: &Point3, params: &[f64; N]) -> (f64, [f64; N]) {
        let seeded = Dual::<N>::seed_all(params);
        let out = self.field(Vec3::from_point(p), &seeded);
        (out.v, out.d)
    }

    /// Central finite-difference parameter gradient. Reference implementation
    /// used to test [`value_and_grad`](ParamSdf::value_and_grad); real code
    /// should use the dual path, which is both exact and `N`× cheaper.
    fn grad_fd(&self, p: &Point3, params: &[f64; N], h: f64) -> [f64; N] {
        std::array::from_fn(|i| {
            let mut lo = *params;
            let mut hi = *params;
            lo[i] -= h;
            hi[i] += h;
            (self.eval(p, &hi) - self.eval(p, &lo)) / (2.0 * h)
        })
    }

    /// Pin the parameters, yielding an ordinary [`Sdf`].
    ///
    /// This is the bridge to the rest of the kernel: a frozen parametric
    /// shape meshes, renders and exports through the existing pipeline with
    /// no special cases, so an optimiser's output is an ordinary part.
    fn freeze(&self, params: [f64; N]) -> Frozen<'_, Self, N> {
        Frozen {
            shape: self,
            params,
        }
    }
}

/// A [`ParamSdf`] with its parameters pinned — an ordinary [`Sdf`].
///
/// Borrows the shape, so it is free to construct inside an optimiser loop.
pub struct Frozen<'a, S: ?Sized, const N: usize> {
    shape: &'a S,
    params: [f64; N],
}

impl<S: ?Sized, const N: usize> Frozen<'_, S, N> {
    /// The parameters this shape is pinned at.
    pub fn params(&self) -> &[f64; N] {
        &self.params
    }
}

/// Uses the trait's finite-difference `grad` and the Lipschitz
/// `eval_interval` default. Both are sound for the fields the tower builds
/// (sharp/smooth CSG over exact primitives is 1-Lipschitz), but note the
/// *spatial* gradient here is still FD — it is a different derivative from
/// the exact *parameter* gradient [`ParamSdf::value_and_grad`] returns.
impl<S: ParamSdf<N> + ?Sized, const N: usize> Sdf for Frozen<'_, S, N> {
    fn eval(&self, p: &Point3) -> f64 {
        self.shape.eval(p, &self.params)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::diff::field;

    /// Unit-ish test shape: a sphere of parameter radius, translated along x
    /// by a second parameter.
    struct MovingBall;

    impl ParamSdf<2> for MovingBall {
        fn field<T: Scalar>(&self, p: Vec3<T>, params: &[T; 2]) -> T {
            let center = Vec3::new(params[1], T::zero(), T::zero());
            field::sphere(p, center, params[0])
        }

        fn param_names(&self) -> [&'static str; 2] {
            ["radius", "x"]
        }
    }

    #[test]
    fn eval_matches_hand_computation() {
        let p = Point3::new(5.0, 0.0, 0.0);
        // |5 - 1| - 2 = 2
        assert!((MovingBall.eval(&p, &[2.0, 1.0]) - 2.0).abs() < 1e-12);
    }

    #[test]
    fn grad_matches_hand_derivation() {
        let p = Point3::new(5.0, 0.0, 0.0);
        let (v, g) = MovingBall.value_and_grad(&p, &[2.0, 1.0]);
        assert!((v - 2.0).abs() < 1e-12);
        // d/dr = -1; d/dx = -1 (moving the ball toward p closes the gap).
        assert!((g[0] - (-1.0)).abs() < 1e-9);
        assert!((g[1] - (-1.0)).abs() < 1e-9);
    }

    #[test]
    fn dual_grad_agrees_with_finite_differences() {
        let p = Point3::new(5.0, 1.0, -2.0);
        let params = [2.0, 1.0];
        let (_, g) = MovingBall.value_and_grad(&p, &params);
        let fd = MovingBall.grad_fd(&p, &params, 1e-6);
        for i in 0..2 {
            assert!(
                (g[i] - fd[i]).abs() < 1e-6,
                "param {i}: {} vs {}",
                g[i],
                fd[i]
            );
        }
    }

    #[test]
    fn param_names_default_and_override() {
        assert_eq!(MovingBall.param_names(), ["radius", "x"]);

        struct Unnamed;
        impl ParamSdf<2> for Unnamed {
            fn field<T: Scalar>(&self, _p: Vec3<T>, params: &[T; 2]) -> T {
                params[0]
            }
        }
        assert_eq!(Unnamed.param_names(), ["θ0", "θ1"]);
    }

    #[test]
    fn frozen_is_an_ordinary_sdf() {
        let frozen = MovingBall.freeze([2.0, 1.0]);
        let p = Point3::new(5.0, 0.0, 0.0);
        // Reached through the `Sdf` trait, not `ParamSdf`.
        assert!((Sdf::eval(&frozen, &p) - 2.0).abs() < 1e-12);
        assert_eq!(frozen.params(), &[2.0, 1.0]);
    }

    #[test]
    fn frozen_spatial_gradient_points_outward() {
        let frozen = MovingBall.freeze([2.0, 0.0]);
        // On the +x surface the outward normal is +x.
        let g = Sdf::grad(&frozen, &Point3::new(2.0, 0.0, 0.0));
        assert!((g.x - 1.0).abs() < 1e-4, "got {g:?}");
        assert!(g.y.abs() < 1e-4 && g.z.abs() < 1e-4);
    }

    #[test]
    fn frozen_meshes_through_the_existing_pipeline() {
        use crate::mesh::{MeshOptions, mesh_sdf};
        use opensolid_core::types::BoundingBox3;

        let frozen = MovingBall.freeze([1.0, 0.0]);
        let tris = mesh_sdf(
            &frozen,
            &MeshOptions {
                bounds: BoundingBox3::new(
                    Point3::new(-2.0, -2.0, -2.0),
                    Point3::new(2.0, 2.0, 2.0),
                ),
                resolution: 24,
            },
        );
        assert!(
            !tris.is_empty(),
            "a frozen parametric shape must mesh like any other Sdf"
        );
    }
}
