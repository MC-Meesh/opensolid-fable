//! Forward-mode dual numbers carrying `N` partial derivatives at once.

use super::scalar::Scalar;
use std::ops::{Add, Div, Mul, Neg, Sub};

/// Value plus its partial derivatives with respect to `N` parameters.
///
/// A dual number is `v + Σ dᵢ εᵢ` where the `εᵢ` are nilpotent
/// (`εᵢεⱼ = 0`). Propagating that algebra through a computation applies the
/// chain rule automatically, so evaluating a field at
/// [`seed`](Dual::seed)ed parameters yields the exact derivative — no step
/// size, no truncation error, no cancellation.
///
/// `N` partials ride along one evaluation, so a full gradient over `N`
/// parameters costs one forward pass with `N`-wide arithmetic. That is the
/// right trade at the parameter counts CAD scripts have (tens); reverse mode
/// wins only once `N` reaches the thousands. See
/// `docs/design/DIFFERENTIABLE.md` §2.
///
/// # Example
///
/// ```
/// use opensolid_frep::diff::{Dual, Scalar};
///
/// // f(x, y) = x² · y at (3, 4): value 36, ∂f/∂x = 2xy = 24, ∂f/∂y = x² = 9.
/// let x = Dual::<2>::seed(3.0, 0);
/// let y = Dual::<2>::seed(4.0, 1);
/// let f = x * x * y;
///
/// assert_eq!(f.val(), 36.0);
/// assert_eq!(f.grad(), [24.0, 9.0]);
/// ```
#[derive(Debug, Clone, Copy)]
pub struct Dual<const N: usize> {
    /// The value of the computation.
    pub v: f64,
    /// `d[i]` is the partial derivative with respect to parameter `i`.
    pub d: [f64; N],
}

impl<const N: usize> Dual<N> {
    /// A constant: value `v`, all partials zero.
    pub fn cst(v: f64) -> Self {
        Self { v, d: [0.0; N] }
    }

    /// An independent variable: value `v`, and `∂self/∂paramᵢ = 1`.
    ///
    /// Seeding parameter `i` is what makes the derivative *with respect to*
    /// that parameter come out the far end.
    ///
    /// # Panics
    ///
    /// If `i >= N`.
    pub fn seed(v: f64, i: usize) -> Self {
        assert!(i < N, "seed index {i} out of range for Dual<{N}>");
        let mut d = [0.0; N];
        d[i] = 1.0;
        Self { v, d }
    }

    /// Seed a whole parameter vector: `params[i]` becomes the variable `i`.
    pub fn seed_all(params: &[f64; N]) -> [Self; N] {
        std::array::from_fn(|i| Self::seed(params[i], i))
    }

    /// The accumulated gradient.
    pub fn grad(&self) -> [f64; N] {
        self.d
    }

    /// Combine value and partials, given `dself/dinner` at the value.
    ///
    /// Every unary rule below is this chain-rule application.
    fn chain(self, v: f64, dv: f64) -> Self {
        Self {
            v,
            d: std::array::from_fn(|i| dv * self.d[i]),
        }
    }
}

impl<const N: usize> Add for Dual<N> {
    type Output = Self;
    fn add(self, o: Self) -> Self {
        Self {
            v: self.v + o.v,
            d: std::array::from_fn(|i| self.d[i] + o.d[i]),
        }
    }
}

impl<const N: usize> Sub for Dual<N> {
    type Output = Self;
    fn sub(self, o: Self) -> Self {
        Self {
            v: self.v - o.v,
            d: std::array::from_fn(|i| self.d[i] - o.d[i]),
        }
    }
}

// `suspicious_arithmetic_impl` flags the `+` inside `Mul`, on the theory it
// is a copy-paste slip. Here it is the product rule: the derivative of a
// product is a sum, and that is the entire point.
#[allow(clippy::suspicious_arithmetic_impl)]
impl<const N: usize> Mul for Dual<N> {
    type Output = Self;
    /// Product rule: `(uv)' = u'v + uv'`.
    fn mul(self, o: Self) -> Self {
        Self {
            v: self.v * o.v,
            d: std::array::from_fn(|i| self.d[i] * o.v + self.v * o.d[i]),
        }
    }
}

impl<const N: usize> Div for Dual<N> {
    type Output = Self;
    /// Quotient rule: `(u/v)' = (u'v - uv') / v²`.
    fn div(self, o: Self) -> Self {
        let inv = 1.0 / o.v;
        let inv2 = inv * inv;
        Self {
            v: self.v * inv,
            d: std::array::from_fn(|i| (self.d[i] * o.v - self.v * o.d[i]) * inv2),
        }
    }
}

impl<const N: usize> Neg for Dual<N> {
    type Output = Self;
    fn neg(self) -> Self {
        Self {
            v: -self.v,
            d: std::array::from_fn(|i| -self.d[i]),
        }
    }
}

/// Compares **values only** — the derivative is not part of the order.
///
/// This is what makes `min`/`max` select a branch by field value and carry
/// that branch's derivative along, which is exactly the subgradient rule
/// sharp CSG needs.
impl<const N: usize> PartialEq for Dual<N> {
    fn eq(&self, o: &Self) -> bool {
        self.v == o.v
    }
}

impl<const N: usize> PartialOrd for Dual<N> {
    fn partial_cmp(&self, o: &Self) -> Option<std::cmp::Ordering> {
        self.v.partial_cmp(&o.v)
    }
}

/// Derivative of `sqrt` blows up at 0 (the cone tip of a distance field).
/// Below this the partials are reported as zero rather than infinite, so a
/// point sampled exactly on a degenerate locus yields a finite (if
/// arbitrary) subgradient instead of poisoning the whole gradient with NaN.
const SQRT_EPS: f64 = 1e-300;

impl<const N: usize> Scalar for Dual<N> {
    fn cst(x: f64) -> Self {
        Dual::cst(x)
    }

    fn val(self) -> f64 {
        self.v
    }

    /// `d(√u) = u' / (2√u)`.
    fn sqrt(self) -> Self {
        let s = self.v.sqrt();
        if s < SQRT_EPS {
            return Self { v: s, d: [0.0; N] };
        }
        self.chain(s, 0.5 / s)
    }

    /// `d(|u|) = sign(u)·u'`. At 0 the subgradient 0 is returned.
    fn abs(self) -> Self {
        let s = if self.v > 0.0 {
            1.0
        } else if self.v < 0.0 {
            -1.0
        } else {
            0.0
        };
        self.chain(self.v.abs(), s)
    }

    fn sin(self) -> Self {
        self.chain(self.v.sin(), self.v.cos())
    }

    fn cos(self) -> Self {
        self.chain(self.v.cos(), -self.v.sin())
    }

    /// `d(eᵘ) = eᵘ·u'`.
    fn exp(self) -> Self {
        let e = self.v.exp();
        self.chain(e, e)
    }

    /// `d(ln u) = u' / u`.
    fn ln(self) -> Self {
        self.chain(self.v.ln(), 1.0 / self.v)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Central finite difference of a scalar function of one parameter.
    fn fd(f: impl Fn(f64) -> f64, x: f64) -> f64 {
        let h = 1e-6;
        (f(x + h) - f(x - h)) / (2.0 * h)
    }

    #[test]
    fn seed_is_the_identity_variable() {
        let x = Dual::<1>::seed(2.0, 0);
        assert_eq!(x.val(), 2.0);
        assert_eq!(x.grad(), [1.0]);
    }

    #[test]
    fn constant_has_zero_derivative() {
        let c = Dual::<2>::cst(5.0);
        assert_eq!(c.grad(), [0.0, 0.0]);
    }

    #[test]
    fn seed_all_seeds_each_index() {
        let p = Dual::<3>::seed_all(&[1.0, 2.0, 3.0]);
        assert_eq!(p[0].grad(), [1.0, 0.0, 0.0]);
        assert_eq!(p[1].grad(), [0.0, 1.0, 0.0]);
        assert_eq!(p[2].val(), 3.0);
    }

    #[test]
    #[should_panic(expected = "out of range")]
    fn seed_beyond_arity_panics() {
        let _ = Dual::<2>::seed(1.0, 2);
    }

    #[test]
    fn product_rule() {
        let x = Dual::<1>::seed(3.0, 0);
        let f = x * x; // d(x²) = 2x = 6
        assert_eq!(f.val(), 9.0);
        assert!((f.grad()[0] - 6.0).abs() < 1e-12);
    }

    #[test]
    fn quotient_rule() {
        // f(x) = (x + 1) / x at 2 → derivative -1/x² = -0.25
        let x = Dual::<1>::seed(2.0, 0);
        let f = (x + Dual::cst(1.0)) / x;
        assert!((f.val() - 1.5).abs() < 1e-12);
        assert!((f.grad()[0] - (-0.25)).abs() < 1e-12);
    }

    #[test]
    fn partials_are_independent() {
        // f(x, y) = x·y + y²  → ∂x = y = 4, ∂y = x + 2y = 11
        let x = Dual::<2>::seed(3.0, 0);
        let y = Dual::<2>::seed(4.0, 1);
        let f = x * y + y * y;
        assert_eq!(f.val(), 28.0);
        assert!((f.grad()[0] - 4.0).abs() < 1e-12);
        assert!((f.grad()[1] - 11.0).abs() < 1e-12);
    }

    #[test]
    fn sqrt_matches_fd() {
        let x = Dual::<1>::seed(2.0, 0);
        assert!((x.sqrt().grad()[0] - fd(f64::sqrt, 2.0)).abs() < 1e-8);
    }

    #[test]
    fn sqrt_at_zero_is_finite() {
        let g = Dual::<1>::seed(0.0, 0).sqrt().grad()[0];
        assert!(g.is_finite(), "sqrt at 0 must not produce inf/NaN");
    }

    #[test]
    fn abs_picks_sign_and_is_zero_at_kink() {
        assert_eq!(Dual::<1>::seed(-3.0, 0).abs().grad(), [-1.0]);
        assert_eq!(Dual::<1>::seed(3.0, 0).abs().grad(), [1.0]);
        assert_eq!(Dual::<1>::seed(0.0, 0).abs().grad(), [0.0]);
    }

    #[test]
    fn trig_matches_fd() {
        let x = Dual::<1>::seed(0.7, 0);
        assert!((x.sin().grad()[0] - fd(f64::sin, 0.7)).abs() < 1e-8);
        assert!((x.cos().grad()[0] - fd(f64::cos, 0.7)).abs() < 1e-8);
    }

    #[test]
    fn min_max_select_the_winning_branch_derivative() {
        let x = Dual::<2>::seed(1.0, 0);
        let y = Dual::<2>::seed(2.0, 1);
        // min picks x, so the gradient is x's: [1, 0].
        assert_eq!(x.min(y).grad(), [1.0, 0.0]);
        // max picks y, so the gradient is y's: [0, 1].
        assert_eq!(x.max(y).grad(), [0.0, 1.0]);
    }

    #[test]
    fn ordering_ignores_derivatives() {
        let a = Dual::<1> { v: 1.0, d: [7.0] };
        let b = Dual::<1> { v: 1.0, d: [9.0] };
        assert!(a == b);
        // Equal values compare Equal however the derivatives differ.
        assert_eq!(a.partial_cmp(&b), Some(std::cmp::Ordering::Equal));
    }

    #[test]
    fn clamp_saturates_and_kills_derivative() {
        let x = Dual::<1>::seed(5.0, 0);
        let c = x.clamp(0.0, 1.0);
        assert_eq!(c.val(), 1.0);
        // Outside the range the output no longer depends on x.
        assert_eq!(c.grad(), [0.0]);
    }

    #[test]
    fn chain_of_ops_matches_fd() {
        // f(x) = √(x² + 1) · sin(x)
        let f = |x: f64| (x * x + 1.0).sqrt() * x.sin();
        let x = Dual::<1>::seed(1.3, 0);
        let d = ((x * x + Dual::cst(1.0)).sqrt() * x.sin()).grad()[0];
        assert!((d - fd(f, 1.3)).abs() < 1e-7);
    }
}
