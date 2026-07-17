//! The scalar abstraction the differentiable field tower is written against.
//!
//! Field code ([`super::field`]) is written once over `T: Scalar` and then
//! instantiated twice: at `f64` it is an ordinary evaluation, at
//! [`Dual<N>`](super::Dual) the same code also carries derivatives with
//! respect to `N` design parameters. There is no second implementation of the
//! geometry to drift out of sync — that is the whole point of the generic.
//!
//! This is deliberately *not* `num_traits::Float`: the crate's dependency
//! budget is nalgebra + thiserror + rayon (see `ROADMAP.md`), and the tower
//! needs only the dozen operations below.

use std::ops::{Add, Div, Mul, Neg, Sub};

/// A real scalar that field code can be generic over.
///
/// Implemented for [`f64`] (plain evaluation) and
/// [`Dual<N>`](super::Dual) (evaluation plus forward-mode derivatives).
///
/// # Non-smooth operations
///
/// [`min`](Scalar::min), [`max`](Scalar::max), [`abs`](Scalar::abs) and
/// [`clamp`](Scalar::clamp) are not differentiable everywhere. Their
/// derivative on the kink is a *subgradient* — one of the one-sided
/// derivatives, chosen by a tie-break rule. This mirrors what
/// [`Sdf::grad`](crate::primitives::Sdf::grad) already promises for spatial
/// gradients on edges, and it is why sharp CSG gives gradients that are
/// correct almost everywhere but not on the seam. See
/// `docs/design/DIFFERENTIABLE.md` §3.
pub trait Scalar:
    Copy
    + Send
    + Sync
    + Add<Output = Self>
    + Sub<Output = Self>
    + Mul<Output = Self>
    + Div<Output = Self>
    + Neg<Output = Self>
    + PartialOrd
{
    /// A constant: value `x`, zero derivative.
    fn cst(x: f64) -> Self;

    /// The value, dropping any derivative information.
    fn val(self) -> f64;

    fn sqrt(self) -> Self;
    fn abs(self) -> Self;
    fn sin(self) -> Self;
    fn cos(self) -> Self;
    fn exp(self) -> Self;

    /// Natural log. Undefined at `x <= 0`, as for `f64`.
    fn ln(self) -> Self;

    /// Smaller of the two. Ties go to `self`.
    fn min(self, other: Self) -> Self {
        if other < self { other } else { self }
    }

    /// Larger of the two. Ties go to `self`.
    fn max(self, other: Self) -> Self {
        if other > self { other } else { self }
    }

    fn clamp(self, lo: f64, hi: f64) -> Self {
        self.max(Self::cst(lo)).min(Self::cst(hi))
    }

    fn zero() -> Self {
        Self::cst(0.0)
    }

    fn one() -> Self {
        Self::cst(1.0)
    }

    fn square(self) -> Self {
        self * self
    }

    /// `max(self, 0)` — the positive part.
    fn relu(self) -> Self {
        self.max(Self::zero())
    }
}

impl Scalar for f64 {
    fn cst(x: f64) -> Self {
        x
    }

    fn val(self) -> f64 {
        self
    }

    fn sqrt(self) -> Self {
        f64::sqrt(self)
    }

    fn abs(self) -> Self {
        f64::abs(self)
    }

    fn sin(self) -> Self {
        f64::sin(self)
    }

    fn cos(self) -> Self {
        f64::cos(self)
    }

    fn exp(self) -> Self {
        f64::exp(self)
    }

    fn ln(self) -> Self {
        f64::ln(self)
    }
}
