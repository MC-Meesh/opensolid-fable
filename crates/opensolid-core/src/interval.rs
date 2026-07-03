//! Closed interval arithmetic over `f64`.
//!
//! The foundation for interval SDF evaluation (octree pruning) and robust
//! SSI predicates: every operation is *conservative* — the result interval
//! contains `op(x, y)` for all `x` and `y` in the operand intervals.
//!
//! Design notes:
//! - An [`Interval`] is always non-empty (`lo <= hi`); a single point is the
//!   degenerate case `lo == hi`. The empty set has no representation —
//!   operations that can produce it (e.g. [`Interval::intersection`],
//!   [`Interval::sqrt`]) return `Option<Interval>` with `None` for empty.
//! - Division by an interval containing zero returns `None` rather than the
//!   whole line: an unbounded result poisons downstream arithmetic with
//!   `inf * 0 = NaN`, whereas `None` forces the caller (a predicate or a
//!   pruner) to handle the singularity explicitly.
//! - Endpoints use round-to-nearest, not directed outward rounding, so
//!   results can under-cover by 1 ulp at the boundary. That is tight enough
//!   for pruning and tolerance-band predicates (`spec/08-tolerances.md`);
//!   exact-arithmetic escalation is a separate concern.
//! - Endpoints must be finite or infinite, never NaN; `lo <= hi` is the
//!   caller's obligation on [`Interval::new`] (debug-asserted).

/// A closed, non-empty interval `[lo, hi]`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Interval {
    pub lo: f64,
    pub hi: f64,
}

impl Interval {
    /// The whole extended real line.
    pub const WHOLE: Interval = Interval {
        lo: f64::NEG_INFINITY,
        hi: f64::INFINITY,
    };

    /// `[lo, hi]`. `lo <= hi` is a caller obligation (debug-asserted);
    /// passing NaN or a reversed pair is a bug, not a recoverable state.
    pub fn new(lo: f64, hi: f64) -> Self {
        debug_assert!(lo <= hi, "invalid interval [{lo}, {hi}]");
        Self { lo, hi }
    }

    /// The degenerate interval `[v, v]`.
    pub fn point(v: f64) -> Self {
        Self::new(v, v)
    }

    /// Interval spanning `a` and `b` in either order.
    pub fn from_unordered(a: f64, b: f64) -> Self {
        Self::new(a.min(b), a.max(b))
    }

    pub fn width(&self) -> f64 {
        self.hi - self.lo
    }

    pub fn midpoint(&self) -> f64 {
        0.5 * (self.lo + self.hi)
    }

    pub fn contains(&self, v: f64) -> bool {
        self.lo <= v && v <= self.hi
    }

    pub fn contains_zero(&self) -> bool {
        self.contains(0.0)
    }

    /// True if `other` lies entirely within `self`.
    pub fn contains_interval(&self, other: &Interval) -> bool {
        self.lo <= other.lo && other.hi <= self.hi
    }

    /// Smallest interval containing both operands.
    pub fn hull(&self, other: &Interval) -> Interval {
        Interval::new(self.lo.min(other.lo), self.hi.max(other.hi))
    }

    /// Overlap of the two intervals; `None` if they are disjoint.
    pub fn intersection(&self, other: &Interval) -> Option<Interval> {
        let lo = self.lo.max(other.lo);
        let hi = self.hi.min(other.hi);
        (lo <= hi).then(|| Interval::new(lo, hi))
    }

    /// Pointwise minimum: contains `x.min(y)` for all `x` in `self`,
    /// `y` in `other`.
    pub fn min(&self, other: &Interval) -> Interval {
        Interval::new(self.lo.min(other.lo), self.hi.min(other.hi))
    }

    /// Pointwise maximum: contains `x.max(y)` for all `x` in `self`,
    /// `y` in `other`.
    pub fn max(&self, other: &Interval) -> Interval {
        Interval::new(self.lo.max(other.lo), self.hi.max(other.hi))
    }

    /// Contains `|x|` for all `x` in `self`.
    pub fn abs(&self) -> Interval {
        if self.lo >= 0.0 {
            *self
        } else if self.hi <= 0.0 {
            Interval::new(-self.hi, -self.lo)
        } else {
            Interval::new(0.0, (-self.lo).max(self.hi))
        }
    }

    /// Contains `x * x` for all `x` in `self`. Tighter than `self * self`,
    /// which treats the operands as independent (e.g. `[-2, 3] * [-2, 3]`
    /// is `[-6, 9]`, but no single `x` squares to a negative value).
    pub fn square(&self) -> Interval {
        let a = self.abs();
        Interval::new(a.lo * a.lo, a.hi * a.hi)
    }

    /// Contains `sqrt(x)` for the part of `self` where it is defined.
    /// `None` if the interval is entirely negative. A straddling interval
    /// is clamped to its non-negative part first.
    pub fn sqrt(&self) -> Option<Interval> {
        if self.hi < 0.0 {
            return None;
        }
        Some(Interval::new(self.lo.max(0.0).sqrt(), self.hi.sqrt()))
    }

    /// Interval division. `None` when the divisor contains zero (see the
    /// module docs for why we refuse rather than widen to the whole line).
    pub fn div(&self, other: &Interval) -> Option<Interval> {
        if other.contains_zero() {
            return None;
        }
        Some(*self * Interval::from_unordered(1.0 / other.lo, 1.0 / other.hi))
    }
}

impl std::ops::Neg for Interval {
    type Output = Interval;
    fn neg(self) -> Interval {
        Interval::new(-self.hi, -self.lo)
    }
}

impl std::ops::Add for Interval {
    type Output = Interval;
    fn add(self, rhs: Interval) -> Interval {
        Interval::new(self.lo + rhs.lo, self.hi + rhs.hi)
    }
}

impl std::ops::Sub for Interval {
    type Output = Interval;
    fn sub(self, rhs: Interval) -> Interval {
        Interval::new(self.lo - rhs.hi, self.hi - rhs.lo)
    }
}

impl std::ops::Mul for Interval {
    type Output = Interval;
    fn mul(self, rhs: Interval) -> Interval {
        let products = [
            self.lo * rhs.lo,
            self.lo * rhs.hi,
            self.hi * rhs.lo,
            self.hi * rhs.hi,
        ];
        let mut lo = products[0];
        let mut hi = products[0];
        for &p in &products[1..] {
            lo = lo.min(p);
            hi = hi.max(p);
        }
        Interval::new(lo, hi)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn iv(lo: f64, hi: f64) -> Interval {
        Interval::new(lo, hi)
    }

    /// Evenly spaced sample points across an interval, endpoints included.
    fn samples(i: &Interval) -> Vec<f64> {
        const STEPS: usize = 8;
        (0..=STEPS)
            .map(|s| i.lo + i.width() * (s as f64 / STEPS as f64))
            .collect()
    }

    fn cases() -> Vec<Interval> {
        vec![
            iv(-3.0, -1.0),
            iv(-2.0, 0.0),
            iv(-1.5, 2.5),
            iv(0.0, 0.0),
            iv(0.0, 1.0),
            iv(0.25, 4.0),
            iv(5.0, 5.0),
        ]
    }

    #[test]
    fn accessors_and_predicates() {
        let i = iv(-1.0, 3.0);
        assert_eq!(i.width(), 4.0);
        assert_eq!(i.midpoint(), 1.0);
        assert!(i.contains(-1.0) && i.contains(3.0) && i.contains(0.5));
        assert!(!i.contains(3.0001));
        assert!(i.contains_zero());
        assert!(!iv(1.0, 2.0).contains_zero());
        assert!(i.contains_interval(&iv(0.0, 1.0)));
        assert!(!i.contains_interval(&iv(0.0, 4.0)));
        assert_eq!(Interval::from_unordered(2.0, -1.0), iv(-1.0, 2.0));
    }

    #[test]
    fn degenerate_point_intervals_behave_like_scalars() {
        let two = Interval::point(2.0);
        let three = Interval::point(3.0);
        assert_eq!(two.width(), 0.0);
        assert_eq!(two.midpoint(), 2.0);
        assert_eq!(two + three, Interval::point(5.0));
        assert_eq!(two - three, Interval::point(-1.0));
        assert_eq!(two * three, Interval::point(6.0));
        assert_eq!(two.div(&three), Some(Interval::point(2.0 / 3.0)));
        assert_eq!(two.square(), Interval::point(4.0));
        assert_eq!((-two).abs(), two);
    }

    #[test]
    fn arithmetic_identities() {
        for a in cases() {
            // x - x and x + (-x) always contain 0 (they are not point zero:
            // interval arithmetic treats the operands as independent).
            assert!((a - a).contains_zero());
            assert!((a + (-a)).contains_zero());
            assert_eq!(-(-a), a);
            // Identity elements.
            assert_eq!(a + Interval::point(0.0), a);
            assert_eq!(a * Interval::point(1.0), a);
            for b in cases() {
                assert_eq!(a + b, b + a);
                assert_eq!(a * b, b * a);
                assert_eq!(a.hull(&b), b.hull(&a));
                assert_eq!(a.min(&b), b.min(&a));
                assert_eq!(a.max(&b), b.max(&a));
            }
        }
    }

    #[test]
    fn hull_and_intersection() {
        assert_eq!(iv(-1.0, 0.5).hull(&iv(2.0, 3.0)), iv(-1.0, 3.0));
        assert_eq!(
            iv(-1.0, 2.0).intersection(&iv(1.0, 3.0)),
            Some(iv(1.0, 2.0))
        );
        // Touching endpoints intersect in a degenerate point.
        assert_eq!(
            iv(-1.0, 1.0).intersection(&iv(1.0, 2.0)),
            Some(Interval::point(1.0))
        );
        // Disjoint intervals: the empty set is represented as None.
        assert_eq!(iv(-1.0, 0.9).intersection(&iv(1.0, 2.0)), None);
        for a in cases() {
            for b in cases() {
                assert!(a.hull(&b).contains_interval(&a));
                assert!(a.hull(&b).contains_interval(&b));
                if let Some(x) = a.intersection(&b) {
                    assert!(a.contains_interval(&x) && b.contains_interval(&x));
                }
            }
        }
    }

    #[test]
    fn division_by_zero_straddling_interval_is_none() {
        let a = iv(1.0, 2.0);
        assert_eq!(a.div(&iv(-1.0, 1.0)), None);
        assert_eq!(a.div(&iv(0.0, 1.0)), None); // zero endpoint counts
        assert_eq!(a.div(&iv(-1.0, 0.0)), None);
        assert_eq!(a.div(&Interval::point(0.0)), None);
        assert!(a.div(&iv(0.5, 1.0)).is_some());
        assert!(a.div(&iv(-2.0, -0.5)).is_some());
    }

    #[test]
    fn sqrt_domain_handling() {
        assert_eq!(iv(-4.0, -1.0).sqrt(), None);
        assert_eq!(iv(4.0, 9.0).sqrt(), Some(iv(2.0, 3.0)));
        // Straddling zero: clamped to the defined part.
        assert_eq!(iv(-1.0, 4.0).sqrt(), Some(iv(0.0, 2.0)));
    }

    #[test]
    fn square_is_tighter_than_self_mul() {
        let a = iv(-2.0, 3.0);
        assert_eq!(a.square(), iv(0.0, 9.0));
        assert_eq!(a * a, iv(-6.0, 9.0));
        assert!((a * a).contains_interval(&a.square()));
    }

    /// The fundamental theorem of interval arithmetic: for every pair of
    /// points inside the operands, the pointwise result lies inside the
    /// interval result.
    #[test]
    fn containment_property_binary_ops() {
        for a in cases() {
            for b in cases() {
                let sum = a + b;
                let diff = a - b;
                let prod = a * b;
                let quot = a.div(&b);
                let mn = a.min(&b);
                let mx = a.max(&b);
                for x in samples(&a) {
                    for y in samples(&b) {
                        assert!(sum.contains(x + y), "{x}+{y} not in {sum:?}");
                        assert!(diff.contains(x - y), "{x}-{y} not in {diff:?}");
                        assert!(prod.contains(x * y), "{x}*{y} not in {prod:?}");
                        if let Some(q) = quot {
                            assert!(q.contains(x / y), "{x}/{y} not in {q:?}");
                        }
                        assert!(mn.contains(x.min(y)));
                        assert!(mx.contains(x.max(y)));
                    }
                }
            }
        }
    }

    #[test]
    fn containment_property_unary_ops() {
        for a in cases() {
            let neg = -a;
            let abs = a.abs();
            let sq = a.square();
            let sqrt = a.sqrt();
            for x in samples(&a) {
                assert!(neg.contains(-x));
                assert!(abs.contains(x.abs()));
                assert!(sq.contains(x * x), "{x}^2 not in {sq:?}");
                if x >= 0.0 {
                    let s = sqrt.expect("interval reaching >= 0 must have a sqrt");
                    assert!(s.contains(x.sqrt()));
                }
            }
        }
    }

    /// Monotonicity (inclusion isotonicity): shrinking an operand can only
    /// shrink or keep the result.
    #[test]
    fn monotonicity_of_inclusion() {
        for a in cases() {
            for b in cases() {
                // Build a strict sub-interval of `a` around its midpoint.
                let sub = Interval::new(a.lo + 0.25 * a.width(), a.hi - 0.25 * a.width());
                assert!(a.contains_interval(&sub));
                assert!((a + b).contains_interval(&(sub + b)));
                assert!((a - b).contains_interval(&(sub - b)));
                assert!((a * b).contains_interval(&(sub * b)));
                assert!((a.min(&b)).contains_interval(&sub.min(&b)));
                assert!((a.max(&b)).contains_interval(&sub.max(&b)));
                assert!(a.abs().contains_interval(&sub.abs()));
                assert!(a.square().contains_interval(&sub.square()));
                if let (Some(d), Some(ds)) = (b.div(&a), b.div(&sub)) {
                    assert!(d.contains_interval(&ds));
                }
            }
        }
    }

    #[test]
    fn whole_line_absorbs_arithmetic() {
        let w = Interval::WHOLE;
        assert!(w.contains_zero());
        assert!(w.contains_interval(&iv(-1e300, 1e300)));
        assert_eq!(w + iv(-1.0, 1.0), w);
        assert_eq!(-w, w);
        // Division by an interval this wide contains zero → refused.
        assert_eq!(iv(1.0, 2.0).div(&w), None);
    }
}
