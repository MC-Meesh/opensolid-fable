//! Projected gradient descent over box-bounded design parameters.
//!
//! Deliberately small. The point of this module is to demonstrate that the
//! gradients from [`objective`](super::objective) are good enough to steer a
//! real design, not to be a competitive optimiser — a serious one (L-BFGS-B,
//! SLSQP, MMA) is a library, and the dependency budget does not stretch to
//! one. The interface is a plain closure returning `(loss, gradient)`, so
//! swapping in a better optimiser later touches nothing else.

/// Box constraints on the parameter vector — a `lo[i] <= θ[i] <= hi[i]` per
/// parameter.
///
/// Manufacturing bounds are not optional in CAD: a wall thickness of −3 mm
/// is not a worse design, it is not a design. Clamping every iterate keeps
/// the search inside the space of parts that can exist.
#[derive(Debug, Clone)]
pub struct Bounds<const N: usize> {
    pub lo: [f64; N],
    pub hi: [f64; N],
}

impl<const N: usize> Bounds<N> {
    /// # Panics
    ///
    /// If any `lo[i] > hi[i]`.
    pub fn new(lo: [f64; N], hi: [f64; N]) -> Self {
        for i in 0..N {
            assert!(lo[i] <= hi[i], "bound {i}: lo {} > hi {}", lo[i], hi[i]);
        }
        Self { lo, hi }
    }

    /// Unbounded in every parameter.
    pub fn unbounded() -> Self {
        Self {
            lo: [f64::NEG_INFINITY; N],
            hi: [f64::INFINITY; N],
        }
    }

    /// The nearest feasible point to `p`.
    pub fn project(&self, p: &[f64; N]) -> [f64; N] {
        std::array::from_fn(|i| p[i].clamp(self.lo[i], self.hi[i]))
    }
}

#[derive(Debug, Clone)]
pub struct DescentOptions {
    pub max_iters: usize,
    /// First trial step. The line search adapts from here.
    pub initial_step: f64,
    /// Stop once the projected step is shorter than this in every parameter.
    pub tol: f64,
    /// Armijo sufficient-decrease constant, in `(0, 1)`.
    pub armijo: f64,
    /// Backtracking factor, in `(0, 1)`.
    pub shrink: f64,
    /// Give up on a line search after this many shrinks.
    pub max_backtracks: usize,
    /// Heavy-ball momentum, in `[0, 1)`. `0` is plain steepest descent.
    ///
    /// Steepest descent zigzags across narrow valleys and creeps along them.
    /// Momentum accumulates the consistent along-valley component while the
    /// oscillating across-valley components cancel, taking the iteration
    /// count from `O(κ)` toward `O(√κ)` for condition number `κ`. On a 600:1
    /// valley this is worth more than 10× (see
    /// `momentum_beats_plain_descent_in_a_narrow_valley`).
    ///
    /// **What it does not fix:** an *active* inequality constraint under a
    /// quadratic penalty. There the iterate rides the penalty wall, the
    /// momentum step is rejected almost every iteration, adaptive restart
    /// dumps the velocity, and the method degenerates to plain descent —
    /// measured on the bracket problem, momentum moved the answer from
    /// -3.5% to -3.2% of the mass target, against a needed 0%. That case
    /// wants a real constrained method (SLSQP, MMA, augmented Lagrangian),
    /// not a better first-order step rule. See
    /// `docs/design/DIFFERENTIABLE.md` §6.
    pub momentum: f64,
}

impl Default for DescentOptions {
    fn default() -> Self {
        Self {
            max_iters: 200,
            initial_step: 0.1,
            tol: 1e-7,
            armijo: 1e-4,
            shrink: 0.5,
            max_backtracks: 40,
            momentum: 0.9,
        }
    }
}

#[derive(Debug, Clone)]
pub struct DescentResult<const N: usize> {
    pub params: [f64; N],
    pub loss: f64,
    pub iters: usize,
    /// True if the run stopped on [`DescentOptions::tol`] rather than on
    /// `max_iters` or a stalled line search.
    pub converged: bool,
    /// Loss at each accepted iterate, starting with the initial point —
    /// handy for asserting monotone progress in tests, and for plotting.
    pub history: Vec<f64>,
}

/// Minimise `loss` from `start`, staying inside `bounds`.
///
/// `loss` returns `(value, gradient)` — exactly the shape
/// [`volume`](super::objective::volume) and friends produce.
///
/// Heavy-ball momentum ([`DescentOptions::momentum`]) carries the velocity
/// between iterations; a backtracking Armijo line search sets how far to go
/// along it. Without the line search a fixed step overshoots the narrow band
/// where the occupancy gradient is supported and the run diverges; without
/// momentum a narrow valley takes O(κ) iterations instead of O(√κ). Neither
/// rescues an *active* constraint — see [`DescentOptions::momentum`].
///
/// **Adaptive restart:** if a momentum step fails to improve, the velocity is
/// dumped and the step retried as plain descent before shrinking. Momentum is
/// only a bet that the last direction still helps — when the bet is wrong
/// (the valley turned, or a bound was hit) the cheapest fix is to stop
/// betting rather than to keep halving a step that points the wrong way.
///
/// Steps are measured *after projection*, so a parameter pinned against a
/// bound does not read as continued progress and spin the loop.
pub fn descend<F, const N: usize>(
    loss: F,
    start: [f64; N],
    bounds: &Bounds<N>,
    opts: &DescentOptions,
) -> DescentResult<N>
where
    F: Fn(&[f64; N]) -> (f64, [f64; N]),
{
    let mut params = bounds.project(&start);
    let (mut value, mut grad) = loss(&params);
    let mut step = opts.initial_step;
    let mut velocity = [0.0; N];
    let mut history = vec![value];

    for iter in 0..opts.max_iters {
        let gnorm2: f64 = grad.iter().map(|g| g * g).sum();
        if gnorm2 == 0.0 {
            return DescentResult {
                params,
                loss: value,
                iters: iter,
                converged: true,
                history,
            };
        }

        // Try the momentum step; on failure fall back to plain descent, then
        // start shrinking. `beta = 0` collapses this to steepest descent.
        let mut accepted = None;
        'search: for _ in 0..opts.max_backtracks {
            for beta in [opts.momentum, 0.0] {
                let v: [f64; N] = std::array::from_fn(|i| beta * velocity[i] - step * grad[i]);
                let trial = bounds.project(&std::array::from_fn(|i| params[i] + v[i]));
                let (tv, tg) = loss(&trial);
                // Armijo against the *actual* displacement, which projection
                // may have shortened. `moved < 0` also screens out a momentum
                // step that is not a descent direction at all — heavy-ball
                // gives no such guarantee, and without this the condition
                // would wave through an uphill step.
                let moved: f64 = (0..N).map(|i| (trial[i] - params[i]) * grad[i]).sum();
                if moved < 0.0 && tv <= value + opts.armijo * moved {
                    accepted = Some((trial, tv, tg, v));
                    break 'search;
                }
                if beta == 0.0 {
                    // Even plain descent failed at this step length.
                    step *= opts.shrink;
                }
            }
        }

        let Some((trial, tv, tg, v)) = accepted else {
            // Line search stalled: no step along this gradient improves.
            return DescentResult {
                params,
                loss: value,
                iters: iter,
                converged: false,
                history,
            };
        };

        let moved_max = (0..N)
            .map(|i| (trial[i] - params[i]).abs())
            .fold(0.0, f64::max);

        params = trial;
        value = tv;
        grad = tg;
        velocity = v;
        history.push(value);

        if moved_max < opts.tol {
            return DescentResult {
                params,
                loss: value,
                iters: iter + 1,
                converged: true,
                history,
            };
        }

        // Creep back up: a successful step earns a slightly bolder next one.
        step /= opts.shrink.sqrt();
    }

    DescentResult {
        params,
        loss: value,
        iters: opts.max_iters,
        converged: false,
        history,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// f(x) = (x - 3)², minimised at 3.
    fn quadratic(p: &[f64; 1]) -> (f64, [f64; 1]) {
        let e = p[0] - 3.0;
        (e * e, [2.0 * e])
    }

    /// Rosenbrock — a curved valley that punishes a naive fixed step.
    fn rosenbrock(p: &[f64; 2]) -> (f64, [f64; 2]) {
        let (x, y) = (p[0], p[1]);
        let f = (1.0 - x).powi(2) + 100.0 * (y - x * x).powi(2);
        let dx = -2.0 * (1.0 - x) - 400.0 * x * (y - x * x);
        let dy = 200.0 * (y - x * x);
        (f, [dx, dy])
    }

    #[test]
    fn finds_the_minimum_of_a_quadratic() {
        let r = descend(
            quadratic,
            [0.0],
            &Bounds::unbounded(),
            &DescentOptions::default(),
        );
        assert!((r.params[0] - 3.0).abs() < 1e-4, "got {}", r.params[0]);
        assert!(r.converged);
    }

    #[test]
    fn loss_decreases_monotonically() {
        let r = descend(
            quadratic,
            [-5.0],
            &Bounds::unbounded(),
            &DescentOptions::default(),
        );
        for w in r.history.windows(2) {
            assert!(w[1] <= w[0] + 1e-12, "loss went up: {} → {}", w[0], w[1]);
        }
    }

    #[test]
    fn starting_at_the_optimum_stops_immediately() {
        let r = descend(
            quadratic,
            [3.0],
            &Bounds::unbounded(),
            &DescentOptions::default(),
        );
        assert!(r.converged);
        assert_eq!(r.iters, 0);
    }

    #[test]
    fn respects_box_bounds() {
        // The optimum is at 3 but the box forbids anything past 1.
        let r = descend(
            quadratic,
            [0.0],
            &Bounds::new([-10.0], [1.0]),
            &DescentOptions::default(),
        );
        assert!(
            r.params[0] <= 1.0 + 1e-12,
            "escaped the box: {}",
            r.params[0]
        );
        assert!((r.params[0] - 1.0).abs() < 1e-6, "should pin to the bound");
    }

    #[test]
    fn projects_an_infeasible_start() {
        let r = descend(
            quadratic,
            [99.0],
            &Bounds::new([-1.0], [1.0]),
            &DescentOptions::default(),
        );
        assert!(r.params[0] <= 1.0 + 1e-12);
    }

    #[test]
    fn line_search_survives_a_curved_valley() {
        let opts = DescentOptions {
            max_iters: 20_000,
            initial_step: 1e-3,
            ..Default::default()
        };
        let r = descend(rosenbrock, [-1.2, 1.0], &Bounds::unbounded(), &opts);
        // Plain gradient descent crawls along Rosenbrock's floor; we only
        // assert it makes real progress rather than diverging.
        assert!(
            r.loss < 0.1,
            "loss {} — line search failed to stabilise",
            r.loss
        );
    }

    /// An ill-conditioned quadratic — a narrow valley, the same shape an
    /// active constraint creates. This is what momentum exists for.
    fn valley(p: &[f64; 2]) -> (f64, [f64; 2]) {
        // Curvature 600:1, the measured conditioning of the bracket problem.
        let (a, b) = (600.0, 1.0);
        (
            a * p[0] * p[0] + b * p[1] * p[1],
            [2.0 * a * p[0], 2.0 * b * p[1]],
        )
    }

    #[test]
    fn momentum_beats_plain_descent_in_a_narrow_valley() {
        let run = |momentum| {
            let opts = DescentOptions {
                max_iters: 400,
                initial_step: 1e-3,
                tol: 1e-12,
                momentum,
                ..Default::default()
            };
            descend(valley, [1.0, 1.0], &Bounds::unbounded(), &opts).loss
        };
        let plain = run(0.0);
        let heavy = run(0.9);
        assert!(
            heavy < plain * 0.1,
            "momentum {heavy:e} should be far below plain descent {plain:e}"
        );
    }

    #[test]
    fn momentum_still_converges_on_a_well_conditioned_problem() {
        let r = descend(
            quadratic,
            [0.0],
            &Bounds::unbounded(),
            &DescentOptions::default(),
        );
        assert!((r.params[0] - 3.0).abs() < 1e-4, "got {}", r.params[0]);
    }

    #[test]
    fn momentum_run_is_still_monotone() {
        // Heavy-ball is not monotone in general; the accept rule makes it so.
        let opts = DescentOptions {
            max_iters: 400,
            initial_step: 1e-3,
            ..Default::default()
        };
        let r = descend(valley, [1.0, 1.0], &Bounds::unbounded(), &opts);
        for w in r.history.windows(2) {
            assert!(w[1] <= w[0] + 1e-12, "loss went up: {} → {}", w[0], w[1]);
        }
    }

    #[test]
    fn momentum_respects_bounds() {
        let opts = DescentOptions {
            momentum: 0.95,
            ..Default::default()
        };
        // Momentum could sail through a bound if projection were skipped.
        let r = descend(quadratic, [0.0], &Bounds::new([-10.0], [1.0]), &opts);
        assert!(
            r.params[0] <= 1.0 + 1e-12,
            "momentum escaped the box: {}",
            r.params[0]
        );
    }

    #[test]
    fn bounds_project_clamps_each_axis() {
        let b = Bounds::new([0.0, 0.0], [1.0, 1.0]);
        assert_eq!(b.project(&[-5.0, 5.0]), [0.0, 1.0]);
    }

    #[test]
    #[should_panic(expected = "lo")]
    fn inverted_bounds_panic() {
        let _ = Bounds::new([1.0], [0.0]);
    }
}
