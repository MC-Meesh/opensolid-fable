// Gradient-based dimension optimization over a playground script's declared
// `param()` variables — the JS half of the of-37i.1 differentiable machinery,
// wired so an agent can drive a model onto a mass/volume/centroid target under
// keep-out constraints without re-authoring the geometry by hand.
//
// Why finite differences over the *field*, not the compile-time AD tower:
// `opensolid-frep::diff` gets exact `∂f/∂θ` in one pass, but only for shapes
// written as a `ParamSdf<N>` tower in Rust at compile time (DIFFERENTIABLE.md
// §2, §7). An agent's model is an opaque JS script that builds an `Arc<dyn Sdf>`
// at runtime, so the tower cannot see its parameters. This module bridges the
// gap the way §7 anticipates deferring: it re-evaluates the whole script at
// perturbed parameters (through WASM) and finite-differences the *smooth field*
// objective — `fieldMeasure`/`fieldClearance`, the occupancy integral, not the
// mesh (§5). The field is smooth in the design, so a finite-difference gradient
// of it is well-behaved where a finite difference of the jumpy mesh volume is
// pure noise. The cost is one script re-evaluation per probe rather than one
// per optimization; the payoff is that *every* op works, including `rotate`,
// which the AD tower does not yet cover (§7 / of-37i.2).
//
// The descent itself mirrors `opensolid-frep::diff::optimize::descend` —
// projected gradient with heavy-ball momentum and an Armijo backtracking line
// search — ported to JS and run in a normalized `[0,1]^N` parameter box so a
// single scalar step size is meaningful across parameters of wildly different
// physical scale (a 0.5–8 mm fillet next to a 5–40 mm height).

import { Shape, runScript } from './kernel.js';

/** Hard ceilings on the guardrail knobs — an agent cannot ask for an unbounded run. */
const CAPS = {
  maxIters: 300,
  timeBudgetMs: 120_000,
  resolution: 64,
};

const DEFAULTS = {
  maxIters: 60,
  timeBudgetMs: 30_000,
  resolution: 32,
  // Descent tuning, in normalized parameter space. These track the Rust
  // `DescentOptions` defaults; see optimize.rs for the reasoning behind each.
  initialStep: 0.2,
  momentum: 0.9,
  armijo: 1e-4,
  shrink: 0.5,
  maxBacktracks: 30,
  tol: 1e-4,
  // Finite-difference step, as a fraction of each parameter's own range.
  fdStep: 5e-3,
  // Quadratic-penalty weight for constraint violations, relative to the
  // (normalized, ~O(1)) objective. Big enough that a violated constraint
  // dominates the objective near the boundary, not so big it swamps the
  // gradient into a wall the line search cannot descend (§6). 50 keeps a
  // ~1%-scale objective from buying its way through a constraint.
  penaltyWeight: 50,
};

const OBJECTIVE_TYPES = ['target_mass', 'target_volume', 'centroid_at'];
const CONSTRAINT_TYPES = ['clearance', 'mass', 'volume'];

/**
 * Run the optimizer against one model entry.
 *
 * @param {{script:string, params:Array<{name:string,default:number,value:number,min?:number,max?:number}>, exact:boolean}} entry
 *   the model as registered in the store
 * @param {object} request the `optimize` tool arguments (params/objective/constraints/options)
 * @returns {{overrides:Record<string,number>, shape:object, report:object}}
 *   the winning parameter point, the final shape rebuilt at that point (in the
 *   model's own exact/SDF mode), and the JSON report handed back to the agent
 */
export function optimize(entry, request) {
  const spec = validateRequest(entry, request);
  const { params, objective, constraints, options } = spec;

  // The descent runs on the SDF field regardless of the model's boolean mode;
  // force the fast SDF path during the search and restore the model's own mode
  // for the final exact measurement.
  Shape.setExactBooleans(false);

  const names = params.map((p) => p.name);
  const lo = params.map((p) => p.min);
  const hi = params.map((p) => p.max);
  const start = params.map((p) => p.start);

  // A FIXED integration domain, enclosing every shape the search might visit.
  // The field volume of a solid that pokes outside the domain is silently
  // clipped, so the box must cover the parameter range's extremes, not just the
  // starting shape. We union the tracked box at the start point and at each
  // corner-ish extreme (all-low, all-high), then the WASM side pads it further.
  const domain = fixedDomain(entry.script, names, lo, hi, start, options.resolution);

  const density = objective.density; // may be undefined for non-mass objectives
  const L = domainExtent(domain); // characteristic length for normalizing distances

  // Exact-mesh calibration of the field estimate. The occupancy-integral field
  // volume is biased a few percent high (DIFFERENTIABLE.md §5 "What accuracy to
  // expect" — the smeared band adds a shell), which is fine for *steering* but
  // would land the reported EXACT part off target by that same few percent. So
  // we correct: at each accepted iterate, measure the exact mesh and record the
  // field-minus-exact gap; the loss then targets the *gap-corrected* estimate.
  // As the gap stabilizes, driving the corrected field to the target drives the
  // exact quantity to the target. The gap is held fixed within an iteration so
  // the objective stays smooth for the finite-difference gradient.
  const calib = { volumeGap: 0, centroidGap: [0, 0, 0] };
  const exactSample = (shape) => {
    let m;
    try {
      m = JSON.parse(shape.measure(undefined));
    } catch {
      return null;
    }
    if (m.volume === null || m.volume === undefined) return null;
    return { volume: m.volume, centroid: m.centroid };
  };
  const refreshCalib = (ev) => {
    const ex = exactSample(ev.shape);
    if (!ex) return null; // intermediate shape does not mesh closed — keep the last gap
    calib.volumeGap = ev.measures.volume - ex.volume;
    if (ex.centroid) {
      calib.centroidGap = ev.measures.centroid.map((c, a) => c - ex.centroid[a]);
    }
    return ex; // the exact sample, so the trajectory can log real (not field) numbers
  };

  // Evaluate the smooth-field measures at a raw parameter vector. Returns null
  // on a script/field failure so the line search can reject the point instead
  // of the whole run dying. Loss is computed separately (it depends on the live
  // calibration) via `lossOf(ev.measures, spec, L, calib)`.
  const evalAt = (theta) => {
    let shape;
    try {
      shape = runScript(entry.script, overridesOf(names, theta)).shape;
    } catch {
      return null;
    }
    let fm;
    try {
      fm = JSON.parse(shape.fieldMeasure(options.resolution, undefined, domain));
    } catch {
      return null;
    }
    const volume = fm.volume;
    const centroid = fm.centroid;
    const mass = density !== undefined ? density * volume : undefined;
    const clearances = [];
    for (const c of constraints) {
      if (c.type !== 'clearance') continue;
      let clr;
      try {
        clr = shape.fieldClearance(c.probes, c.softness);
      } catch {
        return null;
      }
      clearances.push(clr);
    }
    return { shape, measures: { volume, centroid, mass, clearances } };
  };

  // ---- Descent in normalized [0,1]^N space -------------------------------
  // u_i = (theta_i - lo_i) / (hi_i - lo_i); a zero-width range pins u_i = 0.
  const span = params.map((p) => p.max - p.min);
  const toTheta = (u) => u.map((ui, i) => (span[i] > 0 ? lo[i] + ui * span[i] : lo[i]));
  const toU = (theta) => theta.map((t, i) => (span[i] > 0 ? (t - lo[i]) / span[i] : 0));

  const trajectory = [];
  // Log the EXACT mesh measures per iterate where available (computed anyway for
  // calibration), not the band-biased field — so the trajectory an agent reads
  // to judge convergence is in the same units as the final `objective.achieved`.
  // Thin-walled parts bias the field volume tens of percent high; the exact log
  // avoids handing the agent a mass that disagrees with the reported result. It
  // falls back to the field only when the intermediate shape did not mesh closed.
  const record = (iter, theta, field, exact, loss) => {
    const volume = exact ? exact.volume : field.volume;
    const centroid = exact && exact.centroid ? exact.centroid : field.centroid;
    trajectory.push({
      iter,
      loss,
      params: overridesOf(names, theta),
      volume,
      ...(density !== undefined ? { mass: density * volume } : {}),
      centroid,
      ...(exact ? {} : { estimated: true }),
      ...(field.clearances.length ? { clearance: field.clearances } : {}),
    });
  };

  // Loss + normalized gradient at u. The gradient is central-differenced in raw
  // parameter space, clamped to the bounds on each side, then scaled into u.
  const N = params.length;
  const lossGrad = (u) => {
    const theta = toTheta(u);
    const base = evalAt(theta);
    if (!base) return null;
    base.loss = lossOf(base.measures, spec, L, calib);
    const grad = new Array(N).fill(0);
    for (let i = 0; i < N; i++) {
      if (span[i] === 0) continue;
      const h = Math.max(1e-9, options.fdStep * span[i]);
      const hp = Math.min(hi[i], theta[i] + h);
      const hm = Math.max(lo[i], theta[i] - h);
      if (hp === hm) continue; // pinned in a zero-width feasible slice
      const tp = theta.slice();
      tp[i] = hp;
      const tm = theta.slice();
      tm[i] = hm;
      const ep = evalAt(tp);
      const em = evalAt(tm);
      if (!ep || !em) continue;
      const lp = lossOf(ep.measures, spec, L, calib);
      const lm = lossOf(em.measures, spec, L, calib);
      // ∂loss/∂theta_i, then × span[i] to get ∂loss/∂u_i.
      grad[i] = ((lp - lm) / (hp - hm)) * span[i];
    }
    return { theta, base, grad };
  };

  const clamp01 = (u) => u.map((ui) => Math.min(1, Math.max(0, ui)));
  const deadline = Date.now() + options.timeBudgetMs;

  let u = clamp01(toU(start));
  const firstEv = evalAt(toTheta(u));
  if (!firstEv) {
    throw new Error(
      'optimize could not evaluate the model at the starting parameters — the ' +
        'script threw or produced an empty field. Check that it returns a Shape ' +
        'for the given parameter values.',
    );
  }
  const firstExact = refreshCalib(firstEv); // seed the field/exact gap at the start point

  const first = lossGrad(u);
  let value = first.base.loss;
  let grad = first.grad;
  // Parameters the objective is flat in: their gradient is ~0 at the start, so
  // the descent cannot move them. This is the honest replacement for a blanket
  // "the tower does not cover this op" error (the field path covers every op) —
  // it names the parameters that genuinely do not influence the objective.
  const gScale = Math.max(1e-12, Math.max(...grad.map(Math.abs)));
  const inertParams = names.filter((_, i) => span[i] > 0 && Math.abs(grad[i]) < 1e-6 * gScale);

  let velocity = new Array(N).fill(0);
  let step = options.initialStep;
  let iters = 0;
  let converged = false;
  let stopReason = 'max_iters';

  record(0, first.theta, firstEv.measures, firstExact, value);

  for (let iter = 0; iter < options.maxIters; iter++) {
    if (Date.now() > deadline) {
      stopReason = 'time_budget';
      break;
    }
    const gnorm2 = grad.reduce((s, g) => s + g * g, 0);
    if (gnorm2 === 0) {
      converged = true;
      stopReason = 'zero_gradient';
      break;
    }

    // Momentum step, falling back to plain descent, then backtracking — the
    // same accept rule as the Rust `descend`: Armijo against the *projected*
    // displacement, which also screens a momentum step that is not a descent
    // direction at all.
    let accepted = null;
    search: for (let bt = 0; bt < options.maxBacktracks; bt++) {
      for (const beta of [options.momentum, 0]) {
        const v = velocity.map((vi, i) => beta * vi - step * grad[i]);
        const trial = clamp01(u.map((ui, i) => ui + v[i]));
        const moved = trial.reduce((s, ti, i) => s + (ti - u[i]) * grad[i], 0);
        if (moved < 0) {
          const lg = lossGrad(trial);
          if (lg && lg.base.loss <= value + options.armijo * moved) {
            accepted = { trial, lg, v };
            break search;
          }
        }
        if (beta === 0) step *= options.shrink;
      }
    }

    if (!accepted) {
      stopReason = 'line_search_stalled';
      break;
    }

    const movedMax = accepted.trial.reduce((m, ti, i) => Math.max(m, Math.abs(ti - u[i])), 0);
    u = accepted.trial;
    velocity = accepted.v;
    iters = iter + 1;

    // The move is committed: recalibrate against the exact mesh at the new
    // point, then recompute the loss AND gradient under the refreshed gap. The
    // gap shift tilts the objective, so a gradient carried over from the old
    // calibration is no longer a descent direction — reusing it stalls the
    // line search after one step. Re-evaluating here keeps value and grad
    // consistent for the next iteration's Armijo test.
    const exSample = refreshCalib(accepted.lg.base);
    const gNext = lossGrad(u);
    value = gNext ? gNext.base.loss : lossOf(accepted.lg.base.measures, spec, L, calib);
    grad = gNext ? gNext.grad : accepted.lg.grad;
    record(iters, accepted.lg.theta, accepted.lg.base.measures, exSample, value);

    if (movedMax < options.tol) {
      converged = true;
      stopReason = 'tol';
      break;
    }
    // Reward a successful step with a slightly bolder next one.
    step /= Math.sqrt(options.shrink);
  }
  if (iters === options.maxIters) stopReason = 'max_iters';

  // A stalled line search at a point where the objective is essentially met and
  // the constraints hold is a local optimum, not a failure: the gradient is ~0,
  // so no step can improve, and there is nothing left to do. Distinguish that
  // from a stall with the objective still unmet — an active constraint or a
  // bound (§6) — which stays converged:false so the agent does not ship it.
  if (!converged && stopReason === 'line_search_stalled' && value < 1e-4) {
    converged = true;
    stopReason = 'stationary';
  }

  // ---- Final report on the winning point ---------------------------------
  const winTheta = toTheta(u);
  const overrides = overridesOf(names, winTheta);

  // Rebuild in the model's own boolean mode and measure the EXACT mesh, per §5:
  // steer with the biased field estimate, report the exact number.
  Shape.setExactBooleans(entry.exact);
  const finalShape = runScript(entry.script, overrides).shape;
  const exact = JSON.parse(finalShape.measure(undefined));

  const pinned = params
    .filter((p) => p.max > p.min)
    .filter((p) => nearBound(overrides[p.name], p.min, p.max))
    .map((p) => ({
      name: p.name,
      value: overrides[p.name],
      at: Math.abs(overrides[p.name] - p.min) <= Math.abs(overrides[p.name] - p.max) ? 'min' : 'max',
    }));

  const constraintsReport = constraintReport(constraints, finalShape, exact);
  // A soft quadratic penalty does not *enforce* a constraint (§6), so the
  // descent can settle at a point that trades a violated constraint for a
  // better objective. Feasibility is therefore a separate, load-bearing fact
  // from mathematical convergence — surface it, and never report a
  // constraint-violating point as converged, so an agent does not ship it.
  const feasible = constraintsReport.every((c) => c.satisfied);
  if (!feasible) converged = false;

  const report = {
    converged,
    feasible,
    stopReason,
    iterations: iters,
    params: overrides,
    objective: objectiveReport(objective, exact, density),
    constraints: constraintsReport,
    pinned,
    field: {
      domain: { min: Array.from(domain.slice(0, 3)), max: Array.from(domain.slice(3, 6)) },
      resolution: options.resolution,
    },
    exactMeasure: { volume: exact.volume, centroid: exact.centroid, exact: exact.exact, massError: exact.massError },
    trajectory,
    warnings: buildWarnings({ converged, stopReason, pinned, inertParams, exact, constraints: constraintsReport }),
  };

  return { overrides, shape: finalShape, report };
}

// --------------------------------------------------------------------------
// Loss
// --------------------------------------------------------------------------

/**
 * Scalar loss = objective term + Σ constraint penalties, all normalized ~O(1).
 * `calib` holds the field-minus-exact gap (see the descent loop): the field
 * quantities are corrected by it so the loss tracks the *exact* quantity the
 * result will report, not the biased field the quadrature sees.
 */
function lossOf(measures, spec, L, calib) {
  const { objective, constraints, options } = spec;
  const estVolume = measures.volume - calib.volumeGap;
  let loss = objectiveTerm(objective, measures, estVolume, L, calib);
  let ci = 0; // index into measures.clearances, which only holds clearance constraints
  for (const c of constraints) {
    if (c.type === 'clearance') {
      // Clearance reads the raw signed-distance field, which is *not* band-
      // biased (fieldClearance samples sdf.eval directly), so it needs no
      // calibration.
      const clr = measures.clearances[ci++];
      const violation = Math.max(0, c.min - clr);
      loss += options.penaltyWeight * (violation / L) ** 2;
    } else if (c.type === 'mass' || c.type === 'volume') {
      // A mass bound carries its own density (it may differ from the
      // objective's), so derive its mass from the calibrated volume.
      const val = c.type === 'mass' ? c.density * estVolume : estVolume;
      const scale = boundScale(c);
      const over = c.max !== undefined ? Math.max(0, val - c.max) : 0;
      const under = c.min !== undefined ? Math.max(0, c.min - val) : 0;
      loss += options.penaltyWeight * ((over + under) / scale) ** 2;
    }
  }
  return loss;
}

function objectiveTerm(objective, measures, estVolume, L, calib) {
  if (objective.type === 'target_volume') {
    return ((estVolume - objective.value) / objective.value) ** 2;
  }
  if (objective.type === 'target_mass') {
    return ((objective.density * estVolume - objective.value) / objective.value) ** 2;
  }
  // centroid_at: sum of squared normalized error over the constrained axes, on
  // the gap-corrected centroid.
  let term = 0;
  for (let a = 0; a < 3; a++) {
    const t = objective.value[a];
    if (t === null || t === undefined) continue;
    const est = measures.centroid[a] - calib.centroidGap[a];
    term += ((est - t) / L) ** 2;
  }
  return term;
}

/** Normalizing scale for a mass/volume bound penalty. */
function boundScale(c) {
  const ref = c.max !== undefined ? c.max : c.min;
  return Math.abs(ref) > 1e-12 ? Math.abs(ref) : 1;
}

// --------------------------------------------------------------------------
// Reporting
// --------------------------------------------------------------------------

function objectiveReport(objective, exact, density) {
  if (objective.type === 'centroid_at') {
    return {
      type: 'centroid_at',
      target: objective.value,
      achieved: exact.centroid,
      // Null centroid means the exact mesh did not close; the field steered on
      // an estimate but the reported truth is unavailable — surface that.
      ...(exact.centroid ? {} : { unavailable: exact.massError || 'exact mesh has no centroid' }),
    };
  }
  const achieved =
    objective.type === 'target_mass'
      ? exact.volume === null
        ? null
        : density * exact.volume
      : exact.volume;
  const report = { type: objective.type, target: objective.value, achieved };
  if (achieved !== null && achieved !== undefined) {
    report.error = achieved - objective.value;
    report.relativeError = objective.value !== 0 ? (achieved - objective.value) / objective.value : null;
  } else {
    report.unavailable = exact.massError || 'exact mesh has no volume';
  }
  if (objective.type === 'target_mass') report.density = density;
  return report;
}

function constraintReport(constraints, finalShape, exact) {
  return constraints.map((c) => {
    if (c.type === 'clearance') {
      // Report the clearance at the sharp, converged shape.
      let value = null;
      try {
        value = finalShape.fieldClearance(c.probes, c.softness);
      } catch {
        value = null;
      }
      return { type: 'clearance', min: c.min, value, satisfied: value !== null && value >= c.min };
    }
    const measured =
      c.type === 'mass' ? (exact.volume === null ? null : c.density * exact.volume) : exact.volume;
    const satisfied =
      measured !== null &&
      (c.max === undefined || measured <= c.max) &&
      (c.min === undefined || measured >= c.min);
    return {
      type: c.type,
      ...(c.min !== undefined ? { min: c.min } : {}),
      ...(c.max !== undefined ? { max: c.max } : {}),
      value: measured,
      satisfied,
    };
  });
}

function buildWarnings({ converged, stopReason, pinned, inertParams, exact, constraints }) {
  const w = [];
  const violated = (constraints || []).filter((c) => !c.satisfied);
  if (violated.length) {
    w.push(
      `Constraint not satisfied: ${violated.map((c) => c.type).join(', ')}. The soft ` +
        'penalty could not hold it against the objective (an active constraint is this ' +
        "optimizer's weak spot — DIFFERENTIABLE.md §6). Options: raise " +
        '`options.penalty_weight`, relax the objective or the bound, or accept the ' +
        'trade-off. The reported point is NOT feasible.',
    );
  }
  if (!converged && !violated.length) {
    w.push(
      `Did not converge (${stopReason}). The reported parameters are the best ` +
        'point reached, not a proven optimum — treat them as a starting point, not a result.',
    );
  }
  if (pinned.length) {
    w.push(
      `Pinned to a bound: ${pinned.map((p) => `${p.name}@${p.at}`).join(', ')}. ` +
        'A parameter resting on its limit usually means the objective wants to go ' +
        'further than you allowed — consider widening that bound.',
    );
  }
  if (inertParams.length) {
    w.push(
      `No effect on the objective: ${inertParams.join(', ')}. The finite-difference ` +
        'gradient is ~0 in these parameters, so the optimizer cannot use them — ' +
        'either they do not touch the measured quantity, or their influence is ' +
        'below the field resolution.',
    );
  }
  if (exact.volume === null) {
    w.push(
      'The converged shape does not mesh to a closed solid, so the exact objective ' +
        `could not be measured (${exact.massError || 'mesh not closed'}). The field ` +
        'estimate steered the search but the final number is unverified.',
    );
  }
  return w;
}

// --------------------------------------------------------------------------
// Fixed integration domain
// --------------------------------------------------------------------------

/**
 * Union of the shape's tracked box (as reported by a low-res `fieldMeasure`)
 * over the start point and the all-low / all-high parameter corners, returned
 * as a flat `[minx,miny,minz, maxx,maxy,maxz]` Float64Array. Fixed for the whole
 * run so the quadrature grid does not drift as parameters move.
 */
function fixedDomain(script, names, lo, hi, start, resolution) {
  const probeRes = Math.min(16, resolution);
  const corners = [start, lo, hi];
  let box = null;
  for (const theta of corners) {
    let dom;
    try {
      const shape = runScript(script, overridesOf(names, theta)).shape;
      dom = JSON.parse(shape.fieldMeasure(probeRes, 0.1, new Float64Array([]))).domain;
    } catch {
      continue;
    }
    box = box ? unionBox(box, dom) : { min: dom.min.slice(), max: dom.max.slice() };
  }
  if (!box) {
    throw new Error('optimize could not build the model at any sampled parameter corner');
  }
  return new Float64Array([...box.min, ...box.max]);
}

function unionBox(a, dom) {
  return {
    min: a.min.map((v, i) => Math.min(v, dom.min[i])),
    max: a.max.map((v, i) => Math.max(v, dom.max[i])),
  };
}

function domainExtent(domain) {
  const dx = domain[3] - domain[0];
  const dy = domain[4] - domain[1];
  const dz = domain[5] - domain[2];
  return Math.max(dx, dy, dz, 1e-9);
}

// --------------------------------------------------------------------------
// Small helpers
// --------------------------------------------------------------------------

function overridesOf(names, theta) {
  const o = {};
  names.forEach((n, i) => {
    o[n] = theta[i];
  });
  return o;
}

function nearBound(v, min, max) {
  const eps = 1e-4 * (max - min);
  return Math.abs(v - min) <= eps || Math.abs(v - max) <= eps;
}

// --------------------------------------------------------------------------
// Request validation
// --------------------------------------------------------------------------

/**
 * Validate the tool arguments against the model's declared parameters, filling
 * defaults, and return a normalized spec. Throws a message-only Error naming the
 * exact problem — an agent driving this tool needs to know precisely what it got
 * wrong, not that "something" was invalid.
 */
function validateRequest(entry, request) {
  const req = request || {};
  const declared = new Map((entry.params || []).map((p) => [p.name, p]));

  if (!Array.isArray(req.params) || req.params.length === 0) {
    throw new Error(
      'optimize needs a non-empty `params` array naming which design variables may ' +
        'move and their bounds, e.g. [{ "name": "thickness", "min": 2, "max": 12 }]. ' +
        available(declared),
    );
  }

  const seen = new Set();
  const params = req.params.map((p) => {
    const name = p && p.name;
    if (typeof name !== 'string' || name === '') {
      throw new Error('each entry in `params` needs a string `name`.');
    }
    if (!declared.has(name)) {
      throw new Error(
        `param '${name}' is not declared by the model's script. A parameter is only ` +
          "optimizable if the script introduces it with param('" +
          name +
          "', default, {min, max}). " +
          available(declared),
      );
    }
    if (seen.has(name)) throw new Error(`param '${name}' is listed more than once.`);
    seen.add(name);
    // Bounds are required (DIFFERENTIABLE.md §7): fall back to the declaration's
    // bounds, but a parameter with neither is an error, not an unbounded search.
    const decl = declared.get(name);
    const min = num(p.min, decl.min);
    const max = num(p.max, decl.max);
    if (!Number.isFinite(min) || !Number.isFinite(max)) {
      throw new Error(
        `param '${name}' needs finite min and max bounds (a manufacturing bound is ` +
          'not optional in CAD — an optimizer with no bound will happily return a ' +
          'negative thickness). Provide them in the request or in the param() declaration.',
      );
    }
    if (min > max) throw new Error(`param '${name}': min ${min} exceeds max ${max}.`);
    // Start from the request, else the param's currently-baked value, clamped in.
    const rawStart = num(p.start, decl.value ?? decl.default);
    const startVal = Math.min(max, Math.max(min, rawStart));
    return { name, min, max, start: startVal };
  });

  const objective = validateObjective(req.objective);
  const constraints = validateConstraints(req.constraints, objective);
  const options = validateOptions(req.options);
  return { params, objective, constraints, options };
}

function validateObjective(o) {
  if (!o || typeof o !== 'object') {
    throw new Error(`optimize needs an \`objective\`. One of: ${OBJECTIVE_TYPES.join(', ')}.`);
  }
  if (!OBJECTIVE_TYPES.includes(o.type)) {
    throw new Error(`unknown objective type '${o.type}'. One of: ${OBJECTIVE_TYPES.join(', ')}.`);
  }
  if (o.type === 'centroid_at') {
    if (!Array.isArray(o.value) || o.value.length !== 3) {
      throw new Error('centroid_at needs `value` as [x, y, z]; use null for an axis you do not constrain.');
    }
    const anyFinite = o.value.some((v) => Number.isFinite(v));
    if (!anyFinite) throw new Error('centroid_at `value` constrains no axis (all null).');
    return { type: 'centroid_at', value: o.value.map((v) => (Number.isFinite(v) ? v : null)) };
  }
  if (!Number.isFinite(o.value) || o.value <= 0) {
    throw new Error(`${o.type} needs a positive \`value\` (the target ${o.type === 'target_mass' ? 'mass' : 'volume'}).`);
  }
  if (o.type === 'target_mass') {
    const density = num(o.density, undefined);
    if (!Number.isFinite(density) || density <= 0) {
      throw new Error(
        'target_mass needs a positive `density` (mass per model unit³ — e.g. aluminium ' +
          '6061 at 0.0027 g/mm³). Mass is density × the measured volume; without a ' +
          'density there is no mass to target.',
      );
    }
    return { type: 'target_mass', value: o.value, density };
  }
  return { type: 'target_volume', value: o.value };
}

function validateConstraints(list, objective) {
  if (list === undefined || list === null) return [];
  if (!Array.isArray(list)) throw new Error('`constraints` must be an array.');
  return list.map((c, i) => {
    if (!c || !CONSTRAINT_TYPES.includes(c.type)) {
      throw new Error(`constraint ${i}: type must be one of ${CONSTRAINT_TYPES.join(', ')}.`);
    }
    if (c.type === 'clearance') {
      const probes = flattenProbes(c.probes, i);
      const min = num(c.min, undefined);
      if (!Number.isFinite(min)) {
        throw new Error(`constraint ${i} (clearance): needs a finite \`min\` clearance in model units.`);
      }
      const softness = num(c.softness, 0.02);
      if (!(softness > 0)) throw new Error(`constraint ${i} (clearance): \`softness\` must be positive.`);
      return { type: 'clearance', probes, min, softness };
    }
    // mass / volume bound
    const min = num(c.min, undefined);
    const max = num(c.max, undefined);
    if (min === undefined && max === undefined) {
      throw new Error(`constraint ${i} (${c.type}): needs at least one of \`min\` or \`max\`.`);
    }
    if (min !== undefined && !Number.isFinite(min)) throw new Error(`constraint ${i}: \`min\` must be finite.`);
    if (max !== undefined && !Number.isFinite(max)) throw new Error(`constraint ${i}: \`max\` must be finite.`);
    if (min !== undefined && max !== undefined && min > max) {
      throw new Error(`constraint ${i} (${c.type}): min ${min} exceeds max ${max}.`);
    }
    const out = { type: c.type };
    if (min !== undefined) out.min = min;
    if (max !== undefined) out.max = max;
    if (c.type === 'mass') {
      // Reuse the objective's density if the constraint omits one.
      const density = num(c.density, objective.type === 'target_mass' ? objective.density : undefined);
      if (!Number.isFinite(density) || density <= 0) {
        throw new Error(
          `constraint ${i} (mass): needs a positive \`density\` (or a target_mass objective ` +
            'to inherit one from). Mass is density × measured volume.',
        );
      }
      out.density = density;
    }
    return out;
  });
}

function validateOptions(o) {
  const opt = o || {};
  const resolution = clampInt(opt.resolution, DEFAULTS.resolution, 8, CAPS.resolution);
  const maxIters = clampInt(opt.max_iters, DEFAULTS.maxIters, 1, CAPS.maxIters);
  const timeBudgetMs = clampInt(opt.time_budget_ms, DEFAULTS.timeBudgetMs, 500, CAPS.timeBudgetMs);
  const penaltyWeight = Number.isFinite(opt.penalty_weight) && opt.penalty_weight > 0 ? opt.penalty_weight : DEFAULTS.penaltyWeight;
  return {
    resolution,
    maxIters,
    timeBudgetMs,
    penaltyWeight,
    initialStep: DEFAULTS.initialStep,
    momentum: DEFAULTS.momentum,
    armijo: DEFAULTS.armijo,
    shrink: DEFAULTS.shrink,
    maxBacktracks: DEFAULTS.maxBacktracks,
    tol: DEFAULTS.tol,
    fdStep: DEFAULTS.fdStep,
  };
}

function flattenProbes(probes, i) {
  if (!Array.isArray(probes) || probes.length === 0) {
    throw new Error(
      `constraint ${i} (clearance): needs \`probes\`, the keep-out points the solid must ` +
        'stay clear of — either [[x,y,z], …] or a flat [x,y,z, …] array. (Point keep-outs ' +
        'are what the differentiable clearance field supports; body/plane clearance is future work.)',
    );
  }
  // Accept [[x,y,z], ...] or a flat [x,y,z, ...] buffer.
  let flat;
  if (Array.isArray(probes[0])) {
    flat = [];
    for (const p of probes) {
      if (!Array.isArray(p) || p.length !== 3 || !p.every(Number.isFinite)) {
        throw new Error(`constraint ${i} (clearance): each probe must be [x, y, z] of finite numbers.`);
      }
      flat.push(p[0], p[1], p[2]);
    }
  } else {
    if (probes.length % 3 !== 0 || !probes.every(Number.isFinite)) {
      throw new Error(`constraint ${i} (clearance): a flat probe buffer must be finite and a multiple of 3.`);
    }
    flat = probes.slice();
  }
  return new Float64Array(flat);
}

function available(declared) {
  if (declared.size === 0) {
    return "This model declares no params — add param('name', default, {min, max}) calls to its script to make it optimizable.";
  }
  return `Declared params: ${[...declared.keys()].join(', ')}.`;
}

function num(v, fallback) {
  return Number.isFinite(v) ? v : fallback;
}

function clampInt(v, dflt, min, max) {
  if (!Number.isFinite(v)) return dflt;
  return Math.min(max, Math.max(min, Math.floor(v)));
}
