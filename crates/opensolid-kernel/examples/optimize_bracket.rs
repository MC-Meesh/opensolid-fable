//! Gradient-based part optimisation: drive a right-angle bracket onto a mass
//! target while keeping a connector envelope clear.
//!
//! Run from the repo root (**release matters** — this is ~1e6 field
//! evaluations per iteration, and debug is ~50× slower):
//!
//! ```sh
//! cargo run --release -p opensolid-kernel --example optimize_bracket
//! ```
//!
//! The part has two design parameters — plate `thickness` and the inner
//! `fillet` radius. Both add mass; both push the filleted corner toward the
//! keep-out envelope. Descent finds the point on the target-mass contour
//! nearest the start, and the run checks that it clears the envelope.
//!
//! Nothing here re-meshes to measure. Every objective is an integral over the
//! field (`diff::objective`), so one forward pass with dual numbers returns
//! the value *and* `∂/∂θ` for both parameters — see
//! `docs/design/DIFFERENTIABLE.md`.
//!
//! The final geometry is meshed once, at the end, and its mass is verified
//! against the exact divergence-theorem `mass_properties`: we **steer with
//! the field quadrature and report with the mesh**.
//!
//! # What this demo does and does not show
//!
//! It shows gradients solving the continuous inner loop: 25-ish iterations
//! onto an exact mass target, with no rebuild per step, from derivatives that
//! cost one forward pass each.
//!
//! It does **not** show a hard constrained optimisation. Here the clearance
//! constraint is satisfied with margin at the optimum, so it is verified
//! rather than active. That is not a demo convenience — it is the honest edge
//! of the method. Tighten `REQUIRED_CLEARANCE` past ~2.44 mm (the most any
//! 110 g version of this bracket can achieve) and the optimum moves onto the
//! constraint boundary, where this quadratic-penalty + first-order method
//! crawls: measured, 20,000 iterations covered a third of the remaining
//! distance, and neither the penalty weight nor momentum moved the needle.
//! Riding an active constraint needs a real constrained method (SLSQP, MMA,
//! augmented Lagrangian) — see `docs/design/DIFFERENTIABLE.md` §6. The
//! gradients are not the limitation; the optimiser is.

use std::fs::File;
use std::io::BufWriter;

use opensolid_kernel::core::types::{BoundingBox3, Point3};
use opensolid_kernel::frep::diff::objective::{Occupancy, clearance, mass};
use opensolid_kernel::frep::diff::optimize::{Bounds, DescentOptions, descend};
use opensolid_kernel::frep::diff::{ParamSdf, Scalar, Vec3, field};
use opensolid_kernel::{MeshOptions, mass_properties, mesh_sdf_indexed, write_stl_binary};

// ---------------------------------------------------------------- the part

/// Arm length (x), arm height (y), and width (z), in mm.
const ARM_X: f64 = 60.0;
const ARM_Y: f64 = 50.0;
const WIDTH: f64 = 40.0;

/// Aluminium 6061: 2.70 g/cm³ = 2.7e-6 kg/mm³.
const DENSITY: f64 = 2.7e-6;

/// A right-angle bracket: two plates meeting at the origin, smooth-unioned
/// so the inner corner carries a fillet.
///
/// `params[0]` = plate thickness, `params[1]` = fillet radius.
struct Bracket;

impl ParamSdf<2> for Bracket {
    fn field<T: Scalar>(&self, p: Vec3<T>, params: &[T; 2]) -> T {
        let (t, fillet) = (params[0], params[1]);
        let half_t = t * T::cst(0.5);
        let half_w = T::cst(WIDTH * 0.5);

        // Horizontal plate: x ∈ [0, ARM_X], y ∈ [0, t].
        let horizontal = field::box3(
            p,
            Vec3::new(T::cst(ARM_X * 0.5), half_t, T::zero()),
            Vec3::new(T::cst(ARM_X * 0.5), half_t, half_w),
        );
        // Vertical plate: x ∈ [0, t], y ∈ [0, ARM_Y].
        let vertical = field::box3(
            p,
            Vec3::new(half_t, T::cst(ARM_Y * 0.5), T::zero()),
            Vec3::new(half_t, T::cst(ARM_Y * 0.5), half_w),
        );
        // The blend radius *is* the fillet — smooth union fills the corner.
        field::smooth_union(horizontal, vertical, fillet)
    }

    fn param_names(&self) -> [&'static str; 2] {
        ["thickness", "fillet"]
    }
}

// ------------------------------------------------------------- the problem

/// Mass the bracket must hit, in kg.
const TARGET_MASS: f64 = 0.110;

/// Centre of the connector envelope that must stay clear, and its radius.
const KEEPOUT_CENTER: [f64; 3] = [16.0, 16.0, 0.0];
const KEEPOUT_RADIUS: f64 = 3.0;

/// Minimum required gap between the bracket and that envelope, in mm.
///
/// Kept below ~2.44 mm — the best any 110 g version of this bracket can do —
/// so the optimum is feasible rather than pinned to the constraint. See the
/// module header for what happens past that.
const REQUIRED_CLEARANCE: f64 = 1.5;

/// Weight on the clearance penalty, relative to the (normalised) mass error.
const CLEARANCE_WEIGHT: f64 = 5.0;

/// Points on the surface of the keep-out sphere, spread by the Fibonacci
/// lattice so the softmin sees the whole envelope rather than a few poles.
fn keepout_probes(n: usize) -> Vec<Point3> {
    let golden = std::f64::consts::PI * (3.0 - 5.0_f64.sqrt());
    (0..n)
        .map(|i| {
            let y = 1.0 - 2.0 * (i as f64 + 0.5) / n as f64;
            let r = (1.0 - y * y).max(0.0).sqrt();
            let theta = golden * i as f64;
            Point3::new(
                KEEPOUT_CENTER[0] + KEEPOUT_RADIUS * r * theta.cos(),
                KEEPOUT_CENTER[1] + KEEPOUT_RADIUS * y,
                KEEPOUT_CENTER[2] + KEEPOUT_RADIUS * r * theta.sin(),
            )
        })
        .collect()
}

fn main() -> std::io::Result<()> {
    // The quadrature domain must contain the whole part with room to spare.
    let domain = BoundingBox3::new(
        Point3::new(-4.0, -4.0, -WIDTH * 0.5 - 4.0),
        Point3::new(ARM_X + 4.0, ARM_Y + 4.0, WIDTH * 0.5 + 4.0),
    );
    // Resolution is set by the thinnest feature: the band must fit inside a
    // plate, or the two faces' ramps overlap and the volume is blurred. At
    // res 96 the band is ~2 mm against a >= 5 mm plate.
    let occ = Occupancy::for_domain(96, &domain);
    let probes = keepout_probes(64);

    // Softmin temperature: well below the clearance we care about, so the
    // estimate is tight, but large enough to blend neighbouring probes.
    let softness = 0.3;

    let report = |label: &str, p: &[f64; 2]| {
        let (m, _) = mass(&Bracket, p, &domain, &occ, DENSITY);
        let (c, _) = clearance(&Bracket, p, &probes, softness);
        println!(
            "{label:8} thickness {:5.2} mm   fillet {:5.2} mm   mass {:6.1} g \
             ({:+5.1}% vs target)   clearance {:5.2} mm{}",
            p[0],
            p[1],
            m * 1000.0,
            (m - TARGET_MASS) / TARGET_MASS * 100.0,
            c,
            if c < REQUIRED_CLEARANCE {
                "  ← VIOLATED"
            } else {
                ""
            }
        );
    };

    // Loss: normalised squared mass error, plus a one-sided penalty for
    // eating into the required clearance. Both terms and both gradients come
    // from single dual-number forward passes.
    let loss = |p: &[f64; 2]| {
        let (m, dm) = mass(&Bracket, p, &domain, &occ, DENSITY);
        let (c, dc) = clearance(&Bracket, p, &probes, softness);

        // Mass term: ((m - target)/target)².
        let e = (m - TARGET_MASS) / TARGET_MASS;
        let mut value = e * e;
        let mut grad = [0.0; 2];
        for i in 0..2 {
            grad[i] += 2.0 * e * dm[i] / TARGET_MASS;
        }

        // Clearance term: w · relu(required - c)². Zero (and zero-gradient)
        // once the constraint is satisfied, so it stops pulling the moment
        // the design is feasible.
        let violation = REQUIRED_CLEARANCE - c;
        if violation > 0.0 {
            value += CLEARANCE_WEIGHT * violation * violation;
            for i in 0..2 {
                // d/dθ of (required - c)² = -2(required - c)·dc/dθ
                grad[i] += CLEARANCE_WEIGHT * -2.0 * violation * dc[i];
            }
        }
        (value, grad)
    };

    println!(
        "Right-angle bracket — target mass {:.0} g, keep-out clearance >= {REQUIRED_CLEARANCE} mm\n",
        TARGET_MASS * 1000.0
    );

    // Start deliberately wrong on both counts: too light, and a fillet that
    // will collide with the envelope once the plates thicken.
    let start = [7.0, 12.0];
    report("start", &start);

    let bounds = Bounds::new([5.0, 1.0], [20.0, 15.0]);
    let opts = DescentOptions {
        max_iters: 300,
        initial_step: 0.5,
        tol: 1e-6,
        ..Default::default()
    };
    let result = descend(loss, start, &bounds, &opts);

    report("optimum", &result.params);
    println!(
        "\n{} after {} iterations (loss {:.3e} → {:.3e})",
        if result.converged {
            "Converged"
        } else {
            "Stopped"
        },
        result.iters,
        result.history.first().unwrap_or(&f64::NAN),
        result.loss,
    );
    for (name, value) in Bracket.param_names().iter().zip(result.params) {
        println!("   {name:10} = {value:.3} mm");
    }

    // --- Verify against the exact, mesh-based mass properties -------------
    //
    // The optimiser steered on a smoothed field integral, which carries a
    // ~1% band bias. The number we *report* comes from the divergence
    // theorem over the actual triangles.
    let frozen = Bracket.freeze(result.params);
    let mesh = mesh_sdf_indexed(
        &frozen,
        &MeshOptions {
            bounds: domain,
            resolution: 192,
        },
    );
    assert!(
        mesh.is_closed_manifold(),
        "optimised bracket must mesh watertight"
    );

    let props = mass_properties(&mesh).expect("closed manifold has mass properties");
    let exact_mass = props.volume * DENSITY;
    println!(
        "\nExact (mesh) mass {:.1} g vs target {:.1} g — {:+.1}%",
        exact_mass * 1000.0,
        TARGET_MASS * 1000.0,
        (exact_mass - TARGET_MASS) / TARGET_MASS * 100.0
    );
    println!(
        "   centroid ({:.1}, {:.1}, {:.1}) mm   |   {} triangles",
        props.centroid.x,
        props.centroid.y,
        props.centroid.z,
        mesh.triangle_count()
    );

    write_stl_binary(
        &mesh,
        &mut BufWriter::new(File::create("bracket_optimized.stl")?),
    )?;
    println!("\nWrote bracket_optimized.stl");
    Ok(())
}
