//! Adversarial stress of the trimmed-NURBS standalone tessellation path
//! (of-xxef, pairing of-znb — worked independently of that bead's author).
//!
//! of-znb gated the three dm1-id-214 solids on measured volume through the
//! CDT routing (of-37i.6 / of-6fcu). This suite attacks the same path from
//! angles nothing in the tree covers:
//!
//! 1. **Randomized B-spline solids with exact volume gates** — parallelepiped
//!    bodies whose six faces are NURBS patches placed by Greville abscissae
//!    (linear precision makes the geometry exact at any degree, knot count,
//!    and product-form rational weights), measured against the closed-form
//!    triple product at **1e-9 relative**. Every mesh vertex the CDT can
//!    produce lies on the flat faces, so any volume gap is a triangulation
//!    bug (wrong region, dropped/overlapping triangles, flipped winding) —
//!    never tessellation chord error.
//! 2. **Randomized interior trims** — the same solids with each patch's knot
//!    domain extended past its face quad by random margins, so every outer
//!    loop runs strictly inside the domain: the standalone trimmed-NURBS
//!    path (`face_cdt`) with genuinely trimmed patches, which `nurbs_lattice`
//!    used to refuse outright.
//! 3. **Seam** — a solid tube whose wall is ONE u-closed periodic-style
//!    rational patch (9-point full circle, first control row == last) with a
//!    seam edge traversed once each way. This is the dm1-id-214 shape, but as
//!    a unit fixture: `closure_u` / `unwrap_across_closure` /
//!    `recenter_on_domain` are otherwise exercised only by that corpus file.
//! 4. **Pole** — cone bodies whose collapsed apex control rows are nudged by
//!    one ULP, aimed at the exact `norm_squared() == 0.0` pole-vertex sharing
//!    in `triangulate_bounded_face` and at `collapsed_v_rows`' structural
//!    detection under float-noise-scale disagreement between patches.
//! 5. **Imported B-spline solids under rigid motion** — every exactly
//!    imported solid of the NURBS-bearing corpus files must stay tessellable
//!    and keep its measured volume after a random rotation + translation
//!    (`transform_body` moves control points exactly; the B-Rep-native
//!    contour integral must survive to 1e-9, the mesh to tessellation
//!    fidelity).
//!
//! Protocol (same as `boolean_stress.rs` / `step_corpus.rs`): a failing case
//! is documented as a `bd` bug bead with the seed and repro command, and the
//! test is `#[ignore]`d referencing the bug ID. FAILURES ARE EXPECTED AND
//! WANTED — tests must not be softened to pass. Reproduce any case with
//! `cargo test --test trimmed_nurbs_adversarial <name>` (plus
//! `OPENSOLID_CAMPAIGN_SEED` if a campaign run found it).
//!
//! Bugs filed from this suite (first run, 2026-08-01):
//! - of-w3hj: `newton_surface`'s seeded closest-point projection stalls at
//!   feature-scale distance (residual 1e-2..2e-1, `converged=false`) whenever
//!   the seed sits more than ~0.2 of the knot domain from the answer — on a
//!   completely FLAT rational bilinear patch, where the projection problem
//!   is benign and the unseeded per-span search converges from anywhere.
//!   The ring-embedding hint chain hands exactly such seeds to consecutive
//!   boundary samples once interior-trim margins shrink the face to a
//!   fraction of the domain, and `nurbs_param` converts the stall into a
//!   hard `Degenerate` error that refuses to tessellate the whole solid
//!   (seed 0xADF2_0001, trial 9, face 0: degree 1×1, weights in [0.6, 1.8]).

use std::f64::consts::{FRAC_1_SQRT_2, PI, TAU};

use opensolid_kernel::brep::{
    Body, BodyType, Curve3, Edge, FaceSense, FinSense, GeometryStore, KnotVector, LoopType,
    NurbsSurface, SYSTEM_RESOLUTION, ShellOrientation, Surface3, TessellationOptions,
    TopologyStore, rotate_body, tessellate_body, translate_body,
};
use opensolid_kernel::core::{EntityId, Point3, Vector3};
use opensolid_kernel::io::step::read::{
    SolidOutcome, StepImport, StepReadOptions, read_step_bytes,
};
use opensolid_kernel::{brep_mass_properties, mass_properties};

// ---------------------------------------------------------------------
// Deterministic RNG (splitmix64), identical to `boolean_stress.rs`.
// ---------------------------------------------------------------------

/// Campaign remix (of-5rim): `OPENSOLID_CAMPAIGN_SEED=<hex>` XORs every suite
/// seed so the same properties walk fresh configurations each run. Unset (CI,
/// plain `cargo test`), the suite is byte-for-byte deterministic.
fn campaign_seed() -> u64 {
    static MIX: std::sync::OnceLock<u64> = std::sync::OnceLock::new();
    *MIX.get_or_init(|| match std::env::var("OPENSOLID_CAMPAIGN_SEED") {
        Ok(raw) => {
            let hex = raw.trim();
            let hex = hex.strip_prefix("0x").unwrap_or(hex);
            u64::from_str_radix(&hex.replace('_', ""), 16)
                .unwrap_or_else(|_| panic!("OPENSOLID_CAMPAIGN_SEED must be hex, got {raw:?}"))
        }
        Err(_) => 0,
    })
}

struct Rng(u64);

impl Rng {
    fn new(seed: u64) -> Self {
        Rng(seed ^ campaign_seed())
    }

    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    fn unit(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.unit()
    }

    fn pick(&mut self, n: usize) -> usize {
        (self.next_u64() % n as u64) as usize
    }
}

// ---------------------------------------------------------------------
// Measurement helpers (as `step_corpus.rs`)
// ---------------------------------------------------------------------

/// Volume via the standalone store tessellator, only when it produces a
/// closed manifold — `None` conflates "did not tessellate" with "did not
/// close", so callers that must distinguish do so explicitly.
fn closed_volume(store: &TopologyStore, geo: &GeometryStore, body: EntityId<Body>) -> Option<f64> {
    let mesh = tessellate_body(store, geo, body, &TessellationOptions::default()).ok()?;
    mass_properties(&mesh).ok().map(|mp| mp.volume)
}

/// Volume the second way: surface integrals over the B-Rep faces, reduced to
/// contour integrals over each face's trim curves.
fn exact_volume(store: &TopologyStore, geo: &GeometryStore, body: EntityId<Body>) -> Option<f64> {
    brep_mass_properties(store, geo, body)
        .ok()
        .map(|mp| mp.volume)
}

fn rel_gap(measured: f64, truth: f64) -> f64 {
    (measured - truth).abs() / truth.abs().max(1e-300)
}

// ---------------------------------------------------------------------
// Randomized parallelepiped B-spline solids
// ---------------------------------------------------------------------

/// Per-face patch shape: knot vectors, product-form rational weights (one
/// factor per control row/column — the product keeps each parameter line a
/// variation-diminishing 1D rational reparameterization, so the patch stays
/// injective however the factors land), and `[u_lo, u_hi, v_lo, v_hi]`
/// domain margins. Margin 0 puts the face loop on the knot-domain border
/// (the shape `nurbs_lattice` used to grid); margin > 0 pushes the patch
/// past the quad so the loop runs strictly interior — a real trim.
struct FacePatch {
    knots_u: KnotVector,
    knots_v: KnotVector,
    weights_u: Vec<f64>,
    weights_v: Vec<f64>,
    margins: [f64; 4],
}

impl FacePatch {
    fn random(rng: &mut Rng, rational: bool, max_margin: f64) -> Self {
        let knots = |rng: &mut Rng| {
            let degree = 1 + rng.pick(4);
            let spans = 1 + rng.pick(3);
            KnotVector::clamped_uniform(degree, degree + spans).expect("degree+spans controls")
        };
        let knots_u = knots(rng);
        let knots_v = knots(rng);
        let factors = |rng: &mut Rng, knots: &KnotVector| {
            let count = knots.knots().len() - knots.degree() - 1;
            (0..count)
                .map(|_| if rational { rng.range(0.6, 1.8) } else { 1.0 })
                .collect::<Vec<f64>>()
        };
        let weights_u = factors(rng, &knots_u);
        let weights_v = factors(rng, &knots_v);
        let margin = |rng: &mut Rng| {
            if max_margin == 0.0 || rng.pick(4) == 0 {
                0.0
            } else {
                rng.range(0.05, max_margin)
            }
        };
        let margins = [margin(rng), margin(rng), margin(rng), margin(rng)];
        FacePatch {
            knots_u,
            knots_v,
            weights_u,
            weights_v,
            margins,
        }
    }

    fn describe(&self) -> String {
        format!(
            "deg ({},{}) ctrl ({},{}) margins {:?}",
            self.knots_u.degree(),
            self.knots_v.degree(),
            self.weights_u.len(),
            self.weights_v.len(),
            self.margins,
        )
    }
}

/// The Greville abscissae of `knots`, rescaled so the patch domain runs
/// `0..1` (as `boolean_stress.rs`).
fn normalized_grevilles(knots: &KnotVector) -> Vec<f64> {
    let p = knots.degree();
    let u = knots.knots();
    let control_count = u.len() - p - 1;
    let (lo, hi) = (u[p], u[u.len() - p - 1]);
    (0..control_count)
        .map(|i| {
            let g = u[i + 1..=i + p].iter().sum::<f64>() / p as f64;
            (g - lo) / (hi - lo)
        })
        .collect()
}

/// Bilinear blend of quadrilateral `c0→c1→c2→c3`. All quads here are
/// parallelograms, so the blend is affine in `(u, v)` and extrapolates
/// exactly for parameters outside `[0, 1]` — which is what lets a margin
/// extend the patch past its face while the Greville placement keeps the
/// spline reproducing the same plane.
fn bilerp(c0: Point3, c1: Point3, c2: Point3, c3: Point3, u: f64, v: f64) -> Point3 {
    let bottom = c0 + (c1 - c0) * u;
    let top = c3 + (c2 - c3) * u;
    bottom + (top - bottom) * v
}

/// Parallelepiped solid `origin + [0,1]³·(a,b,c)` whose six faces are NURBS
/// patches shaped by `faces` (in `primitives::block` face order). Edges are
/// straight lines between the eight corners; each patch reproduces its
/// face's plane exactly (Greville placement of an affine map; rational
/// weights keep control points in-plane, and clamped boundaries trace the
/// edge lines by variation diminishing), so the solid's volume is exactly
/// `det(a, b, c)`, which the caller keeps positive.
fn nurbs_parallelepiped(
    store: &mut TopologyStore,
    geo: &mut GeometryStore,
    origin: Point3,
    a: Vector3,
    b: Vector3,
    c: Vector3,
    faces: &[FacePatch; 6],
) -> EntityId<Body> {
    const EDGE_PAIRS: [(usize, usize); 12] = [
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 0),
        (4, 5),
        (5, 6),
        (6, 7),
        (7, 4),
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7),
    ];
    // Vertex cycles counterclockwise viewed from outside (`primitives::block`).
    let face_cycles: [[usize; 4]; 6] = [
        [0, 3, 2, 1], // bottom (−c)
        [4, 5, 6, 7], // top (+c)
        [0, 1, 5, 4], // front (−b)
        [1, 2, 6, 5], // right (+a)
        [2, 3, 7, 6], // back (+b)
        [3, 0, 4, 7], // left (−a)
    ];
    let corners = [
        origin,
        origin + a,
        origin + a + b,
        origin + b,
        origin + c,
        origin + a + c,
        origin + a + b + c,
        origin + b + c,
    ];

    let body = store.create_body(BodyType::Solid);
    let shell = store.create_shell(body, true, ShellOrientation::Outward);
    let vertices = corners.map(|p| store.create_vertex(p, SYSTEM_RESOLUTION));

    let edges: Vec<_> = EDGE_PAIRS
        .iter()
        .map(|&(lo, hi)| {
            let line = Curve3::line(corners[lo], corners[hi] - corners[lo]).expect("distinct");
            let length = (corners[hi] - corners[lo]).norm();
            let curve = geo.add_curve(line);
            store.create_edge_with_curve(
                vertices[lo],
                vertices[hi],
                SYSTEM_RESOLUTION,
                curve,
                0.0,
                length,
            )
        })
        .collect();
    let directed_edge = |from: usize, to: usize| -> (EntityId<Edge>, FinSense) {
        let (index, &(lo, _)) = EDGE_PAIRS
            .iter()
            .enumerate()
            .find(|&(_, &(lo, hi))| (lo, hi) == (from, to) || (lo, hi) == (to, from))
            .expect("cycles only use listed edges");
        let sense = if lo == from {
            FinSense::Forward
        } else {
            FinSense::Reversed
        };
        (edges[index], sense)
    };

    for (cycle, patch) in face_cycles.into_iter().zip(faces) {
        let [q0, q1, q2, q3] = cycle.map(|i| corners[i]);
        let us = normalized_grevilles(&patch.knots_u);
        let vs = normalized_grevilles(&patch.knots_v);
        let [mu0, mu1, mv0, mv1] = patch.margins;
        let grid: Vec<Vec<Point3>> = us
            .iter()
            .map(|&gu| {
                let s = -mu0 + gu * (1.0 + mu0 + mu1);
                vs.iter()
                    .map(|&gv| {
                        let t = -mv0 + gv * (1.0 + mv0 + mv1);
                        bilerp(q0, q1, q2, q3, s, t)
                    })
                    .collect()
            })
            .collect();
        let weights: Vec<Vec<f64>> = patch
            .weights_u
            .iter()
            .map(|wu| patch.weights_v.iter().map(|wv| wu * wv).collect())
            .collect();
        let surface =
            NurbsSurface::new(grid, weights, patch.knots_u.clone(), patch.knots_v.clone())
                .expect("rectangular in-plane grid with positive weights");
        let face = store.create_face(shell, FaceSense::Positive);
        store.faces.get_mut(face).expect("just created").surface =
            Some(geo.add_surface(Surface3::nurbs(surface)));
        let loop_edges: Vec<_> = (0..4)
            .map(|i| directed_edge(cycle[i], cycle[(i + 1) % 4]))
            .collect();
        store.create_loop(face, LoopType::Outer, &loop_edges);
    }
    body
}

/// One randomized trial: a parallelepiped with |det| ≥ 1.5 (resampled until
/// so, det flipped positive so the face cycles wind outward), face patches
/// drawn per face with margins up to `max_margin`, gated on the closed-form
/// volume at 1e-9 relative.
fn parallelepiped_trial(rng: &mut Rng, max_margin: f64, seed_label: &str, trial: usize) {
    let (origin, va, vb, vc) = loop {
        let comp = |rng: &mut Rng| rng.range(-2.5, 2.5);
        let origin = Point3::new(
            rng.range(-5.0, 5.0),
            rng.range(-5.0, 5.0),
            rng.range(-5.0, 5.0),
        );
        let va = Vector3::new(comp(rng), comp(rng), comp(rng));
        let vb = Vector3::new(comp(rng), comp(rng), comp(rng));
        let mut vc = Vector3::new(comp(rng), comp(rng), comp(rng));
        let det = va.cross(&vb).dot(&vc);
        if det.abs() < 1.5 {
            continue;
        }
        if det < 0.0 {
            vc = -vc;
        }
        break (origin, va, vb, vc);
    };
    let truth = va.cross(&vb).dot(&vc);
    let rational = rng.pick(2) == 0;
    let faces: [FacePatch; 6] =
        std::array::from_fn(|_| FacePatch::random(rng, rational, max_margin));
    let context = format!(
        "[{seed_label}, trial {trial}] parallelepiped o={origin:?} a={va:?} b={vb:?} c={vc:?} \
         rational={rational} faces: {}",
        faces
            .iter()
            .map(FacePatch::describe)
            .collect::<Vec<_>>()
            .join(" | "),
    );

    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let body = nurbs_parallelepiped(&mut store, &mut geo, origin, va, vb, vc, &faces);

    let mesh = tessellate_body(&store, &geo, body, &TessellationOptions::default())
        .unwrap_or_else(|e| panic!("{context}\nmust tessellate: {e}"));
    let props = mass_properties(&mesh)
        .unwrap_or_else(|e| panic!("{context}\nmust mesh closed and measurable: {e:?}"));
    let gap = rel_gap(props.volume, truth);
    assert!(
        gap <= 1e-9,
        "{context}\nmeshed volume {} vs exact {truth} — relative gap {gap:e} on a body whose \
         every mesh vertex lies on its flat faces; this is a triangulation defect, not chord \
         error",
        props.volume,
    );
}

/// (1) Full-domain patches: every face loop runs along the knot-domain
/// border — the standalone path's boundary-hugging shape, randomized over
/// degree, span count, and rational weights.
#[test]
fn random_bspline_parallelepipeds_measure_exactly() {
    let mut rng = Rng::new(0xADF1_0001);
    for trial in 0..10 {
        parallelepiped_trial(&mut rng, 0.0, "seed 0xADF1_0001", trial);
    }
}

/// (2) Interior trims: patches extended past their face quads by random
/// margins, so the loops run strictly inside the knot domain — the genuinely
/// trimmed standalone path (`face_cdt`), which `nurbs_lattice` refused
/// before of-37i.6.
#[test]
#[ignore = "of-w3hj: seeded NURBS projection stalls at feature scale on flat rational \
            patches (trial 9); un-ignore when the projection fallback lands"]
fn random_interior_trimmed_bspline_parallelepipeds_measure_exactly() {
    let mut rng = Rng::new(0xADF2_0001);
    for trial in 0..12 {
        parallelepiped_trial(&mut rng, 1.2, "seed 0xADF2_0001", trial);
    }
}

// ---------------------------------------------------------------------
// Seam: u-closed periodic-style patch with a both-ways seam edge
// ---------------------------------------------------------------------

/// Solid tube `z ∈ [z0, z0+h]` about `(cx, cy)` whose wall is ONE u-closed
/// rational quadratic patch: the standard 9-control-point full circle
/// (corner weights 1, mid weights 1/√2, double interior knots at each
/// quarter) swept linearly in `v`. First and last control rows coincide, so
/// `closure_u` reports the full period — the dm1-id-214 shape as a unit
/// fixture. The wall loop traverses its one seam edge once each way, and the
/// caps close over full-circle self-loop edges.
fn periodic_nurbs_tube(
    store: &mut TopologyStore,
    geo: &mut GeometryStore,
    cx: f64,
    cy: f64,
    r: f64,
    z0: f64,
    h: f64,
) -> EntityId<Body> {
    let axis = Vector3::z();
    let bottom_center = Point3::new(cx, cy, z0);
    let top_center = Point3::new(cx, cy, z0 + h);
    let seam_foot = bottom_center + Vector3::x() * r;

    let body = store.create_body(BodyType::Solid);
    let shell = store.create_shell(body, true, ShellOrientation::Outward);
    let v_bot = store.create_vertex(seam_foot, SYSTEM_RESOLUTION);
    let v_top = store.create_vertex(seam_foot + axis * h, SYSTEM_RESOLUTION);

    let full_circle =
        |store: &mut TopologyStore, geo: &mut GeometryStore, center: Point3, vertex| {
            let circle = Curve3::circle(center, axis, r).expect("valid circle");
            let curve = geo.add_curve(circle);
            store.create_edge_with_curve(vertex, vertex, SYSTEM_RESOLUTION, curve, 0.0, TAU)
        };
    let e_bot = full_circle(store, geo, bottom_center, v_bot);
    let e_top = full_circle(store, geo, top_center, v_top);
    let seam = {
        let line = Curve3::line(seam_foot, axis).expect("valid seam");
        let curve = geo.add_curve(line);
        store.create_edge_with_curve(v_bot, v_top, SYSTEM_RESOLUTION, curve, 0.0, h)
    };

    // 9-point rational circle in the wall's u direction, swept in v.
    let quarter_dirs = [Vector3::x(), Vector3::y(), -Vector3::x(), -Vector3::y()];
    let mut ring: Vec<(Vector3, f64)> = Vec::with_capacity(9);
    for q in 0..4 {
        let d0 = quarter_dirs[q];
        let d1 = quarter_dirs[(q + 1) % 4];
        ring.push((d0, 1.0));
        ring.push((d0 + d1, FRAC_1_SQRT_2));
    }
    ring.push((Vector3::x(), 1.0));
    let control_points: Vec<Vec<Point3>> = ring
        .iter()
        .map(|(d, _)| vec![bottom_center + d * r, top_center + d * r])
        .collect();
    let weights: Vec<Vec<f64>> = ring.iter().map(|&(_, w)| vec![w, w]).collect();
    let knots_u = KnotVector::new(
        2,
        vec![
            0.0, 0.0, 0.0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1.0, 1.0, 1.0,
        ],
    )
    .expect("full-circle knots");
    let knots_v = KnotVector::clamped_uniform(1, 2).expect("linear sweep");
    let patch = NurbsSurface::new(control_points, weights, knots_u, knots_v)
        .expect("valid periodic-style wall patch");
    assert!(
        patch.closure_u().is_some(),
        "fixture must be u-closed — first and last control rows coincide"
    );

    let wall = store.create_face(shell, FaceSense::Positive);
    store.faces.get_mut(wall).expect("just created").surface =
        Some(geo.add_surface(Surface3::nurbs(patch)));
    store.create_loop(
        wall,
        LoopType::Outer,
        &[
            (e_bot, FinSense::Forward),
            (seam, FinSense::Forward),
            (e_top, FinSense::Reversed),
            (seam, FinSense::Reversed),
        ],
    );

    let f_bottom = store.create_face(shell, FaceSense::Positive);
    store.faces.get_mut(f_bottom).expect("just created").surface =
        Some(geo.add_surface(Surface3::plane(bottom_center, -axis).expect("valid plane")));
    store.create_loop(f_bottom, LoopType::Outer, &[(e_bot, FinSense::Reversed)]);

    let f_top = store.create_face(shell, FaceSense::Positive);
    store.faces.get_mut(f_top).expect("just created").surface =
        Some(geo.add_surface(Surface3::plane(top_center, axis).expect("valid plane")));
    store.create_loop(f_top, LoopType::Outer, &[(e_top, FinSense::Forward)]);
    body
}

/// (3) The u-closed seam fixture must tessellate closed and measure its
/// cylinder volume to tessellation fidelity (the same 3% calibration as
/// of-znb's dm1 gate); where the B-Rep-native integral also measures, the
/// two must agree.
#[test]
fn periodic_seam_nurbs_tube_measures() {
    let (r, h) = (1.3, 2.1);
    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let body = periodic_nurbs_tube(&mut store, &mut geo, 0.4, -0.7, r, 0.2, h);
    let truth = PI * r * r * h;

    let meshed = closed_volume(&store, &geo, body).unwrap_or_else(|| {
        panic!(
            "u-closed periodic wall patch with both-ways seam edge must tessellate closed \
             and measure (unit fixture of the dm1-id-214 shape, of-xxef)"
        )
    });
    let gap = rel_gap(meshed, truth);
    assert!(
        gap <= 3e-2,
        "periodic tube meshed volume {meshed} vs π·r²·h = {truth}: relative gap {gap:e} \
         is past tessellation fidelity"
    );
    if let Some(exact) = exact_volume(&store, &geo, body) {
        let gap = rel_gap(meshed, exact);
        assert!(
            gap <= 3e-2,
            "periodic tube meshed volume {meshed} and B-Rep-native volume {exact} \
             disagree by {gap:e}"
        );
    }
}

// ---------------------------------------------------------------------
// Pole: one-ULP apex disagreements between collapsed-row patches
// ---------------------------------------------------------------------

/// Solid cone about `+Z` (as `boolean_stress.rs`'s `nurbs_cone`: four exact
/// rational quarter patches with their `v = 1` control columns collapsed on
/// the apex), except that `apex_of(patch, row)` decides where each patch
/// places each apex-column control point — the hook the ULP attacks use.
fn nurbs_cone_with_apex(
    store: &mut TopologyStore,
    geo: &mut GeometryStore,
    base_center: Point3,
    r: f64,
    h: f64,
    apex_of: impl Fn(usize, usize) -> Point3,
) -> EntityId<Body> {
    let axis = Vector3::z();
    let dirs = [Vector3::x(), Vector3::y(), -Vector3::x(), -Vector3::y()];
    let apex = base_center + axis * h;

    let body = store.create_body(BodyType::Solid);
    let shell = store.create_shell(body, true, ShellOrientation::Outward);
    let v_base: Vec<_> = dirs
        .iter()
        .map(|d| store.create_vertex(base_center + d * r, SYSTEM_RESOLUTION))
        .collect();
    let v_apex = store.create_vertex(apex, SYSTEM_RESOLUTION);

    let e_base: Vec<_> = (0..4)
        .map(|k| {
            let circle = Curve3::circle(base_center, axis, r).expect("valid circle");
            let curve = geo.add_curve(circle);
            store.create_edge_with_curve(
                v_base[k],
                v_base[(k + 1) % 4],
                SYSTEM_RESOLUTION,
                curve,
                k as f64 * (TAU / 4.0),
                (k + 1) as f64 * (TAU / 4.0),
            )
        })
        .collect();
    let e_seam: Vec<_> = (0..4)
        .map(|k| {
            let from = base_center + dirs[k] * r;
            let slant = apex - from;
            let length = slant.norm();
            let line = Curve3::line(from, slant / length).expect("valid slant");
            let curve = geo.add_curve(line);
            store.create_edge_with_curve(v_base[k], v_apex, SYSTEM_RESOLUTION, curve, 0.0, length)
        })
        .collect();

    let f_base = store.create_face(shell, FaceSense::Positive);
    store.faces.get_mut(f_base).expect("just created").surface =
        Some(geo.add_surface(Surface3::plane(base_center, -axis).expect("valid plane")));
    store.create_loop(
        f_base,
        LoopType::Outer,
        &[
            (e_base[3], FinSense::Reversed),
            (e_base[2], FinSense::Reversed),
            (e_base[1], FinSense::Reversed),
            (e_base[0], FinSense::Reversed),
        ],
    );

    let knots_u = KnotVector::clamped_uniform(2, 3).expect("degree-2 knots");
    let knots_v = KnotVector::clamped_uniform(1, 2).expect("degree-1 knots");
    for k in 0..4 {
        let d0 = dirs[k];
        let d1 = dirs[(k + 1) % 4];
        let ring = [d0, d0 + d1, d1];
        let control_points: Vec<Vec<Point3>> = ring
            .iter()
            .enumerate()
            .map(|(row, d)| vec![base_center + d * r, apex_of(k, row)])
            .collect();
        let weights: Vec<Vec<f64>> = [1.0, FRAC_1_SQRT_2, 1.0]
            .iter()
            .map(|&w| vec![w, w])
            .collect();
        let patch = NurbsSurface::new(control_points, weights, knots_u.clone(), knots_v.clone())
            .expect("valid rational quarter-cone patch");
        let face = store.create_face(shell, FaceSense::Positive);
        store.faces.get_mut(face).expect("just created").surface =
            Some(geo.add_surface(Surface3::nurbs(patch)));
        store.create_loop(
            face,
            LoopType::Outer,
            &[
                (e_base[k], FinSense::Forward),
                (e_seam[(k + 1) % 4], FinSense::Forward),
                (e_seam[k], FinSense::Reversed),
            ],
        );
    }
    body
}

/// (4) One-ULP apex disagreements. Variant A nudges one whole patch's apex
/// column up a ULP (its pole stays exactly collapsed, but disagrees with its
/// neighbours' and with the shared apex vertex); variant B nudges a single
/// control point (the pole row is no longer bitwise a point and the
/// evaluated pole moves off every neighbour's). Both aim at the exact
/// `norm_squared() == 0.0` pole-vertex sharing in `triangulate_bounded_face`;
/// the body must still mesh closed and measure `π r² h / 3` to tessellation
/// fidelity.
#[test]
fn nurbs_cone_pole_survives_one_ulp_apex_perturbations() {
    let mut rng = Rng::new(0xADF4_0001);
    for trial in 0..6 {
        let r = rng.range(0.4, 3.0);
        let h = rng.range(0.5, 4.0);
        let (cx, cy, z0) = (
            rng.range(-4.0, 4.0),
            rng.range(-4.0, 4.0),
            rng.range(-2.0, 2.0),
        );
        let apex = Point3::new(cx, cy, z0 + h);
        let nudged = Point3::new(apex.x, apex.y, apex.z.next_up());
        let victim = rng.pick(4);
        let whole_column = trial % 2 == 0;
        let context = format!(
            "[seed 0xADF4_0001, trial {trial}] cone r={r} h={h} center=({cx},{cy}) z0={z0} \
             victim patch {victim} {}",
            if whole_column {
                "whole apex column +1 ULP"
            } else {
                "single apex control point +1 ULP"
            },
        );

        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let base_center = Point3::new(cx, cy, z0);
        let body = nurbs_cone_with_apex(&mut store, &mut geo, base_center, r, h, |patch, row| {
            if patch == victim && (whole_column || row == 1) {
                nudged
            } else {
                apex
            }
        });

        let truth = PI * r * r * h / 3.0;
        let meshed = closed_volume(&store, &geo, body)
            .unwrap_or_else(|| panic!("{context}\ncone must still tessellate closed and measure"));
        let gap = rel_gap(meshed, truth);
        assert!(
            gap <= 3e-2,
            "{context}\nmeshed volume {meshed} vs π·r²·h/3 = {truth}: relative gap {gap:e}"
        );
    }
}

// ---------------------------------------------------------------------
// Imported B-spline solids under rigid motion
// ---------------------------------------------------------------------

fn load(name: &str) -> Vec<u8> {
    let path = format!("{}/tests/data/step/{name}", env!("CARGO_MANIFEST_DIR"));
    std::fs::read(&path).unwrap_or_else(|e| panic!("cannot read {path}: {e}"))
}

fn import_bytes(bytes: &[u8]) -> (TopologyStore, GeometryStore, StepImport) {
    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let report = read_step_bytes(bytes, &mut store, &mut geo, &StepReadOptions::default())
        .expect("vendored file must parse");
    (store, geo, report)
}

/// (5) Every exactly imported solid of the NURBS-bearing corpus files must
/// keep measuring after a random rigid motion. `transform_body` moves NURBS
/// control points exactly and leaves knots, weights, and stored pcurves
/// untouched, so the B-Rep-native contour integral must reproduce its volume
/// to float noise (1e-9), and the standalone tessellator must keep closing
/// the mesh with the volume held to tessellation fidelity (3% — threshold
/// flips in sampling density can move a chord-error-sized amount, no more).
/// Measurability itself must be motion-invariant in both directions.
#[test]
fn imported_bspline_solids_measure_under_rigid_motion() {
    let mut rng = Rng::new(0xADF5_0001);
    let files = [
        "dm1-id-214.stp",
        "occ/nurbs/lofted_vase.stp",
        "occ/nurbs/bspline_patch_prism.stp",
    ];
    let mut measured_any = false;
    for file in files {
        let (mut store, mut geo, report) = import_bytes(&load(file));
        let bodies: Vec<EntityId<Body>> = report
            .solids
            .iter()
            .filter_map(|s| match &s.outcome {
                SolidOutcome::BRep(body) => Some(*body),
                _ => None,
            })
            .collect();
        assert!(
            !bodies.is_empty(),
            "{file}: expected at least one exact B-Rep solid"
        );
        for (index, &body) in bodies.iter().enumerate() {
            let meshed_before = closed_volume(&store, &geo, body);
            let exact_before = exact_volume(&store, &geo, body);

            let axis = Vector3::new(
                rng.range(-1.0, 1.0),
                rng.range(-1.0, 1.0),
                rng.range(-1.0, 1.0),
            );
            let axis = if axis.norm() < 1e-3 {
                Vector3::z()
            } else {
                axis
            };
            let angle = rng.range(0.3, 5.9);
            let pivot = Point3::new(
                rng.range(-20.0, 20.0),
                rng.range(-20.0, 20.0),
                rng.range(-20.0, 20.0),
            );
            let offset = Vector3::new(
                rng.range(-40.0, 40.0),
                rng.range(-40.0, 40.0),
                rng.range(-40.0, 40.0),
            );
            let context = format!(
                "[seed 0xADF5_0001] {file} solid {index}: rotate {angle} rad about \
                 {axis:?} through {pivot:?}, then translate {offset:?}",
            );
            rotate_body(&mut store, &mut geo, body, pivot, axis, angle)
                .unwrap_or_else(|e| panic!("{context}\nrotate_body: {e}"));
            translate_body(&mut store, &mut geo, body, offset)
                .unwrap_or_else(|e| panic!("{context}\ntranslate_body: {e}"));

            let meshed_after = closed_volume(&store, &geo, body);
            let exact_after = exact_volume(&store, &geo, body);
            assert_eq!(
                meshed_before.is_some(),
                meshed_after.is_some(),
                "{context}\ntessellability must survive rigid motion \
                 (before {meshed_before:?}, after {meshed_after:?})"
            );
            assert_eq!(
                exact_before.is_some(),
                exact_after.is_some(),
                "{context}\nB-Rep-native measurability must survive rigid motion \
                 (before {exact_before:?}, after {exact_after:?})"
            );
            if let (Some(v0), Some(v1)) = (meshed_before, meshed_after) {
                let gap = rel_gap(v1, v0);
                assert!(
                    gap <= 3e-2,
                    "{context}\nmeshed volume moved {gap:e} under rigid motion \
                     ({v0} → {v1})"
                );
                measured_any = true;
            }
            if let (Some(v0), Some(v1)) = (exact_before, exact_after) {
                let gap = rel_gap(v1, v0);
                assert!(
                    gap <= 1e-9,
                    "{context}\nB-Rep-native volume moved {gap:e} under rigid motion \
                     ({v0} → {v1}) — the contour integral sees exactly moved geometry"
                );
            }
        }
    }
    assert!(
        measured_any,
        "no imported B-spline solid measured through the tessellator at all — \
         the standalone path regressed past even dm1-id-214 (of-znb's gate)"
    );
}
