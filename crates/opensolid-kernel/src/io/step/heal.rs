//! Geometry healing for imported B-Rep bodies (`spec/06-step-io.md` §6).
//!
//! Real-world STEP is messy. A file from CATIA, SolidWorks or Creo routinely
//! arrives with each face carrying its *own* `VERTEX_POINT`s and
//! `EDGE_CURVE`s for boundaries it shares with its neighbours (an "unsewn"
//! shell), with those duplicated points disagreeing in the last few decimals
//! (a "gap"), or with a handful of faces whose authored orientation runs
//! against the rest of the shell. None of that is a parsing problem — the
//! [reader](super::read) maps every entity happily — but the resulting body
//! fails [`TopologyStore::check`] and the import degrades to the mesh
//! fallback, losing exactness for a defect measured in nanometres.
//!
//! The healer repairs those bodies in place. It runs *after* the exact
//! mapping pass and *before* the mesh fallback: a body that heals clean is
//! promoted back to [`SolidOutcome::BRep`](super::read::SolidOutcome::BRep),
//! and one that does not still falls back exactly as before. Import heals; it
//! never rejects.
//!
//! # Phase 1 passes
//!
//! - **Gap closure** ([`GeometryHealer::fix_gaps`]) — cluster vertices that
//!   lie within `max_gap` of each other into one, then weld the duplicate
//!   edges that clustering exposes (edges that now span the same vertex pair
//!   *and* whose curves sample to the same points). Welding re-points the
//!   orphaned fins onto the surviving edge, so a shell whose faces never
//!   shared a boundary becomes watertight. The distance closed is absorbed
//!   into the surviving entity's tolerance — this is tolerant modelling
//!   (`spec/08-tolerances.md`), not a silent snap.
//! - **Orientation repair** ([`GeometryHealer::fix_orientation`]) — two-colour
//!   the face adjacency graph so every pair of mated fins traverses its edge
//!   in opposite directions, flipping the minority side of each connected
//!   component. A shell that comes out consistently oriented but *inside out*
//!   (its enclosed signed volume has the wrong sign for its
//!   [`ShellOrientation`]) is then reversed wholesale.
//!
//! Pcurve recompute is not a repair here: the reader derives fin trim
//! geometry for every exactly mapped face as it builds it
//! ([`StepReadOptions::pcurves`](super::read::StepReadOptions::pcurves)), so
//! there is no absent-pcurve state left for the healer to find. The
//! remaining spec §6 operations — edge/surface consistency, edge-curve
//! recomputation from face-face intersection — are phase 2 (`of-3qy.14`).
//!
//! A repair that rewires fins (orientation flips, sewing) leaves their
//! pcurves as mapped: a fin's pcurve depends on its edge's curve and its
//! face's surface, neither of which those repairs touch.
//!
//! # Where healing does *not* reach
//!
//! Healing operates on a *built* body, so it only helps solids the reader
//! mapped successfully and the checker then rejected. A solid that lands in
//! [`SolidOutcome::Failed`](super::read::SolidOutcome::Failed) because the
//! mapping itself failed — a dangling `#id`, an attribute of the wrong type,
//! a `CLOSED_SHELL` with no faces — never produces a body to repair, and no
//! amount of tolerance would make one. Those stay the reader's problem.
//!
//! # What healing will not do
//!
//! Healing never widens a tolerance past
//! [`MAX_ALLOWED_TOLERANCE`], never merges two vertices already joined by an
//! edge (that would collapse the edge and change the Euler counts), and never
//! merges across two shells of one body (that would make the shells
//! non-manifold against each other). Each refusal is recorded in
//! [`HealResult::notes`] rather than applied optimistically.
//!
//! # Example
//!
//! ```
//! use opensolid_brep::{GeometryStore, TopologyStore};
//! use opensolid_kernel::io::step::heal::{GeometryHealer, HealOptions};
//! # use opensolid_brep::{BodyType, FaceSense, FinSense, LoopType, ShellOrientation,
//! #     SYSTEM_RESOLUTION};
//! # use opensolid_core::Point3;
//! let mut store = TopologyStore::new();
//! let geo = GeometryStore::new();
//! # let body = store.create_body(BodyType::Solid);
//! # let shell = store.create_shell(body, true, ShellOrientation::Outward);
//! # let face = store.create_face(shell, FaceSense::Positive);
//! # let a = store.create_vertex(Point3::new(0.0, 0.0, 0.0), SYSTEM_RESOLUTION);
//! # let b = store.create_vertex(Point3::new(1.0, 0.0, 0.0), SYSTEM_RESOLUTION);
//! # let c = store.create_vertex(Point3::new(0.0, 1.0, 0.0), SYSTEM_RESOLUTION);
//! # let e = [
//! #     store.create_edge(a, b, SYSTEM_RESOLUTION),
//! #     store.create_edge(b, c, SYSTEM_RESOLUTION),
//! #     store.create_edge(c, a, SYSTEM_RESOLUTION),
//! # ];
//! # store.create_loop(face, LoopType::Outer, &e.map(|id| (id, FinSense::Forward)));
//! // `body` is some imported solid that failed `store.check(body)`.
//! let result = GeometryHealer::heal(body, &mut store, &geo, &HealOptions::default());
//! for op in &result.operations {
//!     println!("healed: {op}");
//! }
//! ```

use std::collections::HashMap;
use std::fmt;

use opensolid_brep::{
    Body, CheckFailure, CurveEval, Edge, EntityRef, Face, FaceSense, Fin, FinSense, GeometryStore,
    MAX_ALLOWED_TOLERANCE, SYSTEM_RESOLUTION, Shell, ShellOrientation, TessellationOptions,
    TopologyStore, Vertex, tessellate_face,
};
use opensolid_core::{EntityId, Point3};

/// Default vertex-merge tolerance as a fraction of the body's bounding-box
/// diagonal, used when [`HealOptions::max_gap`] is `None`.
///
/// STEP writes coordinates as finite decimals, so a shared corner written
/// twice disagrees at roughly the field's last digit — relative, not
/// absolute. `1e-5` of the diagonal is a micron on a 100 mm part: far below
/// any real feature, far above export round-off. (Compare the reader's
/// `TRIM_TOL_REL`, which is the same idea one decade tighter for a check
/// that must not merge anything.)
pub const HEAL_GAP_REL: f64 = 1e-5;

/// Parameter fractions at which two edges' curves are compared for welding.
/// Interior only: the endpoints are already known to agree (that is what put
/// the edges in the same candidate group).
const SAMPLE_FRACTIONS: [f64; 3] = [0.25, 0.5, 0.75];

/// How aggressively [`read_step`](super::read::read_step) repairs the bodies
/// it maps (`spec/06-step-io.md` §4).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum HealStrategy {
    /// Every phase-1 pass: gap closure then orientation repair. The default —
    /// import always heals, the question is only how far.
    #[default]
    Auto,
    /// Gap closure only. Leaves authored face orientation untouched, for
    /// files whose sense flags are trusted.
    Minimal,
    /// Plan every pass and report what it *would* do, without touching the
    /// body. The import still degrades to the mesh fallback; the reported
    /// operations say what an `Auto` run would have fixed.
    ReportOnly,
    /// Heal nothing. Not part of the spec's strategy set — it exists so a
    /// caller (or a regression test) can see the unhealed outcome.
    Off,
}

impl HealStrategy {
    /// Whether this strategy mutates the body at all.
    fn applies(self) -> bool {
        matches!(self, HealStrategy::Auto | HealStrategy::Minimal)
    }

    /// Whether this strategy includes the orientation-repair pass.
    fn orients(self) -> bool {
        matches!(self, HealStrategy::Auto | HealStrategy::ReportOnly)
    }

    /// Whether this strategy includes the gap-closure pass.
    fn closes_gaps(self) -> bool {
        !matches!(self, HealStrategy::Off)
    }
}

/// Tuning for [`GeometryHealer::heal`].
#[derive(Debug, Clone, PartialEq, Default)]
pub struct HealOptions {
    pub strategy: HealStrategy,
    /// Largest vertex-to-vertex distance that may be closed by merging, in
    /// millimetres (imported geometry is always millimetres — see
    /// [`StepImport::length_scale`](super::read::StepImport::length_scale)).
    ///
    /// `None` derives it per body as [`HEAL_GAP_REL`] × the bounding-box
    /// diagonal, which is what you want unless you know the producer's
    /// tolerance. Values are clamped to [`MAX_ALLOWED_TOLERANCE`]: healing may
    /// not create a body the kernel would reject for tolerance alone.
    pub max_gap: Option<f64>,
}

/// One repair the healer applied (or, under
/// [`HealStrategy::ReportOnly`], would have applied).
#[derive(Debug, Clone, PartialEq)]
pub enum HealOperation {
    /// Coincident vertices collapsed into one at their centroid.
    VerticesMerged {
        kept: EntityId<Vertex>,
        merged: usize,
        /// Largest distance any merged point moved.
        gap: f64,
    },
    /// Duplicate edges over one vertex pair collapsed into one, re-pointing
    /// the orphaned fins — the step that makes an unsewn shell watertight.
    EdgesWelded {
        kept: EntityId<Edge>,
        merged: usize,
        /// Largest distance between the welded curves' samples.
        gap: f64,
    },
    /// An entity's tolerance was raised to cover the distance a repair closed.
    ToleranceElevated {
        entity: EntityRef,
        new_tolerance: f64,
    },
    /// A face was reversed (surface sense and every loop's traversal) to agree
    /// with its neighbours.
    FaceReoriented { face: EntityId<Face> },
    /// A whole shell was reversed: consistently oriented, but enclosing a
    /// signed volume of the wrong sign for its [`ShellOrientation`].
    ShellReversed {
        shell: EntityId<Shell>,
        faces: usize,
    },
}

impl fmt::Display for HealOperation {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            HealOperation::VerticesMerged { kept, merged, gap } => write!(
                f,
                "merged {merged} coincident vertices into {kept:?} (gap {gap:.3e} mm)"
            ),
            HealOperation::EdgesWelded { kept, merged, gap } => write!(
                f,
                "welded {merged} duplicate edges onto {kept:?} (deviation {gap:.3e} mm)"
            ),
            HealOperation::ToleranceElevated {
                entity,
                new_tolerance,
            } => write!(f, "raised {entity:?} tolerance to {new_tolerance:.3e} mm"),
            HealOperation::FaceReoriented { face } => {
                write!(f, "reversed {face:?} to match its neighbours")
            }
            HealOperation::ShellReversed { shell, faces } => write!(
                f,
                "reversed {shell:?} ({faces} faces): enclosed volume had the wrong sign"
            ),
        }
    }
}

/// What one healing run did, and what it could not fix.
#[derive(Debug, Default)]
pub struct HealResult {
    /// Repairs applied, in the order they were applied.
    pub operations: Vec<HealOperation>,
    /// [`TopologyStore::check`] failures the body had on entry.
    pub failures_before: Vec<CheckFailure>,
    /// Failures still present on exit. Under [`HealStrategy::ReportOnly`] this
    /// is `failures_before` unchanged — nothing was applied, so nothing was
    /// re-checked.
    pub remaining: Vec<CheckFailure>,
    /// Repairs deliberately *not* attempted, and why (a merge that would have
    /// collapsed an edge, a tolerance that would have exceeded the kernel
    /// limit, a pass skipped for want of tessellable geometry).
    pub notes: Vec<String>,
}

impl HealResult {
    /// Whether the body entered invalid and left valid.
    pub fn healed(&self) -> bool {
        !self.failures_before.is_empty() && self.remaining.is_empty()
    }

    /// Whether anything was applied (or, under
    /// [`HealStrategy::ReportOnly`], proposed).
    pub fn is_empty(&self) -> bool {
        self.operations.is_empty()
    }
}

/// Phase-1 repair passes over an imported body (`spec/06-step-io.md` §6).
pub struct GeometryHealer;

impl GeometryHealer {
    /// Run every pass the strategy enables, then re-validate.
    ///
    /// The body is left valid, or left exactly as the passes got it — there
    /// is no partial rollback here. Callers that need the original body back
    /// (the STEP reader does, to fall through to its mesh path) must snapshot
    /// or rebuild it themselves.
    pub fn heal(
        body: EntityId<Body>,
        store: &mut TopologyStore,
        geo: &GeometryStore,
        options: &HealOptions,
    ) -> HealResult {
        let mut result = HealResult {
            failures_before: store.check(body),
            ..HealResult::default()
        };
        if store.body(body).is_none() {
            result.remaining = result.failures_before.clone();
            return result;
        }
        let strategy = options.strategy;

        if strategy.closes_gaps() {
            Self::fix_gaps_into(body, store, geo, options, &mut result);
        }
        if strategy.orients() {
            Self::fix_orientation_into(body, store, geo, strategy.applies(), &mut result);
        }

        if strategy.applies() {
            // Topology counts changed (welding removes edges and vertices),
            // so the genus the reader recovered at build time is stale.
            recover_genus(store, body);
            result.remaining = store.check(body);
        } else {
            result.remaining = result.failures_before.clone();
        }
        result
    }

    /// Close vertex gaps and weld the duplicate edges they expose.
    ///
    /// This is the sewing pass: it is what turns a shell whose faces each
    /// authored their own copy of every shared boundary into one whose edges
    /// have two fins apiece.
    pub fn fix_gaps(
        body: EntityId<Body>,
        store: &mut TopologyStore,
        geo: &GeometryStore,
        options: &HealOptions,
    ) -> HealResult {
        let mut result = HealResult {
            failures_before: store.check(body),
            ..HealResult::default()
        };
        Self::fix_gaps_into(body, store, geo, options, &mut result);
        result.remaining = if options.strategy.applies() {
            recover_genus(store, body);
            store.check(body)
        } else {
            result.failures_before.clone()
        };
        result
    }

    /// Make face orientation consistent across every shared edge, then
    /// reverse any shell left enclosing a signed volume of the wrong sign.
    pub fn fix_orientation(
        body: EntityId<Body>,
        store: &mut TopologyStore,
        geo: &GeometryStore,
    ) -> HealResult {
        let mut result = HealResult {
            failures_before: store.check(body),
            ..HealResult::default()
        };
        Self::fix_orientation_into(body, store, geo, true, &mut result);
        result.remaining = store.check(body);
        result
    }

    fn fix_gaps_into(
        body: EntityId<Body>,
        store: &mut TopologyStore,
        geo: &GeometryStore,
        options: &HealOptions,
        result: &mut HealResult,
    ) {
        let max_gap = resolve_max_gap(store, body, options.max_gap, &mut result.notes);
        let plan = plan_gaps(store, geo, body, max_gap, &mut result.notes);
        if !options.strategy.applies() {
            report_gap_plan(&plan, &mut result.operations);
            return;
        }
        apply_vertex_merges(store, &plan.vertices, &mut result.operations);
        apply_edge_welds(store, &plan.edges, &mut result.operations);
    }

    /// `apply` false plans and reports without touching the body — what
    /// [`HealStrategy::ReportOnly`] wants. Note that a dry run sees the body
    /// as authored, so on a shell that gap closure has not sewn yet it can
    /// only report what is visible *before* sewing: the two passes are
    /// planned independently, where an `Auto` run orients the sewn result.
    fn fix_orientation_into(
        body: EntityId<Body>,
        store: &mut TopologyStore,
        geo: &GeometryStore,
        apply: bool,
        result: &mut HealResult,
    ) {
        let Some(flips) = plan_face_flips(store, body, &mut result.notes) else {
            return;
        };
        let reversals = plan_shell_reversals(store, geo, body, &flips, &mut result.notes);
        if flips.is_empty() && reversals.is_empty() {
            return;
        }

        for &face in &flips {
            result
                .operations
                .push(HealOperation::FaceReoriented { face });
        }
        for &(shell, faces) in &reversals {
            result
                .operations
                .push(HealOperation::ShellReversed { shell, faces });
        }

        if !apply {
            return;
        }
        let reversed: Vec<EntityId<Shell>> = reversals.iter().map(|&(s, _)| s).collect();
        let mut flip_set: Vec<EntityId<Face>> = flips;
        for shell in reversed {
            for &face in store.faces_of_shell(shell) {
                match flip_set.iter().position(|&f| f == face) {
                    // Flipping twice is a no-op: drop both.
                    Some(i) => {
                        flip_set.swap_remove(i);
                    }
                    None => flip_set.push(face),
                }
            }
        }
        for face in flip_set {
            flip_face(store, face);
        }
    }
}

// ---------------------------------------------------------------------
// Gap closure
// ---------------------------------------------------------------------

/// One cluster of coincident vertices, collapsed to `kept` at `point`.
#[derive(Debug)]
struct VertexMerge {
    kept: EntityId<Vertex>,
    merged: Vec<EntityId<Vertex>>,
    point: Point3,
    tolerance: f64,
    gap: f64,
}

/// One group of duplicate edges, collapsed to `kept`. Each merged entry
/// carries whether it runs against `kept` (its fins' senses flip).
#[derive(Debug)]
struct EdgeWeld {
    kept: EntityId<Edge>,
    merged: Vec<(EntityId<Edge>, bool)>,
    tolerance: f64,
    gap: f64,
}

/// Edge-weld candidate grouping: an unordered post-merge vertex pair plus the
/// shell using it (`None` when the edge already spans two shells).
type WeldKey = (EntityId<Vertex>, EntityId<Vertex>, Option<EntityId<Shell>>);

/// A weld candidate: the edge, whether it runs against the group's
/// representative, and the largest sample deviation from it.
type WeldMember = (EntityId<Edge>, bool, f64);

#[derive(Debug, Default)]
struct GapPlan {
    vertices: Vec<VertexMerge>,
    edges: Vec<EdgeWeld>,
}

/// Resolve the merge tolerance: the caller's, or [`HEAL_GAP_REL`] of the
/// body's bounding-box diagonal. Always clamped to
/// [`MAX_ALLOWED_TOLERANCE`], since a merge widens a tolerance by up to half
/// the gap and the kernel rejects bodies past that limit anyway.
fn resolve_max_gap(
    store: &TopologyStore,
    body: EntityId<Body>,
    requested: Option<f64>,
    notes: &mut Vec<String>,
) -> f64 {
    let raw = match requested {
        Some(gap) if gap.is_finite() && gap > 0.0 => gap,
        Some(gap) => {
            notes.push(format!("ignoring non-positive max_gap {gap}"));
            derived_max_gap(store, body)
        }
        None => derived_max_gap(store, body),
    };
    if raw > MAX_ALLOWED_TOLERANCE {
        notes.push(format!(
            "clamped max_gap {raw:.3e} mm to the kernel limit {MAX_ALLOWED_TOLERANCE:.3e} mm"
        ));
        return MAX_ALLOWED_TOLERANCE;
    }
    raw.max(SYSTEM_RESOLUTION)
}

fn derived_max_gap(store: &TopologyStore, body: EntityId<Body>) -> f64 {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for vertex in body_vertices(store, body) {
        let p = store.vertex(vertex).expect("live vertex").point;
        for k in 0..3 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    let diagonal: f64 = (0..3)
        .map(|k| {
            let d = hi[k] - lo[k];
            if d.is_finite() { d * d } else { 0.0 }
        })
        .sum::<f64>()
        .sqrt();
    (diagonal * HEAL_GAP_REL).max(SYSTEM_RESOLUTION)
}

fn plan_gaps(
    store: &TopologyStore,
    geo: &GeometryStore,
    body: EntityId<Body>,
    max_gap: f64,
    notes: &mut Vec<String>,
) -> GapPlan {
    let merges = plan_vertex_merges(store, body, max_gap, notes);

    // Edge grouping keys use the *post-merge* vertex identity, so the plan is
    // computed in one pass with no mutation in between.
    let mut canonical: HashMap<EntityId<Vertex>, EntityId<Vertex>> = HashMap::new();
    for merge in &merges {
        for &v in &merge.merged {
            canonical.insert(v, merge.kept);
        }
    }
    let welds = plan_edge_welds(store, geo, body, max_gap, &canonical, notes);
    GapPlan {
        vertices: merges,
        edges: welds,
    }
}

fn plan_vertex_merges(
    store: &TopologyStore,
    body: EntityId<Body>,
    max_gap: f64,
    notes: &mut Vec<String>,
) -> Vec<VertexMerge> {
    let vertices = body_vertices(store, body);
    if vertices.len() < 2 {
        return Vec::new();
    }
    let index: HashMap<EntityId<Vertex>, usize> =
        vertices.iter().enumerate().map(|(i, &v)| (v, i)).collect();
    let points: Vec<Point3> = vertices
        .iter()
        .map(|&v| store.vertex(v).expect("live vertex").point)
        .collect();

    // Hash the points onto a grid of `max_gap` cells and union each point
    // with the already-placed points of its 27-cell neighbourhood: linear in
    // the vertex count, unlike the all-pairs scan a 10^5-vertex import would
    // choke on.
    let cell = max_gap.max(SYSTEM_RESOLUTION);
    let key = |p: &Point3| -> [i64; 3] {
        [
            (p.x / cell).floor() as i64,
            (p.y / cell).floor() as i64,
            (p.z / cell).floor() as i64,
        ]
    };
    let mut grid: HashMap<[i64; 3], Vec<usize>> = HashMap::new();
    let mut dsu = DisjointSet::new(vertices.len());
    for (i, p) in points.iter().enumerate() {
        if !p.coords.iter().all(|c| c.is_finite()) {
            continue;
        }
        let k = key(p);
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let neighbour = [k[0] + dx, k[1] + dy, k[2] + dz];
                    for &j in grid.get(&neighbour).map(Vec::as_slice).unwrap_or(&[]) {
                        if (points[j] - p).norm() <= max_gap {
                            dsu.union(i, j);
                        }
                    }
                }
            }
        }
        grid.entry(k).or_default().push(i);
    }

    let mut clusters: HashMap<usize, Vec<usize>> = HashMap::new();
    for i in 0..vertices.len() {
        clusters.entry(dsu.find(i)).or_default().push(i);
    }

    // A cluster spanning two shells would sew them together at a point, which
    // is non-manifold between shells; a cluster containing both ends of an
    // edge would collapse that edge to nothing. Neither is a phase-1 repair.
    let shell_of = vertex_shells(store, body);
    let mut spans_edge: HashMap<usize, bool> = HashMap::new();
    for edge_id in body_edges(store, body) {
        let edge = store.edge(edge_id).expect("live edge");
        if edge.start_vertex == edge.end_vertex {
            continue;
        }
        let (Some(&a), Some(&b)) = (index.get(&edge.start_vertex), index.get(&edge.end_vertex))
        else {
            continue;
        };
        if dsu.find(a) == dsu.find(b) {
            spans_edge.insert(dsu.find(a), true);
        }
    }

    let mut merges = Vec::new();
    let mut roots: Vec<usize> = clusters.keys().copied().collect();
    // Deterministic output order: arena ids are assigned in build order.
    roots.sort_unstable();
    for root in roots {
        let mut members = clusters.remove(&root).expect("root came from the map");
        if members.len() < 2 {
            continue;
        }
        members.sort_unstable();
        if spans_edge.contains_key(&root) {
            notes.push(format!(
                "skipped merging {} vertices near {:?}: an edge joins two of them \
                 (collapsing it is a phase-2 repair)",
                members.len(),
                points[members[0]]
            ));
            continue;
        }
        let shells: Vec<Option<EntityId<Shell>>> = members
            .iter()
            .map(|&i| shell_of.get(&vertices[i]).copied().flatten())
            .collect();
        if shells.windows(2).any(|w| w[0] != w[1]) {
            notes.push(format!(
                "skipped merging {} vertices near {:?}: they belong to different shells",
                members.len(),
                points[members[0]]
            ));
            continue;
        }

        let centroid = Point3::from(
            members
                .iter()
                .map(|&i| points[i].coords)
                .sum::<opensolid_core::Vector3>()
                / members.len() as f64,
        );
        let gap = members
            .iter()
            .map(|&i| (points[i] - centroid).norm())
            .fold(0.0, f64::max);
        let existing = members
            .iter()
            .map(|&i| store.vertex(vertices[i]).expect("live vertex").tolerance)
            .fold(SYSTEM_RESOLUTION, f64::max);
        let tolerance = existing.max(gap);
        if tolerance > MAX_ALLOWED_TOLERANCE {
            notes.push(format!(
                "skipped merging {} vertices near {:?}: the merge needs tolerance \
                 {tolerance:.3e} mm, past the kernel limit {MAX_ALLOWED_TOLERANCE:.3e} mm",
                members.len(),
                points[members[0]]
            ));
            continue;
        }
        merges.push(VertexMerge {
            kept: vertices[members[0]],
            merged: members[1..].iter().map(|&i| vertices[i]).collect(),
            point: centroid,
            tolerance,
            gap,
        });
    }
    merges
}

fn plan_edge_welds(
    store: &TopologyStore,
    geo: &GeometryStore,
    body: EntityId<Body>,
    max_gap: f64,
    canonical: &HashMap<EntityId<Vertex>, EntityId<Vertex>>,
    notes: &mut Vec<String>,
) -> Vec<EdgeWeld> {
    let canon = |v: EntityId<Vertex>| canonical.get(&v).copied().unwrap_or(v);
    let shell_of = edge_shells(store, body);
    // `EntityId` is deliberately not `Ord` (an arena slot has no meaningful
    // order), so an *unordered* vertex pair is keyed by discovery rank.
    let rank: HashMap<EntityId<Vertex>, usize> = body_vertices(store, body)
        .into_iter()
        .enumerate()
        .map(|(i, v)| (v, i))
        .collect();
    let ordered = |a: EntityId<Vertex>, b: EntityId<Vertex>| {
        let (ra, rb) = (rank.get(&a).copied(), rank.get(&b).copied());
        if ra <= rb { (a, b) } else { (b, a) }
    };

    // Candidate group: same (unordered) post-merge vertex pair, same shell.
    let mut groups: HashMap<WeldKey, Vec<EntityId<Edge>>> = HashMap::new();
    let mut order: Vec<WeldKey> = Vec::new();
    for edge_id in body_edges(store, body) {
        let edge = store.edge(edge_id).expect("live edge");
        let (a, b) = (canon(edge.start_vertex), canon(edge.end_vertex));
        let pair = ordered(a, b);
        let key = (pair.0, pair.1, shell_of.get(&edge_id).copied().flatten());
        let slot = groups.entry(key).or_insert_with(|| {
            order.push(key);
            Vec::new()
        });
        slot.push(edge_id);
    }

    let mut welds = Vec::new();
    for key in order {
        let group = groups.remove(&key).expect("key came from the same pass");
        if group.len() < 2 {
            continue;
        }
        // Two edges over one vertex pair need not be duplicates — the two
        // halves of a split circle share both ends. Sampling the curves tells
        // them apart, and tells a duplicate's direction at the same time.
        let mut samples: Vec<(EntityId<Edge>, [Point3; SAMPLE_FRACTIONS.len()])> = Vec::new();
        for &edge_id in &group {
            match edge_samples(store, geo, edge_id) {
                Some(points) => samples.push((edge_id, points)),
                None => notes.push(format!(
                    "cannot weld {edge_id:?}: it has no attached curve geometry to compare"
                )),
            }
        }

        // Greedy clustering against each cluster's representative: duplicate
        // counts per vertex pair are tiny (2 for a sewn-up shell, 3-4 for a
        // non-manifold one), so the quadratic scan is free.
        let mut clusters: Vec<(usize, Vec<WeldMember>)> = Vec::new();
        'candidate: for (idx, (edge_id, points)) in samples.iter().enumerate() {
            for (rep, members) in clusters.iter_mut() {
                let (_, rep_points) = &samples[*rep];
                if let Some((reversed, deviation)) = curve_match(rep_points, points, max_gap) {
                    members.push((*edge_id, reversed, deviation));
                    continue 'candidate;
                }
            }
            clusters.push((idx, Vec::new()));
        }

        for (rep, members) in clusters {
            if members.is_empty() {
                continue;
            }
            let kept = samples[rep].0;
            let gap = members.iter().map(|&(_, _, d)| d).fold(0.0, f64::max);
            let existing = std::iter::once(kept)
                .chain(members.iter().map(|&(e, _, _)| e))
                .map(|e| store.edge(e).expect("live edge").tolerance)
                .fold(SYSTEM_RESOLUTION, f64::max);
            let tolerance = existing.max(gap);
            if tolerance > MAX_ALLOWED_TOLERANCE {
                notes.push(format!(
                    "skipped welding {} edges onto {kept:?}: the weld needs tolerance \
                     {tolerance:.3e} mm, past the kernel limit {MAX_ALLOWED_TOLERANCE:.3e} mm",
                    members.len()
                ));
                continue;
            }
            welds.push(EdgeWeld {
                kept,
                merged: members.iter().map(|&(e, r, _)| (e, r)).collect(),
                tolerance,
                gap,
            });
        }
    }
    welds
}

/// Whether two sampled curves are the same curve, and if so whether the
/// second runs against the first. Compares forward and reversed sample orders
/// and takes whichever fits inside `tolerance`.
fn curve_match(
    a: &[Point3; SAMPLE_FRACTIONS.len()],
    b: &[Point3; SAMPLE_FRACTIONS.len()],
    tolerance: f64,
) -> Option<(bool, f64)> {
    let n = SAMPLE_FRACTIONS.len();
    let forward = (0..n).map(|i| (a[i] - b[i]).norm()).fold(0.0f64, f64::max);
    let reverse = (0..n)
        .map(|i| (a[i] - b[n - 1 - i]).norm())
        .fold(0.0f64, f64::max);
    if forward <= tolerance && forward <= reverse {
        Some((false, forward))
    } else if reverse <= tolerance {
        Some((true, reverse))
    } else {
        None
    }
}

fn edge_samples(
    store: &TopologyStore,
    geo: &GeometryStore,
    edge_id: EntityId<Edge>,
) -> Option<[Point3; SAMPLE_FRACTIONS.len()]> {
    let edge = store.edge(edge_id)?;
    let curve = geo.curve(edge.curve?)?;
    let span = edge.t_end - edge.t_start;
    let mut out = [Point3::origin(); SAMPLE_FRACTIONS.len()];
    for (slot, f) in out.iter_mut().zip(SAMPLE_FRACTIONS) {
        *slot = curve.point(edge.t_start + span * f);
    }
    Some(out)
}

fn report_gap_plan(plan: &GapPlan, operations: &mut Vec<HealOperation>) {
    for merge in &plan.vertices {
        operations.push(HealOperation::VerticesMerged {
            kept: merge.kept,
            merged: merge.merged.len(),
            gap: merge.gap,
        });
    }
    for weld in &plan.edges {
        operations.push(HealOperation::EdgesWelded {
            kept: weld.kept,
            merged: weld.merged.len(),
            gap: weld.gap,
        });
    }
}

fn apply_vertex_merges(
    store: &mut TopologyStore,
    merges: &[VertexMerge],
    operations: &mut Vec<HealOperation>,
) {
    for merge in merges {
        let mut adopted: Vec<EntityId<Edge>> = Vec::new();
        for &victim in &merge.merged {
            let Some(vertex) = store.vertices.remove(victim) else {
                continue;
            };
            for edge_id in vertex.edges {
                let Some(edge) = store.edges.get_mut(edge_id) else {
                    continue;
                };
                if edge.start_vertex == victim {
                    edge.start_vertex = merge.kept;
                }
                if edge.end_vertex == victim {
                    edge.end_vertex = merge.kept;
                }
                if !adopted.contains(&edge_id) {
                    adopted.push(edge_id);
                }
            }
        }
        let Some(kept) = store.vertices.get_mut(merge.kept) else {
            continue;
        };
        kept.point = merge.point;
        let elevated = merge.tolerance > kept.tolerance;
        kept.tolerance = kept.tolerance.max(merge.tolerance);
        for edge_id in adopted {
            if !kept.edges.contains(&edge_id) {
                kept.edges.push(edge_id);
            }
        }
        operations.push(HealOperation::VerticesMerged {
            kept: merge.kept,
            merged: merge.merged.len(),
            gap: merge.gap,
        });
        if elevated {
            operations.push(HealOperation::ToleranceElevated {
                entity: EntityRef::Vertex(merge.kept),
                new_tolerance: merge.tolerance,
            });
        }
    }
}

fn apply_edge_welds(
    store: &mut TopologyStore,
    welds: &[EdgeWeld],
    operations: &mut Vec<HealOperation>,
) {
    for weld in welds {
        if store.edges.get(weld.kept).is_none() {
            continue;
        }
        let mut moved = 0;
        for &(victim, reversed) in &weld.merged {
            let Some(edge) = store.edges.remove(victim) else {
                continue;
            };
            for &fin_id in &edge.fins {
                let Some(fin) = store.fins.get_mut(fin_id) else {
                    continue;
                };
                fin.edge = weld.kept;
                if reversed {
                    fin.sense = fin.sense.opposite();
                }
                fin.mate = None;
                let kept = store.edges.get_mut(weld.kept).expect("checked above");
                if !kept.fins.contains(&fin_id) {
                    kept.fins.push(fin_id);
                }
            }
            // The victim's curve stays in the geometry store: curves may be
            // shared, and an orphan curve is invisible to `check` where a
            // dangling id would not be.
            for vertex_id in [edge.start_vertex, edge.end_vertex] {
                if let Some(vertex) = store.vertices.get_mut(vertex_id) {
                    vertex.edges.retain(|&e| e != victim);
                }
            }
            moved += 1;
        }
        if moved == 0 {
            continue;
        }

        let kept = store.edges.get_mut(weld.kept).expect("checked above");
        let elevated = weld.tolerance > kept.tolerance;
        kept.tolerance = kept.tolerance.max(weld.tolerance);
        let fins = kept.fins.clone();
        // Exactly two fins is the manifold case, and the only one where a
        // mate link is meaningful; `check` reports anything else.
        if fins.len() == 2 {
            store.fins.get_mut(fins[0]).expect("live fin").mate = Some(fins[1]);
            store.fins.get_mut(fins[1]).expect("live fin").mate = Some(fins[0]);
        }
        // Both endpoints must list the surviving edge — a fin adopted from a
        // victim may be the only user of a vertex the kept edge never saw.
        let (start, end) = {
            let kept = store.edges.get(weld.kept).expect("checked above");
            (kept.start_vertex, kept.end_vertex)
        };
        for vertex_id in [start, end] {
            if let Some(vertex) = store.vertices.get_mut(vertex_id) {
                if !vertex.edges.contains(&weld.kept) {
                    vertex.edges.push(weld.kept);
                }
            }
        }

        operations.push(HealOperation::EdgesWelded {
            kept: weld.kept,
            merged: moved,
            gap: weld.gap,
        });
        if elevated {
            operations.push(HealOperation::ToleranceElevated {
                entity: EntityRef::Edge(weld.kept),
                new_tolerance: weld.tolerance,
            });
        }
    }
}

// ---------------------------------------------------------------------
// Orientation repair
// ---------------------------------------------------------------------

/// Two-colour each shell's face-adjacency graph so that every pair of mated
/// fins traverses its edge in opposite directions, and return the faces that
/// must flip. `None` means the constraint is unsatisfiable (an odd cycle —
/// the shell is non-orientable, or its mate links are corrupt), in which case
/// no flip is safe.
fn plan_face_flips(
    store: &TopologyStore,
    body: EntityId<Body>,
    notes: &mut Vec<String>,
) -> Option<Vec<EntityId<Face>>> {
    let mut flips = Vec::new();
    for &shell in store.shells_of_body(body) {
        let faces = store.faces_of_shell(shell).to_vec();
        let mut colour: HashMap<EntityId<Face>, bool> = HashMap::new();
        for &seed in &faces {
            if colour.contains_key(&seed) {
                continue;
            }
            // One connected component. Its seed is held fixed, so the whole
            // component may come out globally reversed; the volume-sign pass
            // below is what settles that.
            let mut component = vec![seed];
            colour.insert(seed, false);
            let mut queue = vec![seed];
            while let Some(face) = queue.pop() {
                let flip_a = colour[&face];
                for fin_id in fins_of_face(store, face) {
                    let Some(mate_id) = store.fin_mate(fin_id) else {
                        continue;
                    };
                    let (Some(fin), Some(mate)) = (store.fin(fin_id), store.fin(mate_id)) else {
                        continue;
                    };
                    if fin.edge != mate.edge {
                        // Corrupt mate link; `check` reports it, and it makes
                        // the colouring meaningless.
                        notes.push(format!(
                            "orientation repair skipped: {fin_id:?} is mated across two edges"
                        ));
                        return None;
                    }
                    let neighbour = store.fin_face(mate_id);
                    // Consistent means the effective senses differ, where
                    // "effective" is the authored sense XOR the face's flip.
                    let same = forward(fin.sense) == forward(mate.sense);
                    let flip_b = same != flip_a;
                    match colour.get(&neighbour) {
                        Some(&existing) if existing != flip_b => {
                            notes.push(format!(
                                "orientation repair skipped: {shell:?} is not consistently \
                                 orientable (conflict at {neighbour:?})"
                            ));
                            return None;
                        }
                        Some(_) => {}
                        None => {
                            colour.insert(neighbour, flip_b);
                            component.push(neighbour);
                            queue.push(neighbour);
                        }
                    }
                }
            }
            // Either colour class is a valid answer; flipping the smaller one
            // keeps the operation log proportional to the actual defect.
            let flipped = component.iter().filter(|f| colour[f]).count();
            let invert = flipped * 2 > component.len();
            for face in component {
                if colour[&face] != invert {
                    flips.push(face);
                }
            }
        }
    }
    Some(flips)
}

/// Shells left enclosing a signed volume of the wrong sign for their
/// [`ShellOrientation`] once `flips` is applied, with their face counts.
///
/// A file can be perfectly self-consistent and still inside out — every face
/// authored backwards agrees with every other — which no combinatorial check
/// can see. Only a geometric measurement can, so this is the one pass that
/// looks at the surfaces rather than the graph.
///
/// The measurement is virtual: flipping a face's sense negates its
/// tessellated contribution exactly (planar faces are triangulated about the
/// sense-adjusted normal, quadric grids wound by the same flag), so the
/// post-flip total is a signed sum over the pre-flip contributions. That
/// keeps [`HealStrategy::ReportOnly`] honest — it reports the same reversal
/// an `Auto` run would apply, without touching the body.
///
/// Only structurally closed shells are measured: an open shell encloses no
/// volume, so the sum's sign would be an artefact of where the hole is.
fn plan_shell_reversals(
    store: &TopologyStore,
    geo: &GeometryStore,
    body: EntityId<Body>,
    flips: &[EntityId<Face>],
    notes: &mut Vec<String>,
) -> Vec<(EntityId<Shell>, usize)> {
    let mut reversals = Vec::new();
    for &shell in store.shells_of_body(body) {
        let faces = store.faces_of_shell(shell).to_vec();
        if !shell_is_closed(store, &faces) {
            continue;
        }
        let mut total = 0.0;
        let mut magnitude = 0.0;
        let mut measurable = true;
        for &face in &faces {
            let Ok(mesh) = tessellate_face(store, geo, face, &TessellationOptions::default())
            else {
                // NURBS patches and trimmed quadrics have no tessellator yet;
                // without every face the sum is not a closed volume.
                notes.push(format!(
                    "shell orientation left as authored: {face:?} cannot be tessellated"
                ));
                measurable = false;
                break;
            };
            let contribution: f64 = mesh
                .indices
                .iter()
                .map(|tri| {
                    let [a, b, c] = tri.map(|i| mesh.positions[i].coords);
                    a.dot(&b.cross(&c)) / 6.0
                })
                .sum();
            let signed = if flips.contains(&face) {
                -contribution
            } else {
                contribution
            };
            total += signed;
            magnitude += signed.abs();
        }
        if !measurable || magnitude <= 0.0 {
            continue;
        }
        // An outward shell bounds positive volume; a void shell's normals
        // point into the cavity, so it bounds negative volume.
        let want_positive =
            store.shell(shell).expect("live shell").orientation == ShellOrientation::Outward;
        let inverted = if want_positive {
            total < 0.0
        } else {
            total > 0.0
        };
        // Only a decisive sign counts: a sum that is numerical noise against
        // the faces' own magnitudes says the shell does not close, not that
        // it is inside out.
        if inverted && total.abs() > magnitude * 1e-6 {
            reversals.push((shell, faces.len()));
        }
    }
    reversals
}

/// Whether every edge these faces use is shared by exactly two of them —
/// the structural closure that makes an enclosed volume meaningful.
fn shell_is_closed(store: &TopologyStore, faces: &[EntityId<Face>]) -> bool {
    faces.iter().all(|&face| {
        store
            .edges_of_face(face)
            .into_iter()
            .all(|edge| store.fins_of_edge(edge).len() == 2)
    })
}

/// Reverse one face: its surface sense, and the traversal of every loop on
/// it. Both together, so the face's outward normal and its loops' winding
/// stay consistent with each other.
fn flip_face(store: &mut TopologyStore, face: EntityId<Face>) {
    let Some(face_ref) = store.faces.get_mut(face) else {
        return;
    };
    face_ref.sense = match face_ref.sense {
        FaceSense::Positive => FaceSense::Negative,
        FaceSense::Negative => FaceSense::Positive,
    };
    for loop_id in store.loops_of_face(face) {
        let mut fins = store.loops.get(loop_id).expect("live loop").fins.clone();
        if fins.is_empty() {
            continue;
        }
        fins.reverse();
        let n = fins.len();
        for (i, &fin_id) in fins.iter().enumerate() {
            let fin = store.fins.get_mut(fin_id).expect("live fin");
            fin.sense = fin.sense.opposite();
            fin.next = Some(fins[(i + 1) % n]);
            fin.prev = Some(fins[(i + n - 1) % n]);
        }
        store.loops.get_mut(loop_id).expect("live loop").fins = fins;
    }
}

fn forward(sense: FinSense) -> bool {
    sense == FinSense::Forward
}

// ---------------------------------------------------------------------
// Shared helpers
// ---------------------------------------------------------------------

/// Recover a body's shell genus from the Euler-Poincaré formula.
///
/// STEP carries no genus, and healing changes the vertex and edge counts it
/// would be derived from, so both the reader (at build time) and the healer
/// (after any topology change) recompute it here. Any residual genus is
/// attributed to the first shell: a handled *void* would mis-assign, and then
/// fail `check` into the mesh fallback rather than import wrongly. An odd or
/// negative implied genus is not a genus at all — it means the topology is
/// inconsistent, and the formula is left to fail in `check`.
pub(crate) fn recover_genus(store: &mut TopologyStore, body: EntityId<Body>) {
    let Some(&shell) = store.body(body).and_then(|b| b.shells.first()) else {
        return;
    };
    let counts = store.euler_counts(body);
    // `euler_counts` folds the current genus into `counts.genus`; the formula
    // below solves for it afresh, so start from the un-attributed total.
    let others: i64 = store
        .shells_of_body(body)
        .iter()
        .skip(1)
        .map(|&s| store.shell(s).expect("live shell").genus as i64)
        .sum();
    let euler =
        counts.vertices as i64 - counts.edges as i64 + counts.faces as i64 - counts.rings as i64;
    let genus_x2 = 2 * counts.shells as i64 - euler - 2 * others;
    if genus_x2 >= 0 && genus_x2 % 2 == 0 {
        store.shells.get_mut(shell).expect("live shell").genus = (genus_x2 / 2) as u32;
    }
}

fn fins_of_face(store: &TopologyStore, face: EntityId<Face>) -> Vec<EntityId<Fin>> {
    store
        .loops_of_face(face)
        .into_iter()
        .flat_map(|loop_id| store.fins_of_loop(loop_id).to_vec())
        .collect()
}

/// Every vertex reachable from `body`, deduplicated, in a deterministic order.
fn body_vertices(store: &TopologyStore, body: EntityId<Body>) -> Vec<EntityId<Vertex>> {
    let mut out = Vec::new();
    for face in store.faces_of_body(body) {
        for loop_id in store.loops_of_face(face) {
            if let Some(v) = store.loop_(loop_id).and_then(|l| l.vertex) {
                if !out.contains(&v) {
                    out.push(v);
                }
            }
            for &fin_id in store.fins_of_loop(loop_id) {
                let edge_id = store.fin_edge(fin_id);
                let Some(edge) = store.edge(edge_id) else {
                    continue;
                };
                for v in [edge.start_vertex, edge.end_vertex] {
                    if !out.contains(&v) {
                        out.push(v);
                    }
                }
            }
        }
    }
    out
}

/// Every edge reachable from `body`, deduplicated, in a deterministic order.
fn body_edges(store: &TopologyStore, body: EntityId<Body>) -> Vec<EntityId<Edge>> {
    let mut out = Vec::new();
    for face in store.faces_of_body(body) {
        for edge in store.edges_of_face(face) {
            if !out.contains(&edge) {
                out.push(edge);
            }
        }
    }
    out
}

/// Vertex → the single shell using it. `None` for a vertex already shared
/// between shells: it is exactly the case healing must not make worse.
fn vertex_shells(
    store: &TopologyStore,
    body: EntityId<Body>,
) -> HashMap<EntityId<Vertex>, Option<EntityId<Shell>>> {
    let mut out: HashMap<EntityId<Vertex>, Option<EntityId<Shell>>> = HashMap::new();
    for &shell in store.shells_of_body(body) {
        for &face in store.faces_of_shell(shell) {
            for edge_id in store.edges_of_face(face) {
                let Some(edge) = store.edge(edge_id) else {
                    continue;
                };
                for v in [edge.start_vertex, edge.end_vertex] {
                    out.entry(v)
                        .and_modify(|slot| {
                            if *slot != Some(shell) {
                                *slot = None;
                            }
                        })
                        .or_insert(Some(shell));
                }
            }
        }
    }
    out
}

/// Edge → the single shell using it, with the same `None` convention as
/// [`vertex_shells`].
fn edge_shells(
    store: &TopologyStore,
    body: EntityId<Body>,
) -> HashMap<EntityId<Edge>, Option<EntityId<Shell>>> {
    let mut out: HashMap<EntityId<Edge>, Option<EntityId<Shell>>> = HashMap::new();
    for &shell in store.shells_of_body(body) {
        for &face in store.faces_of_shell(shell) {
            for edge in store.edges_of_face(face) {
                out.entry(edge)
                    .and_modify(|slot| {
                        if *slot != Some(shell) {
                            *slot = None;
                        }
                    })
                    .or_insert(Some(shell));
            }
        }
    }
    out
}

/// Union-find over vertex indices, with path halving and union by size.
struct DisjointSet {
    parent: Vec<usize>,
    size: Vec<usize>,
}

impl DisjointSet {
    fn new(n: usize) -> Self {
        Self {
            parent: (0..n).collect(),
            size: vec![1; n],
        }
    }

    fn find(&mut self, mut i: usize) -> usize {
        while self.parent[i] != i {
            self.parent[i] = self.parent[self.parent[i]];
            i = self.parent[i];
        }
        i
    }

    fn union(&mut self, a: usize, b: usize) {
        let (mut ra, mut rb) = (self.find(a), self.find(b));
        if ra == rb {
            return;
        }
        if self.size[ra] < self.size[rb] {
            std::mem::swap(&mut ra, &mut rb);
        }
        self.parent[rb] = ra;
        self.size[ra] += self.size[rb];
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use opensolid_brep::{BodyType, Curve3, LoopType, Surface3, tessellate_body};
    use opensolid_core::Vector3;

    /// A unit block built face-by-face, each face optionally authoring its
    /// own copy of the vertices and edges it shares with its neighbours.
    ///
    /// `jitter` displaces each face's private copy of a shared corner by that
    /// distance along a per-face direction, modelling the last-decimal
    /// disagreement a real exporter leaves behind.
    struct BlockFixture {
        store: TopologyStore,
        geo: GeometryStore,
        body: EntityId<Body>,
    }

    const CORNERS: [[f64; 3]; 8] = [
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [1.0, 1.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 1.0],
        [1.0, 1.0, 1.0],
        [0.0, 1.0, 1.0],
    ];

    /// Vertex cycles counterclockwise viewed from outside, with the outward
    /// normal each implies.
    const FACES: [([usize; 4], [f64; 3]); 6] = [
        ([0, 3, 2, 1], [0.0, 0.0, -1.0]),
        ([4, 5, 6, 7], [0.0, 0.0, 1.0]),
        ([0, 1, 5, 4], [0.0, -1.0, 0.0]),
        ([1, 2, 6, 5], [1.0, 0.0, 0.0]),
        ([2, 3, 7, 6], [0.0, 1.0, 0.0]),
        ([3, 0, 4, 7], [-1.0, 0.0, 0.0]),
    ];

    /// Build a block where every face carries its own vertices and edges —
    /// an unsewn shell, as an exporter that never merged its boundaries
    /// writes. With `jitter = 0` the duplicates are exactly coincident;
    /// with `jitter > 0` they disagree by that distance.
    fn unsewn_block(jitter: f64, reversed_faces: &[usize]) -> BlockFixture {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = store.create_body(BodyType::Solid);
        let shell = store.create_shell(body, true, ShellOrientation::Outward);

        for (f, (cycle, normal)) in FACES.iter().enumerate() {
            let normal = Vector3::new(normal[0], normal[1], normal[2]);
            // A per-face nudge direction that is not parallel to any other
            // face's, so coincident corners scatter rather than stack.
            let nudge = Vector3::new(
                ((f + 1) as f64 * 0.7).sin(),
                ((f + 1) as f64 * 1.3).sin(),
                ((f + 1) as f64 * 2.1).sin(),
            )
            .normalize()
                * jitter;

            let points: Vec<Point3> = cycle
                .iter()
                .map(|&c| Point3::new(CORNERS[c][0], CORNERS[c][1], CORNERS[c][2]) + nudge)
                .collect();
            let vertices: Vec<_> = points
                .iter()
                .map(|&p| store.create_vertex(p, SYSTEM_RESOLUTION))
                .collect();

            let face = store.create_face(shell, FaceSense::Positive);
            let surface =
                geo.add_surface(Surface3::plane(points[0], normal).expect("axis-aligned plane"));
            store.faces.get_mut(face).expect("live face").surface = Some(surface);

            let mut fins = Vec::new();
            for k in 0..4 {
                let (a, b) = (k, (k + 1) % 4);
                let direction = points[b] - points[a];
                let length = direction.norm();
                let curve = geo.add_curve(
                    Curve3::line(points[a], direction / length).expect("nonzero direction"),
                );
                let edge = store.create_edge_with_curve(
                    vertices[a],
                    vertices[b],
                    SYSTEM_RESOLUTION,
                    curve,
                    0.0,
                    length,
                );
                fins.push((edge, FinSense::Forward));
            }
            if reversed_faces.contains(&f) {
                reverse_authored_face(&mut store, face, &mut fins);
            }
            store.create_loop(face, LoopType::Outer, &fins);
        }
        recover_genus(&mut store, body);
        BlockFixture { store, geo, body }
    }

    /// Build a properly sewn block, then break only the orientation of the
    /// listed faces.
    fn sewn_block(reversed_faces: &[usize]) -> BlockFixture {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = store.create_body(BodyType::Solid);
        let shell = store.create_shell(body, true, ShellOrientation::Outward);

        let points: Vec<Point3> = CORNERS
            .iter()
            .map(|c| Point3::new(c[0], c[1], c[2]))
            .collect();
        let vertices: Vec<_> = points
            .iter()
            .map(|&p| store.create_vertex(p, SYSTEM_RESOLUTION))
            .collect();
        let mut edges: HashMap<(usize, usize), EntityId<Edge>> = HashMap::new();

        for (f, (cycle, normal)) in FACES.iter().enumerate() {
            let normal = Vector3::new(normal[0], normal[1], normal[2]);
            let face = store.create_face(shell, FaceSense::Positive);
            let surface = geo.add_surface(
                Surface3::plane(points[cycle[0]], normal).expect("axis-aligned plane"),
            );
            store.faces.get_mut(face).expect("live face").surface = Some(surface);

            let mut fins = Vec::new();
            for k in 0..4 {
                let (from, to) = (cycle[k], cycle[(k + 1) % 4]);
                let key = if from < to { (from, to) } else { (to, from) };
                let edge = *edges.entry(key).or_insert_with(|| {
                    let direction = points[key.1] - points[key.0];
                    let length = direction.norm();
                    let curve = geo.add_curve(
                        Curve3::line(points[key.0], direction / length).expect("nonzero direction"),
                    );
                    store.create_edge_with_curve(
                        vertices[key.0],
                        vertices[key.1],
                        SYSTEM_RESOLUTION,
                        curve,
                        0.0,
                        length,
                    )
                });
                let sense = if from == key.0 {
                    FinSense::Forward
                } else {
                    FinSense::Reversed
                };
                fins.push((edge, sense));
            }
            if reversed_faces.contains(&f) {
                reverse_authored_face(&mut store, face, &mut fins);
            }
            store.create_loop(face, LoopType::Outer, &fins);
        }
        recover_genus(&mut store, body);
        BlockFixture { store, geo, body }
    }

    /// Author one face's *whole* use backwards — the surface sense and the
    /// loop traversal together — which is how a misoriented face arrives in
    /// a real file: the exporter emitted the face use against the shell, not
    /// half of it. (A face whose sense disagrees with its own winding is a
    /// different, geometry-only defect that `check` cannot see; repairing it
    /// is of-3qy.15.)
    fn reverse_authored_face(
        store: &mut TopologyStore,
        face: EntityId<Face>,
        fins: &mut [(EntityId<Edge>, FinSense)],
    ) {
        fins.reverse();
        for (_, sense) in fins.iter_mut() {
            *sense = sense.opposite();
        }
        store.faces.get_mut(face).expect("live face").sense = FaceSense::Negative;
    }

    fn signed_volume(fixture: &BlockFixture) -> f64 {
        let mesh = tessellate_body(
            &fixture.store,
            &fixture.geo,
            fixture.body,
            &TessellationOptions::default(),
        )
        .expect("block tessellates");
        mesh.indices
            .iter()
            .map(|tri| {
                let [a, b, c] = tri.map(|i| mesh.positions[i].coords);
                a.dot(&b.cross(&c)) / 6.0
            })
            .sum()
    }

    #[test]
    fn unsewn_block_is_invalid_before_healing() {
        let f = unsewn_block(0.0, &[]);
        let failures = f.store.check(f.body);
        assert!(
            failures
                .iter()
                .any(|x| matches!(x, CheckFailure::OpenEdgeInClosedShell { .. })),
            "an unsewn shell's edges each have one fin: {failures:?}"
        );
        assert_eq!(f.store.vertices.len(), 24, "6 faces x 4 private corners");
        assert_eq!(f.store.edges.len(), 24, "6 faces x 4 private edges");
    }

    #[test]
    fn healing_sews_an_exactly_coincident_block() {
        let mut f = unsewn_block(0.0, &[]);
        let result = GeometryHealer::heal(f.body, &mut f.store, &f.geo, &HealOptions::default());
        assert!(result.healed(), "remaining: {:?}", result.remaining);
        assert_eq!(f.store.check(f.body), vec![]);
        assert_eq!(f.store.vertices.len(), 8, "24 corners collapse to 8");
        assert_eq!(f.store.edges.len(), 12, "24 half-edges weld to 12");
        for edge in body_edges(&f.store, f.body) {
            assert_eq!(
                f.store.fins_of_edge(edge).len(),
                2,
                "{edge:?} must be shared by two faces after sewing"
            );
        }
        assert!((signed_volume(&f) - 1.0).abs() < 1e-9, "unit block volume");
    }

    #[test]
    fn healing_closes_a_real_gap_and_records_the_tolerance() {
        // A micron of disagreement on a 1 mm block — inside the derived
        // tolerance (1e-5 of the 1.73 mm diagonal), as export round-off is.
        let mut f = unsewn_block(1e-6, &[]);
        let result = GeometryHealer::heal(f.body, &mut f.store, &f.geo, &HealOptions::default());
        assert!(result.healed(), "remaining: {:?}", result.remaining);
        assert_eq!(f.store.vertices.len(), 8);
        assert_eq!(f.store.edges.len(), 12);

        // The closed distance must be absorbed as tolerance, not snapped away.
        for vertex in body_vertices(&f.store, f.body) {
            let v = f.store.vertex(vertex).expect("live vertex");
            assert!(v.is_tolerant(), "{vertex:?} should carry the closed gap");
            assert!(v.tolerance <= MAX_ALLOWED_TOLERANCE);
        }
        assert!(
            result
                .operations
                .iter()
                .any(|op| matches!(op, HealOperation::ToleranceElevated { .. })),
            "the elevation must be reported: {:?}",
            result.operations
        );
    }

    #[test]
    fn gap_wider_than_the_tolerance_is_left_alone() {
        // A 0.2 mm gap on a 1 mm block is a modelling error, not export
        // round-off: healing must not weld it shut.
        let mut f = unsewn_block(0.2, &[]);
        let before = f.store.check(f.body).len();
        let result = GeometryHealer::heal(f.body, &mut f.store, &f.geo, &HealOptions::default());
        assert!(!result.healed());
        assert_eq!(f.store.vertices.len(), 24, "nothing merged");
        assert!(f.store.check(f.body).len() >= before.min(1));
    }

    #[test]
    fn orientation_repair_fixes_a_reversed_face() {
        let mut f = sewn_block(&[3]);
        let before = f.store.check(f.body);
        assert!(
            before
                .iter()
                .any(|x| matches!(x, CheckFailure::InconsistentOrientation { .. })),
            "one reversed face disagrees with its four neighbours: {before:?}"
        );

        let result = GeometryHealer::heal(f.body, &mut f.store, &f.geo, &HealOptions::default());
        assert!(result.healed(), "remaining: {:?}", result.remaining);
        assert_eq!(
            result
                .operations
                .iter()
                .filter(|op| matches!(op, HealOperation::FaceReoriented { .. }))
                .count(),
            1,
            "only the offending face flips: {:?}",
            result.operations
        );
        assert!(
            (signed_volume(&f) - 1.0).abs() < 1e-9,
            "the repaired block still bounds +1 mm^3, not -1"
        );
    }

    #[test]
    fn orientation_repair_uprights_an_inside_out_shell() {
        // Every face but one reversed: two-colouring alone would happily
        // "fix" this by reversing the single odd face out, leaving a block
        // that encloses -1 mm^3. The volume-sign pass must catch it.
        let mut f = sewn_block(&[0, 1, 2, 3, 4]);
        let result = GeometryHealer::heal(f.body, &mut f.store, &f.geo, &HealOptions::default());
        assert!(result.healed(), "remaining: {:?}", result.remaining);
        assert!(
            result
                .operations
                .iter()
                .any(|op| matches!(op, HealOperation::ShellReversed { .. })),
            "the shell must be reported reversed: {:?}",
            result.operations
        );
        assert!(
            (signed_volume(&f) - 1.0).abs() < 1e-9,
            "volume must come out positive, got {}",
            signed_volume(&f)
        );
    }

    #[test]
    fn a_correctly_oriented_body_is_untouched() {
        let mut f = sewn_block(&[]);
        assert_eq!(f.store.check(f.body), vec![]);
        let result = GeometryHealer::heal(f.body, &mut f.store, &f.geo, &HealOptions::default());
        assert!(result.operations.is_empty(), "{:?}", result.operations);
        assert_eq!(f.store.check(f.body), vec![]);
        assert!((signed_volume(&f) - 1.0).abs() < 1e-9);
    }

    #[test]
    fn gaps_and_orientation_heal_together() {
        let mut f = unsewn_block(1e-6, &[2, 5]);
        let result = GeometryHealer::heal(f.body, &mut f.store, &f.geo, &HealOptions::default());
        assert!(result.healed(), "remaining: {:?}", result.remaining);
        assert_eq!(f.store.vertices.len(), 8);
        assert_eq!(f.store.edges.len(), 12);
        assert!((signed_volume(&f) - 1.0).abs() < 1e-6);
    }

    #[test]
    fn minimal_strategy_leaves_orientation_alone() {
        let mut f = sewn_block(&[3]);
        let result = GeometryHealer::heal(
            f.body,
            &mut f.store,
            &f.geo,
            &HealOptions {
                strategy: HealStrategy::Minimal,
                max_gap: None,
            },
        );
        assert!(!result.healed(), "Minimal must not touch orientation");
        assert!(
            result
                .operations
                .iter()
                .all(|op| !matches!(op, HealOperation::FaceReoriented { .. }))
        );
    }

    #[test]
    fn report_only_plans_without_mutating() {
        let mut f = unsewn_block(0.0, &[]);
        let vertices_before = f.store.vertices.len();
        let edges_before = f.store.edges.len();
        let result = GeometryHealer::heal(
            f.body,
            &mut f.store,
            &f.geo,
            &HealOptions {
                strategy: HealStrategy::ReportOnly,
                max_gap: None,
            },
        );
        assert_eq!(f.store.vertices.len(), vertices_before);
        assert_eq!(f.store.edges.len(), edges_before);
        assert_eq!(result.remaining, result.failures_before);
        assert_eq!(
            result
                .operations
                .iter()
                .filter(|op| matches!(op, HealOperation::VerticesMerged { .. }))
                .count(),
            8,
            "one merge per block corner: {:?}",
            result.operations
        );
        assert_eq!(
            result
                .operations
                .iter()
                .filter(|op| matches!(op, HealOperation::EdgesWelded { .. }))
                .count(),
            12,
            "one weld per block edge: {:?}",
            result.operations
        );
    }

    /// The orientation pass must be a dry run under `ReportOnly` too — it is
    /// the pass that mutates faces the caller can still see, so reporting a
    /// flip and then performing it would be the worst of both.
    #[test]
    fn report_only_plans_orientation_without_flipping_anything() {
        let mut f = sewn_block(&[3]);
        let snapshot = |f: &BlockFixture| -> Vec<(FaceSense, Vec<FinSense>)> {
            f.store
                .faces_of_body(f.body)
                .iter()
                .map(|&face| {
                    let sense = f.store.face(face).expect("live face").sense;
                    let winding = fins_of_face(&f.store, face)
                        .into_iter()
                        .map(|fin| f.store.fin(fin).expect("live fin").sense)
                        .collect();
                    (sense, winding)
                })
                .collect()
        };
        let before = snapshot(&f);

        let result = GeometryHealer::heal(
            f.body,
            &mut f.store,
            &f.geo,
            &HealOptions {
                strategy: HealStrategy::ReportOnly,
                max_gap: None,
            },
        );
        assert_eq!(
            result
                .operations
                .iter()
                .filter(|op| matches!(op, HealOperation::FaceReoriented { .. }))
                .count(),
            1,
            "the flip must still be reported: {:?}",
            result.operations
        );
        assert_eq!(
            before,
            snapshot(&f),
            "ReportOnly must leave every face's sense and winding as authored"
        );
        assert_eq!(result.remaining, result.failures_before);
    }

    #[test]
    fn off_strategy_does_nothing() {
        let mut f = unsewn_block(0.0, &[]);
        let result = GeometryHealer::heal(
            f.body,
            &mut f.store,
            &f.geo,
            &HealOptions {
                strategy: HealStrategy::Off,
                max_gap: None,
            },
        );
        assert!(result.operations.is_empty());
        assert_eq!(result.remaining, result.failures_before);
        assert_eq!(f.store.vertices.len(), 24);
    }

    #[test]
    fn merging_never_collapses_an_edge() {
        // Two corners of one face brought within the merge tolerance of each
        // other: merging them would delete the edge between them and break
        // the Euler counts, so the cluster must be refused with a note.
        let mut f = unsewn_block(0.0, &[]);
        let vertices = body_vertices(&f.store, f.body);
        let victim = vertices[1];
        let anchor = f.store.vertex(vertices[0]).expect("live vertex").point;
        f.store.vertices.get_mut(victim).expect("live vertex").point = anchor;

        let result = GeometryHealer::heal(f.body, &mut f.store, &f.geo, &HealOptions::default());
        assert!(
            result
                .notes
                .iter()
                .any(|n| n.contains("an edge joins two of them")),
            "notes: {:?}",
            result.notes
        );
    }

    #[test]
    fn max_gap_is_clamped_to_the_kernel_tolerance_limit() {
        let mut f = unsewn_block(0.0, &[]);
        let result = GeometryHealer::heal(
            f.body,
            &mut f.store,
            &f.geo,
            &HealOptions {
                strategy: HealStrategy::Auto,
                max_gap: Some(1.0),
            },
        );
        assert!(
            result.notes.iter().any(|n| n.contains("clamped max_gap")),
            "notes: {:?}",
            result.notes
        );
    }

    #[test]
    fn heal_operations_render_for_diagnostics() {
        let mut f = unsewn_block(1e-6, &[]);
        let result = GeometryHealer::heal(f.body, &mut f.store, &f.geo, &HealOptions::default());
        assert!(!result.operations.is_empty());
        for op in &result.operations {
            let text = op.to_string();
            assert!(!text.is_empty());
            assert!(text.is_ascii());
        }
    }

    #[test]
    fn healing_a_valid_body_reports_nothing_to_do() {
        let mut f = sewn_block(&[]);
        let result =
            GeometryHealer::fix_gaps(f.body, &mut f.store, &f.geo, &HealOptions::default());
        assert!(result.is_empty());
        assert!(!result.healed(), "there was nothing wrong to heal");
    }
}
