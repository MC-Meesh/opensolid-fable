//! Target 2: `TopologyStore::check` on arbitrary and deliberately corrupted
//! topology graphs.
//!
//! `check` is the kernel's last line of defence — every importer, boolean and
//! Euler operator leans on it to decide whether a body is usable. Its
//! documented contract is unusually strong:
//!
//! > Safe to call on arbitrarily corrupted topology: stale references are
//! > reported as failures and the affected sub-checks are skipped rather than
//! > panicking.
//!
//! A validator that panics on the input it exists to reject is worse than no
//! validator, so that sentence is what this target attacks.
//!
//! # How the graphs are built
//!
//! The fuzzer's bytes decode into a [`Program`] of store operations. Two
//! phases, deliberately:
//!
//! 1. **Build.** `create_*` calls, always with *live* ids and always
//!    respecting the documented preconditions (`create_loop` panics on a
//!    stale id or a second outer loop — those panics are internal-invariant
//!    assertions, not input validation, so provoking them would be testing
//!    the wrong thing). This phase produces well-formed-ish graphs: arbitrary
//!    shape, but no dangling references.
//! 2. **Corrupt.** `Arena::remove` on entities the graph still points at.
//!    This is what manufactures stale references, orphaned fins, faces whose
//!    shell is gone, loops whose face is gone — precisely the states `check`
//!    promises to survive and report.
//!
//! # Post-conditions
//!
//! * **`check` returns** — no panic, no unbounded loop — on every graph the
//!   program can build, however corrupt.
//! * **`check` is pure.** Two calls on an untouched store return the same
//!   failures. A validator that mutates what it inspects would make repair
//!   loops non-deterministic.
//! * **A stale body id is reported, not indexed.** Removing a body and
//!   checking it must yield exactly `[StaleBody]`.
//! * **Every failure is renderable.** `Debug` and `Display` are how failures
//!   reach a user, and they index into ids that may no longer resolve.
//! * **Corruption is never silent.** A graph that validated clean and then
//!   had a referenced entity removed must not still validate clean.

use arbitrary::{Arbitrary, Unstructured};
use opensolid_brep::{
    Body, BodyType, Edge, Face, FaceSense, Fin, FinSense, Loop, LoopType, Shell, ShellOrientation,
    TopologyStore, Vertex,
};
use opensolid_core::arena::EntityId;
use opensolid_core::types::Point3;

/// Cap on operations executed from one input. Bounds both runtime and the
/// arena sizes `check` walks (its sub-checks are quadratic in places, which
/// is fine for real bodies and not fine for a fuzzer without a leash).
pub const MAX_OPS: usize = 256;

/// Fuzz entry point: see the [module docs](self) for the contract.
pub fn fuzz_topology_check(data: &[u8]) {
    let u = Unstructured::new(data);
    let Ok(program) = Program::arbitrary_take_rest(u) else {
        return;
    };
    run_program(&program);
}

/// A decoded store-building program.
#[derive(Debug, Arbitrary)]
pub struct Program {
    build: Vec<BuildOp>,
    corrupt: Vec<CorruptOp>,
}

/// One well-formed store mutation. Entity selectors are indices into the ids
/// created so far, taken modulo the live count, so an op is never a no-op for
/// lack of a valid target once the first body exists.
#[derive(Debug, Arbitrary)]
enum BuildOp {
    Body {
        solid: bool,
        sheet: bool,
    },
    Shell {
        body: u8,
        closed: bool,
        inward: bool,
    },
    Face {
        shell: u8,
        negative: bool,
    },
    Vertex {
        x: i16,
        y: i16,
        z: i16,
        tolerance: Tolerance,
    },
    Edge {
        start: u8,
        end: u8,
        tolerance: Tolerance,
    },
    Loop {
        face: u8,
        inner: bool,
        fins: Vec<(u8, bool)>,
    },
    VertexLoop {
        face: u8,
        vertex: u8,
        singular: bool,
        as_outer: bool,
    },
}

/// Removals that manufacture stale references.
///
/// Every variant shares the `Remove` prefix because every variant *is* a
/// removal; the entity kind is the distinguishing half of the name.
#[allow(clippy::enum_variant_names)]
#[derive(Debug, Arbitrary)]
enum CorruptOp {
    RemoveBody(u8),
    RemoveShell(u8),
    RemoveFace(u8),
    RemoveLoop(u8),
    RemoveFin(u8),
    RemoveEdge(u8),
    RemoveVertex(u8),
}

/// Vertex/edge tolerance, drawn from the values that actually change `check`'s
/// verdict: exact, tolerant-but-legal, over `MAX_ALLOWED_TOLERANCE`, and the
/// invalid ones (negative, zero, non-finite) it must reject rather than
/// propagate into comparisons.
#[derive(Debug, Arbitrary)]
enum Tolerance {
    Exact,
    Tolerant,
    TooLarge,
    Zero,
    Negative,
    Nan,
    Infinite,
}

impl Tolerance {
    fn value(&self) -> f64 {
        use opensolid_brep::SYSTEM_RESOLUTION;
        match self {
            Tolerance::Exact => SYSTEM_RESOLUTION,
            Tolerance::Tolerant => SYSTEM_RESOLUTION * 1000.0,
            Tolerance::TooLarge => opensolid_brep::MAX_ALLOWED_TOLERANCE * 10.0,
            Tolerance::Zero => 0.0,
            Tolerance::Negative => -1.0,
            Tolerance::Nan => f64::NAN,
            Tolerance::Infinite => f64::INFINITY,
        }
    }
}

/// Ids handed out so far, so ops can name earlier entities.
#[derive(Default)]
struct Ids {
    bodies: Vec<EntityId<Body>>,
    shells: Vec<EntityId<Shell>>,
    faces: Vec<EntityId<Face>>,
    loops: Vec<EntityId<Loop>>,
    fins: Vec<EntityId<Fin>>,
    edges: Vec<EntityId<Edge>>,
    vertices: Vec<EntityId<Vertex>>,
}

/// Pick `list[index % len]`, or `None` when nothing has been created yet.
fn pick<T: Copy>(list: &[T], index: u8) -> Option<T> {
    if list.is_empty() {
        None
    } else {
        Some(list[index as usize % list.len()])
    }
}

fn run_program(program: &Program) {
    let mut store = TopologyStore::new();
    let mut ids = Ids::default();

    for op in program.build.iter().take(MAX_OPS) {
        apply_build(&mut store, &mut ids, op);
    }

    // Baseline: whatever the build phase produced, before any removal.
    let clean: Vec<_> = ids
        .bodies
        .iter()
        .map(|&body| (body, check_twice(&store, body)))
        .collect();

    for op in program.corrupt.iter().take(MAX_OPS) {
        apply_corrupt(&mut store, &ids, op);
    }

    for (body, before) in &clean {
        let after = check_twice(&store, *body);
        // Removing an entity the graph still points at can only add failures.
        // A body that was already invalid and now validates clean would mean
        // corruption erased evidence of corruption.
        assert!(
            before.is_empty() || !after.is_empty(),
            "body {body:?} failed validation with {before:?} before corruption \
             and validates clean after it"
        );
    }

    // A body removed outright must be reported as stale, never indexed into.
    if let Some(&body) = ids.bodies.first() {
        store.bodies.remove(body);
        let failures = store.check(body);
        assert_eq!(
            failures.len(),
            1,
            "check on a removed body should report exactly one failure, got {failures:?}"
        );
        assert!(
            matches!(failures[0], opensolid_brep::CheckFailure::StaleBody(id) if id == body),
            "check on a removed body reported {:?}",
            failures[0]
        );
    }
}

/// Run `check` twice and confirm it is a pure function of the store.
fn check_twice(store: &TopologyStore, body: EntityId<Body>) -> Vec<opensolid_brep::CheckFailure> {
    let first = store.check(body);
    let second = store.check(body);
    // Compared by rendering, not by `PartialEq`: a `CheckFailure` can carry a
    // NaN tolerance (reporting one is its job), and NaN is not equal to
    // itself, so `first == second` is false for a perfectly deterministic
    // `check`. The fuzzer found this in the harness before it found anything
    // in the kernel.
    assert_eq!(
        format!("{first:?}"),
        format!("{second:?}"),
        "TopologyStore::check is not deterministic for {body:?}"
    );
    for failure in &first {
        // Both renderings walk ids that may be stale; exercise them.
        let _ = format!("{failure:?}");
        let _ = failure.to_string();
        let _ = failure.is_structural();
    }
    first
}

fn apply_build(store: &mut TopologyStore, ids: &mut Ids, op: &BuildOp) {
    match op {
        BuildOp::Body { solid, sheet } => {
            let body_type = match (solid, sheet) {
                (true, _) => BodyType::Solid,
                (false, true) => BodyType::Sheet,
                (false, false) => BodyType::General,
            };
            ids.bodies.push(store.create_body(body_type));
        }
        BuildOp::Shell {
            body,
            closed,
            inward,
        } => {
            let Some(body) = pick(&ids.bodies, *body) else {
                return;
            };
            // The build phase may run after ids were removed in a previous
            // program, and `create_shell` panics on a stale body.
            if store.body(body).is_none() {
                return;
            }
            let orientation = if *inward {
                ShellOrientation::Inward
            } else {
                ShellOrientation::Outward
            };
            ids.shells
                .push(store.create_shell(body, *closed, orientation));
        }
        BuildOp::Face { shell, negative } => {
            let Some(shell) = pick(&ids.shells, *shell) else {
                return;
            };
            if store.shell(shell).is_none() {
                return;
            }
            let sense = if *negative {
                FaceSense::Negative
            } else {
                FaceSense::Positive
            };
            ids.faces.push(store.create_face(shell, sense));
        }
        BuildOp::Vertex { x, y, z, tolerance } => {
            // Small integer coordinates on purpose: coincident and nearly
            // coincident vertices are what the tolerance checks care about,
            // and a wide float range would make collisions vanishingly rare.
            let point = Point3::new(*x as f64 * 0.5, *y as f64 * 0.5, *z as f64 * 0.5);
            ids.vertices
                .push(store.create_vertex(point, tolerance.value()));
        }
        BuildOp::Edge {
            start,
            end,
            tolerance,
        } => {
            let (Some(start), Some(end)) = (pick(&ids.vertices, *start), pick(&ids.vertices, *end))
            else {
                return;
            };
            if store.vertex(start).is_none() || store.vertex(end).is_none() {
                return;
            }
            ids.edges
                .push(store.create_edge(start, end, tolerance.value()));
        }
        BuildOp::Loop { face, inner, fins } => {
            let Some(face) = pick(&ids.faces, *face) else {
                return;
            };
            let Some(face_ref) = store.face(face) else {
                return;
            };
            // Documented panic: a face may hold only one outer loop.
            let loop_type = if *inner || face_ref.outer_loop.is_some() {
                LoopType::Inner
            } else {
                LoopType::Outer
            };
            let directed: Vec<(EntityId<Edge>, FinSense)> = fins
                .iter()
                .filter_map(|(index, reversed)| {
                    let edge = pick(&ids.edges, *index)?;
                    // Documented panic: `create_loop` asserts live edge ids.
                    store.edge(edge)?;
                    Some((
                        edge,
                        if *reversed {
                            FinSense::Reversed
                        } else {
                            FinSense::Forward
                        },
                    ))
                })
                .collect();
            let loop_id = store.create_loop(face, loop_type, &directed);
            ids.loops.push(loop_id);
            ids.fins.extend_from_slice(store.fins_of_loop(loop_id));
        }
        BuildOp::VertexLoop {
            face,
            vertex,
            singular,
            as_outer,
        } => {
            let (Some(face), Some(vertex)) =
                (pick(&ids.faces, *face), pick(&ids.vertices, *vertex))
            else {
                return;
            };
            let Some(face_ref) = store.face(face) else {
                return;
            };
            if store.vertex(vertex).is_none() {
                return;
            }
            let as_outer = *as_outer && face_ref.outer_loop.is_none();
            let loop_type = if *singular {
                LoopType::Singular
            } else {
                LoopType::Vertex
            };
            ids.loops
                .push(store.create_vertex_loop(face, loop_type, vertex, as_outer));
        }
    }
}

fn apply_corrupt(store: &mut TopologyStore, ids: &Ids, op: &CorruptOp) {
    // Removals go straight through the arena: the point is to leave the
    // *containing* entity still pointing at the hole.
    match op {
        CorruptOp::RemoveBody(i) => {
            let _ = pick(&ids.bodies, *i).and_then(|id| store.bodies.remove(id));
        }
        CorruptOp::RemoveShell(i) => {
            let _ = pick(&ids.shells, *i).and_then(|id| store.shells.remove(id));
        }
        CorruptOp::RemoveFace(i) => {
            let _ = pick(&ids.faces, *i).and_then(|id| store.faces.remove(id));
        }
        CorruptOp::RemoveLoop(i) => {
            let _ = pick(&ids.loops, *i).and_then(|id| store.loops.remove(id));
        }
        CorruptOp::RemoveFin(i) => {
            let _ = pick(&ids.fins, *i).and_then(|id| store.fins.remove(id));
        }
        CorruptOp::RemoveEdge(i) => {
            let _ = pick(&ids.edges, *i).and_then(|id| store.edges.remove(id));
        }
        CorruptOp::RemoveVertex(i) => {
            let _ = pick(&ids.vertices, *i).and_then(|id| store.vertices.remove(id));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use opensolid_brep::CheckFailure;

    fn program(build: Vec<BuildOp>, corrupt: Vec<CorruptOp>) -> Program {
        Program { build, corrupt }
    }

    #[test]
    fn empty_program_is_a_no_op() {
        run_program(&program(vec![], vec![]));
    }

    /// A hand-built solid with one face, one loop and a stale edge behind it:
    /// the canonical "reports, does not panic" case.
    #[test]
    fn stale_edge_behind_a_fin_is_reported() {
        let mut store = TopologyStore::new();
        let body = store.create_body(BodyType::Solid);
        let shell = store.create_shell(body, true, ShellOrientation::Outward);
        let face = store.create_face(shell, FaceSense::Positive);
        let v0 = store.create_vertex(Point3::new(0.0, 0.0, 0.0), 1e-9);
        let v1 = store.create_vertex(Point3::new(1.0, 0.0, 0.0), 1e-9);
        let edge = store.create_edge(v0, v1, 1e-9);
        store.create_loop(face, LoopType::Outer, &[(edge, FinSense::Forward)]);

        store.edges.remove(edge);
        let failures = store.check(body);
        assert!(
            failures
                .iter()
                .any(|f| matches!(f, CheckFailure::StaleReference { .. })),
            "expected a stale-reference failure, got {failures:?}"
        );
    }

    #[test]
    fn removed_body_reports_stale_body() {
        run_program(&program(
            vec![BuildOp::Body {
                solid: true,
                sheet: false,
            }],
            vec![],
        ));
    }

    /// Every removal kind, applied to a graph that uses every entity kind.
    #[test]
    fn every_removal_kind_survives_check() {
        let build = vec![
            BuildOp::Body {
                solid: true,
                sheet: false,
            },
            BuildOp::Shell {
                body: 0,
                closed: true,
                inward: false,
            },
            BuildOp::Face {
                shell: 0,
                negative: false,
            },
            BuildOp::Vertex {
                x: 0,
                y: 0,
                z: 0,
                tolerance: Tolerance::Exact,
            },
            BuildOp::Vertex {
                x: 2,
                y: 0,
                z: 0,
                tolerance: Tolerance::Tolerant,
            },
            BuildOp::Edge {
                start: 0,
                end: 1,
                tolerance: Tolerance::Exact,
            },
            BuildOp::Loop {
                face: 0,
                inner: false,
                fins: vec![(0, false), (0, true)],
            },
            BuildOp::VertexLoop {
                face: 0,
                vertex: 0,
                singular: true,
                as_outer: false,
            },
        ];
        for corrupt in [
            CorruptOp::RemoveShell(0),
            CorruptOp::RemoveFace(0),
            CorruptOp::RemoveLoop(0),
            CorruptOp::RemoveFin(0),
            CorruptOp::RemoveEdge(0),
            CorruptOp::RemoveVertex(0),
        ] {
            run_program(&program(clone_build(&build), vec![corrupt]));
        }
    }

    /// Non-finite and out-of-range tolerances must be reported, not
    /// propagated into comparisons that silently succeed. The vertex has to
    /// be reachable from the body (`check` only walks what the body owns), so
    /// each case builds the full body → shell → face → loop → edge → vertex
    /// chain.
    #[test]
    fn invalid_tolerances_are_reported() {
        for tolerance in [
            Tolerance::Zero,
            Tolerance::Negative,
            Tolerance::Nan,
            Tolerance::Infinite,
            Tolerance::TooLarge,
        ] {
            let mut store = TopologyStore::new();
            let body = store.create_body(BodyType::Sheet);
            let shell = store.create_shell(body, false, ShellOrientation::Outward);
            let face = store.create_face(shell, FaceSense::Positive);
            let v0 = store.create_vertex(Point3::new(0.0, 0.0, 0.0), tolerance.value());
            let v1 = store.create_vertex(Point3::new(1.0, 0.0, 0.0), 1e-9);
            let edge = store.create_edge(v0, v1, 1e-9);
            store.create_loop(
                face,
                LoopType::Outer,
                &[(edge, FinSense::Forward), (edge, FinSense::Reversed)],
            );

            let failures = store.check(body);
            let expected_exceeded = matches!(tolerance, Tolerance::TooLarge);
            let found = failures.iter().any(|f| match f {
                CheckFailure::InvalidTolerance { .. } => !expected_exceeded,
                CheckFailure::ToleranceExceeded { .. } => expected_exceeded,
                _ => false,
            });
            assert!(
                found,
                "tolerance {tolerance:?} was not reported; check returned {failures:?}"
            );
        }
    }

    /// `BuildOp` is not `Clone` (it is a fuzz input type, not a domain type),
    /// so the multi-case test rebuilds it by hand.
    fn clone_build(ops: &[BuildOp]) -> Vec<BuildOp> {
        ops.iter()
            .map(|op| match op {
                BuildOp::Body { solid, sheet } => BuildOp::Body {
                    solid: *solid,
                    sheet: *sheet,
                },
                BuildOp::Shell {
                    body,
                    closed,
                    inward,
                } => BuildOp::Shell {
                    body: *body,
                    closed: *closed,
                    inward: *inward,
                },
                BuildOp::Face { shell, negative } => BuildOp::Face {
                    shell: *shell,
                    negative: *negative,
                },
                BuildOp::Vertex { x, y, z, tolerance } => BuildOp::Vertex {
                    x: *x,
                    y: *y,
                    z: *z,
                    tolerance: clone_tolerance(tolerance),
                },
                BuildOp::Edge {
                    start,
                    end,
                    tolerance,
                } => BuildOp::Edge {
                    start: *start,
                    end: *end,
                    tolerance: clone_tolerance(tolerance),
                },
                BuildOp::Loop { face, inner, fins } => BuildOp::Loop {
                    face: *face,
                    inner: *inner,
                    fins: fins.clone(),
                },
                BuildOp::VertexLoop {
                    face,
                    vertex,
                    singular,
                    as_outer,
                } => BuildOp::VertexLoop {
                    face: *face,
                    vertex: *vertex,
                    singular: *singular,
                    as_outer: *as_outer,
                },
            })
            .collect()
    }

    fn clone_tolerance(t: &Tolerance) -> Tolerance {
        match t {
            Tolerance::Exact => Tolerance::Exact,
            Tolerance::Tolerant => Tolerance::Tolerant,
            Tolerance::TooLarge => Tolerance::TooLarge,
            Tolerance::Zero => Tolerance::Zero,
            Tolerance::Negative => Tolerance::Negative,
            Tolerance::Nan => Tolerance::Nan,
            Tolerance::Infinite => Tolerance::Infinite,
        }
    }

    /// Byte-driven decoding must never panic, whatever the bytes are.
    #[test]
    fn arbitrary_decoding_is_total() {
        let mut seed = 0x2545_f491_4f6c_dd1du64;
        for _ in 0..2000 {
            let mut bytes = [0u8; 96];
            for byte in bytes.iter_mut() {
                seed = seed
                    .wrapping_mul(6364136223846793005)
                    .wrapping_add(1442695040888963407);
                *byte = (seed >> 33) as u8;
            }
            fuzz_topology_check(&bytes);
        }
    }
}
