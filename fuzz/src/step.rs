//! Target 1: the STEP Part 21 front end — lex, parse, and full AP203 import.
//!
//! The input is raw file bytes, unmodified: this is the one target where the
//! fuzzer's mutations and a real attacker's input are the same thing, because
//! `read_step_bytes` is what a user points at an untrusted `.stp` file.
//!
//! # Post-conditions
//!
//! * **No panic, no hang, bounded memory** on any byte string. `parse_bytes`
//!   already caps aggregate nesting at 64 (the fix for of-1dd, a stack
//!   overflow at 500-deep nesting); this target is what keeps that class from
//!   coming back through some other recursion.
//! * **`parse` and `parse_bytes` agree.** For input that happens to be valid
//!   UTF-8 the two entry points must produce the same `StepFile` and the same
//!   error, since one delegates to the other. A divergence would mean the
//!   `&str` path had grown its own logic.
//! * **The entity graph is traversable.** Every instance is reachable through
//!   `StepFile::get` by its own id, ids are unique, and every value nests to a
//!   bounded depth.
//! * **An exact import validates.** `SolidOutcome::BRep(body)` is documented
//!   to mean "this body passed `TopologyStore::check`" — the reader only
//!   returns it when `check` came back empty, before or after healing. So
//!   re-running `check` on the returned body must still be empty. If it is
//!   not, either the healer's own report diverged from a fresh check or
//!   `finish_exact_body` corrupted the body on the way out.
//! * **What imports, exports.** Bodies that imported exactly must survive
//!   `write_step`, and what the writer emits must parse back. A writer that
//!   emits a file our own parser rejects is a round-trip bug regardless of
//!   how strange the input was.

use opensolid_brep::{Body, GeometryStore, TopologyStore};
use opensolid_core::arena::EntityId;
use opensolid_kernel::io::step::{
    self, SolidOutcome, StepFile, StepReadOptions, StepWriteOptions, Value,
};

/// Inputs larger than this are dropped before parsing.
///
/// Generous, because the committed seed corpus points libFuzzer at the real
/// AP203 test files (`crates/opensolid-kernel/tests/data/step`), the largest
/// of which is ~1 MiB. Parsing is linear, so a large seed costs milliseconds,
/// not seconds.
pub const MAX_PARSE_BYTES: usize = 2 * 1024 * 1024;

/// Inputs larger than this are parsed but not imported.
///
/// The AP203 mapper is far more expensive per byte than the parser — it
/// tessellates, heals and projects pcurves — and a fuzzer that spends seconds
/// per execution explores nothing. 32 KiB is enough to express a complete
/// multi-face solid, which is what the mapper's interesting paths need.
pub const MAX_IMPORT_BYTES: usize = 32 * 1024;

/// Nesting depth beyond which a parsed `Value` tree is considered a parser
/// bug. `parse` caps aggregates at 64; this is that limit plus headroom for
/// typed-parameter wrappers, and exists so a regression in the depth guard
/// surfaces as a harness assertion rather than as a stack overflow somewhere
/// downstream.
const MAX_OBSERVED_DEPTH: usize = 256;

/// Fuzz entry point: see the [module docs](self) for the contract.
pub fn fuzz_step_parse(data: &[u8]) {
    if data.len() > MAX_PARSE_BYTES {
        return;
    }

    let parsed = step::parse_bytes(data);

    // `parse` must be exactly `parse_bytes` on the UTF-8 subset.
    if let Ok(text) = std::str::from_utf8(data) {
        match (&parsed, step::parse(text)) {
            (Ok(a), Ok(b)) => {
                assert_eq!(*a, b, "parse and parse_bytes disagree on valid-UTF-8 input")
            }
            (Err(a), Err(b)) => assert_eq!(
                a.to_string(),
                b.to_string(),
                "parse and parse_bytes report different errors"
            ),
            (a, b) => panic!(
                "parse and parse_bytes disagree on success: bytes={:?} str={:?}",
                a.is_ok(),
                b.is_ok()
            ),
        }
    }

    let Ok(file) = parsed else {
        // A rejected file is a success: the contract is "report, do not crash".
        return;
    };

    walk_graph(&file);

    if data.len() <= MAX_IMPORT_BYTES {
        import(data);
    }
}

/// Traverse every instance and every parameter of a parsed file.
///
/// Catches index/lookup corruption that a parse-only target would miss: the
/// `id -> index` map is built separately from the instance vector, so a graph
/// can parse cleanly and still be unusable.
fn walk_graph(file: &StepFile) {
    // Comparing the two accessors against each other is the point, so the
    // explicit length comparison is deliberate.
    #[allow(clippy::len_zero)]
    {
        assert_eq!(file.is_empty(), file.len() == 0);
    }

    let mut seen_ids = std::collections::HashSet::new();
    for instance in &file.data {
        assert!(
            seen_ids.insert(instance.id),
            "duplicate instance name #{} survived parsing",
            instance.id
        );
        let looked_up = file
            .get(instance.id)
            .unwrap_or_else(|| panic!("instance #{} is not indexed", instance.id));
        assert_eq!(
            looked_up, instance,
            "StepFile::get(#{}) returned a different instance",
            instance.id
        );

        let records = match &instance.entity {
            step::EntityRecord::Simple(r) => std::slice::from_ref(r),
            step::EntityRecord::Complex(rs) => rs.as_slice(),
        };
        for record in records {
            // `part` must find every record that is actually present.
            assert!(
                instance.entity.part(&record.type_name).is_some(),
                "part({}) missed a record that is present",
                record.type_name
            );
            for value in &record.attributes {
                walk_value(file, value);
            }
        }
    }

    for record in &file.header.records {
        for value in &record.attributes {
            walk_value(file, value);
        }
    }
}

/// Visit one parameter tree iteratively (an explicit stack, so a depth-guard
/// regression cannot blow *this* thread's stack before the assertion fires).
fn walk_value(file: &StepFile, root: &Value) {
    let mut stack = vec![(root, 1usize)];
    while let Some((value, depth)) = stack.pop() {
        assert!(
            depth <= MAX_OBSERVED_DEPTH,
            "parsed value nests deeper than {MAX_OBSERVED_DEPTH}; the parser's depth guard is gone"
        );
        match value {
            Value::List(items) => stack.extend(items.iter().map(|v| (v, depth + 1))),
            Value::Typed { value, .. } => stack.push((value, depth + 1)),
            Value::Ref(id) => {
                // Dangling references are legal at this layer (the mapper
                // reports them); this only exercises the lookup path.
                let _ = file.get(*id);
                assert_eq!(value.as_ref_id(), Some(*id));
            }
            Value::Real(x) => {
                assert_eq!(value.as_real(), Some(*x));
            }
            Value::Integer(n) => {
                assert_eq!(value.as_integer(), Some(*n));
            }
            Value::Str(_) | Value::Enum(_) | Value::Binary(_) | Value::Unset | Value::Derived => {}
        }
    }
}

/// Run the full AP203 import, then the export round trip on whatever imported
/// exactly.
fn import(data: &[u8]) {
    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let options = StepReadOptions::default();

    let Ok(report) = step::read_step_bytes(data, &mut store, &mut geo, &options) else {
        // Only a parse error can land here, and `parse_bytes` already
        // succeeded above — but the reader owns its own error type, so treat
        // a mismatch as uninteresting rather than asserting on it.
        return;
    };

    assert!(
        report.length_scale.is_finite() && report.length_scale > 0.0,
        "import reported a non-positive length scale: {}",
        report.length_scale
    );
    assert!(
        report.angle_scale.is_finite() && report.angle_scale > 0.0,
        "import reported a non-positive angle scale: {}",
        report.angle_scale
    );

    let mut bodies: Vec<EntityId<Body>> = Vec::new();
    for solid in &report.solids {
        match &solid.outcome {
            SolidOutcome::BRep(body) => {
                let failures = store.check(*body);
                assert!(
                    failures.is_empty(),
                    "reader returned SolidOutcome::BRep for #{} but check() reports {failures:?}",
                    solid.step_id
                );
                bodies.push(*body);
            }
            SolidOutcome::Mesh { mesh, .. } => {
                // A fallback mesh is the reader's promise of a closed manifold
                // it could hand to F-Rep; an empty one is not that.
                assert!(
                    !mesh.indices.is_empty(),
                    "reader returned an empty fallback mesh for #{}",
                    solid.step_id
                );
            }
            SolidOutcome::Failed => {}
        }
    }

    for placed in &report.instances {
        assert!(
            placed.solid < report.solids.len(),
            "placed occurrence indexes solid {} of {}",
            placed.solid,
            report.solids.len()
        );
    }

    if bodies.is_empty() {
        return;
    }

    let written = step::write_step(&store, &geo, &bodies, &StepWriteOptions::default())
        .expect("write_step rejected bodies that imported exactly");
    step::parse(&written).expect("write_step emitted a file our own parser rejects");
}

#[cfg(test)]
mod tests {
    use super::*;

    const MINIMAL: &str = "\
ISO-10303-21;
HEADER;
FILE_DESCRIPTION((''), '2;1');
ENDSEC;
DATA;
#1 = CARTESIAN_POINT('', (0.0, 0.0, 0.0));
ENDSEC;
END-ISO-10303-21;
";

    #[test]
    fn accepts_a_minimal_file() {
        fuzz_step_parse(MINIMAL.as_bytes());
    }

    #[test]
    fn survives_truncation_at_every_offset() {
        // The classic parser killer: every prefix of a valid file.
        for end in 0..MINIMAL.len() {
            fuzz_step_parse(&MINIMAL.as_bytes()[..end]);
        }
    }

    #[test]
    fn survives_deep_nesting() {
        // of-1dd's shape: nesting past the parser's guard must be an error,
        // not a stack overflow.
        for depth in [1usize, 63, 64, 65, 500, 5000] {
            let src = format!(
                "ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n#1 = THING('', {}0.0{});\nENDSEC;\nEND-ISO-10303-21;\n",
                "(".repeat(depth),
                ")".repeat(depth)
            );
            fuzz_step_parse(src.as_bytes());
        }
    }

    #[test]
    fn survives_non_utf8_and_oversize() {
        fuzz_step_parse(&[0x80, 0xfe, 0xff, b';']);
        fuzz_step_parse(&vec![b'#'; MAX_PARSE_BYTES + 1]);
    }
}
