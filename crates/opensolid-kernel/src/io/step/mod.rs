//! STEP Part 21 (ISO-10303-21) exchange: parsing, AP203 import, AP203 export.
//!
//! This module's own types parse the textual exchange structure of a STEP
//! file into a flat entity graph — a map from instance name (`#123`) to a
//! typed record. The parsing layer has **no geometry knowledge**: it does
//! not know what a `CARTESIAN_POINT` is, only that it is a keyword with a
//! list of parameters. Turning that graph into kernel geometry/topology
//! (AP203 semantics) is the [`read`] submodule — see [`read::read_step`] —
//! and the reverse direction, serializing kernel B-Rep bodies to an AP203
//! file, is the [`write`] submodule — see [`write::write_step`].
//!
//! # What it handles
//!
//! - The `ISO-10303-21; … END-ISO-10303-21;` envelope.
//! - `HEADER` and (one or more) `DATA` sections, each ended by `ENDSEC;`.
//! - Simple instances `#id = TYPE(args);` and complex (multiple-inheritance)
//!   instances `#id = (TYPE_A(args) TYPE_B(args) …);`.
//! - Every Part 21 parameter kind: integers, reals, strings, enumerations
//!   (`.T.`), instance references (`#id`), the unset (`$`) and derived (`*`)
//!   placeholders, typed parameters (`LENGTH_MEASURE(1.0)`), binary literals,
//!   and arbitrarily nested aggregates (lists).
//! - Comments (`/* … */`), free-form whitespace, and tokens (including
//!   strings) that span multiple physical lines.
//! - Forward references: an instance may refer to an `#id` that appears later
//!   in the file. References are stored as plain numbers; nothing is resolved
//!   here, so ordering is irrelevant.
//!
//! Strings are returned with the Part 21 apostrophe escape (`''`) collapsed to
//! a single `'`; other control directives (`\X\`, `\X2\…\X0\`, `\S\`) are left
//! verbatim for the mapper to decode, since their meaning is charset-dependent.
//!
//! # Example
//!
//! ```
//! use opensolid_kernel::io::step;
//!
//! let src = "\
//! ISO-10303-21;
//! HEADER;
//! FILE_DESCRIPTION((''), '2;1');
//! ENDSEC;
//! DATA;
//! #1 = CARTESIAN_POINT('', (0.0, 0.0, 0.0));
//! ENDSEC;
//! END-ISO-10303-21;
//! ";
//! let file = step::parse(src).unwrap();
//! let point = file.get(1).unwrap().as_simple().unwrap();
//! assert_eq!(point.type_name, "CARTESIAN_POINT");
//! assert_eq!(point.attributes.len(), 2);
//! ```

mod lex;
mod parse;
pub mod read;
pub mod write;

use std::collections::HashMap;

pub use parse::{StepError, parse, parse_bytes};
pub use read::{
    Diagnostic, ImportedSolid, Severity, SolidOutcome, StepImport, StepReadOptions, read_step,
    read_step_bytes,
};
pub use write::{LengthUnit, StepWriteError, StepWriteOptions, write_step};

/// A parsed STEP Part 21 file: its header records plus the data-section entity
/// graph, indexed by instance name for O(1) lookup.
#[derive(Debug, Clone, PartialEq)]
pub struct StepFile {
    /// Records from the `HEADER` section, in file order (typically
    /// `FILE_DESCRIPTION`, `FILE_NAME`, `FILE_SCHEMA`).
    pub header: Header,
    /// Data-section instances, in file order. Multiple `DATA` sections are
    /// concatenated.
    pub data: Vec<Instance>,
    /// Instance name (`#id`) → index into [`data`](Self::data).
    index: HashMap<u64, usize>,
}

impl StepFile {
    /// Look up a data-section instance by its numeric name (`#id`).
    ///
    /// Returns `None` for an unknown id (e.g. a dangling forward reference).
    pub fn get(&self, id: u64) -> Option<&Instance> {
        self.index.get(&id).map(|&i| &self.data[i])
    }

    /// Number of data-section instances.
    pub fn len(&self) -> usize {
        self.data.len()
    }

    /// Whether the data section is empty.
    pub fn is_empty(&self) -> bool {
        self.data.is_empty()
    }
}

/// The `HEADER` section: an ordered list of raw records.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Header {
    /// Header records in file order.
    pub records: Vec<SimpleRecord>,
}

impl Header {
    /// First header record with the given type name (case-sensitive; STEP
    /// keywords are upper-case), if any.
    pub fn get(&self, type_name: &str) -> Option<&SimpleRecord> {
        self.records.iter().find(|r| r.type_name == type_name)
    }
}

/// A data-section instance: a name (`#id`) bound to one entity record.
#[derive(Debug, Clone, PartialEq)]
pub struct Instance {
    /// Numeric instance name (the digits after `#`).
    pub id: u64,
    /// The bound entity — simple or complex.
    pub entity: EntityRecord,
}

/// The right-hand side of an instance: either one simple record or a complex
/// instance built from several partial records (multiple inheritance).
#[derive(Debug, Clone, PartialEq)]
pub enum EntityRecord {
    /// `TYPE(args)` — a single typed record.
    Simple(SimpleRecord),
    /// `(TYPE_A(args) TYPE_B(args) …)` — a complex instance combining several
    /// partial records, in file order.
    Complex(Vec<SimpleRecord>),
}

impl EntityRecord {
    /// The simple record, if this is a [`Simple`](EntityRecord::Simple).
    pub fn as_simple(&self) -> Option<&SimpleRecord> {
        match self {
            EntityRecord::Simple(r) => Some(r),
            EntityRecord::Complex(_) => None,
        }
    }

    /// The partial records, if this is a [`Complex`](EntityRecord::Complex).
    pub fn as_complex(&self) -> Option<&[SimpleRecord]> {
        match self {
            EntityRecord::Complex(rs) => Some(rs),
            EntityRecord::Simple(_) => None,
        }
    }

    /// The first partial record whose type name matches, searching a complex
    /// instance's parts (or the single simple record). Convenience for the
    /// mapper, which usually cares about one leaf type of a complex instance.
    pub fn part(&self, type_name: &str) -> Option<&SimpleRecord> {
        match self {
            EntityRecord::Simple(r) => (r.type_name == type_name).then_some(r),
            EntityRecord::Complex(rs) => rs.iter().find(|r| r.type_name == type_name),
        }
    }
}

impl Instance {
    /// The simple record bound to this instance, if it is not complex.
    pub fn as_simple(&self) -> Option<&SimpleRecord> {
        self.entity.as_simple()
    }
}

/// A `KEYWORD(arg, arg, …)` record: a type name and its ordered parameters.
#[derive(Debug, Clone, PartialEq)]
pub struct SimpleRecord {
    /// The entity type keyword, e.g. `"CARTESIAN_POINT"`.
    pub type_name: String,
    /// Parameters in declared order.
    pub attributes: Vec<Value>,
}

/// A single Part 21 parameter value (§7 of ISO 10303-21).
#[derive(Debug, Clone, PartialEq)]
pub enum Value {
    /// An integer literal (no decimal point), e.g. `4`.
    Integer(i64),
    /// A real literal (has a decimal point), e.g. `1.5`, `-3.0E-2`.
    Real(f64),
    /// A string literal, with `''` collapsed to `'`. Other Part 21 control
    /// directives are preserved verbatim.
    Str(String),
    /// An enumeration constant with the surrounding dots removed, e.g. `.T.`
    /// becomes `"T"`.
    Enum(String),
    /// An instance reference `#id`. Not resolved here — the target may appear
    /// later in the file (forward reference) or be absent.
    Ref(u64),
    /// A typed parameter `KEYWORD(value)`, e.g. `LENGTH_MEASURE(1.0)`. When the
    /// keyword wraps several parameters they are boxed as a [`Value::List`].
    Typed {
        /// The wrapping type keyword.
        type_name: String,
        /// The wrapped value (a [`Value::List`] if it had multiple entries).
        value: Box<Value>,
    },
    /// An aggregate `(a, b, …)` — a list, set, bag, or array. Part 21 does not
    /// distinguish these syntactically, so all become a `List`.
    List(Vec<Value>),
    /// The unset/null placeholder `$` (an optional attribute with no value).
    Unset,
    /// The derived-value placeholder `*` (an inherited attribute redeclared as
    /// derived in a subtype).
    Derived,
    /// A binary literal `"…"` with its leading unused-bit-count digit,
    /// preserved as the raw hex string (including the leading digit).
    Binary(String),
}

impl Value {
    /// The referenced instance id, if this is a [`Ref`](Value::Ref).
    pub fn as_ref_id(&self) -> Option<u64> {
        match self {
            Value::Ref(id) => Some(*id),
            _ => None,
        }
    }

    /// The real value, if this is a [`Real`](Value::Real). Integers are *not*
    /// coerced — STEP distinguishes the two and the mapper should be explicit.
    pub fn as_real(&self) -> Option<f64> {
        match self {
            Value::Real(x) => Some(*x),
            _ => None,
        }
    }

    /// The integer value, if this is an [`Integer`](Value::Integer).
    pub fn as_integer(&self) -> Option<i64> {
        match self {
            Value::Integer(n) => Some(*n),
            _ => None,
        }
    }

    /// The string contents, if this is a [`Str`](Value::Str).
    pub fn as_str(&self) -> Option<&str> {
        match self {
            Value::Str(s) => Some(s),
            _ => None,
        }
    }

    /// The enumeration name (dots stripped), if this is an [`Enum`](Value::Enum).
    pub fn as_enum(&self) -> Option<&str> {
        match self {
            Value::Enum(s) => Some(s),
            _ => None,
        }
    }

    /// The elements, if this is a [`List`](Value::List).
    pub fn as_list(&self) -> Option<&[Value]> {
        match self {
            Value::List(xs) => Some(xs),
            _ => None,
        }
    }
}
