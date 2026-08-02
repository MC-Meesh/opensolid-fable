//! Structured errors across the wasm boundary (of-2y4.9).
//!
//! wasm-bindgen rejects a Rust `Result::Err(String)` by throwing the raw
//! string, so the only channel to JS is that string's *content*. Every
//! boundary error is therefore serialized as one JSON object —
//! `{"code", "category", "message", "hint"?}` — mirroring the STEP reader's
//! diagnostic model, so an agent can branch on `code`/`category` instead of
//! pattern-matching prose. `message` stays the human-readable sentence the
//! flat-string era produced; `hint`, when present, is a concrete next step
//! beyond what the message states.
//!
//! Codes and categories are an agent-facing contract: add new ones freely,
//! never rename existing ones. The JS half of the contract lives in
//! `tools/mcp-server/src/tools.js` (`errInfo`), which detects a structured
//! error by parsing the thrown string as JSON.

use opensolid_core::error::CoreError;
use opensolid_kernel::assembly::MateError;
use opensolid_kernel::io::step::{StepError, StepWriteError};

use crate::json_escape;

/// One boundary error, ready to serialize. Build it from a kernel error via
/// `From`, or hand-roll one for argument checks done in the binding layer
/// itself, then convert with [`WireError::json`].
pub(crate) struct WireError {
    /// Stable machine-readable code, unique per failure kind.
    pub code: &'static str,
    /// Coarse failure class: `argument`, `geometry`, `tolerance`,
    /// `unsupported`, `io`.
    pub category: &'static str,
    /// Human-readable sentence — what the flat string used to be.
    pub message: String,
    /// Concrete next step, when one is known beyond the message.
    pub hint: Option<String>,
}

impl WireError {
    pub fn new(code: &'static str, category: &'static str, message: impl Into<String>) -> Self {
        WireError {
            code,
            category,
            message: message.into(),
            hint: None,
        }
    }

    /// An argument check failed in the binding layer itself (bad count,
    /// non-finite number, unknown enum string, ...).
    pub fn invalid_argument(message: impl Into<String>) -> Self {
        WireError::new("invalid_argument", "argument", message)
    }

    pub fn with_hint(mut self, hint: impl Into<String>) -> Self {
        self.hint = Some(hint.into());
        self
    }

    /// Prepend an operation context (`"STEP export failed"`) to the message,
    /// keeping code, category, and hint intact — the structured replacement
    /// for `format!("{prefix}: {e}")`.
    pub fn context(mut self, prefix: &str) -> Self {
        self.message = format!("{prefix}: {}", self.message);
        self
    }

    /// Serialize for the boundary. Always a single JSON object starting with
    /// `{"code":` — that prefix is what the JS side keys its detection on.
    pub fn json(&self) -> String {
        let mut out = format!(
            "{{\"code\":\"{}\",\"category\":\"{}\",\"message\":\"{}\"",
            self.code,
            self.category,
            json_escape(&self.message)
        );
        if let Some(hint) = &self.hint {
            out.push_str(",\"hint\":\"");
            out.push_str(&json_escape(hint));
            out.push('"');
        }
        out.push('}');
        out
    }
}

impl From<&CoreError> for WireError {
    fn from(e: &CoreError) -> Self {
        WireError {
            code: e.code(),
            category: e.category(),
            message: e.to_string(),
            hint: e.hint(),
        }
    }
}

impl From<&MateError> for WireError {
    fn from(e: &MateError) -> Self {
        let code = match e {
            MateError::DegenerateFeature { .. } => "degenerate_mate_feature",
            MateError::FeatureMismatch { .. } => "mate_feature_mismatch",
            MateError::MissingValue => "mate_missing_value",
            // `MateError` is non_exhaustive; new variants surface with a
            // generic code until given one here.
            _ => "mate_error",
        };
        // All mate errors are spec errors the caller can rewrite, so they
        // share the `argument` category with the binding-layer checks.
        WireError::new(code, "argument", e.to_string())
    }
}

impl From<&StepError> for WireError {
    fn from(e: &StepError) -> Self {
        WireError::new("step_parse_error", "io", e.to_string())
    }
}

impl From<&StepWriteError> for WireError {
    fn from(e: &StepWriteError) -> Self {
        let code = match e {
            StepWriteError::StaleBody => "stale_body",
            StepWriteError::Unsupported(_) => "step_unsupported",
            StepWriteError::Invalid(_) => "invalid_body",
        };
        WireError::new(code, "io", e.to_string())
    }
}

/// `.map_err(wire)` — the one-word spelling for boundary signatures, taking
/// any error `WireError` knows how to classify.
pub(crate) fn wire<E>(e: E) -> String
where
    for<'a> WireError: From<&'a E>,
{
    WireError::from(&e).json()
}

#[cfg(test)]
mod tests {
    use super::*;
    use opensolid_core::mesh::ManifoldDefects;

    #[test]
    fn core_error_serializes_code_category_message() {
        let err = CoreError::InvalidArgument {
            argument: "radius",
            reason: "must be positive and finite, got -1".into(),
        };
        assert_eq!(
            wire(err),
            "{\"code\":\"invalid_argument\",\"category\":\"argument\",\
             \"message\":\"invalid argument `radius`: must be positive and finite, got -1\"}"
        );
    }

    #[test]
    fn non_manifold_error_carries_defect_hint() {
        let err = CoreError::NonManifold {
            context: "sdf_to_brep",
            defects: ManifoldDefects {
                pinched_edges: 1,
                ..ManifoldDefects::default()
            },
        };
        let json = wire(err);
        assert!(
            json.starts_with("{\"code\":\"non_manifold_mesh\""),
            "{json}"
        );
        assert!(json.contains("\"category\":\"geometry\""), "{json}");
        assert!(json.contains("\"hint\":\""), "{json}");
        assert!(json.contains("nudge the feature size"), "{json}");
    }

    #[test]
    fn message_and_hint_are_json_escaped() {
        let json = WireError::invalid_argument("a \"quoted\"\nvalue")
            .with_hint("use \\ carefully")
            .json();
        assert_eq!(
            json,
            "{\"code\":\"invalid_argument\",\"category\":\"argument\",\
             \"message\":\"a \\\"quoted\\\"\\nvalue\",\"hint\":\"use \\\\ carefully\"}"
        );
    }

    #[test]
    fn context_prefixes_message_only() {
        let err = CoreError::NotImplemented { feature: "fillets" };
        let json = WireError::from(&err).context("STEP export failed").json();
        assert!(
            json.contains("\"message\":\"STEP export failed: not implemented: fillets\""),
            "{json}"
        );
        assert!(json.starts_with("{\"code\":\"not_implemented\""), "{json}");
    }

    #[test]
    fn mate_errors_map_to_argument_codes() {
        let err = MateError::MissingValue;
        assert_eq!(
            wire(err),
            "{\"code\":\"mate_missing_value\",\"category\":\"argument\",\
             \"message\":\"distance mate requires an offset value\"}"
        );
    }

    #[test]
    fn step_write_errors_map_to_io() {
        let err = StepWriteError::Unsupported("periodic surface".into());
        let json = wire(err);
        assert!(json.starts_with("{\"code\":\"step_unsupported\""), "{json}");
        assert!(json.contains("\"category\":\"io\""), "{json}");
    }
}
