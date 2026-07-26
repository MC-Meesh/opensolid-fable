//! Minimal JSON reader for the checked-in OCC reference data (of-ipt.16).
//!
//! The kernel keeps its dependency list to nalgebra/thiserror/rayon, and a
//! test-only JSON dependency is not worth an exception — the reference files
//! are machine-written by `scripts/occ_reference.py`, so this only has to
//! read the subset that generator emits: objects, arrays, strings without
//! escapes beyond the standard ones, numbers, booleans, null.
//!
//! It is strict on purpose: anything it cannot parse panics with a byte
//! offset rather than returning a default, because a silently-empty
//! reference would turn the oracle green for the wrong reason.

#![allow(dead_code)] // the accessor set is shared by several test targets

use std::collections::BTreeMap;

#[derive(Debug, Clone, PartialEq)]
pub enum Json {
    Null,
    Bool(bool),
    Number(f64),
    String(String),
    Array(Vec<Json>),
    Object(BTreeMap<String, Json>),
}

impl Json {
    /// Parse a complete JSON document. Panics on anything malformed.
    pub fn parse(text: &str) -> Json {
        let bytes = text.as_bytes();
        let mut parser = Parser { bytes, pos: 0 };
        parser.skip_whitespace();
        let value = parser.value();
        parser.skip_whitespace();
        assert!(
            parser.pos == bytes.len(),
            "trailing input at byte {}",
            parser.pos
        );
        value
    }

    /// Field of an object. Panics if this is not an object.
    pub fn get(&self, key: &str) -> Option<&Json> {
        match self {
            Json::Object(map) => map.get(key),
            other => panic!("expected an object to read {key:?} from, got {other:?}"),
        }
    }

    /// Required field of an object.
    pub fn field(&self, key: &str) -> &Json {
        self.get(key)
            .unwrap_or_else(|| panic!("missing required field {key:?} in {self:?}"))
    }

    pub fn as_f64(&self) -> f64 {
        match self {
            Json::Number(n) => *n,
            other => panic!("expected a number, got {other:?}"),
        }
    }

    pub fn as_usize(&self) -> usize {
        let n = self.as_f64();
        assert!(
            n >= 0.0 && n.fract() == 0.0 && n <= usize::MAX as f64,
            "expected a non-negative integer, got {n}"
        );
        n as usize
    }

    pub fn as_bool(&self) -> bool {
        match self {
            Json::Bool(b) => *b,
            other => panic!("expected a bool, got {other:?}"),
        }
    }

    pub fn as_str(&self) -> &str {
        match self {
            Json::String(s) => s,
            other => panic!("expected a string, got {other:?}"),
        }
    }

    pub fn as_array(&self) -> &[Json] {
        match self {
            Json::Array(items) => items,
            other => panic!("expected an array, got {other:?}"),
        }
    }
}

struct Parser<'a> {
    bytes: &'a [u8],
    pos: usize,
}

impl<'a> Parser<'a> {
    fn skip_whitespace(&mut self) {
        while matches!(self.bytes.get(self.pos), Some(b' ' | b'\t' | b'\n' | b'\r')) {
            self.pos += 1;
        }
    }

    fn peek(&self) -> u8 {
        *self
            .bytes
            .get(self.pos)
            .unwrap_or_else(|| panic!("unexpected end of JSON input at byte {}", self.pos))
    }

    fn expect(&mut self, byte: u8) {
        let found = self.peek();
        assert!(
            found == byte,
            "expected {:?} at byte {}, found {:?}",
            byte as char,
            self.pos,
            found as char
        );
        self.pos += 1;
    }

    fn literal(&mut self, word: &str) {
        assert!(
            self.bytes[self.pos..].starts_with(word.as_bytes()),
            "expected {word:?} at byte {}",
            self.pos
        );
        self.pos += word.len();
    }

    fn value(&mut self) -> Json {
        match self.peek() {
            b'{' => self.object(),
            b'[' => self.array(),
            b'"' => Json::String(self.string()),
            b't' => {
                self.literal("true");
                Json::Bool(true)
            }
            b'f' => {
                self.literal("false");
                Json::Bool(false)
            }
            b'n' => {
                self.literal("null");
                Json::Null
            }
            _ => self.number(),
        }
    }

    fn object(&mut self) -> Json {
        self.expect(b'{');
        let mut map = BTreeMap::new();
        self.skip_whitespace();
        if self.peek() == b'}' {
            self.pos += 1;
            return Json::Object(map);
        }
        loop {
            self.skip_whitespace();
            let key = self.string();
            self.skip_whitespace();
            self.expect(b':');
            self.skip_whitespace();
            let value = self.value();
            assert!(
                map.insert(key.clone(), value).is_none(),
                "duplicate key {key:?}"
            );
            self.skip_whitespace();
            match self.peek() {
                b',' => self.pos += 1,
                b'}' => {
                    self.pos += 1;
                    return Json::Object(map);
                }
                other => panic!(
                    "expected ',' or '}}' at byte {}, found {:?}",
                    self.pos, other as char
                ),
            }
        }
    }

    fn array(&mut self) -> Json {
        self.expect(b'[');
        let mut items = Vec::new();
        self.skip_whitespace();
        if self.peek() == b']' {
            self.pos += 1;
            return Json::Array(items);
        }
        loop {
            self.skip_whitespace();
            items.push(self.value());
            self.skip_whitespace();
            match self.peek() {
                b',' => self.pos += 1,
                b']' => {
                    self.pos += 1;
                    return Json::Array(items);
                }
                other => panic!(
                    "expected ',' or ']' at byte {}, found {:?}",
                    self.pos, other as char
                ),
            }
        }
    }

    fn string(&mut self) -> String {
        self.expect(b'"');
        let mut out = String::new();
        loop {
            let byte = self.peek();
            self.pos += 1;
            match byte {
                b'"' => return out,
                b'\\' => {
                    let escape = self.peek();
                    self.pos += 1;
                    out.push(match escape {
                        b'"' => '"',
                        b'\\' => '\\',
                        b'/' => '/',
                        b'b' => '\u{8}',
                        b'f' => '\u{c}',
                        b'n' => '\n',
                        b'r' => '\r',
                        b't' => '\t',
                        b'u' => {
                            let hex = std::str::from_utf8(&self.bytes[self.pos..self.pos + 4])
                                .expect("\\u escape must be ASCII hex");
                            self.pos += 4;
                            let code = u32::from_str_radix(hex, 16)
                                .unwrap_or_else(|e| panic!("bad \\u escape {hex:?}: {e}"));
                            char::from_u32(code)
                                .unwrap_or_else(|| panic!("bad code point U+{code:04X}"))
                        }
                        other => panic!("unknown escape \\{:?}", other as char),
                    });
                }
                // Multi-byte UTF-8 passes through untouched.
                _ => {
                    let start = self.pos - 1;
                    let len = utf8_len(byte);
                    self.pos = start + len;
                    out.push_str(
                        std::str::from_utf8(&self.bytes[start..self.pos])
                            .unwrap_or_else(|e| panic!("invalid UTF-8 in string: {e}")),
                    );
                }
            }
        }
    }

    fn number(&mut self) -> Json {
        let start = self.pos;
        if matches!(self.peek(), b'-' | b'+') {
            self.pos += 1;
        }
        while matches!(
            self.bytes.get(self.pos),
            Some(b'0'..=b'9' | b'.' | b'e' | b'E' | b'-' | b'+')
        ) {
            self.pos += 1;
        }
        let text = std::str::from_utf8(&self.bytes[start..self.pos]).expect("ASCII number");
        Json::Number(
            text.parse()
                .unwrap_or_else(|e| panic!("bad number {text:?} at byte {start}: {e}")),
        )
    }
}

fn utf8_len(lead: u8) -> usize {
    match lead {
        0x00..=0x7F => 1,
        0xC0..=0xDF => 2,
        0xE0..=0xEF => 3,
        _ => 4,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_the_shapes_the_generator_emits() {
        let value = Json::parse(
            r#"{
                "schema": "opensolid-occ-reference/1",
                "source": {"bytes": 23827, "fnv1a64": "b77c0a659cb12675"},
                "totals": {"solids": 1, "volume": 355877.882829, "area": 4.6607e4},
                "solids": [{"centroid": [-3.13e-16, 0, -15.17], "valid": true}],
                "nothing": null
            }"#,
        );
        assert_eq!(value.field("schema").as_str(), "opensolid-occ-reference/1");
        assert_eq!(value.field("source").field("bytes").as_usize(), 23827);
        assert_eq!(
            value.field("source").field("fnv1a64").as_str(),
            "b77c0a659cb12675"
        );
        assert_eq!(
            value.field("totals").field("volume").as_f64(),
            355877.882829
        );
        assert_eq!(value.field("totals").field("area").as_f64(), 46607.0);
        let solids = value.field("solids").as_array();
        assert_eq!(solids.len(), 1);
        assert!(solids[0].field("valid").as_bool());
        assert_eq!(solids[0].field("centroid").as_array()[2].as_f64(), -15.17);
        assert_eq!(value.field("nothing"), &Json::Null);
        assert!(value.get("absent").is_none());
    }

    #[test]
    fn handles_escapes_empty_containers_and_negative_exponents() {
        let value = Json::parse(r#"{"a": "x\"y\\z\né", "b": [], "c": {}, "d": -1.5e-9}"#);
        assert_eq!(value.field("a").as_str(), "x\"y\\z\né");
        assert!(value.field("b").as_array().is_empty());
        assert_eq!(value.field("c").get("anything"), None);
        assert_eq!(value.field("d").as_f64(), -1.5e-9);
    }

    #[test]
    #[should_panic(expected = "trailing input")]
    fn rejects_trailing_garbage() {
        Json::parse("{} oops");
    }

    #[test]
    #[should_panic(expected = "unexpected end of JSON input")]
    fn rejects_truncation() {
        Json::parse(r#"{"a": [1, 2"#);
    }
}
