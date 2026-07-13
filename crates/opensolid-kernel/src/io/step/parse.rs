//! Recursive-descent parser turning the [`Lexer`] token stream into a
//! [`StepFile`]. One token of lookahead; a single forward pass over the input.

use std::collections::HashMap;

use thiserror::Error;

use super::lex::{LexError, Lexer, Token};
use super::{EntityRecord, Header, Instance, SimpleRecord, StepFile, Value};

/// Maximum nesting depth of aggregates/typed parameters within a single
/// parameter value. Real STEP files nest 2-4 levels; the cap exists so a
/// crafted file cannot overflow the stack — both during the recursive parse
/// and when the resulting [`Value`] tree is (recursively) dropped.
const MAX_NESTING_DEPTH: usize = 64;

/// A STEP Part 21 parse error with source location.
#[derive(Debug, Clone, PartialEq, Error)]
#[error("STEP parse error at line {line}, column {column}: {message}")]
pub struct StepError {
    /// 1-based line number.
    pub line: usize,
    /// 1-based column number (in bytes).
    pub column: usize,
    /// Byte offset from the start of input.
    pub offset: usize,
    /// Human-readable description.
    pub message: String,
}

impl StepError {
    fn from_lex(src: &[u8], e: LexError) -> StepError {
        let (line, column) = line_col(src, e.offset);
        StepError {
            line,
            column,
            offset: e.offset,
            message: e.message,
        }
    }
}

/// Parse STEP Part 21 source text into a [`StepFile`].
///
/// Returns [`StepError`] on the first syntactic problem, with line/column
/// pointing at the offending token.
pub fn parse(input: &str) -> Result<StepFile, StepError> {
    parse_bytes(input.as_bytes())
}

/// Parse STEP Part 21 source from raw bytes (STEP files are ASCII/Latin-1;
/// working on bytes avoids a UTF-8 validation pass over large files).
pub fn parse_bytes(input: &[u8]) -> Result<StepFile, StepError> {
    Parser::new(input)?.parse_file()
}

struct Parser<'a> {
    src: &'a [u8],
    lexer: Lexer<'a>,
    /// One-token lookahead.
    peeked: Token<'a>,
    /// Byte offset of `peeked`.
    peeked_at: usize,
    /// Byte offset of the most recently `bump`ed (consumed) token — for errors
    /// that refer to the token just taken rather than the current lookahead.
    cur_at: usize,
    /// Current aggregate/typed-parameter nesting depth inside a value.
    depth: usize,
}

impl<'a> Parser<'a> {
    fn new(src: &'a [u8]) -> Result<Self, StepError> {
        let mut lexer = Lexer::new(src);
        let peeked_at = lexer.offset();
        let peeked = lexer
            .next_token()
            .map_err(|e| StepError::from_lex(src, e))?;
        Ok(Parser {
            src,
            lexer,
            peeked,
            peeked_at,
            cur_at: 0,
            depth: 0,
        })
    }

    /// The lookahead token.
    fn peek(&self) -> &Token<'a> {
        &self.peeked
    }

    /// Consume and return the lookahead, loading the next token behind it.
    fn bump(&mut self) -> Result<Token<'a>, StepError> {
        let next_at = self.lexer.offset();
        let next = self.lexer.next_token().map_err(|e| self.lex_err(e))?;
        self.cur_at = self.peeked_at;
        self.peeked_at = next_at;
        Ok(std::mem::replace(&mut self.peeked, next))
    }

    fn lex_err(&self, e: LexError) -> StepError {
        self.error_at(e.offset, e.message)
    }

    /// Build an error at the current lookahead token's position.
    fn error(&self, message: impl Into<String>) -> StepError {
        self.error_at(self.peeked_at, message)
    }

    fn error_at(&self, offset: usize, message: impl Into<String>) -> StepError {
        let (line, column) = line_col(self.src, offset);
        StepError {
            line,
            column,
            offset,
            message: message.into(),
        }
    }

    fn expect(&mut self, want: &Token<'_>) -> Result<(), StepError> {
        if self.peek() == want {
            self.bump()?;
            Ok(())
        } else {
            Err(self.error(format!("expected {want}, found {}", self.peek())))
        }
    }

    /// Consume a keyword with the exact given text (case-sensitive).
    fn expect_keyword(&mut self, want: &str) -> Result<(), StepError> {
        match self.peek() {
            Token::Keyword(k) if *k == want.as_bytes() => {
                self.bump()?;
                Ok(())
            }
            other => Err(self.error(format!("expected keyword `{want}`, found {other}"))),
        }
    }

    fn parse_file(&mut self) -> Result<StepFile, StepError> {
        self.expect_keyword("ISO-10303-21")?;
        self.expect(&Token::Semicolon)?;

        let header = self.parse_header()?;

        let mut data: Vec<Instance> = Vec::new();
        let mut index: HashMap<u64, usize> = HashMap::new();
        // One or more DATA sections (Part 21 permits several).
        while matches!(self.peek(), Token::Keyword(k) if *k == b"DATA") {
            self.parse_data_section(&mut data, &mut index)?;
        }

        self.expect_keyword("END-ISO-10303-21")?;
        self.expect(&Token::Semicolon)?;

        match self.peek() {
            Token::Eof => {}
            other => {
                return Err(self.error(format!("expected end of input, found {other}")));
            }
        }

        Ok(StepFile {
            header,
            data,
            index,
        })
    }

    fn parse_header(&mut self) -> Result<Header, StepError> {
        self.expect_keyword("HEADER")?;
        self.expect(&Token::Semicolon)?;
        let mut records = Vec::new();
        loop {
            match self.peek() {
                Token::Keyword(k) if *k == b"ENDSEC" => {
                    self.bump()?;
                    self.expect(&Token::Semicolon)?;
                    break;
                }
                Token::Keyword(_) => {
                    let record = self.parse_simple_record()?;
                    self.expect(&Token::Semicolon)?;
                    records.push(record);
                }
                other => {
                    return Err(self.error(format!(
                        "expected a header record or `ENDSEC`, found {other}"
                    )));
                }
            }
        }
        Ok(Header { records })
    }

    fn parse_data_section(
        &mut self,
        data: &mut Vec<Instance>,
        index: &mut HashMap<u64, usize>,
    ) -> Result<(), StepError> {
        self.expect_keyword("DATA")?;
        // Optional `DATA(qualifiers)` form: skip a balanced parameter group.
        if matches!(self.peek(), Token::LParen) {
            self.skip_balanced_group()?;
        }
        self.expect(&Token::Semicolon)?;

        loop {
            match self.peek() {
                Token::Keyword(k) if *k == b"ENDSEC" => {
                    self.bump()?;
                    self.expect(&Token::Semicolon)?;
                    break;
                }
                Token::Ref(_) => {
                    let instance = self.parse_instance()?;
                    let id = instance.id;
                    let idx = data.len();
                    if index.insert(id, idx).is_some() {
                        return Err(
                            self.error_at(self.cur_at, format!("duplicate instance name `#{id}`"))
                        );
                    }
                    data.push(instance);
                }
                other => {
                    return Err(self.error(format!(
                        "expected an instance `#id = …` or `ENDSEC`, found {other}"
                    )));
                }
            }
        }
        Ok(())
    }

    fn parse_instance(&mut self) -> Result<Instance, StepError> {
        let id = match self.bump()? {
            Token::Ref(id) => id,
            other => {
                return Err(self.error_at(self.cur_at, format!("expected `#id`, found {other}")));
            }
        };
        self.expect(&Token::Equals)?;
        let entity = self.parse_entity_record()?;
        self.expect(&Token::Semicolon)?;
        Ok(Instance { id, entity })
    }

    /// Parse the right-hand side: a simple record `TYPE(args)` or a complex
    /// instance `(TYPE_A(args) TYPE_B(args) …)`.
    fn parse_entity_record(&mut self) -> Result<EntityRecord, StepError> {
        match self.peek() {
            Token::Keyword(_) => Ok(EntityRecord::Simple(self.parse_simple_record()?)),
            Token::LParen => {
                self.bump()?; // consume '('
                let mut parts = Vec::new();
                loop {
                    match self.peek() {
                        Token::Keyword(_) => parts.push(self.parse_simple_record()?),
                        Token::RParen => {
                            self.bump()?;
                            break;
                        }
                        other => {
                            return Err(self.error(format!(
                                "expected a partial record or `)` in complex instance, found {other}"
                            )));
                        }
                    }
                }
                if parts.is_empty() {
                    return Err(self.error("complex instance has no partial records"));
                }
                Ok(EntityRecord::Complex(parts))
            }
            other => Err(self.error(format!(
                "expected an entity record (`TYPE(…)` or `(…)`), found {other}"
            ))),
        }
    }

    /// Parse `KEYWORD ( param, param, … )`.
    fn parse_simple_record(&mut self) -> Result<SimpleRecord, StepError> {
        let type_name = match self.bump()? {
            Token::Keyword(k) => keyword_string(k),
            other => {
                return Err(self.error_at(
                    self.cur_at,
                    format!("expected a type keyword, found {other}"),
                ));
            }
        };
        self.expect(&Token::LParen)?;
        let attributes = self.parse_parameter_list()?;
        Ok(SimpleRecord {
            type_name,
            attributes,
        })
    }

    /// Parse a comma-separated parameter list up to and including the closing
    /// `)`. The opening `(` must already be consumed. An empty list is allowed.
    fn parse_parameter_list(&mut self) -> Result<Vec<Value>, StepError> {
        let mut params = Vec::new();
        if matches!(self.peek(), Token::RParen) {
            self.bump()?;
            return Ok(params);
        }
        loop {
            params.push(self.parse_value()?);
            match self.bump()? {
                Token::Comma => continue,
                Token::RParen => break,
                other => {
                    return Err(self.error_at(
                        self.cur_at,
                        format!("expected `,` or `)` in parameter list, found {other}"),
                    ));
                }
            }
        }
        Ok(params)
    }

    fn parse_value(&mut self) -> Result<Value, StepError> {
        match self.peek() {
            Token::Integer(n) => {
                let n = *n;
                self.bump()?;
                Ok(Value::Integer(n))
            }
            Token::Real(x) => {
                let x = *x;
                self.bump()?;
                Ok(Value::Real(x))
            }
            Token::Ref(id) => {
                let id = *id;
                self.bump()?;
                Ok(Value::Ref(id))
            }
            Token::Dollar => {
                self.bump()?;
                Ok(Value::Unset)
            }
            Token::Star => {
                self.bump()?;
                Ok(Value::Derived)
            }
            Token::Str(raw) => {
                let s = decode_string(raw);
                self.bump()?;
                Ok(Value::Str(s))
            }
            Token::Enum(raw) => {
                let s = keyword_string(raw);
                self.bump()?;
                Ok(Value::Enum(s))
            }
            Token::Binary(raw) => {
                let s = keyword_string(raw);
                self.bump()?;
                Ok(Value::Binary(s))
            }
            Token::LParen => {
                self.bump()?;
                let items = self.parse_nested_parameter_list()?;
                Ok(Value::List(items))
            }
            Token::Keyword(k) => {
                // A typed parameter: `KEYWORD(inner…)`.
                let type_name = keyword_string(k);
                self.bump()?;
                self.expect(&Token::LParen)?;
                let inner = self.parse_nested_parameter_list()?;
                let value = match inner.len() {
                    1 => Box::new(inner.into_iter().next().unwrap()),
                    _ => Box::new(Value::List(inner)),
                };
                Ok(Value::Typed { type_name, value })
            }
            other => Err(self.error(format!("expected a parameter value, found {other}"))),
        }
    }

    /// Parse a parameter list one nesting level down, enforcing
    /// [`MAX_NESTING_DEPTH`]. The opening `(` must already be consumed.
    fn parse_nested_parameter_list(&mut self) -> Result<Vec<Value>, StepError> {
        self.depth += 1;
        if self.depth > MAX_NESTING_DEPTH {
            return Err(self.error_at(
                self.cur_at,
                format!("aggregate nesting too deep (limit {MAX_NESTING_DEPTH})"),
            ));
        }
        let items = self.parse_parameter_list()?;
        self.depth -= 1;
        Ok(items)
    }

    /// Skip a balanced `( … )` group, consuming the opening and closing parens.
    /// Used for the optional `DATA(…)` qualifier we do not interpret.
    fn skip_balanced_group(&mut self) -> Result<(), StepError> {
        self.expect(&Token::LParen)?;
        let mut depth = 1usize;
        while depth > 0 {
            match self.bump()? {
                Token::LParen => depth += 1,
                Token::RParen => depth -= 1,
                Token::Eof => return Err(self.error("unbalanced parentheses before end of input")),
                _ => {}
            }
        }
        Ok(())
    }
}

fn keyword_string(bytes: &[u8]) -> String {
    // Keyword/enum/binary bytes are ASCII by construction in the lexer.
    String::from_utf8_lossy(bytes).into_owned()
}

/// Decode a raw Part 21 string body: collapse the `''` apostrophe escape to a
/// single `'`. Other control directives are preserved verbatim. The bytes are
/// treated as Latin-1-ish text via lossy UTF-8 decoding, which is exact for
/// the ASCII that dominates real files.
fn decode_string(raw: &[u8]) -> String {
    if !raw.contains(&b'\'') {
        return String::from_utf8_lossy(raw).into_owned();
    }
    let mut out = Vec::with_capacity(raw.len());
    let mut i = 0;
    while i < raw.len() {
        if raw[i] == b'\'' && raw.get(i + 1) == Some(&b'\'') {
            out.push(b'\'');
            i += 2;
        } else {
            out.push(raw[i]);
            i += 1;
        }
    }
    String::from_utf8_lossy(&out).into_owned()
}

/// Compute 1-based (line, column) for a byte offset.
fn line_col(src: &[u8], offset: usize) -> (usize, usize) {
    let clamped = offset.min(src.len());
    let mut line = 1;
    let mut col = 1;
    for &b in &src[..clamped] {
        if b == b'\n' {
            line += 1;
            col = 1;
        } else {
            col += 1;
        }
    }
    (line, col)
}

#[cfg(test)]
mod tests {
    use super::super::Value;
    use super::*;

    /// Wrap DATA-section body text in a minimal but complete Part 21 envelope.
    fn wrap(data: &str) -> String {
        format!("ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n{data}\nENDSEC;\nEND-ISO-10303-21;\n")
    }

    /// Parse the single instance `#1` from a DATA body and return its record.
    fn one(data: &str) -> Instance {
        let file = parse(&wrap(data)).unwrap();
        assert_eq!(file.len(), 1);
        file.data.into_iter().next().unwrap()
    }

    fn simple(data: &str) -> SimpleRecord {
        one(data).entity.as_simple().unwrap().clone()
    }

    #[test]
    fn minimal_file_parses() {
        let src = "\
ISO-10303-21;
HEADER;
FILE_DESCRIPTION((''), '2;1');
FILE_NAME('part.step','2026-05-16',('Author'),('Org'),'','OpenSolid','');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1 = CARTESIAN_POINT('', (0.0, 0.0, 0.0));
ENDSEC;
END-ISO-10303-21;
";
        let file = parse(src).unwrap();
        assert_eq!(file.header.records.len(), 3);
        assert_eq!(
            file.header.get("FILE_SCHEMA").unwrap().type_name,
            "FILE_SCHEMA"
        );
        assert_eq!(file.len(), 1);
        let p = file.get(1).unwrap().as_simple().unwrap();
        assert_eq!(p.type_name, "CARTESIAN_POINT");
        assert_eq!(p.attributes.len(), 2);
        assert_eq!(p.attributes[0], Value::Str(String::new()));
        assert_eq!(
            p.attributes[1],
            Value::List(vec![Value::Real(0.0), Value::Real(0.0), Value::Real(0.0)])
        );
    }

    #[test]
    fn all_scalar_parameter_kinds() {
        let rec = simple("#1 = MIXED(4, -2.5, 'hi', .TRUE., #7, $, *, 6.02E23);");
        assert_eq!(
            rec.attributes,
            vec![
                Value::Integer(4),
                Value::Real(-2.5),
                Value::Str("hi".to_string()),
                Value::Enum("TRUE".to_string()),
                Value::Ref(7),
                Value::Unset,
                Value::Derived,
                Value::Real(6.02e23),
            ]
        );
    }

    #[test]
    fn typed_parameter_wraps_single_value() {
        let rec = simple("#1 = X(LENGTH_MEASURE(1.5));");
        assert_eq!(
            rec.attributes[0],
            Value::Typed {
                type_name: "LENGTH_MEASURE".to_string(),
                value: Box::new(Value::Real(1.5)),
            }
        );
    }

    #[test]
    fn typed_parameter_with_multiple_values_becomes_list() {
        let rec = simple("#1 = X(PAIR(1, 2));");
        assert_eq!(
            rec.attributes[0],
            Value::Typed {
                type_name: "PAIR".to_string(),
                value: Box::new(Value::List(vec![Value::Integer(1), Value::Integer(2)])),
            }
        );
    }

    #[test]
    fn empty_parameter_list() {
        let rec = simple("#1 = EMPTY();");
        assert!(rec.attributes.is_empty());
    }

    // ---- Torture tests (required by of-3qy.1) ----

    #[test]
    fn torture_nested_aggregates() {
        // Deeply nested lists mixing every scalar kind.
        let rec = simple("#1 = NEST(((1, 2), (3, (4, #5))), (), (.A., 'x'));");
        let Value::List(top) = &rec.attributes[0] else {
            panic!("expected list");
        };
        // ((1,2),(3,(4,#5)))
        assert_eq!(top.len(), 2);
        assert_eq!(
            top[0],
            Value::List(vec![Value::Integer(1), Value::Integer(2)])
        );
        assert_eq!(
            top[1],
            Value::List(vec![
                Value::Integer(3),
                Value::List(vec![Value::Integer(4), Value::Ref(5)]),
            ])
        );
        // The empty aggregate.
        assert_eq!(rec.attributes[1], Value::List(vec![]));
        // A list containing an enum and a string.
        assert_eq!(
            rec.attributes[2],
            Value::List(vec![
                Value::Enum("A".to_string()),
                Value::Str("x".to_string())
            ])
        );
    }

    #[test]
    fn torture_multi_line_strings() {
        // A string containing newlines and doubled apostrophes, spanning lines,
        // inside a record whose parameters are also split across lines.
        let data =
            "#1 = DESC(\n  'first line\nsecond line with an '' apostrophe\nthird line',\n  42\n);";
        let rec = simple(data);
        assert_eq!(
            rec.attributes[0],
            Value::Str("first line\nsecond line with an ' apostrophe\nthird line".to_string())
        );
        assert_eq!(rec.attributes[1], Value::Integer(42));
    }

    #[test]
    fn torture_forward_and_backward_refs_resolve_by_id() {
        // #1 references #2 (forward) and #2 references #1 (backward). The
        // parser stores ids without resolving; lookup works regardless of order.
        let file = parse(&wrap("#1 = A(#2);\n#2 = B(#1);")).unwrap();
        assert_eq!(
            file.get(1).unwrap().as_simple().unwrap().attributes[0],
            Value::Ref(2)
        );
        assert_eq!(
            file.get(2).unwrap().as_simple().unwrap().attributes[0],
            Value::Ref(1)
        );
        // A dangling reference target simply isn't in the graph.
        assert!(file.get(99).is_none());
    }

    #[test]
    fn torture_complex_instance() {
        // Multiple-inheritance instance: several partial records, no commas.
        let inst = one("#1 = (NAMED_UNIT(*) LENGTH_UNIT() SI_UNIT(.MILLI., .METRE.));");
        let parts = inst.entity.as_complex().unwrap();
        assert_eq!(parts.len(), 3);
        assert_eq!(parts[0].type_name, "NAMED_UNIT");
        assert_eq!(parts[0].attributes, vec![Value::Derived]);
        assert_eq!(parts[1].type_name, "LENGTH_UNIT");
        assert!(parts[1].attributes.is_empty());
        assert_eq!(parts[2].type_name, "SI_UNIT");
        assert_eq!(
            parts[2].attributes,
            vec![
                Value::Enum("MILLI".to_string()),
                Value::Enum("METRE".to_string())
            ]
        );
        // `part` accessor finds a leaf type by name.
        assert!(inst.entity.part("SI_UNIT").is_some());
        assert!(inst.entity.part("MISSING").is_none());
    }

    #[test]
    fn torture_comments_between_everything() {
        let data = "#1 /* id */ = /* eq */ POINT /* type */ ( /* open */ 1.0 /* a */ , 2.0 /* b */ ) /* close */ ;";
        let rec = simple(data);
        assert_eq!(rec.type_name, "POINT");
        assert_eq!(rec.attributes, vec![Value::Real(1.0), Value::Real(2.0)]);
    }

    #[test]
    fn binary_literal_preserved() {
        let rec = simple("#1 = FLAGS(\"03FF\");");
        assert_eq!(rec.attributes[0], Value::Binary("03FF".to_string()));
    }

    #[test]
    fn multiple_data_sections_concatenate() {
        let src = "\
ISO-10303-21;
HEADER;
ENDSEC;
DATA;
#1 = A();
ENDSEC;
DATA;
#2 = B();
ENDSEC;
END-ISO-10303-21;
";
        let file = parse(src).unwrap();
        assert_eq!(file.len(), 2);
        assert!(file.get(1).is_some());
        assert!(file.get(2).is_some());
    }

    /// Build `#1 = THING('', ((((…0.…))));` with `depth` nested parens.
    fn nested_aggregate_src(depth: usize) -> String {
        wrap(&format!(
            "#1 = THING('', {}0.{});",
            "(".repeat(depth),
            ")".repeat(depth)
        ))
    }

    #[test]
    fn nesting_at_limit_parses() {
        let file = parse(&nested_aggregate_src(MAX_NESTING_DEPTH)).unwrap();
        // Walk down to the innermost value to confirm the shape survived.
        let mut v = &file.get(1).unwrap().as_simple().unwrap().attributes[1];
        for _ in 0..MAX_NESTING_DEPTH {
            let Value::List(items) = v else {
                panic!("expected list");
            };
            assert_eq!(items.len(), 1);
            v = &items[0];
        }
        assert_eq!(*v, Value::Real(0.0));
    }

    #[test]
    fn nesting_one_past_limit_errors() {
        let err = parse(&nested_aggregate_src(MAX_NESTING_DEPTH + 1)).unwrap_err();
        assert!(err.message.contains("nesting too deep"), "{}", err.message);
    }

    /// Regression for of-1dd: ~500 nested parens in a 1KB file used to
    /// overflow the stack and abort the whole process. Must now be a clean
    /// parse error.
    #[test]
    fn deeply_nested_aggregates_error_instead_of_crashing() {
        let err = parse(&nested_aggregate_src(512)).unwrap_err();
        assert!(err.message.contains("nesting too deep"), "{}", err.message);
    }

    /// Typed parameters recurse through the same path and must honour the
    /// same limit.
    #[test]
    fn deeply_nested_typed_parameters_error_instead_of_crashing() {
        let src = wrap(&format!(
            "#1 = X({}0.{});",
            "T(".repeat(512),
            ")".repeat(512)
        ));
        let err = parse(&src).unwrap_err();
        assert!(err.message.contains("nesting too deep"), "{}", err.message);
    }

    // ---- Error reporting ----

    #[test]
    fn missing_magic_header_errors() {
        let err = parse("HEADER;\nENDSEC;\n").unwrap_err();
        assert_eq!(err.line, 1);
        assert!(err.message.contains("ISO-10303-21"));
    }

    #[test]
    fn duplicate_instance_name_errors() {
        let err = parse(&wrap("#1 = A();\n#1 = B();")).unwrap_err();
        assert!(err.message.contains("duplicate instance name"));
    }

    #[test]
    fn missing_semicolon_errors_with_location() {
        // The instance terminator is missing before ENDSEC.
        let err = parse(&wrap("#1 = A()")).unwrap_err();
        assert!(err.message.contains("expected `;`") || err.message.contains("`;`"));
    }

    #[test]
    fn unterminated_string_reports_position() {
        let err = parse(&wrap("#1 = A('oops);")).unwrap_err();
        assert!(err.message.contains("unterminated string"));
        assert!(err.line >= 1 && err.column >= 1);
    }

    #[test]
    fn trailing_garbage_after_footer_errors() {
        let mut src = wrap("#1 = A();");
        src.push_str("JUNK\n");
        let err = parse(&src).unwrap_err();
        assert!(err.message.contains("end of input"));
    }

    /// Streaming sanity: a synthetic ≥10 MB file must parse in well under two
    /// seconds and yield the expected instance count. Guards against accidental
    /// O(n²) behaviour or per-token pathologies on large real-world files.
    #[test]
    fn torture_big_file_under_two_seconds() {
        use std::fmt::Write as _;
        use std::time::Instant;

        // Build a body of representative instances until it exceeds 10 MB.
        let mut body = String::with_capacity(11 * 1024 * 1024);
        let mut id: u64 = 0;
        while body.len() < 10 * 1024 * 1024 {
            id += 1;
            // A points-and-placements mix touching most value kinds per line.
            writeln!(
                body,
                "#{id} = CARTESIAN_POINT('P{id}', ({a}.0, {b}.5, -{c}.25));",
                a = id % 1000,
                b = id % 97,
                c = id % 13,
            )
            .unwrap();
            id += 1;
            writeln!(
                body,
                "#{id} = AXIS2_PLACEMENT_3D('', #{r1}, #{r2}, $);",
                r1 = id - 1,
                r2 = id,
            )
            .unwrap();
        }
        let src = wrap(&body);
        assert!(
            src.len() > 10 * 1024 * 1024,
            "synthetic file should exceed 10 MB"
        );

        let start = Instant::now();
        let file = parse(&src).unwrap();
        let elapsed = start.elapsed();

        assert_eq!(file.len() as u64, id, "every instance parsed");
        assert!(
            elapsed.as_secs_f64() < 2.0,
            "parsing {} MB took {:?}, expected < 2s",
            src.len() / (1024 * 1024),
            elapsed
        );
    }
}
