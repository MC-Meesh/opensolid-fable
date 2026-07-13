//! Byte-level tokenizer for STEP Part 21.
//!
//! The lexer scans the source bytes in a single forward pass and yields tokens
//! on demand (pull-based), so a multi-megabyte file is never fully
//! materialized as a token vector. Tokens borrow slices from the source where
//! possible; numeric literals are parsed eagerly. Whitespace and `/* … */`
//! comments are skipped between tokens.

use std::fmt;

/// A single lexical token. Slice-bearing variants borrow from the source.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum Token<'a> {
    LParen,
    RParen,
    Comma,
    Semicolon,
    Equals,
    /// `$` — unset/null.
    Dollar,
    /// `*` — derived value.
    Star,
    /// `#id` — an instance reference or, on the left of `=`, an instance name.
    Ref(u64),
    /// An integer literal (no decimal point).
    Integer(i64),
    /// A real literal (has a decimal point and/or exponent).
    Real(f64),
    /// Raw bytes between the quotes of a `'…'` string, with the closing quote
    /// excluded but `''` escapes *not* yet collapsed.
    Str(&'a [u8]),
    /// The bytes between the dots of an enumeration `.NAME.`, dots excluded.
    Enum(&'a [u8]),
    /// A keyword / type name (also section words like `HEADER`, and the magic
    /// `ISO-10303-21` / `END-ISO-10303-21` lines).
    Keyword(&'a [u8]),
    /// Raw bytes between the quotes of a `"…"` binary literal.
    Binary(&'a [u8]),
    /// End of input.
    Eof,
}

impl fmt::Display for Token<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Token::LParen => f.write_str("`(`"),
            Token::RParen => f.write_str("`)`"),
            Token::Comma => f.write_str("`,`"),
            Token::Semicolon => f.write_str("`;`"),
            Token::Equals => f.write_str("`=`"),
            Token::Dollar => f.write_str("`$`"),
            Token::Star => f.write_str("`*`"),
            Token::Ref(id) => write!(f, "`#{id}`"),
            Token::Integer(n) => write!(f, "integer `{n}`"),
            Token::Real(x) => write!(f, "real `{x}`"),
            Token::Str(_) => f.write_str("string"),
            Token::Enum(b) => write!(f, "enum `.{}.`", String::from_utf8_lossy(b)),
            Token::Keyword(b) => write!(f, "keyword `{}`", String::from_utf8_lossy(b)),
            Token::Binary(_) => f.write_str("binary literal"),
            Token::Eof => f.write_str("end of input"),
        }
    }
}

/// A lexical error, carrying the byte offset where it occurred.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct LexError {
    pub offset: usize,
    pub message: String,
}

/// Pull-based tokenizer over a byte slice.
pub(crate) struct Lexer<'a> {
    src: &'a [u8],
    pos: usize,
}

impl<'a> Lexer<'a> {
    pub(crate) fn new(src: &'a [u8]) -> Self {
        Lexer { src, pos: 0 }
    }

    /// Current byte offset (start of the next unread token).
    pub(crate) fn offset(&self) -> usize {
        self.pos
    }

    fn err(&self, offset: usize, message: impl Into<String>) -> LexError {
        LexError {
            offset,
            message: message.into(),
        }
    }

    /// Advance past whitespace and `/* … */` comments. Returns an error for an
    /// unterminated comment.
    fn skip_trivia(&mut self) -> Result<(), LexError> {
        loop {
            match self.src.get(self.pos) {
                Some(b) if is_ws(*b) => self.pos += 1,
                Some(b'/') if self.src.get(self.pos + 1) == Some(&b'*') => {
                    let start = self.pos;
                    self.pos += 2;
                    loop {
                        match self.src.get(self.pos) {
                            Some(b'*') if self.src.get(self.pos + 1) == Some(&b'/') => {
                                self.pos += 2;
                                break;
                            }
                            Some(_) => self.pos += 1,
                            None => return Err(self.err(start, "unterminated `/* */` comment")),
                        }
                    }
                }
                _ => return Ok(()),
            }
        }
    }

    /// Read and return the next token, or [`Token::Eof`] at end of input.
    pub(crate) fn next_token(&mut self) -> Result<Token<'a>, LexError> {
        self.skip_trivia()?;
        let start = self.pos;
        let Some(&b) = self.src.get(self.pos) else {
            return Ok(Token::Eof);
        };
        match b {
            b'(' => {
                self.pos += 1;
                Ok(Token::LParen)
            }
            b')' => {
                self.pos += 1;
                Ok(Token::RParen)
            }
            b',' => {
                self.pos += 1;
                Ok(Token::Comma)
            }
            b';' => {
                self.pos += 1;
                Ok(Token::Semicolon)
            }
            b'=' => {
                self.pos += 1;
                Ok(Token::Equals)
            }
            b'$' => {
                self.pos += 1;
                Ok(Token::Dollar)
            }
            b'*' => {
                self.pos += 1;
                Ok(Token::Star)
            }
            b'#' => self.lex_ref(start),
            b'\'' => self.lex_string(start),
            b'"' => self.lex_binary(start),
            b'.' => self.lex_enum(start),
            b'+' | b'-' | b'0'..=b'9' => self.lex_number(start),
            _ if is_keyword_start(b) => Ok(self.lex_keyword()),
            _ => Err(self.err(start, format!("unexpected character `{}`", char::from(b)))),
        }
    }

    fn lex_ref(&mut self, start: usize) -> Result<Token<'a>, LexError> {
        self.pos += 1; // consume '#'
        let digits_start = self.pos;
        while matches!(self.src.get(self.pos), Some(b'0'..=b'9')) {
            self.pos += 1;
        }
        if self.pos == digits_start {
            return Err(self.err(start, "`#` must be followed by a numeric instance name"));
        }
        let text = &self.src[digits_start..self.pos];
        let id = parse_u64(text).ok_or_else(|| self.err(start, "instance name out of range"))?;
        Ok(Token::Ref(id))
    }

    fn lex_string(&mut self, start: usize) -> Result<Token<'a>, LexError> {
        self.pos += 1; // consume opening quote
        let body_start = self.pos;
        loop {
            match self.src.get(self.pos) {
                Some(b'\'') => {
                    // A doubled quote is an escaped apostrophe, not the end.
                    if self.src.get(self.pos + 1) == Some(&b'\'') {
                        self.pos += 2;
                    } else {
                        let body = &self.src[body_start..self.pos];
                        self.pos += 1; // consume closing quote
                        return Ok(Token::Str(body));
                    }
                }
                Some(_) => self.pos += 1,
                None => return Err(self.err(start, "unterminated string literal")),
            }
        }
    }

    fn lex_binary(&mut self, start: usize) -> Result<Token<'a>, LexError> {
        self.pos += 1; // consume opening quote
        let body_start = self.pos;
        loop {
            match self.src.get(self.pos) {
                Some(b'"') => {
                    let body = &self.src[body_start..self.pos];
                    self.pos += 1;
                    return Ok(Token::Binary(body));
                }
                Some(_) => self.pos += 1,
                None => return Err(self.err(start, "unterminated binary literal")),
            }
        }
    }

    fn lex_enum(&mut self, start: usize) -> Result<Token<'a>, LexError> {
        self.pos += 1; // consume opening dot
        let body_start = self.pos;
        while matches!(self.src.get(self.pos), Some(b) if is_enum_char(*b)) {
            self.pos += 1;
        }
        if self.src.get(self.pos) != Some(&b'.') {
            return Err(self.err(start, "unterminated enumeration; expected closing `.`"));
        }
        let body = &self.src[body_start..self.pos];
        self.pos += 1; // consume closing dot
        Ok(Token::Enum(body))
    }

    fn lex_number(&mut self, start: usize) -> Result<Token<'a>, LexError> {
        // Optional sign.
        if matches!(self.src.get(self.pos), Some(b'+' | b'-')) {
            self.pos += 1;
        }
        let int_digits_start = self.pos;
        while matches!(self.src.get(self.pos), Some(b'0'..=b'9')) {
            self.pos += 1;
        }
        let mut is_real = false;
        // Fractional part.
        if self.src.get(self.pos) == Some(&b'.') {
            is_real = true;
            self.pos += 1;
            while matches!(self.src.get(self.pos), Some(b'0'..=b'9')) {
                self.pos += 1;
            }
        }
        // Exponent.
        if matches!(self.src.get(self.pos), Some(b'e' | b'E')) {
            is_real = true;
            self.pos += 1;
            if matches!(self.src.get(self.pos), Some(b'+' | b'-')) {
                self.pos += 1;
            }
            let exp_start = self.pos;
            while matches!(self.src.get(self.pos), Some(b'0'..=b'9')) {
                self.pos += 1;
            }
            if self.pos == exp_start {
                return Err(self.err(start, "malformed real: exponent has no digits"));
            }
        }
        if self.pos == int_digits_start && !is_real {
            return Err(self.err(start, "malformed number: no digits"));
        }
        let text = &self.src[start..self.pos];
        // The lexer only accepts ASCII digits/sign/dot/exponent above, so the
        // slice is guaranteed valid UTF-8.
        let s = str_from_ascii(text);
        if is_real {
            s.parse::<f64>()
                .map(Token::Real)
                .map_err(|_| self.err(start, "malformed real literal"))
        } else {
            s.parse::<i64>()
                .map(Token::Integer)
                .map_err(|_| self.err(start, "integer literal out of range"))
        }
    }

    fn lex_keyword(&mut self) -> Token<'a> {
        let start = self.pos;
        self.pos += 1; // first char already validated as a keyword start
        while matches!(self.src.get(self.pos), Some(b) if is_keyword_char(*b)) {
            self.pos += 1;
        }
        Token::Keyword(&self.src[start..self.pos])
    }
}

fn is_ws(b: u8) -> bool {
    matches!(b, b' ' | b'\t' | b'\r' | b'\n' | 0x0b | 0x0c)
}

/// A keyword may start with a letter, `_`, or `!` (user-defined keyword).
fn is_keyword_start(b: u8) -> bool {
    b.is_ascii_alphabetic() || b == b'_' || b == b'!'
}

/// Keyword continuation additionally allows digits and `-`, so the magic lines
/// `ISO-10303-21` and `END-ISO-10303-21` lex as single keywords. In valid Part
/// 21 a `-` only otherwise begins a number, which is disambiguated up front.
fn is_keyword_char(b: u8) -> bool {
    b.is_ascii_alphanumeric() || b == b'_' || b == b'-'
}

fn is_enum_char(b: u8) -> bool {
    b.is_ascii_alphanumeric() || b == b'_'
}

/// Parse ASCII digit bytes into a `u64` without an intermediate allocation.
fn parse_u64(digits: &[u8]) -> Option<u64> {
    let mut acc: u64 = 0;
    for &d in digits {
        acc = acc.checked_mul(10)?.checked_add(u64::from(d - b'0'))?;
    }
    Some(acc)
}

/// View an all-ASCII byte slice as `&str`. Only called on slices the lexer has
/// already confirmed contain ASCII digits/sign/dot/exponent.
fn str_from_ascii(bytes: &[u8]) -> &str {
    debug_assert!(bytes.is_ascii());
    // SAFETY: caller guarantees the bytes are ASCII, hence valid UTF-8.
    unsafe { std::str::from_utf8_unchecked(bytes) }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lex_all(src: &str) -> Vec<Token<'_>> {
        let mut lx = Lexer::new(src.as_bytes());
        let mut out = Vec::new();
        loop {
            let t = lx.next_token().unwrap();
            let done = t == Token::Eof;
            out.push(t);
            if done {
                break;
            }
        }
        out
    }

    #[test]
    fn punctuation_and_placeholders() {
        assert_eq!(
            lex_all("(),;=$*"),
            vec![
                Token::LParen,
                Token::RParen,
                Token::Comma,
                Token::Semicolon,
                Token::Equals,
                Token::Dollar,
                Token::Star,
                Token::Eof,
            ]
        );
    }

    #[test]
    fn integers_and_reals_are_distinguished() {
        assert_eq!(
            lex_all("4 -7 +3 1.0 -2.5 6.02E23 1.E-3"),
            vec![
                Token::Integer(4),
                Token::Integer(-7),
                Token::Integer(3),
                Token::Real(1.0),
                Token::Real(-2.5),
                Token::Real(6.02e23),
                Token::Real(1.0e-3),
                Token::Eof,
            ]
        );
    }

    #[test]
    fn refs_enums_keywords() {
        assert_eq!(
            lex_all("#123 .TRUE. CARTESIAN_POINT !USER_DEFINED"),
            vec![
                Token::Ref(123),
                Token::Enum(b"TRUE"),
                Token::Keyword(b"CARTESIAN_POINT"),
                Token::Keyword(b"!USER_DEFINED"),
                Token::Eof,
            ]
        );
    }

    #[test]
    fn magic_lines_lex_as_single_keywords() {
        assert_eq!(
            lex_all("ISO-10303-21 END-ISO-10303-21"),
            vec![
                Token::Keyword(b"ISO-10303-21"),
                Token::Keyword(b"END-ISO-10303-21"),
                Token::Eof,
            ]
        );
    }

    #[test]
    fn string_with_escaped_apostrophe() {
        // 'it''s' — the doubled quote is not a terminator.
        assert_eq!(lex_all("'it''s'"), vec![Token::Str(b"it''s"), Token::Eof]);
    }

    #[test]
    fn string_spanning_multiple_lines() {
        let toks = lex_all("'line one\nline two'");
        assert_eq!(toks, vec![Token::Str(b"line one\nline two"), Token::Eof]);
    }

    #[test]
    fn comments_are_skipped() {
        assert_eq!(
            lex_all("/* a comment */ #1 /* another */ = "),
            vec![Token::Ref(1), Token::Equals, Token::Eof]
        );
    }

    #[test]
    fn binary_literal() {
        assert_eq!(lex_all("\"01F\""), vec![Token::Binary(b"01F"), Token::Eof]);
    }

    #[test]
    fn unterminated_string_errors() {
        let mut lx = Lexer::new(b"'oops");
        assert!(lx.next_token().is_err());
    }

    #[test]
    fn unterminated_comment_errors() {
        let mut lx = Lexer::new(b"/* oops");
        assert!(lx.next_token().is_err());
    }

    #[test]
    fn bare_hash_errors() {
        let mut lx = Lexer::new(b"# ");
        assert!(lx.next_token().is_err());
    }
}
