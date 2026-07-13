// Shape operation graph: the structured model behind bidirectional
// script <-> GUI sync. Pure JS (no React, no WASM) so it is fully
// unit-testable.
//
// The script is parsed into a list of statements. Statements written in the
// canonical Shape-API subset —
//
//   const name = Shape.ctor(nums...).method(args...)...;
//   return expr;
//
// — become structured `def`/`ret` nodes with a full expression AST and exact
// character ranges in the source. Anything else (comments live between
// statements; arbitrary JS like `const r = a * 2;`) becomes an opaque `raw`
// segment that is never rewritten. GUI mutations regenerate only the single
// statement they own and splice it back by range, so hand-written code
// survives roundtrips byte-for-byte.
//
// Expression AST:
//   { kind: 'ctor', name, args }            Shape.sphere(0.5)
//   { kind: 'ref',  name }                  body
//   { kind: 'call', target, method, args }  <target>.translate(0, 1, 0)
// Argument:
//   { kind: 'num', value }   |   { kind: 'expr', expr }

// ---------------------------------------------------------------------------
// Tokenizer

const PUNCT = new Set(['.', ',', '(', ')', ';', '=', '-']);
const IDENT_START = /[A-Za-z_$]/;
const IDENT_PART = /[A-Za-z0-9_$]/;
const DIGIT = /[0-9]/;

function tokenize(source) {
  const tokens = [];
  let i = 0;
  const n = source.length;
  while (i < n) {
    const c = source[i];
    if (c === ' ' || c === '\t' || c === '\n' || c === '\r') {
      i += 1;
    } else if (c === '/' && source[i + 1] === '/') {
      const nl = source.indexOf('\n', i);
      i = nl === -1 ? n : nl + 1;
    } else if (c === '/' && source[i + 1] === '*') {
      const close = source.indexOf('*/', i + 2);
      i = close === -1 ? n : close + 2;
    } else if (IDENT_START.test(c)) {
      const start = i;
      while (i < n && IDENT_PART.test(source[i])) i += 1;
      tokens.push({ kind: 'ident', text: source.slice(start, i), start, end: i });
    } else if (DIGIT.test(c) || (c === '.' && DIGIT.test(source[i + 1] ?? ''))) {
      const start = i;
      while (i < n && /[0-9.eE]/.test(source[i])) {
        // Allow an exponent sign directly after e/E.
        if ((source[i] === 'e' || source[i] === 'E') && /[+-]/.test(source[i + 1] ?? '')) i += 1;
        i += 1;
      }
      tokens.push({ kind: 'num', text: source.slice(start, i), start, end: i });
    } else if (PUNCT.has(c)) {
      tokens.push({ kind: c, text: c, start: i, end: i + 1 });
      i += 1;
    } else {
      tokens.push({ kind: 'unknown', text: c, start: i, end: i + 1 });
      i += 1;
    }
  }
  tokens.push({ kind: 'eof', text: '', start: n, end: n });
  return tokens;
}

// ---------------------------------------------------------------------------
// Parser

class ParseFail extends Error {}

class Parser {
  constructor(tokens) {
    this.tokens = tokens;
    this.pos = 0;
  }

  peek(offset = 0) {
    return this.tokens[Math.min(this.pos + offset, this.tokens.length - 1)];
  }

  next() {
    const t = this.peek();
    if (t.kind !== 'eof') this.pos += 1;
    return t;
  }

  expect(kind) {
    const t = this.peek();
    if (t.kind !== kind) throw new ParseFail(`expected ${kind}, got ${t.kind}`);
    return this.next();
  }

  parseExpr() {
    let expr;
    const t = this.peek();
    if (t.kind !== 'ident') throw new ParseFail('expected identifier');
    if (t.text === 'Shape' && this.peek(1).kind === '.') {
      this.next();
      this.next();
      const name = this.expect('ident').text;
      const args = this.parseArgs();
      expr = { kind: 'ctor', name, args };
    } else {
      this.next();
      expr = { kind: 'ref', name: t.text };
    }
    while (this.peek().kind === '.') {
      this.next();
      const method = this.expect('ident').text;
      const args = this.parseArgs();
      expr = { kind: 'call', target: expr, method, args };
    }
    return expr;
  }

  parseArgs() {
    this.expect('(');
    const args = [];
    if (this.peek().kind !== ')') {
      for (;;) {
        args.push(this.parseArg());
        if (this.peek().kind !== ',') break;
        this.next();
      }
    }
    this.expect(')');
    return args;
  }

  parseArg() {
    const t = this.peek();
    if (t.kind === '-' || t.kind === 'num') {
      let negative = false;
      if (t.kind === '-') {
        negative = true;
        this.next();
      }
      const numTok = this.expect('num');
      const value = Number(numTok.text);
      if (!Number.isFinite(value)) throw new ParseFail(`bad number ${numTok.text}`);
      const arg = { kind: 'num', value: negative ? -value : value };
      // A number followed by more expression syntax (e.g. `2 * r`) is not in
      // the canonical subset.
      if (this.peek().kind === 'unknown' || this.peek().kind === 'num') {
        throw new ParseFail('arithmetic in arguments is not canonical');
      }
      return arg;
    }
    if (t.kind === 'ident') {
      return { kind: 'expr', expr: this.parseExpr() };
    }
    throw new ParseFail(`unexpected argument token ${t.kind}`);
  }

  /** Parse one canonical statement, or return null with position restored. */
  tryStatement() {
    const start = this.pos;
    try {
      const t = this.peek();
      if (t.kind !== 'ident') throw new ParseFail('not a statement');
      if (t.text === 'const' || t.text === 'let' || t.text === 'var') {
        const keyword = this.next().text;
        const name = this.expect('ident').text;
        this.expect('=');
        const expr = this.parseExpr();
        const semi = this.expect(';');
        return {
          kind: 'def',
          keyword,
          name,
          expr,
          start: t.start,
          end: semi.end,
        };
      }
      if (t.text === 'return') {
        this.next();
        const expr = this.parseExpr();
        const semi = this.expect(';');
        return { kind: 'ret', expr, start: t.start, end: semi.end };
      }
      throw new ParseFail('not a canonical statement');
    } catch (err) {
      if (!(err instanceof ParseFail)) throw err;
      this.pos = start;
      return null;
    }
  }

  /** Consume tokens through the next ';' (or EOF) as an opaque raw segment. */
  rawStatement() {
    const first = this.next();
    let last = first;
    while (this.peek().kind !== 'eof' && last.kind !== ';') {
      last = this.next();
    }
    return { kind: 'raw', start: first.start, end: last.end };
  }
}

/**
 * Parse a script into a shape operation graph.
 *
 * Never throws: statements outside the canonical subset degrade to `raw`
 * segments. Returns `{ source, statements }` where each statement carries its
 * exact `[start, end)` character range in `source`.
 */
export function parseScript(source) {
  const parser = new Parser(tokenize(source));
  const statements = [];
  while (parser.peek().kind !== 'eof') {
    const stmt = parser.tryStatement() ?? parser.rawStatement();
    statements.push(stmt);
  }
  return { source, statements };
}

// ---------------------------------------------------------------------------
// Serializer

/** Format a number for script output, trimming float noise (1e-6 precision). */
export function fmtNum(value) {
  const rounded = Number(value.toFixed(6));
  return String(Object.is(rounded, -0) ? 0 : rounded);
}

function argToSource(arg) {
  return arg.kind === 'num' ? fmtNum(arg.value) : exprToSource(arg.expr);
}

export function exprToSource(expr) {
  switch (expr.kind) {
    case 'ref':
      return expr.name;
    case 'ctor':
      return `Shape.${expr.name}(${expr.args.map(argToSource).join(', ')})`;
    case 'call':
      return `${exprToSource(expr.target)}.${expr.method}(${expr.args.map(argToSource).join(', ')})`;
    default:
      throw new Error(`unknown expr kind ${expr.kind}`);
  }
}

export function statementToSource(stmt) {
  if (stmt.kind === 'def') {
    return `${stmt.keyword} ${stmt.name} = ${exprToSource(stmt.expr)};`;
  }
  if (stmt.kind === 'ret') {
    return `return ${exprToSource(stmt.expr)};`;
  }
  throw new Error('raw statements are never regenerated');
}

// ---------------------------------------------------------------------------
// Graph queries

/**
 * Flatten an expression into its chain: the primary link (ctor or ref)
 * followed by one link per method call, in application order.
 */
export function chainOf(expr) {
  const links = [];
  let e = expr;
  while (e.kind === 'call') {
    links.unshift({ kind: 'call', name: e.method, args: e.args, expr: e });
    e = e.target;
  }
  links.unshift(
    e.kind === 'ctor'
      ? { kind: 'ctor', name: e.name, args: e.args, expr: e }
      : { kind: 'ref', name: e.name, expr: e }
  );
  return links;
}

function refsInExpr(expr, out) {
  switch (expr.kind) {
    case 'ref':
      out.add(expr.name);
      break;
    case 'ctor':
      for (const a of expr.args) if (a.kind === 'expr') refsInExpr(a.expr, out);
      break;
    case 'call':
      refsInExpr(expr.target, out);
      for (const a of expr.args) if (a.kind === 'expr') refsInExpr(a.expr, out);
      break;
    default:
      break;
  }
}

/** Names of all shape variables an expression references. */
export function referencedNames(expr) {
  const out = new Set();
  refsInExpr(expr, out);
  return out;
}

/**
 * Sum of the trailing `.translate(...)` calls (all-numeric args) at the end
 * of an expression chain — the offset the GUI gizmo displays and edits.
 */
export function trailingTranslation(expr) {
  const t = [0, 0, 0];
  let e = expr;
  while (
    e.kind === 'call' &&
    e.method === 'translate' &&
    e.args.length === 3 &&
    e.args.every((a) => a.kind === 'num')
  ) {
    t[0] += e.args[0].value;
    t[1] += e.args[1].value;
    t[2] += e.args[2].value;
    e = e.target;
  }
  return t;
}

/** Human label for a statement's expression, e.g. "roundedBox · smoothUnion". */
function summarize(expr) {
  return chainOf(expr)
    .map((link) => link.name)
    .join(' · ');
}

/**
 * Display nodes for the scene tree. One node per statement; `def` and `ret`
 * nodes carry the chain, translation, and referenced names, `raw` nodes are
 * opaque.
 */
export function listNodes(graph) {
  return graph.statements.map((stmt, index) => {
    if (stmt.kind === 'def') {
      return {
        id: stmt.name,
        kind: 'def',
        name: stmt.name,
        index,
        label: summarize(stmt.expr),
        chain: chainOf(stmt.expr),
        translation: trailingTranslation(stmt.expr),
        refs: referencedNames(stmt.expr),
      };
    }
    if (stmt.kind === 'ret') {
      return {
        id: `ret@${index}`,
        kind: 'ret',
        index,
        label: summarize(stmt.expr),
        chain: chainOf(stmt.expr),
        refs: referencedNames(stmt.expr),
      };
    }
    return {
      id: `raw@${index}`,
      kind: 'raw',
      index,
      label: graph.source.slice(stmt.start, stmt.end).trim(),
    };
  });
}

// ---------------------------------------------------------------------------
// Mutations — each returns { source } with the new script text, or { error }.
// They regenerate only the statement they touch; all other bytes of the
// script (comments, raw code, formatting) pass through unchanged.

function splice(source, start, end, text) {
  return source.slice(0, start) + text + source.slice(end);
}

function respliceStatement(graph, stmt) {
  return { source: splice(graph.source, stmt.start, stmt.end, statementToSource(stmt)) };
}

function findDef(graph, name) {
  return graph.statements.find((s) => s.kind === 'def' && s.name === name) ?? null;
}

function statementByNodeId(graph, nodeId) {
  const at = nodeId.match(/^(?:ret|raw)@(\d+)$/);
  if (at) return graph.statements[Number(at[1])] ?? null;
  return findDef(graph, nodeId);
}

/**
 * Set one numeric argument of a chain link. `nodeId` is a def name or
 * `ret@<index>`; `linkIndex` indexes into `chainOf` for that statement.
 */
export function updateNumericArg(graph, nodeId, linkIndex, argIndex, value) {
  const stmt = statementByNodeId(graph, nodeId);
  if (!stmt || stmt.kind === 'raw') return { error: `no editable node "${nodeId}"` };
  if (!Number.isFinite(value)) return { error: 'value must be a finite number' };
  const link = chainOf(stmt.expr)[linkIndex];
  const arg = link?.args?.[argIndex];
  if (!arg || arg.kind !== 'num') {
    return { error: `no numeric argument at ${nodeId}[${linkIndex}][${argIndex}]` };
  }
  arg.value = value;
  return respliceStatement(graph, stmt);
}

/** Palette defaults: constructor arguments for each newly added primitive.
 * Adding one is a store mutation (storeSync.addPrimitiveNode), not a script
 * splice — the script view is regenerated from the tree afterwards. */
export const PALETTE = [
  { ctor: 'sphere', label: 'Sphere', args: [0.5] },
  { ctor: 'box3', label: 'Box', args: [0.5, 0.5, 0.5] },
  { ctor: 'roundedBox', label: 'Rounded box', args: [0.5, 0.5, 0.5, 0.1] },
  { ctor: 'cylinder', label: 'Cylinder', args: [0.3, 0.6] },
  { ctor: 'torus', label: 'Torus', args: [0.6, 0.2] },
];

/**
 * Delete a def node. Refuses (with { error }) when other statements still
 * reference it, so the script never breaks.
 */
export function deleteShape(graph, name) {
  const stmt = findDef(graph, name);
  if (!stmt) return { error: `no shape named "${name}"` };
  for (const other of graph.statements) {
    if (other === stmt || other.kind === 'raw') continue;
    if (referencedNames(other.expr).has(name)) {
      const where = other.kind === 'def' ? `"${other.name}"` : 'the return value';
      return { error: `"${name}" is still used by ${where}` };
    }
  }
  // Take the trailing newline with the statement so no blank line is left.
  let end = stmt.end;
  if (graph.source[end] === '\r') end += 1;
  if (graph.source[end] === '\n') end += 1;
  return { source: splice(graph.source, stmt.start, end, '') };
}

// ---------------------------------------------------------------------------
// Display metadata

/** Parameter names per constructor / method, for GUI labels. */
export const PARAM_NAMES = {
  sphere: ['r'],
  box3: ['hx', 'hy', 'hz'],
  roundedBox: ['hx', 'hy', 'hz', 'r'],
  cylinder: ['r', 'halfH'],
  torus: ['R', 'r'],
  capsule: ['x1', 'y1', 'z1', 'x2', 'y2', 'z2', 'r'],
  translate: ['x', 'y', 'z'],
  smoothUnion: ['other', 'r'],
  union: ['other'],
  intersect: ['other'],
  subtract: ['other'],
};

/** Label for argument `argIndex` of a chain link, e.g. "r" or "p2". */
export function argLabel(linkName, argIndex) {
  return PARAM_NAMES[linkName]?.[argIndex] ?? `p${argIndex}`;
}
