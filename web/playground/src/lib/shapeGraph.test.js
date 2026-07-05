import { describe, expect, it } from 'vitest';
import {
  addShape,
  argLabel,
  chainOf,
  deleteShape,
  exprToSource,
  fmtNum,
  listNodes,
  parseScript,
  referencedNames,
  setTranslation,
  statementToSource,
  trailingTranslation,
  updateNumericArg,
} from './shapeGraph.js';
import { DEFAULT_SCRIPT } from './defaultScript.js';

const SIMPLE = `const body = Shape.roundedBox(1.0, 0.55, 0.8, 0.15);
const bump = Shape.sphere(0.55).translate(0, 0.65, 0);
const solid = body.smoothUnion(bump, 0.25);
const hole = Shape.cylinder(0.28, 2.0);
return solid.subtract(hole);
`;

describe('parseScript', () => {
  it('parses the canonical demo script into defs and a return', () => {
    const graph = parseScript(SIMPLE);
    expect(graph.statements.map((s) => s.kind)).toEqual([
      'def',
      'def',
      'def',
      'def',
      'ret',
    ]);
    expect(graph.statements.map((s) => s.name).slice(0, 4)).toEqual([
      'body',
      'bump',
      'solid',
      'hole',
    ]);
  });

  it('parses the shipped default script (comments and all)', () => {
    const graph = parseScript(DEFAULT_SCRIPT);
    const kinds = graph.statements.map((s) => s.kind);
    expect(kinds).toEqual(['def', 'def', 'def', 'def', 'ret']);
  });

  it('records exact statement source ranges', () => {
    const graph = parseScript(SIMPLE);
    for (const stmt of graph.statements) {
      const text = SIMPLE.slice(stmt.start, stmt.end);
      expect(text.endsWith(';')).toBe(true);
      if (stmt.kind === 'def') expect(text).toContain(`${stmt.name} =`);
    }
  });

  it('parses nested constructor expressions as arguments', () => {
    const graph = parseScript(
      'return Shape.box3(1, 1, 1).subtract(Shape.sphere(0.5).translate(0, 1, 0));'
    );
    const ret = graph.statements[0];
    expect(ret.kind).toBe('ret');
    const sub = ret.expr;
    expect(sub.method).toBe('subtract');
    expect(sub.args[0].kind).toBe('expr');
    expect(sub.args[0].expr.method).toBe('translate');
  });

  it('parses negative and exponent numbers', () => {
    const graph = parseScript('const a = Shape.sphere(1).translate(-0.5, 1e-2, -2.5E1);');
    const args = graph.statements[0].expr.args;
    expect(args.map((a) => a.value)).toEqual([-0.5, 0.01, -25]);
  });

  it('degrades non-canonical statements to raw segments', () => {
    const src = `const r = 0.5 * 2;
const a = Shape.sphere(1);
let weird = someFn(a);
return a;
`;
    const graph = parseScript(src);
    expect(graph.statements.map((s) => s.kind)).toEqual(['raw', 'def', 'raw', 'ret']);
  });

  it('treats arithmetic in arguments as non-canonical', () => {
    const graph = parseScript('const a = Shape.sphere(r * 2);');
    expect(graph.statements[0].kind).toBe('raw');
  });

  it('never throws on garbage input', () => {
    for (const src of ['@#$%^', 'const = ;;;', 'return', '((((', '']) {
      expect(() => parseScript(src)).not.toThrow();
    }
  });
});

describe('serialization', () => {
  it('serialization is a fixed point after one canonicalization pass', () => {
    // `1.0` canonicalizes to `1` on first write; after that, statements
    // roundtrip byte-for-byte.
    const graph = parseScript(SIMPLE);
    for (const stmt of graph.statements) {
      const once = statementToSource(stmt);
      const reparsed = parseScript(once).statements[0];
      expect(reparsed.kind).toBe(stmt.kind);
      expect(statementToSource(reparsed)).toBe(once);
    }
  });

  it('formats numbers without float noise', () => {
    expect(fmtNum(0.6500000000000001)).toBe('0.65');
    expect(fmtNum(-0.0000001)).toBe('0');
    expect(fmtNum(2)).toBe('2');
    expect(fmtNum(-1.25)).toBe('-1.25');
  });

  it('serializes nested expressions', () => {
    const src = 'return a.union(Shape.sphere(0.5).translate(1, 0, 0));';
    const graph = parseScript(src);
    expect(exprToSource(graph.statements[0].expr)).toBe(
      'a.union(Shape.sphere(0.5).translate(1, 0, 0))'
    );
  });
});

describe('graph queries', () => {
  it('flattens chains in application order', () => {
    const graph = parseScript('const a = Shape.sphere(1).translate(0, 1, 0).union(b);');
    const chain = chainOf(graph.statements[0].expr);
    expect(chain.map((l) => l.name)).toEqual(['sphere', 'translate', 'union']);
    expect(chain[0].kind).toBe('ctor');
    expect(chain[1].kind).toBe('call');
  });

  it('collects referenced names, including nested ones', () => {
    const graph = parseScript('return a.union(b.subtract(Shape.sphere(1).union(c)));');
    expect([...referencedNames(graph.statements[0].expr)].sort()).toEqual(['a', 'b', 'c']);
  });

  it('sums consecutive trailing translations', () => {
    const graph = parseScript(
      'const a = Shape.sphere(1).translate(1, 0, 0).translate(0, 2, 0);'
    );
    expect(trailingTranslation(graph.statements[0].expr)).toEqual([1, 2, 0]);
  });

  it('ignores translations buried before other calls', () => {
    const graph = parseScript('const a = Shape.sphere(1).translate(1, 0, 0).union(b);');
    expect(trailingTranslation(graph.statements[0].expr)).toEqual([0, 0, 0]);
  });

  it('lists display nodes with labels and translations', () => {
    const nodes = listNodes(parseScript(SIMPLE));
    expect(nodes.map((n) => n.kind)).toEqual(['def', 'def', 'def', 'def', 'ret']);
    expect(nodes[1].translation).toEqual([0, 0.65, 0]);
    expect(nodes[1].label).toBe('sphere · translate');
    expect(nodes[4].label).toBe('solid · subtract');
  });

  it('lists raw statements as opaque nodes', () => {
    const nodes = listNodes(parseScript('const x = 1 + 2;\nreturn a;'));
    expect(nodes[0].kind).toBe('raw');
    expect(nodes[0].label).toBe('const x = 1 + 2;');
  });
});

describe('updateNumericArg', () => {
  it('rewrites a single constructor argument in place', () => {
    const graph = parseScript(SIMPLE);
    const { source } = updateNumericArg(graph, 'body', 0, 3, 0.3);
    expect(source).toContain('const body = Shape.roundedBox(1, 0.55, 0.8, 0.3);');
    // Every other statement is untouched.
    expect(source).toContain('const bump = Shape.sphere(0.55).translate(0, 0.65, 0);');
    expect(source).toContain('return solid.subtract(hole);');
  });

  it('edits numeric args of chained method calls', () => {
    const graph = parseScript(SIMPLE);
    const { source } = updateNumericArg(graph, 'solid', 1, 1, 0.4);
    expect(source).toContain('const solid = body.smoothUnion(bump, 0.4);');
  });

  it('edits the return statement via its node id', () => {
    const graph = parseScript('return Shape.sphere(1);');
    const { source } = updateNumericArg(graph, 'ret@0', 0, 0, 2);
    expect(source).toBe('return Shape.sphere(2);');
  });

  it('rejects non-numeric targets and bad values', () => {
    const graph = parseScript(SIMPLE);
    expect(updateNumericArg(graph, 'solid', 1, 0, 1).error).toBeTruthy(); // arg 0 is a ref
    expect(updateNumericArg(graph, 'body', 0, 0, NaN).error).toBeTruthy();
    expect(updateNumericArg(graph, 'nope', 0, 0, 1).error).toBeTruthy();
  });

  it('preserves comments and raw code around the edit', () => {
    const src = `// heading comment
const a = Shape.sphere(1); // trailing note
const magic = compute(a);
return a;
`;
    const graph = parseScript(src);
    const { source } = updateNumericArg(graph, 'a', 0, 0, 2);
    expect(source).toBe(`// heading comment
const a = Shape.sphere(2); // trailing note
const magic = compute(a);
return a;
`);
  });
});

describe('setTranslation', () => {
  it('replaces an existing trailing translate with the absolute position', () => {
    const graph = parseScript(SIMPLE);
    const { source } = setTranslation(graph, 'bump', [0.5, 1, -0.25]);
    expect(source).toContain('const bump = Shape.sphere(0.55).translate(0.5, 1, -0.25);');
  });

  it('appends a translate when the chain has none', () => {
    const graph = parseScript(SIMPLE);
    const { source } = setTranslation(graph, 'hole', [1, 0, 0]);
    expect(source).toContain('const hole = Shape.cylinder(0.28, 2).translate(1, 0, 0);');
  });

  it('coalesces stacked trailing translates into one', () => {
    const graph = parseScript('const a = Shape.sphere(1).translate(1, 0, 0).translate(0, 1, 0);');
    const { source } = setTranslation(graph, 'a', [2, 2, 2]);
    expect(source).toBe('const a = Shape.sphere(1).translate(2, 2, 2);');
  });

  it('drops the call entirely for a zero translation', () => {
    const graph = parseScript('const a = Shape.sphere(1).translate(1, 0, 0);');
    const { source } = setTranslation(graph, 'a', [0, 0, 0]);
    expect(source).toBe('const a = Shape.sphere(1);');
  });

  it('does not disturb translates that are not trailing', () => {
    const graph = parseScript('const a = Shape.sphere(1).translate(1, 0, 0).union(b);');
    const { source } = setTranslation(graph, 'a', [0, 5, 0]);
    expect(source).toBe('const a = Shape.sphere(1).translate(1, 0, 0).union(b).translate(0, 5, 0);');
  });
});

describe('addShape', () => {
  it('inserts a def before the return and unions it into the result', () => {
    const graph = parseScript(SIMPLE);
    const { source, name } = addShape(graph, 'sphere', [0.5]);
    expect(name).toBe('s1');
    expect(source).toContain('const s1 = Shape.sphere(0.5);\nreturn solid.subtract(hole).union(s1);');
    // The new script still parses fully canonically.
    const reparsed = parseScript(source);
    expect(reparsed.statements.map((s) => s.kind)).toEqual([
      'def',
      'def',
      'def',
      'def',
      'def',
      'ret',
    ]);
  });

  it('generates fresh names that skip taken ones', () => {
    const graph = parseScript('const s1 = Shape.sphere(1);\nreturn s1;');
    const { name } = addShape(graph, 'box3', [0.5, 0.5, 0.5]);
    expect(name).toBe('s2');
  });

  it('creates a return statement when the script has none', () => {
    const graph = parseScript('// empty scene\n');
    const { source } = addShape(graph, 'torus', [0.6, 0.2]);
    expect(source).toBe('// empty scene\nconst s1 = Shape.torus(0.6, 0.2);\nreturn s1;\n');
  });

  it('keeps comments above the return statement intact', () => {
    const src = 'const a = Shape.sphere(1);\n// the output\nreturn a;\n';
    const graph = parseScript(src);
    const { source } = addShape(graph, 'sphere', [0.5]);
    expect(source).toBe(
      'const a = Shape.sphere(1);\n// the output\nconst s1 = Shape.sphere(0.5);\nreturn a.union(s1);\n'
    );
  });
});

describe('deleteShape', () => {
  it('removes an unreferenced def and its line', () => {
    const src = 'const a = Shape.sphere(1);\nconst b = Shape.box3(1, 1, 1);\nreturn b;\n';
    const graph = parseScript(src);
    const { source } = deleteShape(graph, 'a');
    expect(source).toBe('const b = Shape.box3(1, 1, 1);\nreturn b;\n');
  });

  it('refuses to delete a def that is still referenced', () => {
    const graph = parseScript(SIMPLE);
    expect(deleteShape(graph, 'bump').error).toMatch(/still used by "solid"/);
    expect(deleteShape(graph, 'solid').error).toMatch(/still used by the return value/);
  });

  it('reports unknown names', () => {
    expect(deleteShape(parseScript(SIMPLE), 'ghost').error).toBeTruthy();
  });
});

describe('argLabel', () => {
  it('names known parameters and falls back to positional labels', () => {
    expect(argLabel('sphere', 0)).toBe('r');
    expect(argLabel('roundedBox', 3)).toBe('r');
    expect(argLabel('translate', 2)).toBe('z');
    expect(argLabel('mystery', 4)).toBe('p4');
  });
});

describe('GUI -> script -> GUI roundtrip', () => {
  it('a full edit session preserves hand-written content', () => {
    const src = `// My part, do not touch this comment.
const base = Shape.box3(1, 0.4, 1);
const custom = base.union(externallyDefined);
return base;
`;
    let graph = parseScript(src);

    const added = addShape(graph, 'cylinder', [0.3, 0.6]);
    graph = parseScript(added.source);
    const moved = setTranslation(graph, added.name, [0, 1, 0]);
    graph = parseScript(moved.source);
    const resized = updateNumericArg(graph, 'base', 0, 1, 0.5);
    graph = parseScript(resized.source);

    expect(resized.source).toBe(`// My part, do not touch this comment.
const base = Shape.box3(1, 0.5, 1);
const custom = base.union(externallyDefined);
const s1 = Shape.cylinder(0.3, 0.6).translate(0, 1, 0);
return base.union(s1);
`);
  });
});
