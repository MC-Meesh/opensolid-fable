/**
 * Layout smoke tests (of-4eh.20): the app chrome must fit one window height —
 * no document-level scrollbar at the reference viewport sizes (1280x800 and
 * up). jsdom has no layout engine, so a pixel measurement cannot prove that
 * here; instead these tests pin the CSS contract that makes a document
 * scrollbar impossible at ANY viewport size:
 *
 *   - html/body/#root are height-locked to the viewport with no margin,
 *   - .app fills that height exactly and clips overflow,
 *   - the only scroll containers are interior panes (editor, tree list,
 *     error readout, floating overlays) — never the chrome itself,
 *   - the header toolbar never wraps to a second row,
 *   - the sidebar's width clamp leaves the viewport real room at 1280px.
 *
 * Any edit that reintroduces document scrolling has to break one of these.
 */
import { readFileSync } from 'node:fs';
import { describe, expect, it } from 'vitest';
import { renderToString } from 'react-dom/server';
import App from './App.jsx';
import { WasmProvider } from './wasm/WasmContext.jsx';

const css = readFileSync(new URL('./styles.css', import.meta.url), 'utf8');

// Naive rule parser — good enough for this hand-written flat stylesheet
// (no nesting, no at-rule blocks except @keyframes, which we skip).
function parseRules(text) {
  const rules = [];
  const re = /([^{}]+)\{([^{}]*)\}/g;
  const stripped = text.replace(/\/\*[\s\S]*?\*\//g, '');
  let match;
  while ((match = re.exec(stripped)) !== null) {
    const selector = match[1].trim();
    if (selector.startsWith('@') || /^\d+%$|^from$|^to$/.test(selector)) continue;
    rules.push({ selector, block: match[2] });
  }
  return rules;
}

function block(selector) {
  const rule = parseRules(css).find((r) => r.selector === selector);
  expect(rule, `stylesheet must keep a \`${selector}\` rule`).toBeDefined();
  return rule.block;
}

const REFERENCE_VIEWPORTS = [
  [1280, 800],
  [1440, 900],
  [1920, 1080],
];

describe('layout contract: everything fits one window height', () => {
  it('height-locks the document chain (html/body/#root) with no margin', () => {
    const root = block('html, body, #root');
    expect(root).toMatch(/height:\s*100%/);
    expect(root).toMatch(/margin:\s*0/);
  });

  it('sizes .app to the viewport and clips overflow — the chrome never scrolls', () => {
    const app = block('.app');
    expect(app).toMatch(/height:\s*100%/);
    expect(app).toMatch(/overflow:\s*hidden/);
  });

  it('only sanctioned interior panes are scroll containers', () => {
    // Every scrollable region must be inside the sidebar or a floating
    // viewport overlay, all of which are height-bounded by .app.
    const allowed = new Set([
      '.feature-tree-body', // tree list (sidebar Tree tab)
      '.error', // script error readout (sidebar bottom)
      '.prop-panel', // floating property panel overlay
      '.mass-panel', // floating mass properties overlay (max-height bounded)
      '.wasm-error pre', // wasm failure screen detail
      '.error-boundary pre', // crash detail
    ]);
    // (The code editor scrolls too, via CodeMirror's own .cm-scroller.)
    for (const { selector, block: body } of parseRules(css)) {
      if (/overflow(?:-[xy])?:\s*(auto|scroll)/.test(body)) {
        expect(allowed, `unexpected scroll container: ${selector}`).toContain(selector);
      }
    }
  });

  it('keeps the main toolbar to a single row', () => {
    const toolbar = block('.main-toolbar');
    expect(toolbar).toMatch(/flex-wrap:\s*nowrap/);
  });

  it('gives the viewport all remaining width', () => {
    // Without flex: 1 the viewport column collapses to its in-flow content
    // (the absolutely positioned canvas contributes no width).
    const right = block('.right');
    expect(right).toMatch(/flex:\s*1/);
    expect(right).toMatch(/min-width:\s*0/);
  });

  it.each(REFERENCE_VIEWPORTS)(
    'leaves the viewport at least 700px beside a maxed-out sidebar at %dx%d',
    (width) => {
      const sidebar = block('.sidebar');
      const maxWidth = Number(/max-width:\s*(\d+)px/.exec(sidebar)?.[1]);
      const splitter = 5;
      expect(maxWidth).toBeGreaterThan(0);
      expect(width - maxWidth - splitter).toBeGreaterThanOrEqual(700);
    }
  );

  it('renders the chrome as exactly sidebar | splitter | viewport', () => {
    const loader = {
      getState: () => ({
        status: 'ready',
        error: null,
        api: { WasmShape: class {}, WasmProfile2D: class {} },
      }),
      subscribe: () => () => {},
      ensure: () => Promise.resolve(null),
      retry: () => Promise.resolve(null),
    };
    const html = renderToString(
      <WasmProvider loader={loader}>
        <App />
      </WasmProvider>
    );
    const sidebar = html.indexOf('class="sidebar"');
    const splitter = html.indexOf('class="splitter"');
    const right = html.indexOf('class="right"');
    expect(sidebar).toBeGreaterThanOrEqual(0);
    expect(splitter).toBeGreaterThan(sidebar);
    expect(right).toBeGreaterThan(splitter);
    // No legacy stacked panels outside the tabbed sidebar.
    expect(html).not.toContain('scene-panel');
    expect(html).not.toContain('class="left"');
  });
});
