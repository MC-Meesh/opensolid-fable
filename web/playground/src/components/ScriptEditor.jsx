import { useEffect, useRef } from 'react';
import { EditorView, keymap } from '@codemirror/view';
import { Prec } from '@codemirror/state';
import { basicSetup } from 'codemirror';
import { javascript } from '@codemirror/lang-javascript';
import { oneDark } from '@codemirror/theme-one-dark';

const editorTheme = EditorView.theme({
  '&': {
    flex: '1',
    minHeight: '0',
    fontSize: '12.5px',
    backgroundColor: 'var(--panel)',
  },
  '.cm-scroller': {
    fontFamily: 'ui-monospace, SFMono-Regular, Menlo, Consolas, monospace',
    lineHeight: '1.5',
  },
  '&.cm-focused': { outline: 'none' },
});

/**
 * CodeMirror 6 editor for the shape script.
 *
 * Uncontrolled: `initialDoc` seeds the document on mount; edits flow out
 * through `onChange`. Mod-Enter (Cmd on macOS, Ctrl elsewhere) fires `onRun`.
 */
export default function ScriptEditor({ initialDoc, onChange, onRun }) {
  const hostRef = useRef(null);
  const onChangeRef = useRef(onChange);
  const onRunRef = useRef(onRun);
  onChangeRef.current = onChange;
  onRunRef.current = onRun;
  const initialDocRef = useRef(initialDoc);

  useEffect(() => {
    const view = new EditorView({
      doc: initialDocRef.current,
      parent: hostRef.current,
      extensions: [
        Prec.highest(
          keymap.of([
            {
              key: 'Mod-Enter',
              run: () => {
                onRunRef.current?.();
                return true;
              },
            },
          ])
        ),
        basicSetup,
        javascript(),
        oneDark,
        editorTheme,
        EditorView.updateListener.of((update) => {
          if (update.docChanged) {
            onChangeRef.current?.(update.state.doc.toString());
          }
        }),
      ],
    });
    return () => view.destroy();
  }, []);

  return <div className="editor" ref={hostRef} />;
}
