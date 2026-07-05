import { forwardRef, useEffect, useImperativeHandle, useRef } from 'react';
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

const ScriptEditor = forwardRef(function ScriptEditor({ initialDoc, onChange, onRun }, ref) {
  const hostRef = useRef(null);
  const viewRef = useRef(null);
  const onChangeRef = useRef(onChange);
  const onRunRef = useRef(onRun);
  onChangeRef.current = onChange;
  onRunRef.current = onRun;
  const initialDocRef = useRef(initialDoc);

  useImperativeHandle(ref, () => ({
    setDoc(text) {
      const view = viewRef.current;
      if (view) {
        view.dispatch({
          changes: { from: 0, to: view.state.doc.length, insert: text },
        });
      }
    },
  }));

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
    viewRef.current = view;
    return () => {
      view.destroy();
      viewRef.current = null;
    };
  }, []);

  return <div className="editor" ref={hostRef} />;
});

export default ScriptEditor;
