/**
 * React face of the WASM lifecycle store (src/wasm/loader.js). Components
 * read init status and the bound API classes from here instead of running
 * their own init — there is exactly one WASM lifecycle in the app.
 */
import {
  createContext,
  useContext,
  useEffect,
  useMemo,
  useSyncExternalStore,
} from 'react';
import { wasmLoader } from './loader.js';

const WasmContext = createContext(null);

export function WasmProvider({ loader = wasmLoader, children }) {
  const state = useSyncExternalStore(loader.subscribe, loader.getState, loader.getState);

  useEffect(() => {
    loader.ensure().catch(() => {
      // Failure is surfaced through state.status === 'failed'; swallowing
      // here just avoids an unhandled-rejection console error.
    });
  }, [loader]);

  const value = useMemo(
    () => ({
      status: state.status,
      error: state.error,
      api: state.api,
      ready: state.status === 'ready',
      retry: () => {
        loader.retry().catch(() => {});
      },
    }),
    [state, loader]
  );

  return <WasmContext.Provider value={value}>{children}</WasmContext.Provider>;
}

export function useWasm() {
  const ctx = useContext(WasmContext);
  if (!ctx) throw new Error('useWasm must be used inside <WasmProvider>');
  return ctx;
}
