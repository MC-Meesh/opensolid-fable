/**
 * Full-viewport error state for a failed WASM init. Replaces the loading
 * overlay — the app must never sit on an infinite spinner. The reason
 * string comes from src/wasm/loader.js and already names the failing URL,
 * HTTP status, and the `npm run wasm` fix.
 */
export default function WasmErrorScreen({ error, onRetry }) {
  return (
    <div className="wasm-error" role="alert">
      <h2>WASM engine failed to load</h2>
      <pre>{error}</pre>
      <button onClick={onRetry}>Retry</button>
    </div>
  );
}
