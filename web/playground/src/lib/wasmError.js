// Kernel errors cross the wasm boundary as one JSON string (of-2y4.9):
//   {"code": ..., "category": ..., "message": ..., "hint"?: ...}
// wasm-bindgen throws a Rust `Err(String)` as the raw string, so anywhere the
// playground displays a caught error it may be holding that JSON. This turns
// it back into the human sentence (with the hint appended when present) and
// leaves every other thrown value untouched.
export function wasmErrorMessage(err) {
  const raw = String(err?.message ?? err);
  if (raw.startsWith('{"code":"')) {
    try {
      const parsed = JSON.parse(raw);
      if (parsed && typeof parsed.message === 'string') {
        return parsed.hint ? `${parsed.message}\n\nHint: ${parsed.hint}` : parsed.message;
      }
    } catch {
      // Looked structured but was not valid JSON — show it as-is.
    }
  }
  return raw;
}
