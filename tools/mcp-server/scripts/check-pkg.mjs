// Fail fast with a clear message if the generated wasm package is missing.
// `pkg/` is build output (not checked in); `npm run build` regenerates it.

import { existsSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, resolve } from 'node:path';

const here = dirname(fileURLToPath(import.meta.url));
const pkg = resolve(here, '..', 'pkg', 'opensolid_wasm.js');

if (!existsSync(pkg)) {
  console.error(
    'Missing tools/mcp-server/pkg — the wasm kernel is not built.\n' +
      'Build it first:\n\n  cd tools/mcp-server && npm run build\n',
  );
  process.exit(1);
}
