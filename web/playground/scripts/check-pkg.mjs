// Preflight check that the generated wasm-bindgen output exists.
//
//   node scripts/check-pkg.mjs          exit 1 if pkg/ is missing (prebuild)
//   node scripts/check-pkg.mjs --warn   warn but continue (predev — the dev
//                                       server still starts and the browser
//                                       shows an actionable error screen)
import { existsSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const projectRoot = path.dirname(path.dirname(fileURLToPath(import.meta.url)));
const pkgEntry = path.join(projectRoot, 'pkg', 'opensolid_wasm.js');
const warnOnly = process.argv.includes('--warn');

if (existsSync(pkgEntry)) {
  process.exit(0);
}

const banner = `
${'='.repeat(72)}
  Missing generated WASM package: ${path.relative(process.cwd(), pkgEntry)}

  pkg/ is build output, not checked in. Generate it with:

      npm run wasm

  (requires the wasm32-unknown-unknown target and wasm-pack — see README.md)
${'='.repeat(72)}
`;

console.error(banner);

if (warnOnly) {
  console.error('Continuing anyway; the app will show an error screen until pkg/ exists.\n');
  process.exit(0);
}
process.exit(1);
