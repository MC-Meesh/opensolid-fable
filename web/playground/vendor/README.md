# Vendored runtime dependencies

The playground loads no CDN resources at runtime; these files are committed
so a clone works offline.

- `three.module.min.js`, `three.core.min.js`, `OrbitControls.js`, `LICENSE`
  — three.js **r180** (npm `three@0.180.0`), MIT licensed. Since r167 the
  three build is split in two: `three.module.min.js` imports its sibling
  `./three.core.min.js`, so both must be vendored together.
  `OrbitControls.js` is unmodified from `examples/jsm/controls/`; its
  `from 'three'` import is resolved by the import map in `../index.html`.

To upgrade, replace the files with the same paths from the new npm release:

```
curl -fsSL -o three.module.min.js https://cdn.jsdelivr.net/npm/three@<ver>/build/three.module.min.js
curl -fsSL -o three.core.min.js   https://cdn.jsdelivr.net/npm/three@<ver>/build/three.core.min.js
curl -fsSL -o OrbitControls.js    https://cdn.jsdelivr.net/npm/three@<ver>/examples/jsm/controls/OrbitControls.js
curl -fsSL -o LICENSE             https://cdn.jsdelivr.net/npm/three@<ver>/LICENSE
```
