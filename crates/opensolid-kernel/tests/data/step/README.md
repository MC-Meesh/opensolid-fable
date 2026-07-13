# Vendored STEP test corpus

Real-world STEP Part 21 files exercised by `tests/step_corpus.rs`. All three
were written by CATIA V5R20 (AP214, `AUTOMOTIVE_DESIGN` schema) and are
long-standing CAx-IF / PDES interoperability test parts.

Vendored from the [STEPcode](https://github.com/stepcode/stepcode) repository
(`data/ap214e3/`, commit on the `develop` branch, fetched 2026-07-12), which
distributes them under the BSD-3-Clause license. Retrieved files are
unmodified.

| File | Size | Contents |
|------|------|----------|
| `sg1-c5-214.stp` | 24 KB | 1 solid — planes, cylinders, one cone (all analytic) |
| `io1-cm-214.stp` | 41 KB | 1 solid — planes, cylinders, one torus (all analytic) |
| `dm1-id-214.stp` | 86 KB | 3 solids — includes B-spline curves/surfaces (exercises the mesh/diagnostic fallback paths) |

The reader maps `MANIFOLD_SOLID_BREP` entities regardless of the declared
schema, so AP214 files are valid input for the AP203-oriented importer; the
product/assembly skeleton entities are simply ignored.
