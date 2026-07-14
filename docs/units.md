# Document Units

The OpenSolid kernel is **unitless**: a coordinate is a bare `f64`. A box
built with `box3(1, 0.55, 0.8)` has extents `1`, `0.55`, `0.8` in whatever
unit the reader decides to interpret them as. That ambiguity is fine inside
the kernel, but it is a real interop problem the moment geometry leaves it —
a STEP file with no declared unit is opened by another CAD system as
millimetres, inches, or metres essentially at random.

The **document unit** closes that gap. It is a single per-document setting
(default **millimetres**) that gives the numbers a meaning for:

- **Display** — every length readout in the property panel, on-canvas sketch
  dimensions, and the mesh-accuracy status bar carries the unit suffix.
- **Entry** — dimension fields are understood to be in the document unit.
- **Export** — the STEP file declares the matching length unit, so importers
  resolve the geometry to the intended scale.

## The invariant: numbers never change

Switching the document unit is a **metadata + display** change only. It does
**not** rescale coordinates. `box3(1, …)` stays `1` whether the document is in
millimetres or inches; only its label and the exported STEP unit declaration
change. This keeps the kernel unitless and makes the setting cheap and
reversible — there is no lossy conversion to undo.

(A future "convert units and rescale" command is a separate feature; this one
deliberately does not touch geometry.)

## Supported units

| Key  | Label | STEP declaration                                    |
| ---- | ----- | --------------------------------------------------- |
| `mm` | mm    | `SI_UNIT(.MILLI.,.METRE.)`                           |
| `cm` | cm    | `SI_UNIT(.CENTI.,.METRE.)`                           |
| `m`  | m     | `SI_UNIT($,.METRE.)`                                 |
| `in` | in    | `CONVERSION_BASED_UNIT('INCH', … 25.4 mm)`          |

Inch is not an SI unit, so STEP declares it the standard way: a
`CONVERSION_BASED_UNIT` defined as `25.4` of the base millimetre `SI_UNIT`,
which importers resolve to the correct scale.

## Where it lives

- **Kernel writer** — `crates/opensolid-kernel/src/io/step/write.rs`:
  `LengthUnit` (`Millimetre` | `Centimetre` | `Metre` | `Inch`) on
  `StepWriteOptions`, emitting the declarations above. Coordinates are written
  verbatim, interpreted in the declared unit.
- **WASM bridge** — `crates/opensolid-wasm/src/step.rs` maps the document-unit
  key to `LengthUnit`; `exportStep(accuracy, unit)` in `lib.rs` exposes it.
  Unknown/omitted keys fall back to millimetres.
- **Playground** — `web/playground/src/lib/units.js` defines the unit model;
  `App.jsx` owns the setting (persisted in `localStorage`) and threads it to
  the toolbar unit picker (Export group), property panel, sketch canvas, and
  status bar.

## Rebuilding after a WASM change

The unit key reaches STEP through the WASM package. After changing the Rust
side, rebuild it so the playground picks up the new signature:

```bash
cd web/playground && npm run wasm
```

A `pkg/` built before the `unit` argument existed simply ignores the extra
argument and exports in millimetres — the export still works, it just isn't
unit-aware until rebuilt.
