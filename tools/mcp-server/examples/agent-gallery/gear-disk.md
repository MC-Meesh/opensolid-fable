# Agent transcript: a toothed disk from a circular pattern

The script vocabulary is a real programming language, so a circular pattern is just a `for` loop rotating one feature around the axis — the axis the disk is actually on.

Every tool call and result below is **real, unedited output** from the OpenSolid MCP server, captured by [`build-gallery.mjs`](build-gallery.mjs). The agent narration is prose framing; the numbers, renders, and files are the machine’s. Regenerate with `node examples/agent-gallery/build-gallery.mjs`.

> **User:** Make a 16-tooth spur-gear-style disk: a root disk with rectangular teeth on a circular pattern and a central bore. Top and iso views, and an STL for printing.

---

**Agent:** One tooth box, rotated into 16 positions by a loop, unioned onto a root cylinder, minus a central bore. `(2 * Math.PI * i) / TEETH` spaces the teeth evenly — `rotate` is in radians, so a full turn is `2π`, not `360`. The circular pattern has to turn about the **same axis the disk is on** — `Shape.cylinder` is **+Y**, so that is `rotate(0, 1, 0, ...)`. Pattern about Z instead and the teeth swing up out of the disk plane into a ring of floating blocks, which still meshes and still reports `valid: true`.

> 🔧 **`create_model`**
> ```js
> // A toothed disk: a root disk with N teeth placed on a circular pattern by
> // rotating one tooth box around the disk's own axis (+Y, the cylinder axis),
> // plus a central bore. The pattern is just a JS loop — the script vocabulary
> // is a real programming language. rotate() takes RADIANS: a full turn is 2π.
> const TEETH = 16, TH = 4, ROOT = 16, BORE = 4;
> let gear = Shape.cylinder(ROOT, TH);                          // disk faces in xz, axis +Y
> const tooth = Shape.box3(3, TH, 2.2).translate(ROOT + 1.5, 0, 0);  // radial x, thick y
> for (let i = 0; i < TEETH; i++) {
>   gear = gear.union(tooth.rotate(0, 1, 0, (2 * Math.PI * i) / TEETH));  // pattern about +Y
> }
> return gear.subtract(Shape.cylinder(BORE, TH + 2));           // central bore, coaxial
> ```
> ```json
> {
>   "model_id": "model-5",
>   "name": "gear-disk",
>   "exact": false,
>   "mesh": {
>     "triangles": 95856,
>     "vertices": 47928
>   },
>   "boundingBox": {
>     "min": [
>       -20.5,
>       -4,
>       -20.500000000000004
>     ],
>     "max": [
>       20.5,
>       4,
>       20.5
>     ],
>     "size": [
>       41,
>       8,
>       41
>     ]
>   },
>   "volume": 8601.150151593016,
>   "valid": true,
>   "issues": []
> }
> ```

**Agent:** All 16 teeth resolved — `valid: true`, volume 8601 mm³. Top view to check the tooth count and spacing, then iso:

> 🔧 **`get_screenshot`** `{ "model_id": "model-5", "view": "top", "width": 720, "height": 540 }`
>
> ![gear-disk — top view](../output/gear-disk-top.png)
>
> *(real 720×540 render, 22 KB PNG)*

> 🔧 **`get_screenshot`** `{ "model_id": "model-5", "view": "iso", "width": 720, "height": 540 }`
>
> ![gear-disk — iso view](../output/gear-disk-iso.png)
>
> *(real 720×540 render, 33 KB PNG)*

**Agent:** Sixteen evenly-spaced teeth, and the disk reads 8 mm thick in y — which is the check that matters here. Had I patterned about Z, the top view would still show a tidy ring of sixteen blocks and `valid` would still be `true`, but the bounding box would come back 41 × 41 × 32 instead of 41 × 8 × 41: teeth orbiting the disk rather than sitting on its rim. STL exports the mesh directly:

> 🔧 **`export`** `{ "model_id": "model-5", "format": "stl", "path": "gear-disk.stl" }`
> ```json
> {
>   "model_id": "model-5",
>   "format": "stl",
>   "path": "output/gear-disk.stl",
>   "bytes": 4792884
> }
> ```

**Agent:** And STEP, for the mechanical model:

> 🔧 **`export`** `{ "model_id": "model-5", "format": "step", "path": "gear-disk.step" }`
> ```json
> {
>   "model_id": "model-5",
>   "format": "step",
>   "path": "output/gear-disk.step",
>   "bytes": 41674481
> }
> ```
