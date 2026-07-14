# Agent transcript: a mounting bracket with four holes

An MCP-capable agent turns a one-line spec into a manufacturable part without ever opening a GUI — script, screenshot, measure, export.

Every tool call and result below is **real, unedited output** from the OpenSolid MCP server, captured by [`build-gallery.mjs`](build-gallery.mjs). The agent narration is prose framing; the numbers, renders, and files are the machine’s. Regenerate with `node examples/agent-gallery/build-gallery.mjs`.

> **User:** Design a 90° angle bracket: a 60×40×4 mm base flange with a 60×4×40 mm wall along the back edge, and four Ø6 mm mounting holes in the base. Export STEP and STL and tell me its mass.

---

**Agent:** The kernel centers primitives on the origin and takes half-extents, so a 60×40×4 base is `box3(30, 20, 2)` and the back wall is `box3(30, 2, 20)` stood up along −Y. I’ll union them, then punch four Ø6 (r=3) holes through the base on a rectangular pattern — the pattern is just a JS loop.

> 🔧 **`create_model`**
> ```js
> // 90° angle bracket: a 60×40×4 horizontal flange and a 60×4×40 vertical
> // flange along the back edge, with four Ø6 mounting holes in the base.
> const base = Shape.box3(30, 20, 2);                        // 60 × 40 × 4
> const wall = Shape.box3(30, 2, 20).translate(0, -18, 22);  // 60 × 4 × 40, back edge
> let bracket = base.union(wall);
> const hole = Shape.cylinder(3, 6);                         // r=3, punches through
> for (const x of [-20, 20]) for (const y of [-12, 6]) {
>   bracket = bracket.subtract(hole.translate(x, y, 0));
> }
> return bracket;
> ```
> ```json
> {
>   "model_id": "model-1",
>   "name": "angle-bracket",
>   "exact": false,
>   "mesh": {
>     "triangles": 22166,
>     "vertices": 11077
>   },
>   "boundingBox": {
>     "min": [
>       -30,
>       -20,
>       -2
>     ],
>     "max": [
>       30,
>       20,
>       42
>     ],
>     "size": [
>       60,
>       40,
>       44
>     ]
>   },
>   "volume": 18032.011256997677,
>   "valid": true,
>   "issues": []
> }
> ```

**Agent:** `create_model` returns immediately with `valid: true` and volume 18032 mm³ — the four holes and the boolean union all resolved into one closed solid. Let me eyeball it before exporting.

> 🔧 **`get_screenshot`** `{ "model_id": "model-1", "view": "iso", "width": 720, "height": 540 }`
>
> ![angle-bracket — iso view](../output/angle-bracket-iso.png)
>
> *(real 720×540 render, 17 KB PNG)*

**Agent:** That matches the spec. Mass properties, then the files:

> 🔧 **`measure`** `{ "model_id": "model-1", "query": "mass" }`
> ```json
> {
>   "volume": 18032.011256997677,
>   "surfaceArea": 10703.113814485621,
>   "centroid": [
>     -0.0018492654194972567,
>     -9.333888727169896,
>     11.583349370088724
>   ],
>   "inertia": [
>     [
>       6124309.147046339,
>       -275.66871196687083,
>       1414.1311746387219
>     ],
>     [
>       -275.66871196687083,
>       8707097.60635368,
>       1812303.553753316
>     ],
>     [
>       1414.1311746387219,
>       1812303.553753316,
>       7971354.13934768
>     ]
>   ],
>   "exact": false
> }
> ```

**Agent:** Volume 18032 mm³ = 18.0 cm³; in 6061 aluminium (2.70 g/cm³) that’s about **49 g**. Exporting STEP and STL:

> 🔧 **`export`** `{ "model_id": "model-1", "format": "step", "path": "angle-bracket.step" }`
> ```json
> {
>   "model_id": "model-1",
>   "format": "step",
>   "path": "output/angle-bracket.step",
>   "bytes": 6517305
> }
> ```

> 🔧 **`export`** `{ "model_id": "model-1", "format": "stl", "path": "angle-bracket.stl" }`
> ```json
> {
>   "model_id": "model-1",
>   "format": "stl",
>   "path": "output/angle-bracket.stl",
>   "bytes": 1108384
> }
> ```

**Agent:** Done — a valid, watertight bracket with four through-holes, delivered as an analytic-fallback STEP file and a print-ready STL, straight from the prompt.
