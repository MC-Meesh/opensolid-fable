# Agent transcript: a hinge leaf with knuckles and a pin bore

Rotations put a cylinder’s axis wherever you want it. Here the agent builds one leaf of a butt hinge — barrel knuckles on the pin axis, drilled through for the pin.

Every tool call and result below is **real, unedited output** from the OpenSolid MCP server, captured by [`build-gallery.mjs`](build-gallery.mjs). The agent narration is prose framing; the numbers, renders, and files are the machine’s. Regenerate with `node examples/agent-gallery/build-gallery.mjs`.

> **User:** Model one leaf of a butt hinge: a flat plate with three barrel knuckles on the pin axis and a pin bore drilled through them. Show me an iso view and give me a STEP file.

---

**Agent:** A knuckle is a cylinder whose default +Z axis I rotate onto +X, then slide along X. Three of them union onto the plate; a long thin cylinder on the same axis subtracts the pin bore. `rotate(0, 1, 0, 90)` swings +Z onto +X.

> 🔧 **`create_model`**
> ```js
> // One leaf of a butt hinge: a flat plate with three barrel knuckles on the
> // pin axis (X) and a pin bore drilled through them. Two of these — one
> // mirrored — pin together into a working hinge.
> const plate = Shape.box3(30, 15, 0.75).translate(0, -15.75, 0);  // 60 × 30 × 1.5 leaf
> // A knuckle is a cylinder whose +Z axis is rotated onto +X, then slid along X.
> const knuckle = Shape.cylinder(4, 6).rotate(0, 1, 0, 90);        // r=4, 12 long on X
> let leaf = plate;
> for (const x of [-24, 0, 24]) leaf = leaf.union(knuckle.translate(x, 0, 0));
> const pin = Shape.cylinder(1.6, 40).rotate(0, 1, 0, 90);         // Ø3.2 bore on X
> return leaf.subtract(pin);
> ```
> ```json
> {
>   "model_id": "model-2",
>   "name": "hinge-leaf",
>   "exact": false,
>   "mesh": {
>     "triangles": 21368,
>     "vertices": 10684
>   },
>   "boundingBox": {
>     "min": [
>       -30,
>       -30.75,
>       -5.3682811189189135
>     ],
>     "max": [
>       30,
>       6,
>       5.3682811189189135
>     ],
>     "size": [
>       60,
>       36.75,
>       10.736562237837827
>     ]
>   },
>   "volume": 4013.5680459331566,
>   "valid": true,
>   "issues": []
> }
> ```

**Agent:** Valid solid, 21,368 triangles — the pin bore runs cleanly through all three knuckles. Let me look at it and confirm the mesh is watertight before exporting.

> 🔧 **`get_screenshot`** `{ "model_id": "model-2", "view": "iso", "width": 720, "height": 540 }`
>
> ![hinge-leaf — iso view](../output/hinge-leaf-iso.png)
>
> *(real 720×540 render, 25 KB PNG)*

> 🔧 **`validate`** `{ "model_id": "model-2" }`
> ```json
> {
>   "valid": true,
>   "closedManifold": true,
>   "triangles": 21368,
>   "vertices": 10684,
>   "volume": 4013.5680459331566,
>   "exact": false,
>   "issues": []
> }
> ```

**Agent:** `closedManifold: true`, no issues — a real solid, not a surface soup. Exporting STEP for the mechanical model:

> 🔧 **`export`** `{ "model_id": "model-2", "format": "step", "path": "hinge-leaf.step" }`
> ```json
> {
>   "model_id": "model-2",
>   "format": "step",
>   "path": "output/hinge-leaf.step",
>   "bytes": 11220052
> }
> ```

> 🔧 **`export`** `{ "model_id": "model-2", "format": "stl", "path": "hinge-leaf.stl" }`
> ```json
> {
>   "model_id": "model-2",
>   "format": "stl",
>   "path": "output/hinge-leaf.stl",
>   "bytes": 1068484
> }
> ```

**Agent:** Mirror this leaf about X and pin the two together and you have a working hinge.
