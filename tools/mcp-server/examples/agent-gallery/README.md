# Agent gallery

Six worked examples of an MCP-capable agent operating the OpenSolid CAD kernel
end to end — prompt in, manufacturable part out — with **no GUI**. Each
transcript is real, unedited output from the [OpenSolid MCP server](../../README.md),
captured by [`build-gallery.mjs`](build-gallery.mjs): the agent writes a script,
gets mesh stats and a validity flag, renders screenshots, measures mass
properties, and exports STEP/STL/OBJ.

Regenerate the whole gallery (renders, exports, and these transcripts):

```bash
cd tools/mcp-server
npm run build     # only needed after a change under crates/
node examples/agent-gallery/build-gallery.mjs
```

| Example | Slug |
|---------|------|
| [a mounting bracket with four holes](angle-bracket.md) | angle-bracket |
| [a hinge leaf with knuckles and a pin bore](hinge-leaf.md) | hinge-leaf |
| [a shelled enclosure with a press-fit lid](enclosure.md) | enclosure |
| [a toothed disk from a circular pattern](gear-disk.md) | gear-disk |
| [a bottle from a revolved, shelled profile](bottle.md) | bottle |
| [a right-angle bracket with a gusset and filleted corner](bracket-right-angle.md) | bracket-right-angle |

Exported files (STEP/STL/OBJ) and PNG renders land in
[`../output/`](../output/); [`manifest.json`](../output/manifest.json) is the
machine-readable record of the run. See the
[Agent Guide](../../../../docs/AGENT_GUIDE.md) for how to connect a client, the
full tool reference, and the failure modes these examples exercise.

`bracket-right-angle` is also the acceptance part, built over the real MCP
stdio transport and gated by
[`test/bracket-acceptance.test.js`](../../test/bracket-acceptance.test.js). What
it cost to build — and the kernel bugs it surfaced — is written up in the
[friction log](../../../../docs/dogfood-bracket-friction-log.md).
