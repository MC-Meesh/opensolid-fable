// Mesh extraction and file-format assembly (STL, OBJ) from a WasmShape.
// Pure functions over flat mesh buffers, mirroring the playground's exporters
// so downstream files match what the GUI produces.

/**
 * Mesh a shape to flat typed-array buffers. Uses adaptive meshing (triangle
 * budget tracks surface complexity); exact boolean results serve their
 * validated analytic tessellation regardless of `accuracy`.
 *
 * @param {object} shape a WasmShape
 * @param {{accuracy?:number}} [opts]
 * @returns {{positions:Float32Array, normals:Float32Array, indices:Uint32Array, triangles:number, vertices:number}}
 */
export function getMesh(shape, opts = {}) {
  const bounds = shape.bounds();
  const extent = Math.max(
    bounds[3] - bounds[0],
    bounds[4] - bounds[1],
    bounds[5] - bounds[2],
    1e-9,
  );
  const accuracy =
    Number.isFinite(opts.accuracy) && opts.accuracy > 0 ? opts.accuracy : 5e-3 * extent;
  const data = shape.meshAdaptive(accuracy, undefined);
  const positions = data.positions;
  const normals = data.normals;
  const indices = data.indices;
  return {
    positions,
    normals,
    indices,
    triangles: indices.length / 3,
    vertices: positions.length / 3,
  };
}

/**
 * Build a binary STL from flat mesh buffers. Facet normals are recomputed
 * from geometry (STL cannot carry per-vertex normals).
 *
 * @param {Float32Array} positions xyz-interleaved vertex positions
 * @param {Uint32Array} indices flat triangle indices, three per triangle
 * @returns {Buffer}
 */
export function buildBinaryStl(positions, indices) {
  const triCount = Math.floor(indices.length / 3);
  const buffer = Buffer.alloc(84 + triCount * 50);

  buffer.write('OpenSolid MCP binary STL', 0, 'ascii');
  buffer.writeUInt32LE(triCount, 80);

  let offset = 84;
  for (let t = 0; t < triCount; t++) {
    const i0 = indices[t * 3] * 3;
    const i1 = indices[t * 3 + 1] * 3;
    const i2 = indices[t * 3 + 2] * 3;

    const ax = positions[i0], ay = positions[i0 + 1], az = positions[i0 + 2];
    const bx = positions[i1], by = positions[i1 + 1], bz = positions[i1 + 2];
    const cx = positions[i2], cy = positions[i2 + 1], cz = positions[i2 + 2];

    const ux = bx - ax, uy = by - ay, uz = bz - az;
    const vx = cx - ax, vy = cy - ay, vz = cz - az;
    let nx = uy * vz - uz * vy;
    let ny = uz * vx - ux * vz;
    let nz = ux * vy - uy * vx;
    const len = Math.hypot(nx, ny, nz);
    if (len > 1e-30) {
      nx /= len; ny /= len; nz /= len;
    } else {
      nx = ny = nz = 0;
    }

    for (const value of [nx, ny, nz, ax, ay, az, bx, by, bz, cx, cy, cz]) {
      buffer.writeFloatLE(value, offset);
      offset += 4;
    }
    // Attribute byte count.
    buffer.writeUInt16LE(0, offset);
    offset += 2;
  }
  return buffer;
}

/**
 * Build a Wavefront OBJ (ASCII) from flat mesh buffers, including per-vertex
 * normals. OBJ indices are 1-based.
 *
 * @param {Float32Array} positions
 * @param {Float32Array} normals
 * @param {Uint32Array} indices
 * @returns {string}
 */
export function buildObj(positions, normals, indices) {
  const lines = ['# OpenSolid MCP export', 'o model'];
  const vertexCount = positions.length / 3;
  for (let i = 0; i < vertexCount; i++) {
    lines.push(`v ${positions[i * 3]} ${positions[i * 3 + 1]} ${positions[i * 3 + 2]}`);
  }
  const hasNormals = normals && normals.length === positions.length;
  if (hasNormals) {
    for (let i = 0; i < vertexCount; i++) {
      lines.push(`vn ${normals[i * 3]} ${normals[i * 3 + 1]} ${normals[i * 3 + 2]}`);
    }
  }
  for (let t = 0; t < indices.length; t += 3) {
    const a = indices[t] + 1;
    const b = indices[t + 1] + 1;
    const c = indices[t + 2] + 1;
    if (hasNormals) {
      lines.push(`f ${a}//${a} ${b}//${b} ${c}//${c}`);
    } else {
      lines.push(`f ${a} ${b} ${c}`);
    }
  }
  return lines.join('\n') + '\n';
}
