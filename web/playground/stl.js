// Binary STL assembly from flat mesh buffers. Pure function so it can be
// unit-tested in Node as well as used by the browser app.

/**
 * Build a binary STL file from flat mesh arrays.
 *
 * @param {Float32Array} positions xyz-interleaved vertex positions
 * @param {Uint32Array} indices flat triangle indices, three per triangle
 * @returns {ArrayBuffer} complete binary STL file contents
 *
 * Facet normals are recomputed from the triangle geometry (right-hand rule
 * over the existing counter-clockwise winding), which is what the STL format
 * expects; per-vertex shading normals are not representable in STL.
 */
export function buildBinaryStl(positions, indices) {
  const triCount = Math.floor(indices.length / 3);
  const buffer = new ArrayBuffer(84 + triCount * 50);
  const view = new DataView(buffer);

  const header = 'OpenSolid playground binary STL';
  for (let i = 0; i < header.length; i++) view.setUint8(i, header.charCodeAt(i));
  view.setUint32(80, triCount, true);

  let offset = 84;
  for (let t = 0; t < triCount; t++) {
    const i0 = indices[t * 3] * 3;
    const i1 = indices[t * 3 + 1] * 3;
    const i2 = indices[t * 3 + 2] * 3;

    const ax = positions[i0], ay = positions[i0 + 1], az = positions[i0 + 2];
    const bx = positions[i1], by = positions[i1 + 1], bz = positions[i1 + 2];
    const cx = positions[i2], cy = positions[i2 + 1], cz = positions[i2 + 2];

    // Facet normal = normalize((b - a) × (c - a)); zero for degenerate tris.
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
      view.setFloat32(offset, value, true);
      offset += 4;
    }
    view.setUint16(offset, 0, true);
    offset += 2;
  }

  return buffer;
}
