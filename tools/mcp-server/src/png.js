// Minimal PNG encoder (8-bit RGBA, color type 6) built on Node's zlib. No
// third-party dependency — the screenshot tool renders in a software
// rasterizer and encodes here.

import { deflateSync, inflateSync } from 'node:zlib';

const SIGNATURE = Buffer.from([137, 80, 78, 71, 13, 10, 26, 10]);

// Standard PNG CRC-32 (polynomial 0xEDB88320), table built once.
const CRC_TABLE = (() => {
  const table = new Uint32Array(256);
  for (let n = 0; n < 256; n++) {
    let c = n;
    for (let k = 0; k < 8; k++) {
      c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1;
    }
    table[n] = c >>> 0;
  }
  return table;
})();

function crc32(buf) {
  let c = 0xffffffff;
  for (let i = 0; i < buf.length; i++) {
    c = CRC_TABLE[(c ^ buf[i]) & 0xff] ^ (c >>> 8);
  }
  return (c ^ 0xffffffff) >>> 0;
}

function chunk(type, data) {
  const typeBuf = Buffer.from(type, 'ascii');
  const length = Buffer.alloc(4);
  length.writeUInt32BE(data.length, 0);
  const crcInput = Buffer.concat([typeBuf, data]);
  const crc = Buffer.alloc(4);
  crc.writeUInt32BE(crc32(crcInput), 0);
  return Buffer.concat([length, typeBuf, data, crc]);
}

/**
 * Encode an RGBA pixel buffer as a PNG.
 *
 * @param {Uint8Array|Buffer} rgba row-major RGBA bytes, length = width*height*4
 * @param {number} width
 * @param {number} height
 * @returns {Buffer} complete PNG file bytes
 */
export function encodePng(rgba, width, height) {
  if (rgba.length !== width * height * 4) {
    throw new Error(
      `rgba length ${rgba.length} does not match ${width}x${height}x4 = ${width * height * 4}`,
    );
  }

  const ihdr = Buffer.alloc(13);
  ihdr.writeUInt32BE(width, 0);
  ihdr.writeUInt32BE(height, 4);
  ihdr.writeUInt8(8, 8); // bit depth
  ihdr.writeUInt8(6, 9); // color type: RGBA
  ihdr.writeUInt8(0, 10); // compression
  ihdr.writeUInt8(0, 11); // filter
  ihdr.writeUInt8(0, 12); // interlace

  // Prefix each scanline with filter type 0 (none).
  const stride = width * 4;
  const raw = Buffer.alloc((stride + 1) * height);
  for (let y = 0; y < height; y++) {
    raw[y * (stride + 1)] = 0;
    Buffer.from(rgba.buffer, rgba.byteOffset + y * stride, stride).copy(
      raw,
      y * (stride + 1) + 1,
    );
  }

  const idat = deflateSync(raw);

  return Buffer.concat([
    SIGNATURE,
    chunk('IHDR', ihdr),
    chunk('IDAT', idat),
    chunk('IEND', Buffer.alloc(0)),
  ]);
}

/**
 * Decode a PNG this module wrote, back to `{width, height, rgba}`.
 *
 * Deliberately narrow: 8-bit RGBA, no interlacing, filter type 0 on every
 * scanline — exactly what {@link encodePng} emits, and it throws rather than
 * guessing at anything else. It lives here so the encoder's format
 * assumptions are stated once.
 *
 * The renderer's claims are claims about *pixels* — that a section's cut face
 * is amber, that a hidden edge left no ink, that a line drawing sits on a
 * light ground. Asserting those against the compressed file bytes is not
 * possible, and asserting them against a checked-in reference image makes
 * every deliberate change a binary diff. This is the third option.
 *
 * @param {Buffer} png
 * @returns {{width:number, height:number, rgba:Buffer}}
 */
export function decodePng(png) {
  if (png.length < 8 || !png.subarray(0, 8).equals(SIGNATURE)) {
    throw new Error('not a PNG (bad signature)');
  }
  let width = 0;
  let height = 0;
  const idat = [];
  let offset = 8;
  while (offset + 8 <= png.length) {
    const length = png.readUInt32BE(offset);
    const type = png.toString('ascii', offset + 4, offset + 8);
    const data = png.subarray(offset + 8, offset + 8 + length);
    if (type === 'IHDR') {
      width = data.readUInt32BE(0);
      height = data.readUInt32BE(4);
      if (data[8] !== 8 || data[9] !== 6 || data[12] !== 0) {
        throw new Error('decodePng only handles 8-bit RGBA, non-interlaced PNGs');
      }
    } else if (type === 'IDAT') {
      idat.push(Buffer.from(data));
    } else if (type === 'IEND') {
      break;
    }
    offset += 12 + length; // length + type + data + crc
  }
  const raw = inflateSync(Buffer.concat(idat));
  const stride = width * 4;
  const rgba = Buffer.alloc(stride * height);
  for (let y = 0; y < height; y++) {
    const filter = raw[y * (stride + 1)];
    if (filter !== 0) {
      throw new Error(`decodePng expects filter type 0 on every row, got ${filter} on row ${y}`);
    }
    raw.copy(rgba, y * stride, y * (stride + 1) + 1, y * (stride + 1) + 1 + stride);
  }
  return { width, height, rgba };
}
