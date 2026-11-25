// Status screen frames in JavaScript, for the live preview page and the Node scripts beside it.
// Frames are RGB565 Uint16Arrays, row by row, like host/goldens/*.rgb565 and host/frames.h.

export const WIDTH = 135;
export const HEIGHT = 240;
export const PIXELS = WIDTH * HEIGHT;

// FNV-1a 64 over the little-endian bytes, the same as frames::hash, in 16-bit limbs so it
// needs no BigInt. Returns 16 hex digits.
export function fnv1a64(pixels) {
  let h0 = 0x2325, h1 = 0x8422, h2 = 0x9ce4, h3 = 0xcbf2;  // 0xcbf29ce484222325
  const step = (byte) => {
    h0 ^= byte;
    // h * 0x100000001b3 = h * 0x1b3 + (h << 40)
    let t0 = h0 * 0x1b3;
    let t1 = h1 * 0x1b3;
    let t2 = h2 * 0x1b3 + (h0 << 8);
    let t3 = h3 * 0x1b3 + (h1 << 8);
    t1 += t0 >>> 16;
    t2 += t1 >>> 16;
    t3 += t2 >>> 16;
    h0 = t0 & 0xffff;
    h1 = t1 & 0xffff;
    h2 = t2 & 0xffff;
    h3 = t3 & 0xffff;
  };
  for (let i = 0; i < pixels.length; i++) {
    step(pixels[i] & 0xff);
    step(pixels[i] >>> 8);
  }
  return [h3, h2, h1, h0].map((limb) => limb.toString(16).padStart(4, '0')).join('');
}

// The 8-bit channels host/preview.cpp writes to its PNG.
export function channels(pixel) {
  return [
    Math.floor((pixel >>> 11) * 255 / 31),
    Math.floor((pixel >>> 5 & 0x3f) * 255 / 63),
    Math.floor((pixel & 0x1f) * 255 / 31),
  ];
}

// The inverse of channels(), exact for every RGB565 value.
export function fromChannels(r, g, b) {
  return Math.round(r * 31 / 255) << 11 | Math.round(g * 63 / 255) << 5 | Math.round(b * 31 / 255);
}

export function toRgba(pixels, rgba = new Uint8ClampedArray(pixels.length * 4)) {
  for (let i = 0; i < pixels.length; i++) {
    const [r, g, b] = channels(pixels[i]);
    rgba[4 * i] = r;
    rgba[4 * i + 1] = g;
    rgba[4 * i + 2] = b;
    rgba[4 * i + 3] = 255;
  }
  return rgba;
}

// Where actual differs from expected, in the frames CLI's terms.
export function compare(expected, actual) {
  let changed = 0, first = -1;
  let left = WIDTH, top = HEIGHT, right = -1, bottom = -1;
  for (let i = 0; i < actual.length; i++) {
    if (expected[i] === actual[i]) continue;
    if (changed++ === 0) first = i;
    const x = i % WIDTH, y = Math.floor(i / WIDTH);
    left = Math.min(left, x);
    right = Math.max(right, x);
    top = Math.min(top, y);
    bottom = Math.max(bottom, y);
  }
  if (!changed) return { changed };
  return {
    changed,
    first: { x: first % WIDTH, y: Math.floor(first / WIDTH), expected: expected[first], actual: actual[first] },
    bbox: { x: left, y: top, w: right - left + 1, h: bottom - top + 1 },
  };
}

// A raw .rgb565 file's bytes as pixels (little-endian, whatever the host's byte order).
export function fromBytes(bytes) {
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const pixels = new Uint16Array(bytes.byteLength / 2);
  for (let i = 0; i < pixels.length; i++) pixels[i] = view.getUint16(2 * i, true);
  return pixels;
}
