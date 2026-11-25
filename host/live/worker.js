// Runs one build of the Wasm renderer off the page's thread, so a renderer that hangs or crashes
// can be dropped without losing the frames already on screen.
//
// Messages in:  { url } loads that build's preview.mjs, then renders like { attitude };
//               { attitude: null | { fixture, pitch, roll } } renders every fixture again.
// Message out:  { ok: true, frames: [{ id, pixels, hash }] } or { ok: false, error }.

let renderer = null;

const hex = (hi, lo) => (hi >>> 0).toString(16).padStart(8, '0') + (lo >>> 0).toString(16).padStart(8, '0');

self.onmessage = async ({ data }) => {
  try {
    if (data.url) {
      const { default: createPreview } = await import(data.url);
      renderer = await createPreview();
    }
    const r = renderer;
    const size = r._frame_width() * r._frame_height();
    const frames = [];
    for (let i = 0; i < r._fixture_count(); i++) {
      const override = data.attitude?.fixture === i ? data.attitude : null;
      if (override) r._render_attitude(i, override.pitch, override.roll);
      else r._render_fixture(i);
      const start = r._frame_pixels() / 2;
      frames.push({
        id: r.UTF8ToString(r._fixture_id(i)),
        pixels: r.HEAPU16.slice(start, start + size),
        hash: hex(r._frame_hash_hi(), r._frame_hash_lo()),
        pitch: r._fixture_pitch(i),
        roll: r._fixture_roll(i),
      });
    }
    self.postMessage({ ok: true, frames }, frames.map((f) => f.pixels.buffer));
  } catch (error) {
    self.postMessage({ ok: false, error: String(error?.stack ?? error) });
  }
};
