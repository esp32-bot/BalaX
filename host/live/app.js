// The live preview page: shows each fixture's newest frame, compares it with the reference frame
// in host/goldens/, and keeps the last good frames on screen when a build fails.

import { channels, compare, fnv1a64, fromBytes, fromChannels, HEIGHT, PIXELS, WIDTH } from './frame.js';

const RENDER_TIMEOUT_MS = 3000;
const PREVIEW_SCALE = 3, PREVIEW_GAP = 8;  // host/preview.cpp's layout, for bx-preview mode
const now = () => performance.timeOrigin + performance.now();
const $ = (id) => document.getElementById(id);

const state = {
  mode: null,
  fixtures: [],
  goldens: new Map(),  // id -> pixels
  frames: new Map(),   // id -> pixels on screen
  cards: new Map(),    // id -> { canvas, badge }
  shownGen: 0,         // the build whose frames are on screen
  latestFramesGen: 0,  // the newest build with frames, shown or still loading
  error: null,         // { gen, title, log } of the newest failure

  worker: null,        // wasm mode: the worker holding the newest good build
  attitude: null,      // wasm mode: { fixture, pitch, roll } while the sliders are moved
  slidersFixture: -1,
  lastPaint: null,
};

function card(id) {
  const figure = document.createElement('figure');
  const canvas = document.createElement('canvas');
  canvas.width = WIDTH;
  canvas.height = HEIGHT;
  const caption = document.createElement('figcaption');
  const name = document.createElement('span');
  name.textContent = id;
  const badge = document.createElement('span');
  badge.className = 'badge';
  caption.append(name, badge);
  figure.append(canvas, caption);
  $('screens').append(figure);
  state.cards.set(id, { canvas, badge });
}

function paintCard(id, pixels) {
  const { canvas, badge } = state.cards.get(id) ?? {};
  if (!canvas) return;
  const golden = state.goldens.get(id);
  const showDiff = $('diff').checked && golden;
  const image = new ImageData(WIDTH, HEIGHT);
  for (let i = 0; i < PIXELS; i++) {
    let [r, g, b] = channels(pixels[i]);
    if (showDiff) {
      if (golden[i] !== pixels[i]) [r, g, b] = [255, 0, 255];
      else [r, g, b] = [r * 0.3, g * 0.3, b * 0.3];
    }
    image.data.set([r, g, b, 255], 4 * i);
  }
  canvas.getContext('2d').putImageData(image, 0, 0);
  const overridden = state.attitude?.id === id;
  if (!golden) {
    badge.textContent = 'no reference frame';
    badge.className = 'badge missing';
    return;
  }
  const result = compare(golden, pixels);
  if (!result.changed) {
    badge.textContent = 'matches reference';
    badge.className = 'badge pass';
  } else {
    const { x, y, w, h } = result.bbox;
    badge.textContent = `${result.changed} px differ · ${w}×${h} at ${x},${y}`;
    badge.className = overridden ? 'badge note' : 'badge fail';
  }
  if (overridden) badge.textContent += ' · attitude moved';
}

function paintAll() {
  for (const [id, pixels] of state.frames) paintCard(id, pixels);
}

const nextPaint = () => new Promise((done) => requestAnimationFrame(() => requestAnimationFrame(done)));

function renderInWorker(worker, message) {
  return new Promise((done, fail) => {
    const timer = setTimeout(() => fail(new Error(`the renderer didn't answer in ${RENDER_TIMEOUT_MS} ms`)), RENDER_TIMEOUT_MS);
    worker.onmessage = ({ data }) => {
      clearTimeout(timer);
      data.ok ? done(data.frames) : fail(new Error(data.error));
    };
    worker.onerror = (event) => {
      clearTimeout(timer);
      fail(new Error(event.message || 'the renderer failed to load'));
    };
    worker.postMessage(message);
  });
}

async function fetchBytes(url) {
  const response = await fetch(url, { cache: 'no-store' });
  if (!response.ok) throw new Error(`${url}: ${response.status}`);
  return new Uint8Array(await response.arrayBuffer());
}

// Each mode's way to the frames of one build, as [{ id, pixels }].
const LOADERS = {
  async wasm(message) {
    const worker = new Worker('worker.js', { type: 'module' });
    try {
      const frames = await renderInWorker(worker, { url: `/gen/${message.gen}/preview.mjs`, attitude: wasmAttitude() });
      state.worker?.terminate();
      state.worker = worker;
      return frames;
    } catch (error) {
      worker.terminate();
      throw error;
    }
  },
  async native(message) {
    const manifest = JSON.parse(new TextDecoder().decode(await fetchBytes(`/gen/${message.gen}/manifest.json`)));
    return Promise.all(manifest.fixtures.map(async (f) => ({
      id: f.id, pixels: fromBytes(await fetchBytes(`/gen/${message.gen}/${f.file}`)),
    })));
  },
  async 'bx-preview'(message) {
    const blob = new Blob([await fetchBytes(`/gen/${message.gen}/preview.png`)], { type: 'image/png' });
    const bitmap = await createImageBitmap(blob, { colorSpaceConversion: 'none', premultiplyAlpha: 'none' });
    const canvas = new OffscreenCanvas(bitmap.width, bitmap.height);
    const context = canvas.getContext('2d', { willReadFrequently: true });
    context.drawImage(bitmap, 0, 0);
    const { data, width } = context.getImageData(0, 0, bitmap.width, bitmap.height);
    const cell = WIDTH + 2 * PREVIEW_GAP;
    return message.fixtures.map((id, index) => {
      const pixels = new Uint16Array(PIXELS);
      const left = (index * cell + PREVIEW_GAP) * PREVIEW_SCALE, top = PREVIEW_GAP * PREVIEW_SCALE;
      for (let y = 0; y < HEIGHT; y++) {
        for (let x = 0; x < WIDTH; x++) {
          const at = 4 * ((top + y * PREVIEW_SCALE) * width + left + x * PREVIEW_SCALE);
          pixels[y * WIDTH + x] = fromChannels(data[at], data[at + 1], data[at + 2]);
        }
      }
      return { id, pixels };
    });
  },
};

function wasmAttitude() {
  const a = state.attitude;
  return a ? { fixture: state.fixtures.indexOf(a.id), pitch: a.pitch, roll: a.roll } : null;
}

// Shows the newest failure until a newer build's frames are on screen.
function showError(gen, what, log) {
  state.error = { gen, what, log: log ?? '' };
  renderError();
}

function renderError() {
  const error = state.error;
  if (!error || error.gen <= state.shownGen) {
    $('error').hidden = true;
    return;
  }
  $('error-title').textContent = `${error.what}; showing ${state.shownGen ? `build ${state.shownGen}` : 'no frames yet'}.`;
  $('error-log').textContent = error.log;
  $('error').hidden = false;
}

async function onFrames(message) {
  const receivedAt = now();
  try {
    const frames = await LOADERS[state.mode](message);
    if (message.gen < state.latestFramesGen) return;  // a newer build's frames arrived meanwhile
    const loadedAt = now();
    for (const { id, pixels } of frames) state.frames.set(id, pixels);
    paintAll();
    await nextPaint();
    const paintedAt = now();
    state.shownGen = message.gen;
    renderError();
    const hashes = Object.fromEntries(frames.map(({ id, pixels }) => [id, fnv1a64(pixels)]));
    const differing = frames.filter(({ id, pixels }) => state.goldens.has(id) && compare(state.goldens.get(id), pixels).changed);
    state.lastPaint = { gen: message.gen, build: message.finishedAt - message.startedAt, total: paintedAt - message.changedAt };
    $('gen').textContent = `build ${message.gen}`;
    $('summary').textContent = `built in ${state.lastPaint.build.toFixed(0)} ms · on screen ${state.lastPaint.total.toFixed(0)} ms after the save · ` +
      (differing.length ? `${differing.length} of ${frames.length} differ from the reference` : 'all match the reference');
    $('summary').className = differing.length ? 'fail' : 'pass';
    if (state.mode === 'wasm') setupSliders(frames);
    fetch('/api/paint', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ gen: message.gen, mode: state.mode, changedAt: message.changedAt, startedAt: message.startedAt,
        finishedAt: message.finishedAt, receivedAt, loadedAt, paintedAt, hashes, attitudeMoved: !!state.attitude }),
    }).catch(() => {});
  } catch (error) {
    showError(message.gen, `Build ${message.gen} didn't render`, String(error?.message ?? error));
  }
}

function onEvent({ data }) {
  const message = JSON.parse(data);
  if (message.type === 'frames') {
    state.latestFramesGen = Math.max(state.latestFramesGen, message.gen);
    onFrames(message);
  } else if (message.type === 'error') {
    showError(message.gen, `Build ${message.gen} failed`, message.log);
  }
}

// Wasm mode only: move one fixture's attitude without rebuilding.
function setupSliders(frames) {
  if (state.slidersFixture >= 0) return;
  const pick = $('attitude-fixture');
  frames.forEach(({ id }) => pick.add(new Option(id, id)));
  pick.value = frames.find((f) => f.id === 'balancing')?.id ?? frames[0].id;
  state.slidersFixture = 0;
  const reset = () => {
    const f = frames.find((frame) => frame.id === pick.value);
    $('pitch').value = Number.isFinite(f.pitch) ? f.pitch : 0;
    $('roll').value = Number.isFinite(f.roll) ? f.roll : 0;
    state.attitude = null;
    update(false);
  };
  const update = async (moved = true) => {
    $('pitch-out').textContent = `${Number($('pitch').value).toFixed(2)}°`;
    $('roll-out').textContent = `${Number($('roll').value).toFixed(2)}°`;
    if (moved) state.attitude = { id: pick.value, pitch: Number($('pitch').value), roll: Number($('roll').value) };
    if (!state.worker) return;
    try {
      const rendered = await renderInWorker(state.worker, { attitude: wasmAttitude() });
      for (const { id, pixels } of rendered) state.frames.set(id, pixels);
      paintAll();
    } catch (error) {
      showError(state.shownGen + 0.5, 'The renderer stopped answering', String(error?.message ?? error));  // until the next build
    }
  };
  $('pitch').oninput = $('roll').oninput = () => update();
  pick.onchange = reset;
  $('reset').onclick = reset;
  $('attitude').hidden = false;
  reset();
}

async function main() {
  const info = await (await fetch('/api/state', { cache: 'no-store' })).json();
  state.mode = info.mode;
  state.fixtures = info.fixtures;
  $('mode').textContent = info.mode;
  $('watched').textContent = `watching ${info.watched} files`;
  for (const id of info.fixtures) {
    card(id);
    try {
      state.goldens.set(id, fromBytes(await fetchBytes(`/goldens/${id}.rgb565`)));
    } catch {
      // No reference frame yet: `bx frames write-goldens` makes them.
    }
  }
  $('diff').onchange = paintAll;
  new EventSource('/events').onmessage = onEvent;
}

main().catch((error) => showError(Infinity, 'The preview failed to start', String(error?.stack ?? error)));
