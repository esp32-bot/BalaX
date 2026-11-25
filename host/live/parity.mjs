#!/usr/bin/env node
// Checks that the Wasm build renders exactly what the native build renders: every fixture byte
// for byte against host/goldens/, and every sweep attitude by hash against `frames hashes --sweep`.
//
//   node host/live/parity.mjs        (after building host/build/live-native and live-wasm)
//
// Writes host/build/live/parity.json and prints one summary line. Exit 0 when everything matches.

import { execFileSync } from 'node:child_process';
import { mkdirSync, readFileSync, writeFileSync } from 'node:fs';
import { dirname, join, resolve } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';

import { channels, fnv1a64, fromChannels, PIXELS } from './frame.js';

const REPO = resolve(dirname(fileURLToPath(import.meta.url)), '../..');
const FRAMES = join(REPO, 'host/build/live-native/frames');
const MODULE = join(REPO, 'host/build/live-wasm/preview.mjs');
const OUT = join(REPO, 'host/build/live');

const native = JSON.parse(execFileSync(FRAMES, ['hashes', '--sweep'], { encoding: 'utf8', maxBuffer: 1 << 24 }));
const { default: createPreview } = await import(pathToFileURL(MODULE).href);
const wasm = await createPreview();
const pixels = () => wasm.HEAPU16.subarray(wasm._frame_pixels() / 2, wasm._frame_pixels() / 2 + PIXELS);
const wasmHash = () => (wasm._frame_hash_hi() >>> 0).toString(16).padStart(8, '0') +
  (wasm._frame_hash_lo() >>> 0).toString(16).padStart(8, '0');

const fixtures = [];
for (let i = 0; i < wasm._fixture_count(); i++) {
  const id = wasm.UTF8ToString(wasm._fixture_id(i));
  wasm._render_fixture(i);
  const frame = pixels();
  const golden = readFileSync(join(REPO, 'host/goldens', `${id}.rgb565`));
  const bytes = Buffer.from(frame.buffer, frame.byteOffset, frame.byteLength);  // Wasm memory is little-endian
  fixtures.push({
    id,
    bytesMatchGolden: Buffer.compare(bytes, golden) === 0,
    wasmHash: wasmHash(),
    nativeHash: native.fixtures[i]?.hash,
    jsHash: fnv1a64(frame),  // the page's hash, checked against C++'s
  });
}

const sweepMismatches = [];
const count = wasm._sweep_count();
const started = performance.now();
for (let k = 0; k < count; k++) {
  wasm._render_sweep(k);
  const hash = wasmHash();
  if (hash !== native.sweep[k]) sweepMismatches.push({ k, wasm: hash, native: native.sweep[k] });
}
const sweepMs = performance.now() - started;

// The page reads bx preview's PNG back into RGB565, so channels() must invert exactly.
let roundTripErrors = 0;
for (let p = 0; p < 0x10000; p++) if (fromChannels(...channels(p)) !== p) roundTripErrors++;

const ok = fixtures.every((f) => f.bytesMatchGolden && f.wasmHash === f.nativeHash && f.jsHash === f.nativeHash) &&
  count === native.sweep.length && sweepMismatches.length === 0 && roundTripErrors === 0;
const report = {
  schemaVersion: 1,
  status: ok ? 'pass' : 'fail',
  fixtures,
  sweep: { frames: count, nativeFrames: native.sweep.length, mismatches: sweepMismatches.length,
    firstMismatches: sweepMismatches.slice(0, 10), wasmMsPerFrame: +(sweepMs / count).toFixed(4) },
  rgb565RoundTripErrors: roundTripErrors,
};
mkdirSync(OUT, { recursive: true });
writeFileSync(join(OUT, 'parity.json'), JSON.stringify(report, null, 2) + '\n');
console.log(JSON.stringify({
  status: report.status,
  fixturesExact: fixtures.filter((f) => f.bytesMatchGolden).length + '/' + fixtures.length,
  sweepMismatches: sweepMismatches.length + '/' + count,
  jsHashMatches: fixtures.every((f) => f.jsHash === f.nativeHash),
  rgb565RoundTripErrors: roundTripErrors,
  wasmMsPerFrame: report.sweep.wasmMsPerFrame,
}));
process.exit(ok ? 0 : 1);
