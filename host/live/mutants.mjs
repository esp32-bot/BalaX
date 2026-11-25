#!/usr/bin/env node
// Seeds small bugs into the status screen renderer, one at a time, and records whether each
// check catches it and how long the check takes from the save to its verdict:
//
//   frames   incremental native build + `frames verify` against host/goldens/
//   bx test  `tools/bx test`, today's laptop checks (C++ and Python)
//
//   node host/live/mutants.mjs
//
// The last mutant changes a screen no fixture shows, as a control: neither check should catch it.
// The edited files must be unchanged in git first, and are restored byte for byte afterwards,
// on Ctrl+C too. Writes host/build/live/mutants.json.

import { spawnSync } from 'node:child_process';
import { mkdirSync, readFileSync, writeFileSync } from 'node:fs';
import { dirname, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const REPO = resolve(dirname(fileURLToPath(import.meta.url)), '../..');
const NATIVE = join(REPO, 'host/build/live-native');
const ENV = { ...process.env, TMPDIR: join(REPO, 'host/build/tmp') };
const VIEW = 'main/status_view.cpp';
const LAYOUT = 'main/status_layout.h';

const MUTANTS = [
  { id: 'title-color', file: LAYOUT, from: 'TITLE = gfx::rgb565(123, 211, 255)', to: 'TITLE = gfx::rgb565(115, 211, 255)' },
  { id: 'title-text', file: VIEW, from: '"BalaX", TITLE', to: '"Balax", TITLE' },
  { id: 'hint-row', file: VIEW, from: 'centered(strip, FULL_WIDTH, HINT_Y, hint', to: 'centered(strip, FULL_WIDTH, HINT_Y + 1, hint' },
  { id: 'pitch-scale', file: VIEW, from: 'values.pitch_deg * 1.25f', to: 'values.pitch_deg * 1.20f' },
  { id: 'horizon-rounding', file: VIEW, from: 'std::ceil(x_mid - distance / slope)', to: 'std::floor(x_mid - distance / slope)' },
  { id: 'charging-color', file: VIEW, from: 'v.battery_ma > 5 ? TITLE', to: 'v.battery_ma > 100 ? TITLE' },
  { id: 'motor-bar-length', file: VIEW, from: '(meter.h / 2 - 1)', to: '(meter.h / 2)' },
  { id: 'pmic-warm-threshold', file: VIEW, from: 'thermal_color(v.pmic_c, 42, 46)', to: 'thermal_color(v.pmic_c, 45, 46)' },
  { id: 'uptime-minutes', file: VIEW, from: 's / 60 % 60', to: 's / 60' },
  { id: 'ip-row', file: VIEW, from: 'strip.text(MARGIN, 211, connected', to: 'strip.text(MARGIN, 212, connected' },
  { id: 'armed-hint (no fixture shows it)', file: VIEW, from: '"Stand it up"', to: '"Stand it up!"', covered: false },
];

const files = [...new Set(MUTANTS.map((m) => m.file))];
const originals = new Map(files.map((file) => [file, readFileSync(join(REPO, file))]));
const restore = () => {
  for (const [file, bytes] of originals) {
    if (!readFileSync(join(REPO, file)).equals(bytes)) writeFileSync(join(REPO, file), bytes);
  }
};
const gitClean = () => spawnSync('git', ['diff', '--quiet', '--', ...files], { cwd: REPO }).status === 0;
process.on('SIGINT', () => { restore(); process.exit(130); });
process.on('SIGTERM', () => { restore(); process.exit(143); });

const now = () => performance.now();
const run = (command, args) => spawnSync(command, args, { cwd: REPO, env: ENV, encoding: 'utf8', maxBuffer: 1 << 26 });

// Incremental build + verify, timed from just before the save.
function framesCheck() {
  const started = now();
  const built = run('ninja', ['-C', NATIVE, 'frames']);
  if (built.status !== 0) return { verdict: 'build-error', caught: true, ms: now() - started };
  const verified = run(join(NATIVE, 'frames'), ['verify']);
  const ms = now() - started;
  const report = JSON.parse(verified.stdout);
  return {
    exit: verified.status,
    caught: verified.status === 1,
    ms,
    failing: report.fixtures.filter((f) => f.status === 'fail').map((f) => `${f.id}:${f.changed}px`),
  };
}

function bxTest() {
  const started = now();
  const result = run(join(REPO, 'tools/bx'), ['test']);
  const output = (result.stdout + result.stderr).split('\n');
  return {
    exit: result.status,
    caught: result.status !== 0,
    ms: now() - started,
    failing: output.filter((line) => /FAIL|failed/.test(line) && !/0 failed/.test(line)).slice(0, 3),
  };
}

if (!gitClean()) {
  console.log(JSON.stringify({ status: 'infra', error: `${files.join(' and ')} have uncommitted changes` }));
  process.exit(4);
}
for (const m of MUTANTS) {
  if (originals.get(m.file).toString().split(m.from).length !== 2) {
    console.log(JSON.stringify({ status: 'infra', error: `mutant ${m.id} doesn't match exactly once in ${m.file}` }));
    process.exit(4);
  }
}

const results = [];
try {
  const baseline = { frames: framesCheck(), bxTest: bxTest() };
  if (baseline.frames.caught || baseline.bxTest.caught) throw new Error('the unmutated sources already fail a check');
  for (const m of MUTANTS) {
    const text = originals.get(m.file).toString();
    const started = now();
    writeFileSync(join(REPO, m.file), text.replace(m.from, m.to));
    const frames = framesCheck();
    frames.editToVerdictMs = now() - started;
    const bx = bxTest();
    restore();
    results.push({ id: m.id, file: m.file, covered: m.covered !== false, frames, bxTest: bx });
    console.log(JSON.stringify({ id: m.id, frames: { caught: frames.caught, ms: Math.round(frames.editToVerdictMs) },
      bxTest: { caught: bx.caught, ms: Math.round(bx.ms) } }));
  }
} finally {
  restore();
}
// Leave the build matching the restored sources.
const after = framesCheck();
const covered = results.filter((r) => r.covered);
const p = (values, q) => [...values].sort((a, b) => a - b)[Math.min(values.length - 1, Math.ceil(q * values.length) - 1)];
const summary = {
  covered: covered.length,
  framesCaught: covered.filter((r) => r.frames.caught).length,
  bxTestCaught: covered.filter((r) => r.bxTest.caught).length,
  controlCaughtByFrames: results.filter((r) => !r.covered).some((r) => r.frames.caught),
  controlCaughtByBxTest: results.filter((r) => !r.covered).some((r) => r.bxTest.caught),
  framesEditToVerdictMs: { p50: Math.round(p(results.map((r) => r.frames.editToVerdictMs), 0.5)),
    max: Math.round(Math.max(...results.map((r) => r.frames.editToVerdictMs))) },
  bxTestMs: { p50: Math.round(p(results.map((r) => r.bxTest.ms), 0.5)), max: Math.round(Math.max(...results.map((r) => r.bxTest.ms))) },
  cleanAfter: !after.caught,
  sourcesRestored: gitClean(),
};
mkdirSync(join(REPO, 'host/build/live'), { recursive: true });
writeFileSync(join(REPO, 'host/build/live/mutants.json'), JSON.stringify({ schemaVersion: 1, summary, results }, null, 2) + '\n');
console.log(JSON.stringify(summary));
process.exit(summary.sourcesRestored && summary.cleanAfter ? 0 : 1);
