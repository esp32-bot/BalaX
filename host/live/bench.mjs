#!/usr/bin/env node
// Measures save-to-paint for each live preview mode: saves real renderer edits, then waits for
// headless Chrome to report the new frames on screen, and checks they show the edit.
//
//   node host/live/bench.mjs [--modes wasm,native,bx-preview] [--edits 32]
//
// The edits alternate between a color in status_layout.h and the title in status_view.cpp,
// each followed by putting the file back. Both files must be unchanged in git first, and are
// restored byte for byte afterwards, on Ctrl+C too. Writes host/build/live/bench.json.

import { execFileSync, spawn } from 'node:child_process';
import { mkdirSync, readFileSync, rmSync, writeFileSync } from 'node:fs';
import { dirname, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

import { now, startServer } from './serve.mjs';

const REPO = resolve(dirname(fileURLToPath(import.meta.url)), '../..');
const OUT = join(REPO, 'host/build/live');
const CHROME = '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome';
const PAINT_TIMEOUT_MS = 20_000;
const SETTLE_MS = 300;

const EDITS = [
  { file: 'main/status_layout.h', from: 'TITLE = gfx::rgb565(123, 211, 255)', to: 'TITLE = gfx::rgb565(115, 211, 255)' },
  { file: 'main/status_view.cpp', from: 'strip.text(MARGIN, 2, "BalaX", TITLE)', to: 'strip.text(MARGIN, 2, "Balax", TITLE)' },
];

const arg = (name, fallback) => {
  const i = process.argv.indexOf(name);
  return i > 0 && i + 1 < process.argv.length ? process.argv[i + 1] : fallback;
};
const modes = arg('--modes', 'wasm,native,bx-preview').split(',');
const editCount = Number(arg('--edits', 32));
const sleep = (ms) => new Promise((done) => setTimeout(done, ms));

const originals = new Map(EDITS.map(({ file }) => [file, readFileSync(join(REPO, file))]));
const restore = () => {
  for (const [file, bytes] of originals) {
    if (!readFileSync(join(REPO, file)).equals(bytes)) writeFileSync(join(REPO, file), bytes);
  }
};
const gitClean = () => {
  try {
    execFileSync('git', ['diff', '--quiet', '--', ...originals.keys()], { cwd: REPO });
    return true;
  } catch {
    return false;
  }
};
let chrome = null;
const bail = (code) => {
  restore();
  chrome?.kill('SIGKILL');
  process.exit(code);
};
process.on('SIGINT', () => bail(130));
process.on('SIGTERM', () => bail(143));

if (!gitClean()) {
  console.log(JSON.stringify({ status: 'infra', error: `${[...originals.keys()].join(' and ')} have uncommitted changes; the bench edits them` }));
  process.exit(4);
}
for (const edit of EDITS) {
  const text = originals.get(edit.file).toString();
  if (text.split(edit.from).length !== 2) {
    console.log(JSON.stringify({ status: 'infra', error: `can't find the edit in ${edit.file}: ${edit.from}` }));
    process.exit(4);
  }
}

const percentile = (values, p) => {
  const sorted = [...values].sort((a, b) => a - b);
  return sorted[Math.min(sorted.length - 1, Math.ceil(p / 100 * sorted.length) - 1)];
};
const round = (value) => Math.round(value * 10) / 10;
const stats = (values) => ({ p50: round(percentile(values, 50)), p95: round(percentile(values, 95)),
  min: round(Math.min(...values)), max: round(Math.max(...values)) });

function launchChrome(url, mode) {
  const profile = join(OUT, `chrome-profile-${mode}`);
  rmSync(profile, { recursive: true, force: true });
  mkdirSync(join(REPO, 'host/build/tmp'), { recursive: true });
  return spawn(CHROME, [
    '--headless=new', '--no-first-run', '--no-default-browser-check', '--disable-extensions',
    '--disable-component-update', '--disable-background-networking', '--disable-sync', '--disable-default-apps',
    // Even so, Google Chrome updates ~/Library/Application Support/Google/Chrome/Crashpad/settings.dat.
    '--disable-breakpad', '--disable-crash-reporter', `--crash-dumps-dir=${join(profile, 'crashpad')}`, '--mute-audio', '--disable-background-timer-throttling', '--disable-renderer-backgrounding',
    '--disable-backgrounding-occluded-windows', `--user-data-dir=${profile}`, `--disk-cache-dir=${join(profile, 'cache')}`,
    '--window-size=1280,900', url,
  ], { stdio: 'ignore', env: { ...process.env, TMPDIR: join(REPO, 'host/build/tmp') } });
}

async function benchMode(mode) {
  const live = await startServer({ mode, port: 0, quiet: true });
  const paints = [];
  let waiter = null;
  live.events.on('paint', (report) => {
    paints.push(report);
    if (waiter && report.changedAt >= waiter.after) {
      waiter.done(report);
      waiter = null;
    }
  });
  live.events.on('build', (build) => {
    if (!build.ok && waiter && build.changedAt >= waiter.after) {
      waiter.fail(new Error(`build ${build.gen} failed`));
      waiter = null;
    }
  });
  const nextPaint = (after) => new Promise((done, fail) => {
    const timer = setTimeout(() => fail(new Error('no paint in time')), PAINT_TIMEOUT_MS);
    waiter = { after, done: (r) => { clearTimeout(timer); done(r); }, fail: (e) => { clearTimeout(timer); fail(e); } };
  });

  chrome = launchChrome(live.url, mode);
  const first = await nextPaint(0);
  const goldenHashes = JSON.parse(execFileSync(join(REPO, 'host/build/live-native/frames'), ['hashes'], { encoding: 'utf8' }))
    .fixtures.reduce((all, f) => ({ ...all, [f.id]: f.hash }), {});
  const expected = new Map([['0,0', goldenHashes]]);  // edit state -> hashes the page must show
  const applied = [false, false];
  const samples = [];
  const problems = [];
  if (JSON.stringify(first.hashes) !== JSON.stringify(goldenHashes)) problems.push('first paint differs from the reference frames');

  for (let i = 0; i < editCount; i++) {
    while (live.isBuilding()) await sleep(20);
    await sleep(SETTLE_MS);
    const which = Math.floor(i / 2) % EDITS.length;
    const edit = EDITS[which];
    applied[which] = !applied[which];
    const text = originals.get(edit.file).toString();
    const savedAt = now();
    const waiting = nextPaint(savedAt);
    writeFileSync(join(REPO, edit.file), applied[which] ? text.replace(edit.from, edit.to) : text);
    const savedDoneAt = now();
    let report;
    try {
      report = await waiting;
    } catch (error) {
      problems.push(`edit ${i}: ${error.message}`);
      continue;
    }
    const key = applied.map(Number).join(',');
    const hashes = JSON.stringify(report.hashes);
    if (!expected.has(key)) {
      expected.set(key, report.hashes);
      const unchanged = Object.keys(report.hashes).filter((id) => report.hashes[id] === goldenHashes[id]);
      if (unchanged.length) problems.push(`edit ${i}: ${unchanged.join(', ')} didn't change`);
    } else if (hashes !== JSON.stringify(expected.get(key))) {
      problems.push(`edit ${i}: frames differ from the last time the sources were in this state`);
    }
    samples.push({
      edit: i, file: edit.file, applied: applied[which], gen: report.gen,
      saveToPaint: report.paintedAt - savedAt,
      writeMs: savedDoneAt - savedAt,
      detect: report.changedAt - savedAt,
      build: report.finishedAt - report.startedAt,
      deliver: report.paintedAt - report.finishedAt,
    });
  }

  restore();
  while (live.isBuilding()) await sleep(20);
  await sleep(SETTLE_MS);
  while (live.isBuilding()) await sleep(20);
  chrome.kill('SIGTERM');
  await new Promise((done) => chrome.once('exit', done));
  chrome = null;
  await live.close();
  rmSync(join(OUT, `chrome-profile-${mode}`), { recursive: true, force: true });

  const pick = (key) => samples.map((s) => s[key]);
  return {
    mode,
    edits: editCount,
    measured: samples.length,
    correct: problems.length === 0,
    problems,
    saveToPaint: stats(pick('saveToPaint')),
    detect: stats(pick('detect')),
    build: stats(pick('build')),
    deliver: stats(pick('deliver')),
    samples,
  };
}

const results = [];
try {
  for (const mode of modes) {
    const result = await benchMode(mode);
    results.push(result);
    const { samples, ...summary } = result;
    console.log(JSON.stringify({ ...summary, problems: summary.problems.slice(0, 3) }));
  }
} finally {
  restore();
  chrome?.kill('SIGKILL');
}
const clean = gitClean();
mkdirSync(OUT, { recursive: true });
writeFileSync(join(OUT, 'bench.json'), JSON.stringify({ schemaVersion: 1, at: new Date().toISOString(),
  sourcesRestored: clean, results }, null, 2) + '\n');
console.log(JSON.stringify({ sourcesRestored: clean, written: 'host/build/live/bench.json' }));
process.exit(clean && results.every((r) => r.correct) ? 0 : 1);
