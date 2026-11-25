#!/usr/bin/env node
// Live preview of the status screen: rebuilds when a renderer source is saved and pushes the
// new frames to the page.
//
//   node host/live/serve.mjs [--mode wasm|native|bx-preview] [--port 8790]
//
//   wasm        incremental Emscripten build; the page runs the renderer (attitude sliders work)
//   native      incremental native build; `frames dump` writes the frames the page shows
//   bx-preview  `tools/bx preview` on every save: today's workflow, automated, for comparison
//
// It watches the files the compiler reports as dependencies (`ninja -t deps`), so a header in
// the shared esp32-tools components counts too. Listens on 127.0.0.1 only, and refuses requests
// whose Host or Origin isn't this server. Ctrl+C stops it.

import { spawn } from 'node:child_process';
import { EventEmitter } from 'node:events';
import { existsSync, mkdirSync, watch } from 'node:fs';
import { copyFile, readdir, readFile, rm } from 'node:fs/promises';
import { createServer } from 'node:http';
import { dirname, join, resolve, sep } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';

const HERE = dirname(fileURLToPath(import.meta.url));
const REPO = resolve(HERE, '../..');
const OUT = join(REPO, 'host/build/live');
const NATIVE = join(REPO, 'host/build/live-native');
const WASM = join(REPO, 'host/build/live-wasm');
const MODES = ['wasm', 'native', 'bx-preview'];
const DEBOUNCE_MS = 10;
const KEEP_GENERATIONS = 4;
const LOG_LIMIT = 16_000;

// Epoch milliseconds with sub-millisecond resolution, comparable with the page's clock.
export const now = () => performance.timeOrigin + performance.now();

const SHARED_COMP = [join(REPO, '../../esp32-tools/tools/components'), join(REPO, '../tools/components'),
  join(REPO, 'components')].map((dir) => resolve(dir)).find((dir) => existsSync(dir));

const ENV = {
  ...process.env,
  TMPDIR: join(REPO, 'host/build/tmp'),
  EM_CACHE: process.env.EM_CACHE ?? join(REPO, 'host/build/em-cache'),
  NINJA_STATUS: '',
};

const TYPES = {
  '.html': 'text/html; charset=utf-8', '.js': 'text/javascript', '.mjs': 'text/javascript',
  '.css': 'text/css', '.json': 'application/json', '.wasm': 'application/wasm',
  '.png': 'image/png', '.rgb565': 'application/octet-stream',
};
const PAGE_FILES = new Set(['index.html', 'app.js', 'worker.js', 'frame.js', 'style.css']);

// Runs a command, collecting the end of its output. Resolves with the exit code; never rejects.
function run(command, args, { limit = LOG_LIMIT } = {}) {
  return new Promise((done) => {
    let log = '';
    const child = spawn(command, args, { cwd: REPO, env: ENV });
    const collect = (chunk) => { log = (log + chunk).slice(-limit); };
    child.stdout?.on('data', collect);
    child.stderr?.on('data', collect);
    child.on('error', (error) => done({ code: 127, log: log + String(error) }));
    child.on('close', (code) => done({ code: code ?? 1, log }));
  });
}

const clean = (log) => log.replace(/\x1b\[[0-9;?]*[ -/]*[@-~]/g, '').replace(/\r/g, '');

async function configure(dir, emscripten) {
  if (existsSync(join(dir, 'build.ninja'))) return;
  const args = ['-S', join(REPO, 'host'), '-B', dir, '-G', 'Ninja'];
  const result = emscripten ? await run('emcmake', ['cmake', ...args]) : await run('cmake', args);
  if (result.code) throw new Error(`configuring ${dir} failed:\n${clean(result.log).slice(-4000)}`);
}

// The repository and shared component files a build directory's objects were compiled from.
async function dependencies(dir) {
  const result = await run('ninja', ['-C', dir, '-t', 'deps'], { limit: Infinity });
  const files = new Set();
  for (const line of result.log.split('\n')) {
    const file = line.trim();
    if (!line.startsWith(' ') || !file) continue;
    const path = resolve(dir, file);
    const inRepo = path.startsWith(REPO + sep) && !path.startsWith(join(REPO, 'host/build') + sep);
    if (inRepo || (SHARED_COMP && path.startsWith(SHARED_COMP + sep))) files.add(path);
  }
  return files;
}

const BUILDERS = {
  async wasm(dir) {
    const built = await run('ninja', ['-C', WASM, 'preview_wasm']);
    if (built.code) return built;
    await copyFile(join(WASM, 'preview.mjs'), join(dir, 'preview.mjs'));
    await copyFile(join(WASM, 'preview.wasm'), join(dir, 'preview.wasm'));
    return { code: 0, log: built.log, files: ['preview.mjs', 'preview.wasm'] };
  },
  async native(dir) {
    const built = await run('ninja', ['-C', NATIVE, 'frames']);
    if (built.code) return built;
    const dumped = await run(join(NATIVE, 'frames'), ['dump', '--out', dir]);
    return dumped.code ? { code: dumped.code, log: built.log + dumped.log } : { code: 0, log: built.log, files: ['manifest.json'] };
  },
  async 'bx-preview'(dir) {
    const built = await run(join(REPO, 'tools/bx'), ['preview']);
    if (built.code) return built;
    await copyFile(join(REPO, 'host/build/preview.png'), join(dir, 'preview.png'));
    return { code: 0, log: built.log, files: ['preview.png'] };
  },
};

export async function startServer({ mode = 'wasm', port = 8790, quiet = false } = {}) {
  if (!MODES.includes(mode)) throw new Error(`unknown mode ${mode}; use ${MODES.join(', ')}`);
  const say = quiet ? () => {} : (line) => console.log(line);
  const events = new EventEmitter();
  mkdirSync(OUT, { recursive: true });
  mkdirSync(ENV.TMPDIR, { recursive: true });
  for (const entry of await readdir(OUT)) {
    if (/^gen-\d+$/.test(entry)) await rm(join(OUT, entry), { recursive: true, force: true });
  }

  // The native build lists the fixtures, and gives bx-preview mode its dependency list.
  await configure(NATIVE, false);
  const first = await run('ninja', ['-C', NATIVE, 'frames']);
  if (first.code) throw new Error(`native build failed:\n${clean(first.log).slice(-4000)}`);
  const listed = await run(join(NATIVE, 'frames'), ['hashes']);
  const fixtures = JSON.parse(listed.log).fixtures.map((f) => f.id);
  if (mode === 'wasm') await configure(WASM, true);

  let generation = 0;
  // For pages that connect later: the last good frames, then the failure since, if any.
  let lastFrames = null;
  let lastError = null;
  let building = false;
  let pending = null;  // changes waiting for a build
  let timer = null;
  const clients = new Set();
  const watchers = new Map();  // directory -> FSWatcher
  let watched = new Set();

  const broadcast = (message) => {
    if (message.type === 'frames') {
      lastFrames = message;
      lastError = null;
    } else {
      lastError = message;
    }
    const data = `data: ${JSON.stringify(message)}\n\n`;
    for (const client of clients) client.write(data);
  };

  const changed = (path) => {
    const at = now();
    pending ??= { changedAt: at, files: new Set() };
    pending.files.add(path);
    events.emit('change', { path, at });
    clearTimeout(timer);
    if (!building) timer = setTimeout(rebuild, DEBOUNCE_MS);
  };

  const watchFiles = (files) => {
    watched = files;
    for (const dir of new Set([...files].map((file) => dirname(file)))) {
      if (watchers.has(dir)) continue;
      watchers.set(dir, watch(dir, (type, name) => {
        const path = name ? join(dir, name.toString()) : null;
        if (!path || watched.has(path)) changed(path ?? dir);
      }));
    }
  };

  async function rebuild() {
    if (building || !pending) return;
    const batch = pending;
    pending = null;
    building = true;
    const gen = ++generation;
    const dir = join(OUT, `gen-${gen}`);
    mkdirSync(dir, { recursive: true });
    const startedAt = now();
    let result;
    try {
      result = await BUILDERS[mode](dir);
    } catch (error) {
      result = { code: 1, log: String(error?.stack ?? error) };
    }
    const finishedAt = now();
    const timing = { changedAt: batch.changedAt, startedAt, finishedAt };
    if (result.code === 0) {
      broadcast({ type: 'frames', mode, gen, files: result.files, fixtures, ...timing });
      say(`gen ${gen}: built in ${(finishedAt - startedAt).toFixed(0)} ms`);
      const keep = new Set(Array.from({ length: KEEP_GENERATIONS }, (_, i) => `gen-${gen - i}`));
      for (const entry of await readdir(OUT)) {
        if (/^gen-\d+$/.test(entry) && !keep.has(entry)) await rm(join(OUT, entry), { recursive: true, force: true });
      }
      if (mode !== 'bx-preview') watchFiles(await dependencies(mode === 'wasm' ? WASM : NATIVE));
    } else {
      await rm(dir, { recursive: true, force: true });
      broadcast({ type: 'error', mode, gen, log: clean(result.log).slice(-LOG_LIMIT), ...timing });
      say(`gen ${gen}: build failed, the page keeps the last good frames`);
    }
    events.emit('build', { ok: result.code === 0, mode, gen, ...timing });
    building = false;
    if (pending) timer = setTimeout(rebuild, DEBOUNCE_MS);
  }

  const deps = await dependencies(NATIVE);
  if (mode === 'bx-preview') deps.add(join(REPO, 'host/preview.cpp'));
  pending = { changedAt: now(), files: new Set() };
  await rebuild();
  watchFiles(mode === 'wasm' ? await dependencies(WASM) : deps);

  let allowedHosts = new Set();  // filled in once the port is known
  const server = createServer(async (req, res) => {
    const origin = `http://${req.headers.host}`;
    if (!allowedHosts.has(req.headers.host ?? '') ||
        (req.method !== 'GET' && req.method !== 'HEAD' && req.headers.origin !== origin)) {
      res.writeHead(403).end();
      return;
    }
    const url = new URL(req.url, origin);
    const send = (status, type, body) => {
      res.writeHead(status, { 'Content-Type': type, 'Cache-Control': 'no-store', 'X-Content-Type-Options': 'nosniff' });
      res.end(body);
    };
    try {
      if (req.method === 'GET' && url.pathname === '/events') {
        res.writeHead(200, { 'Content-Type': 'text/event-stream', 'Cache-Control': 'no-store', Connection: 'keep-alive' });
        res.write('retry: 500\n\n');
        for (const message of [lastFrames, lastError]) {
          if (message) res.write(`data: ${JSON.stringify(message)}\n\n`);
        }
        clients.add(res);
        req.on('close', () => clients.delete(res));
        return;
      }
      if (req.method === 'GET' && url.pathname === '/api/state') {
        send(200, TYPES['.json'], JSON.stringify({ mode, fixtures, gen: generation, watched: watched.size }));
        return;
      }
      if (req.method === 'POST' && url.pathname === '/api/paint') {
        let body = '';
        for await (const chunk of req) {
          body += chunk;
          if (body.length > 65_536) throw new Error('paint report too large');
        }
        const report = JSON.parse(body);
        events.emit('paint', report);
        if (report.changedAt) say(`gen ${report.gen}: painted ${(report.paintedAt - report.changedAt).toFixed(0)} ms after the save`);
        send(204, 'text/plain', '');
        return;
      }
      let file = null;
      const page = url.pathname === '/' ? 'index.html' : url.pathname.slice(1);
      const golden = url.pathname.match(/^\/goldens\/([a-z0-9-]+\.rgb565)$/);
      const generated = url.pathname.match(/^\/gen\/(\d+)\/([a-z0-9_-]+\.(?:mjs|wasm|json|rgb565|png))$/);
      if (PAGE_FILES.has(page)) file = join(HERE, page);
      else if (golden) file = join(REPO, 'host/goldens', golden[1]);
      else if (generated) file = join(OUT, `gen-${generated[1]}`, generated[2]);
      if (req.method !== 'GET' || !file) {
        send(404, 'text/plain', 'not found');
        return;
      }
      const body = await readFile(file).catch(() => null);
      if (!body) send(404, 'text/plain', 'not found');
      else send(200, TYPES[file.slice(file.lastIndexOf('.'))] ?? 'application/octet-stream', body);
    } catch (error) {
      send(400, 'text/plain', String(error?.message ?? error));
    }
  });
  await new Promise((ready, fail) => {
    server.once('error', fail);
    server.listen(port, '127.0.0.1', ready);
  });
  const actualPort = server.address().port;
  allowedHosts = new Set([`127.0.0.1:${actualPort}`, `localhost:${actualPort}`]);
  const url = `http://127.0.0.1:${actualPort}/`;
  say(`BalaX live preview (${mode}): ${url}  watching ${watched.size} files`);

  const close = async () => {
    clearTimeout(timer);
    for (const watcher of watchers.values()) watcher.close();
    for (const client of clients) client.end();
    server.closeAllConnections?.();
    await new Promise((done) => server.close(done));
    for (const entry of await readdir(OUT)) {
      if (/^gen-\d+$/.test(entry)) await rm(join(OUT, entry), { recursive: true, force: true });
    }
  };
  return { url, port: actualPort, events, close, isBuilding: () => building || pending !== null };
}

if (process.argv[1] && import.meta.url === pathToFileURL(resolve(process.argv[1])).href) {
  const arg = (name, fallback) => {
    const i = process.argv.indexOf(name);
    return i > 0 && i + 1 < process.argv.length ? process.argv[i + 1] : fallback;
  };
  try {
    const live = await startServer({ mode: arg('--mode', 'wasm'), port: Number(arg('--port', 8790)) });
    const stop = async () => {
      await live.close();
      process.exit(0);
    };
    process.on('SIGINT', stop);
    process.on('SIGTERM', stop);
  } catch (error) {
    console.error(String(error?.message ?? error));
    process.exit(1);
  }
}
