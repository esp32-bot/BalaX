# Status screen live preview and frame checks

The status screen renderer (`main/status_view.cpp`) is plain C++, so the laptop can run the same code the robot does. These tools build it incrementally, show it in a browser as you save, and check it against reference frames.

## Check the screen

```sh
./tools/bx frames                 # verify every fixture against host/goldens/
./tools/bx frames write-goldens   # accept the current rendering as the new reference
```

The fixtures, the made-up robot states in `host/fixtures.h`, are the same ones `bx preview` draws. The reference frames in `host/goldens/` are raw little-endian RGB565, 135 × 240, rendered strip by strip the way the robot's LCD task draws them. `frames` prints one JSON line naming any pixel that changed, with the first one and the rectangle around them, and exits 0 on a pass, 1 on a difference, 2 on bad arguments and 4 when a reference frame is missing. `frames verify --negative-control` flips one pixel of every render first, to show the comparison catches it.

## Live preview

```sh
./tools/bx live            # or: ./tools/bx live native
```

Open <http://127.0.0.1:8790/>. Each save of a renderer source, including headers in the shared `esp32-tools` components, rebuilds and repaints every fixture. The page marks each fixture as matching the reference or not, can overlay the differing pixels, and keeps the last good frames on screen with the compiler output when a build fails.

| Mode | Rebuild | Page |
| --- | --- | --- |
| `wasm` (default) | Emscripten, `host/build/live-wasm` | runs the renderer in a worker; pitch and roll sliders redraw without a rebuild |
| `native` | clang, `host/build/live-native` | shows the frames `frames dump` writes |
| `bx-preview` | `tools/bx preview` | today's workflow, automated, for comparison |

The server listens on 127.0.0.1 only, and refuses requests whose `Host` or `Origin` isn't its own. Wasm mode needs Emscripten (`brew install emscripten`). The first build fills `host/build/em-cache` in about 20 seconds.

## Measurements

Each script restores any source it edits byte for byte, and refuses to start if that file has uncommitted changes. Results go to `host/build/live/`.

| Script | Measures |
| --- | --- |
| `node host/live/bench.mjs` | save-to-paint in each mode: 32 alternating edits each, timed to headless Chrome's paint, checking each paint shows the edit |
| `node host/live/mutants.mjs` | seeded renderer bugs: does `bx test` or `frames verify` catch each one, and how long from the save to the verdict |
| `node host/live/parity.mjs` | the Wasm build renders exactly what the native build does: fixtures byte for byte, 2,113 attitudes by hash |
| `host/live/idf-floor.sh` | ESP-IDF's rebuild time after the same kind of edit, without flashing |
