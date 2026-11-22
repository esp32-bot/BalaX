# Agent instructions

This repository is the BalaX self-balancing robot firmware (`esp32-bot/balax`), paired with the sibling web remote and 3D telemetry app in `~/gh/esp32-bot/balax-remote` and shared UI/controls in `~/gh/g-ai/rc`.

## Silicon & Hardware Reference (Read Before Low-Level Work)

- Consult [`docs/silicon/README.md`](docs/silicon/README.md) before modifying pin maps, DMA/SRAM/PSRAM allocations, clocks, I²C/SPI/I²S/SDMMC peripherals, power rails, thermal limits, or camera code:
  - [`docs/silicon/M5STICKC_PLUS.md`](docs/silicon/M5STICKC_PLUS.md) — `ESP32-PICO-D4` (swapped flash pins, SRAM/DMA map, errata `[GPIO-3.11]`/`ADC2`/cache-disable), `AXP192`, `MPU6886`, `ST7789V2`, `BalaC` (`STM32F030F4P6` + `L9110S`).
  - [`docs/silicon/ESP32_CAM.md`](docs/silicon/ESP32_CAM.md) — AI-Thinker `ESP32-S`, 4 MB PSRAM (`GPIO16` CS#, `[CPU-3.9]`), mandatory 1-bit SDMMC (`GPIO2/14/15`), `GPIO32` active-LOW camera power gate, I²S0 parallel DMA, brownout/power & UART RPC wiring.
  - [`docs/silicon/OV2640.md`](docs/silicon/OV2640.md) — Dual-bank SCCB (`0x30`/`0xFF`), hardware JPEG DSP (`QS`), `XCLK`/`PCLK` DMA ceiling, rolling shutter & robotics tuning.

## Thin-Robot Offload & Continuous Optimization Policy

- **Only put on the robot what MUST run on the robot:** deterministic 100 Hz balance/IMU control & safety interlocks on Core 1, minimal hardware/sensor drivers, minimal authenticated LAN/ESP-NOW transport, tiny OTA receiver, and (when added) a sandboxed RAM mini-program VM.
- **Offload everything else to the companion remote app (`balax-remote` + `@g-ai/rc`)** running on a smartphone, tablet, or laptop:
  - **GitHub Releases OTA** belongs in `balax-remote` (browser HTTPS fetch + SHA-256 verify + `IndexedDB` cache + `zlib` compression → pushed through the exact same local LAN OTA endpoint as every other firmware source, keeping TLS/`mbedtls` off the robot).
  - **JoyX** acts both as a physical joystick and as an **ESP-NOW ↔ USB/Wi-Fi companion bridge** for `balax-remote` on a phone or laptop — do not duplicate heavy UI, networking, or update logic on JoyX or BalaX.
  - **Calibration solvers, 6-DOF fusion/HIL replay, offline evaluation, telemetry charts, and program authoring/storage** live in browser TypeScript/WASM (`balax-remote` + `@g-ai/rc`), never in PC-only Python scripts or bloated robot firmware.
- **Actively optimize CPU, RAM, Flash, and thermal efficiency** using our monitoring & profiling tools:
  - Query live metrics via `balax lan-cmd "probe ..."` (`cpu_mhz`, `cpu0_load`, `cpu1_load`, `ram_free_kb`, `ram_min_kb`, `pmic_c`, `imu_c`, `battery_ma`, `wifi_ps`, `tick_ms`, `overruns`).
  - Inspect flash/RAM contributions with `idf.py size-components` / `esp_idf_size --archives build/balax.map` and verify in the `balax-remote` **CPU**, **RAM**, and **Flash** monitor tabs.
  - Respect the sealed M5StickC Plus thermal budget (80 MHz CPU, `WIFI_PS_MIN_MODEM`, `+13 dBm` TX cap, 200 Hz RTOS tick, LCD skip at `brightness == 0`, pre-flash battery `≥ 30%` & temp gates).

## Dev servers & user testing (0SFS convention)

- **Never leave a background dev server running as the handoff.** A backgrounded dev server started by an agent tool call is tied to that tool turn and cuts out when a new message arrives.
- Stop every temporary server or child process you start before returning, unless the user explicitly asks you to keep it running; leave the user's servers alone.
- For user testing of `balax-remote`, always give the user the exact command below to run in their own terminal so it persists across turns:

```sh
cd ~/gh/esp32-bot/balax-remote && npm run dev
```

## Browser-first TS/WASM tooling (no PC-only Python workflows)

- **Do as much as possible in TypeScript/WASM inside the browser web app (`balax-remote` + `@g-ai/rc`)**, not in PC-only Python scripts.
- Calibration wizards (6-face accel ellipsoid, CG lever-arm offset, IMU zeroing), HIL maneuver recording/replay, offline evaluation scorecards, GitHub release OTA fetching, and telemetry diagnostics MUST live in TypeScript/WASM with a browser UI so they work on phones, tablets, or laptops without a PC CLI.
- Keep Python strictly for ESP-IDF host build/flash tooling (`tools/bx`, `idf.py`, `tools/lan.py`) where the toolchain requires it.
- Browser-recorded HIL captures should be downloadable as JSON and, when running on the local dev bridge, mirrored to `balax-remote/recordings/hil/` so both the browser UI and LLM agent can analyze the same captures.

## Preventing Zed / ACP freezes

- **Keep tool stdout/stderr tiny and ANSI-free.** Streaming large Ninja/CMake/esptool logs, `\r` progress bars, raw serial monitors, or binary payloads through ACP stalls Zed's Agent Panel UI (especially in longer conversations).
- `./tools/bx build` and `./tools/bx lan-flash` automatically detect non-interactive shells (`! -t 1` or `TERM=dumb`) and redirect `idf.py build` output to `build/bx-build.log`, printing only a 1-line summary on success or the last 40 ANSI-stripped lines on failure.
- For any other noisy command (`npm run build`, `curl`, `git diff`, `lan-monitor`), redirect verbose output to a temp/build log file or bound it with `tail`/`head`, and never print raw binary telemetry to stdout.
- Keep conversations concise; hand off to a fresh thread with a short markdown summary before a thread grows huge.

## Shared rules

- **Modular, reusable 3D UI & RC libraries:** Shared 3D UI components (floating log `GameLogOverlay` / `createGameLog`, docked windows/tabs `WorkspaceDockSlot` / `TabStrip` / `DockPanel`, settings, theme, diagnostics) live in `~/gh/g-ai/3d-ui` (`@g-ai/3d-ui`). Shared RC/telemetry, 3D camera gestures (`attachOrbitCameraInput` + `createCameraGlide` inertial glide), 6-DOF IMU physics/calibration solvers (`imuPhysics.ts`, `imuCalibration.ts`), artifact store (`artifactStore.ts`), and BXP1 compiler live in `~/gh/g-ai/rc` (`@g-ai/rc`). Reuse them rather than duplicating per app.
- **Camera inertia:** Every user-steered 3D camera uses `@g-ai/rc`'s `attachOrbitCameraInput` with inertial glide (`createCameraGlide`).
- **Tabbed UI & non-blocking notices:** New UI panels belong in the app's tabbed workspace, and transient/warning/error notices belong in `@g-ai/3d-ui`'s `<GameLogOverlay />` (never giant blocking top banners).
- **Always use `balax.local` / `balax-remote.local`** (never hardcode LAN IPs) in docs, defaults, and user instructions.
- **Firmware & host checks:** Run `./tools/bx test` to run host C++ and Python unit tests, `lan-flash` for authenticated LAN OTA updates to `balax.local` (with pre-flight battery/thermal checks), and `balax lan-cmd "<cmd>"` for authenticated LAN console commands.
