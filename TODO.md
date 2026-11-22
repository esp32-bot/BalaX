# BalaX & esp32-bot Master TODO

## 1. Organization, Disk Layout & Modularity (`esp32-bot` & `g-AI-org`)
- [x] Transfer `BalaX` and `JoyX` to `https://github.com/esp32-bot`
- [x] Remove old Arduino / PlatformIO files from `BalaX` `main` branch (preserved on `arduino` branch)
- [x] Configure Zed for ESP-IDF C++23 (`.clangd`, `.zed/settings.json` with `esp-clangd`, `.zed/tasks.json`)
- [x] Add local dev dashboard (`tools/bx gui`)
- [x] Move local repos on disk into `~/gh/esp32-bot/` and `~/gh/g-ai/` (with symlinks in `~/gh/Felipegalind0/`):
  - [x] `~/gh/esp32-bot/balax` (`https://github.com/esp32-bot/BalaX`)
  - [x] `~/gh/esp32-bot/joyx` (`https://github.com/esp32-bot/JoyX`)
  - [x] `~/gh/esp32-bot/balax-remote` (`https://github.com/esp32-bot/balax-remote`)
  - [x] `~/gh/esp32-bot/tools` (`https://github.com/esp32-bot/tools`)
  - [x] `~/gh/g-ai/rc` (`https://github.com/g-AI-org/rc`)
  - [x] `~/gh/g-ai/gdog-remote` (`https://github.com/g-AI-org/gdog-remote`)
- [x] Maximize modularity across `@esp32-bot` and `@g-AI-org`:
  - [x] Extract shared ESP-IDF components (`m5stickc_plus`, `gfx`, `settings`, `console_commands`, `ota`) and CLI tools into `esp32-bot/tools` (`~/gh/esp32-bot/tools/components`), leaving only `balac` and balancing logic in `balax`
  - [x] Create shared `@g-ai/rc` (`g-AI-org/rc`) low-latency remote control, link diagnostics, joystick math, and live tuning library and wire it into `esp32-bot/balax-remote`, `g-AI-org/gdog-remote`, and `0SFS/rc`

## 2. Zero-Install Browser Flasher, LAN OTA & Firmware Hosting (`esp32-bot.github.io`)
- [x] Create static WebSerial flasher & USB console tool (`web/index.html`) importing shared `https://esp32-bot.github.io/tools/flasher.js`
- [x] Add `tools/bx dist [stable|dev]` (`tools/dist.py`) to package `stable` and `dev` firmware channels (`web/fw/{stable|dev}/manifest.json` + binaries)
- [x] Add Wi-Fi HTTPS OTA update component (`ota [stable|dev|url|version]`) in `esp32-bot/tools/components/ota`
- [x] Add cryptographic LAN OTA flashing (`tools/bx lan-flash`), LAN log monitoring (`tools/bx lan-monitor`), and LAN command runner (`tools/bx lan-cmd`) using PBKDF2-HMAC-SHA256 + 256-bit hardware TRNG challenge nonces

## 3. Developer Setup One-Liner (`esp32-bot/tools`)
- [x] Create one-liner bootstrap installer script (`install.sh` / `https://esp32-bot.github.io/tools/install.sh`) that clones `tools`, `balax`, `joyx`, and `balax-remote` into `~/gh/esp32-bot/` and launches the BalaX Studio GUI

## 4. Web Remote PWA & 0SFS-Style Babylon.js Live 3D Visualizer (`esp32-bot/balax-remote`)
- [x] Scaffold installable React + Vite PWA using `@g-ai/rc` with floating joystick, live PID gain sliders, and 100 Hz telemetry strip chart
- [x] Add 0SFS-style Babylon.js live 3D M5StickC Plus + BalaC visualizer (`src/BalaXBabylonScene.tsx`) driven by real-time 100 Hz pitch, wheel velocities, and robot state
- [x] Add compressed binary (`BXZ1` delta + ZigZag + LEB128 varint) zero-drop telemetry streaming (`main/telemetry_stream.cpp` + `GET /api/stream?ack=<tick>`) with RAM queue + 64 KB Flash spillover (`0x3e0000`) that frees memory immediately on ACK
- [x] Add localhost telemetry bridge & `.jsonl.gz` flight recorder (`telemetryBridgePlugin.ts` on `http://localhost:5180/api/bridge/{status,trace,cmd}`) so agents and humans can query full session telemetry locally
- [ ] Log compressed telemetry onto the ESP32-CAM SD card when an ESP32-CAM hat/companion is attached (for offline outdoor runs without LAN)
- [ ] Future links: WebBluetooth, ESP-NOW long-range relay, ESP32-CAM stabilized stream

## 5. BalaX Firmware Tasks
- [x] Turn softAP off by default when connected to a Wi-Fi network to save power (`main/wifi.cpp`), and automatically re-enable softAP when disconnected
- [x] Set local robot volume override (`CONFIG_BALAX_DEFAULT_VOLUME=60` in gitignored `sdkconfig.local`) while keeping repo default at `100`
- [ ] Evaluate unported features from Arduino branch if still needed: Madgwick filter, 6-side IMU calibration, JoyC polar mode
- [ ] Test ESP-NOW link with a real JoyX remote while driving
