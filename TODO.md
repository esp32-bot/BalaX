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
  - [x] Extract shared ESP-IDF components (`m5stickc_plus`, `gfx`, `settings`, `console_commands`) and CLI tools into `esp32-bot/tools` (`~/gh/esp32-bot/tools/components`), leaving only `balac` and balancing logic in `balax`
  - [x] Create shared `@g-ai/rc` (`g-AI-org/rc`) low-latency remote control, link diagnostics, joystick math, and live tuning library and wire it into `esp32-bot/balax-remote`, `g-AI-org/gdog-remote`, and `0SFS/rc`

## 2. Zero-Install Browser Flasher & Firmware Hosting (`esp32-bot.github.io`)
- [x] Create static WebSerial flasher & USB console tool (`web/index.html`) so family/cousins can flash and configure robots from Chrome without installing Zed or ESP-IDF
- [x] Add `tools/bx dist [stable|dev]` (`tools/dist.py`) to package `stable` and `dev` firmware channels (`web/fw/{stable|dev}/manifest.json` + binaries)
- [ ] Add Wi-Fi OTA update component in firmware so robots check `esp32-bot.github.io/balax/fw/{stable|dev}/manifest.json` for new firmware

## 3. Developer Setup One-Liner (`esp32-bot/tools`)
- [x] Create one-liner bootstrap installer script (`install.sh` / `https://esp32-bot.github.io/tools/install.sh`) that clones `tools`, `balax`, `joyx`, and `balax-remote` into `~/gh/esp32-bot/` and launches the BalaX Studio GUI

## 4. Web Remote PWA (`esp32-bot/balax-remote`, powered by `@g-ai/rc`)
- [x] Scaffold installable React + Vite PWA using `@g-ai/rc` with floating camera-surface joystick, 50 Hz control loop, live RTT/Hz diagnostics, 60 FPS telemetry strip chart, 3D attitude indicator, and live PID gain sliders
- [ ] Add WebSocket / UDP telemetry & control server endpoint in BalaX firmware (`main/wifi.cpp`) to stream 100 Hz telemetry to `balax-remote`
- [ ] Future links: WebBluetooth, ESP-NOW long-range relay, ESP32-CAM stabilized stream

## 5. BalaX Firmware Tasks
- [ ] Tune balance gains on the floor (currently balances for a while, then falls)
- [ ] Evaluate unported features from Arduino branch if still needed: Madgwick filter, 6-side IMU calibration, JoyC polar mode
- [ ] Optimize Wi-Fi power save / CPU frequency when idle while keeping ESP-NOW / Wi-Fi latency low
- [ ] Test ESP-NOW link with a real JoyX remote while driving
