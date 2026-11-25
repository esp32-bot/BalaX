# BalaX camera and sleep handoff

The firmware sleep fix and Camera workspace changes are local. The next step is hardware verification after an authorized firmware update. Preserve the existing uncommitted work across all checkouts.

## StickC automatic sleep

- The old idle policy could count a moving, disarmed robot as idle. Camera status requests do not issue a BalaX sleep command. Firmware logs distinguish idle timeout, thermal protection, and Wi-Fi sleep switching to USB sleep.
- The 100 Hz loop now tracks continuous disarmed stillness with the existing IMU fusion thresholds and a current-sample check. Motion, arming, and failed IMU reads break the interval, including brief movement between status polls. Stale IMU data blocks automatic sleep. Power management rechecks fresh telemetry before entering sleep.
- Ordinary automatic sleep requires READY and no active robot telemetry, remote, or program workload for the configured stationary idle interval. Thermal protection retains its three consecutive hot readings and requires disarmed stillness throughout. Explicit power commands retain their existing behavior. See [Hardware](HARDWARE.md).
- `./tools/bx test` passed 47 C++ and 12 Python tests. `./tools/bx build` produced `build/balax.bin`. After an authorized update, test with FPV active, including brief movement near the timeout, followed by a full stationary idle interval. Verify that an armed robot with stopped motors stays awake. The firmware was not flashed or verified on hardware during this session.
- If a reset persists, distinguish it from sleep before changing the policy again. Camera Auto can fall back to USB and currently selects the first serial port without identifying the board. Opening the StickC port can reset it. The bridge can also infer USB Sleep from missed Wi-Fi probes and any available serial device. Those are separate, unverified failure paths.

## Camera and monitor tabs

- In `~/gh/esp32-bot/balax-remote`, Camera uses shared `@g-ai/ui` controls with help, sliders, reset, and saved settings. **Sparse threshold** is under **Codec** and accepts zero. **Power** has an independent stream CPU setting. Apply sends changed fields only, failures leave changes pending, and Discard restores applied values. Workspace v9 layouts migrate to v10 with Camera and 3D View remapped.
- CPU, RAM, and Flash show BalaX and ESP32-CAM independently. The Remote shares a passive camera status poll across visible Camera and monitor tabs. Older firmware, sleep, and stale readings can leave metrics unavailable. The camera firmware in `~/gh/esp32-bot/balax-cam` extends `/api/status` with core load, DRAM and PSRAM capacity and low-water marks, and flash partition metadata. The clock reading is an active limit.
- Remote passed 17 tests, browser TypeScript checking, production build, and mocked Chrome checks at desktop and phone widths. The camera host suite and ESP-IDF build passed. The 974,400-byte camera image fits `ota_0` and exceeds the factory rescue slot. Live camera metrics still require an authorized update and hardware checks.

```sh
cd ~/gh/esp32-bot/balax-remote && npm run dev
```

## Shared dependencies and follow-ups

- The shared tools checkout moved to `~/gh/esp32-tools/tools`. BalaX build and host checks use it. The tools GitHub Pages redeploy remains pending.
- Build `~/gh/g-ai/ui` before `~/gh/g-ai/3d-ui`. Shared settings controls and their UI rules live in the former. Windowing and logs live in the latter. The 0SFS migration covers Fly, drone-camera, Replay, and RC. Fly typecheck and 104 targeted tests passed. Native Vite resolution needed an ignored local dependency link in the sibling FOSS Earth app.
- The shared UI extraction passed 126 tests in Claude's run. The 3d-ui suite had six pre-existing Node 26 localStorage failures reproduced on its baseline. Its package consumer plan and dev installer manifests still need the new UI dependency.
- Deferred camera work includes diagnosing the grid artifact, measuring the OV2640 quality and frame-rate tradeoffs, improving UART/IMU synchronization, optical flow, and digital PTZ with a sensor crop map. Read the [silicon references](silicon/README.md) before work on that pipeline.
