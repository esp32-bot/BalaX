# BalaX 3D Telemetry, Camera, & 6-Axis Physics Handoff

**Date**: 2026-11-22  
**Repos Involved**:
1. Firmware: `/Users/felg/gh/Felipegalind0/BalaX` (ESP32-PICO-D4 M5StickC Plus)
2. Remote Web App: `~/gh/esp32-bot/balax-remote` (Vite + React + Babylon.js)
3. Shared UI & Controls: `~/gh/g-ai/rc` (`@g-ai/rc`)

---

## 1. What Caused Zed Freezing & The Robot Reboot Loop

### Root Cause of Zed Freezing
Zed froze because tool execution logs containing massive raw streams, binary data, or escape sequences overwhelmed Zed's ACP protocol stream. Specifically:
- Running `bx build` / `bx lan-flash` with direct stdout/stderr in synchronous commands flooded the ACP client buffer.
- When Zed was killed/restarted, background commands were abruptly terminated.

### Root Cause of Robot Reboot Loop
When the new 18-field 6-axis IMU `TraceSample` was added:
1. `sizeof(TraceSample)` grew from 36 bytes to 64 bytes.
2. In `main/telemetry_stream.cpp`, `flush_staging_unlocked()` had allocated:
   ```cpp
   robot::TraceSample local_samples[BLOCK_SAMPLES]; // 50 * 64 = 3,200 bytes!
   uint8_t encoded[2048];                          // 2,048 bytes!
   ```
   **Totaling > 5.2 KB on the stack** inside the `robot` task, whose stack size in `main/config.h` was only `ROBOT_STACK_BYTES = 4096`!
3. This caused an instant FreeRTOS stack overflow crash as soon as telemetry stream began buffering samples, leading to a reboot loop every time the web app connected.

**Fix Applied**:
- In `main/telemetry_stream.cpp`: Moved `flush_local_samples` and `flush_encoded` out of the local stack frame into file-scope static BSS memory.
- In `main/config.h`: Increased `ROBOT_STACK_BYTES` from `4096` to `6144` as extra headroom.
- Also fixed a typo in `telemetry_stream.cpp` (`encoded[20482048]` -> `flush_encoded[2048]`).

---

## 2. Completed Implementations & Verified State

### A. Camera Gestures & Inversion (`@g-ai/rc`)
Located in `/Users/felg/gh/g-ai/rc/src/cameraInput.ts`:
- **Left & Right Click Orbit**: Pointer drag (`button === 0` and `button === 2`) now orbits the camera.
- **Click Pass-Through**: Clicks below `mouseDragThresholdPx` (5px) and clicks on pickable Babylon meshes pass through without triggering camera orbit.
- **Drag Direction Fix**: Grab-and-pull direction fixed (`orbitDelta(-dx, -dy)`).
- **Pitch Inversion**: Sign restored to natural user expectation (`const pitchSign = s.invertPitch ? -1 : 1`).
- **Babylon Adapter**: Added `createBabylonArcRotateAdapter(camera)` so any Babylon.js project can pass its `ArcRotateCamera` directly with consistent beta clamping and radius zoom limits.

### B. 6-DOF IMU Rigid-Body Kinematics & Physics (`@g-ai/rc`)
Located in `/Users/felg/gh/g-ai/rc/src/imuPhysics.ts`:
- Implemented `createWheeledRobotPhysics()` and `computeImuGravityAttitude()`:
  - Consumes live 6-axis IMU telemetry (`accelXG`, `accelYG`, `accelZG`, `gyroXDps`, `gyroYDps`, `gyroZDps`, `yawDeg`) and motor outputs (`motorL`, `motorR`).
  - Runs in Babylon's render loop (`scene.onBeforeRenderObservable` / `engine.runRenderLoop`) at full 60–120 fps.
  - Smoothly integrates 3-axis gyro angular velocity with complementary spring-damper convergence toward accelerometer spherical gravity vector.
  - Eliminates the 100 ms polling batch stutter.
  - Implements ground contact elevation (keeps wheels and tilted chassis above ground plane without clipping).

### C. 3D Scene (`balax-remote`)
Located in `~/gh/esp32-bot/balax-remote/src/BalaXBabylonScene.tsx`:
- Node hierarchy updated: `robotRoot` (planar position X/Z, axle height Y, yaw heading) -> `rollNode` (lateral roll Z) -> `pivotNode` (pitch tilt X).
- Wheels parented to `rollNode` so lateral roll tilts the wheel axle accurately.
- Render loop drives `pivotNode.rotation.x = -k.pitchRad`, `rollNode.rotation.z = -k.rollRad`, and `robotRoot.rotation.y = k.yawRad`.

### D. Firmware 6-Axis IMU Streaming (`BalaX`)
- `main/trace_sample.h`: Added `accel_x_g`, `accel_y_g`, `accel_z_g`, `gyro_x_dps`, `gyro_y_dps`, `gyro_z_dps`, `yaw_deg`.
- `main/robot.cpp`: Populates all 6 axes from `motion_.accel_g` and `motion_.gyro_dps`, plus `terms.yaw_deg`.
- `main/telemetry_stream.cpp`: Encodes 18 fields using BXZ2 block tag (`BXZ2`), with backward compatibility for 11-field `BXZ1`.
- `host/test_telemetry_stream.cpp`: Roundtrip test passes with 0 failures (`./tools/bx test`).

### E. Frontend & Bridge Telemetry (`balax-remote`)
- `src/bxz1.ts`: Decodes both `BXZ1` and `BXZ2` streams with full 6-axis IMU fields.
- `telemetryBridgePlugin.ts`: Includes 6-axis IMU columns in CSV export and serializes `/api/challenge` and `/api/ota` with `withRobotLock`.
- `src/App.tsx`: Telemetry tab displays live Accel X/Y/Z (g), Gyro X/Y/Z (°/s), and Roll/Yaw (°).
- Vite build passes cleanly (`npm run build`).

---

## 3. Next Steps to Resume in New Session

1. **Build & Flash the Fixed Firmware**:
   The stack overflow fix is in `main/telemetry_stream.cpp` and `main/config.h`. Run:
   ```sh
   cd /Users/felg/gh/Felipegalind0/BalaX
   ./tools/bx build
   ./tools/bx lan-flash 10.0.0.91
   ```
   *(Or connect via USB-C and run `./tools/bx flash` if Wi-Fi is temporarily offline).*

2. **Verify Telemetry & 3D Simulation**:
   - Dev server command (run in your own terminal):
     ```sh
     cd ~/gh/esp32-bot/balax-remote && npm run dev
     ```
   - Open browser at `http://localhost:5180`.
   - Verify:
     - 3D robot rolls, pitches, and turns smoothly in real time when you tilt or pick up the robot.
     - Telemetry tab shows live Accel, Gyro, and Roll/Yaw readouts.
     - Left-click drag and right-click drag orbit smoothly with momentum glide.
     - Invert pitch setting matches your desired steering direction.
