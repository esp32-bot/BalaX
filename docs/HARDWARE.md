# Hardware and protocol facts

> **Deep silicon datasheets & pin/errata guides:** See [`docs/silicon/README.md`](silicon/README.md) — covering [**M5StickC Plus & BalaC**](silicon/M5STICKC_PLUS.md) (`ESP32-PICO-D4`, `AXP192`, `MPU6886`, `ST7789V2`, `STM32F030F4P6`), [**AI-Thinker ESP32-CAM**](silicon/ESP32_CAM.md) (`ESP32-S`, PSRAM, 1-bit SDMMC, I²S0 DMA), and [**OV2640**](silicon/OV2640.md) (dual-bank SCCB, hardware JPEG DSP, clocking, rolling shutter).

## Thermal & Battery Safety (IMPORTANT)
- **Sealed M5StickC Plus enclosure:** The M5StickC Plus packs an ESP32-PICO-D4, AXP192 PMIC, IPS LCD backlight, and a tiny internal 120 mAh LiPo pouch inside a closed plastic shell with very little airflow and **no direct battery thermistor**.
- **Blocked sides / add-ons:** Taping or mounting extra modules (e.g., an ESP32-CAM) against the case further traps heat inside the stick.
- **LiPo safe temperature limits (for extended use):**
  - **Charging:** `0 °C – 45 °C` max (never charge a warm/hot stick — linear USB charging inside the AXP192 adds another ~300–500 mW of heat!).
  - **Extended discharge / longevity:** Keep the cell **`≤ 45 °C`** (sustained `45–50 °C+` accelerates electrolyte breakdown and puffing inside the tight plastic shell; `60 °C` is the absolute short-term datasheet ceiling, not safe for extended operation).
  - Because there is **no battery thermistor**, we monitor `pmic_c` (AXP192 die right next to the cell) and `imu_c` (MPU6886 die, typically `+8–13 °C` warmer than `pmic_c`) conservatively:
    - **Warm caution (`balax-remote` amber banner):** `pmic_c ≥ 42 °C` or `imu_c ≥ 52 °C` — dim screen to `0%` or pause soon.
    - **Too Hot / Cool Down (`balax-remote` red banner + `lan-flash` block):** `pmic_c ≥ 46 °C` or `imu_c ≥ 58 °C` — disconnect, power off, and let BalaX cool down.
- **Recommended operating practice:**
  - Operate BalaX in **cooler ambient temperatures** (avoid direct sun, hot rooms, or enclosed warm surfaces).
  - Check temps anytime with `balax lan-cmd "probe pmic_c imu_c battery_pct battery_ma cpu_mhz"`.
  - When driving or tuning via `balax-remote`, turn off the onboard LCD (`set brightness 0` + `save`, or slide **brightness** to `0` in the web UI) — this cuts ~15–20 mA of backlight heat and skips SPI display refreshes completely.
  - Charge BalaX while **powered off** (or cool & idle), not while streaming Wi-Fi in a warm room.
- **Firmware efficiency defaults:**
  - **80 MHz CPU** (lowest speed keeping the 80 MHz APB bus for Wi-Fi, 400 kHz I2C, and SPI).
  - **200 Hz RTOS tick** (5× fewer idle timer wakeups than 1000 Hz while matching the exact 100 Hz / 10 ms balance loop).
  - **Wi-Fi modem-sleep (`WIFI_PS_MIN_MODEM`)** automatically enabled in STA mode + **`+13 dBm` max TX power** (20 mW vs 100 mW default).
  - **Core pinning:** Core 1 runs only the deterministic 100 Hz IMU/balance loop (~10–15% load) while Core 0 handles Wi-Fi/HTTP/display; on ESP32 both cores share one clock & voltage rail, so keeping Core 1 deterministic and maximizing simultaneous `waiti` idle time is more energy-efficient than splitting work 50/50 across cores.

### Power modes (`off` vs `sleep`) and wake reality
- **Why not full AXP192 shutdown for remote USB wake?** Clearing AXP192 `Reg 0x32` bit 7 cuts `DC-DC1` (3.3 V to the ESP32-PICO-D4), making remote USB `RTS→EN` wake impossible — only pressing the physical power button (or replugging USB) restores `DC-DC1`.
- **`off` mode (USB-charge deep sleep — plugged in only):**
  - Guards: refused while motors are live (`ARMED` / `TAKING_OFF`), and refused when `usb_v < 4.0 V` unless `force` is passed.
  - Hardware state: pauses Core 1 100 Hz loop (motors 0), puts `MPU6886` into 8 µA sleep (`PWR_MGMT_1` bit 6), stops Wi-Fi (`esp_wifi_stop()`), cuts AXP192 `LDO2` (LCD backlight), `LDO3` (LCD logic), `EXTEN` (5 V boost), and `LDOio0` (mic) while keeping `DC-DC1` + 100 mA LiPo charger ON, then enters `esp_deep_sleep_start()` (~1–2 mA system draw so the cell charges cool).
  - Wake sources: USB serial `RTS→EN` reset pulse (`./tools/bx wake --usb` or `balax-remote` **Wake USB**, using `DTR=False, RTS=True` → `DTR=False, RTS=False` so it boots normally rather than into the ROM bootloader), plus physical Button A (`GPIO37`, `RTC_GPIO1`) ext0 active-LOW backup wake.
- **`sleep` mode (Wi-Fi connected low-power sleep — wireless):**
  - Guards: explicit sleep is refused while motors are live unless forced. Automatic idle sleep requires `READY`, disarmed, and continuous IMU-confirmed stillness for `idle_sleep_s` (default 180 s, `0` disables). Thermal sleep requires 3 consecutive hot samples (`pmic_c ≥ 46 °C` or `imu_c ≥ 58 °C`) during the same disarmed, stationary interval.
  - Hardware state: pauses Core 1 100 Hz loop (motors 0), sleeps `MPU6886`, blanks/cuts `LDO2`/`EXTEN`/`LDOio0`, switches STA to `WIFI_PS_MAX_MODEM` (`listen_interval = 3`), drops CPU min to 40 MHz with automatic tickless light sleep (`CONFIG_FREERTOS_USE_TICKLESS_IDLE=y`), and throttles `status_task` to 0.5 Hz with LCD draws skipped.
  - Wake sources: `wake` command, active control/OTA/program commands (`arm`, `takeoff`, `stream`, `ota`, `prog`), JoyX ESP-NOW packets, UART0 activity, or physical Button A (`GPIO37`). Passive queries (`status`, `probe`, `power`, `/api/challenge`) and `/api/stream` (returns 0 bytes + `X-Power-Mode: sleep`) do **not** wake a sleeping robot.
  - While awake, `esp_pm_configure` keeps `light_sleep_enable = false` at 80 MHz so the 100 Hz Core 1 balance loop timing is completely unaffected.

## Buses
- Internal I2C port 0 (SDA 21, SCL 22, 400 kHz): AXP192 at 0x34, MPU6886 at 0x68.
- Hat I2C port 1 (SDA 0, SCL 26, 100 kHz): BalaC at 0x38. Write `[motor, int8]`; the right
  motor is negated. A missing base is not fatal (the probe sets `present`).
- IMU axes: pitch rate = gyro.x, yaw rate = -gyro.y, forward = accel.z, up = accel.y.
  "Upright" counts either way up (`|up_g| > UPRIGHT_G`), as in Arduino.
- AXP192 power button: reg 0x46, bit 1 short press, bit 0 long press.
- IMU and battery are read in one I2C transaction each; the PMIC is read once a second
  outside the control loop because it shares the bus.

## JoyX ESP-NOW protocol
- Frame `{uint8_t i; char a[64]}` (65 bytes), always sent in full (the remote copies that much).
- Remote to robot: `ARM`, `DISARM`, `REQUEST_TAKEOFF`, `c+XX+YY` (stick).
- Robot replies `ARMED` / `DISARMED` / `TAKING_OFF`, or `c+XX+YY` followed by yaw `%+04.0f`,
  pitch `%+04.0f`, left and right motor `%+04d` (raw power, -127..127, clamped to +-999; the
  remote parses raw at indexes 15-22), then two core loads `%+03d`.
  When `index % 100 == 0` the reply starts `s%+d(armed)%+03d(battery)` instead of `c`.
- JoyX finds the robot by scanning for SSID `MARIN<n>-robot` and uses the AP BSSID as the peer
  (channel 0, unencrypted). The robot replies from `WIFI_IF_AP`.
- Both radios share one channel. The robot AP starts on channel 1 and follows the STA
  channel once connected. Scans hop channels, so they are held while the motors are live or
  the remote is live (`may_scan_for_networks` in main.cpp).
- IDF 6.1 callbacks: recv `(const esp_now_recv_info_t*, const uint8_t*, int)`, send
  `(const esp_now_send_info_t*, esp_now_send_status_t)`. Error 12396 = `ESP_ERR_ESPNOW_IF`.

## Wi-Fi
- Starts in STA mode (falls back to APSTA when no known network is found), `WIFI_STORAGE_RAM`, `nvs_enable=false`.
- Uses `WIFI_PS_MIN_MODEM` + `+13 dBm` max TX power whenever connected in STA mode (softAP off) to minimize heat; switches to `WIFI_PS_NONE` only while softAP is active.
- Scan, pick the lowest-index known network (stronger RSSI breaks ties), then connect with
  bssid and channel set (fast connect). Retries back off 2 s to 60 s. 2.4 GHz only.
- Wrong-password detection: disconnect reasons AUTH_FAIL / 4WAY_HANDSHAKE_TIMEOUT /
  HANDSHAKE_TIMEOUT.
- Cost: Wi-Fi adds about 750 KB of image (1.07 MB of the 1.97 MB OTA slot) and takes the
  heap from about 300 KB to 219 KB.

## macOS notes for tools/wifi.py
macOS redacts the current SSID. Use `networksetup -listpreferredwirelessnetworks <dev>` and
the Keychain item `security find-generic-password -D "AirPort network password" -a SSID -w`.
