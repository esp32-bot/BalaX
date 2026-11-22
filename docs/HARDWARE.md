# Hardware and protocol facts

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
- APSTA mode, WIFI_STORAGE_RAM, `nvs_enable=false`, `WIFI_PS_NONE` (pins CPU at 240 MHz; an
  energy trade-off for ESP-NOW latency).
- Scan, pick the lowest-index known network (stronger RSSI breaks ties), then connect with
  bssid and channel set (fast connect). Retries back off 2 s to 60 s. 2.4 GHz only.
- Wrong-password detection: disconnect reasons AUTH_FAIL / 4WAY_HANDSHAKE_TIMEOUT /
  HANDSHAKE_TIMEOUT.
- Cost: Wi-Fi adds about 750 KB of image (1.07 MB of the 1.97 MB OTA slot) and takes the
  heap from about 300 KB to 219 KB.

## macOS notes for tools/wifi.py
macOS redacts the current SSID. Use `networksetup -listpreferredwirelessnetworks <dev>` and
the Keychain item `security find-generic-password -D "AirPort network password" -a SSID -w`.
