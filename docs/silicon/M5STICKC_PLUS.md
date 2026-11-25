# M5StickC Plus (`K016-P`) & BalaC Base — Silicon Reference

Ground-truth silicon reference for the **M5StickC Plus** (used by both **BalaX** and **JoyX**) and the **M5Stack BalaC** motor base.

> **Board revision warning:** BalaX targets **M5StickC Plus (`K016-P`)** (`ESP32-PICO-D4` + `AXP192`). Do **not** confuse it with **M5StickC Plus2** (`ESP32-PICO-V3-02`, no AXP192, IR/LED moved to `GPIO19`, LCD `DC`/`RST` moved to `GPIO14`/`GPIO12`).

---

## 1. ESP32-PICO-D4 System-in-Package (SiP)

The `ESP32-PICO-D4` packages an `ESP32` die (rev `v1.0` on our StickC Plus; `sdkconfig.defaults` sets `CONFIG_ESP32_REV_MIN_1=y`), a **40 MHz crystal**, **4 MB QIO SPI Flash**, filter capacitors, and RF matching inside a single 7×7 mm LGA48 module.

### 1.1 CPU, Clocks & Power Management
- **Cores:** Dual Xtensa LX6 32-bit cores (PRO_CPU = Core 0, APP_CPU = Core 1), single-precision FPU, up to 240 MHz.
- **Shared clock & voltage rail:** Both cores share one PLL clock and VDD rail. Running Core 1 deterministically at **100 Hz** (priority 20) while Core 0 batches Wi-Fi/HTTP/status maximizes simultaneous `waiti` idle gating.
- **BalaX clock choice (`80 MHz`):** Lowest frequency that preserves the **80 MHz APB bus** required by Wi-Fi, 400 kHz I²C, 27 MHz SPI2, and 13-bit LEDC PWM, while cutting heat inside the sealed plastic shell.
- **RTOS tick (`200 Hz` / 5 ms):** Evenly divides the 10 ms (100 Hz) balance period with 5× fewer idle timer wakeups than 1000 Hz.
- **Hardware crypto & TRNG:** Hardware SHA-256, AES, RSA, and hardware TRNG (`esp_fill_random`). **Caveat:** True physical entropy requires Wi-Fi/BT RF or SAR ADC noise to be active (otherwise it falls back to pseudo-random); Wi-Fi is active during LAN auth challenge generation.

### 1.2 Internal Memory Architecture (520 KB SRAM — NO PSRAM)

| Region | Size | Data Bus (DRAM) | Instruction Bus (IRAM) | DMA Capable | Notes |
| :--- | :---: | :--- | :--- | :---: | :--- |
| **Internal ROM** | 448 KB | `0x3FF9_0000–0x3FF9_FFFF` | `0x4000_0000–0x4005_FFFF` | No | Bootloader & ROM helpers |
| **SRAM 0** | 192 KB | *None* | `0x4007_0000–0x4009_FFFF` | **No** | First 64 KB = Flash XIP cache (32 KB per core); remaining 128 KB = executable **IRAM**. Any data access to IRAM must be **32-bit aligned**. |
| **SRAM 1** | 128 KB | `0x3FFE_0000–0x3FFF_FFFF` | `0x400A_0000–0x400B_FFFF` | **Yes** | Shared **D/IRAM** (split dynamically by linker between IRAM overflow and DRAM heap). |
| **SRAM 2** | 200 KB | `0x3FFA_E000–0x3FFD_FFFF` | *None* | **Yes** | Pure **DRAM** (static globals, FreeRTOS stacks, DMA buffers, heap). |
| **RTC FAST / SLOW** | 8 + 8 KB | `0x3FF8_0000` / `0x5000_0000` | `0x400C_0000` (FAST, PRO_CPU only) | No | Survives deep sleep; `RTC_NOINIT_ATTR` survives software resets. |
| **SPI Flash XIP** | 4 MB | `0x3F40_0000–0x3F7F_FFFF` (DROM) | `0x400C_2000–0x40BF_FFFF` (IROM) | No | Cached via SRAM 0 (80 MHz QIO in `sdkconfig.defaults`). |

**Practical heap reality in BalaX:**
- Total DMA-capable DRAM is 328 KB; Wi-Fi + lwIP + ESP-NOW consume ~80–100 KB static/dynamic heap, leaving ~219 KB free after Wi-Fi init (~108 KB used at steady state).
- Keep `ram_free_kb` and `ram_min_kb` healthy (`probe ram_free_kb ram_min_kb`); avoid large stack allocations on Core 1 (`ROBOT_STACK_BYTES = 6144`) or heap fragmentation from `std::string` / JSON builders in hot loops.

### 1.3 CRITICAL: ESP32-PICO-D4 Internal Flash Pin Mapping (Differs from WROOM-32!)

Unlike `ESP32-WROOM-32` / `ESP32-S` (where flash uses `GPIO 6–11` and `GPIO 16/17` are free), the **ESP32-PICO-D4 routes its integrated 4 MB flash to `GPIO 6, 7, 8, 11, 16, 17`**, freeing **`GPIO 9` and `GPIO 10`**:

| GPIO | PICO-D4 Flash Signal | Availability on M5StickC Plus |
| :--- | :--- | :--- |
| `GPIO 6` (`CLK`) | `FLASH_CLK` | **RESERVED — never touch** |
| `GPIO 7` (`SD0`) | `FLASH_SD3` | **RESERVED — never touch** |
| `GPIO 8` (`SD1`) | `FLASH_SD1` | **RESERVED — never touch** |
| `GPIO 11` (`CMD`) | `FLASH_SD2` | **RESERVED — never touch** |
| `GPIO 16` (`IO16`) | `FLASH_CS` | **RESERVED — never touch** (crashes flash XIP!) |
| `GPIO 17` (`IO17`) | `FLASH_SD0` | **RESERVED — never touch** (crashes flash XIP!) |
| `GPIO 9` (`SD2`) | *Not used by flash* | **Free** → wired to **IR LED** (active-LOW) |
| `GPIO 10` (`SD3`) | *Not used by flash* | **Free** → wired to **Red LED** (active-LOW) |

### 1.4 Silicon Errata & Architectural Constraints That Affect BalaX
1. **Flash Write Cache Disable (All ESP32 revs):** Every SPI flash erase/write (NVS `save`, OTA partition write, telemetry flash spillover at `0x3E0000`) **disables the flash cache on both cores** and suspends non-IRAM tasks/interrupts for milliseconds (a 4 KB sector erase takes ~30–100 ms!).
   - **Rule:** Never perform flash writes/erases during active balancing! Stop telemetry flash spillover and require `DISARMED` (or safe state) before NVS/OTA writes.
   - Any ISR registered with `ESP_INTR_FLAG_IRAM` must keep all code (`IRAM_ATTR`) and read-only data (`DRAM_ATTR` / `DRAM_STR`) in internal RAM.
2. **Erratum `[GPIO-3.11]` — 80 ns Glitch on `GPIO36` & `GPIO39` (All ESP32 revs, unfixed):** Whenever SAR ADC1/ADC2 or Wi-Fi power-save wakes RTC peripherals, `GPIO36` (HAT shared pin) and `GPIO39` (**Button B**) suffer a ~80 ns spurious LOW pulse.
   - **Rule:** Never attach edge-triggered interrupts to `GPIO36` or `GPIO39` while Wi-Fi or ADC is active; poll and debounce instead.
3. **ADC2 vs. Wi-Fi Conflict:** `ADC2` (`GPIO 0, 2, 4, 12–15, 25–27`) cannot be read while Wi-Fi is active. Use `ADC1` (`GPIO 32–36, 39`) for any analog input during Wi-Fi operation.
4. **Input-Only GPIOs (`GPIO 34–39`):** No output driver and **no internal pull-up or pull-down resistors**. Button A (`GPIO37`) and Button B (`GPIO39`) rely on external board pull-ups.
5. **Strapping Pins (`GPIO 0, 2, 5, 12, 15`):**
   - `GPIO 0`: LOW at reset = serial download mode (shared with HAT `SDA` & `MIC_CLK`).
   - `GPIO 2`: Must be LOW/floating during serial download (wired to passive buzzer).
   - `GPIO 12` (`MTDI`): **Must be LOW at reset** so `VDD_SDIO` = 3.3 V for the 3.3 V internal flash (HIGH selects 1.8 V and prevents boot).

---

## 2. Complete M5StickC Plus Pin & Bus Map

Verified against `~/gh/esp32-tools/tools/components/m5stickc_plus/include/m5stickc_plus/pins.h`:

### 2.1 Internal I²C Bus (Port 0 — `GPIO21` SDA, `GPIO22` SCL, 400 kHz)
| 7-bit Addr | Silicon | Role | BalaX Access Pattern |
| :---: | :--- | :--- | :--- |
| `0x34` | **AXP192** | PMIC, LiPo charger, ADCs, power rails, power key | Polled at **1 Hz** on Core 0 (plus power-key check) — kept outside 100 Hz control loop to avoid bus contention |
| `0x51` | **BM8563** | Real-time clock (backed by AXP192 backup charger) | Unused by BalaX currently; shares bus with AXP192/MPU6886 |
| `0x68` | **MPU6886** | 6-axis accel + gyro + die temp (`WHO_AM_I = 0x19`) | Burst-read (14 bytes from `0x3B`) at **100 Hz** on Core 1 |

### 2.2 SPI2 Display, Buzzer, LEDs, Buttons, Mic
| Peripheral | Silicon | GPIOs | Logic / Configuration |
| :--- | :--- | :--- | :--- |
| **LCD** | **ST7789V2** | `MOSI=15`, `SCLK=13`, `CS=5`, `DC=23`, `RST=18` | `SPI2_HOST`, 27 MHz, SPI Mode 0, RGB565 big-endian, power via AXP192 `LDO2`/`LDO3` |
| **Buzzer** | Passive piezo | `GPIO2` | `LEDC_LOW_SPEED_MODE`, Timer 0, Ch 0, 13-bit @ 80 MHz APB (up to 9.7 kHz) |
| **Red LED** | Onboard LED | `GPIO10` | Active-LOW (`0` = ON) |
| **IR TX** | IR emitter | `GPIO9` | Active-LOW (PWM/RMT capable) |
| **Button A** | Front M5 btn | `GPIO37` | Input-only, active-LOW, external pull-up |
| **Button B** | Side btn | `GPIO39` | Input-only, active-LOW, external pull-up (**erratum `[GPIO-3.11]`** — poll, don't IRQ!) |
| **Power Btn** | Side power btn | *(via AXP192)* | Latched IRQ in AXP192 reg `0x46` (bit 1 short press, bit 0 long press) |
| **PDM Mic** | **SPM1423** | `CLK=GPIO0`, `DATA=GPIO34` | **Conflicts with HAT `G0` (BalaC I²C SDA)!** Powered by AXP192 `LDOio0` (3.3 V) |

### 2.3 External Headers & Wiring Traps

**Top 8-Pin HAT Header (2.54 mm):** `GND`, `5V_OUT`, `G26`, `G36/G25`, `G0`, `BAT`, `3V3`, `5V_IN`
- **BalaC Hat I²C (Port 1, 100 kHz):** `SDA = GPIO0`, `SCL = GPIO26`.
- **`G36/G25` Shared Pad Trap:** Both `GPIO36` (input-only, ADC1) and `GPIO25` (I/O, DAC1, ADC2) are wired to the **same single header contact**. Whenever using one, explicitly set the other to floating input (`gpio_pullup_dis` + `gpio_pulldown_dis`).
- **`G0` Triple-Duty Trap:** Shared by HAT `SDA`, `SPM1423` mic clock, and ESP32 boot strapping. Never pull `G0` low during reset unless flashing.
- **`5V_IN` (`VBUS_VIN`, 4.8–5.5 V):** Feeds the AXP192 input alongside USB-C to power the stick and charge its internal 120 mAh cell (used by the BalaC base).

**Bottom Grove Port (`HY2.0-4P`):** `GND` (Black), `5V` (Red, AXP192 `EXTEN` boost), **`GPIO32`** (Yellow), **`GPIO33`** (White)
- `GPIO32` and `GPIO33` are completely free, full I/O, ADC1-capable, non-strapping pins — **the cleanest interface for attaching an ESP32-CAM companion UART** (`G32/G33` ↔ ESP32-CAM `U0RX/U0TX` `GPIO3/GPIO1` + common `GND`, up to 921600 baud; do **not** power ESP32-CAM from the Grove 5V pin!).

---

## 3. AXP192 Power Management IC (`0x34`)

X-Powers PMIC managing all M5StickC Plus rails, linear LiPo charging, coulomb/ADC telemetry, and power button.

### 3.1 M5StickC Plus Power Rail Tree
| AXP192 Rail | Startup State (BalaX) | Supplies | Notes |
| :--- | :--- | :--- | :--- |
| `DC-DC1` | ON (3.3 V) | ESP32-PICO-D4, MPU6886, 3V3 rail | High-efficiency buck converter |
| `LDO1` | Always-ON (3.3 V) | BM8563 RTC | Low quiescent standby rail |
| `LDO2` | ON, 2.5–3.0 V (`reg 0x28` hi nibble, steps 7–12) | **ST7789V2 Backlight** | Turning `LDO2` OFF (`brightness = 0`) saves ~15–20 mA & cuts heat! |
| `LDO3` | ON, 3.0 V (`reg 0x28` lo nibble = `0xC`) | **ST7789V2 Logic VDD** | Must be ON before `display.init()` |
| `LDOio0` (`GPIO0`) | ON, 3.3 V (`reg 0x90=0x02, 0x91=0xF0`) | **SPM1423 Microphone** | Can be disabled if mic is unused to shave mW |
| `EXTEN` / `IPSOUT` | ON (`reg 0x12 = 0x4D`) | 5 V boost for Grove / HAT `5V_OUT` | Do not draw high current (e.g. ESP32-CAM) from tiny internal cell |

### 3.2 Key Registers & ADC Scaling (`axp192_registers.h`)
| Reg | Function | BalaX Value / Formula |
| :---: | :--- | :--- |
| `0x12` | Power Output Control | `0x4D` (`EXTEN`, `LDO3`, `LDO2`, `DC-DC1`); bit 2 = `LDO2` enable |
| `0x28` | `LDO2`/`LDO3` Voltage | `1.8 V + 0.1 V × nibble` (`0xCC` = 3.0 V / 3.0 V) |
| `0x30` | VBUS-IPSOUT Path | `0x80` (no VBUS current/voltage hold limit) |
| `0x32` | Shutdown & Bat Detect | `0x46` normal; set bit 7 (`0x80`) to cut system power completely |
| `0x33` | Charge Control 1 | **`0xC0` = 4.2 V target, 100 mA max** (0.83 C on 120 mAh cell — **never increase!**) |
| `0x35` | Backup Battery Charge | `0xA2` (3.0 V, 200 µA for RTC backup) |
| `0x36` | PEK Timing | `0x0C` (128 ms power-on, 1 s long-press IRQ, 4 s hard power-off) |
| `0x46` | IRQ Status 3 (PEK) | Bit 1 = short press, Bit 0 = long press (write 1s back to clear) |
| `0x5A` / `0x5C` | VBUS Voltage / Current | 12-bit: `1.7 mV/LSB` / `0.375 mA/LSB` |
| `0x5E` | **AXP192 Die Temp (`pmic_c`)** | 12-bit: `raw × 0.1 °C − 144.7 °C` |
| `0x78` | Battery Voltage | 12-bit: `1.1 mV/LSB` |
| `0x7A` / `0x7C` | Battery Charge / Discharge mA | 13-bit: `0.5 mA/LSB` |

### 3.3 LiPo & Thermal Safety Ground Truth
- The internal cell is a **120 mAh 3.7 V pouch** sealed directly beside the AXP192 with **no battery thermistor (TS pin unused)**.
- Linear USB charging inside the AXP192 dissipates `(5.0 V − V_bat) × I_chg` directly as heat inside the shell.
- Keep `pmic_c < 42 °C` nominal (`≥ 42 °C` amber caution, `≥ 46 °C` red thermal block in `balax-remote` and `lan-flash`). See [`../HARDWARE.md`](../HARDWARE.md) for full safety limits.

---

## 4. MPU6886 6-Axis IMU (`0x68`, `WHO_AM_I = 0x19`)

TDK InvenSense 16-bit 3-axis accelerometer + 3-axis gyroscope + on-die temperature sensor (datasheet `DS-000193 Rev 1.1`).

### 4.1 BalaX Configuration (`mpu6886_registers.h`)
| Register | Addr | BalaX Value | Meaning |
| :--- | :---: | :---: | :--- |
| `PWR_MGMT_1` | `0x6B` | `0x01` | Auto-select best clock (gyro PLL once spun up) |
| `SMPLRT_DIV` | `0x19` | `5` | Internal ODR = `1000 Hz / (1 + 5) = 166.7 Hz` (faster than 100 Hz control loop) |
| `CONFIG` | `0x1A` | `1` | Gyro DLPF = **176 Hz** (1 kHz base rate) |
| `GYRO_CONFIG` | `0x1B` | `0` | **±250 °/s** (`131.072 LSB/(°/s)`, `250 / 32768 °/s/LSB`) — max resolution |
| `ACCEL_CONFIG` | `0x1C` | `0` | **±2 g** (`16384 LSB/g`, `2 / 32768 g/LSB`) — max resolution |
| `ACCEL_CONFIG_2` | `0x1D` | `0` | Accel DLPF = **218.1 Hz** (1 kHz base rate) |
| `ACCEL_XOUT_H` | `0x3B` | 14-byte burst | Big-endian `int16`: `ax, ay, az, temp, gx, gy, gz` in a single atomic I²C read |

- **Temperature formula (`imu_c`):** `T (°C) = raw / 326.8 + 25.0`. Note: `imu_c` runs ~8–13 °C hotter than `pmic_c` because the MPU6886 sits close to the ESP32-PICO-D4 die (`imu_c ≥ 58 °C` triggers thermal lockout).

### 4.2 Noise, Drift & Calibration Implications (Datasheet Specs)
| Parameter | Typical | Max | Impact on BalaX |
| :--- | :---: | :---: | :--- |
| **Gyro rate noise density** | `0.004 °/s/√Hz` | `0.01 °/s/√Hz` | `0.04 °/s rms` @ 100 Hz BW — very clean short-term pitch integration |
| **Gyro initial ZRO** | `±1 °/s` | `±10 °/s` | Requires startup bias calibration (`CALIBRATING` state) |
| **Gyro ZRO vs. temperature** | `±0.01 °/s/°C` | `±0.05 °/s/°C` | As `imu_c` warms by +15 °C, gyro bias drifts up to 0.15–0.75 °/s! Online bias estimation in `imu_fusion.h` is essential |
| **Gyro / Accel cross-axis** | `±1 %` | `±4.5 %` | Corrected by 6-face calibration (`imuCalibration.ts` in `@g-ai/rc`) |
| **Accel noise density** | `100 µg/√Hz` | `170 µg/√Hz` | `1.0 mg rms` @ 100 Hz BW (wheel/gear vibration dominates in practice) |
| **Accel board-level zero-g offset** | `±40 mg` | `±100 mg` | Up to ~2–6° raw tilt error before 6-face calibration + `LEVELING` |
| **Accel zero-g vs. temperature** | `±0.5 mg/°C` (XY), `±1 mg/°C` (Z) | `±1.2 mg/°C` | Minor (~0.03–0.07°/°C), manageable with online fusion |
| **On-chip FIFO** | **1024 bytes** | — | Unused today (polled at 100 Hz); available if burst/oversampling is ever added |

### 4.3 Physical Axis Orientation in BalaX
- **Pitch rate:** `+gyro.x`
- **Yaw rate:** `-gyro.y`
- **Forward accel:** `+accel.z`
- **Up accel:** `+accel.y` (upright either way up when `|accel.y| > UPRIGHT_G`)
- Because the MPU6886 sits above the wheel axle by lever arm `L`, angular acceleration $\ddot{\theta}$ injects tangential acceleration $L\ddot{\theta}$ into `accel.z` and centripetal acceleration $L\dot{\theta}^2$ into `accel.y` — compensated in `imu_fusion.h` and calibrated in `balax-remote`.

---

## 5. ST7789V2 IPS TFT Display (SPI2)

- **Native controller GRAM:** `240 × 320` pixels; M5StickC Plus glass exposes a **`135 × 240`** IPS window at offset **`GAP_X = 53, GAP_Y = 40`** (180° rotation).
- **Wire format:** 16-bit **RGB565 big-endian** (byte-swapped on little-endian ESP32), `LCD_RGB_ELEMENT_ORDER_RGB`, display color inversion ON (required for IPS panel polarity).
- **SPI speed & DMA:** `SPI2_HOST` at **27 MHz** (24-row DMA stripes, `135 × 24 × 2 = 6480 bytes` in DMA-capable internal DRAM). Row-hashing (FNV-1a) skips unchanged stripes.
- **Thermal optimization:** When `brightness == 0`, AXP192 `LDO2` turns off the backlight **and** firmware skips SPI screen refreshes completely.

---

## 6. BalaC Base Silicon (`K038` / `K038-B`)

M5Stack does not publish a full PDF schematic for the BalaC base, but its silicon and I²C protocol are verified across M5Stack specs and `components/balac/motors.cpp`:

| Component | Silicon / Spec | Notes |
| :--- | :--- | :--- |
| **Motor Sub-MCU** | **STM32F030F4P6** (ARM Cortex-M0, 48 MHz) | I²C slave at **`0x38`** on HAT I²C (`SDA=G0, SCL=G26`, **100 kHz**) |
| **Motor Drivers** | **2 × L9110S** H-bridge ICs | Driven by STM32 PWM |
| **Actuators** | **2 × 9G 360° continuous servos** (FM90) | **No wheel encoders!** Open-loop PWM command only (`-127..+127`); right motor sign is negated in `motors.cpp` due to mirrored mounting |
| **Base Power** | **16340 Li-ion (3.7 V, 700 mAh)** + boost/charger | Powers motors directly and back-feeds StickC Plus `5V_IN` via HAT header. **Use this external rail (not StickC's 120 mAh cell) to power an ESP32-CAM companion.** |

- **I²C write frame:** 2 bytes `[channel, int8_t power]`, where `channel = 0x00` (left) or `0x01` (right), `power ∈ [-127, +127]`.
- **Hot-plug safe:** Missing or switched-off BalaC base is non-fatal; `balac::Motors` re-probes `0x38` every 1 s.
