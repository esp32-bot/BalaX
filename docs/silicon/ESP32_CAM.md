# AI-Thinker ESP32-CAM (Rev 1.6) — Silicon Reference

Ground-truth silicon reference for the **AI-Thinker ESP32-CAM** companion module (planned `balax-cam.local` camera + MicroSD storage + UART RPC companion for BalaX).

---

## 1. Core Silicon Overview

| Subsystem | Silicon | Key Specs & Constraints |
| :--- | :--- | :--- |
| **MCU Module** | **ESP32-S** (`ESP32-D0WDQ6` rev 1.x or `D0WD-V3`) | Dual Xtensa LX6 up to 240 MHz, 520 KB SRAM, 2.4 GHz Wi-Fi / BLE, 40 MHz XTAL |
| **SPI Flash** | 4 MB QIO Flash (inside ESP32-S) | Connected internally to **`GPIO 6–11`** (never use as GPIOs) |
| **External PSRAM** | **64 Mbit (8 MB die)** SPI PSRAM (`IPS6404L` / `ESP-PSRAM64H`) | **4 MB directly memory-mapped** (`0x3F80_0000–0x3FBF_FFFF`); remaining 4 MB requires `himem` banking. Wired to **`GPIO 16 (CS#)`** and **`GPIO 17 (CLK)`** + shared `GPIO 6–11` |
| **Camera Interface** | 24-pin FPC → **OV2640** (see [`OV2640.md`](OV2640.md)) | 8-bit parallel DVP captured via **ESP32 I²S0** peripheral + LEDC `XCLK` |
| **MicroSD Slot** | SDMMC Host **Slot 1 (`HS2_*`)** | Supports 4-bit or **1-bit mode** (47 kΩ board pull-ups `R8–R11`) |
| **Power Regulator** | **AMS1117-3.3** linear LDO + 2× **XC6206** (`2.8 V`, `1.2 V`) | `AMS1117` needs ≥ 4.5 V on `5V` pin (~1.1 V dropout); camera LDOs gated by P-MOSFET **`Q2` (`GPIO 32`, active-LOW)** |
| **LEDs** | High-power white Flash LED + Red status LED | Flash LED on **`GPIO 4`** via `S8050` NPN (`Q1`, active-HIGH); Red LED on **`GPIO 33`** (active-LOW, not on header) |
| **USB-UART** | **None onboard** | Requires external 3.3 V FTDI or `ESP32-CAM-MB` (CH340) + `GPIO 0` LOW to flash |

---

## 2. Complete Pin Allocation & Conflict Matrix

Almost every GPIO on the ESP32-S is wired to the OV2640, PSRAM, or MicroSD slot. **Never assign a pin without checking this table.**

### 2.1 Exposed 16-Pin Headers (`2 × 8` pins)

| Header Pin | GPIO | Connected Onboard Silicon | Safe to Use Externally? |
| :--- | :---: | :--- | :--- |
| `5V` | — | Input to `AMS1117-3.3` | **Primary power input** (clean 5 V, ≥ 500–600 mA peak; recommend 1 A+ rail) |
| `GND` (×3) | — | Common ground | Always share common GND with BalaX / BalaC |
| `3V3` | — | `AMS1117-3.3` output | Avoid back-powering unless `5V` is unconnected; small bypass caps |
| `VCC` | — | Configurable `3.3 V` or `5 V` output (`R1`/`R2` 0 Ω) | Usually strapped to `3.3 V` out from factory |
| `IO16` | `GPIO 16` | **PSRAM `CS#`** (and `U2RXD`) | **NO when PSRAM is enabled!** Causes instant boot loops / PSRAM corruption |
| `IO0` | `GPIO 0` | **OV2640 `XCLK`** + Boot strap (10 kΩ pull-up `R19`) | Pull LOW at reset **only** to enter serial bootloader; driven at 10–20 MHz by LEDC at runtime |
| `U0T` | `GPIO 1` | `UART0 TX` | **Best free pin** (serial flash, logs, or **UART RPC to BalaX Grove `G32/G33`** once console logs are redirected/quieted) |
| `U0R` | `GPIO 3` | `UART0 RX` | **Best free pin** (paired with `U0T`) |
| `IO2` | `GPIO 2` | **SD `DATA0`** (`HS2_DATA0`) + Boot strap | Used in **both 1-bit & 4-bit SD**; must not be driven HIGH during serial download (remove SD card if flashing fails!) |
| `IO4` | `GPIO 4` | **SD `DATA1`** + **Flash LED** (`Q1` NPN, 1 kΩ `R13`) | **Free only in 1-bit SD mode** (drive LOW to keep blinding flash LED off!) |
| `IO12` | `GPIO 12` | **SD `DATA2`** + **`MTDI` boot strap** | **Free only in 1-bit SD mode** — **CRITICAL:** pulling HIGH at reset selects 1.8 V flash & bricks boot! |
| `IO13` | `GPIO 13` | **SD `DATA3`** | **Cleanest free GPIO when using 1-bit SD mode** (no boot strap, no LED) |
| `IO14` | `GPIO 14` | **SD `CLK`** (`HS2_CLK`) | Used in **both 1-bit & 4-bit SD** |
| `IO15` | `GPIO 15` | **SD `CMD`** (`HS2_CMD`) + Boot log strap | Used in **both 1-bit & 4-bit SD** |

### 2.2 Internal-Only Pins (Not on Headers)

| Signal | GPIO | `esp32-camera` Macro | Notes |
| :--- | :---: | :--- | :--- |
| **Camera Power** | `GPIO 32` | `PWDN_GPIO_NUM` | Drives P-MOSFET `Q2` feeding `XC6206-2.8V` (`AVDD`) & `XC6206-1.2V` (`DVDD`). **`LOW` = ON, `HIGH` = OFF** |
| **Camera Reset** | `−1` (N/C) | `RESET_GPIO_NUM` | Hardware RC reset (`4.7 kΩ` / `0.1 µF`) — no GPIO |
| **SCCB SDA / SCL** | `GPIO 26` / `27` | `SIOD` / `SIOC` | 4.7 kΩ pull-ups (`R18`/`R17`); SCCB bus for OV2640 (`0x30`) |
| **DVP D0–D3 (`Y2–Y5`)** | `GPIO 5, 18, 19, 21` | `Y2`–`Y5` | Lower 4 data bits from OV2640 |
| **DVP D4–D7 (`Y6–Y9`)** | `GPIO 36, 39, 34, 35` | `Y6`–`Y9` | Input-only GPIOs (`GPIO36/39` are also affected by `[GPIO-3.11]` when ADC/Wi-Fi sleep toggles, though synchronous I²S sampling masks short glitches) |
| **VSYNC / HREF / PCLK** | `GPIO 25, 23, 22` | `VSYNC` / `HREF` / `PCLK` | DVP frame/line/pixel clocks |
| **PSRAM CLK** | `GPIO 17` | *(internal)* | Shared SPI clock to PSRAM chip `U5` |
| **Red Status LED** | `GPIO 33` | *(internal)* | Active-LOW (`0` = ON) — safe status indicator with zero pin conflicts |

---

## 3. External SPI PSRAM — Architecture, Speed & Traps

- **Bus sharing:** PSRAM shares the 4-bit SPI bus (`GPIO 6–11`) with the 4 MB SPI Flash via separate chip selects (`GPIO 16` = PSRAM `CS#`). Consequently, **CPU cannot simultaneously fetch uncached flash instructions and PSRAM data on the same SPI cycle**.
- **Mapping & cache:** Mapped through the same 64 KB SRAM0 cache into `0x3F80_0000–0x3FBF_FFFF` (4 MB). Accesses that miss cache are **~10–20× slower** than internal DRAM.
  - Place **large frame buffers** (`fb_location = CAMERA_FB_IN_PSRAM`) and HTTP/artifact buffers in PSRAM.
  - Keep **DMA descriptors, Wi-Fi/lwIP hot structures, FreeRTOS task stacks, and ISRs** in internal DMA-capable DRAM (`SRAM1`/`SRAM2`).
- **Erratum `[CPU-3.9]` (ESP32 Rev 1.0 / 1.1):** Many AI-Thinker boards ship with Rev 1 `ESP32-D0WDQ6` silicon where certain store→load instruction sequences corrupt PSRAM through cache.
  - **Fix:** Enable `CONFIG_SPIRAM_CACHE_WORKAROUND=y` (`-mfix-esp32-psram-cache-issue`) unless you are 100% certain every target board is **Rev 3 (`D0WD-V3`)**. Be aware the workaround relocates Newlib routines out of ROM and costs **≥ 10–15 KB of internal IRAM**.
- **Flash Write Interlock:** Any SPI flash write/erase (NVS, OTA) **disables the cache for both Flash AND PSRAM**. Always stop camera capture / pause PSRAM-touching ISRs before writing flash (`esp_camera_deinit()` or pause DMA before OTA).
- **PSRAM Speed:** Configure `80 MHz` QSPI if the module's PSRAM die supports it (verify on boot); fall back to `40 MHz` if PSRAM memtest fails (common on counterfeit/marginal clone boards).

---

## 4. MicroSD Slot — Why 1-Bit SDMMC Mode Is Mandatory

In ESP-IDF (`sdmmc_host` + `sdmmc_slot_config_t`), always configure **`slot_config.width = 1`**:

| Mode | GPIOs Used | Throughput (typical) | Problems Avoided |
| :--- | :--- | :---: | :--- |
| **4-bit SDMMC** | `2, 4, 12, 13, 14, 15` | ~4–8 MB/s | **Bad on ESP32-CAM:** strobes blinding `GPIO 4` flash LED on every read/write; `GPIO 12` (`MTDI`) card pull-up can brick boot at 1.8 V |
| **1-bit SDMMC (Recommended)** | **`2, 14, 15`** | ~1.5–3 MB/s (plenty for BalaX logs, `.bin.z`, `.wasm`) | Eliminates flash-LED strobing, frees `GPIO 4`, `12`, `13`, and avoids `MTDI` boot glitches |

**Mandatory post-mount hygiene:**
1. After mounting 1-bit SDMMC, explicitly configure `GPIO 4` as output `LOW` so the flash LED stays completely off.
2. Disable or carefully gate `GPIO 4` flash LED thermal bursts — the 1 W white LED draws **~200–300 mA** and heats the board rapidly.
3. **Flashing gotcha:** Because `GPIO 2` is SD `DATA0` and also a boot strap (must be LOW/floating during serial download), some MicroSD cards prevent entering the ROM bootloader while inserted. Pop the card out if USB serial flashing times out, or use **LAN OTA** once flashed.

---

## 5. I²S0 Parallel Camera DMA Pipeline

ESP32 classic has **no dedicated MIPI CSI or DVP peripheral** — Espressif's `esp32-camera` driver repurposes **I²S0** in parallel LCD/camera slave mode:

```
OV2640 (D0–D7, PCLK, HREF, VSYNC)
   │  (8-bit parallel DVP)
   ▼
ESP32 I²S0 Parallel Slave + Linked DMA Descriptors (in Internal DRAM)
   │  (DMA burst copies scanlines → PSRAM frame buffer)
   ▼
PSRAM Frame Buffers (fb_count = 2, CAMERA_GRAB_LATEST)
   │
   ▼
Core 0 HTTP MJPEG / Snapshot / SD Writer (zero CPU JPEG encoding!)
```

**Critical design rules:**
- **I²S0 is consumed:** You cannot use `I²S0` for audio while the camera is active (`I²S1` remains free, though no GPIOs are left to wire it cleanly).
- **Keep DMA ring buffers in internal DRAM:** DMA can only target `SRAM1`/`SRAM2` directly; `esp32-camera` uses an internal DRAM bounce buffer and a high-priority copy task (`cam_task`) to move chunks into PSRAM.
- **Avoid `EV-VSYNC-OVF` / `FB-OVF` (frame drops):** Caused when `PCLK` exceeds the rate `cam_task` can copy to PSRAM while Wi-Fi or SDMMC is also active. Fixes:
  1. Use **JPEG mode** (`PIXFORMAT_JPEG`) — 10–30× less data than RGB565/YUV!
  2. Drop `XCLK` from `20 MHz` to `12–16 MHz` if overflows occur.
  3. Use `fb_count = 2` with `grab_mode = CAMERA_GRAB_LATEST` so FPV never serves stale queued frames.
  4. Pin `cam_task` and Wi-Fi cleanly (e.g., camera DMA/copy on Core 1, Wi-Fi/HTTP/SD on Core 0).

---

## 6. Power, Brownout, Thermal & BalaX Companion Integration

### 6.1 Power & Brownout Traps
- **Current spikes:** Idle ~80–120 mA; active Wi-Fi + OV2640 streaming **~220–350 mA average**, with **500–600 mA+ Wi-Fi TX calibration spikes**.
- **AMS1117-3.3 dropout (~1.1 V):** Supplying < 4.5 V to the `5V` pin (thin jumper wires, weak USB port, or saggy battery) drops the 3.3 V rail below the ESP32 brownout threshold (`~2.43 V`), causing endless `Brownout detector was triggered` reset loops — **never disable the brownout detector to "hide" a bad power supply** (it corrupts flash/SD writes!).
- **AMS1117 heat:** At 300 mA from 5 V, the LDO burns $(5.0 - 3.3)\text{ V} \times 0.3\text{ A} = 0.51\text{ W}$ as pure heat, right next to the ESP32 and OV2640.
- **Camera power-down savings:** When camera streaming is idle (e.g., ESP32-CAM acting purely as a MicroSD storage/RPC server), driving **`GPIO 32` HIGH** turns off P-MOSFET `Q2` and completely powers down the OV2640 + its 2.8 V/1.2 V LDOs (~140 mW saved + cooler PCB!).

### 6.2 BalaX Companion Wiring & Thermal Rules (from `SPEC_OTA_STORAGE_PROGRAMS.md`)
- **Power:** Feed ESP32-CAM `5V` from the **BalaC base's external 5 V / 16340 battery rail** (or dedicated UBEC) with **common `GND`** — **NEVER power ESP32-CAM from the M5StickC Plus 120 mAh battery or Grove 5V!**
- **Mechanical air gap:** Do **not** tape the ESP32-CAM flat against the M5StickC Plus shell — leave an air gap so neither board traps the other's heat (StickC Plus has no battery thermistor!).
- **UART RPC link (1-bit SD safe):**
  - M5StickC Plus Grove **`G32` / `G33`** ↔ ESP32-CAM **`GPIO 3 (U0RX)` / `GPIO 1 (U0TX)`** + **`GND`** at up to `921600 baud`.
  - Disable default ROM/ESP-IDF UART0 console logging in production (`CONFIG_ESP_CONSOLE_NONE=y` or redirect logs to LAN/SD) so boot/debug text doesn't corrupt framed UART RPC packets.
- **Wi-Fi TX power cap:** Just like BalaX, cap Wi-Fi TX power (`+13 dBm` / `esp_wifi_set_max_tx_power(52)`) on LAN to cut peak current spikes, brownout risk, and PCB heat.
- **Antenna selector (`0 Ω` resistor):** AI-Thinker has both an onboard PCB antenna and an IPEX/U.FL connector; verify which pad the 0 Ω jumper bridges (factory default = PCB antenna).
