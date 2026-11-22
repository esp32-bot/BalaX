# BalaX Silicon Reference (`docs/silicon/`)

Hardware ground truth for the silicon on **M5StickC Plus** (and **BalaC** base), **AI-Thinker ESP32-CAM**, and the **OmniVision OV2640** image sensor. Read the relevant guide before touching pin maps, DMA/SRAM/PSRAM buffers, clocks, I²C/SPI/I²S/SDMMC peripherals, power rails, thermal budgets, or camera pipelines.

## Guides

| Document | Hardware Covered | Read When Working On |
| :--- | :--- | :--- |
| [**`M5STICKC_PLUS.md`**](M5STICKC_PLUS.md) | **ESP32-PICO-D4** SiP, **AXP192** PMIC, **MPU6886** 6-axis IMU, **ST7789V2** LCD, **BM8563** RTC, **SPM1423** mic, **BalaC** base (**STM32F030F4P6** + 2× **L9110S**) | Core 1 balance loop, IMU fusion/calibration, SRAM/IRAM/DMA budgets, thermal/LiPo safety, display/buzzer/buttons, HAT/Grove wiring, JoyX (same StickC Plus board) |
| [**`ESP32_CAM.md`**](ESP32_CAM.md) | **ESP32-S** (`ESP32-D0WDQ6` / `D0WD-V3`), external **4 MB / 8 MB SPI PSRAM**, **MicroSD** (SDMMC Slot 1), **I²S0** parallel camera DMA, **AMS1117-3.3** & camera LDO power tree | `balax-cam.local` companion firmware, MicroSD telemetry/artifact storage, UART RPC bridge to BalaX, PSRAM frame buffers, avoiding pin conflicts & brownouts |
| [**`OV2640.md`**](OV2640.md) | **OmniVision OV2640** 2 MP UXGA CMOS sensor, dual-bank **SCCB** (`0x30` / `0xFF`), on-chip **DSP & hardware JPEG engine**, **DVP** clocking (`XCLK`/`PCLK`), rolling shutter | Live FPV streaming, JPEG quality/FPS/FOV tuning, stopping `FB-OVF` / `EV-VSYNC-OVF` drops, exposure/gain locking for robotics vision, browser offload |

See also [`../HARDWARE.md`](../HARDWARE.md) for BalaX runtime thermal thresholds, Wi-Fi defaults, and JoyX ESP-NOW wire framing, and [`../SPEC_OTA_STORAGE_PROGRAMS.md`](../SPEC_OTA_STORAGE_PROGRAMS.md) for the Thin-Robot Offload & CAM SD companion architecture.

---

## Side-by-Side Silicon Comparison (StickC Plus vs. ESP32-CAM)

| Property | M5StickC Plus (`K016-P`) | AI-Thinker ESP32-CAM (Rev 1.6) |
| :--- | :--- | :--- |
| **SoC / Module** | `ESP32-PICO-D4` SiP (rev v1.0+, 40 MHz XTAL) | `ESP32-S` (`ESP32-D0WDQ6` / `D0WD-V3`, 40 MHz XTAL) |
| **CPU / Operating Freq** | Dual Xtensa LX6 (max 240 MHz; **80 MHz** in BalaX) | Dual Xtensa LX6 (max 240 MHz; 160–240 MHz typical for cam/Wi-Fi) |
| **Internal SRAM** | 520 KB (192 KB SRAM0 / 128 KB SRAM1 / 200 KB SRAM2; 328 KB DMA-capable) | Identical 520 KB internal SRAM architecture |
| **SPI Flash** | 4 MB integrated on **GPIO 6, 7, 8, 11, 16, 17** | 4 MB inside ESP32-S on **GPIO 6–11** |
| **External PSRAM** | **None** | **4 MB mapped** (`0x3F80_0000–0x3FBF_FFFF`, 64 Mbit die) on **GPIO 16 (CS#) & GPIO 17 (CLK)** |
| **Free GPIOs (typical)** | Grove `G32`/`G33`, HAT `G0`/`G26`/`G36+G25` | Very scarce: `U0TX`/`U0RX` (`G1`/`G3`), plus `G13`/`G12`/`G4` **only in 1-bit SD mode** |
| **Power & Thermal** | **AXP192** PMIC + sealed **120 mAh** LiPo (no thermistor!) | **AMS1117-3.3** LDO (1.1 V dropout) + external 5 V ≥ 500–600 mA peak |
| **USB-UART** | Onboard bridge (CH9102F / FT232) | **None** (needs external FTDI / `ESP32-CAM-MB` + `GPIO0` LOW to flash) |
| **Role in BalaX** | Deterministic 100 Hz balance/IMU + minimal LAN/ESP-NOW | Optional companion: hardware JPEG stream, GB-scale MicroSD storage, UART RPC |

---

## Top 12 Silicon Traps (Read Before Coding)

1. **Swapped Flash Pins (`ESP32-PICO-D4` vs. `ESP32-S`):** On M5StickC Plus (`PICO-D4`), **GPIO 16 & 17 are wired to internal flash** (never touch them!), while **GPIO 9 & 10 are free** (used for IR and red LED). On ESP32-CAM (`ESP32-S`), **GPIO 6–11 are internal flash**, and **GPIO 16 & 17 are PSRAM CS#/CLK** (never use header pin `IO16` when PSRAM is enabled!).
2. **Flash Writes Disable Cache on BOTH Cores:** Any SPI flash write/erase (NVS, OTA, LittleFS, telemetry spillover) stalls flash XIP and PSRAM access on both cores. Never write flash while Core 1 is balancing, and never let `ESP_INTR_FLAG_IRAM` ISRs touch flash constants or PSRAM pointers (`Cache disabled but cached memory region accessed`).
3. **ADC2 Cannot Run With Wi-Fi:** On all ESP32 classic silicon, Wi-Fi preempts **ADC2** (`GPIO 0, 2, 4, 12–15, 25–27`). Only **ADC1** (`GPIO 32–36, 39`) works concurrently with Wi-Fi.
4. **Erratum `[GPIO-3.11]` (80 ns Glitch on `GPIO36` & `GPIO39`):** Powering on SAR ADC or Wi-Fi modem-sleep pulls `GPIO36` and `GPIO39` low for ~80 ns on **all ESP32 revisions**. Never attach edge interrupts to `GPIO39` (StickC Plus Button B) or `GPIO36` (HAT header) while Wi-Fi is active; always poll/debounce.
5. **M5StickC Plus HAT `G36/G25` Shared Pin:** Both `GPIO36` and `GPIO25` are tied to one physical HAT pad. Whichever one you are not using **must** be left as a floating input (no pull-up/pull-down).
6. **M5StickC Plus `GPIO0` Triple Duty:** `GPIO0` is simultaneously the ESP32 boot strap, the onboard `SPM1423` mic clock, and **HAT I²C SDA** (`BalaC` motors). Do not enable the PDM mic while using the BalaC base.
7. **M5StickC Plus Sealed 120 mAh LiPo Has NO Thermistor:** Never raise AXP192 charge current above `100 mA` (`reg 0x33 = 0xC0`), never power an ESP32-CAM from the StickC battery, and enforce `pmic_c` / `imu_c` thermal ceilings.
8. **StickC Plus ≠ StickC Plus2:** Never mix schematics. Plus (`K016-P`) has `ESP32-PICO-D4` + `AXP192`; Plus2 has `ESP32-PICO-V3-02` + no AXP192 + moved LCD/LED pins.
9. **ESP32-CAM Always Needs 1-Bit SDMMC Mode:** 4-bit SDMMC uses `GPIO 4` (flashes the blinding white LED on every SD access!) and `GPIO 12` (`MTDI` boot strap — pulling HIGH selects 1.8 V flash and bricks boot!). Always initialize SDMMC in **1-bit mode** (`GPIO 2, 14, 15`), and beware `GPIO 2` SD pull-ups blocking serial download mode.
10. **ESP32-CAM `GPIO32` Is Active-LOW Power Gate:** `GPIO32` drives P-MOSFET `Q2` feeding the OV2640's `2.8 V` and `1.2 V` LDOs (`LOW` = powered ON, `HIGH` = OFF).
11. **OV2640 Won't Answer SCCB Without `XCLK`:** The OV2640 requires an active `XCLK` (6–24 MHz on `GPIO0`) before responding on SCCB (`0x30`), and its registers are split into two banks switched via `0xFF` (`0x00` = DSP, `0x01` = Sensor).
12. **ESP32 I²S0 DMA Bandwidth Ceiling & Rolling Shutter:** OV2640 can output up to 36 MHz `PCLK`, but ESP32 classic I²S0 DMA → PSRAM saturates around 16–20 MHz `PCLK` (`EV-VSYNC-OVF` / torn JPEGs). Always use **OV2640 hardware JPEG**, `fb_count = 2` + `CAMERA_GRAB_LATEST`, `QS = 10–16`, and offload computer vision to `balax-remote` in the browser.
