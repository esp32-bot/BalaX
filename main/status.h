#pragma once

// Live readings: temperatures, power, CPU load, memory, uptime, balancing, Wi-Fi, flash wear, and VM cache.
// Renders the LCD in small strips, sharing scratch space with the Micro-BIOS primitives.

#include <cstdint>

#include "esp_err.h"
#include "status_values.h"

// Samples attitude each STATUS_POLL_MS and system readings each STATUS_PERIOD_MS.
// Display updates pause while the backlight is off, sleeping, or OTA/BXVM owns the LCD.
esp_err_t start_status_screen();
// The readings as last shown; safe from any task.
status::Values latest_status();
// Reads everything afresh and prints it as `name value` lines.
void print_status();
// Probes one or more named live metrics (e.g. `probe cpu_mhz cpu0_load pmic_c`), or `list`.
int print_probe(int argc, char** argv);

namespace status_lcd {

// Temporarily pauses the status screen redraws while OTA or a mini-program owns the LCD.
void set_busy(bool busy);
// Marks the LCD dirty and wakes status_task (used when waking from Wi-Fi sleep).
void request_repaint();
void notify_task();
// Thread-safe Micro-BIOS primitives routed through board.display.fill_rect.
void clear(uint16_t rgb565);
void fill_rect(int x, int y, int w, int h, uint16_t rgb565);
void draw_text(int x, int y, const char* text, uint16_t fg, uint16_t bg, int scale);
void draw_bar(int x, int y, int w, int h, int pct, uint16_t fg, uint16_t bg);
// Convenience 0..100% OTA progress banner + bar using the zero-framebuffer Micro-BIOS.
void draw_ota_progress(const char* label, int pct, const char* detail = nullptr);

}  // namespace status_lcd
