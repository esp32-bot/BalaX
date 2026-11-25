#pragma once

// Named status screen states, shared by `bx preview`, `bx frames` and the live preview.
// The Wi-Fi network and address are made up: 192.0.2.0/24 is reserved for documentation.

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>

#include "battery_soc.h"
#include "status_view.h"

namespace fixtures {

struct Fixture {
    const char* id;
    status::Values values;
    float pitch_deg = NAN;
    float roll_deg = NAN;
    uint32_t flash_erases = 0;
    uint32_t flash_skipped = 0;
    uint32_t vm_hits = 0;
    bool vm_running = false;

    status::ScreenValues screen() const { return screen(pitch_deg, roll_deg); }
    // The same state at another attitude, for the live preview's sliders and the parity sweep.
    status::ScreenValues screen(float pitch, float roll) const {
        return {values, pitch, roll, flash_erases, flash_skipped, vm_hits, vm_running};
    }
};

inline status::Values charging() {
    status::Values v;
    v.pmic_c = 44.5f;
    v.imu_c = 53.5f;
    v.battery_v = 3.97f;
    v.battery_ma = 83;
    v.usb_v = 5.10f;
    v.usb_ma = 120;
    v.cpu_mhz = 80;
    v.cpu0_load = 0.3f;
    v.cpu1_load = 0;
    v.ram_used_kb = 92;
    v.ram_total_kb = 300;
    v.uptime_s = 83;
    v.brightness_pct = 50;
    v.volume_pct = 70;
    v.state = balance::State::READY;
    v.pitch_deg = -88.5f;
    v.motor_left = 0;
    v.motor_right = 0;
    v.tick_ms = 0.31f;
    v.wifi_phase = "connected";
    snprintf(v.wifi_ssid, sizeof v.wifi_ssid, "Lab 2.4GHz");
    snprintf(v.wifi_ip, sizeof v.wifi_ip, "192.0.2.10");
    v.wifi_rssi = -54;
    v.wifi_channel = 6;
    snprintf(v.ap_ssid, sizeof v.ap_ssid, "BalaX");
    v.battery_pct = battery::percent(v.battery_v, v.battery_ma);
    return v;
}

inline status::Values on_battery() {
    status::Values v = charging();
    v.battery_v = 3.80f;
    v.battery_ma = -65;
    v.usb_v = 0;
    v.usb_ma = 0;
    v.cpu_mhz = 80;
    v.cpu0_load = 35.2f;
    v.cpu1_load = 88.4f;
    v.ram_used_kb = 231;
    v.uptime_s = 3725;
    v.state = balance::State::BALANCING;
    v.pitch_deg = 1.3f;
    v.motor_left = 45;
    v.motor_right = -38;
    v.tick_ms = 0.42f;
    v.remote_live = true;
    v.remote_forward = 0.5f;
    v.remote_turn = -0.3f;
    v.battery_pct = battery::percent(v.battery_v, v.battery_ma);
    return v;
}

inline status::Values low_and_failing() {
    status::Values v = on_battery();
    v.battery_v = 3.62f;
    v.battery_ma = -140;
    v.imu_c = NAN;  // a sensor that stopped answering
    v.uptime_s = 98765;
    v.state = balance::State::FALLEN;
    v.fault = balance::Fault::TILT;
    v.pitch_deg = 41.2f;
    v.motor_left = NAN;  // the base switched off
    v.motor_right = NAN;
    v.wifi_phase = "none in range";
    v.wifi_ssid[0] = '\0';
    v.wifi_channel = 1;
    v.remote_live = false;
    v.battery_pct = battery::percent(v.battery_v, v.battery_ma);
    return v;
}

// In the order the preview lays them out. `boot` is a screen before any reading arrives.
inline const std::array<Fixture, 4>& all() {
    static const std::array<Fixture, 4> list = {{
        {.id = "boot", .values = {}},
        {.id = "charging", .values = charging(), .pitch_deg = -18, .roll_deg = 0},
        {.id = "balancing", .values = on_battery(), .pitch_deg = 1.3f, .roll_deg = 12,
         .flash_erases = 250, .flash_skipped = 6, .vm_hits = 3},
        {.id = "fallen", .values = low_and_failing(), .pitch_deg = 26, .roll_deg = -25,
         .flash_erases = 250, .flash_skipped = 6, .vm_hits = 3},
    }};
    return list;
}

}  // namespace fixtures
