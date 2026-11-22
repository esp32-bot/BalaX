#pragma once

// The robot's live readings, as the status screen shows them and the console prints them.
// NaN means a reading isn't available, and empty text that there's nothing to show. Plain
// C++, so laptop builds can use it.

#include <cmath>

#include "balance.h"

namespace status {

inline constexpr int SSID_SIZE = 33;  // 32 bytes and a terminator
inline constexpr int IP_SIZE = 16;    // "255.255.255.255"

struct Values {
    float pmic_c = NAN;
    float imu_c = NAN;
    float battery_v = NAN;
    float battery_ma = NAN;  // positive while charging
    float battery_pct = NAN;
    float usb_v = NAN;
    float usb_ma = NAN;
    float cpu_mhz = NAN;
    float cpu0_load = NAN;
    float cpu1_load = NAN;
    float ram_used_kb = NAN;  // of the heap, what the firmware can allocate
    float ram_total_kb = NAN;
    float uptime_s = NAN;
    float brightness_pct = NAN;
    float volume_pct = NAN;

    // Balancing.
    balance::State state = balance::State::CALIBRATING;
    balance::Fault fault = balance::Fault::NONE;
    float pitch_deg = NAN;
    float motor_left = NAN;  // power, -127..127, forward positive; NaN while the base doesn't answer
    float motor_right = NAN;
    float tick_ms = NAN;     // the control loop's longest tick in the last second
    float overruns = NAN;    // control loop ticks that ran late, since startup

    // Wi-Fi.
    const char* wifi_phase = "";     // "searching", "connected", ...
    char wifi_ssid[SSID_SIZE] = "";  // the network it's on
    char wifi_ip[IP_SIZE] = "";
    float wifi_rssi = NAN;
    float wifi_channel = NAN;
    char ap_ssid[SSID_SIZE] = "";
    float ap_clients = NAN;

    // The remote.
    bool remote_live = false;
    float remote_forward = NAN;  // the stick, -1..1
    float remote_turn = NAN;
    float remote_messages = NAN;
    float remote_failed = NAN;  // replies that didn't get through
};

// How the console prints each number.
struct Field {
    const char* name;
    int decimals;
    float Values::*value;
};

inline constexpr Field FIELDS[] = {
    {"pmic_c", 1, &Values::pmic_c},
    {"imu_c", 1, &Values::imu_c},
    {"battery_v", 2, &Values::battery_v},
    {"battery_ma", 0, &Values::battery_ma},
    {"battery_pct", 0, &Values::battery_pct},
    {"usb_v", 2, &Values::usb_v},
    {"usb_ma", 0, &Values::usb_ma},
    {"cpu_mhz", 0, &Values::cpu_mhz},
    {"cpu0_load", 1, &Values::cpu0_load},
    {"cpu1_load", 1, &Values::cpu1_load},
    {"ram_used_kb", 0, &Values::ram_used_kb},
    {"ram_total_kb", 0, &Values::ram_total_kb},
    {"uptime_s", 0, &Values::uptime_s},
    {"pitch_deg", 1, &Values::pitch_deg},
    {"motor_left", 0, &Values::motor_left},
    {"motor_right", 0, &Values::motor_right},
    {"tick_ms", 2, &Values::tick_ms},
    {"overruns", 0, &Values::overruns},
    {"wifi_rssi", 0, &Values::wifi_rssi},
    {"wifi_channel", 0, &Values::wifi_channel},
    {"ap_clients", 0, &Values::ap_clients},
    {"remote_msgs", 0, &Values::remote_messages},
    {"remote_failed", 0, &Values::remote_failed},
};

}  // namespace status
