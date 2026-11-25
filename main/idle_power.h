#pragma once

// Pure C++ idle-power & thermal sleep policy (no ESP-IDF dependencies so host tests cover it).
//
// Two explicit low-power modes:
//   - WIFI_SLEEP ("wifi-sleep"): Wi-Fi connected low-power sleep (pauses 100 Hz loop, sleeps
//                IMU/LCD/EXTEN/mic, WIFI_PS_MAX_MODEM + automatic light sleep, faint slow LED pulse
//                every 4 s; wakes over Wi-Fi, USB console, or Button A).
//   - USB_SLEEP  ("usb-sleep"):  Deep sleep for USB-attached cool charging (~1-2 mA total, Wi-Fi/CPU/
//                LCD/IMU/EXTEN off, AXP192 DC-DC1 + 100 mA charger on). Wakes briefly every 10 s to
//                emit an even dimmer LED pulse and verify USB VBUS; if USB is unplugged, automatically
//                transitions to "wifi-sleep" so the robot stays reachable on battery!

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace idle_power {

constexpr float USB_PRESENT_MIN_V = 4.0f;
constexpr uint16_t DEFAULT_IDLE_SLEEP_S = 180;
constexpr uint16_t MAX_IDLE_SLEEP_S = 3600;
constexpr float THERMAL_PMIC_C = 46.0f;
constexpr float THERMAL_IMU_C = 58.0f;
constexpr int THERMAL_CONSECUTIVE_SAMPLES = 3;
constexpr int64_t MAX_MOTION_SAMPLE_AGE_US = 100'000;

constexpr bool fresh_stationary_sample(int64_t now_us, int64_t stationary_since_us, int64_t sampled_at_us) {
    return stationary_since_us > 0 && stationary_since_us <= sampled_at_us &&
           sampled_at_us <= now_us && now_us - sampled_at_us <= MAX_MOTION_SAMPLE_AGE_US;
}

// Subtle status LED heartbeat parameters (GPIO10 red LED, active-LOW).
// Microsecond pulses keep perceived brightness very faint without waking LEDC clocks.
constexpr uint32_t WIFI_SLEEP_LED_PERIOD_MS = 4000;
constexpr uint32_t WIFI_SLEEP_LED_PULSE_US = 1200;  // faint slow pulse in wifi-sleep
constexpr uint32_t USB_SLEEP_POLL_S = 10;           // slower 10 s cadence in usb-sleep
constexpr uint32_t USB_SLEEP_LED_PULSE_US = 250;    // ~5x dimmer whisper pulse in usb-sleep

enum class PowerMode : uint8_t {
    AWAKE,
    WIFI_SLEEP,
    USB_SLEEP,
};

constexpr const char* name(PowerMode mode) {
    switch (mode) {
        case PowerMode::AWAKE:      return "awake";
        case PowerMode::WIFI_SLEEP: return "wifi-sleep";
        case PowerMode::USB_SLEEP:  return "usb-sleep";
    }
    return "unknown";
}

struct GuardResult {
    bool ok = false;
    const char* reason = nullptr;
};

constexpr bool usb_present(float usb_v) {
    return std::isfinite(usb_v) && usb_v >= USB_PRESENT_MIN_V;
}

// While in usb-sleep, unplugging USB automatically transitions to wifi-sleep unless forced.
constexpr bool should_switch_usb_sleep_to_wifi(float usb_v, bool forced) {
    return !forced && !usb_present(usb_v);
}

// While in wifi-sleep, plugging in USB automatically transitions to cool-charge usb-sleep
// after WIFI_SLEEP_USB_DEBOUNCE consecutive samples with usb_v >= 4.0 V (unless forced).
constexpr int WIFI_SLEEP_USB_DEBOUNCE = 2;
inline bool should_switch_wifi_sleep_to_usb(float usb_v, bool forced, int& consecutive_usb_samples) {
    if (forced || !usb_present(usb_v)) {
        consecutive_usb_samples = 0;
        return false;
    }
    return ++consecutive_usb_samples >= WIFI_SLEEP_USB_DEBOUNCE;
}

inline GuardResult can_enter_sleep(bool motors_live, bool force = false) {
    if (motors_live && !force) {
        return {false, "Refusing wifi-sleep while balancing; disarm first (or pass 'wifi-sleep force')."};
    }
    return {true, nullptr};
}

inline GuardResult can_enter_off(bool motors_live, float usb_v, bool force = false) {
    if (motors_live && !force) {
        return {false, "Refusing usb-sleep while balancing; disarm first (or pass 'usb-sleep force')."};
    }
    if (!force && !usb_present(usb_v)) {
        return {
            false,
            "Refusing usb-sleep: USB power not detected. Plug in USB for usb-sleep, use 'wifi-sleep', or pass 'usb-sleep force'.",
        };
    }
    return {true, nullptr};
}

enum class AutoAction : uint8_t {
    NONE,
    IDLE_SLEEP,
    THERMAL_SLEEP,
};

struct PolicyInputs {
    bool sleeping = false;
    bool motors_live = false;
    bool armed = false;
    bool state_ready = false;
    bool stream_active = false;
    bool remote_live = false;
    bool prog_running = false;
    int64_t stationary_since_us = 0;
    int64_t sampled_at_us = 0;
    float pmic_c = NAN;
    float imu_c = NAN;
    uint32_t idle_sleep_s = DEFAULT_IDLE_SLEEP_S;
};

class IdlePolicy {
public:
    explicit IdlePolicy(int64_t initial_us = 0) : last_activity_us_(initial_us) {}

    void note_activity(int64_t now_us) {
        last_activity_us_ = now_us;
    }

    int64_t idle_seconds(int64_t now_us) const {
        if (now_us <= last_activity_us_) {
            return 0;
        }
        return (now_us - last_activity_us_) / 1'000'000LL;
    }

    int hot_samples() const { return hot_samples_; }

    AutoAction evaluate(int64_t now_us, const PolicyInputs& in) {
        if (in.sleeping) {
            hot_samples_ = 0;
            return AutoAction::NONE;
        }

        if (in.motors_live || in.armed || !fresh_stationary_sample(now_us, in.stationary_since_us, in.sampled_at_us)) {
            hot_samples_ = 0;
            note_activity(now_us);
            return AutoAction::NONE;
        }
        last_activity_us_ = std::max(last_activity_us_, in.stationary_since_us);

        const bool over_temp =
            (std::isfinite(in.pmic_c) && in.pmic_c >= THERMAL_PMIC_C) ||
            (std::isfinite(in.imu_c) && in.imu_c >= THERMAL_IMU_C);
        if (over_temp) {
            // Motion can start and stop between two status polls.
            if (in.stationary_since_us > last_hot_sample_us_) hot_samples_ = 0;
            last_hot_sample_us_ = now_us;
            if (++hot_samples_ >= THERMAL_CONSECUTIVE_SAMPLES) {
                hot_samples_ = 0;
                return AutoAction::THERMAL_SLEEP;
            }
        } else {
            hot_samples_ = 0;
        }

        if (!in.state_ready || in.stream_active || in.remote_live || in.prog_running) {
            last_activity_us_ = now_us;
            return AutoAction::NONE;
        }

        if (in.idle_sleep_s > 0) {
            const int64_t timeout_us = static_cast<int64_t>(in.idle_sleep_s) * 1'000'000LL;
            if (now_us - last_activity_us_ >= timeout_us) {
                return AutoAction::IDLE_SLEEP;
            }
        }
        return AutoAction::NONE;
    }

private:
    int64_t last_activity_us_ = 0;
    int64_t last_hot_sample_us_ = 0;
    int hot_samples_ = 0;
};

}  // namespace idle_power
