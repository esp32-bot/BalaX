#include <string_view>
// Laptop tests for idle_power.h (sleep/off safety guards, USB requirement, auto-idle & thermal sleep).

#include "check.h"
#include "idle_power.h"

namespace {

using namespace idle_power;

AutoAction evaluate(IdlePolicy& policy, int64_t seconds, PolicyInputs& in) {
    const int64_t now_us = seconds * 1'000'000LL;
    in.sampled_at_us = now_us;
    return policy.evaluate(now_us, in);
}

TEST(sleep_and_off_safety_guards) {
    // Sleep allowed when disarmed, blocked while balancing unless forced.
    CHECK(can_enter_sleep(false, false).ok);
    CHECK(!can_enter_sleep(true, false).ok);
    CHECK(can_enter_sleep(true, true).ok);

    // Off requires USB power (>= 4.0 V) AND not balancing unless forced.
    CHECK(can_enter_off(false, 5.05f, false).ok);
    CHECK(can_enter_off(false, 4.0f, false).ok);
    CHECK(!can_enter_off(false, 3.8f, false).ok);
    CHECK(!can_enter_off(false, 0.0f, false).ok);
    CHECK(!can_enter_off(false, NAN, false).ok);
    CHECK(!can_enter_off(true, 5.1f, false).ok);
    CHECK(can_enter_off(false, 0.0f, true).ok);
    CHECK(can_enter_off(true, 0.0f, true).ok);
}

TEST(idle_auto_sleep_timeout_and_activity_reset) {
    IdlePolicy policy(1'000'000);
    PolicyInputs in{};
    in.state_ready = true;
    in.stationary_since_us = 1'000'000;
    in.idle_sleep_s = 180;
    in.pmic_c = 34.0f;
    in.imu_c = 42.0f;

    // Not idle long enough at 179 s.
    CHECK(evaluate(policy, 179, in) == AutoAction::NONE);
    CHECK(policy.idle_seconds(179'000'000LL) == 178);

    // Activity resets timer.
    policy.note_activity(179'000'000LL);
    CHECK(evaluate(policy, 250, in) == AutoAction::NONE);
    CHECK(policy.idle_seconds(250'000'000LL) == 71);

    // Active telemetry stream keeps robot awake.
    in.stream_active = true;
    CHECK(evaluate(policy, 500, in) == AutoAction::NONE);
    in.stream_active = false;
    CHECK(policy.idle_seconds(500'000'000LL) == 0);

    // After 180 s of quiet READY state -> IDLE_SLEEP.
    CHECK(evaluate(policy, 679, in) == AutoAction::NONE);
    CHECK(evaluate(policy, 680, in) == AutoAction::IDLE_SLEEP);

    // Timeout 0 disables auto-idle sleep.
    in.idle_sleep_s = 0;
    CHECK(evaluate(policy, 999, in) == AutoAction::NONE);
}

TEST(auto_sleep_requires_uninterrupted_disarmed_stationary_interval) {
    IdlePolicy policy;
    PolicyInputs in{};
    in.state_ready = true;
    in.idle_sleep_s = 10;
    in.stationary_since_us = 1'000'000;
    CHECK(evaluate(policy, 10, in) == AutoAction::NONE);

    // A brief movement between polls starts the full timer over.
    in.stationary_since_us = 10'500'000;
    CHECK(evaluate(policy, 11, in) == AutoAction::NONE);
    CHECK(evaluate(policy, 20, in) == AutoAction::NONE);
    CHECK(evaluate(policy, 21, in) == AutoAction::IDLE_SLEEP);

    // A stalled loop or failed IMU read cannot qualify as stationary.
    CHECK(policy.evaluate(22'000'000, in) == AutoAction::NONE);
    in.stationary_since_us = 0;
    CHECK(evaluate(policy, 32, in) == AutoAction::NONE);
    in.stationary_since_us = 33'000'000;
    in.armed = true;
    CHECK(evaluate(policy, 43, in) == AutoAction::NONE);
}

TEST(thermal_sleep_debounce_and_balance_safety) {
    IdlePolicy policy(0);
    PolicyInputs in{};
    in.state_ready = true;
    in.stationary_since_us = 1'000'000;
    in.idle_sleep_s = 0;
    in.pmic_c = 46.5f;
    in.imu_c = 50.0f;

    // Even armed states whose motors are off block thermal sleep.
    in.armed = true;
    for (int i = 0; i < 5; ++i) {
        CHECK(evaluate(policy, i + 1, in) == AutoAction::NONE);
    }

    // When disarmed, requires 3 consecutive hot samples before sleeping.
    in.armed = false;
    in.stationary_since_us = 10'000'000;
    CHECK(evaluate(policy, 10, in) == AutoAction::NONE);
    CHECK(evaluate(policy, 11, in) == AutoAction::NONE);
    // Transient cool sample resets counter.
    in.pmic_c = 40.0f;
    CHECK(evaluate(policy, 12, in) == AutoAction::NONE);
    CHECK(policy.hot_samples() == 0);

    // 3 consecutive hot IMU readings trigger THERMAL_SLEEP.
    in.imu_c = 58.2f;
    CHECK(evaluate(policy, 13, in) == AutoAction::NONE);
    in.stationary_since_us = 13'500'000;
    CHECK(evaluate(policy, 14, in) == AutoAction::NONE);
    CHECK(evaluate(policy, 15, in) == AutoAction::NONE);
    CHECK(evaluate(policy, 16, in) == AutoAction::THERMAL_SLEEP);

    // Once sleeping, evaluate returns NONE.
    in.sleeping = true;
    CHECK(evaluate(policy, 17, in) == AutoAction::NONE);
}


TEST(power_mode_names_and_usb_unplug_handoff) {
    CHECK(std::string_view(name(PowerMode::AWAKE)) == "awake");
    CHECK(std::string_view(name(PowerMode::WIFI_SLEEP)) == "wifi-sleep");
    CHECK(std::string_view(name(PowerMode::USB_SLEEP)) == "usb-sleep");

    // USB still plugged -> stay in usb-sleep
    CHECK(!should_switch_usb_sleep_to_wifi(5.0f, false));
    CHECK(!should_switch_usb_sleep_to_wifi(4.0f, false));
    // USB unplugged -> automatically switch to wifi-sleep!
    CHECK(should_switch_usb_sleep_to_wifi(3.5f, false));
    CHECK(should_switch_usb_sleep_to_wifi(0.0f, false));
    CHECK(should_switch_usb_sleep_to_wifi(NAN, false));
    // Forced usb-sleep stays in usb-sleep even without USB
    CHECK(!should_switch_usb_sleep_to_wifi(0.0f, true));

    // Bidirectional: plugging in USB while in wifi-sleep switches to usb-sleep after debounce!
    int usb_ticks = 0;
    CHECK(!should_switch_wifi_sleep_to_usb(0.0f, false, usb_ticks));
    CHECK(usb_ticks == 0);
    CHECK(!should_switch_wifi_sleep_to_usb(5.05f, false, usb_ticks));  // sample 1 (debounce)
    CHECK(usb_ticks == 1);
    CHECK(!should_switch_wifi_sleep_to_usb(0.5f, false, usb_ticks));   // glitch resets counter
    CHECK(usb_ticks == 0);
    CHECK(!should_switch_wifi_sleep_to_usb(5.0f, false, usb_ticks));   // sample 1
    CHECK(should_switch_wifi_sleep_to_usb(5.0f, false, usb_ticks));    // sample 2 -> switch!
    // Forced wifi-sleep stays in wifi-sleep even when USB is plugged in
    CHECK(!should_switch_wifi_sleep_to_usb(5.0f, true, usb_ticks));
    CHECK(usb_ticks == 0);

    // LED cadence: usb-sleep must be slower and dimmer than wifi-sleep
    CHECK(USB_SLEEP_POLL_S * 1000 > WIFI_SLEEP_LED_PERIOD_MS);
    CHECK(USB_SLEEP_LED_PULSE_US < WIFI_SLEEP_LED_PULSE_US);
}

}  // namespace
