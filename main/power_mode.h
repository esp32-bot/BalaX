#pragma once

#include <cstdint>

#include "esp_err.h"
#include "idle_power.h"
#include "robot.h"
#include "status_values.h"

namespace power_mode {

// Call at the VERY START of app_main() before board/Wi-Fi/LCD init.
// If waking from the 10 s usb-sleep RTC timer:
//   - While USB is still plugged in: emits an ultra-dim 0.25 ms LED pulse and re-enters deep sleep.
//   - If USB was unplugged: sets boot_to_wifi_sleep() so app_main() boots straight into wifi-sleep!
void handle_usb_sleep_wakeup();
bool boot_to_wifi_sleep();

esp_err_t init();

idle_power::PowerMode current();
bool is_sleeping();
int64_t idle_seconds();

// Marks active use (resets auto-idle sleep timer). Does not wake a sleeping robot on its own.
void note_activity();

// Enter Wi-Fi connected low-power sleep ("wifi-sleep"). Refuses while motors are live unless force=true.
idle_power::GuardResult enter_sleep(bool force = false);

// Wake from wifi-sleep back to full 100 Hz awake mode. Idempotent when already awake.
void wake();

// Called from Core 1 when Button A is pressed during wifi-sleep; defers full wake to Core 0.
void request_wake_from_button();

// Emit a faint slow heartbeat flash on the red LED while in wifi-sleep.
void pulse_wifi_sleep_led();

// Schedule USB-charge deep sleep ("usb-sleep"). Refuses while motors are live or when USB power is
// absent unless force=true. Sends the caller's response cleanly before stopping Wi-Fi and CPU.
idle_power::GuardResult enter_off(bool force = false);

// Called at ~1 Hz (or 0.5 Hz while sleeping) from status_task on Core 0.
void poll_from_status_task(const status::Values& status, const robot::Telemetry& robot, bool stream_active,
                           bool prog_running);

}  // namespace power_mode
