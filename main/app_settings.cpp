#include "app_settings.h"

#include "board.h"
#include "config.h"
#include "idle_power.h"
#include "power_mode.h"

settings::Setting<uint8_t> volume(
    "volume", DEFAULT_VOLUME, 0, m5::Speaker::MAX_VOLUME, "Buzzer, 0 mutes; 10 steps = 6 dB",
    [](uint8_t value) { board.speaker.set_volume(value); });

settings::Setting<uint8_t> brightness(
    "brightness", DEFAULT_BRIGHTNESS, 0, m5::Axp192::MAX_BACKLIGHT, "Backlight, 0 turns it off",
    [](uint8_t value) { board.pmic.set_backlight(power_mode::is_sleeping() ? 0 : value); });

settings::Setting<uint16_t> idle_sleep_s(
    "idle_sleep_s", idle_power::DEFAULT_IDLE_SLEEP_S, 0, idle_power::MAX_IDLE_SLEEP_S,
    "Auto Wi-Fi sleep after disarmed, stationary idle seconds (0 disables)");
