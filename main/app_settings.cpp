#include "app_settings.h"

#include "board.h"
#include "config.h"

settings::Setting<uint8_t> volume(
    "volume", DEFAULT_VOLUME, 0, m5::Speaker::MAX_VOLUME, "Buzzer, 0 mutes; 10 steps = 6 dB",
    [](uint8_t value) { board.speaker.set_volume(value); });

settings::Setting<uint8_t> brightness(
    "brightness", DEFAULT_BRIGHTNESS, 0, m5::Axp192::MAX_BACKLIGHT, "Backlight, 0 turns it off",
    [](uint8_t value) { board.pmic.set_backlight(value); });
