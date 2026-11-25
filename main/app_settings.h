#pragma once

// The robot's runtime settings; see settings.h for how to change them from the console.

#include <cstdint>

#include "settings.h"

extern settings::Setting<uint8_t> volume;
extern settings::Setting<uint8_t> brightness;
extern settings::Setting<uint16_t> idle_sleep_s;
