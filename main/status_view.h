#pragma once

// Shared by the robot LCD and the host preview. The robot renders one strip at a time.
#include <cstdint>
#include <span>

#include "status_values.h"

namespace status {

struct ScreenValues {
    const Values& status;
    float pitch_deg = NAN;
    float roll_deg = NAN;
    uint32_t flash_erases = 0;
    uint32_t flash_skipped = 0;
    uint32_t vm_hits = 0;
    bool vm_running = false;
};

inline constexpr int STRIP_ROWS = 10;
inline constexpr int ATTITUDE_ROWS = 120;

// Renders whole rows starting at first_row, into a screen-width RGB565 pixel span.
void draw(std::span<uint16_t> pixels, int first_row, const ScreenValues& values);

}  // namespace status
