#pragma once

// Everything the robot plays. The speaker driver holds no sound data itself.

#include <cstdint>
#include <span>
#include <string_view>

#include "m5stickc_plus/speaker.h"

inline constexpr m5::Note STARTUP_MELODY[] = {
    {1200, 50}, {1500, 50}, {1200, 50}, {1500, 50}, {1200, 50},
    {0, 200},
    {1000, 200}, {1200, 200}, {1400, 600},
};

inline constexpr m5::Note SHUTDOWN_MELODY[] = {
    {1400, 200}, {1200, 200}, {1000, 400},
};

// What `beep` plays without arguments.
inline constexpr m5::Note DEFAULT_BEEP = {2000, 150};

// Repeated after a fall until it's acknowledged.
inline constexpr m5::Note ALARM_BEEP = {911, 50};

constexpr uint32_t duration_ms(std::span<const m5::Note> notes) {
    uint32_t ms = 0;
    for (const m5::Note& note : notes) {
        ms += note.ms;
    }
    return ms;
}

struct Melody {
    std::string_view name;
    std::span<const m5::Note> notes;
};

// What `play` can play, by name.
inline constexpr Melody MELODIES[] = {
    {"startup", STARTUP_MELODY},
    {"shutdown", SHUTDOWN_MELODY},
};
