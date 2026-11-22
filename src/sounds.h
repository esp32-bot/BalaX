#ifndef _SOUNDS_H
#define _SOUNDS_H

// Everything the robot plays: named tones and melodies.
// The speaker driver (speaker.h) plays them; it holds no sound data itself.

#include "speaker.h"

// Abort alarm, beeped until the abort is acknowledged.
constexpr uint16_t TONE_ABORT_HZ = 911;

// IMU calibration: held while a side is being calibrated...
constexpr uint16_t TONE_CALIBRATING_SIDE_HZ = 1200;
// ...then raised while that side's samples are captured.
constexpr uint16_t TONE_SIDE_ALIGNED_HZ = 1400;
// Base pitch of the guide tone while turning the robot onto an uncalibrated side;
// it rises the further the reading is past the alignment threshold.
constexpr uint16_t TONE_ALIGNMENT_BASE_HZ = 2000;

constexpr Speaker_Note STARTUP_MELODY[] = {
    {1200, 50}, {1500, 50}, {1200, 50}, {1500, 50}, {1200, 50},
    {0, 200},
    {1000, 200}, {1200, 200}, {1400, 600},
};

constexpr Speaker_Note SHUTDOWN_MELODY[] = {
    {1400, 200}, {1200, 200}, {1000, 400},
};

#endif
