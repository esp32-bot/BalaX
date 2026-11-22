#pragma once

// State of charge of the M5StickC Plus's LiPo cell, estimated from its voltage and current.
// Plain C++, so laptop builds can use it.

#include <cmath>
#include <cstddef>
#include <iterator>

namespace battery {

struct CurvePoint {
    float volts;
    float percent;
};

// Resting voltage of a typical LiPo cell at each state of charge.
inline constexpr CurvePoint RESTING_CURVE[] = {
    {3.30f, 0},  {3.60f, 5},  {3.69f, 10}, {3.71f, 15}, {3.73f, 20}, {3.75f, 25}, {3.77f, 30},
    {3.79f, 35}, {3.80f, 40}, {3.82f, 45}, {3.84f, 50}, {3.85f, 55}, {3.87f, 60}, {3.91f, 65},
    {3.95f, 70}, {3.98f, 75}, {4.02f, 80}, {4.08f, 85}, {4.11f, 90}, {4.15f, 95}, {4.20f, 100},
};

// Current through the cell's internal resistance moves its voltage off the resting curve: up
// while charging, down while discharging. A rough figure for a small cell.
inline constexpr float INTERNAL_OHMS = 0.3f;
inline constexpr float MA_PER_A = 1000;

// 0 to 100, or NaN if the voltage isn't known. `milliamps` is positive while charging.
inline float percent(float volts, float milliamps) {
    if (std::isnan(volts)) {
        return NAN;
    }
    const float drop = std::isnan(milliamps) ? 0 : milliamps / MA_PER_A * INTERNAL_OHMS;
    const float resting = volts - drop;
    if (resting <= RESTING_CURVE[0].volts) {
        return 0;
    }
    for (size_t i = 1; i < std::size(RESTING_CURVE); ++i) {
        const CurvePoint& low = RESTING_CURVE[i - 1];
        const CurvePoint& high = RESTING_CURVE[i];
        if (resting < high.volts) {
            return low.percent + (resting - low.volts) / (high.volts - low.volts) * (high.percent - low.percent);
        }
    }
    return 100;
}

}  // namespace battery
