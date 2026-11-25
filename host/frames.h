#pragma once

// Whole status screen frames for host checks, shared by the frames CLI and the Wasm module so
// both render, hash and sweep the same way.

#include <array>
#include <cstdint>
#include <span>

#include "fixtures.h"
#include "m5stickc_plus/screen.h"
#include "status_view.h"

namespace frames {

inline constexpr int WIDTH = m5::SCREEN_WIDTH;
inline constexpr int HEIGHT = m5::SCREEN_HEIGHT;
using Frame = std::array<uint16_t, WIDTH * HEIGHT>;  // RGB565, row by row

// Strip by strip, the way the robot's LCD task draws it.
inline void render(Frame& frame, const status::ScreenValues& values) {
    static_assert(HEIGHT % status::STRIP_ROWS == 0);
    for (int y = 0; y < HEIGHT; y += status::STRIP_ROWS) {
        status::draw(std::span(frame).subspan(y * WIDTH, WIDTH * status::STRIP_ROWS), y, values);
    }
}

// FNV-1a over the frame's little-endian bytes, as stored in host/goldens/.
inline uint64_t hash(std::span<const uint16_t> pixels) {
    uint64_t h = 0xcbf29ce484222325u;
    for (const uint16_t p : pixels) {
        h = (h ^ (p & 0xff)) * 0x100000001b3u;
        h = (h ^ (p >> 8)) * 0x100000001b3u;
    }
    return h;
}

// Attitudes the native/Wasm parity check renders on both builds, all on the balancing fixture:
// roll -45..45 at its pitch, pitch -60..60 at its roll, and a coarse pitch x roll grid.
inline constexpr int SWEEP_FIXTURE = 2;
inline constexpr int ROLL_STEPS = 361;   // 0.25 degree steps
inline constexpr int PITCH_STEPS = 481;  // 0.25 degree steps
inline constexpr int GRID_PITCHES = 41;  // -40..40, 2 degree steps
inline constexpr int GRID_ROLLS = 31;    // -45..45, 3 degree steps
inline constexpr int SWEEP_COUNT = ROLL_STEPS + PITCH_STEPS + GRID_PITCHES * GRID_ROLLS;

struct Attitude {
    float pitch_deg;
    float roll_deg;
};

inline Attitude sweep_point(int k) {
    if (k < ROLL_STEPS) return {1.3f, -45.0f + 0.25f * k};
    k -= ROLL_STEPS;
    if (k < PITCH_STEPS) return {-60.0f + 0.25f * k, 12.0f};
    k -= PITCH_STEPS;
    return {-40.0f + 2.0f * (k / GRID_ROLLS), -45.0f + 3.0f * (k % GRID_ROLLS)};
}

inline void render_sweep(Frame& frame, int k) {
    const Attitude a = sweep_point(k);
    render(frame, fixtures::all()[SWEEP_FIXTURE].screen(a.pitch_deg, a.roll_deg));
}

}  // namespace frames
