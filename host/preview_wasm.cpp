// The status screen renderer as a WebAssembly module, for the live preview page and the
// native/Wasm parity check. Each call renders into one frame that JavaScript reads in place.

#include <cstdint>

#include <emscripten/emscripten.h>

#include "fixtures.h"
#include "frames.h"

namespace {

frames::Frame frame;
uint64_t frame_hash = 0;

bool known(int fixture) {
    return fixture >= 0 && fixture < static_cast<int>(fixtures::all().size());
}

int rendered() {
    frame_hash = frames::hash(frame);
    return 1;
}

}  // namespace

extern "C" {

EMSCRIPTEN_KEEPALIVE int frame_width() { return frames::WIDTH; }
EMSCRIPTEN_KEEPALIVE int frame_height() { return frames::HEIGHT; }
EMSCRIPTEN_KEEPALIVE uint16_t* frame_pixels() { return frame.data(); }
EMSCRIPTEN_KEEPALIVE uint32_t frame_hash_lo() { return static_cast<uint32_t>(frame_hash); }
EMSCRIPTEN_KEEPALIVE uint32_t frame_hash_hi() { return static_cast<uint32_t>(frame_hash >> 32); }

EMSCRIPTEN_KEEPALIVE int fixture_count() { return static_cast<int>(fixtures::all().size()); }
EMSCRIPTEN_KEEPALIVE const char* fixture_id(int fixture) { return known(fixture) ? fixtures::all()[fixture].id : ""; }
EMSCRIPTEN_KEEPALIVE float fixture_pitch(int fixture) { return known(fixture) ? fixtures::all()[fixture].pitch_deg : 0; }
EMSCRIPTEN_KEEPALIVE float fixture_roll(int fixture) { return known(fixture) ? fixtures::all()[fixture].roll_deg : 0; }

// 0 for an unknown fixture.
EMSCRIPTEN_KEEPALIVE int render_fixture(int fixture) {
    if (!known(fixture)) return 0;
    frames::render(frame, fixtures::all()[fixture].screen());
    return rendered();
}

EMSCRIPTEN_KEEPALIVE int render_attitude(int fixture, float pitch_deg, float roll_deg) {
    if (!known(fixture)) return 0;
    frames::render(frame, fixtures::all()[fixture].screen(pitch_deg, roll_deg));
    return rendered();
}

EMSCRIPTEN_KEEPALIVE int sweep_count() { return frames::SWEEP_COUNT; }

EMSCRIPTEN_KEEPALIVE int render_sweep(int k) {
    if (k < 0 || k >= frames::SWEEP_COUNT) return 0;
    frames::render_sweep(frame, k);
    return rendered();
}

}  // extern "C"
