#include <array>
#include <span>

#include "check.h"
#include "status_layout.h"
#include "status_view.h"

namespace {

using Frame = std::array<uint16_t, m5::SCREEN_WIDTH * m5::SCREEN_HEIGHT>;

uint16_t pixel(const Frame& frame, int x, int y) {
    return frame[y * m5::SCREEN_WIDTH + x];
}

}  // namespace

TEST(status_screen_strips_match_the_host_render) {
    status::Values v;
    v.state = balance::State::BALANCING;
    v.battery_pct = 100;
    v.battery_v = 4.17f;
    v.battery_ma = 98;
    v.cpu0_load = 31;
    v.cpu1_load = 15;
    v.ram_used_kb = 103;
    v.ram_total_kb = 244;
    v.motor_left = -127;
    v.motor_right = 127;
    v.remote_live = true;
    v.remote_forward = -0.4f;
    v.remote_turn = 0.8f;
    const status::ScreenValues values{v, 12, -27, 250, 6, 8};
    Frame full;
    Frame strips;
    status::draw(full, 0, values);
    constexpr int pixels_per_strip = m5::SCREEN_WIDTH * status::STRIP_ROWS;
    for (int y = 0; y < m5::SCREEN_HEIGHT; y += status::STRIP_ROWS) {
        status::draw(std::span(strips).subspan(y * m5::SCREEN_WIDTH, pixels_per_strip), y, values);
    }
    CHECK(full == strips);
}

TEST(status_horizon_tracks_pitch_and_roll_and_hides_during_calibration) {
    status::Values v;
    v.state = balance::State::READY;
    Frame frame;
    status::ScreenValues values{v, 0, 0};
    status::draw(frame, 0, values);
    CHECK(pixel(frame, 25, 60) == status::SKY);
    CHECK(pixel(frame, 25, 92) == status::GROUND);

    values.pitch_deg = 12;
    status::draw(frame, 0, values);
    CHECK(pixel(frame, 25, 79) == status::SKY);
    values.pitch_deg = -12;
    status::draw(frame, 0, values);
    CHECK(pixel(frame, 25, 66) == status::GROUND);

    values.pitch_deg = 0;
    values.roll_deg = 30;
    status::draw(frame, 0, values);
    CHECK(pixel(frame, 25, 74) == status::SKY);
    CHECK(pixel(frame, 109, 74) == status::GROUND);

    v.state = balance::State::CALIBRATING;
    status::draw(frame, 0, values);
    CHECK(pixel(frame, 25, 60) == status::PANEL);
}
