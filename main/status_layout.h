#pragma once

#include <cstdint>

#include "gfx/canvas.h"
#include "m5stickc_plus/screen.h"

namespace status {

struct Rect {
    int x, y, w, h;
    constexpr int right() const { return x + w; }
    constexpr int bottom() const { return y + h; }
    constexpr int center_x() const { return x + w / 2; }
    constexpr int center_y() const { return y + h / 2; }
};

inline constexpr uint16_t BACKGROUND = gfx::rgb565(4, 10, 17);
inline constexpr uint16_t PANEL = gfx::rgb565(13, 25, 37);
inline constexpr uint16_t TITLE = gfx::rgb565(123, 211, 255);
inline constexpr uint16_t LABEL = gfx::rgb565(181, 205, 224);
inline constexpr uint16_t VALUE = gfx::rgb565(255, 255, 255);
inline constexpr uint16_t OUTLINE = gfx::rgb565(66, 92, 115);
inline constexpr uint16_t ABSENT = gfx::rgb565(141, 165, 184);
inline constexpr uint16_t FLOW = gfx::rgb565(115, 240, 154);
inline constexpr uint16_t WARNING = gfx::rgb565(255, 208, 116);
inline constexpr uint16_t DANGER = gfx::rgb565(255, 107, 104);
inline constexpr uint16_t USAGE = gfx::rgb565(19, 74, 113);
inline constexpr uint16_t SKY = gfx::rgb565(69, 149, 211);
inline constexpr uint16_t GROUND = gfx::rgb565(154, 94, 52);

inline constexpr int MARGIN = 3;
inline constexpr int STATE_Y = 12;
inline constexpr int STATE_SCALE = 2;
inline constexpr int HINT_Y = 30;
inline constexpr Rect HORIZON = {MARGIN, 40, m5::SCREEN_WIDTH - 2 * MARGIN, 72};
inline constexpr Rect BATTERY_BAR = {MARGIN, 133, m5::SCREEN_WIDTH - 2 * MARGIN, 6};
inline constexpr Rect CPU_BOX = {MARGIN, 142, 74, 26};
inline constexpr Rect RAM_BOX = {80, 142, 52, 26};
inline constexpr Rect USB_BOX = {MARGIN, 171, 40, 28};
inline constexpr Rect PMIC_BOX = {47, 171, 41, 28};
inline constexpr Rect IMU_BOX = {92, 171, 40, 28};

constexpr Rect inside(Rect area) {
    return {area.x + 1, area.y + 1, area.w - 2, area.h - 2};
}

constexpr Rect half(Rect area, int side) {
    const int width = area.w / 2;
    return {area.x + side * width, area.y, side == 0 ? width : area.w - width, area.h};
}

static_assert(gfx::Canvas::text_width("CALIBRATING", STATE_SCALE) <= m5::SCREEN_WIDTH);
static_assert(HORIZON.right() <= m5::SCREEN_WIDTH);
static_assert(RAM_BOX.right() <= m5::SCREEN_WIDTH && IMU_BOX.right() <= m5::SCREEN_WIDTH);

}  // namespace status
