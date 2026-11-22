#pragma once

// Where everything goes on the status screen, and its colors. The screen is a diagram of how
// the robot's parts connect, like the stickers on M5Stack products, each part with its live
// readings. Plain C++, so laptop builds can use it.

#include <cstdint>
#include <iterator>

#include "gfx/canvas.h"
#include "m5stickc_plus/screen.h"
#include "status_values.h"

namespace status {

struct Rect {
    int x, y, w, h;
    constexpr int right() const { return x + w; }
    constexpr int bottom() const { return y + h; }
    constexpr int center_x() const { return x + w / 2; }
    constexpr int center_y() const { return y + h / 2; }
};

// Colors.
inline constexpr uint16_t BACKGROUND = gfx::rgb565(0, 0, 0);
inline constexpr uint16_t TITLE = gfx::rgb565(0, 200, 255);
inline constexpr uint16_t LABEL = gfx::rgb565(255, 180, 0);  // names of parts
inline constexpr uint16_t VALUE = gfx::rgb565(255, 255, 255);
inline constexpr uint16_t OUTLINE = gfx::rgb565(150, 150, 150);
inline constexpr uint16_t WIRE = gfx::rgb565(90, 90, 90);
inline constexpr uint16_t BUS = gfx::rgb565(130, 130, 130);  // names of wires
inline constexpr uint16_t ABSENT = gfx::rgb565(60, 60, 60);  // a part that isn't connected
inline constexpr uint16_t FLOW = gfx::rgb565(0, 220, 80);    // a wire carrying current
inline constexpr uint16_t CHARGE_HIGH = gfx::rgb565(0, 100, 30);
inline constexpr uint16_t CHARGE_MID = gfx::rgb565(130, 95, 0);
inline constexpr uint16_t CHARGE_LOW = gfx::rgb565(150, 0, 0);
inline constexpr uint16_t USAGE = gfx::rgb565(0, 70, 130);  // how much of the CPU or RAM is in use
inline constexpr uint16_t FORWARD = gfx::rgb565(0, 100, 30);   // wheel power, by direction
inline constexpr uint16_t BACKWARD = gfx::rgb565(130, 60, 0);
// Robot states: waiting for nothing, waiting for someone, moving, stopped by a fault.
inline constexpr uint16_t STATE_IDLE = VALUE;
inline constexpr uint16_t STATE_WAITING = LABEL;
inline constexpr uint16_t STATE_MOVING = FLOW;
inline constexpr uint16_t STATE_FAULT = gfx::rgb565(255, 40, 40);

// What the colors mean.
inline constexpr float USB_PRESENT_V = 4.0f;
inline constexpr float FLOW_MIN_MA = 5;  // less counts as no current
inline constexpr float CHARGE_MID_PCT = 50;
inline constexpr float CHARGE_LOW_PCT = 20;
inline constexpr float FULL_MOTOR_POWER = 127;

// Boxes: an outline, padding, then rows of text.
inline constexpr int MARGIN = 1;         // from the screen edges
inline constexpr int PADDING = 1;        // inside the outline
inline constexpr int ROW_PITCH = 10;     // from one row of text to the next
inline constexpr int ROW_GAP = 12;       // between rows of parts, for the wires and their names
inline constexpr int ARROW = 4;          // arrowhead length
inline constexpr int BUS_LABEL_GAP = 3;  // from a wire to its name

constexpr int box_width(int chars) {
    return gfx::Canvas::text_width(chars) + 2 * (PADDING + 1);
}
constexpr int box_height(int rows) {
    return (rows - 1) * ROW_PITCH + gfx::Canvas::TEXT_HEIGHT + 2 * (PADDING + 1);
}
constexpr int text_top(const Rect& box, int row) {
    return box.y + 1 + PADDING + row * ROW_PITCH;
}
constexpr Rect inside(const Rect& box) {
    return {box.x + 1, box.y + 1, box.w - 2, box.h - 2};
}
// The left (0) or right (1) half of the inside of a box.
constexpr Rect half(const Rect& box, int side) {
    const Rect all = inside(box);
    const int left_w = all.w / 2;
    return side == 0 ? Rect{all.x, all.y, left_w, all.h} : Rect{all.x + left_w, all.y, all.w - left_w, all.h};
}
// Where left-aligned text starts in a box, and how many characters fit.
constexpr int text_left(const Rect& box) {
    return box.x + 1 + PADDING;
}
constexpr int chars_fitting(const Rect& box) {
    return (box.w - 2 * (PADDING + 1) + 1) / gfx::Canvas::CHAR_ADVANCE;
}

inline constexpr int TITLE_Y = 2;

// USB -> power chip -> battery along the top, the power chip also feeding the CPU below.
inline constexpr char PMIC_NAME[] = "AXP192";
inline constexpr int POWER_Y = 14;
inline constexpr int BATTERY_NUB_W = 12;  // on top: the battery stands up, filling from the bottom
inline constexpr int BATTERY_NUB_H = 3;
inline constexpr int POWER_BOX_Y = POWER_Y + BATTERY_NUB_H;
inline constexpr Rect USB_BOX = {MARGIN, POWER_BOX_Y, box_width(5), box_height(3)};
inline constexpr Rect BATTERY = {m5::SCREEN_WIDTH - MARGIN - box_width(6), POWER_BOX_Y, box_width(6), box_height(3)};
inline constexpr int POWER_WIRE_Y = USB_BOX.center_y();
inline constexpr int PMIC_W = box_width(6);
inline constexpr Rect PMIC_BOX = {(USB_BOX.right() + BATTERY.x - PMIC_W) / 2, POWER_WIRE_Y - box_height(2) / 2, PMIC_W,
                                  box_height(2)};

// The ESP32's CPU and RAM side by side, sharing an edge.
inline constexpr int CHIP_Y = BATTERY.bottom() + ROW_GAP;
inline constexpr int LOAD_CHARS = 5;  // "99.9%"
inline constexpr Rect CPU_BOX = {MARGIN, CHIP_Y, box_width(2 * LOAD_CHARS + 1), box_height(2)};  // "99.9% 99.9%"
inline constexpr Rect RAM_BOX = {CPU_BOX.right() - 1, CHIP_Y, box_width(5), box_height(2)};  // "300KB"

// Parts wired to the CPU.
struct Peripheral {
    const char* name;
    const char* bus;
    float Values::*value;
    int decimals;
    const char* unit;
};

// The display, right of the ESP32, its wire's name above it.
inline constexpr Peripheral LCD = {"LCD", "SPI", &Values::brightness_pct, 0, "%"};
inline constexpr Rect LCD_BOX = {m5::SCREEN_WIDTH - MARGIN - box_width(4), CHIP_Y, box_width(4),
                                 box_height(2)};  // "100%"
inline constexpr int LCD_BUS_Y = LCD_BOX.y - ROW_PITCH;

// The rest along the bottom, under the CPU, each on its own wire up to it.
inline constexpr Peripheral PERIPHERALS[] = {
    {"IMU", "I2C", &Values::imu_c, 1, "C"},
    {"BUZ", "PWM", &Values::volume_pct, 0, "%"},
};
inline constexpr int PERIPHERAL_COUNT = std::size(PERIPHERALS);
inline constexpr int PERIPHERAL_W = box_width(5);  // "53.5C"
inline constexpr int PERIPHERAL_GAP = 4;
constexpr Rect peripheral_box(int index) {
    return {MARGIN + index * (PERIPHERAL_W + PERIPHERAL_GAP), CPU_BOX.bottom() + ROW_GAP, PERIPHERAL_W, box_height(2)};
}

// The BalaC base on the hat connector, right of them under the ESP32, each wheel's power
// filling its half.
inline constexpr char BASE_NAME[] = "BalaC";
inline constexpr char BASE_BUS[] = "I2C";
inline constexpr int MOTOR_CHARS = 4;  // "+127"
inline constexpr int BASE_W = box_width(2 * MOTOR_CHARS + 1);
inline constexpr Rect BASE_BOX = {m5::SCREEN_WIDTH - MARGIN - BASE_W, peripheral_box(0).y, BASE_W, box_height(2)};
inline constexpr int BASE_BUS_X = RAM_BOX.center_x();

// What the robot is doing, big, with what it needs from you and how it's doing under it.
inline constexpr int STATE_SCALE = 2;
inline constexpr int STATE_Y = BASE_BOX.bottom() + ROW_GAP / 2;
inline constexpr int HINT_Y = STATE_Y + STATE_SCALE * gfx::Canvas::TEXT_HEIGHT + 2;
inline constexpr int DETAIL_Y = HINT_Y + ROW_PITCH;

// Wi-Fi across the screen, and below it the remote on its ESP-NOW link.
inline constexpr Rect WIFI_BOX = {MARGIN, DETAIL_Y + ROW_PITCH + 2, m5::SCREEN_WIDTH - 2 * MARGIN, box_height(3)};
// Signal strength as bars after the address, like a phone's: one more lit at each of these dBm.
inline constexpr float SIGNAL_BAR_DBM[] = {-85, -75, -67, -55};
inline constexpr int SIGNAL_BAR_W = 2;
inline constexpr int SIGNAL_BAR_GAP = 1;
inline constexpr int SIGNAL_BAR_STEP = 2;  // each bar taller than the last by this
inline constexpr int SIGNAL_W = static_cast<int>(std::size(SIGNAL_BAR_DBM)) * (SIGNAL_BAR_W + SIGNAL_BAR_GAP) - SIGNAL_BAR_GAP;
inline constexpr char REMOTE_NAME[] = "JoyX";
inline constexpr char REMOTE_BUS[] = "ESP-NOW";
inline constexpr int STICK_CHARS = 4;  // "+0.5"
inline constexpr int REMOTE_W = box_width(2 * STICK_CHARS + 1);
inline constexpr Rect REMOTE_BOX = {(m5::SCREEN_WIDTH - REMOTE_W) / 2, WIFI_BOX.bottom() + ROW_GAP, REMOTE_W,
                                    box_height(2)};

static_assert(USB_BOX.right() + ARROW < PMIC_BOX.x && PMIC_BOX.right() + ARROW < BATTERY.x);
static_assert(CPU_BOX.x < PMIC_BOX.center_x() && PMIC_BOX.center_x() < RAM_BOX.right() - 1, "3V3 wire hits the ESP32");
static_assert(RAM_BOX.right() < LCD_BOX.x && BATTERY.bottom() < LCD_BUS_Y);
static_assert(CPU_BOX.x < peripheral_box(0).center_x() &&
              peripheral_box(PERIPHERAL_COUNT - 1).center_x() < CPU_BOX.right() - 1, "wires hit the CPU");
static_assert(peripheral_box(PERIPHERAL_COUNT - 1).right() <= m5::SCREEN_WIDTH - MARGIN);
static_assert(peripheral_box(PERIPHERAL_COUNT - 1).right() < BASE_BOX.x);
static_assert(BASE_BOX.x < BASE_BUS_X && BASE_BUS_X < BASE_BOX.right() - 1, "hat wire hits the base");
static_assert(BASE_BUS_X + BUS_LABEL_GAP + gfx::Canvas::text_width(BASE_BUS) < LCD_BOX.right(), "hat wire's name fits");
static_assert(gfx::Canvas::text_width("CALIBRATING", STATE_SCALE) <= m5::SCREEN_WIDTH - 2 * MARGIN);
static_assert(REMOTE_BOX.bottom() <= m5::SCREEN_HEIGHT);
static_assert(static_cast<int>(std::size(SIGNAL_BAR_DBM)) * SIGNAL_BAR_STEP <= gfx::Canvas::TEXT_HEIGHT, "bars as tall as capitals");
static_assert(gfx::Canvas::text_width(IP_SIZE - 1) + gfx::Canvas::CHAR_ADVANCE + SIGNAL_W <=
              WIFI_BOX.w - 2 * (PADDING + 1), "an address and the bars fit");

}  // namespace status
