#include "status_view.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <string_view>

#include "status_layout.h"

namespace status {

namespace {

using gfx::Canvas;

constexpr float PERCENT = 100;
constexpr float FULL_LOAD = 99.95f;  // would show as "100.0%", too wide, so shows as "100%"
constexpr long SECONDS_PER_MINUTE = 60;
constexpr long SECONDS_PER_HOUR = 3600;
constexpr int RIGHT = 1;
constexpr int LEFT = -1;
constexpr int NONE = 0;

// Short text for the screen, like "5.10V", or "--V" when the reading isn't available.
struct Text {
    char chars[32];
    operator std::string_view() const { return chars; }
};

Text format(float value, int decimals, const char* unit, bool show_sign = false) {
    Text text;
    if (std::isnan(value)) {
        snprintf(text.chars, sizeof text.chars, "--%s", unit);
    } else {
        snprintf(text.chars, sizeof text.chars, show_sign ? "%+.*f%s" : "%.*f%s", decimals, value, unit);
    }
    return text;
}

// "05.3%": always LOAD_CHARS wide, so the loads don't jump around.
Text format_load(float percent) {
    if (std::isnan(percent) || percent >= FULL_LOAD) {
        return format(std::isnan(percent) ? percent : PERCENT, 0, "%");
    }
    Text text;
    snprintf(text.chars, sizeof text.chars, "%0*.1f%%", LOAD_CHARS - 1, percent);
    return text;
}

// "1:02:03"
Text format_uptime(float seconds) {
    const long s = std::isnan(seconds) ? 0 : static_cast<long>(seconds);
    Text text;
    snprintf(text.chars, sizeof text.chars, "%ld:%02ld:%02ld", s / SECONDS_PER_HOUR,
             s % SECONDS_PER_HOUR / SECONDS_PER_MINUTE, s % SECONDS_PER_MINUTE);
    return text;
}

// Digits and capitals leave the font's bottom (descender) row blank.
constexpr int CAPITAL_HEIGHT = Canvas::TEXT_HEIGHT - 1;

// Where text goes to look centered between `top` and `bottom`.
int middle_text_y(int top, int bottom) {
    return (top + bottom - Canvas::TEXT_HEIGHT) / 2 + 1;
}

void outline(Canvas& canvas, const Rect& box, uint16_t color = OUTLINE) {
    canvas.rect(box.x, box.y, box.w, box.h, color);
}

void centered_on(Canvas& canvas, int center_x, int y, std::string_view text, uint16_t color, int scale = 1) {
    canvas.text(center_x - Canvas::text_width(text, scale) / 2, y, text, color, scale);
}

void centered(Canvas& canvas, const Rect& box, int row, std::string_view text, uint16_t color) {
    centered_on(canvas, box.center_x(), text_top(box, row), text, color);
}

// Fills `area` from the bottom up, `percent` of the way, like a battery.
void fill_level(Canvas& canvas, const Rect& area, float percent, uint16_t color) {
    if (std::isnan(percent)) {
        return;
    }
    const int height = static_cast<int>(area.h * std::clamp(percent, 0.0f, PERCENT) / PERCENT);
    canvas.fill_rect(area.x, area.bottom() - height, area.w, height, color);
}

// A box with its name and then one value per row, centered; all gray for a part that isn't
// connected.
void small_box(Canvas& canvas, const Rect& box, std::string_view name, std::initializer_list<Text> values,
               bool connected = true) {
    outline(canvas, box, connected ? OUTLINE : ABSENT);
    centered(canvas, box, 0, name, connected ? LABEL : ABSENT);
    for (int row = 1; const Text& value : values) {
        centered(canvas, box, row++, value, connected ? VALUE : ABSENT);
    }
}

void draw_part(Canvas& canvas, const Rect& box, const Peripheral& part, const Values& values) {
    small_box(canvas, box, part.name, {format(values.*part.value, part.decimals, part.unit)});
}

// An arrowhead with its tip at (x, y).
void arrowhead(Canvas& canvas, int x, int y, int direction, uint16_t color) {
    for (int i = 0; i < ARROW; ++i) {
        canvas.vline(x - direction * i, y - i, 2 * i + 1, color);
    }
}

// A wire straight down from `top` to `bottom`, named beside it.
void bus(Canvas& canvas, int x, int top, int bottom, std::string_view name, uint16_t color = WIRE) {
    canvas.vline(x, top, bottom - top, color);
    canvas.text(x + BUS_LABEL_GAP, middle_text_y(top, bottom), name, BUS);
}

void draw_title(Canvas& canvas, const Values& values) {
    canvas.text(MARGIN, TITLE_Y, "BalaX", TITLE);
    const Text uptime = format_uptime(values.uptime_s);
    canvas.text(m5::SCREEN_WIDTH - MARGIN - Canvas::text_width(uptime), TITLE_Y, uptime, VALUE);
}

// A power wire from the box ending at `left` to the box starting at `right`: green with an
// arrowhead against the box current flows into, gray when none flows.
void power_wire(Canvas& canvas, int left, int right, int flow) {
    canvas.hline(left, POWER_WIRE_Y, right - left, flow == NONE ? WIRE : FLOW);
    if (flow == RIGHT) {
        arrowhead(canvas, right - 1, POWER_WIRE_Y, RIGHT, FLOW);
    } else if (flow == LEFT) {
        arrowhead(canvas, left, POWER_WIRE_Y, LEFT, FLOW);
    }
}

bool usb_connected(const Values& values) {
    return values.usb_v >= USB_PRESENT_V;  // false for NaN too
}

// Current from USB into the power chip, and between it and the battery. (Comparisons with
// NaN are false, so an unknown reading shows no current.)
void draw_power_wires(Canvas& canvas, const Values& values) {
    power_wire(canvas, USB_BOX.right(), PMIC_BOX.x, usb_connected(values) ? RIGHT : NONE);
    const int battery_flow = values.battery_ma >= FLOW_MIN_MA    ? RIGHT
                             : values.battery_ma <= -FLOW_MIN_MA ? LEFT
                                                                 : NONE;
    power_wire(canvas, PMIC_BOX.right(), BATTERY.x, battery_flow);
}

uint16_t charge_color(float percent) {
    return percent >= CHARGE_MID_PCT ? CHARGE_HIGH : percent >= CHARGE_LOW_PCT ? CHARGE_MID : CHARGE_LOW;
}

// A battery standing up, filled from the bottom as far as it's charged.
void draw_battery(Canvas& canvas, const Values& values) {
    fill_level(canvas, inside(BATTERY), values.battery_pct, charge_color(values.battery_pct));
    outline(canvas, BATTERY);
    canvas.fill_rect(BATTERY.center_x() - BATTERY_NUB_W / 2, BATTERY.y - BATTERY_NUB_H, BATTERY_NUB_W, BATTERY_NUB_H,
                     OUTLINE);
    centered(canvas, BATTERY, 0, format(values.battery_pct, 0, "%"), VALUE);
    centered(canvas, BATTERY, 1, format(values.battery_v, 2, "V"), VALUE);
    centered(canvas, BATTERY, 2, format(values.battery_ma, 0, "mA", true), VALUE);
}

// "CPU 240MHz" over each core's load, each core filling its half of the box from the bottom
// as far as it's busy.
void draw_cpu(Canvas& canvas, const Values& values) {
    const float loads[] = {values.cpu0_load, values.cpu1_load};
    for (int core = 0; const float load : loads) {
        const Rect area = half(CPU_BOX, core++);
        fill_level(canvas, area, load, USAGE);
        centered_on(canvas, area.center_x(), text_top(CPU_BOX, 1), format_load(load), VALUE);
    }
    outline(canvas, CPU_BOX);
    const Text mhz = format(values.cpu_mhz, 0, "MHz");
    const int width = Canvas::text_width("CPU") + Canvas::CHAR_ADVANCE + Canvas::text_width(mhz);
    const int x = CPU_BOX.center_x() - width / 2;
    const int top = text_top(CPU_BOX, 0);
    canvas.text(x + canvas.text(x, top, "CPU", LABEL) + Canvas::CHAR_ADVANCE, top, mhz, VALUE);
}

// Used over total, as a fraction, filled from the bottom as far as it's used.
void draw_ram(Canvas& canvas, const Values& values) {
    fill_level(canvas, inside(RAM_BOX), values.ram_used_kb / values.ram_total_kb * PERCENT, USAGE);
    outline(canvas, RAM_BOX);
    centered(canvas, RAM_BOX, 0, format(values.ram_used_kb, 0, ""), VALUE);
    const Text total = format(values.ram_total_kb, 0, "KB");
    const int bar_w = Canvas::text_width(total);
    canvas.hline(RAM_BOX.center_x() - bar_w / 2, text_top(RAM_BOX, 0) + Canvas::TEXT_HEIGHT, bar_w, VALUE);
    centered(canvas, RAM_BOX, 1, total, VALUE);
}

// The wheels' power, each filling its half of the box by how hard it drives, colored by which
// way.
void draw_base(Canvas& canvas, const Values& values) {
    bus(canvas, BASE_BUS_X, RAM_BOX.bottom(), BASE_BOX.y, BASE_BUS);
    const bool present = !std::isnan(values.motor_left);
    Text motors;
    if (present) {
        const float powers[] = {values.motor_left, values.motor_right};
        for (int side = 0; const float power : powers) {
            fill_level(canvas, half(BASE_BOX, side++), std::abs(power) / FULL_MOTOR_POWER * PERCENT,
                       power >= 0 ? FORWARD : BACKWARD);
        }
        snprintf(motors.chars, sizeof motors.chars, "%+0*.0f %+0*.0f", MOTOR_CHARS, values.motor_left, MOTOR_CHARS,
                 values.motor_right);
    } else {
        motors = format(NAN, 0, "");
    }
    small_box(canvas, BASE_BOX, BASE_NAME, {motors}, present);
}

struct StateLook {
    uint16_t color;
    std::string_view hint;  // what the robot needs from you
};

StateLook look(balance::State state) {
    switch (state) {
        case balance::State::CALIBRATING: return {STATE_WAITING, "keep it still"};
        case balance::State::READY: return {STATE_IDLE, "press A to arm"};
        case balance::State::ARMED: return {STATE_WAITING, "stand it up"};
        case balance::State::LEVELING: return {STATE_WAITING, "hold it upright"};
        case balance::State::TAKEOFF: return {STATE_MOVING, ""};
        case balance::State::BALANCING: return {STATE_MOVING, ""};
        case balance::State::FALLEN: return {STATE_FAULT, "press A"};
    }
    return {STATE_IDLE, ""};
}

// The state in big letters, what it needs from you under it ("TILT: press A" after a fall),
// then the tilt and the control loop's longest tick.
void draw_state(Canvas& canvas, const Values& values) {
    const int center = m5::SCREEN_WIDTH / 2;
    const StateLook state = look(values.state);
    centered_on(canvas, center, STATE_Y, balance::name(values.state), state.color, STATE_SCALE);
    Text hint;
    if (values.fault != balance::Fault::NONE) {
        const std::string_view fault = balance::name(values.fault);
        snprintf(hint.chars, sizeof hint.chars, "%.*s: %.*s", static_cast<int>(fault.size()), fault.data(),
                 static_cast<int>(state.hint.size()), state.hint.data());
    } else {
        snprintf(hint.chars, sizeof hint.chars, "%.*s", static_cast<int>(state.hint.size()), state.hint.data());
    }
    centered_on(canvas, center, HINT_Y, hint, state.color);
    const int x = MARGIN + canvas.text(MARGIN, DETAIL_Y, "pitch", LABEL) + Canvas::CHAR_ADVANCE;
    canvas.text(x, DETAIL_Y, format(values.pitch_deg, 1, "", true), VALUE);
    const Text tick = format(values.tick_ms, 2, "ms");
    canvas.text(m5::SCREEN_WIDTH - MARGIN - Canvas::text_width(tick), DETAIL_Y, tick, VALUE);
}

// A left-aligned row of a box: a name, then a value cut to what fits.
void labeled_row(Canvas& canvas, const Rect& box, int row, std::string_view name, std::string_view value,
                 uint16_t color) {
    const int top = text_top(box, row);
    int x = text_left(box);
    int room = chars_fitting(box);
    if (!name.empty()) {
        x += canvas.text(x, top, name, LABEL) + Canvas::CHAR_ADVANCE;
        room -= static_cast<int>(name.size()) + 1;
    }
    canvas.text(x, top, value.substr(0, std::max(room, 0)), color);
}

// Signal strength as bars standing on `row`'s line of text, at the right of the box.
void signal_bars(Canvas& canvas, const Rect& box, int row, float rssi) {
    const int base = text_top(box, row) + CAPITAL_HEIGHT;
    int x = box.right() - 1 - PADDING - SIGNAL_W;
    for (int bar = 1; const float dbm : SIGNAL_BAR_DBM) {
        const int height = bar++ * SIGNAL_BAR_STEP - 1;
        canvas.fill_rect(x, base - height, SIGNAL_BAR_W, height, rssi >= dbm ? VALUE : ABSENT);
        x += SIGNAL_BAR_W + SIGNAL_BAR_GAP;
    }
}

// The network it's on, its address and signal, then its own access point and the channel
// both use.
void draw_wifi(Canvas& canvas, const Values& values) {
    outline(canvas, WIFI_BOX);
    const bool connected = values.wifi_ssid[0] != '\0';
    const Text none = format(NAN, 0, "");
    labeled_row(canvas, WIFI_BOX, 0, "WiFi", connected ? values.wifi_ssid : values.wifi_phase,
                connected ? VALUE : ABSENT);
    labeled_row(canvas, WIFI_BOX, 1, "", connected ? std::string_view(values.wifi_ip) : none,
                connected ? VALUE : ABSENT);
    if (connected) {
        signal_bars(canvas, WIFI_BOX, 1, values.wifi_rssi);
    }
    const Text channel = format(values.wifi_channel, 0, "");
    char access_point[SSID_SIZE + sizeof " ch" + sizeof channel.chars];
    snprintf(access_point, sizeof access_point, "%s ch%s", values.ap_ssid, channel.chars);
    labeled_row(canvas, WIFI_BOX, 2, "AP", access_point, VALUE);
}

// The remote, on a wire that's green while it's talking, with its stick.
void draw_remote(Canvas& canvas, const Values& values) {
    const bool live = values.remote_live;
    bus(canvas, REMOTE_BOX.center_x(), WIFI_BOX.bottom(), REMOTE_BOX.y, REMOTE_BUS, live ? FLOW : WIRE);
    Text stick;
    if (live) {
        snprintf(stick.chars, sizeof stick.chars, "%+.1f %+.1f", values.remote_forward, values.remote_turn);
    } else {
        stick = format(NAN, 0, "");
    }
    small_box(canvas, REMOTE_BOX, REMOTE_NAME, {stick}, live);
}

}  // namespace

void draw(Canvas& canvas, const Values& values) {
    canvas.fill(BACKGROUND);
    draw_title(canvas, values);

    draw_power_wires(canvas, values);
    small_box(canvas, USB_BOX, "USBC", {format(values.usb_v, 2, "V"), format(values.usb_ma, 0, "mA")},
              usb_connected(values));
    small_box(canvas, PMIC_BOX, PMIC_NAME, {format(values.pmic_c, 1, "C")});
    draw_battery(canvas, values);

    bus(canvas, PMIC_BOX.center_x(), PMIC_BOX.bottom(), CPU_BOX.y, "3V3");
    draw_cpu(canvas, values);
    draw_ram(canvas, values);

    canvas.hline(RAM_BOX.right(), LCD_BOX.center_y(), LCD_BOX.x - RAM_BOX.right(), WIRE);
    centered_on(canvas, LCD_BOX.center_x(), LCD_BUS_Y, LCD.bus, BUS);
    draw_part(canvas, LCD_BOX, LCD, values);

    for (int index = 0; const Peripheral& part : PERIPHERALS) {
        const Rect box = peripheral_box(index++);
        bus(canvas, box.center_x(), CPU_BOX.bottom(), box.y, part.bus);
        draw_part(canvas, box, part, values);
    }
    draw_base(canvas, values);

    draw_state(canvas, values);
    draw_wifi(canvas, values);
    draw_remote(canvas, values);
}

}  // namespace status
