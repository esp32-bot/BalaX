#include "status_view.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <string_view>

#include "status_layout.h"

namespace status {
namespace {

using namespace std::string_view_literals;

// gfx::Canvas clips to its own buffer. Translate screen coordinates to the current strip.
class ScreenStrip {
public:
    ScreenStrip(std::span<uint16_t> pixels, int first_row)
        : canvas_(pixels, m5::SCREEN_WIDTH, static_cast<int>(pixels.size() / m5::SCREEN_WIDTH)),
          first_row_(first_row) {
        assert(pixels.size() % m5::SCREEN_WIDTH == 0);
        assert(first_row_ >= 0 && first_row_ + canvas_.height() <= m5::SCREEN_HEIGHT);
    }

    int top() const { return first_row_; }
    int bottom() const { return first_row_ + canvas_.height(); }
    bool includes(int y0, int y1) const { return y0 < bottom() && y1 > top(); }
    void fill(uint16_t color) { canvas_.fill(color); }
    void fill_rect(Rect r, uint16_t color) { canvas_.fill_rect(r.x, r.y - first_row_, r.w, r.h, color); }
    void rect(Rect r, uint16_t color) { canvas_.rect(r.x, r.y - first_row_, r.w, r.h, color); }
    void line(int x, int y, int width, uint16_t color) { canvas_.hline(x, y - first_row_, width, color); }
    void text(int x, int y, std::string_view text, uint16_t color, int scale = 1) {
        if (includes(y, y + gfx::Canvas::TEXT_HEIGHT * scale)) {
            canvas_.text(x, y - first_row_, text, color, scale);
        }
    }

private:
    gfx::Canvas canvas_;
    int first_row_;
};

struct Text {
    char chars[32];
    operator std::string_view() const { return chars; }
};

Text format(float value, int decimals, const char* unit, bool signed_value = false) {
    Text text;
    if (std::isfinite(value)) {
        snprintf(text.chars, sizeof text.chars, signed_value ? "%+.*f%s" : "%.*f%s", decimals, value, unit);
    } else {
        snprintf(text.chars, sizeof text.chars, "--%s", unit);
    }
    return text;
}

void centered(ScreenStrip& strip, Rect area, int y, std::string_view text, uint16_t color, int scale = 1) {
    const int chars = (area.w / scale + 1) / gfx::Canvas::CHAR_ADVANCE;
    text = text.substr(0, chars);
    strip.text(area.center_x() - gfx::Canvas::text_width(text, scale) / 2, y, text, color, scale);
}

void right_aligned(ScreenStrip& strip, int right, int y, std::string_view text, uint16_t color) {
    strip.text(right - gfx::Canvas::text_width(text), y, text, color);
}

constexpr Rect FULL_WIDTH = {0, 0, m5::SCREEN_WIDTH, m5::SCREEN_HEIGHT};

struct StateLook {
    uint16_t color;
    const char* hint;
};

StateLook look(balance::State state) {
    switch (state) {
        case balance::State::CALIBRATING: return {WARNING, "Keep it still"};
        case balance::State::READY: return {VALUE, "Press A to arm"};
        case balance::State::ARMED: return {WARNING, "Stand it up"};
        case balance::State::LEVELING: return {WARNING, "Hold it upright"};
        case balance::State::TAKEOFF:
        case balance::State::BALANCING: return {FLOW, ""};
        case balance::State::FALLEN: return {DANGER, "Press A"};
    }
    return {VALUE, ""};
}

void draw_header(ScreenStrip& strip, const Values& v) {
    strip.text(MARGIN, 2, "BalaX", TITLE);
    if (std::isfinite(v.uptime_s)) {
        const long s = static_cast<long>(v.uptime_s);
        Text uptime;
        snprintf(uptime.chars, sizeof uptime.chars, "%ld:%02ld:%02ld", s / 3600, s / 60 % 60, s % 60);
        right_aligned(strip, m5::SCREEN_WIDTH - MARGIN, 2, uptime, LABEL);
    }
    const StateLook state = look(v.state);
    centered(strip, FULL_WIDTH, STATE_Y, balance::name(v.state), state.color, STATE_SCALE);
    const auto fault = balance::name(v.fault);
    Text hint;
    if (v.fault == balance::Fault::NONE) {
        snprintf(hint.chars, sizeof hint.chars, "%s", state.hint);
    } else {
        snprintf(hint.chars, sizeof hint.chars, "%.*s: %s", static_cast<int>(fault.size()), fault.data(), state.hint);
    }
    centered(strip, FULL_WIDTH, HINT_Y, hint, state.color);
}

void dot(ScreenStrip& strip, int x, int y, int radius, uint16_t color) {
    for (int row = -radius; row <= radius; ++row) {
        const int width = static_cast<int>(std::sqrt(radius * radius - row * row));
        strip.line(x - width, y + row, 2 * width + 1, color);
    }
}

void level(ScreenStrip& strip, Rect area, float fraction, uint16_t color) {
    if (std::isfinite(fraction)) {
        const int height = static_cast<int>(area.h * std::clamp(fraction, 0.0f, 1.0f));
        strip.fill_rect({area.x, area.bottom() - height, area.w, height}, color);
    }
}

void draw_motor(ScreenStrip& strip, int x, float power) {
    const Rect meter = {x, HORIZON.y + 7, 8, HORIZON.h - 21};
    strip.fill_rect(meter, BACKGROUND);
    strip.rect(meter, OUTLINE);
    if (std::isfinite(power)) {
        const int travel = static_cast<int>((meter.h / 2 - 1) * std::clamp(power / 127.0f, -1.0f, 1.0f));
        const int zero = meter.center_y();
        strip.fill_rect({meter.x + 1, std::min(zero, zero - travel), meter.w - 2, std::abs(travel)},
                        power >= 0 ? FLOW : DANGER);
        strip.line(meter.x + 1, zero, meter.w - 2, VALUE);
    }
}

void draw_horizon(ScreenStrip& strip, const ScreenValues& values) {
    const Values& v = values.status;
    const Rect area = inside(HORIZON);
    strip.rect(HORIZON, OUTLINE);
    if (v.state == balance::State::CALIBRATING || !std::isfinite(values.pitch_deg) ||
        !std::isfinite(values.roll_deg)) {
        strip.fill_rect(area, PANEL);
        centered(strip, area, area.center_y() - 4, "Attitude unavailable", LABEL);
        return;
    }

    const int x_mid = area.center_x();
    const int y_mid = area.center_y() - 4;
    const float radians = values.roll_deg * 0.01745329252f;
    const float slope = std::sin(radians);
    const float up = std::cos(radians);
    const float pitch = values.pitch_deg * 1.25f;
    for (int y = std::max(area.y, strip.top()); y < std::min(area.bottom(), strip.bottom()); ++y) {
        const float distance = (y - y_mid) * up - pitch;
        if (std::abs(slope) < 0.001f) {
            strip.line(area.x, y, area.w, distance < 0 ? SKY : GROUND);
            continue;
        }
        const int split = std::clamp(static_cast<int>(std::ceil(x_mid - distance / slope)), area.x, area.right());
        strip.line(area.x, y, split - area.x, slope > 0 ? SKY : GROUND);
        strip.line(split, y, area.right() - split, slope > 0 ? GROUND : SKY);
    }

    // Fixed aircraft reference, signed wheel output bars and the live JoyX stick position.
    for (int y = y_mid - 1; y <= y_mid + 1; ++y) {
        strip.line(x_mid - 23, y, 15, y == y_mid ? VALUE : BACKGROUND);
        strip.line(x_mid + 9, y, 15, y == y_mid ? VALUE : BACKGROUND);
    }
    dot(strip, x_mid, y_mid, 3, BACKGROUND);
    dot(strip, x_mid, y_mid, 1, VALUE);
    draw_motor(strip, area.x + 4, v.motor_left);
    draw_motor(strip, area.right() - 12, v.motor_right);
    if (v.remote_live && std::isfinite(v.remote_turn) && std::isfinite(v.remote_forward)) {
        const int x = x_mid + static_cast<int>(std::lround(std::clamp(v.remote_turn, -1.0f, 1.0f) * 36));
        const int y = y_mid - static_cast<int>(std::lround(std::clamp(v.remote_forward, -1.0f, 1.0f) * 20));
        dot(strip, x, y, 4, BACKGROUND);
        dot(strip, x, y, 2, FLOW);
    }

    const Rect footer = {area.x, area.bottom() - 11, area.w, 11};
    strip.fill_rect(footer, BACKGROUND);
    Text angles;
    snprintf(angles.chars, sizeof angles.chars, "P%+.0f R%+.0f", values.pitch_deg, values.roll_deg);
    strip.text(footer.x + 2, footer.y + 2, angles, VALUE);
    right_aligned(strip, footer.right() - 2, footer.y + 2, v.remote_live ? "JoyX" : "", FLOW);
}

void draw_battery(ScreenStrip& strip, const Values& v) {
    const uint16_t color = !std::isfinite(v.battery_pct) ? ABSENT
                           : v.battery_pct < 25 ? DANGER
                           : v.battery_ma > 5 ? TITLE
                           : v.battery_pct < 45 ? WARNING : FLOW;
    const Text pct = format(v.battery_pct, 0, "%");
    strip.text(MARGIN, 116, "BAT", LABEL);
    strip.text(27, 113, pct, color, 2);
    right_aligned(strip, m5::SCREEN_WIDTH - MARGIN, 113, format(v.battery_v, 2, "V"), VALUE);
    right_aligned(strip, m5::SCREEN_WIDTH - MARGIN, 123, format(v.battery_ma, 0, "mA", true), color);
    strip.fill_rect(BATTERY_BAR, PANEL);
    strip.rect(BATTERY_BAR, OUTLINE);
    if (std::isfinite(v.battery_pct)) {
        Rect fill = inside(BATTERY_BAR);
        fill.w = static_cast<int>(fill.w * std::clamp(v.battery_pct, 0.0f, 100.0f) / 100);
        strip.fill_rect(fill, color);
    }
}

void draw_compute(ScreenStrip& strip, const Values& v) {
    strip.fill_rect(CPU_BOX, PANEL);
    const Rect cpu = inside(CPU_BOX);
    level(strip, half(cpu, 0), v.cpu0_load / 100, USAGE);
    level(strip, half(cpu, 1), v.cpu1_load / 100, USAGE);
    strip.rect(CPU_BOX, OUTLINE);
    strip.fill_rect({cpu.center_x(), cpu.y + 11, 1, cpu.h - 11}, OUTLINE);
    Text speed;
    const Text mhz = format(v.cpu_mhz, 0, "MHz");
    snprintf(speed.chars, sizeof speed.chars, "CPU %.12s", mhz.chars);
    centered(strip, cpu, cpu.y + 2, speed, LABEL);
    centered(strip, half(cpu, 0), cpu.y + 13, format(v.cpu0_load, 0, "%"), VALUE);
    centered(strip, half(cpu, 1), cpu.y + 13, format(v.cpu1_load, 0, "%"), VALUE);

    strip.fill_rect(RAM_BOX, PANEL);
    const Rect ram = inside(RAM_BOX);
    level(strip, ram, v.ram_used_kb / v.ram_total_kb, USAGE);
    strip.rect(RAM_BOX, OUTLINE);
    centered(strip, ram, ram.y + 2, "RAM used", LABEL);
    Text ratio;
    const Text used = format(v.ram_used_kb, 0, "");
    const Text total = format(v.ram_total_kb, 0, "");
    snprintf(ratio.chars, sizeof ratio.chars, "%.7s/%.7s", used.chars, total.chars);
    centered(strip, ram, ram.y + 13, ratio, VALUE);
}

uint16_t thermal_color(float value, float warm, float hot) {
    return !std::isfinite(value) ? ABSENT : value >= hot ? DANGER : value >= warm ? WARNING : VALUE;
}

void draw_part(ScreenStrip& strip, Rect box, std::string_view name, std::string_view top,
               std::string_view bottom, uint16_t color) {
    strip.fill_rect(box, PANEL);
    strip.rect(box, OUTLINE);
    const Rect area = inside(box);
    centered(strip, area, area.y + 1, name, LABEL);
    centered(strip, area, area.y + 9, top, color);
    centered(strip, area, area.y + 17, bottom, color);
}

void draw_components(ScreenStrip& strip, const Values& v) {
    const bool usb_connected = v.usb_v >= 4;
    draw_part(strip, USB_BOX, "USB", usb_connected ? format(v.usb_v, 2, "V") : format(NAN, 0, "V"),
              usb_connected ? format(v.usb_ma, 0, "mA") : format(NAN, 0, "mA"), usb_connected ? VALUE : ABSENT);
    draw_part(strip, PMIC_BOX, "AXP192", format(v.pmic_c, 1, "C"), "",
              thermal_color(v.pmic_c, 42, 46));
    draw_part(strip, IMU_BOX, "IMU", format(v.imu_c, 1, "C"), "",
              thermal_color(v.imu_c, 52, 58));
    strip.line(USB_BOX.right(), USB_BOX.center_y(), PMIC_BOX.x - USB_BOX.right(), usb_connected ? FLOW : OUTLINE);
    strip.line(PMIC_BOX.right(), PMIC_BOX.center_y(), IMU_BOX.x - PMIC_BOX.right(), OUTLINE);
}

void draw_footer(ScreenStrip& strip, const ScreenValues& values) {
    const Values& v = values.status;
    const bool connected = v.wifi_ssid[0] != '\0';
    Text wifi;
    snprintf(wifi.chars, sizeof wifi.chars, "WiFi %.16s", connected ? v.wifi_ssid : v.wifi_phase);
    strip.text(MARGIN, 201, wifi, connected ? TITLE : LABEL);
    strip.text(MARGIN, 211, connected ? std::string_view(v.wifi_ip) : "balax.local"sv, VALUE);
    if (connected && std::isfinite(v.wifi_rssi)) {
        right_aligned(strip, m5::SCREEN_WIDTH - MARGIN, 211, format(v.wifi_rssi, 0, "dBm"), LABEL);
    }
    Text timing;
    const Text tick = format(v.tick_ms, 2, "ms");
    const Text overruns = format(v.overruns, 0, "");
    snprintf(timing.chars, sizeof timing.chars, "Tick %.8s  O:%.8s", tick.chars, overruns.chars);
    strip.text(MARGIN, 222, timing, v.overruns > 0 ? DANGER : LABEL);
    Text flash;
    snprintf(flash.chars, sizeof flash.chars, "E:%lu S:%lu", static_cast<unsigned long>(values.flash_erases),
             static_cast<unsigned long>(values.flash_skipped));
    strip.text(MARGIN, 232, std::string_view(flash).substr(0, 13), LABEL);
    Text vm;
    snprintf(vm.chars, sizeof vm.chars, "VM:%lu", static_cast<unsigned long>(values.vm_hits));
    right_aligned(strip, m5::SCREEN_WIDTH - MARGIN, 232, std::string_view(vm).substr(0, 8),
                  values.vm_running ? WARNING : LABEL);
}

}  // namespace

void draw(std::span<uint16_t> pixels, int first_row, const ScreenValues& values) {
    ScreenStrip strip(pixels, first_row);
    strip.fill(BACKGROUND);
    if (strip.includes(0, 39)) draw_header(strip, values.status);
    if (strip.includes(HORIZON.y, HORIZON.bottom())) draw_horizon(strip, values);
    if (strip.includes(113, BATTERY_BAR.bottom())) draw_battery(strip, values.status);
    if (strip.includes(CPU_BOX.y, CPU_BOX.bottom())) draw_compute(strip, values.status);
    if (strip.includes(USB_BOX.y, USB_BOX.bottom())) draw_components(strip, values.status);
    if (strip.includes(201, m5::SCREEN_HEIGHT)) draw_footer(strip, values);
}

}  // namespace status
