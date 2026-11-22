// Renders the status screen on the laptop, to see a layout without flashing the robot.
//
//   tools/bx preview      writes host/build/preview.png
//
// Shows a few made-up states side by side, magnified.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <span>
#include <vector>

#include <zlib.h>

#include "battery_soc.h"
#include "m5stickc_plus/screen.h"
#include "status_view.h"

namespace {

constexpr int SCALE = 3;
constexpr int GAP = 8;  // around each screen, in screen pixels
constexpr uint8_t SURROUND[] = {40, 40, 40};

status::Values charging() {
    status::Values v;
    v.pmic_c = 44.5f;
    v.imu_c = 53.5f;
    v.battery_v = 3.97f;
    v.battery_ma = 83;
    v.usb_v = 5.10f;
    v.usb_ma = 120;
    v.cpu_mhz = 80;
    v.cpu0_load = 0.3f;
    v.cpu1_load = 0;
    v.ram_used_kb = 92;
    v.ram_total_kb = 300;
    v.uptime_s = 83;
    v.brightness_pct = 50;
    v.volume_pct = 70;
    v.state = balance::State::READY;
    v.pitch_deg = -88.5f;
    v.motor_left = 0;
    v.motor_right = 0;
    v.tick_ms = 0.31f;
    v.wifi_phase = "connected";
    snprintf(v.wifi_ssid, sizeof v.wifi_ssid, "Home Network 2.4GHz");
    snprintf(v.wifi_ip, sizeof v.wifi_ip, "192.168.100.123");
    v.wifi_rssi = -54;
    v.wifi_channel = 6;
    snprintf(v.ap_ssid, sizeof v.ap_ssid, "BalaX");
    return v;
}

status::Values on_battery() {
    status::Values v = charging();
    v.battery_v = 3.80f;
    v.battery_ma = -65;
    v.usb_v = 0;
    v.usb_ma = 0;
    v.cpu_mhz = 80;
    v.cpu0_load = 35.2f;
    v.cpu1_load = 88.4f;
    v.ram_used_kb = 231;
    v.uptime_s = 3725;
    v.state = balance::State::BALANCING;
    v.pitch_deg = 1.3f;
    v.motor_left = 45;
    v.motor_right = -38;
    v.tick_ms = 0.42f;
    v.remote_live = true;
    v.remote_forward = 0.5f;
    v.remote_turn = -0.3f;
    return v;
}

status::Values low_and_failing() {
    status::Values v = on_battery();
    v.battery_v = 3.62f;
    v.battery_ma = -140;
    v.imu_c = NAN;  // a sensor that stopped answering
    v.uptime_s = 98765;
    v.state = balance::State::FALLEN;
    v.fault = balance::Fault::TILT;
    v.pitch_deg = 41.2f;
    v.motor_left = NAN;  // the base switched off
    v.motor_right = NAN;
    v.wifi_phase = "none in range";
    v.wifi_ssid[0] = '\0';
    v.wifi_channel = 1;
    v.remote_live = false;
    return v;
}

void append_be32(std::vector<uint8_t>& out, uint32_t value) {
    for (int shift = 24; shift >= 0; shift -= 8) {
        out.push_back(value >> shift & 0xff);
    }
}

void append_chunk(std::vector<uint8_t>& png, const char* type, std::span<const uint8_t> data) {
    append_be32(png, data.size());
    const size_t start = png.size();
    png.insert(png.end(), type, type + 4);
    png.insert(png.end(), data.begin(), data.end());
    append_be32(png, crc32(0, &png[start], png.size() - start));
}

// 8-bit RGB, `rgb` row by row.
bool write_png(const char* path, int width, int height, const std::vector<uint8_t>& rgb) {
    std::vector<uint8_t> rows;
    for (int y = 0; y < height; ++y) {
        rows.push_back(0);  // no filter
        rows.insert(rows.end(), rgb.begin() + y * width * 3, rgb.begin() + (y + 1) * width * 3);
    }
    uLongf packed_size = compressBound(rows.size());
    std::vector<uint8_t> packed(packed_size);
    if (compress2(packed.data(), &packed_size, rows.data(), rows.size(), Z_BEST_COMPRESSION) != Z_OK) {
        return false;
    }
    packed.resize(packed_size);

    std::vector<uint8_t> header;
    append_be32(header, width);
    append_be32(header, height);
    header.insert(header.end(), {8, 2, 0, 0, 0});  // 8 bits, RGB, deflate, no filter, no interlace

    std::vector<uint8_t> png = {0x89, 'P', 'N', 'G', '\r', '\n', 0x1a, '\n'};
    append_chunk(png, "IHDR", header);
    append_chunk(png, "IDAT", packed);
    append_chunk(png, "IEND", {});
    FILE* file = fopen(path, "wb");
    if (!file) {
        return false;
    }
    const bool written = fwrite(png.data(), 1, png.size(), file) == png.size();
    return fclose(file) == 0 && written;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        fprintf(stderr, "usage: %s out.png\n", argv[0]);
        return 2;
    }
    status::Values boot;
    status::Values ready = charging();
    status::Values balance = on_battery();
    status::Values fallen = low_and_failing();
    for (status::Values* values : {&ready, &balance, &fallen}) {
        values->battery_pct = battery::percent(values->battery_v, values->battery_ma);
    }
    const status::ScreenValues states[] = {
        {boot},
        {ready, -18, 0},
        {balance, 1.3f, 12, 250, 6, 3},
        {fallen, 26, -25, 250, 6, 3},
    };
    const int count = std::size(states);
    const int cell_w = m5::SCREEN_WIDTH + 2 * GAP;
    const int cell_h = m5::SCREEN_HEIGHT + 2 * GAP;
    const int width = count * cell_w * SCALE;
    const int height = cell_h * SCALE;
    std::vector<uint8_t> rgb(width * height * 3);
    for (size_t i = 0; i < rgb.size(); ++i) {
        rgb[i] = SURROUND[i % 3];
    }

    std::vector<uint16_t> frame(m5::SCREEN_WIDTH * m5::SCREEN_HEIGHT);
    for (int index = 0; const status::ScreenValues& values : states) {
        status::draw(frame, 0, values);
        const int left = (index++ * cell_w + GAP) * SCALE;
        for (int y = 0; y < m5::SCREEN_HEIGHT * SCALE; ++y) {
            for (int x = 0; x < m5::SCREEN_WIDTH * SCALE; ++x) {
                const uint16_t p = frame[y / SCALE * m5::SCREEN_WIDTH + x / SCALE];
                uint8_t* out = &rgb[((y + GAP * SCALE) * width + left + x) * 3];
                out[0] = (p >> 11) * 255 / 31;
                out[1] = (p >> 5 & 0x3f) * 255 / 63;
                out[2] = (p & 0x1f) * 255 / 31;
            }
        }
    }
    if (!write_png(argv[1], width, height, rgb)) {
        fprintf(stderr, "can't write %s\n", argv[1]);
        return 1;
    }
    return 0;
}
