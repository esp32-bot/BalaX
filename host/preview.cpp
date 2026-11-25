// Renders the status screen on the laptop, to see a layout without flashing the robot.
//
//   tools/bx preview      writes host/build/preview.png
//
// Shows the made-up states in host/fixtures.h side by side, magnified.

#include <cstdint>
#include <cstdio>
#include <span>
#include <vector>

#include <zlib.h>

#include "fixtures.h"
#include "m5stickc_plus/screen.h"
#include "status_view.h"

namespace {

constexpr int SCALE = 3;
constexpr int GAP = 8;  // around each screen, in screen pixels
constexpr uint8_t SURROUND[] = {40, 40, 40};

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
    const auto& states = fixtures::all();
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
    for (int index = 0; const fixtures::Fixture& fixture : states) {
        status::draw(frame, 0, fixture.screen());
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
