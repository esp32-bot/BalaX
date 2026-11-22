#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Shared Micro-BIOS ABI v1 (designed to live once in `factory` and be reused by `ota_0` with 0 duplication).
#define BALAX_BIOS_MAGIC   0x42584249u  // "BXBI"
#define BALAX_BIOS_ABI_VER 1u

// RGB565 palette constants
#define BIOS_COL_BG      0x0000u  // black
#define BIOS_COL_TEXT    0xFFFFu  // white
#define BIOS_COL_MUTED   0x8410u  // gray
#define BIOS_COL_BLUE    0x5D3Fu  // #58a6ff
#define BIOS_COL_GREEN   0x3DC9u  // #3fb950
#define BIOS_COL_AMBER   0xD4C4u  // #d29922
#define BIOS_COL_RED     0xF289u  // #f85149
#define BIOS_COL_BAR_BG  0x18E3u  // dark slate

// Physically minimal packed 3x5 uppercase + digit + symbol bitmap font (48 glyphs x 2 bytes = 96 bytes!).
// Each 15-bit glyph packs 5 rows of 3 bits (MSB = row 0 col 0: bit 14..12 = row 0, ..., bit 2..0 = row 4).
static inline uint16_t bios_glyph_3x5(char ch) {
    if (ch >= 'a' && ch <= 'z') ch = (char)(ch - 'a' + 'A');
    switch (ch) {
        case '0': return 0b111101101101111;
        case '1': return 0b010110010010111;
        case '2': return 0b111001111100111;
        case '3': return 0b111001111001111;
        case '4': return 0b101101111001001;
        case '5': return 0b111100111001111;
        case '6': return 0b111100111101111;
        case '7': return 0b111001010010010;
        case '8': return 0b111101111101111;
        case '9': return 0b111101111001111;
        case 'A': return 0b010101111101101;
        case 'B': return 0b110101110101110;
        case 'C': return 0b111100100100111;
        case 'D': return 0b110101101101110;
        case 'E': return 0b111100110100111;
        case 'F': return 0b111100110100100;
        case 'G': return 0b111100101101111;
        case 'H': return 0b101101111101101;
        case 'I': return 0b111010010010111;
        case 'J': return 0b001001001101111;
        case 'K': return 0b101101110101101;
        case 'L': return 0b100100100100111;
        case 'M': return 0b101111111101101;
        case 'N': return 0b111101101101101;
        case 'O': return 0b111101101101111;
        case 'P': return 0b111101111100100;
        case 'Q': return 0b111101101111001;
        case 'R': return 0b110101110101101;
        case 'S': return 0b111100111001111;
        case 'T': return 0b111010010010010;
        case 'U': return 0b101101101101111;
        case 'V': return 0b101101101101010;
        case 'W': return 0b101101111111101;
        case 'X': return 0b101101010101101;
        case 'Y': return 0b101101010010010;
        case 'Z': return 0b111001010100111;
        case '.': return 0b000000000000010;
        case ':': return 0b000010000010000;
        case '/': return 0b001001010100100;
        case '%': return 0b101001010100101;
        case '+': return 0b000010111010000;
        case '-': return 0b000000111000000;
        case '!': return 0b010010010000010;
        case '?': return 0b111001010000010;
        case '_': return 0b000000000000111;
        case '(': return 0b010100100100010;
        case ')': return 0b010001001001010;
        default:  return 0b000000000000000;  // space
    }
}

typedef void (*bios_fill_rect_fn)(void* ctx, int x, int y, int w, int h, uint16_t rgb565);
typedef void (*bios_draw_pixels_fn)(void* ctx, int x, int y, int w, int h, const uint16_t* pixels);

// Zero-framebuffer text renderer via fill_rect callback (0 B static DRAM, ultra-tiny code).
static inline void bios_draw_text(bios_fill_rect_fn fill_rect,
                                  void* ctx,
                                  int x,
                                  int y,
                                  const char* text,
                                  uint16_t fg,
                                  uint16_t bg,
                                  int scale) {
    if (!fill_rect || !text) return;
    if (scale < 1) scale = 1;
    const int char_w = 4 * scale;
    const int char_h = 5 * scale;
    int cx = x;
    for (const char* p = text; *p; ++p) {
        if (*p == '\n') {
            cx = x;
            y += char_h + scale;
            continue;
        }
        fill_rect(ctx, cx, y, char_w, char_h, bg);
        const uint16_t g = bios_glyph_3x5(*p);
        for (int row = 0; row < 5; ++row) {
            for (int col = 0; col < 3; ++col) {
                const int bit = 14 - (row * 3 + col);
                if ((g >> bit) & 1u) {
                    fill_rect(ctx, cx + col * scale, y + row * scale, scale, scale, fg);
                }
            }
        }
        cx += char_w;
    }
}

// Fast batched zero-framebuffer text renderer: renders a full line of glyphs into a tiny
// stack strip (1 SPI transaction per line instead of 1 per pixel -> ~200x less CPU/SPI!).
static inline void bios_draw_text_fast(bios_draw_pixels_fn draw_pixels,
                                       void* ctx,
                                       int x,
                                       int y,
                                       const char* text,
                                       uint16_t fg,
                                       uint16_t bg,
                                       int scale,
                                       int max_w,
                                       uint16_t* strip) {
    if (!draw_pixels || !text || !strip) return;
    if (scale < 1) scale = 1;
    if (scale > 2) scale = 2;
    const int char_w = 4 * scale;
    const int char_h = 5 * scale;
    const int strip_max_w = (max_w > 0 && max_w <= 135) ? max_w : 135;

    const char* p = text;
    while (*p) {
        int line_chars = 0;
        const char* q = p;
        while (*q && *q != '\n') {
            ++line_chars;
            ++q;
        }
        int line_w = line_chars * char_w;
        if (x + line_w > strip_max_w) line_w = strip_max_w - x;
        if (line_w > 0) {
            const int total_px = line_w * char_h;
            for (int i = 0; i < total_px; ++i) strip[i] = bg;
            for (int ci = 0; ci < line_chars; ++ci) {
                const int cx = ci * char_w;
                if (cx >= line_w) break;
                const uint16_t g = bios_glyph_3x5(p[ci]);
                for (int row = 0; row < 5; ++row) {
                    for (int col = 0; col < 3; ++col) {
                        const int bit = 14 - (row * 3 + col);
                        if ((g >> bit) & 1u) {
                            for (int sy = 0; sy < scale; ++sy) {
                                const int py = row * scale + sy;
                                for (int sx = 0; sx < scale; ++sx) {
                                    const int px = cx + col * scale + sx;
                                    if (px < line_w) {
                                        strip[py * line_w + px] = fg;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            draw_pixels(ctx, x, y, line_w, char_h, strip);
        }
        p = (*q == '\n') ? (q + 1) : q;
        y += char_h + scale;
    }
}

// Zero-framebuffer loading bar (0..100%).
static inline void bios_draw_bar(bios_fill_rect_fn fill_rect,
                                 void* ctx,
                                 int x,
                                 int y,
                                 int w,
                                 int h,
                                 int pct,
                                 uint16_t fg,
                                 uint16_t bg) {
    if (!fill_rect || w <= 2 || h <= 2) return;
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    fill_rect(ctx, x, y, w, 1, fg);
    fill_rect(ctx, x, y + h - 1, w, 1, fg);
    fill_rect(ctx, x, y, 1, h, fg);
    fill_rect(ctx, x + w - 1, y, 1, h, fg);
    const int inner_w = w - 2;
    const int inner_h = h - 2;
    const int fill_w = (inner_w * pct) / 100;
    if (fill_w > 0) {
        fill_rect(ctx, x + 1, y + 1, fill_w, inner_h, fg);
    }
    if (fill_w < inner_w) {
        fill_rect(ctx, x + 1 + fill_w, y + 1, inner_w - fill_w, inner_h, bg);
    }
}

// Versioned cross-partition Micro-BIOS jump table (for `factory` -> `ota_0` reuse).
typedef struct {
    uint32_t magic;
    uint16_t abi_version;
    uint16_t code_bytes;
    uint32_t crc32;
    void    (*led_set)(int on);
    void    (*beep)(uint16_t hz, uint16_t ms);
    uint8_t (*buttons_read)(void);
    void    (*lcd_clear)(uint16_t rgb565);
    void    (*lcd_rect)(int x, int y, int w, int h, uint16_t rgb565);
    void    (*lcd_text)(int x, int y, const char* s, uint16_t fg, uint16_t bg, int scale);
    void    (*lcd_bar)(int x, int y, int w, int h, int pct, uint16_t fg, uint16_t bg);
} balax_bios_v1_t;

#ifdef __cplusplus
}
#endif
