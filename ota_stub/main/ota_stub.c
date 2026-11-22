// BalaX Minimal Standalone OTA Stub (lives in `factory` partition).
// Pure C, -Os, 80 MHz, STA-only, IPv4-only, zero C++/mbedTLS/esp_http_server/mdns!
// Includes:
//   - Shared Micro-BIOS (direct ST7789 SPI + 96-byte 3x5 font + loading bar + LED + buzzer + buttons)
//   - 30-line raw UDP mDNS responder for `balax.local`
//   - Raw LwIP TCP port 80 server: GET /api/challenge, POST /api/ota (raw + Mask ROM tinfl zlib + 4 KB sector-skip!)
//   - 60-second idle auto-return to `ota_0` (stays in rescue mode if `ota_0` is invalid or Button A held)

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "esp_event.h"
#include "esp_image_format.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_random.h"
#include "esp_rom_sys.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "micro_bios.h"
#include "miniz.h"
#include "nvs_flash.h"
#include "stub_wifi.h"

#define PIN_LED 10
#define PIN_BUZZER 2
#define PIN_BTN_A 37
#define PIN_BTN_B 39
#define PIN_LCD_MOSI 15
#define PIN_LCD_SCLK 13
#define PIN_LCD_CS 5
#define PIN_LCD_DC 23
#define PIN_LCD_RST 18
#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22
#define LCD_W 135
#define LCD_H 240
#define LCD_X_OFF 52
#define LCD_Y_OFF 40
#define SECTOR_BYTES 4096

// --- Pure-C SHA-256 + HMAC-SHA256 ---
typedef struct {
    uint32_t h[8];
    uint8_t buf[64];
    size_t buf_len;
    uint64_t total_len;
} c_sha256_t;

static const uint32_t SHA_K[64] = {
    0x428a2f98u, 0x71374491u, 0xb5c0fbcfu, 0xe9b5dba5u, 0x3956c25bu, 0x59f111f1u, 0x923f82a4u, 0xab1c5ed5u,
    0xd807aa98u, 0x12835b01u, 0x243185beu, 0x550c7dc3u, 0x72be5d74u, 0x80deb1feu, 0x9bdc06a7u, 0xc19bf174u,
    0xe49b69c1u, 0xefbe4786u, 0x0fc19dc6u, 0x240ca1ccu, 0x2de92c6fu, 0x4a7484aau, 0x5cb0a9dcu, 0x76f988dau,
    0x983e5152u, 0xa831c66du, 0xb00327c8u, 0xbf597fc7u, 0xc6e00bf3u, 0xd5a79147u, 0x06ca6351u, 0x14292967u,
    0x27b70a85u, 0x2e1b2138u, 0x4d2c6dfcu, 0x53380d13u, 0x650a7354u, 0x766a0abbu, 0x81c2c92eu, 0x92722c85u,
    0xa2bfe8a1u, 0xa81a664bu, 0xc24b8b70u, 0xc76c51a3u, 0xd192e819u, 0xd6990624u, 0xf40e3585u, 0x106aa070u,
    0x19a4c116u, 0x1e376c08u, 0x2748774cu, 0x34b0bcb5u, 0x391c0cb3u, 0x4ed8aa4au, 0x5b9cca4fu, 0x682e6ff3u,
    0x748f82eeu, 0x78a5636fu, 0x84c87814u, 0x8cc70208u, 0x90befffau, 0xa4506cebu, 0xbef9a3f7u, 0xc67178f2u,
};

static inline uint32_t rotr32(uint32_t x, uint32_t n) { return (x >> n) | (x << (32 - n)); }

static void sha_init(c_sha256_t* s) {
    s->h[0] = 0x6a09e667u; s->h[1] = 0xbb67ae85u; s->h[2] = 0x3c6ef372u; s->h[3] = 0xa54ff53au;
    s->h[4] = 0x510e527fu; s->h[5] = 0x9b05688cu; s->h[6] = 0x1f83d9abu; s->h[7] = 0x5be0cd19u;
    s->buf_len = 0; s->total_len = 0;
}

static void sha_compress(c_sha256_t* s, const uint8_t blk[64]) {
    uint32_t w[64];
    for (int i = 0; i < 16; ++i)
        w[i] = ((uint32_t)blk[i*4]<<24)|((uint32_t)blk[i*4+1]<<16)|((uint32_t)blk[i*4+2]<<8)|blk[i*4+3];
    for (int i = 16; i < 64; ++i)
        w[i] = w[i-16] + (rotr32(w[i-15],7)^rotr32(w[i-15],18)^(w[i-15]>>3)) + w[i-7] +
               (rotr32(w[i-2],17)^rotr32(w[i-2],19)^(w[i-2]>>10));
    uint32_t a=s->h[0], b=s->h[1], c=s->h[2], d=s->h[3], e=s->h[4], f=s->h[5], g=s->h[6], hh=s->h[7];
    for (int i = 0; i < 64; ++i) {
        uint32_t t1 = hh + (rotr32(e,6)^rotr32(e,11)^rotr32(e,25)) + ((e&f)^((~e)&g)) + SHA_K[i] + w[i];
        uint32_t t2 = (rotr32(a,2)^rotr32(a,13)^rotr32(a,22)) + ((a&b)^(a&c)^(b&c));
        hh=g; g=f; f=e; e=d+t1; d=c; c=b; b=a; a=t1+t2;
    }
    s->h[0]+=a; s->h[1]+=b; s->h[2]+=c; s->h[3]+=d; s->h[4]+=e; s->h[5]+=f; s->h[6]+=g; s->h[7]+=hh;
}

static void sha_update(c_sha256_t* s, const uint8_t* p, size_t n) {
    s->total_len += n;
    size_t pos = 0;
    if (s->buf_len) {
        size_t take = (64 - s->buf_len < n) ? (64 - s->buf_len) : n;
        memcpy(s->buf + s->buf_len, p, take);
        s->buf_len += take; pos += take;
        if (s->buf_len == 64) { sha_compress(s, s->buf); s->buf_len = 0; }
    }
    while (pos + 64 <= n) { sha_compress(s, p + pos); pos += 64; }
    if (pos < n) { s->buf_len = n - pos; memcpy(s->buf, p + pos, s->buf_len); }
}

static void sha_final(c_sha256_t* s, uint8_t out[32]) {
    uint64_t bits = s->total_len * 8;
    s->buf[s->buf_len++] = 0x80;
    if (s->buf_len > 56) { memset(s->buf + s->buf_len, 0, 64 - s->buf_len); sha_compress(s, s->buf); s->buf_len = 0; }
    memset(s->buf + s->buf_len, 0, 56 - s->buf_len);
    for (int i = 0; i < 8; ++i) s->buf[56 + i] = (uint8_t)(bits >> (56 - i * 8));
    sha_compress(s, s->buf);
    for (int i = 0; i < 8; ++i) {
        out[i*4]=(uint8_t)(s->h[i]>>24); out[i*4+1]=(uint8_t)(s->h[i]>>16);
        out[i*4+2]=(uint8_t)(s->h[i]>>8); out[i*4+3]=(uint8_t)s->h[i];
    }
}

typedef struct { c_sha256_t inner; uint8_t opad[64]; } c_hmac_t;

static void hmac_init(c_hmac_t* hm, const uint8_t key[32]) {
    uint8_t ipad[64];
    for (int i = 0; i < 64; ++i) {
        uint8_t k = i < 32 ? key[i] : 0;
        ipad[i] = k ^ 0x36; hm->opad[i] = k ^ 0x5c;
    }
    sha_init(&hm->inner);
    sha_update(&hm->inner, ipad, 64);
}

static void hmac_final(c_hmac_t* hm, uint8_t out[32]) {
    uint8_t ih[32]; sha_final(&hm->inner, ih);
    c_sha256_t o; sha_init(&o); sha_update(&o, hm->opad, 64); sha_update(&o, ih, 32); sha_final(&o, out);
}

static int hex_nib(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static int from_hex(const char* h, uint8_t* out, size_t n) {
    if (!h || strlen(h) < n * 2) return 0;
    for (size_t i = 0; i < n; ++i) {
        int a = hex_nib(h[i*2]), b = hex_nib(h[i*2+1]);
        if (a < 0 || b < 0) return 0;
        out[i] = (uint8_t)((a << 4) | b);
    }
    return 1;
}

static void to_hex(const uint8_t* in, size_t n, char* out) {
    static const char H[] = "0123456789abcdef";
    for (size_t i = 0; i < n; ++i) { out[i*2] = H[in[i]>>4]; out[i*2+1] = H[in[i]&15]; }
    out[n*2] = 0;
}

// --- Shared Micro-BIOS Hardware Primitives (LED, Speaker, Buttons, ST7789 SPI) ---
static spi_device_handle_t lcd_spi = NULL;

static void bios_led_set(int on) { gpio_set_level(PIN_LED, on ? 0 : 1); }

static void bios_beep(uint16_t hz, uint16_t ms) {
    if (hz == 0 || ms == 0) return;
    uint32_t half_us = 500000u / hz;
    int64_t end = esp_timer_get_time() + (int64_t)ms * 1000;
    while (esp_timer_get_time() < end) {
        gpio_set_level(PIN_BUZZER, 1); esp_rom_delay_us(half_us);
        gpio_set_level(PIN_BUZZER, 0); esp_rom_delay_us(half_us);
    }
}

static uint8_t bios_buttons_read(void) {
    return (uint8_t)((gpio_get_level(PIN_BTN_A) == 0 ? 1 : 0) | (gpio_get_level(PIN_BTN_B) == 0 ? 2 : 0));
}

static void lcd_send(uint8_t is_data, const uint8_t* buf, size_t len) {
    if (!lcd_spi || len == 0) return;
    gpio_set_level(PIN_LCD_DC, is_data);
    spi_transaction_t t; memset(&t, 0, sizeof t);
    t.length = len * 8; t.tx_buffer = buf;
    spi_device_polling_transmit(lcd_spi, &t);
}

static void lcd_cmd(uint8_t c) { lcd_send(0, &c, 1); }

static void bios_lcd_rect(int x, int y, int w, int h, uint16_t col) {
    if (!lcd_spi || w <= 0 || h <= 0) return;
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > LCD_W) w = LCD_W - x;
    if (y + h > LCD_H) h = LCD_H - y;
    if (w <= 0 || h <= 0) return;

    uint16_t x0 = (uint16_t)(x + LCD_X_OFF), x1 = (uint16_t)(x + w - 1 + LCD_X_OFF);
    uint16_t y0 = (uint16_t)(y + LCD_Y_OFF), y1 = (uint16_t)(y + h - 1 + LCD_Y_OFF);
    uint8_t ca[4] = {x0>>8, x0&0xFF, x1>>8, x1&0xFF};
    uint8_t ra[4] = {y0>>8, y0&0xFF, y1>>8, y1&0xFF};
    lcd_cmd(0x2A); lcd_send(1, ca, 4);
    lcd_cmd(0x2B); lcd_send(1, ra, 4);
    lcd_cmd(0x2C);

    uint16_t be = (uint16_t)((col >> 8) | (col << 8));
    uint16_t line[136];
    size_t total = (size_t)w * (size_t)h;
    size_t chunk = total < 136 ? total : 136;
    for (size_t i = 0; i < chunk; ++i) line[i] = be;
    while (total > 0) {
        size_t n = total < 136 ? total : 136;
        lcd_send(1, (const uint8_t*)line, n * 2);
        total -= n;
    }
}

static void bios_fill_cb(void* ctx, int x, int y, int w, int h, uint16_t col) {
    (void)ctx; bios_lcd_rect(x, y, w, h, col);
}
static void bios_lcd_clear(uint16_t col) { bios_lcd_rect(0, 0, LCD_W, LCD_H, col); }
static void bios_lcd_text(int x, int y, const char* s, uint16_t fg, uint16_t bg, int sc) {
    bios_draw_text(bios_fill_cb, NULL, x, y, s, fg, bg, sc);
}
static void bios_lcd_bar(int x, int y, int w, int h, int pct, uint16_t fg, uint16_t bg) {
    bios_draw_bar(bios_fill_cb, NULL, x, y, w, h, pct, fg, bg);
}

// Exported Micro-BIOS v1 descriptor (versioned ABI for cross-partition reuse)
const balax_bios_v1_t BALAX_MICRO_BIOS = {
    .magic = BALAX_BIOS_MAGIC,
    .abi_version = BALAX_BIOS_ABI_VER,
    .code_bytes = 2048,
    .crc32 = 0,
    .led_set = bios_led_set,
    .beep = bios_beep,
    .buttons_read = bios_buttons_read,
    .lcd_clear = bios_lcd_clear,
    .lcd_rect = bios_lcd_rect,
    .lcd_text = bios_lcd_text,
    .lcd_bar = bios_lcd_bar,
};

static void hw_init(void) {
    gpio_set_direction(PIN_LED, GPIO_MODE_OUTPUT); bios_led_set(0);
    gpio_set_direction(PIN_BUZZER, GPIO_MODE_OUTPUT); gpio_set_level(PIN_BUZZER, 0);
    gpio_set_direction(PIN_BTN_A, GPIO_MODE_INPUT);
    gpio_set_direction(PIN_BTN_B, GPIO_MODE_INPUT);
    gpio_set_direction(PIN_LCD_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_LCD_RST, GPIO_MODE_OUTPUT);

    // Wake AXP192 LDO2 (LCD backlight) & LDO3 (LCD logic)
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0, .sda_io_num = PIN_I2C_SDA, .scl_io_num = PIN_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT, .glitch_ignore_cnt = 7, .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t i2c_bus = NULL;
    if (i2c_new_master_bus(&bus_cfg, &i2c_bus) == ESP_OK) {
        i2c_device_config_t dev_cfg = { .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = 0x34, .scl_speed_hz = 100000 };
        i2c_master_dev_handle_t axp = NULL;
        if (i2c_master_bus_add_device(i2c_bus, &dev_cfg, &axp) == ESP_OK) {
            uint8_t pwr[2] = {0x12, 0x4D}; i2c_master_transmit(axp, pwr, 2, 50);
            uint8_t ldo[2] = {0x28, 0xCC}; i2c_master_transmit(axp, ldo, 2, 50);
        }
    }

    spi_bus_config_t sb = { .mosi_io_num = PIN_LCD_MOSI, .miso_io_num = -1, .sclk_io_num = PIN_LCD_SCLK,
                            .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = 512 };
    if (spi_bus_initialize(SPI2_HOST, &sb, SPI_DMA_DISABLED) == ESP_OK) {
        spi_device_interface_config_t sd = { .clock_speed_hz = 20000000, .mode = 0, .spics_io_num = PIN_LCD_CS, .queue_size = 1 };
        spi_bus_add_device(SPI2_HOST, &sd, &lcd_spi);
    }
    gpio_set_level(PIN_LCD_RST, 0); vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(PIN_LCD_RST, 1); vTaskDelay(pdMS_TO_TICKS(50));
    lcd_cmd(0x11); vTaskDelay(pdMS_TO_TICKS(120));  // SLPOUT
    uint8_t pix = 0x55; lcd_cmd(0x3A); lcd_send(1, &pix, 1);  // RGB565
    uint8_t mad = 0x00; lcd_cmd(0x36); lcd_send(1, &mad, 1);
    lcd_cmd(0x21);  // INVON
    lcd_cmd(0x29);  // DISPON
    bios_lcd_clear(BIOS_COL_BG);
    bios_lcd_text(8, 16, "BALAX STUB", BIOS_COL_BLUE, BIOS_COL_BG, 2);
    bios_lcd_text(8, 40, "CONNECTING WIFI", BIOS_COL_TEXT, BIOS_COL_BG, 1);
}

// --- 30-line Minimal UDP mDNS Responder for `balax.local` ---
static uint32_t g_ip_be = 0;

static void mdns_task(void* arg) {
    (void)arg;
    int s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s < 0) vTaskDelete(NULL);
    int reuse = 1;
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof reuse);
    struct sockaddr_in bind_addr = { .sin_family = AF_INET, .sin_port = htons(5353), .sin_addr.s_addr = htonl(INADDR_ANY) };
    bind(s, (struct sockaddr*)&bind_addr, sizeof bind_addr);
    struct ip_mreq mreq = { .imr_multiaddr.s_addr = inet_addr("224.0.0.251"), .imr_interface.s_addr = htonl(INADDR_ANY) };
    setsockopt(s, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof mreq);

    // Pre-built mDNS A-record answer for "balax.local"
    static uint8_t pkt[39] = {
        0x00,0x00, 0x84,0x00, 0x00,0x00, 0x00,0x01, 0x00,0x00, 0x00,0x00,
        5,'b','a','l','a','x', 5,'l','o','c','a','l', 0,
        0x00,0x01, 0x80,0x01, 0x00,0x00,0x00,120, 0x00,0x04, 0,0,0,0
    };
    struct sockaddr_in mcast = { .sin_family = AF_INET, .sin_port = htons(5353), .sin_addr.s_addr = inet_addr("224.0.0.251") };
    uint8_t rx[256];
    for (;;) {
        int r = recv(s, rx, sizeof rx, 0);
        if (r > 18 && g_ip_be != 0 && (rx[2] & 0x80) == 0) {
            if (memmem(rx + 12, (size_t)(r - 12), "\x05" "balax" "\x05" "local", 12) != NULL) {
                memcpy(pkt + 35, &g_ip_be, 4);
                sendto(s, pkt, sizeof pkt, 0, (struct sockaddr*)&mcast, sizeof mcast);
            }
        }
    }
}

static void wifi_event_handler(void* arg, esp_event_base_t base, int32_t id, void* data) {
    (void)arg;
    if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* e = (ip_event_got_ip_t*)data;
        g_ip_be = e->ip_info.ip.addr;
        char ip_str[24];
        snprintf(ip_str, sizeof ip_str, IPSTR, IP2STR(&e->ip_info.ip));
        bios_lcd_text(8, 40, "OTA READY       ", BIOS_COL_GREEN, BIOS_COL_BG, 1);
        bios_lcd_text(8, 54, ip_str, BIOS_COL_TEXT, BIOS_COL_BG, 1);
        bios_lcd_text(8, 68, "BALAX.LOCAL", BIOS_COL_BLUE, BIOS_COL_BG, 1);
        bios_beep(1200, 60);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        g_ip_be = 0;
        esp_wifi_connect();
    }
}

static int ota0_is_valid(void) {
    const esp_partition_t* p = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);
    if (!p) return 0;
    esp_partition_pos_t pos = { .offset = p->address, .size = p->size };
    esp_image_metadata_t meta;
    return esp_image_verify(ESP_IMAGE_VERIFY, &pos, &meta) == ESP_OK;
}

static void return_to_ota0(void) {
    const esp_partition_t* p = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);
    if (p && ota0_is_valid()) {
        esp_ota_set_boot_partition(p);
        esp_restart();
    }
}

// --- Raw LwIP TCP Port 80 Server (GET /api/challenge + POST /api/ota with ROM tinfl + Sector-Skip!) ---
static uint8_t g_nonce[16];
static int g_nonce_valid = 0;
static int64_t g_last_activity_us = 0;

static const char* find_hdr(const char* hdrs, const char* key, char* out, size_t cap) {
    const char* p = strcasestr(hdrs, key);
    if (!p) return NULL;
    p += strlen(key);
    while (*p == ' ' || *p == '\t') ++p;
    size_t i = 0;
    while (*p && *p != '\r' && *p != '\n' && i + 1 < cap) out[i++] = *p++;
    out[i] = 0;
    return out;
}

static void send_http(int fd, const char* status, const char* ctype, const char* body) {
    char buf[384];
    int n = snprintf(buf, sizeof buf,
                     "HTTP/1.1 %s\r\nContent-Type: %s\r\nContent-Length: %u\r\n"
                     "Access-Control-Allow-Origin: *\r\nConnection: close\r\n\r\n%s",
                     status, ctype, (unsigned)strlen(body), body);
    send(fd, buf, (size_t)n, 0);
}

static void handle_client(int fd) {
    g_last_activity_us = esp_timer_get_time();
    char hdr[1024];
    size_t hlen = 0;
    char* body_start = NULL;
    while (hlen + 1 < sizeof hdr) {
        int r = recv(fd, hdr + hlen, sizeof hdr - 1 - hlen, 0);
        if (r <= 0) return;
        hlen += (size_t)r;
        hdr[hlen] = 0;
        body_start = strstr(hdr, "\r\n\r\n");
        if (body_start) { body_start += 4; break; }
    }
    if (!body_start) return;

    if (strncmp(hdr, "GET /api/challenge", 18) == 0) {
        esp_fill_random(g_nonce, 16);
        g_nonce_valid = 1;
        char nhex[33]; to_hex(g_nonce, 16, nhex);
        char json[200];
        snprintf(json, sizeof json,
                 "{\"nonce\":\"%s\",\"partition\":\"factory\",\"version\":\"stub-1.0\","
                 "\"compressed_ota\":true,\"sector_skip\":true,\"is_stub\":true}", nhex);
        send_http(fd, "200 OK", "application/json", json);
        return;
    }

    if (strncmp(hdr, "POST /api/ota", 13) != 0) {
        send_http(fd, "404 Not Found", "text/plain", "stub only serves /api/challenge and /api/ota");
        return;
    }

    char nhex[40] = {0}, shex[72] = {0}, clen_s[24] = {0}, ctype[64] = {0};
    if (!g_nonce_valid || !find_hdr(hdr, "X-Nonce:", nhex, sizeof nhex) ||
        !find_hdr(hdr, "X-Signature:", shex, sizeof shex) ||
        !find_hdr(hdr, "Content-Length:", clen_s, sizeof clen_s)) {
        send_http(fd, "401 Unauthorized", "text/plain", "Missing auth headers");
        return;
    }
    find_hdr(hdr, "Content-Type:", ctype, sizeof ctype);
    uint8_t req_nonce[16], exp_sig[32];
    if (!from_hex(nhex, req_nonce, 16) || memcmp(req_nonce, g_nonce, 16) != 0 || !from_hex(shex, exp_sig, 32)) {
        send_http(fd, "401 Unauthorized", "text/plain", "Invalid nonce or signature");
        return;
    }
    g_nonce_valid = 0;

    const esp_partition_t* ota0 = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);
    if (!ota0) { send_http(fd, "500 Internal Server Error", "text/plain", "No ota_0"); return; }

    size_t content_len = (size_t)strtoul(clen_s, NULL, 10);
    int compressed = (strstr(ctype, "application/x-balax-zlib") != NULL);
    uint8_t* pending = malloc(SECTOR_BYTES);
    uint8_t* existing = malloc(SECTOR_BYTES);
    tinfl_decompressor* inf = compressed ? malloc(sizeof(tinfl_decompressor)) : NULL;
    uint8_t* dict = compressed ? malloc(TINFL_LZ_DICT_SIZE) : NULL;
    if (!pending || !existing || (compressed && (!inf || !dict))) {
        free(pending); free(existing); free(inf); free(dict);
        send_http(fd, "500 Internal Server Error", "text/plain", "OOM");
        return;
    }
    if (compressed) tinfl_init(inf);

    c_hmac_t hm; hmac_init(&hm, STUB_LAN_AUTH_KEY); sha_update(&hm.inner, req_nonce, 16);
    size_t pend_len = 0, wofs = 0, wire_got = 0, raw_got = 0, dict_ofs = 0;
    uint32_t erased = 0, skipped = 0;
    int err = 0, done = 0, last_pct = -1;

    bios_lcd_text(8, 92, compressed ? "FLASH ZLIB+SKIP" : "FLASH RAW+SKIP ", BIOS_COL_AMBER, BIOS_COL_BG, 1);

    // Helper macro to commit 4 KB sectors with read-compare-skip
    #define FLUSH_SEC() do { \
        if (pend_len > 0) { \
            if (pend_len < SECTOR_BYTES) memset(pending + pend_len, 0xFF, SECTOR_BYTES - pend_len); \
            if (wofs + SECTOR_BYTES > ota0->size) { err = 1; } \
            else if (esp_partition_read(ota0, wofs, existing, SECTOR_BYTES) == ESP_OK && memcmp(existing, pending, SECTOR_BYTES) == 0) { \
                ++skipped; \
            } else { \
                if (esp_partition_erase_range(ota0, wofs, SECTOR_BYTES) != ESP_OK || \
                    esp_partition_write(ota0, wofs, pending, SECTOR_BYTES) != ESP_OK) err = 1; \
                else ++erased; \
            } \
            wofs += SECTOR_BYTES; pend_len = 0; \
        } \
    } while (0)

    size_t first_len = hlen - (size_t)(body_start - hdr);
    uint8_t rx[1024];
    while (!err && wire_got < content_len) {
        const uint8_t* chunk = rx;
        size_t cl = 0;
        if (first_len > 0) {
            chunk = (const uint8_t*)body_start;
            cl = first_len; first_len = 0;
        } else {
            size_t want = content_len - wire_got;
            if (want > sizeof rx) want = sizeof rx;
            int r = recv(fd, rx, want, 0);
            if (r <= 0) { err = 1; break; }
            cl = (size_t)r;
        }
        wire_got += cl;
        sha_update(&hm.inner, chunk, cl);

        if (!compressed) {
            for (size_t i = 0; i < cl && !err; ) {
                size_t take = SECTOR_BYTES - pend_len;
                if (take > cl - i) take = cl - i;
                memcpy(pending + pend_len, chunk + i, take);
                pend_len += take; i += take; raw_got += take;
                if (pend_len == SECTOR_BYTES) FLUSH_SEC();
            }
        } else {
            size_t ip = 0;
            uint32_t fl = TINFL_FLAG_PARSE_ZLIB_HEADER | TINFL_FLAG_COMPUTE_ADLER32 |
                          (wire_got < content_len ? (uint32_t)TINFL_FLAG_HAS_MORE_INPUT : 0u);
            while (ip < cl && !done && !err) {
                size_t inb = cl - ip, outb = TINFL_LZ_DICT_SIZE - dict_ofs;
                tinfl_status st = tinfl_decompress(inf, chunk + ip, &inb, dict, dict + dict_ofs, &outb, fl);
                ip += inb;
                for (size_t i = 0; i < outb && !err; ) {
                    size_t take = SECTOR_BYTES - pend_len;
                    if (take > outb - i) take = outb - i;
                    memcpy(pending + pend_len, dict + dict_ofs + i, take);
                    pend_len += take; i += take; raw_got += take;
                    if (pend_len == SECTOR_BYTES) FLUSH_SEC();
                }
                dict_ofs = (dict_ofs + outb) & (TINFL_LZ_DICT_SIZE - 1);
                if (st == TINFL_STATUS_DONE) done = 1;
                else if (st < 0) err = 1;
                else if (st == TINFL_STATUS_NEEDS_MORE_INPUT && inb == 0 && outb == 0) break;
            }
        }
        int pct = content_len ? (int)((wire_got * 100ULL) / content_len) : 0;
        if (pct >= last_pct + 5) {
            last_pct = pct;
            bios_lcd_bar(8, 108, LCD_W - 16, 14, pct, BIOS_COL_GREEN, BIOS_COL_BAR_BG);
        }
        g_last_activity_us = esp_timer_get_time();
    }
    if (!err) FLUSH_SEC();
    #undef FLUSH_SEC

    uint8_t got_sig[32]; hmac_final(&hm, got_sig);
    if (err || (compressed && !done) || memcmp(got_sig, exp_sig, 32) != 0 || !ota0_is_valid()) {
        free(pending); free(existing); free(inf); free(dict);
        bios_lcd_text(8, 130, "OTA FAILED!", BIOS_COL_RED, BIOS_COL_BG, 1);
        send_http(fd, "400 Bad Request", "text/plain", "OTA verification failed");
        return;
    }

    esp_ota_set_boot_partition(ota0);
    bios_lcd_bar(8, 108, LCD_W - 16, 14, 100, BIOS_COL_GREEN, BIOS_COL_BAR_BG);
    bios_lcd_text(8, 130, "REBOOTING...", BIOS_COL_GREEN, BIOS_COL_BG, 1);
    bios_beep(1600, 90);
    char resp[220];
    snprintf(resp, sizeof resp,
             "{\"ok\":true,\"partition\":\"ota_0\",\"compressed\":%s,\"bytes\":%u,\"raw_bytes\":%u,"
             "\"sectors_erased\":%lu,\"sectors_skipped\":%lu}",
             compressed ? "true" : "false", (unsigned)wire_got, (unsigned)raw_got,
             (unsigned long)erased, (unsigned long)skipped);
    send_http(fd, "200 OK", "application/json", resp);
    free(pending); free(existing); free(inf); free(dict);
    vTaskDelay(pdMS_TO_TICKS(350));
    esp_restart();
}

void app_main(void) {
    nvs_flash_init();
    hw_init();
    int stay_forever = (bios_buttons_read() & 1) || !ota0_is_valid();

    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wcfg);
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);
    esp_wifi_set_mode(WIFI_MODE_STA);
    wifi_config_t sta = {0};
    if (STUB_WIFI_COUNT > 0 && STUB_WIFI_NETWORKS[0].ssid) {
        strncpy((char*)sta.sta.ssid, STUB_WIFI_NETWORKS[0].ssid, sizeof(sta.sta.ssid) - 1);
        strncpy((char*)sta.sta.password, STUB_WIFI_NETWORKS[0].password, sizeof(sta.sta.password) - 1);
    }
    esp_wifi_set_config(WIFI_IF_STA, &sta);
    esp_wifi_start();
    esp_wifi_set_max_tx_power(52);  // +13 dBm thermal cap
    esp_wifi_connect();

    xTaskCreate(mdns_task, "mdns", 2560, NULL, 3, NULL);

    int srv = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    int reuse = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof reuse);
    struct sockaddr_in addr = { .sin_family = AF_INET, .sin_port = htons(80), .sin_addr.s_addr = htonl(INADDR_ANY) };
    bind(srv, (struct sockaddr*)&addr, sizeof addr);
    listen(srv, 2);

    struct timeval tv = { .tv_sec = 1, .tv_usec = 0 };
    setsockopt(srv, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
    g_last_activity_us = esp_timer_get_time();

    for (;;) {
        int cli = accept(srv, NULL, NULL);
        if (cli >= 0) {
            struct timeval ctv = { .tv_sec = 15, .tv_usec = 0 };
            setsockopt(cli, SOL_SOCKET, SO_RCVTIMEO, &ctv, sizeof ctv);
            handle_client(cli);
            close(cli);
        }
        // 60-second auto-return failsafe back to main app (unless Button A held or ota_0 invalid)
        if (!stay_forever && (esp_timer_get_time() - g_last_activity_us) > 60LL * 1000 * 1000) {
            return_to_ota0();
        }
    }
}
