#include "status.h"

#include <array>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "app_settings.h"
#include "battery_soc.h"
#include "board.h"
#include "config.h"
#include "cpu_stats.h"
#include "esp_heap_caps.h"
#include "esp_netif_ip_addr.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lan_server.h"
#include "m5stickc_plus/screen.h"
#include "micro_bios.h"
#include "prog_vm.h"
#include "remote.h"
#include "robot.h"
#include "status_view.h"
#include "wifi.h"

namespace {

constexpr float BYTES_PER_KB = 1024;
constexpr float US_PER_S = 1e6f;
constexpr float US_PER_MS = 1e3f;
constexpr uint32_t POLLS_PER_PERIOD = STATUS_PERIOD_MS / STATUS_POLL_MS;
static_assert(POLLS_PER_PERIOD * STATUS_POLL_MS == STATUS_PERIOD_MS, "the period is whole polls");

CpuStats cpu;  // sampled by the status task

portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
status::Values latest;  // guarded by `lock`

SemaphoreHandle_t lcd_mutex = nullptr;
std::atomic<bool> lcd_owner_busy{false};
std::atomic<bool> lcd_needs_repaint{true};

void ensure_lcd_mutex() {
    if (!lcd_mutex) {
        lcd_mutex = xSemaphoreCreateMutex();
    }
}

void bios_display_fill(void*, int x, int y, int w, int h, uint16_t rgb565) {
    board.display.fill_rect(x, y, w, h, rgb565);
}

void bios_display_pixels(void*, int x, int y, int w, int h, const uint16_t* pixels) {
    board.display.draw_pixels(x, y, w, h, pixels);
}

// The status renderer and Micro-BIOS text share the same 2.7 KB scratch buffer.
static_assert(status::STRIP_ROWS >= 10);
uint16_t lcd_line_strip[m5::SCREEN_WIDTH * status::STRIP_ROWS];  // guarded by lcd_mutex
static_assert(m5::SCREEN_HEIGHT % status::STRIP_ROWS == 0);
static_assert(status::ATTITUDE_ROWS % status::STRIP_ROWS == 0);

void draw_line_fast(int x, int y, const char* text, uint16_t fg, uint16_t bg, int scale) {
    bios_draw_text_fast(bios_display_pixels, nullptr, x, y, text, fg, bg, scale, m5::SCREEN_WIDTH, lcd_line_strip);
}

const char* wifi_ps_name() {
    wifi_ps_type_t ps = WIFI_PS_NONE;
    if (esp_wifi_get_ps(&ps) != ESP_OK) {
        return "unknown";
    }
    switch (ps) {
        case WIFI_PS_NONE:      return "none";
        case WIFI_PS_MIN_MODEM: return "min_modem";
        case WIFI_PS_MAX_MODEM: return "max_modem";
        default:                return "other";
    }
}

float wifi_tx_dbm() {
    int8_t power_qdbm = 0;
    if (esp_wifi_get_max_tx_power(&power_qdbm) != ESP_OK) {
        return NAN;
    }
    return power_qdbm * 0.25f;
}

// From the power chip, on the bus the control loop reads the IMU on, so only once a period.
void read_power(status::Values& values) {
    values.pmic_c = board.pmic.temperature_c();
    values.battery_v = board.pmic.battery_v();
    values.battery_ma = board.pmic.battery_ma();
    values.battery_pct = battery::percent(values.battery_v, values.battery_ma);
    values.usb_v = board.pmic.usb_v();
    values.usb_ma = board.pmic.usb_ma();
}
static_assert(portNUM_PROCESSORS == 2, "one load reading per core");

void read_system(status::Values& values) {
    values.cpu_mhz = cpu.mhz();
    values.cpu0_load = cpu.load_percent(0);
    values.cpu1_load = cpu.load_percent(1);
    values.ram_total_kb = heap_caps_get_total_size(MALLOC_CAP_DEFAULT) / BYTES_PER_KB;
    values.ram_used_kb = values.ram_total_kb - esp_get_free_heap_size() / BYTES_PER_KB;
    values.uptime_s = esp_timer_get_time() / US_PER_S;
    values.brightness_pct = brightness.get();
    values.volume_pct = volume.get();

    const wifi::Status wifi = wifi::status();
    snprintf(values.wifi_ssid, sizeof values.wifi_ssid, "%s", wifi.connected() ? wifi.ssid : "");
    values.wifi_phase = wifi::describe(wifi.phase);
    const esp_ip4_addr_t ip = {wifi.ip};
    snprintf(values.wifi_ip, sizeof values.wifi_ip, IPSTR, IP2STR(&ip));
    values.wifi_rssi = wifi.connected() ? wifi.rssi : NAN;
    values.wifi_channel = wifi.channel;
    snprintf(values.ap_ssid, sizeof values.ap_ssid, "%s", wifi.ap_ssid);
    values.ap_clients = wifi.ap_clients;
}

// What can change in a moment, from memory.
void read_robot(status::Values& values, const robot::Telemetry& robot) {
    values.state = robot.state;
    values.fault = robot.fault;
    values.pitch_deg = robot.pitch_deg;
    values.motor_left = robot.base_present ? robot.motors.left : NAN;
    values.motor_right = robot.base_present ? robot.motors.right : NAN;
    values.imu_c = robot.imu_c;
    values.tick_ms = robot.max_tick_us / US_PER_MS;
    values.overruns = robot.overruns;

    const remote::Link link = remote::link();
    values.remote_live = link.live;
    values.remote_forward = link.forward;
    values.remote_turn = link.turn;
    values.remote_messages = link.messages;
    values.remote_failed = link.failed_replies;
}

uint32_t strip_hash() {
    uint32_t hash = 2166136261u;  // FNV-1a, as in the display driver
    for (const uint16_t pixel : lcd_line_strip) {
        hash = (hash ^ pixel) * 16777619u;
    }
    return hash;
}

void draw_status(const status::ScreenValues& values, bool update_system) {
    if (!lcd_mutex || xSemaphoreTake(lcd_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return;
    }
    if (brightness.get() == 0 || lcd_owner_busy.load()) {
        xSemaphoreGive(lcd_mutex);
        return;
    }
    static std::array<uint32_t, m5::SCREEN_HEIGHT / status::STRIP_ROWS> shown{};
    const bool repaint = lcd_needs_repaint.exchange(false);
    const int rows = update_system || repaint ? m5::SCREEN_HEIGHT : status::ATTITUDE_ROWS;
    for (int y = 0; y < rows; y += status::STRIP_ROWS) {
        if (lcd_owner_busy.load()) {
            lcd_needs_repaint.store(true);
            break;
        }
        status::draw(lcd_line_strip, y, values);
        const uint32_t hash = strip_hash();
        uint32_t& last = shown[y / status::STRIP_ROWS];
        if (!repaint && hash == last) {
            continue;
        }
        if (board.display.draw_pixels(0, y, m5::SCREEN_WIDTH, status::STRIP_ROWS, lcd_line_strip) != ESP_OK) {
            lcd_needs_repaint.store(true);
            break;
        }
        last = hash;
    }
    xSemaphoreGive(lcd_mutex);
}

void status_task(void*) {
    status::Values values;
    status::ScreenValues screen{values};
    TickType_t wake = xTaskGetTickCount();
    for (uint32_t poll = 0;; ++poll) {
        const bool period = poll % POLLS_PER_PERIOD == 0;
        if (period) {
            cpu.sample();
            read_power(values);
            read_system(values);
            const auto ota = lan_server::ota_stats();
            const auto vm = prog_vm::stats();
            screen.flash_erases = ota.flash_erases_total;
            screen.flash_skipped = ota.last_sectors_skipped;
            screen.vm_hits = vm.cache_hits;
            screen.vm_running = vm.running;
        }
        const robot::Telemetry robot = robot::telemetry();
        read_robot(values, robot);
        screen.pitch_deg = std::isfinite(robot.imu_c) ? robot.fused_deg.pitch : NAN;
        screen.roll_deg = std::isfinite(robot.imu_c) ? robot.fused_deg.roll : NAN;
        taskENTER_CRITICAL(&lock);
        latest = values;
        taskEXIT_CRITICAL(&lock);

        if (brightness.get() == 0) {
            lcd_needs_repaint.store(true);
        } else if (!lcd_owner_busy.load()) {
            draw_status(screen, period);
        }
        xTaskDelayUntil(&wake, pdMS_TO_TICKS(STATUS_POLL_MS));
    }
}

bool print_one_field(const char* name, const status::Values& values, const robot::Telemetry& rt, bool labeled) {
    auto emit_str = [&](const char* val) {
        if (labeled) printf("%-13s %s\n", name, val);
        else printf("%s\n", val);
    };
    auto emit_num = [&](float val, int decimals) {
        if (labeled) printf("%-13s %.*f\n", name, decimals, val);
        else printf("%.*f\n", decimals, val);
    };

    for (const status::Field& field : status::FIELDS) {
        if (strcmp(name, field.name) == 0) {
            emit_num(values.*field.value, field.decimals);
            return true;
        }
    }
    if (strcmp(name, "state") == 0) {
        const auto s = balance::name(values.state);
        if (labeled) printf("%-13s %.*s\n", name, static_cast<int>(s.size()), s.data());
        else printf("%.*s\n", static_cast<int>(s.size()), s.data());
        return true;
    }
    if (strcmp(name, "fault") == 0) {
        const auto s = balance::name(values.fault);
        if (labeled) printf("%-13s %.*s\n", name, static_cast<int>(s.size()), s.data());
        else printf("%.*s\n", static_cast<int>(s.size()), s.data());
        return true;
    }
    if (strcmp(name, "brightness") == 0) { emit_num(values.brightness_pct, 0); return true; }
    if (strcmp(name, "volume") == 0)     { emit_num(values.volume_pct, 0); return true; }
    if (strcmp(name, "ram_free_kb") == 0) {
        emit_num(esp_get_free_heap_size() / BYTES_PER_KB, 1);
        return true;
    }
    if (strcmp(name, "ram_min_kb") == 0) {
        emit_num(esp_get_minimum_free_heap_size() / BYTES_PER_KB, 1);
        return true;
    }
    if (strcmp(name, "roll_deg") == 0)   { emit_num(rt.fused_deg.roll, 1); return true; }
    if (strcmp(name, "yaw_deg") == 0)    { emit_num(rt.fused_deg.yaw, 1); return true; }
    if (strcmp(name, "imu_rest") == 0)   { emit_str(rt.imu_resting ? "yes" : "no"); return true; }
    if (strcmp(name, "wifi") == 0)       { emit_str(values.wifi_phase); return true; }
    if (strcmp(name, "wifi_ssid") == 0)  { emit_str(values.wifi_ssid); return true; }
    if (strcmp(name, "wifi_ip") == 0)    { emit_str(values.wifi_ip); return true; }
    if (strcmp(name, "wifi_ps") == 0)    { emit_str(wifi_ps_name()); return true; }
    if (strcmp(name, "wifi_tx_dbm") == 0){ emit_num(wifi_tx_dbm(), 1); return true; }
    if (strcmp(name, "ap_ssid") == 0)    { emit_str(values.ap_ssid); return true; }
    if (strcmp(name, "remote_live") == 0){ emit_str(values.remote_live ? "yes" : "no"); return true; }

    const auto ota_s = lan_server::ota_stats();
    if (strcmp(name, "flash_erases") == 0) { emit_num(static_cast<float>(ota_s.flash_erases_total), 0); return true; }
    if (strcmp(name, "ota_erased") == 0)   { emit_num(static_cast<float>(ota_s.last_sectors_erased), 0); return true; }
    if (strcmp(name, "ota_skipped") == 0)  { emit_num(static_cast<float>(ota_s.last_sectors_skipped), 0); return true; }

    const auto vm = prog_vm::stats();
    if (strcmp(name, "prog_running") == 0)  { emit_str(vm.running ? "yes" : "no"); return true; }
    if (strcmp(name, "cache_hits") == 0)    { emit_num(static_cast<float>(vm.cache_hits), 0); return true; }
    if (strcmp(name, "cache_misses") == 0)  { emit_num(static_cast<float>(vm.cache_misses), 0); return true; }
    if (strcmp(name, "cache_kb") == 0)      { emit_num(vm.cache_used_bytes / BYTES_PER_KB, 2); return true; }
    if (strcmp(name, "cache_entries") == 0) { emit_num(static_cast<float>(vm.cache_entries), 0); return true; }
    return false;
}

}  // namespace

namespace status_lcd {

void set_busy(bool busy) {
    lcd_owner_busy.store(busy);
    if (!busy) {
        lcd_needs_repaint.store(true);
    }
}

void clear(uint16_t rgb565) {
    ensure_lcd_mutex();
    if (lcd_mutex && xSemaphoreTake(lcd_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        board.display.fill_rect(0, 0, m5::SCREEN_WIDTH, m5::SCREEN_HEIGHT, rgb565);
        xSemaphoreGive(lcd_mutex);
    }
}

void fill_rect(int x, int y, int w, int h, uint16_t rgb565) {
    ensure_lcd_mutex();
    if (lcd_mutex && xSemaphoreTake(lcd_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        board.display.fill_rect(x, y, w, h, rgb565);
        xSemaphoreGive(lcd_mutex);
    }
}

void draw_text(int x, int y, const char* text, uint16_t fg, uint16_t bg, int scale) {
    ensure_lcd_mutex();
    if (lcd_mutex && xSemaphoreTake(lcd_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        draw_line_fast(x, y, text, fg, bg, scale);
        xSemaphoreGive(lcd_mutex);
    }
}

void draw_bar(int x, int y, int w, int h, int pct, uint16_t fg, uint16_t bg) {
    ensure_lcd_mutex();
    if (lcd_mutex && xSemaphoreTake(lcd_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        bios_draw_bar(bios_display_fill, nullptr, x, y, w, h, pct, fg, bg);
        xSemaphoreGive(lcd_mutex);
    }
}

void draw_ota_progress(const char* label, int pct, const char* detail) {
    ensure_lcd_mutex();
    if (!lcd_mutex || xSemaphoreTake(lcd_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }
    if (pct <= 0) {
        board.display.fill_rect(0, 0, m5::SCREEN_WIDTH, m5::SCREEN_HEIGHT, BIOS_COL_BG);
        draw_line_fast(8, 20, "BALAX OTA", BIOS_COL_BLUE, BIOS_COL_BG, 2);
    }
    char pct_buf[24];
    snprintf(pct_buf, sizeof pct_buf, "%3d%%    ", pct < 0 ? 0 : (pct > 100 ? 100 : pct));
    draw_line_fast(8, 52, label ? label : "FLASHING...", BIOS_COL_TEXT, BIOS_COL_BG, 1);
    bios_draw_bar(bios_display_fill, nullptr, 8, 70, m5::SCREEN_WIDTH - 16, 16, pct, BIOS_COL_GREEN, BIOS_COL_BAR_BG);
    draw_line_fast(8, 96, pct_buf, BIOS_COL_GREEN, BIOS_COL_BG, 2);
    if (detail) {
        draw_line_fast(8, 124, detail, BIOS_COL_MUTED, BIOS_COL_BG, 1);
    }
    xSemaphoreGive(lcd_mutex);
}

}  // namespace status_lcd

esp_err_t start_status_screen() {
    ensure_lcd_mutex();
    return xTaskCreatePinnedToCore(status_task, "status", STATUS_STACK_BYTES, nullptr, STATUS_PRIORITY, nullptr,
                                   STATUS_CORE) == pdPASS
               ? ESP_OK
               : ESP_ERR_NO_MEM;
}

status::Values latest_status() {
    taskENTER_CRITICAL(&lock);
    const status::Values values = latest;
    taskEXIT_CRITICAL(&lock);
    return values;
}

void print_status() {
    status::Values values;
    read_power(values);
    read_system(values);
    read_robot(values, robot::telemetry());
    const std::string_view state = balance::name(values.state);
    const std::string_view fault = balance::name(values.fault);
    printf("%-13s %.*s\n", "state", static_cast<int>(state.size()), state.data());
    printf("%-13s %.*s\n", "fault", static_cast<int>(fault.size()), fault.data());
    for (const status::Field& field : status::FIELDS) {
        printf("%-13s %.*f\n", field.name, field.decimals, values.*field.value);
    }
    printf("%-13s %.0f\n", "brightness", values.brightness_pct);
    printf("%-13s %.1f\n", "ram_free_kb", esp_get_free_heap_size() / BYTES_PER_KB);
    printf("%-13s %.1f\n", "ram_min_kb", esp_get_minimum_free_heap_size() / BYTES_PER_KB);
    printf("%-13s %s\n", "wifi", values.wifi_phase);
    printf("%-13s %s\n", "wifi_ps", wifi_ps_name());
    printf("%-13s %.1f\n", "wifi_tx_dbm", wifi_tx_dbm());
    printf("%-13s %s\n", "wifi_ssid", values.wifi_ssid);
    printf("%-13s %s\n", "wifi_ip", values.wifi_ip);
    printf("%-13s %s\n", "ap_ssid", values.ap_ssid);
    printf("%-13s %s\n", "remote_live", values.remote_live ? "yes" : "no");
    const auto ota_s = lan_server::ota_stats();
    printf("%-13s %lu\n", "flash_erases", static_cast<unsigned long>(ota_s.flash_erases_total));
    printf("%-13s %lu\n", "ota_erased", static_cast<unsigned long>(ota_s.last_sectors_erased));
    printf("%-13s %lu\n", "ota_skipped", static_cast<unsigned long>(ota_s.last_sectors_skipped));
    const auto vm = prog_vm::stats();
    printf("%-13s %s\n", "prog_running", vm.running ? "yes" : "no");
    printf("%-13s %lu\n", "cache_hits", static_cast<unsigned long>(vm.cache_hits));
    printf("%-13s %lu\n", "cache_misses", static_cast<unsigned long>(vm.cache_misses));
    printf("%-13s %.2f\n", "cache_kb", vm.cache_used_bytes / BYTES_PER_KB);
    printf("%-13s %u\n", "cache_entries", static_cast<unsigned>(vm.cache_entries));
}

int print_probe(int argc, char** argv) {
    if (argc <= 1) {
        print_status();
        return 0;
    }
    if (argc == 2 && strcmp(argv[1], "list") == 0) {
        printf("state fault");
        for (const status::Field& field : status::FIELDS) {
            printf(" %s", field.name);
        }
        printf(" brightness volume ram_free_kb ram_min_kb roll_deg yaw_deg imu_rest wifi wifi_ps wifi_tx_dbm wifi_ssid wifi_ip ap_ssid remote_live flash_erases ota_erased ota_skipped prog_running cache_hits cache_misses cache_kb cache_entries\n");
        return 0;
    }
    status::Values values;
    read_power(values);
    read_system(values);
    const robot::Telemetry rt = robot::telemetry();
    read_robot(values, rt);
    const bool labeled = argc > 2;
    int unknown = 0;
    for (int i = 1; i < argc; ++i) {
        if (!print_one_field(argv[i], values, rt, labeled)) {
            printf("Unknown probe field: %s (run `probe list`)\n", argv[i]);
            ++unknown;
        }
    }
    return unknown == 0 ? 0 : 1;
}
