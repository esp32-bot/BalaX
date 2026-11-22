#include "prog_vm.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "board.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "m5stickc_plus/screen.h"
#include "micro_bios.h"
#include "robot.h"
#include "sha256_hmac.h"
#include "status.h"

namespace prog_vm {
namespace {

constexpr uint32_t VM_STACK_BYTES = 4096;
constexpr UBaseType_t VM_PRIORITY = 4;  // Core 0, well below Core 1 Priority 20 balance loop
constexpr BaseType_t VM_CORE = 0;
constexpr size_t MIN_FREE_HEAP_BYTES = 28 * 1024;
constexpr float HARD_PMIC_CUTOFF_C = 58.0f;
constexpr float HARD_IMU_CUTOFF_C = 65.0f;
constexpr float MOTION_PMIC_LIMIT_C = 50.0f;
constexpr size_t MAX_LOGS = 24;

struct LogEntry {
    int32_t ms = 0;
    int32_t code = 0;
    int32_t value = 0;
};

SemaphoreHandle_t vm_mutex = nullptr;
bxvm::DramAppletCache cache(bxvm::DRAM_CACHE_BUDGET_BYTES);

std::atomic<bool> running_flag{false};
std::atomic<bool> abort_flag{false};

std::vector<uint8_t> active_bytecode;
std::string active_name;
std::string active_sha;
bool active_cache_hit = false;
int64_t run_start_us = 0;
uint32_t runs_total = 0;
bxvm::RunResult last_result{};
LogEntry logs[MAX_LOGS];
size_t log_count = 0;

void ensure_mutex() {
    if (!vm_mutex) {
        vm_mutex = xSemaphoreCreateMutex();
    }
}

void push_log(int32_t code, int32_t value) {
    if (!vm_mutex || xSemaphoreTake(vm_mutex, pdMS_TO_TICKS(20)) != pdTRUE) return;
    const int32_t ms = static_cast<int32_t>((esp_timer_get_time() - run_start_us) / 1000);
    if (log_count < MAX_LOGS) {
        logs[log_count++] = LogEntry{ms, code, value};
    } else {
        std::memmove(logs, logs + 1, (MAX_LOGS - 1) * sizeof(LogEntry));
        logs[MAX_LOGS - 1] = LogEntry{ms, code, value};
    }
    xSemaphoreGive(vm_mutex);
}

class EspHost final : public bxvm::Host {
public:
    bool owns_lcd = false;
    bool thermal_aborted = false;

    bool should_abort() const override {
        if (abort_flag.load()) return true;
        const auto s = latest_status();
        const auto rt = robot::telemetry();
        if ((std::isfinite(s.pmic_c) && s.pmic_c >= HARD_PMIC_CUTOFF_C) ||
            (std::isfinite(rt.imu_c) && rt.imu_c >= HARD_IMU_CUTOFF_C)) {
            const_cast<EspHost*>(this)->thermal_aborted = true;
            return true;
        }
        return false;
    }

    void beep(int32_t hz, int32_t ms) override {
        const uint16_t clamped_hz = static_cast<uint16_t>(std::clamp<int32_t>(hz, 0, 8000));
        const uint16_t clamped_ms = static_cast<uint16_t>(std::clamp<int32_t>(ms, 0, 2000));
        board.speaker.play(m5::Note{clamped_hz, clamped_ms});
    }

    void led(bool on) override { board.led.set(on); }

    bool sleep_ms(int32_t ms) override {
        if (ms <= 0) return should_abort();
        int32_t rem = std::min<int32_t>(ms, 30000);
        while (rem > 0) {
            if (should_abort()) return true;
            const int32_t step = std::min<int32_t>(rem, 20);
            vTaskDelay(pdMS_TO_TICKS(step));
            rem -= step;
        }
        return should_abort();
    }

    void set_stick(int32_t fwd_pct, int32_t turn_pct) override {
        const auto s = latest_status();
        if (std::isfinite(s.pmic_c) && s.pmic_c >= MOTION_PMIC_LIMIT_C && (fwd_pct != 0 || turn_pct != 0)) {
            robot::steer(0.0f, 0.0f);
            return;
        }
        robot::steer(static_cast<float>(fwd_pct) / 100.0f, static_cast<float>(turn_pct) / 100.0f);
    }

    void arm() override {
        const auto s = latest_status();
        if (std::isfinite(s.pmic_c) && s.pmic_c >= MOTION_PMIC_LIMIT_C) return;
        robot::request(balance::Request::ARM);
    }

    void disarm() override { robot::request(balance::Request::DISARM); }

    void takeoff() override {
        const auto s = latest_status();
        if (std::isfinite(s.pmic_c) && s.pmic_c >= MOTION_PMIC_LIMIT_C) return;
        robot::request(balance::Request::TAKEOFF);
    }

    int32_t state() const override { return static_cast<int32_t>(robot::telemetry().state); }

    int32_t pitch_cdeg() const override {
        const float p = robot::telemetry().fused_deg.pitch;
        return std::isfinite(p) ? static_cast<int32_t>(std::lround(p * 100.0f)) : 0;
    }

    int32_t roll_cdeg() const override {
        const float r = robot::telemetry().fused_deg.roll;
        return std::isfinite(r) ? static_cast<int32_t>(std::lround(r * 100.0f)) : 0;
    }

    int32_t yaw_cdeg() const override {
        const float y = robot::telemetry().fused_deg.yaw;
        return std::isfinite(y) ? static_cast<int32_t>(std::lround(y * 100.0f)) : 0;
    }

    int32_t battery_pct() const override {
        const float b = latest_status().battery_pct;
        return std::isfinite(b) ? static_cast<int32_t>(std::lround(b)) : 0;
    }

    int32_t pmic_dc() const override {
        const float t = latest_status().pmic_c;
        return std::isfinite(t) ? static_cast<int32_t>(std::lround(t * 10.0f)) : 0;
    }

    int32_t uptime_ms() const override {
        return static_cast<int32_t>((esp_timer_get_time() - run_start_us) / 1000);
    }

    void claim_lcd() {
        if (!owns_lcd) {
            owns_lcd = true;
            status_lcd::set_busy(true);
        }
    }

    void lcd_clear(uint16_t rgb565) override {
        claim_lcd();
        status_lcd::clear(rgb565);
    }

    void lcd_text(int32_t x, int32_t y, const char* text, uint16_t fg, int32_t scale) override {
        claim_lcd();
        status_lcd::draw_text(static_cast<int>(x), static_cast<int>(y), text, fg, BIOS_COL_BG,
                              static_cast<int>(std::clamp<int32_t>(scale, 1, 4)));
    }

    void lcd_bar(int32_t y, int32_t pct, uint16_t fg) override {
        claim_lcd();
        status_lcd::draw_bar(8, static_cast<int>(y), m5::SCREEN_WIDTH - 16, 14, static_cast<int>(pct), fg,
                             BIOS_COL_BAR_BG);
    }

    void log(int32_t code, int32_t value) override { push_log(code, value); }
};

void vm_task(void*) {
    EspHost host;
    const bxvm::RunResult res = bxvm::execute(active_bytecode.data(), active_bytecode.size(), host);
    // Always restore safe state on exit!
    robot::steer(0.0f, 0.0f);
    board.led.set(false);
    if (host.owns_lcd) {
        status_lcd::set_busy(false);
    }
    if (vm_mutex && xSemaphoreTake(vm_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        last_result = res;
        if (host.thermal_aborted) {
            last_result.trap = "thermal_cutoff";
        }
        active_bytecode.clear();
        active_bytecode.shrink_to_fit();
        running_flag.store(false);
        xSemaphoreGive(vm_mutex);
    } else {
        running_flag.store(false);
    }
    vTaskDelete(nullptr);
}

LaunchResult launch_locked(const std::string& name, const std::string& sha_hex,
                           const uint8_t* data, size_t len, bool cache_hit) {
    LaunchResult out{};
    std::snprintf(out.sha256, sizeof out.sha256, "%s", sha_hex.c_str());
    out.cache_hit = cache_hit;
    if (running_flag.load()) {
        out.error = "already_running";
        return out;
    }
    if (esp_get_free_heap_size() < MIN_FREE_HEAP_BYTES + len) {
        out.error = "low_memory";
        return out;
    }
    active_bytecode.assign(data, data + len);
    active_name = name;
    active_sha = sha_hex;
    active_cache_hit = cache_hit;
    run_start_us = esp_timer_get_time();
    log_count = 0;
    last_result = bxvm::RunResult{};
    abort_flag.store(false);
    running_flag.store(true);
    ++runs_total;

    if (xTaskCreatePinnedToCore(vm_task, "bxvm", VM_STACK_BYTES, nullptr, VM_PRIORITY, nullptr, VM_CORE) != pdPASS) {
        running_flag.store(false);
        active_bytecode.clear();
        active_bytecode.shrink_to_fit();
        out.error = "task_create_failed";
        return out;
    }
    out.ok = true;
    return out;
}

}  // namespace

void init() { ensure_mutex(); }

LaunchResult start_from_bytes(const char* name, const uint8_t* data, size_t len, bool pin) {
    ensure_mutex();
    LaunchResult out{};
    if (!data || len < bxvm::HEADER_BYTES || len > bxvm::MAX_PROG_BYTES) {
        out.error = "invalid_size";
        return out;
    }
    if (std::memcmp(data, bxvm::MAGIC, 4) != 0) {
        out.error = "bad_magic";
        return out;
    }
    const auto digest = crypto::sha256(data, len);
    char sha_hex[65];
    crypto::to_hex(digest.data(), digest.size(), sha_hex);

    if (!vm_mutex || xSemaphoreTake(vm_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        out.error = "busy";
        return out;
    }
    const std::string prog_name = (name && *name) ? std::string(name) : std::string("applet");
    cache.put(sha_hex, prog_name, data, len, pin, 1.0f);
    out = launch_locked(prog_name, sha_hex, data, len, false);
    xSemaphoreGive(vm_mutex);
    return out;
}

LaunchResult start_from_cache(const char* sha256_hex) {
    ensure_mutex();
    LaunchResult out{};
    if (!sha256_hex || std::strlen(sha256_hex) != 64) {
        out.error = "bad_sha256";
        return out;
    }
    if (!vm_mutex || xSemaphoreTake(vm_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        out.error = "busy";
        return out;
    }
    const bxvm::CacheEntry* entry = cache.get(sha256_hex);
    if (!entry) {
        xSemaphoreGive(vm_mutex);
        std::snprintf(out.sha256, sizeof out.sha256, "%s", sha256_hex);
        out.error = "cache_miss";
        return out;
    }
    out = launch_locked(entry->name, entry->sha256_hex, entry->bytes.data(), entry->bytes.size(), true);
    xSemaphoreGive(vm_mutex);
    return out;
}

void stop() {
    abort_flag.store(true);
    robot::steer(0.0f, 0.0f);
}

bool set_pinned(const char* sha256_hex, bool pinned) {
    ensure_mutex();
    if (!sha256_hex || !vm_mutex || xSemaphoreTake(vm_mutex, pdMS_TO_TICKS(100)) != pdTRUE) return false;
    const bool ok = cache.set_pinned(sha256_hex, pinned);
    xSemaphoreGive(vm_mutex);
    return ok;
}

void clear_cache() {
    ensure_mutex();
    if (!vm_mutex || xSemaphoreTake(vm_mutex, pdMS_TO_TICKS(100)) != pdTRUE) return;
    cache.clear_unpinned();
    xSemaphoreGive(vm_mutex);
}

RuntimeStats stats() {
    ensure_mutex();
    RuntimeStats s{};
    s.running = running_flag.load();
    if (vm_mutex && xSemaphoreTake(vm_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        const auto cs = cache.stats();
        s.last_ok = last_result.ok;
        s.last_aborted = last_result.aborted;
        s.last_steps = last_result.steps;
        s.runs_total = runs_total;
        s.cache_hits = cs.hits;
        s.cache_misses = cs.misses;
        s.cache_evictions = cs.evictions;
        s.cache_used_bytes = cs.used_bytes;
        s.cache_budget_bytes = cs.budget_bytes;
        s.cache_entries = cs.entries;
        xSemaphoreGive(vm_mutex);
    }
    return s;
}

size_t format_status_json(char* buf, size_t cap) {
    ensure_mutex();
    if (!buf || cap == 0) return 0;
    if (!vm_mutex || xSemaphoreTake(vm_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return static_cast<size_t>(std::snprintf(buf, cap, "{\"error\":\"busy\"}"));
    }
    const auto cs = cache.stats();
    size_t n = static_cast<size_t>(std::snprintf(
        buf, cap,
        "{\"running\":%s,\"name\":\"%s\",\"sha256\":\"%s\",\"cache_hit\":%s,"
        "\"last_ok\":%s,\"last_aborted\":%s,\"last_steps\":%lu,\"last_trap\":\"%s\","
        "\"runs_total\":%lu,\"cache\":{\"hits\":%lu,\"misses\":%lu,\"evictions\":%lu,"
        "\"used_bytes\":%u,\"budget_bytes\":%u,\"entries\":%u},\"logs\":[",
        running_flag.load() ? "true" : "false",
        active_name.c_str(),
        active_sha.c_str(),
        active_cache_hit ? "true" : "false",
        last_result.ok ? "true" : "false",
        last_result.aborted ? "true" : "false",
        static_cast<unsigned long>(last_result.steps),
        last_result.trap ? last_result.trap : "",
        static_cast<unsigned long>(runs_total),
        static_cast<unsigned long>(cs.hits),
        static_cast<unsigned long>(cs.misses),
        static_cast<unsigned long>(cs.evictions),
        static_cast<unsigned>(cs.used_bytes),
        static_cast<unsigned>(cs.budget_bytes),
        static_cast<unsigned>(cs.entries)));
    for (size_t i = 0; i < log_count && n + 48 < cap; ++i) {
        n += static_cast<size_t>(std::snprintf(
            buf + n, cap - n, "%s{\"ms\":%ld,\"code\":%ld,\"val\":%ld}",
            i ? "," : "",
            static_cast<long>(logs[i].ms),
            static_cast<long>(logs[i].code),
            static_cast<long>(logs[i].value)));
    }
    if (n + 3 < cap) {
        buf[n++] = ']';
        buf[n++] = '}';
        buf[n] = '\0';
    }
    xSemaphoreGive(vm_mutex);
    return n;
}

size_t format_cache_json(char* buf, size_t cap) {
    ensure_mutex();
    if (!buf || cap == 0) return 0;
    if (!vm_mutex || xSemaphoreTake(vm_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return static_cast<size_t>(std::snprintf(buf, cap, "{\"error\":\"busy\"}"));
    }
    const auto cs = cache.stats();
    size_t n = static_cast<size_t>(std::snprintf(
        buf, cap,
        "{\"hits\":%lu,\"misses\":%lu,\"evictions\":%lu,\"used_bytes\":%u,\"budget_bytes\":%u,\"entries\":[",
        static_cast<unsigned long>(cs.hits),
        static_cast<unsigned long>(cs.misses),
        static_cast<unsigned long>(cs.evictions),
        static_cast<unsigned>(cs.used_bytes),
        static_cast<unsigned>(cs.budget_bytes)));
    const auto& entries = cache.entries();
    for (size_t i = 0; i < entries.size() && n + 160 < cap; ++i) {
        const auto& e = entries[i];
        n += static_cast<size_t>(std::snprintf(
            buf + n, cap - n,
            "%s{\"sha256\":\"%s\",\"name\":\"%s\",\"bytes\":%u,\"freq\":%.2f,\"score\":%.3f,\"pinned\":%s}",
            i ? "," : "",
            e.sha256_hex.c_str(),
            e.name.c_str(),
            static_cast<unsigned>(e.bytes.size()),
            e.freq_decayed,
            cache.score_of(e),
            e.pinned ? "true" : "false"));
    }
    if (n + 3 < cap) {
        buf[n++] = ']';
        buf[n++] = '}';
        buf[n] = '\0';
    }
    xSemaphoreGive(vm_mutex);
    return n;
}

void print_status() {
    const auto s = stats();
    printf("prog_running  %s\n", s.running ? "yes" : "no");
    printf("runs_total    %lu\n", static_cast<unsigned long>(s.runs_total));
    printf("last_ok       %s\n", s.last_ok ? "yes" : "no");
    printf("last_steps    %lu\n", static_cast<unsigned long>(s.last_steps));
    printf("cache_entries %u (%u / %u bytes)\n",
           static_cast<unsigned>(s.cache_entries),
           static_cast<unsigned>(s.cache_used_bytes),
           static_cast<unsigned>(s.cache_budget_bytes));
    printf("cache_hits    %lu (misses %lu, evictions %lu)\n",
           static_cast<unsigned long>(s.cache_hits),
           static_cast<unsigned long>(s.cache_misses),
           static_cast<unsigned long>(s.cache_evictions));
}

}  // namespace prog_vm
