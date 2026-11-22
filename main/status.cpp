#include "status.h"

#include <cmath>
#include <cstdio>

#include "app_settings.h"
#include "battery_soc.h"
#include "board.h"
#include "config.h"
#include "cpu_stats.h"
#include "esp_heap_caps.h"
#include "esp_netif_ip_addr.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "gfx/canvas.h"
#include "m5stickc_plus/screen.h"
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

// Drawn here in full, then only the rows that changed go to the display.
uint16_t frame[m5::SCREEN_WIDTH * m5::SCREEN_HEIGHT];

portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
status::Values latest;  // guarded by `lock`

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
void read_robot(status::Values& values) {
    const robot::Telemetry robot = robot::telemetry();
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

// What's worth redrawing for at once rather than at the next period.
bool changed_at_a_glance(const status::Values& now, const status::Values& shown) {
    return now.state != shown.state || now.fault != shown.fault || now.remote_live != shown.remote_live ||
           std::isnan(now.motor_left) != std::isnan(shown.motor_left);
}

void status_task(void*) {
    gfx::Canvas canvas(frame, m5::SCREEN_WIDTH, m5::SCREEN_HEIGHT);
    status::Values values;
    status::Values shown;
    TickType_t wake = xTaskGetTickCount();
    for (uint32_t poll = 0;; ++poll) {
        const bool period = poll % POLLS_PER_PERIOD == 0;
        if (period) {
            cpu.sample();
            read_power(values);
            read_system(values);
        }
        read_robot(values);
        taskENTER_CRITICAL(&lock);
        latest = values;
        taskEXIT_CRITICAL(&lock);
        if (period || changed_at_a_glance(values, shown)) {
            status::draw(canvas, values);
            board.display.present(canvas.pixels());
            shown = values;
        }
        xTaskDelayUntil(&wake, pdMS_TO_TICKS(STATUS_POLL_MS));
    }
}

}  // namespace

esp_err_t start_status_screen() {
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
    read_robot(values);
    const std::string_view state = balance::name(values.state);
    const std::string_view fault = balance::name(values.fault);
    printf("%-13s %.*s\n", "state", static_cast<int>(state.size()), state.data());
    printf("%-13s %.*s\n", "fault", static_cast<int>(fault.size()), fault.data());
    for (const status::Field& field : status::FIELDS) {
        printf("%-13s %.*f\n", field.name, field.decimals, values.*field.value);
    }
    printf("%-13s %s\n", "wifi", values.wifi_phase);
    printf("%-13s %s\n", "wifi_ssid", values.wifi_ssid);
    printf("%-13s %s\n", "wifi_ip", values.wifi_ip);
    printf("%-13s %s\n", "ap_ssid", values.ap_ssid);
    printf("%-13s %s\n", "remote_live", values.remote_live ? "yes" : "no");
}
