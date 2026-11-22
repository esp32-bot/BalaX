#include "board.h"
#include "commands.h"
#include "config.h"
#include "esp_console.h"
#include "esp_err.h"
#include "esp_pm.h"
#include "imu_calibration.h"
#include "lan_server.h"
#include "nvs_flash.h"
#include "remote.h"
#include "robot.h"
#include "settings.h"
#include "sounds.h"
#include "status.h"
#include "telemetry_stream.h"
#include "wifi.h"
#include "wifi_networks.h"

namespace {

void init_nvs() {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // Left over from another firmware or partition layout: start fresh.
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

// Looking for networks hops channels, which would cut off the remote for a moment.
bool may_scan_for_networks() {
    return !robot::telemetry().motors_live && !remote::link().live;
}

void start_console() {
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = CONSOLE_PROMPT;
    repl_config.task_stack_size = CONSOLE_STACK_BYTES;
    repl_config.task_priority = CONSOLE_PRIORITY;
    repl_config.task_core_id = CONSOLE_CORE;
    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_console_register_help_command());
    ESP_ERROR_CHECK(settings::register_commands());
    ESP_ERROR_CHECK(register_app_commands());
    esp_console_repl_t* repl = nullptr;
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}

}  // namespace

extern "C" void app_main() {
    init_nvs();
    const esp_pm_config_t pm = {
        .max_freq_mhz = CPU_MAX_MHZ,
        .min_freq_mhz = CPU_MIN_MHZ,
        .light_sleep_enable = false,
    };
    ESP_ERROR_CHECK(esp_pm_configure(&pm));
    ESP_ERROR_CHECK(init_board());
    ESP_ERROR_CHECK(settings::load_all());  // applies volume, brightness, gains & saved IMU calibration
    ESP_ERROR_CHECK(robot::start());
    // Only calibrate once on a fresh flash; future reboots use the saved NVS offsets immediately.
    if (!imu_cal::is_calibrated()) {
        robot::zero_imu();
    }
    board.speaker.play(STARTUP_MELODY);
    ESP_ERROR_CHECK(wifi::start(may_scan_for_networks));
    lan_server::set_stream_callback(telemetry_stream::pull_compressed);
    ESP_ERROR_CHECK(lan_server::start(LAN_AUTH_KEY));
    ESP_ERROR_CHECK(remote::start());
    ESP_ERROR_CHECK(start_status_screen());
    start_console();
}
