#include "power_mode.h"

#include <atomic>

#include "app_settings.h"
#include "board.h"
#include "config.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_rom_sys.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "m5stickc_plus/i2c_device.h"
#include "m5stickc_plus/pins.h"
#include "prog_vm.h"
#include "status.h"
#include "telemetry_stream.h"
#include "wifi.h"

namespace power_mode {

namespace {

constexpr char TAG[] = "power";
constexpr uint32_t OFF_FLUSH_DELAY_MS = 180;
constexpr uint32_t OFF_TASK_STACK_BYTES = 3072;
constexpr uint8_t AXP192_ADDR = 0x34;
constexpr uint8_t AXP192_REG_VBUS_V_H = 0x5A;

// Survives ESP32 deep sleep (RTC SLOW memory).
RTC_DATA_ATTR bool rtc_in_usb_sleep = false;
RTC_DATA_ATTR bool rtc_usb_sleep_forced = false;
bool s_boot_to_wifi_sleep = false;

SemaphoreHandle_t mode_mutex = nullptr;
std::atomic<idle_power::PowerMode> active_mode{idle_power::PowerMode::AWAKE};
std::atomic<bool> wake_from_button_pending{false};
std::atomic<bool> off_scheduled{false};
std::atomic<bool> off_forced_flag{false};
std::atomic<bool> wifi_sleep_forced_flag{false};
int wifi_sleep_usb_samples = 0;
idle_power::IdlePolicy policy{0};
int64_t last_led_pulse_us = 0;

void pulse_led_us(uint32_t us) {
    gpio_config_t io{};
    io.pin_bit_mask = 1ULL << static_cast<uint32_t>(m5::LED);
    io.mode = GPIO_MODE_OUTPUT;
    gpio_config(&io);
    gpio_set_level(m5::LED, 0);  // active-LOW ON
    esp_rom_delay_us(us);
    gpio_set_level(m5::LED, 1);  // OFF
}

// Fast minimal VBUS voltage read (~0.5 ms) during RTC timer wake without booting LCD/Wi-Fi/IMU.
float read_usb_v_minimal() {
    i2c_master_bus_handle_t bus = nullptr;
    if (m5::new_i2c_bus(m5::INTERNAL_I2C_PORT, m5::INTERNAL_I2C_SDA, m5::INTERNAL_I2C_SCL, &bus) != ESP_OK) {
        return NAN;
    }
    i2c_device_config_t dev_cfg{};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = AXP192_ADDR;
    dev_cfg.scl_speed_hz = 400000;
    i2c_master_dev_handle_t dev = nullptr;
    float usb_v = NAN;
    if (i2c_master_bus_add_device(bus, &dev_cfg, &dev) == ESP_OK) {
        const uint8_t reg = AXP192_REG_VBUS_V_H;
        uint8_t buf[2] = {0, 0};
        if (i2c_master_transmit_receive(dev, &reg, 1, buf, 2, 50) == ESP_OK) {
            const uint16_t raw = static_cast<uint16_t>((static_cast<uint16_t>(buf[0]) << 4) | (buf[1] & 0x0F));
            usb_v = static_cast<float>(raw) * 0.0017f;  // 1.7 mV / LSB
        }
        i2c_master_bus_rm_device(dev);
    }
    i2c_del_master_bus(bus);
    return usb_v;
}

void arm_usb_sleep_wakeups() {
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
    esp_sleep_enable_ext0_wakeup(m5::BUTTON_A, 0);
    esp_sleep_enable_timer_wakeup(static_cast<uint64_t>(idle_power::USB_SLEEP_POLL_S) * 1000000ULL);
}

void apply_awake_pm() {
    const esp_pm_config_t pm = {
        .max_freq_mhz = CPU_MAX_MHZ,
        .min_freq_mhz = CPU_MIN_MHZ,
        .light_sleep_enable = false,
    };
    esp_pm_configure(&pm);
}

void apply_sleep_pm() {
    // Allow automatic light sleep + 40 MHz XTAL idle between Wi-Fi DTIM beacons while sleeping.
    const esp_pm_config_t pm = {
        .max_freq_mhz = CPU_MAX_MHZ,
        .min_freq_mhz = CPU_SLEEP_MIN_MHZ,
        .light_sleep_enable = true,
    };
    if (esp_pm_configure(&pm) != ESP_OK) {
        const esp_pm_config_t fallback = {
            .max_freq_mhz = CPU_MAX_MHZ,
            .min_freq_mhz = CPU_SLEEP_MIN_MHZ,
            .light_sleep_enable = false,
        };
        esp_pm_configure(&fallback);
    }
}

void enable_light_sleep_wakeups() {
    // Let UART0 console activity and Button A wake automatic light sleep cleanly.
    uart_set_wakeup_threshold(UART_NUM_0, 3);
    esp_sleep_enable_uart_wakeup(UART_NUM_0);
    gpio_wakeup_enable(m5::BUTTON_A, GPIO_INTR_LOW_LEVEL);
    esp_sleep_enable_gpio_wakeup();
}

void disable_light_sleep_wakeups() {
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_UART);
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO);
    gpio_wakeup_disable(m5::BUTTON_A);
}

void usb_sleep_task(void*) {
    // Give HTTP / USB console response a moment to flush cleanly before cutting Wi-Fi and CPU.
    vTaskDelay(pdMS_TO_TICKS(OFF_FLUSH_DELAY_MS));

    prog_vm::stop();
    telemetry_stream::stop_and_free();
    robot::set_sleeping(true);
    board.led.set(false);
    board.speaker.set_volume(0);

    // Stop Wi-Fi cleanly so RF draws 0 mA during USB charging.
    esp_wifi_stop();

    // Cut backlight (LDO2), LCD logic (LDO3), 5V Grove boost (EXTEN), and mic LDO (LDOio0)
    // while keeping DC-DC1 (ESP32 deep sleep) and 100 mA LiPo charger ON so USB EN reset wakes it.
    board.pmic.prepare_usb_off();

    rtc_in_usb_sleep = true;
    rtc_usb_sleep_forced = off_forced_flag.load(std::memory_order_acquire);
    arm_usb_sleep_wakeups();

    ESP_LOGI(TAG, "Entering usb-sleep (deep sleep, 10s LED/USB poll). Wake via USB EN, Button A, or unplugging USB.");
    vTaskDelay(pdMS_TO_TICKS(20));
    esp_deep_sleep_start();
}

}  // namespace

void handle_usb_sleep_wakeup() {
    const esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause == ESP_SLEEP_WAKEUP_TIMER && rtc_in_usb_sleep) {
        const float usb_v = read_usb_v_minimal();
        if (idle_power::should_switch_usb_sleep_to_wifi(usb_v, rtc_usb_sleep_forced)) {
            // USB was unplugged during usb-sleep -> silently boot into wifi-sleep so robot stays reachable!
            rtc_in_usb_sleep = false;
            rtc_usb_sleep_forced = false;
            s_boot_to_wifi_sleep = true;
            return;
        }
        // Still plugged in (or forced): emit ultra-faint slow LED whisper and return immediately to deep sleep.
        pulse_led_us(idle_power::USB_SLEEP_LED_PULSE_US);
        gpio_reset_pin(m5::LED);
        arm_usb_sleep_wakeups();
        esp_deep_sleep_start();
    }
    // Any non-timer wakeup (USB RTS->EN reset, Button A ext0, power-on) boots awake normally.
    rtc_in_usb_sleep = false;
    rtc_usb_sleep_forced = false;
    s_boot_to_wifi_sleep = false;
}

bool boot_to_wifi_sleep() {
    return s_boot_to_wifi_sleep;
}

esp_err_t init() {
    mode_mutex = xSemaphoreCreateMutex();
    if (!mode_mutex) {
        return ESP_ERR_NO_MEM;
    }
    policy.note_activity(esp_timer_get_time());
    return ESP_OK;
}

idle_power::PowerMode current() {
    return active_mode.load(std::memory_order_acquire);
}

bool is_sleeping() {
    return current() != idle_power::PowerMode::AWAKE;
}

int64_t idle_seconds() {
    if (!mode_mutex || xSemaphoreTake(mode_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return 0;
    }
    const int64_t secs = policy.idle_seconds(esp_timer_get_time());
    xSemaphoreGive(mode_mutex);
    return secs;
}

void note_activity() {
    if (!mode_mutex || xSemaphoreTake(mode_mutex, pdMS_TO_TICKS(20)) != pdTRUE) {
        return;
    }
    policy.note_activity(esp_timer_get_time());
    xSemaphoreGive(mode_mutex);
}

void pulse_wifi_sleep_led() {
    if (active_mode.load(std::memory_order_acquire) != idle_power::PowerMode::WIFI_SLEEP) {
        return;
    }
    const int64_t now_us = esp_timer_get_time();
    const int64_t period_us = static_cast<int64_t>(idle_power::WIFI_SLEEP_LED_PERIOD_MS) * 1000LL;
    if (last_led_pulse_us != 0 && (now_us - last_led_pulse_us) < period_us) {
        return;
    }
    last_led_pulse_us = now_us;
    board.led.set(true);
    esp_rom_delay_us(idle_power::WIFI_SLEEP_LED_PULSE_US);
    board.led.set(false);
}

static idle_power::GuardResult enter_sleep_impl(bool force, int64_t stationary_since_us, bool require_ready) {
    if (!mode_mutex || xSemaphoreTake(mode_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
        return {false, "Power manager busy."};
    }
    if (off_scheduled.load(std::memory_order_acquire)) {
        xSemaphoreGive(mode_mutex);
        return {false, "Entering usb-sleep."};
    }
    if (active_mode.load(std::memory_order_acquire) == idle_power::PowerMode::WIFI_SLEEP) {
        xSemaphoreGive(mode_mutex);
        return {true, nullptr};
    }

    const robot::Telemetry rt = robot::telemetry();
    if (stationary_since_us > 0 &&
        (rt.armed || rt.motors_live || rt.stationary_since_us != stationary_since_us ||
         !idle_power::fresh_stationary_sample(esp_timer_get_time(), rt.stationary_since_us, rt.sampled_at_us) ||
         (require_ready && rt.state != balance::State::READY))) {
        xSemaphoreGive(mode_mutex);
        return {false, "Automatic sleep requires a disarmed, stationary robot."};
    }
    const idle_power::GuardResult guard = idle_power::can_enter_sleep(rt.motors_live, force);
    if (!guard.ok) {
        xSemaphoreGive(mode_mutex);
        return guard;
    }

    // Stop active workloads before parking Core 1 and peripherals.
    prog_vm::stop();
    telemetry_stream::stop_and_free();
    robot::set_sleeping(true);
    board.led.set(false);
    board.pmic.set_idle_rails(true);
    wifi::set_sleep_mode(true);
    enable_light_sleep_wakeups();
    apply_sleep_pm();
    last_led_pulse_us = 0;
    wifi_sleep_forced_flag.store(force, std::memory_order_release);
    wifi_sleep_usb_samples = 0;

    active_mode.store(idle_power::PowerMode::WIFI_SLEEP, std::memory_order_release);
    ESP_LOGI(TAG, "Entered wifi-sleep (WIFI_PS_MAX_MODEM + light sleep, IMU/LCD/EXTEN off, faint 4s LED pulse)");
    xSemaphoreGive(mode_mutex);
    return {true, nullptr};
}

idle_power::GuardResult enter_sleep(bool force) {
    return enter_sleep_impl(force, 0, false);
}

void wake() {
    if (!mode_mutex || xSemaphoreTake(mode_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
        return;
    }
    wake_from_button_pending.store(false, std::memory_order_release);
    policy.note_activity(esp_timer_get_time());

    if (active_mode.load(std::memory_order_acquire) == idle_power::PowerMode::AWAKE) {
        xSemaphoreGive(mode_mutex);
        return;
    }

    // Restore full 80 MHz CPU/APB first before waking peripherals & Core 1.
    apply_awake_pm();
    disable_light_sleep_wakeups();
    wifi::set_sleep_mode(false);
    board.pmic.set_idle_rails(false);
    board.pmic.set_backlight(brightness.get());
    board.led.set(false);
    robot::set_sleeping(false);

    active_mode.store(idle_power::PowerMode::AWAKE, std::memory_order_release);
    status_lcd::request_repaint();
    ESP_LOGI(TAG, "Woke from wifi-sleep (80 MHz, 100 Hz balance loop active)");
    xSemaphoreGive(mode_mutex);
}

void request_wake_from_button() {
    wake_from_button_pending.store(true, std::memory_order_release);
    status_lcd::notify_task();
}

idle_power::GuardResult enter_off(bool force) {
    if (!mode_mutex || xSemaphoreTake(mode_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
        return {false, "Power manager busy."};
    }
    if (off_scheduled.load(std::memory_order_acquire)) {
        xSemaphoreGive(mode_mutex);
        return {true, nullptr};
    }

    const robot::Telemetry rt = robot::telemetry();
    const float usb_v = board.pmic.usb_v();
    const idle_power::GuardResult guard = idle_power::can_enter_off(rt.motors_live, usb_v, force);
    if (!guard.ok) {
        xSemaphoreGive(mode_mutex);
        return guard;
    }

    off_forced_flag.store(force, std::memory_order_release);
    off_scheduled.store(true, std::memory_order_release);
    active_mode.store(idle_power::PowerMode::USB_SLEEP, std::memory_order_release);
    const BaseType_t ok =
        xTaskCreatePinnedToCore(usb_sleep_task, "usb_sleep", OFF_TASK_STACK_BYTES, nullptr, 5, nullptr, 0);
    if (ok != pdPASS) {
        off_scheduled.store(false, std::memory_order_release);
        active_mode.store(idle_power::PowerMode::AWAKE, std::memory_order_release);
        xSemaphoreGive(mode_mutex);
        return {false, "Failed to schedule usb-sleep task (out of memory)."};
    }
    xSemaphoreGive(mode_mutex);
    return {true, nullptr};
}

void poll_from_status_task(const status::Values& status, const robot::Telemetry& robot_state, bool stream_active,
                           bool prog_running) {
    if (off_scheduled.load(std::memory_order_acquire)) {
        return;
    }
    if (wake_from_button_pending.exchange(false, std::memory_order_acq_rel)) {
        wake();
        return;
    }
    if (active_mode.load(std::memory_order_acquire) == idle_power::PowerMode::WIFI_SLEEP) {
        pulse_wifi_sleep_led();
        const float usb_v = board.pmic.usb_v();
        if (idle_power::should_switch_wifi_sleep_to_usb(
                usb_v, wifi_sleep_forced_flag.load(std::memory_order_acquire), wifi_sleep_usb_samples)) {
            ESP_LOGI(TAG, "USB detected (%.2f V) during wifi-sleep -> switching to usb-sleep", usb_v);
            enter_off(false);
            return;
        }
    } else {
        wifi_sleep_usb_samples = 0;
    }
    if (!mode_mutex || xSemaphoreTake(mode_mutex, pdMS_TO_TICKS(20)) != pdTRUE) {
        return;
    }
    const idle_power::PolicyInputs in = {
        .sleeping = active_mode.load(std::memory_order_acquire) != idle_power::PowerMode::AWAKE,
        .motors_live = robot_state.motors_live,
        .armed = robot_state.armed,
        .state_ready = robot_state.state == balance::State::READY,
        .stream_active = stream_active,
        .remote_live = status.remote_live,
        .prog_running = prog_running,
        .stationary_since_us = robot_state.stationary_since_us,
        .sampled_at_us = robot_state.sampled_at_us,
        .pmic_c = status.pmic_c,
        .imu_c = status.imu_c,
        .idle_sleep_s = idle_sleep_s.get(),
    };
    const idle_power::AutoAction action = policy.evaluate(esp_timer_get_time(), in);
    xSemaphoreGive(mode_mutex);

    if (action == idle_power::AutoAction::THERMAL_SLEEP &&
        enter_sleep_impl(false, in.stationary_since_us, false).ok) {
        ESP_LOGW(TAG, "Thermal protection triggered (PMIC %.1f C, IMU %.1f C): entered wifi-sleep",
                 status.pmic_c, status.imu_c);
    } else if (action == idle_power::AutoAction::IDLE_SLEEP &&
               enter_sleep_impl(false, in.stationary_since_us, true).ok) {
        ESP_LOGI(TAG, "Idle timeout reached (%us): entered wifi-sleep",
                 static_cast<unsigned>(idle_sleep_s.get()));
    }
}

}  // namespace power_mode
