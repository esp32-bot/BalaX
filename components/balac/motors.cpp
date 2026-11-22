#include "balac/motors.h"

#include <algorithm>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace balac {

namespace {

constexpr uint16_t ADDRESS = 0x38;
// The speed the Arduino firmware used. Each write is [motor, int8 power].
constexpr uint32_t I2C_HZ = 100'000;
constexpr uint8_t LEFT = 0;
constexpr uint8_t RIGHT = 1;
constexpr int PROBE_TIMEOUT_MS = 10;
constexpr uint32_t CHECK_PERIOD_MS = 1000;

}  // namespace

esp_err_t Motors::init(i2c_master_bus_handle_t bus) {
    bus_ = bus;
    const esp_err_t err = device_.init(bus, ADDRESS, I2C_HZ);
    if (err == ESP_OK) {
        last_check_ = xTaskGetTickCount();
        present_ = i2c_master_probe(bus_, ADDRESS, PROBE_TIMEOUT_MS) == ESP_OK;
    }
    return err;
}

esp_err_t Motors::set(int left, int right) {
    if (!present_) {
        const uint32_t now = xTaskGetTickCount();
        if (now - last_check_ < pdMS_TO_TICKS(CHECK_PERIOD_MS)) {
            return ESP_ERR_NOT_FOUND;
        }
        last_check_ = now;
        if (i2c_master_probe(bus_, ADDRESS, PROBE_TIMEOUT_MS) != ESP_OK) {
            return ESP_ERR_NOT_FOUND;
        }
        present_ = true;
        sent_left_ = sent_right_ = UNKNOWN;
    }
    esp_err_t err = send(LEFT, left, sent_left_);
    if (err == ESP_OK) {
        err = send(RIGHT, -right, sent_right_);  // mounted facing the other way
    }
    if (err != ESP_OK) {
        present_ = false;
        last_check_ = xTaskGetTickCount();
    }
    return err;
}

esp_err_t Motors::send(uint8_t motor, int power, int& sent) {
    power = std::clamp(power, -MAX_POWER, MAX_POWER);
    if (power == sent) {
        return ESP_OK;
    }
    sent = UNKNOWN;
    const esp_err_t err = device_.write(motor, static_cast<uint8_t>(static_cast<int8_t>(power)));
    if (err == ESP_OK) {
        sent = power;
    }
    return err;
}

}  // namespace balac
