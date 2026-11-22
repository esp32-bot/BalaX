#include "board.h"

#include "esp_check.h"
#include "m5stickc_plus/i2c_device.h"
#include "m5stickc_plus/pins.h"

namespace {

constexpr char TAG[] = "board";

}  // namespace

Board board;

esp_err_t init_board() {
    i2c_master_bus_handle_t bus = nullptr;
    ESP_RETURN_ON_ERROR(m5::new_i2c_bus(m5::INTERNAL_I2C_PORT, m5::INTERNAL_I2C_SDA, m5::INTERNAL_I2C_SCL, &bus),
                        TAG, "internal I2C bus");
    // The power chip first: it switches on the display's power.
    ESP_RETURN_ON_ERROR(board.pmic.init(bus), TAG, "AXP192 power chip");
    ESP_RETURN_ON_ERROR(board.imu.init(bus), TAG, "MPU6886 IMU");
    ESP_RETURN_ON_ERROR(board.display.init(), TAG, "display");
    ESP_RETURN_ON_ERROR(board.speaker.init(), TAG, "speaker");
    ESP_RETURN_ON_ERROR(board.led.init(), TAG, "LED");
    ESP_RETURN_ON_ERROR(board.button_a.init(m5::BUTTON_A), TAG, "button A");
    i2c_master_bus_handle_t hat_bus = nullptr;
    ESP_RETURN_ON_ERROR(m5::new_i2c_bus(m5::HAT_I2C_PORT, m5::HAT_SDA, m5::HAT_SCL, &hat_bus), TAG, "hat I2C bus");
    ESP_RETURN_ON_ERROR(board.motors.init(hat_bus), TAG, "BalaC motors");
    return ESP_OK;
}
