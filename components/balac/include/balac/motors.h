#pragma once

#include <cstdint>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "m5stickc_plus/i2c_device.h"

namespace balac {

// The BalaC base's two wheel motors, run by its own chip on the hat I2C bus. The base has its
// own power switch, so it can come and go: while it's away, set() checks for it about once
// a second instead of failing on every call.
class Motors {
public:
    static constexpr int MAX_POWER = 127;

    esp_err_t init(i2c_master_bus_handle_t bus);
    // Power for each wheel, -MAX_POWER..MAX_POWER (clamped), positive driving the robot
    // forward. Sends only what changed since the last call.
    esp_err_t set(int left, int right);
    bool present() const { return present_; }

private:
    static constexpr int UNKNOWN = MAX_POWER + 1;  // not a power, so the next set() sends

    esp_err_t send(uint8_t motor, int power, int& sent);

    i2c_master_bus_handle_t bus_ = nullptr;
    m5::I2cDevice device_;
    bool present_ = false;
    uint32_t last_check_ = 0;  // FreeRTOS ticks
    int sent_left_ = UNKNOWN;
    int sent_right_ = UNKNOWN;
};

}  // namespace balac
