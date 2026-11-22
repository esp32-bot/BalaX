#pragma once

// Portable 100 Hz control-loop trace sample for `telemetry_stream.h` (usable on ESP-IDF and host tests).

#include <cstdint>

#include "balance.h"

namespace robot {

struct TraceSample {
    uint32_t tick = 0;
    balance::State state = balance::State::CALIBRATING;
    balance::Fault fault = balance::Fault::NONE;
    int8_t motor_l = 0;
    int8_t motor_r = 0;
    float pitch_deg = 0;
    float pitch_dps = 0;
    float angle_i = 0;
    float rate_i = 0;
    float speed = 0;
    float distance = 0;
    float power = 0;
    // Full 6-axis IMU & yaw state at 100 Hz
    float accel_x_g = 0;
    float accel_y_g = 0;
    float accel_z_g = 0;
    float gyro_x_dps = 0;
    float gyro_y_dps = 0;
    float gyro_z_dps = 0;
    float yaw_deg = 0;
};

}  // namespace robot
