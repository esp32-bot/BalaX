#pragma once

// The control loop: every balance::TICK_S it reads the IMU and the buttons, steps
// balance::Robot and drives the wheels, and after a fall it sounds the alarm. It has the
// second core to itself; other tasks reach it through these functions, safe from any task.

#include <cmath>
#include <cstddef>
#include <cstdint>

#include "balance.h"
#include "esp_err.h"
#include "imu_fusion.h"
#include "trace_sample.h"

namespace robot {

struct Telemetry {
    balance::State state = balance::State::CALIBRATING;
    balance::Fault fault = balance::Fault::NONE;
    bool armed = false;
    bool motors_live = false;     // the wheels may be turning
    bool base_present = false;    // the BalaC base answers
    float pitch_deg = NAN;
    float yaw_dps = NAN;
    imu_fusion::EulerDeg fused_deg{};
    bool imu_resting = false;
    balance::Motors motors;
    balance::DebugTerms terms;
    float imu_c = NAN;
    uint32_t max_tick_us = 0;     // the longest tick of the last second
    uint32_t overruns = 0;        // ticks that ran past their time, since startup
};

struct ZeroImuResult {
    bool ok = false;
    bool moved = false;
    bool preserved_6face = false;
    float gyro_bx = 0;
    float gyro_by = 0;
    float gyro_bz = 0;
    float acc_bx = 0;
    float acc_by = 0;
    float acc_bz = 0;
    float max_gyro_spread_dps = 0;
};

esp_err_t start();
// Requests queue up, in order; a full queue drops the newest.
void request(balance::Request request);
// The remote's stick, -1..1 each way. Without an update for STEER_TIMEOUT_US it goes back
// to the middle.
void steer(float forward, float turn);
Telemetry telemetry();

// Captures ~1.0s of stationary raw IMU samples, saves biases to NVS flash (preserving
// an existing 6-face accel calibration if present), resets heading/online bias, and
// disables future boot-time calibration waits.
ZeroImuResult zero_imu();
// Zeroes fused yaw heading without touching flash calibration.
void reset_heading();
// Re-seeds on-device 6-DOF fusion and clears online bias (call after updating flash cal).
void reset_fusion();

}  // namespace robot
