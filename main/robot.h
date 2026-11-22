#pragma once

// The control loop: every balance::TICK_S it reads the IMU and the buttons, steps
// balance::Robot and drives the wheels, and after a fall it sounds the alarm. It has the
// second core to itself; other tasks reach it through these functions, safe from any task.

#include <cmath>
#include <cstdint>

#include "balance.h"
#include "esp_err.h"

namespace robot {

struct Telemetry {
    balance::State state = balance::State::CALIBRATING;
    balance::Fault fault = balance::Fault::NONE;
    bool armed = false;
    bool motors_live = false;     // the wheels may be turning
    bool base_present = false;    // the BalaC base answers
    float pitch_deg = NAN;
    float yaw_dps = NAN;
    balance::Motors motors;
    float imu_c = NAN;
    uint32_t max_tick_us = 0;     // the longest tick of the last second
    uint32_t overruns = 0;        // ticks that ran past their time, since startup
};

esp_err_t start();
// Requests queue up, in order; a full queue drops the newest.
void request(balance::Request request);
// The remote's stick, -1..1 each way. Without an update for STEER_TIMEOUT_US it goes back
// to the middle.
void steer(float forward, float turn);
Telemetry telemetry();

}  // namespace robot
