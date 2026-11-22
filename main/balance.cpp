#include "balance.h"

#include <algorithm>
#include <cmath>

namespace balance {

void Robot::request(Request request) {
    const bool fallen = state_ == State::FALLEN;
    switch (request) {
        case Request::NONE:
            break;
        case Request::ARM:
            if (fallen || state_ == State::CALIBRATING) {
                arm_when_calibrated_ = true;
                if (fallen) {
                    enter(State::CALIBRATING);  // it fell, so it's lying still
                }
            } else if (state_ == State::READY) {
                enter(State::ARMED);
            }
            break;
        case Request::DISARM:
            arm_when_calibrated_ = false;
            if (fallen) {
                enter(State::CALIBRATING);
            } else if (state_ != State::CALIBRATING) {
                enter(State::READY);
            }
            break;
        case Request::TOGGLE_ARM:
            this->request(fallen || armed() ? Request::DISARM : Request::ARM);
            break;
        case Request::TAKEOFF:
            if (state_ == State::READY || state_ == State::ARMED) {
                reset_motion();
                enter(State::TAKEOFF);
            } else {
                this->request(Request::ARM);
            }
            break;
    }
}

void Robot::steer(float forward, float turn) {
    move_rate_ = forward * MAX_MOVE_RATE;
    turn_step_ = -turn * TURN_DPS * TICK_S;
}

void Robot::sensor_failed() {
    if (motors_live() || state_ == State::LEVELING) {
        fall(Fault::SENSOR);
    }
}

Motors Robot::step(const Imu& imu, const Gains& gains) {
    ++ticks_;
    average_up_g_ += (imu.up_g - average_up_g_) * AVERAGE_WEIGHT;  // whatever the state, so it's never stale
    switch (state_) {
        case State::CALIBRATING:
            calibrate_gyro(imu);
            break;
        case State::LEVELING:
            level(imu);
            break;
        default:
            estimate(imu, gains);
            break;
    }

    switch (state_) {
        case State::ARMED:
            if (upright(average_up_g_)) {
                enter(State::LEVELING);
            }
            break;
        case State::TAKEOFF:
            if (upright(average_up_g_)) {
                enter(State::BALANCING);
                drive(gains);
            } else if (ticks_ > TAKEOFF_TICKS) {
                enter(State::READY);  // didn't come up
            }
            break;
        case State::BALANCING:
            if (std::abs(pitch_) > gains.max_angle) {
                fall(Fault::TILT);
            } else {
                drive(gains);
                if (overpower_ticks_ > gains.overpower_ticks) {
                    fall(Fault::OVERPOWER);  // pushing against something, or stuck
                }
            }
            break;
        default:
            break;
    }
    return motors_;
}

bool Robot::armed() const {
    switch (state_) {
        case State::ARMED:
        case State::LEVELING:
        case State::TAKEOFF:
        case State::BALANCING:
            return true;
        case State::CALIBRATING:
            return arm_when_calibrated_;
        default:
            return false;
    }
}

void Robot::enter(State state) {
    state_ = state;
    ticks_ = 0;
    if (state != State::FALLEN) {
        fault_ = Fault::NONE;
    }
    pitch_dps_sum_ = yaw_dps_sum_ = forward_g_sum_ = 0;
    motors_ = state == State::TAKEOFF ? Motors{TAKEOFF_POWER, TAKEOFF_POWER} : Motors{};
}

void Robot::fall(Fault fault) {
    arm_when_calibrated_ = false;
    enter(State::FALLEN);
    fault_ = fault;
}

void Robot::estimate(const Imu& imu, const Gains& gains) {
    pitch_dps_ = imu.pitch_dps - pitch_dps_offset_;
    const float yaw_dps = imu.yaw_dps - yaw_dps_offset_;
    average_yaw_dps_ += (yaw_dps - average_yaw_dps_) * AVERAGE_WEIGHT;
    yaw_ += average_yaw_dps_ * TICK_S;
    // The gyro tracks quick changes; the accelerometer slowly pulls out its drift.
    const float accel_pitch = (imu.forward_g - forward_g_offset_) * DEG_PER_G;
    pitch_ += (pitch_dps_ + (accel_pitch - pitch_) * gains.filter) * TICK_S;
}

void Robot::calibrate_gyro(const Imu& imu) {
    if (ticks_ <= GYRO_SETTLE_TICKS) {
        return;
    }
    pitch_dps_sum_ += imu.pitch_dps;
    yaw_dps_sum_ += imu.yaw_dps;
    if (ticks_ == GYRO_SETTLE_TICKS + GYRO_SAMPLE_TICKS) {
        pitch_dps_offset_ = pitch_dps_sum_ / GYRO_SAMPLE_TICKS;
        yaw_dps_offset_ = yaw_dps_sum_ / GYRO_SAMPLE_TICKS;
        enter(arm_when_calibrated_ ? State::ARMED : State::READY);
    }
}

// Held upright: what the accelerometer reads now is zero tilt.
void Robot::level(const Imu& imu) {
    if (!upright(average_up_g_)) {
        enter(State::ARMED);  // let go too soon; start over once it's up again
        return;
    }
    if (ticks_ <= LEVEL_SETTLE_TICKS) {
        return;
    }
    forward_g_sum_ += imu.forward_g;
    yaw_dps_sum_ += imu.yaw_dps;
    if (ticks_ == LEVEL_SETTLE_TICKS + LEVEL_SAMPLE_TICKS) {
        forward_g_offset_ = forward_g_sum_ / LEVEL_SAMPLE_TICKS;
        yaw_dps_offset_ = yaw_dps_sum_ / LEVEL_SAMPLE_TICKS;
        reset_motion();
        pitch_ = pitch_dps_ = 0;
        enter(State::BALANCING);
    }
}

void Robot::reset_motion() {
    yaw_ = move_target_ = turn_target_ = 0;
    speed_ = distance_ = angle_i_ = rate_i_ = 0;
    power_ = power_left_ = power_right_ = 0;
    overpower_ticks_ = 0;
    left_ = right_ = Wheel{};
}

void Robot::drive(const Gains& gains) {
    const float turn_factor = std::abs(move_rate_) > MOVING_RATE
                                  ? std::clamp(-(power_right_ + power_left_) / TURN_POWER_SCALE, -1.0f, 1.0f)
                                  : 1.0f;
    turn_target_ += turn_step_ * turn_factor;
    move_target_ += MOVE_STEP * (move_rate_ + gains.trim / 100);
    speed_ += power_ * TICK_S;
    distance_ += gains.k_distance * (speed_ * TICK_S - move_target_);
    angle_i_ += gains.k_angle_i * pitch_ * TICK_S;
    rate_i_ += gains.k_rate_i * pitch_dps_ * TICK_S;
    power_ = angle_i_ + distance_ + gains.k_speed * speed_ + gains.k_angle * pitch_ + gains.k_rate * pitch_dps_ +
             rate_i_;
    overpower_ticks_ = std::abs(power_) > OVERPOWER ? overpower_ticks_ + 1 : 0;
    if (overpower_ticks_ > gains.overpower_ticks) {
        return;
    }
    power_ = std::clamp(power_, -gains.max_power, gains.max_power);
    const float yaw_power = (yaw_ - turn_target_) * gains.k_yaw;
    power_right_ = power_ - yaw_power;
    power_left_ = power_ + yaw_power;
    motors_ = {shape(power_left_ * gains.motor_gain, left_, gains),
               shape(power_right_ * gains.motor_gain, right_, gains)};
}

// Motor power for a wheel: at least enough to turn it, and a little more for the first
// ticks after it changes direction.
int Robot::shape(float power, Wheel& wheel, const Gains& gains) {
    const int clamped = static_cast<int>(std::clamp(power, -gains.max_power, gains.max_power));
    const int direction = (clamped > 0) - (clamped < 0);
    if (direction == 0) {
        wheel.direction = 0;
        return 0;
    }
    wheel.ticks = wheel.direction == direction ? std::min(wheel.ticks + 1, MAX_PUNCH_TICKS) : 0;
    wheel.direction = direction;
    const int deadband = std::lround(gains.deadband);
    const int least = wheel.ticks < gains.punch_ticks ? std::max<int>(std::lround(gains.punch), deadband) : deadband;
    return direction * std::max(std::abs(clamped), least);
}

bool Robot::upright(float up_g) {
    return std::abs(up_g) > UPRIGHT_G;  // either way up, as the Arduino firmware did
}

}  // namespace balance
