#include "balance.h"

#include <algorithm>
#include <cmath>

namespace balance {

void Robot::use_saved_calibration() {
    external_calibration_ = true;
    pitch_dps_offset_ = 0;
    yaw_dps_offset_ = 0;
    if (state_ == State::CALIBRATING) {
        enter(arm_when_calibrated_ ? State::ARMED : State::READY);
    }
}

void Robot::request(Request request) {
    const bool fallen = state_ == State::FALLEN;
    switch (request) {
        case Request::NONE:
            break;
        case Request::ARM:
            if (fallen) {
                if (external_calibration_) {
                    enter(State::ARMED);
                } else {
                    arm_when_calibrated_ = true;
                    enter(State::CALIBRATING);  // it fell, so it's lying still
                }
            } else if (state_ == State::CALIBRATING) {
                arm_when_calibrated_ = true;
            } else if (state_ == State::READY) {
                enter(State::ARMED);
            }
            break;
        case Request::DISARM:
            arm_when_calibrated_ = false;
            if (fallen) {
                enter(external_calibration_ ? State::READY : State::CALIBRATING);
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

Motors Robot::step(const Imu& imu, const Gains& gains, const control::Config& ctrl) {
    ++ticks_;
    average_up_g_ += (imu.up_g - average_up_g_) * AVERAGE_WEIGHT;  // whatever the state, so it's never stale
    switch (state_) {
        case State::CALIBRATING:
            calibrate_gyro(imu);
            break;
        case State::LEVELING:
            level(imu, ctrl);
            break;
        default:
            estimate(imu, gains, ctrl);
            break;
    }

    switch (state_) {
        case State::ARMED:
            if (upright(average_up_g_)) {
                enter(State::LEVELING);
            }
            break;
        case State::TAKEOFF:
            if (ctrl.takeoff_mode() == control::TakeoffMode::ENERGY_SWINGUP) {
                bool ready_to_catch = false;
                const int cmd =
                    control::step_energy_swingup(ctrl, pitch_, pitch_dps_, gains.max_power, ready_to_catch);
                if (ready_to_catch || upright(average_up_g_)) {
                    // Keep fused pitch_, real pitch_dps_, and estimated wheel velocity for smooth LQR soft-catch!
                    const float last_cmd = 0.5f * static_cast<float>(motors_.left + motors_.right);
                    const float est_vel =
                        control::effective_effort(last_cmd, gains.deadband) * (ctrl.vel_gain / std::max(1.0f, ctrl.vel_drag));
                    enter(State::BALANCING);
                    ctrl_state_.vel = est_vel;
                    ctrl_state_.last_u = last_cmd;
                    ctrl_state_.prev_u_eff = control::effective_effort(last_cmd, gains.deadband);
                    drive(gains, ctrl);
                } else if (ticks_ > TAKEOFF_TICKS * 2) {
                    enter(State::READY);  // didn't come up
                } else {
                    motors_ = {cmd, cmd};
                }
            } else {
                if (upright(average_up_g_)) {
                    pitch_ = (imu.forward_g - forward_g_offset_) * DEG_PER_G;
                    pitch_dps_ = 0;
                    enter(State::BALANCING);
                    drive(gains, ctrl);
                } else if (ticks_ > TAKEOFF_TICKS) {
                    enter(State::READY);  // didn't come up
                }
            }
            break;
        case State::BALANCING:
            if (std::abs(pitch_) > gains.max_angle) {
                fall(Fault::TILT);
            } else {
                drive(gains, ctrl);
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
    pitch_dps_sum_ = yaw_dps_sum_ = forward_g_sum_ = fused_pitch_sum_ = 0;
    motors_ = state == State::TAKEOFF ? Motors{TAKEOFF_POWER, TAKEOFF_POWER} : Motors{};
}

void Robot::fall(Fault fault) {
    arm_when_calibrated_ = false;
    enter(State::FALLEN);
    fault_ = fault;
}

void Robot::estimate(const Imu& imu, const Gains& gains, const control::Config& ctrl) {
    if (ctrl.estimator_mode() == control::EstimatorMode::VQF_6DOF) {
        if (imu.has_fused) {
            pitch_dps_ = imu.fused_pitch_dps;
            yaw_dps_ = imu.fused_yaw_dps;
            average_yaw_dps_ += (yaw_dps_ - average_yaw_dps_) * AVERAGE_WEIGHT;
            yaw_ += yaw_dps_ * TICK_S;
            pitch_ = imu.fused_pitch_deg - fused_pitch_offset_;
            return;
        }
        pitch_dps_ = imu.pitch_dps - pitch_dps_offset_;
        yaw_dps_ = imu.yaw_dps - yaw_dps_offset_;
        average_yaw_dps_ += (yaw_dps_ - average_yaw_dps_) * AVERAGE_WEIGHT;
        yaw_ += average_yaw_dps_ * TICK_S;
        control::step_vqf_fallback_estimator(imu.forward_g, imu.up_g, forward_g_offset_, pitch_dps_, TICK_S, pitch_);
        return;
    }
    pitch_dps_ = imu.pitch_dps - pitch_dps_offset_;
    yaw_dps_ = imu.yaw_dps - yaw_dps_offset_;
    average_yaw_dps_ += (yaw_dps_ - average_yaw_dps_) * AVERAGE_WEIGHT;
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
    forward_g_sum_ += imu.forward_g;
    if (ticks_ == GYRO_SETTLE_TICKS + GYRO_SAMPLE_TICKS) {
        pitch_dps_offset_ = pitch_dps_sum_ / GYRO_SAMPLE_TICKS;
        yaw_dps_offset_ = yaw_dps_sum_ / GYRO_SAMPLE_TICKS;
        const float mean_forward_g = forward_g_sum_ / GYRO_SAMPLE_TICKS;
        pitch_ = (mean_forward_g - forward_g_offset_) * DEG_PER_G;
        pitch_dps_ = 0;
        enter(arm_when_calibrated_ ? State::ARMED : State::READY);
    }
}

// Held upright: what the accelerometer reads now is zero tilt.
void Robot::level(const Imu& imu, const control::Config& ctrl) {
    (void)ctrl;
    if (!upright(average_up_g_)) {
        enter(State::ARMED);  // let go too soon; start over once it's up again
        return;
    }
    if (ticks_ <= LEVEL_SETTLE_TICKS) {
        return;
    }
    forward_g_sum_ += imu.forward_g;
    fused_pitch_sum_ += imu.has_fused ? imu.fused_pitch_deg : 0.0f;
    yaw_dps_sum_ += imu.yaw_dps;
    if (ticks_ == LEVEL_SETTLE_TICKS + LEVEL_SAMPLE_TICKS) {
        forward_g_offset_ = forward_g_sum_ / LEVEL_SAMPLE_TICKS;
        fused_pitch_offset_ = imu.has_fused ? (fused_pitch_sum_ / LEVEL_SAMPLE_TICKS) : 0.0f;
        if (!external_calibration_) {
            yaw_dps_offset_ = yaw_dps_sum_ / LEVEL_SAMPLE_TICKS;
        }
        reset_motion();
        pitch_ = pitch_dps_ = 0;
        enter(State::BALANCING);
    }
}

void Robot::reset_motion() {
    yaw_ = yaw_dps_ = move_target_ = turn_target_ = turn_rate_cmd_ = 0;
    speed_ = distance_ = angle_i_ = rate_i_ = 0;
    power_ = power_left_ = power_right_ = 0;
    overpower_ticks_ = 0;
    left_ = right_ = Wheel{};
    ctrl_state_.reset();
}

void Robot::drive(const Gains& gains, const control::Config& ctrl) {
    ++ctrl_state_.balance_ticks;
    const bool sota_ctrl = ctrl.controller_mode() == control::ControllerMode::LQR_ESO;
    const float v_cmd = move_rate_ + gains.trim / 100.0f;
    // Cap cruise speed at 0.15 m/s (~55 PWM back-EMF) so 70+ PWM of balance recovery headroom always remains.
    const float v_cmd_mps = std::clamp(v_cmd / MAX_MOVE_RATE, -1.0f, 1.0f) * 0.15f;
    const float turn_factor = (!sota_ctrl && std::abs(move_rate_) > MOVING_RATE)
                                  ? std::clamp(-(power_right_ + power_left_) / TURN_POWER_SCALE, -1.0f, 1.0f)
                                  : 1.0f;
    if (!sota_ctrl) {
        turn_target_ += turn_step_ * turn_factor;
    }

    float motor_scale = gains.motor_gain;
    if (sota_ctrl) {
        // Stage 2 (SOTA): 5-State Lag-Compensated LQR + Exact-ZOH Velocity Observer + Bumpless Online CG ESO.
        power_ = control::step_lqr_eso(ctrl_state_, ctrl, pitch_, pitch_dps_, v_cmd_mps, gains.deadband,
                                       gains.max_power, TICK_S);
        speed_ = ctrl_state_.vel;
        distance_ = ctrl_state_.pos;
        angle_i_ = ctrl_state_.eq_deg;
        rate_i_ = 0.0f;
        motor_scale = 1.0f;  // LQR gains already output directly in motor-command units (-127..127)
        overpower_ticks_ = std::abs(power_) > gains.max_power * 1.35f ? overpower_ticks_ + 1 : 0;
    } else {
        // Stage 2 (Classic 2020): exact original Arduino PID + raw speed/distance integrators.
        // When composed with Lyapunov Perch, bleed integrators smoothly while perched to prevent windup.
        if (ctrl_state_.perched) {
            speed_ *= 0.90f;
            distance_ *= 0.90f;
            angle_i_ *= 0.90f;
            rate_i_ *= 0.90f;
            power_ = gains.k_angle * pitch_ + gains.k_rate * pitch_dps_;
            ctrl_state_.vel = std::clamp(speed_ * 0.05f, -2.0f, 2.0f);
            overpower_ticks_ = 0;
        } else {
            move_target_ += MOVE_STEP * v_cmd;
            speed_ += power_ * TICK_S;
            distance_ += gains.k_distance * (speed_ * TICK_S - move_target_);
            angle_i_ += gains.k_angle_i * pitch_ * TICK_S;
            rate_i_ += gains.k_rate_i * pitch_dps_ * TICK_S;
            power_ = angle_i_ + distance_ + gains.k_speed * speed_ + gains.k_angle * pitch_ +
                     gains.k_rate * pitch_dps_ + rate_i_;
            ctrl_state_.vel = std::clamp(speed_ * 0.05f, -2.0f, 2.0f);
            overpower_ticks_ = std::abs(power_) > OVERPOWER ? overpower_ticks_ + 1 : 0;
        }
    }

    if (overpower_ticks_ > gains.overpower_ticks) {
        return;
    }

    // Stage 3: Event-Triggered Lyapunov Perch Gate (intentional zero-motor-power passive standing).
    const float raw_motor_cmd = power_ * motor_scale;
    const float active_turn = sota_ctrl ? (std::abs(turn_step_) + std::abs(turn_rate_cmd_) * TICK_S) : turn_step_;
    if (control::step_perch_gate(ctrl_state_, ctrl, pitch_, pitch_dps_, v_cmd, active_turn, raw_motor_cmd,
                                 gains.max_power)) {
        power_ = 0.0f;
        power_left_ = 0.0f;
        power_right_ = 0.0f;
        turn_rate_cmd_ = 0.0f;
        turn_target_ = yaw_;
        left_ = right_ = Wheel{};
        motors_ = {0, 0};
        ctrl_state_.last_u = 0.0f;
        ++ctrl_state_.perch_ticks;
        return;
    }

    power_ = std::clamp(power_, -gains.max_power, gains.max_power);
    float yaw_power = 0.0f;
    if (sota_ctrl) {
        // SOTA zero-windup yaw-rate + heading-lock controller with strict balance-priority headroom:
        // Balance ALWAYS gets 100% of both wheels before yaw differential is allocated, so turning
        // can never starve pitch recovery or trigger a tilt failsafe.
        const float target_yaw_dps = turn_step_ / TICK_S;
        constexpr float TURN_SLEW_DPS2 = 240.0f;
        const float max_dr = TURN_SLEW_DPS2 * TICK_S;
        turn_rate_cmd_ += std::clamp(target_yaw_dps - turn_rate_cmd_, -max_dr, max_dr);

        float raw_yaw = 0.0f;
        if (std::abs(turn_rate_cmd_) > 1.0f) {
            raw_yaw = -turn_rate_cmd_ * 0.24f + (yaw_dps_ - turn_rate_cmd_) * 0.35f;
            turn_target_ = yaw_;  // zero heading windup while actively turning
        } else {
            turn_target_ = std::clamp(turn_target_, yaw_ - 12.0f, yaw_ + 12.0f);
            raw_yaw = (yaw_ - turn_target_) * 1.4f + yaw_dps_ * 0.18f;
        }
        const float yaw_headroom = std::max(0.0f, gains.max_power - std::abs(power_));
        const float yaw_cap = std::min(gains.max_power * 0.28f, yaw_headroom);
        yaw_power = std::clamp(raw_yaw, -yaw_cap, yaw_cap);
    } else {
        yaw_power = (yaw_ - turn_target_) * gains.k_yaw;
    }

    power_right_ = power_ - yaw_power;
    power_left_ = power_ + yaw_power;
    motors_ = {shape(power_left_ * motor_scale, left_, gains, ctrl),
               shape(power_right_ * motor_scale, right_, gains, ctrl)};

    ctrl_state_.last_u = 0.5f * static_cast<float>(motors_.left + motors_.right);
    if (motors_.left != 0 || motors_.right != 0) {
        ++ctrl_state_.active_ticks;
        const float l = static_cast<float>(motors_.left);
        const float r = static_cast<float>(motors_.right);
        ctrl_state_.energy_u2 += 0.5f * (l * l + r * r) * TICK_S;
    }
}

// Stage 4: Motor power shaper (Classic 2020 punch vs 2026 smooth stiction compensator).
int Robot::shape(float power, Wheel& wheel, const Gains& gains, const control::Config& ctrl) {
    if (ctrl.shaper_mode() == control::ShaperMode::SMOOTH_STICTION) {
        return control::shape_smooth_stiction(power, gains.deadband, gains.punch, gains.max_power,
                                              wheel.direction, wheel.ticks, ctrl.coast_u);
    }
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
