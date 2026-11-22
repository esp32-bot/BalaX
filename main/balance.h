#pragma once

// Balancing: the robot's states, its tilt from the IMU, and the wheel power that keeps it
// up. The control loop (robot.cpp) calls Robot::step every TICK_S. Plain C++ with no
// hardware, so laptop tests (host/) can run it.

#include <cstdint>
#include <string_view>

#include "balance_config.h"

namespace balance {

// An IMU reading in the robot's axes, standing: pitch tips it forward and back, yaw turns
// it, and gravity is along `up`.
struct Imu {
    float pitch_dps;
    float yaw_dps;
    float forward_g;
    float up_g;
};

struct Gains {
    float k_angle = DEFAULT_K_ANGLE;
    float k_rate = DEFAULT_K_RATE;
    float k_angle_i = DEFAULT_K_ANGLE_I;
    float k_rate_i = DEFAULT_K_RATE_I;
    float k_yaw = DEFAULT_K_YAW;
    float k_distance = DEFAULT_K_DISTANCE;
    float k_speed = DEFAULT_K_SPEED;
    float motor_gain = DEFAULT_MOTOR_GAIN;
    float deadband = DEFAULT_DEADBAND;
    float punch = DEFAULT_PUNCH;
    float punch_ticks = DEFAULT_PUNCH_TICKS;
    float max_power = DEFAULT_MAX_POWER;
    float trim = DEFAULT_TRIM;
    float filter = DEFAULT_FILTER;
    float max_angle = DEFAULT_MAX_ANGLE;
    float overpower_ticks = DEFAULT_OVERPOWER_TICKS;
};

enum class State : uint8_t {
    CALIBRATING,  // lying still while the gyro's offsets are measured
    READY,        // disarmed, motors off
    ARMED,        // waiting to be stood up
    LEVELING,     // stood up and held still while it measures upright, motors off
    TAKEOFF,      // full power forward to pop up onto its wheels
    BALANCING,
    FALLEN,       // stopped by a fault; the alarm sounds until it's acknowledged
};

enum class Fault : uint8_t { NONE, TILT, OVERPOWER, SENSOR };

constexpr std::string_view name(State state) {
    switch (state) {
        case State::CALIBRATING: return "CALIBRATING";
        case State::READY: return "READY";
        case State::ARMED: return "ARMED";
        case State::LEVELING: return "LEVELING";
        case State::TAKEOFF: return "TAKEOFF";
        case State::BALANCING: return "BALANCING";
        case State::FALLEN: return "FALLEN";
    }
    return "?";
}

constexpr std::string_view name(Fault fault) {
    switch (fault) {
        case Fault::NONE: return "NONE";
        case Fault::TILT: return "TILT";
        case Fault::OVERPOWER: return "OVERPOWER";
        case Fault::SENSOR: return "SENSOR";
    }
    return "?";
}

// From the buttons, the console and the remote.
enum class Request : uint8_t {
    NONE,
    ARM,          // also acknowledges a fault
    DISARM,       // also acknowledges a fault
    TOGGLE_ARM,   // button A: arms or disarms, or acknowledges a fault
    TAKEOFF,      // arms and takes off, when lying down
};

// Wheel power, -127..127 each, positive driving forward.
struct Motors {
    int left = 0;
    int right = 0;
};

struct DebugTerms {
    float pitch_deg = 0;
    float pitch_dps = 0;
    float angle_i = 0;
    float rate_i = 0;
    float speed = 0;
    float distance = 0;
    float power = 0;
    float yaw_deg = 0;
};

class Robot {
public:
    // Use flash-saved IMU calibration: skip startup & post-fall CALIBRATING waits.
    void use_saved_calibration();
    void request(Request request);
    // The remote's stick, -1..1 each way; 0, 0 holds position and heading.
    void steer(float forward, float turn);
    // Stops the robot if it's moving: the IMU stopped answering.
    void sensor_failed();
    // One tick of the control loop.
    Motors step(const Imu& imu, const Gains& gains);

    State state() const { return state_; }
    Fault fault() const { return fault_; }
    bool armed() const;
    bool motors_live() const { return state_ == State::TAKEOFF || state_ == State::BALANCING; }
    float pitch_deg() const { return pitch_; }
    float pitch_dps() const { return pitch_dps_; }
    float yaw_dps() const { return average_yaw_dps_; }
    Motors motors() const { return motors_; }
    DebugTerms debug_terms() const {
        return {
            .pitch_deg = pitch_,
            .pitch_dps = pitch_dps_,
            .angle_i = angle_i_,
            .rate_i = rate_i_,
            .speed = speed_,
            .distance = distance_,
            .power = power_,
            .yaw_deg = yaw_,
        };
    }

private:
    struct Wheel {
        int direction = 0;  // of the last nonzero power: 1, -1, or 0 after a stop
        int ticks = 0;      // in that direction
    };

    void enter(State state);
    void fall(Fault fault);
    void estimate(const Imu& imu, const Gains& gains);
    void calibrate_gyro(const Imu& imu);
    void level(const Imu& imu);
    void reset_motion();
    void drive(const Gains& gains);
    static int shape(float power, Wheel& wheel, const Gains& gains);
    static bool upright(float up_g);

    State state_ = State::CALIBRATING;
    Fault fault_ = Fault::NONE;
    int ticks_ = 0;  // in this state
    bool arm_when_calibrated_ = false;
    bool external_calibration_ = false;

    // Calibration.
    float pitch_dps_offset_ = 0;
    float yaw_dps_offset_ = 0;
    float forward_g_offset_ = 0;
    float pitch_dps_sum_ = 0;
    float yaw_dps_sum_ = 0;
    float forward_g_sum_ = 0;

    // Estimate.
    float pitch_dps_ = 0;
    float pitch_ = 0;  // degrees, forward positive
    float yaw_ = 0;    // degrees turned since balancing started
    float average_yaw_dps_ = 0;
    float average_up_g_ = 0;

    // Steering.
    float move_rate_ = 0;
    float turn_step_ = 0;  // degrees of heading per tick

    // Controller.
    float move_target_ = 0;
    float turn_target_ = 0;
    float speed_ = 0;
    float distance_ = 0;
    float angle_i_ = 0;
    float rate_i_ = 0;
    float power_ = 0;
    float power_left_ = 0;
    float power_right_ = 0;
    int overpower_ticks_ = 0;
    Wheel left_;
    Wheel right_;
    Motors motors_;
};

}  // namespace balance
