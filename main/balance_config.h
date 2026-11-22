#pragma once

// Numbers behind the balancing (balance.h): timing, limits, filters and the default gains.
// The gains can be changed on the running robot (`get`/`set`, app_settings.cpp); these are
// their defaults. Plain C++, so laptop builds can use it.

namespace balance {

// The control loop runs every TICK_S.
inline constexpr float TICK_S = 0.01f;
inline constexpr int TICKS_PER_S = 100;

// Calibration, in ticks: wait for the robot to be still, then average the IMU.
inline constexpr int GYRO_SETTLE_TICKS = 150;  // at startup, lying still
inline constexpr int GYRO_SAMPLE_TICKS = 100;
inline constexpr int LEVEL_SETTLE_TICKS = 80;  // once stood up, before balancing
inline constexpr int LEVEL_SAMPLE_TICKS = 100;

// Standing up: gravity mostly along the robot's vertical axis.
inline constexpr float UPRIGHT_G = 0.9f;
// Taking off: both wheels at full power to pop the robot up, giving up after a second.
inline constexpr int TAKEOFF_POWER = 127;
inline constexpr int TAKEOFF_TICKS = TICKS_PER_S;

// Tilt estimate: a complementary filter of the gyro and the accelerometer.
inline constexpr float AVERAGE_WEIGHT = 0.1f;  // of each new reading in the running averages
inline constexpr float DEG_PER_G = 57.3f;      // small tilts: degrees ≈ 57.3 × gravity along the forward axis

// Steering: full stick forward moves at MAX_MOVE_RATE, full stick sideways turns at
// TURN_DPS. Moving, the turn slows with the wheel power.
inline constexpr float MAX_MOVE_RATE = 2.0f;
inline constexpr float MOVE_STEP = 0.2f * TICK_S;
inline constexpr float TURN_DPS = 69.0f;
inline constexpr float MOVING_RATE = 0.1f;          // above this the robot counts as moving
inline constexpr float TURN_POWER_SCALE = 10.0f;    // wheel power for a full-speed turn while moving
inline constexpr float OVERPOWER = 1000.0f;         // controller output that counts as over the limit
inline constexpr int MAX_PUNCH_TICKS = 100;         // the punch counter stops here

// Default gains, as tuned on the Arduino firmware.
inline constexpr float DEFAULT_K_ANGLE = 65.0f;        // power per degree of tilt
inline constexpr float DEFAULT_K_RATE = 2.0f;          // power per °/s of tilting
inline constexpr float DEFAULT_K_ANGLE_I = 2400.0f;    // integral of the tilt
inline constexpr float DEFAULT_K_RATE_I = 10.0f;       // integral of the tilt rate
inline constexpr float DEFAULT_K_YAW = 4.0f;           // power per degree off the heading
inline constexpr float DEFAULT_K_DISTANCE = 300.0f;    // holding position
inline constexpr float DEFAULT_K_SPEED = 9.0f;         // damping wheel speed
inline constexpr float DEFAULT_MOTOR_GAIN = 0.5f;      // controller power to motor power
inline constexpr float DEFAULT_DEADBAND = 5.0f;        // least motor power that turns a wheel
inline constexpr float DEFAULT_PUNCH = 2.0f;           // least power on a change of direction
inline constexpr float DEFAULT_PUNCH_TICKS = 1.0f;     // how long that lasts
inline constexpr float DEFAULT_MAX_POWER = 126.0f;
inline constexpr float DEFAULT_TRIM = 0.0f;            // forward/back balance, in % of a full move
inline constexpr float DEFAULT_FILTER = 0.1f;          // how fast the accelerometer corrects the gyro
inline constexpr float DEFAULT_MAX_ANGLE = 40.0f;      // tilt that counts as fallen
inline constexpr float DEFAULT_OVERPOWER_TICKS = 80.0f;  // over the power limit this long counts as stuck

}  // namespace balance
