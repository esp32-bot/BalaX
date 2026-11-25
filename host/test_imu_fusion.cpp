#include <cmath>

#include "balance.h"
#include "check.h"
#include "imu_fusion.h"

namespace {

using imu_fusion::ImuFusion6Dof;

}  // namespace

TEST(imu_fusion_upright_and_flat_on_back) {
    ImuFusion6Dof f(100.0f);
    // Standing upright on wheels: M5StickC Plus mounted with +Y down => accel_y = -1g
    f.update_m5stick(0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.01f);
    auto e = f.euler_deg();
    CHECK(std::abs(e.pitch) < 0.5f);
    CHECK(std::abs(e.roll) < 0.5f);

    // Lying flat on back (LCD facing ceiling => accel_z = +1g)
    ImuFusion6Dof flat(100.0f);
    flat.update_m5stick(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.01f);
    e = flat.euler_deg();
    CHECK(std::abs(e.pitch - 90.0f) < 0.5f);
    CHECK(std::abs(e.roll) < 0.5f);
}

TEST(imu_fusion_spins_3dof_while_lying_flat_without_gimbal_lock) {
    ImuFusion6Dof f(100.0f);
    // Start lying flat on back (LCD facing ceiling => accel_z = +1g)
    f.update_m5stick(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.01f);
    f.reset_heading();
    const float yaw0 = f.euler_deg().yaw;
    CHECK(std::abs(yaw0) < 0.5f);

    // Spin 90 deg CCW flat on desk: world-up is body +Z, so gyro_z = +90 dps for 1.0 s
    for (int i = 0; i < 100; ++i) {
        f.update_m5stick(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 90.0f, 0.01f);
    }
    const auto e = f.euler_deg();
    CHECK(std::abs(e.pitch - 90.0f) < 1.5f);
    CHECK(std::abs(e.roll) < 1.5f);
    CHECK(std::abs(e.yaw - 90.0f) < 2.0f);
}

TEST(imu_fusion_rejects_linear_acceleration_tilt_corruption) {
    ImuFusion6Dof f(100.0f);
    for (int i = 0; i < 50; ++i) {
        f.update_m5stick(0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.01f);
    }
    // Strong lateral linear acceleration (+0.8g along X) with zero rotation for 0.3s
    for (int i = 0; i < 30; ++i) {
        f.update_m5stick(0.8f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.01f);
    }
    const auto e = f.euler_deg();
    // Adaptive gating + inertial low-pass should keep false tilt well under 5 deg
    CHECK(std::abs(e.roll) < 5.0f);
    CHECK(std::abs(e.pitch) < 2.0f);
}

TEST(imu_fusion_rest_detection_adapts_residual_thermal_bias) {
    ImuFusion6Dof f(100.0f);
    // Simulate a 0.8 deg/s residual thermal gyro drift on yaw while sitting still
    for (int i = 0; i < 2000; ++i) {
        f.update_m5stick(0.0f, -1.0f, 0.0f, 0.0f, 0.8f, 0.0f, 0.01f);
    }
    CHECK(f.is_resting());
    const float yaw_before = f.euler_deg().yaw;
    for (int i = 0; i < 500; ++i) {
        f.update_m5stick(0.0f, -1.0f, 0.0f, 0.0f, 0.8f, 0.0f, 0.01f);
    }
    const float yaw_after = f.euler_deg().yaw;
    // After rest-phase bias convergence, 5 seconds of stillness should drift < 0.2 deg
    CHECK(std::abs(yaw_after - yaw_before) < 0.2f);
}

TEST(imu_fusion_stationarity_breaks_before_rest_hysteresis) {
    ImuFusion6Dof f(100.0f);
    CHECK(!f.is_stationary());
    for (int i = 0; i < 1100; ++i) {
        f.update_m5stick(0, -1, 0, 0, 0, 0, 0.01f);
    }
    CHECK(f.is_stationary());
    f.update_m5stick(0, -1, 0, 0, 5, 0, 0.01f);
    CHECK(f.is_resting());
    CHECK(!f.is_stationary());
    f.update_m5stick(0.12f, -1, 0, 0, 0, 0, 0.01f);
    CHECK(!f.is_stationary());
}

TEST(balance_skips_boot_and_fall_calibration_when_saved_in_flash) {
    balance::Robot robot;
    CHECK(robot.state() == balance::State::CALIBRATING);
    robot.use_saved_calibration();
    CHECK(robot.state() == balance::State::READY);
    robot.request(balance::Request::ARM);
    CHECK(robot.state() == balance::State::ARMED);
}
