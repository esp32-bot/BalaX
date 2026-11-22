#include "imu_calibration.h"

#include <algorithm>

#include "settings.h"

namespace {

constexpr float MAX_GYRO_BIAS_DPS = 50.0f;
constexpr float MAX_ACCEL_BIAS_G = 0.5f;
constexpr float MIN_ACCEL_SCALE = 0.80f;
constexpr float MAX_ACCEL_SCALE = 1.25f;
constexpr float MAX_LEVER_ARM_M = 0.35f;

// Keys in NVS flash (<= 15 chars each).
settings::Setting<uint8_t> imu_cal_valid(
    "imu_cal", 0, 0, 1, "1 once 6-axis IMU calibration has been saved to flash");
settings::Setting<uint8_t> acc_cal6_valid(
    "acc_cal6", 0, 0, 1, "1 once 6-face ellipsoid accel calibration has been saved to flash");
settings::Setting<float> gyro_bx(
    "gyro_bx", 0.0f, -MAX_GYRO_BIAS_DPS, MAX_GYRO_BIAS_DPS, "IMU gyro X bias (deg/s)");
settings::Setting<float> gyro_by(
    "gyro_by", 0.0f, -MAX_GYRO_BIAS_DPS, MAX_GYRO_BIAS_DPS, "IMU gyro Y bias (deg/s)");
settings::Setting<float> gyro_bz(
    "gyro_bz", 0.0f, -MAX_GYRO_BIAS_DPS, MAX_GYRO_BIAS_DPS, "IMU gyro Z bias (deg/s)");
settings::Setting<float> acc_bx(
    "acc_bx", 0.0f, -MAX_ACCEL_BIAS_G, MAX_ACCEL_BIAS_G, "IMU accel X bias (g)");
settings::Setting<float> acc_by(
    "acc_by", 0.0f, -MAX_ACCEL_BIAS_G, MAX_ACCEL_BIAS_G, "IMU accel Y bias (g)");
settings::Setting<float> acc_bz(
    "acc_bz", 0.0f, -MAX_ACCEL_BIAS_G, MAX_ACCEL_BIAS_G, "IMU accel Z bias (g)");
settings::Setting<float> acc_sx(
    "acc_sx", 1.0f, MIN_ACCEL_SCALE, MAX_ACCEL_SCALE, "IMU accel X scale factor");
settings::Setting<float> acc_sy(
    "acc_sy", 1.0f, MIN_ACCEL_SCALE, MAX_ACCEL_SCALE, "IMU accel Y scale factor");
settings::Setting<float> acc_sz(
    "acc_sz", 1.0f, MIN_ACCEL_SCALE, MAX_ACCEL_SCALE, "IMU accel Z scale factor");
settings::Setting<float> imu_rx(
    "imu_rx", 0.0f, -MAX_LEVER_ARM_M, MAX_LEVER_ARM_M, "CG->IMU X lever arm (m, +Left)");
settings::Setting<float> imu_ry(
    "imu_ry", 0.0f, -MAX_LEVER_ARM_M, MAX_LEVER_ARM_M, "CG->IMU Y lever arm (m, +Up)");
settings::Setting<float> imu_rz(
    "imu_rz", 0.0f, -MAX_LEVER_ARM_M, MAX_LEVER_ARM_M, "CG->IMU Z lever arm (m, +Forward)");

float clamp_sym(float v, float limit) {
    return std::clamp(v, -limit, limit);
}

float clamp_scale(float v) {
    return std::clamp(v, MIN_ACCEL_SCALE, MAX_ACCEL_SCALE);
}

}  // namespace

namespace imu_cal {

bool is_calibrated() {
    return imu_cal_valid.get() != 0;
}

bool has_6face_accel() {
    return acc_cal6_valid.get() != 0;
}

Offsets current() {
    return {
        .valid = is_calibrated(),
        .accel_6face = has_6face_accel(),
        .gyro_dps = {gyro_bx.get(), gyro_by.get(), gyro_bz.get()},
        .accel_g = {acc_bx.get(), acc_by.get(), acc_bz.get()},
        .accel_scale = {acc_sx.get(), acc_sy.get(), acc_sz.get()},
        .lever_arm_m = {imu_rx.get(), imu_ry.get(), imu_rz.get()},
    };
}

void apply(m5::Motion& motion) {
    motion.gyro_dps.x -= gyro_bx.get();
    motion.gyro_dps.y -= gyro_by.get();
    motion.gyro_dps.z -= gyro_bz.get();
    motion.accel_g.x = (motion.accel_g.x - acc_bx.get()) * acc_sx.get();
    motion.accel_g.y = (motion.accel_g.y - acc_by.get()) * acc_sy.get();
    motion.accel_g.z = (motion.accel_g.z - acc_bz.get()) * acc_sz.get();
}

esp_err_t save(const m5::Vector3& gyro_bias_dps, const m5::Vector3& accel_bias_g) {
    gyro_bx.set(clamp_sym(gyro_bias_dps.x, MAX_GYRO_BIAS_DPS));
    gyro_by.set(clamp_sym(gyro_bias_dps.y, MAX_GYRO_BIAS_DPS));
    gyro_bz.set(clamp_sym(gyro_bias_dps.z, MAX_GYRO_BIAS_DPS));
    acc_bx.set(clamp_sym(accel_bias_g.x, MAX_ACCEL_BIAS_G));
    acc_by.set(clamp_sym(accel_bias_g.y, MAX_ACCEL_BIAS_G));
    acc_bz.set(clamp_sym(accel_bias_g.z, MAX_ACCEL_BIAS_G));
    acc_sx.set(1.0f);
    acc_sy.set(1.0f);
    acc_sz.set(1.0f);
    acc_cal6_valid.set(0);
    imu_cal_valid.set(1);
    return settings::save_all();
}

esp_err_t save_gyro(const m5::Vector3& gyro_bias_dps) {
    gyro_bx.set(clamp_sym(gyro_bias_dps.x, MAX_GYRO_BIAS_DPS));
    gyro_by.set(clamp_sym(gyro_bias_dps.y, MAX_GYRO_BIAS_DPS));
    gyro_bz.set(clamp_sym(gyro_bias_dps.z, MAX_GYRO_BIAS_DPS));
    imu_cal_valid.set(1);
    return settings::save_all();
}

esp_err_t save_accel(const m5::Vector3& accel_bias_g, const m5::Vector3& accel_scale) {
    acc_bx.set(clamp_sym(accel_bias_g.x, MAX_ACCEL_BIAS_G));
    acc_by.set(clamp_sym(accel_bias_g.y, MAX_ACCEL_BIAS_G));
    acc_bz.set(clamp_sym(accel_bias_g.z, MAX_ACCEL_BIAS_G));
    acc_sx.set(clamp_scale(accel_scale.x));
    acc_sy.set(clamp_scale(accel_scale.y));
    acc_sz.set(clamp_scale(accel_scale.z));
    acc_cal6_valid.set(1);
    imu_cal_valid.set(1);
    return settings::save_all();
}

esp_err_t save_lever_arm(const m5::Vector3& lever_arm_m) {
    imu_rx.set(clamp_sym(lever_arm_m.x, MAX_LEVER_ARM_M));
    imu_ry.set(clamp_sym(lever_arm_m.y, MAX_LEVER_ARM_M));
    imu_rz.set(clamp_sym(lever_arm_m.z, MAX_LEVER_ARM_M));
    return settings::save_all();
}

}  // namespace imu_cal
