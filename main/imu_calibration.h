#pragma once

// Persistent 6-axis IMU calibration & IMU-to-CG lever-arm offset stored in NVS flash (`settings`).
// Calibrate once via the web UI (6-face accel ellipsoid + CG lever-arm + zero gyro) instead of
// re-calibrating on every reboot.

#include "esp_err.h"
#include "m5stickc_plus/mpu6886.h"

namespace imu_cal {

struct Offsets {
    bool valid = false;
    bool accel_6face = false;
    m5::Vector3 gyro_dps{0, 0, 0};
    m5::Vector3 accel_g{0, 0, 0};
    m5::Vector3 accel_scale{1, 1, 1};
    // Vector from robot CG/pivot to IMU in robot body frame (+X Left, +Y Up, +Z Forward), meters.
    m5::Vector3 lever_arm_m{0, 0, 0};
};

// True once a calibration has been saved to flash (or applied in this session).
bool is_calibrated();

// True once a 6-face ellipsoid accelerometer calibration has been saved to flash.
bool has_6face_accel();

// Thread-safe snapshot of the active offsets.
Offsets current();

// Applies gyro bias, accel bias, and per-axis accel scale to a raw IMU reading.
void apply(m5::Motion& motion);

// Updates active gyro + radial accel biases (single-pose zero) and commits to NVS flash.
esp_err_t save(const m5::Vector3& gyro_bias_dps, const m5::Vector3& accel_bias_g);

// Updates only gyro bias (preserving an existing 6-face accel calibration) and commits to NVS flash.
esp_err_t save_gyro(const m5::Vector3& gyro_bias_dps);

// Updates 6-face ellipsoid accel bias & per-axis scale factors and commits to NVS flash.
esp_err_t save_accel(const m5::Vector3& accel_bias_g, const m5::Vector3& accel_scale);

// Updates 3D IMU-to-CG lever-arm vector (meters, body frame) and commits to NVS flash.
esp_err_t save_lever_arm(const m5::Vector3& lever_arm_m);

}  // namespace imu_cal
