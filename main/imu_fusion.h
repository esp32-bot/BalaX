#pragma once

// 2026 SOTA 6-DOF IMU Sensor Fusion (VQF-inspired decoupled quaternion filter):
//   - Exact SO(3) exponential-map gyroscope integration (zero gimbal lock at any 3D pose)
//   - Inertial-frame 2nd-order Butterworth low-pass filtered accelerometer gravity alignment
//     (immune to rotational phase lag and linear acceleration tilt corruption)
//   - Adaptive gravity trust gating when |norm(a) - 1g| spikes during linear motion
//   - Windowed-RMS rest-state detection + online residual gyro bias adaptation (tracks thermal drift)
//
// Right-handed Robot Body Frame B:
//   +X = Left (-accel_imu.x), +Y = Up (-accel_imu.y), +Z = Forward (+accel_imu.z)
//   standing upright on wheels => proper acceleration a_b = (0, +1, 0).
// Plain C++ with zero hardware dependencies so host unit tests run it directly.

#include <algorithm>
#include <cmath>

namespace imu_fusion {

struct Vec3 {
    float x = 0;
    float y = 0;
    float z = 0;
};

struct Quat {
    float w = 1;
    float x = 0;
    float y = 0;
    float z = 0;
};

struct EulerDeg {
    float pitch = 0;  // + = leaning back (LCD facing up)
    float roll = 0;   // + = leaning right
    float yaw = 0;    // + = CCW / left looking from above
};

inline Quat quat_mul(const Quat& a, const Quat& b) {
    return {
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    };
}

inline Quat quat_normalize(const Quat& q) {
    const float n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (n < 1e-12f) {
        return {1, 0, 0, 0};
    }
    const float inv = 1.0f / n;
    return {q.w * inv, q.x * inv, q.y * inv, q.z * inv};
}

inline Vec3 quat_rotate(const Quat& q, const Vec3& v) {
    const float tx = 2.0f * (q.y * v.z - q.z * v.y);
    const float ty = 2.0f * (q.z * v.x - q.x * v.z);
    const float tz = 2.0f * (q.x * v.y - q.y * v.x);
    return {
        v.x + q.w * tx + (q.y * tz - q.z * ty),
        v.y + q.w * ty + (q.z * tx - q.x * tz),
        v.z + q.w * tz + (q.x * ty - q.y * tx),
    };
}

inline Quat quat_from_axis_angle(float ax, float ay, float az, float angle_rad) {
    const float half = 0.5f * angle_rad;
    const float s = std::sin(half);
    return {std::cos(half), ax * s, ay * s, az * s};
}

// Shortest-arc unit quaternion rotating unit vector `from` onto unit vector `to`.
inline Quat quat_shortest_arc(const Vec3& from, const Vec3& to) {
    const float dot = std::clamp(from.x * to.x + from.y * to.y + from.z * to.z, -1.0f, 1.0f);
    if (dot < -0.99999f) {
        Vec3 ortho = std::abs(from.x) < 0.9f ? Vec3{1, 0, 0} : Vec3{0, 0, 1};
        Vec3 axis = {
            from.y * ortho.z - from.z * ortho.y,
            from.z * ortho.x - from.x * ortho.z,
            from.x * ortho.y - from.y * ortho.x,
        };
        const float n = std::sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
        return n > 1e-9f ? Quat{0, axis.x / n, axis.y / n, axis.z / n} : Quat{0, 1, 0, 0};
    }
    const Vec3 c = {
        from.y * to.z - from.z * to.y,
        from.z * to.x - from.x * to.z,
        from.x * to.y - from.y * to.x,
    };
    return quat_normalize({1.0f + dot, c.x, c.y, c.z});
}

class BiquadLowPass3 {
public:
    void configure(float fc_hz, float fs_hz) {
        const float k = std::tan(3.14159265358979323846f * fc_hz / fs_hz);
        constexpr float SQRT2 = 1.4142135623730950488f;
        const float norm = 1.0f / (1.0f + SQRT2 * k + k * k);
        b0_ = k * k * norm;
        b1_ = 2.0f * b0_;
        b2_ = b0_;
        a1_ = 2.0f * (k * k - 1.0f) * norm;
        a2_ = (1.0f - SQRT2 * k + k * k) * norm;
    }

    void reset(const Vec3& v) {
        x1_ = x2_ = y1_ = y2_ = v;
        initialized_ = true;
    }

    Vec3 step(const Vec3& x) {
        if (!initialized_) {
            reset(x);
            return x;
        }
        const Vec3 y = {
            b0_ * x.x + b1_ * x1_.x + b2_ * x2_.x - a1_ * y1_.x - a2_ * y2_.x,
            b0_ * x.y + b1_ * x1_.y + b2_ * x2_.y - a1_ * y1_.y - a2_ * y2_.y,
            b0_ * x.z + b1_ * x1_.z + b2_ * x2_.z - a1_ * y1_.z - a2_ * y2_.z,
        };
        x2_ = x1_;
        x1_ = x;
        y2_ = y1_;
        y1_ = y;
        return y;
    }

private:
    float b0_ = 1, b1_ = 0, b2_ = 0, a1_ = 0, a2_ = 0;
    Vec3 x1_{}, x2_{}, y1_{}, y2_{};
    bool initialized_ = false;
};

class ImuFusion6Dof {
public:
    static constexpr float DEG_TO_RAD = 0.01745329251994329577f;
    static constexpr float RAD_TO_DEG = 57.295779513082320877f;

    explicit ImuFusion6Dof(float sample_hz = 100.0f) : fs_(sample_hz) {
        acc_lp_.configure(0.6f, fs_);
    }

    void reset() {
        q_gyr_ = {1, 0, 0, 0};
        q_corr_ = {1, 0, 0, 0};
        q_fused_ = {1, 0, 0, 0};
        bias_rad_ = {0, 0, 0};
        acc_mean_ = {0, 1, 0};
        gyro_mean_ = {0, 0, 0};
        acc_var_ = 0.0f;
        gyro_var_ = 0.0f;
        rest_time_s_ = 0;
        resting_ = false;
        seeded_ = false;
    }

    // Zeroes world yaw heading while keeping current gravity-referenced tilt/roll intact.
    void reset_heading() {
        const float yaw_rad = heading_rad();
        const Quat dq = quat_from_axis_angle(0, 1, 0, -yaw_rad);
        q_corr_ = quat_normalize(quat_mul(dq, q_corr_));
        q_fused_ = quat_normalize(quat_mul(q_corr_, q_gyr_));
    }

    // Clears online residual gyro bias (call after saving a fresh flash calibration).
    void reset_online_bias() {
        bias_rad_ = {0, 0, 0};
        rest_time_s_ = 0;
    }

    // Update from calibrated M5StickC Plus MPU6886 axes:
    //   IMU frame in BalaC: +X = Right, +Y = Down, +Z = Forward (out of LCD)
    //   Body frame B (RH):  +X = Left (-ax), +Y = Up (-ay), +Z = Forward (+az)
    void update_m5stick(float ax_g, float ay_g, float az_g, float gx_dps, float gy_dps, float gz_dps, float dt_s) {
        update_body({-ax_g, -ay_g, az_g}, {-gx_dps * DEG_TO_RAD, -gy_dps * DEG_TO_RAD, gz_dps * DEG_TO_RAD}, dt_s);
    }

    // Update directly in Right-Handed Robot Body Frame B (+X Left, +Y Up, +Z Forward).
    void update_body(const Vec3& accel_b_g, const Vec3& gyro_b_rad, float dt_s) {
        const float dt = std::clamp(dt_s, 0.0005f, 0.05f);
        const float a_norm = std::sqrt(accel_b_g.x * accel_b_g.x + accel_b_g.y * accel_b_g.y + accel_b_g.z * accel_b_g.z);

        // Seed exact initial tilt from gravity on the first valid sample (zero startup transient)
        if (!seeded_ && a_norm > 0.25f) {
            const Vec3 a_unit = {accel_b_g.x / a_norm, accel_b_g.y / a_norm, accel_b_g.z / a_norm};
            q_gyr_ = {1, 0, 0, 0};
            q_corr_ = quat_shortest_arc(a_unit, {0, 1, 0});
            q_fused_ = q_corr_;
            acc_lp_.reset(a_unit);
            acc_mean_ = accel_b_g;
            gyro_mean_ = gyro_b_rad;
            acc_var_ = 0.0f;
            gyro_var_ = 0.0f;
            seeded_ = true;
            return;
        }

        // 1. Windowed-RMS rest detection & online residual gyro bias tracking (handles thermal drift)
        const float alpha_stat = 1.0f - std::exp(-dt / 0.20f);
        const float alpha_var = 1.0f - std::exp(-dt / 0.12f);
        acc_mean_.x += (accel_b_g.x - acc_mean_.x) * alpha_stat;
        acc_mean_.y += (accel_b_g.y - acc_mean_.y) * alpha_stat;
        acc_mean_.z += (accel_b_g.z - acc_mean_.z) * alpha_stat;
        gyro_mean_.x += (gyro_b_rad.x - gyro_mean_.x) * alpha_stat;
        gyro_mean_.y += (gyro_b_rad.y - gyro_mean_.y) * alpha_stat;
        gyro_mean_.z += (gyro_b_rad.z - gyro_mean_.z) * alpha_stat;

        const float wx_corr = gyro_b_rad.x - bias_rad_.x;
        const float wy_corr = gyro_b_rad.y - bias_rad_.y;
        const float wz_corr = gyro_b_rad.z - bias_rad_.z;
        const float w_mag = std::sqrt(wx_corr * wx_corr + wy_corr * wy_corr + wz_corr * wz_corr);

        const float dax = accel_b_g.x - acc_mean_.x;
        const float day = accel_b_g.y - acc_mean_.y;
        const float daz = accel_b_g.z - acc_mean_.z;
        const float a_dev2 = dax * dax + day * day + daz * daz;
        const float a_dev = std::sqrt(a_dev2);

        const float dwx = gyro_b_rad.x - gyro_mean_.x;
        const float dwy = gyro_b_rad.y - gyro_mean_.y;
        const float dwz = gyro_b_rad.z - gyro_mean_.z;
        const float w_dev2 = dwx * dwx + dwy * dwy + dwz * dwz;

        acc_var_ += (a_dev2 - acc_var_) * alpha_var;
        gyro_var_ += (w_dev2 - gyro_var_) * alpha_var;
        const float a_rms = std::sqrt(std::max(0.0f, acc_var_));
        const float w_rms = std::sqrt(std::max(0.0f, gyro_var_));

        constexpr float REST_W_THRESH = 3.5f * DEG_TO_RAD;    // 3.5 deg/s steady rate
        constexpr float REST_WRMS_THRESH = 2.6f * DEG_TO_RAD; // 2.6 deg/s RMS jitter (100 Hz MPU6886)
        constexpr float REST_ARMS_THRESH = 0.034f;            // 0.034 g RMS jitter (tolerates 100 Hz MPU6886 noise)
        constexpr float INSTANT_W_BREAK = 10.0f * DEG_TO_RAD; // immediate exit on real rotation
        constexpr float INSTANT_A_BREAK = 0.085f;             // immediate exit on real linear impulse

        if (w_mag > INSTANT_W_BREAK || a_dev > INSTANT_A_BREAK) {
            rest_time_s_ = 0.0f;
        } else if (w_mag < REST_W_THRESH && w_rms < REST_WRMS_THRESH && a_rms < REST_ARMS_THRESH) {
            rest_time_s_ = std::min(rest_time_s_ + dt, 10.0f);
        } else {
            rest_time_s_ = std::max(0.0f, rest_time_s_ - 3.0f * dt);
        }
        resting_ = rest_time_s_ >= 0.30f;

        if (resting_) {
            // Slowly adapt residual thermal gyro bias (clamped to +-3 deg/s around flash bias)
            const float alpha_b = 1.0f - std::exp(-dt / 4.0f);
            constexpr float MAX_ONLINE_BIAS = 3.0f * DEG_TO_RAD;
            bias_rad_.x = std::clamp(bias_rad_.x + (gyro_b_rad.x - bias_rad_.x) * alpha_b, -MAX_ONLINE_BIAS, MAX_ONLINE_BIAS);
            bias_rad_.y = std::clamp(bias_rad_.y + (gyro_b_rad.y - bias_rad_.y) * alpha_b, -MAX_ONLINE_BIAS, MAX_ONLINE_BIAS);
            bias_rad_.z = std::clamp(bias_rad_.z + (gyro_b_rad.z - bias_rad_.z) * alpha_b, -MAX_ONLINE_BIAS, MAX_ONLINE_BIAS);
        }

        // 2. Exact SO(3) exponential-map gyro integration
        const Vec3 w = {gyro_b_rad.x - bias_rad_.x, gyro_b_rad.y - bias_rad_.y, gyro_b_rad.z - bias_rad_.z};
        const float w_norm = std::sqrt(w.x * w.x + w.y * w.y + w.z * w.z);
        if (!resting_ || w_norm > 0.35f * DEG_TO_RAD) {
            if (w_norm > 1e-7f) {
                const Quat dq = quat_from_axis_angle(w.x / w_norm, w.y / w_norm, w.z / w_norm, w_norm * dt);
                q_gyr_ = quat_normalize(quat_mul(q_gyr_, dq));
            }
        }

        // 3. VQF Inertial-frame low-pass accelerometer gravity alignment
        if (a_norm > 0.2f) {
            const Vec3 a_inert = quat_rotate(q_gyr_, accel_b_g);
            const Vec3 a_lp = acc_lp_.step(a_inert);
            const float lp_norm = std::sqrt(a_lp.x * a_lp.x + a_lp.y * a_lp.y + a_lp.z * a_lp.z);
            if (lp_norm > 0.2f) {
                // Gaussian trust gate: strongly reject tilt correction during linear acceleration
                const float g_err = a_norm - 1.0f;
                const float trust = std::exp(-(g_err * g_err) / (2.0f * 0.14f * 0.14f));
                const float gain = (1.0f - std::exp(-dt / (resting_ ? 0.35f : 1.1f))) * trust;

                const Vec3 g_est_w = quat_rotate(q_corr_, {a_lp.x / lp_norm, a_lp.y / lp_norm, a_lp.z / lp_norm});
                // Horizontal tilt error axis = g_est_w x (0, 1, 0) = (-g_est_w.z, 0, g_est_w.x)
                // Strictly orthogonal to world Y => NEVER corrupts world yaw!
                const float err_x = -g_est_w.z;
                const float err_z = g_est_w.x;
                const float err_sin = std::hypot(err_x, err_z);
                if (err_sin > 1e-6f && gain > 1e-6f) {
                    const float err_angle = std::atan2(err_sin, std::clamp(g_est_w.y, -1.0f, 1.0f));
                    const Quat dq_tilt = quat_from_axis_angle(err_x / err_sin, 0.0f, err_z / err_sin, err_angle * gain);
                    q_corr_ = quat_normalize(quat_mul(dq_tilt, q_corr_));
                }
            }
        }

        q_fused_ = quat_normalize(quat_mul(q_corr_, q_gyr_));
    }

    Quat quaternion() const { return q_fused_; }
    bool is_resting() const { return resting_; }
    Vec3 online_bias_dps() const { return {bias_rad_.x * RAD_TO_DEG, bias_rad_.y * RAD_TO_DEG, bias_rad_.z * RAD_TO_DEG}; }

    // Gravity-compensated world linear acceleration in m/s^2 (Right-Handed World Frame).
    Vec3 linear_accel_world_ms2(const Vec3& accel_b_g) const {
        const Vec3 a_w = quat_rotate(q_fused_, accel_b_g);
        constexpr float G_MS2 = 9.80665f;
        return {a_w.x * G_MS2, (a_w.y - 1.0f) * G_MS2, a_w.z * G_MS2};
    }

    float heading_rad() const {
        const Vec3 fwd_w = quat_rotate(q_fused_, {0, 0, 1});
        if (std::hypot(fwd_w.x, fwd_w.z) > 0.2f) {
            return std::atan2(fwd_w.x, fwd_w.z);
        }
        const Vec3 up_w = quat_rotate(q_fused_, {0, 1, 0});
        const float sign = fwd_w.y >= 0.0f ? -1.0f : 1.0f;
        return std::atan2(sign * up_w.x, sign * up_w.z);
    }

    // Human-readable pitch, roll, yaw in degrees derived from the fused quaternion.
    EulerDeg euler_deg() const {
        const Quat q_conj = {q_fused_.w, -q_fused_.x, -q_fused_.y, -q_fused_.z};
        const Vec3 up_b = quat_rotate(q_conj, {0, 1, 0});
        const float pitch = std::atan2(up_b.z, up_b.y) * RAD_TO_DEG;
        const float roll = std::atan2(up_b.x, std::hypot(up_b.y, up_b.z)) * RAD_TO_DEG;
        return {pitch, roll, heading_rad() * RAD_TO_DEG};
    }

private:
    float fs_ = 100.0f;
    Quat q_gyr_{1, 0, 0, 0};
    Quat q_corr_{1, 0, 0, 0};
    Quat q_fused_{1, 0, 0, 0};
    Vec3 bias_rad_{0, 0, 0};
    Vec3 acc_mean_{0, 1, 0};
    Vec3 gyro_mean_{0, 0, 0};
    float acc_var_ = 0.0f;
    float gyro_var_ = 0.0f;
    BiquadLowPass3 acc_lp_;
    float rest_time_s_ = 0.0f;
    bool resting_ = false;
    bool seeded_ = false;
};

}  // namespace imu_fusion
