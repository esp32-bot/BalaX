#pragma once

// Hardware-free nonlinear 2D inverted-pendulum-on-wheels simulator for BalaX.
// Identified directly against real M5Stack BalaC N20 gearmotor + L9110S physics:
//   - Nonlinear gravity omega_n^2 * sin(theta - true_eq) (wn=11.0 rad/s, l_eff ~ 8.1 cm)
//   - High back-EMF N20 300-RPM gearmotor (wd=30 1/s, tau_m=33 ms, vmax=0.37 m/s)
//   - 1-tick (10 ms) I2C motor actuation latency + L9110S/N20 deadzone + +-127 saturation
//   - Surface tire contact-patch static friction cone (hard floor 0.25 deg vs soft bed 1.5 deg)
//   - Realistic MPU6886 sensor noise, thermal gyro bias drift, and wheel linear acceleration on accel

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "balance.h"
#include "imu_fusion.h"

namespace sim {

struct Surface {
    float omega_n = 11.0f;         // rad/s inverted pendulum natural frequency (l_eff ~ 8.1 cm)
    float motor_accel = 0.09f;     // m/s^2 per unit effective motor command (nominal N20 plant)
    float couple = 14.0f;          // rad/s^2 body pitch accel per m/s^2 wheel accel
    float stator_torque = 0.04f;   // rad/s^2 direct N20 stator reaction torque per unit u_eff
    float wheel_drag = 30.0f;      // 1/s N20 gearmotor back-EMF + rolling viscous damping (tau_m ~ 33 ms)
    float body_drag = 0.35f;       // 1/s air/bearing pitch damping
    float motor_deadband = 4.5f;   // L9110S + gearbox deadband (motor units)
    float patch_deg = 0.25f;       // tire contact-patch passive restoring half-angle (0.25 floor, 1.5 bed)
    float patch_vel_max = 0.04f;   // max wheel speed (m/s) for static contact-patch stick
    float true_eq_deg = 0.0f;      // true CG / slope equilibrium angle (deg)
    float gyro_bias_dps = 0.0f;    // uncalibrated residual thermal gyro drift (deg/s)
    float gyro_noise_dps = 0.22f;  // MPU6886 100 Hz gyro white noise amplitude
    float accel_noise_g = 0.012f;  // MPU6886 100 Hz accel white noise amplitude
    float imu_height_m = 0.055f;   // IMU height above wheel axle for tangential accel coupling
};

struct Result {
    bool entered_balancing = false;  // the arm/level phase reached BALANCING before the run began
    bool fell = false;
    bool stood_up = false;
    float rms_pitch_deg = 0.0f;
    float max_pitch_deg = 0.0f;
    float final_eq_deg = 0.0f;
    float max_pos_m = 0.0f;
    float final_pos_m = 0.0f;
    float energy_u2 = 0.0f;         // integral(u^2 dt)
    float duty_pct = 0.0f;          // % ticks with non-zero motor output
    float perch_pct = 0.0f;         // % ticks in passive perch (u == 0)
    float steady_energy_u2 = 0.0f;  // integral(u^2 dt) over second half of run
    int ticks_balanced = 0;
};

struct Rng {
    uint32_t s = 0x12345678u;
    float next_sym() {
        s = s * 1664525u + 1013904223u;
        const float a = static_cast<float>((s >> 8) & 0xFFFFu) / 32768.0f - 1.0f;
        s = s * 1664525u + 1013904223u;
        const float b = static_cast<float>((s >> 8) & 0xFFFFu) / 32768.0f - 1.0f;
        return 0.5f * (a + b);
    }
};

class Plant {
public:
    Surface surf{};
    float theta = 0.0f;   // rad (+ = forward)
    float omega = 0.0f;   // rad/s
    float vel = 0.0f;     // m/s
    float pos = 0.0f;     // m
    float last_a_w = 0.0f;
    float last_alpha = 0.0f;
    int delayed_motor = 0;
    Rng rng{};

    void step(int motor_cmd, float dt_s, float ext_torque_rad_s2 = 0.0f) {
        const int applied = std::clamp(delayed_motor, -127, 127);
        delayed_motor = std::clamp(motor_cmd, -127, 127);

        constexpr int SUBS = 10;
        const float h = dt_s / static_cast<float>(SUBS);
        const float eq_rad = surf.true_eq_deg * control::DEG_TO_RAD;
        const float patch_rad = surf.patch_deg * control::DEG_TO_RAD;

        const float abs_m = std::abs(static_cast<float>(applied));
        const float u_eff = abs_m <= surf.motor_deadband
                                ? 0.0f
                                : (applied > 0 ? 1.0f : -1.0f) * (abs_m - surf.motor_deadband);

        for (int i = 0; i < SUBS; ++i) {
            const float rel = theta - eq_rad;
            const bool stuck_on_patch = (u_eff == 0.0f) && (std::abs(vel) < surf.patch_vel_max) &&
                                        (std::abs(rel) <= patch_rad) && (std::abs(omega) < 0.22f) &&
                                        (std::abs(ext_torque_rad_s2) < 0.5f);

            float a_w = 0.0f;
            float alpha = 0.0f;
            if (stuck_on_patch) {
                // Contact patch & static tire friction hold robot upright with u == 0!
                vel *= 0.85f;
                const float k_patch = surf.omega_n * surf.omega_n * 1.4f;
                alpha = -k_patch * rel - 14.0f * omega;
            } else {
                a_w = surf.motor_accel * u_eff - surf.wheel_drag * vel;
                const float grav = surf.omega_n * surf.omega_n * std::sin(rel);
                const float patch_res =
                    (patch_rad > 0.0f && std::abs(rel) < patch_rad) ? -0.75f * grav : 0.0f;
                // When lying nearly flat (>42 deg), wheel-floor contact adds chassis fulcrum kick leverage.
                const float flat_kick =
                    std::abs(theta) > 0.72f ? (1.65f * std::sin(std::abs(theta)) * u_eff) : 0.0f;
                alpha = grav + patch_res - surf.couple * a_w * std::cos(theta) - surf.stator_torque * u_eff -
                        flat_kick - surf.body_drag * omega + ext_torque_rad_s2;
            }

            // Floor hard stops when lying flat (+-85 deg)
            constexpr float MAX_TILT = 85.0f * control::DEG_TO_RAD;
            omega += alpha * h;
            theta += omega * h;
            if (theta > MAX_TILT) {
                theta = MAX_TILT;
                if (omega > 0.0f) omega = 0.0f;
            } else if (theta < -MAX_TILT) {
                theta = -MAX_TILT;
                if (omega < 0.0f) omega = 0.0f;
            }
            vel += a_w * h;
            pos += vel * h;
            last_a_w = a_w;
            last_alpha = alpha;
        }
    }

    balance::Imu sample_imu(imu_fusion::ImuFusion6Dof& fusion, float dt_s) {
        constexpr float G = 9.80665f;
        const float sin_t = std::sin(theta);
        const float cos_t = std::cos(theta);
        // True specific force measured by MPU6886 (+Y down, +Z forward):
        const float az_g = sin_t + (last_a_w * cos_t + surf.imu_height_m * last_alpha) / G +
                           rng.next_sym() * surf.accel_noise_g;
        const float ay_g = -cos_t + (last_a_w * sin_t - surf.imu_height_m * omega * omega) / G +
                           rng.next_sym() * surf.accel_noise_g;
        const float ax_g = rng.next_sym() * surf.accel_noise_g;
        const float gx_dps = omega * control::RAD_TO_DEG + surf.gyro_bias_dps +
                             rng.next_sym() * surf.gyro_noise_dps;
        const float gy_dps = rng.next_sym() * surf.gyro_noise_dps;
        const float gz_dps = rng.next_sym() * surf.gyro_noise_dps;

        fusion.update_m5stick(ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, dt_s);
        const auto fused = fusion.euler_deg();
        const auto bias = fusion.online_bias_dps();

        balance::Imu imu{};
        imu.pitch_dps = gx_dps;
        imu.yaw_dps = -gy_dps;
        imu.forward_g = az_g;
        imu.up_g = ay_g;
        imu.has_fused = true;
        imu.fused_pitch_deg = fused.pitch;
        imu.fused_pitch_dps = gx_dps + bias.x;
        imu.fused_yaw_dps = -gy_dps - bias.y;
        return imu;
    }
};

// Realistic balance scenario: hold upright at 0 deg during leveling, then release with an
// observable physical initial disturbance impulse (both Classic gyro & VQF see the exact same motion!).
inline Result run_balance_scenario(const Surface& surf, const control::Config& cfg, float init_push_torque,
                                   float duration_s, float push_at_s = -1.0f, float push_torque = 0.0f) {
    Plant plant{};
    plant.surf = surf;
    plant.theta = 0.0f;

    imu_fusion::ImuFusion6Dof fusion(static_cast<float>(balance::TICKS_PER_S));
    balance::Robot robot{};
    robot.use_saved_calibration();
    const balance::Gains gains{};

    // Hold upright at 0 deg during leveling, just like standing BalaX up by hand.
    robot.request(balance::Request::ARM);
    for (int i = 0; i < balance::LEVEL_SETTLE_TICKS + balance::LEVEL_SAMPLE_TICKS + 40; ++i) {
        const auto imu = plant.sample_imu(fusion, balance::TICK_S);
        robot.step(imu, gains, cfg);
        if (robot.state() == balance::State::BALANCING) break;
    }

    Result res{};
    // A controller that never arms can't fall either, so tests check this before trusting !fell.
    res.entered_balancing = robot.state() == balance::State::BALANCING;
    const int total_ticks = static_cast<int>(duration_s * balance::TICKS_PER_S);
    const int half_ticks = total_ticks / 2;
    const int push_tick = push_at_s > 0.0f ? static_cast<int>(push_at_s * balance::TICKS_PER_S) : -1;
    double sq_pitch = 0.0;

    for (int t = 0; t < total_ticks; ++t) {
        float ext = 0.0f;
        // Brief physical release nudge (60 ms) right after letting go — observable by ALL estimators!
        if (std::abs(init_push_torque) > 0.01f && t >= 4 && t < 10) {
            ext += init_push_torque;
        }
        if (push_tick >= 0 && t >= push_tick && t < push_tick + 6) {
            ext += push_torque;
        }
        const auto imu = plant.sample_imu(fusion, balance::TICK_S);
        const auto motors = robot.step(imu, gains, cfg);
        if (robot.state() == balance::State::FALLEN) {
            res.fell = true;
            break;
        }
        const int cmd = (motors.left + motors.right) / 2;
        plant.step(cmd, balance::TICK_S, ext);

        const float p_deg = plant.theta * control::RAD_TO_DEG - surf.true_eq_deg;
        sq_pitch += static_cast<double>(p_deg) * static_cast<double>(p_deg);
        res.max_pitch_deg = std::max(res.max_pitch_deg, std::abs(p_deg));
        res.max_pos_m = std::max(res.max_pos_m, std::abs(plant.pos));
        if (t >= half_ticks) {
            const float u = static_cast<float>(cmd);
            res.steady_energy_u2 += u * u * balance::TICK_S;
        }
        ++res.ticks_balanced;
    }

    const auto dbg = robot.debug_terms();
    res.rms_pitch_deg = res.ticks_balanced > 0
                            ? static_cast<float>(std::sqrt(sq_pitch / static_cast<double>(res.ticks_balanced)))
                            : 99.0f;
    res.final_eq_deg = dbg.eq_deg;
    res.energy_u2 = dbg.energy_u2;
    res.duty_pct = dbg.duty_pct;
    res.perch_pct = dbg.perch_pct;
    return res;
}

inline Result run_takeoff_scenario(const Surface& surf, const control::Config& cfg, float start_deg,
                                   float duration_s) {
    Plant plant{};
    plant.surf = surf;
    plant.theta = start_deg * control::DEG_TO_RAD;

    imu_fusion::ImuFusion6Dof fusion(static_cast<float>(balance::TICKS_PER_S));
    balance::Robot robot{};
    robot.use_saved_calibration();
    const balance::Gains gains{};

    // Let fusion settle lying flat before takeoff request.
    for (int i = 0; i < 40; ++i) {
        const auto imu = plant.sample_imu(fusion, balance::TICK_S);
        robot.step(imu, gains, cfg);
    }

    robot.request(balance::Request::TAKEOFF);

    Result res{};
    const int total_ticks = static_cast<int>(duration_s * balance::TICKS_PER_S);
    double sq_pitch = 0.0;
    for (int t = 0; t < total_ticks; ++t) {
        const auto imu = plant.sample_imu(fusion, balance::TICK_S);
        const auto motors = robot.step(imu, gains, cfg);
        const int cmd = (motors.left + motors.right) / 2;
        plant.step(cmd, balance::TICK_S, 0.0f);

        if (robot.state() == balance::State::BALANCING) {
            res.stood_up = true;
            const float p_deg = plant.theta * control::RAD_TO_DEG - surf.true_eq_deg;
            sq_pitch += static_cast<double>(p_deg) * static_cast<double>(p_deg);
            res.max_pitch_deg = std::max(res.max_pitch_deg, std::abs(p_deg));
            ++res.ticks_balanced;
        } else if (robot.state() == balance::State::FALLEN) {
            res.fell = true;
            break;
        }
        const float u = static_cast<float>(cmd);
        res.energy_u2 += u * u * balance::TICK_S;
    }

    const auto dbg = robot.debug_terms();
    res.rms_pitch_deg = res.ticks_balanced > 0
                            ? static_cast<float>(std::sqrt(sq_pitch / static_cast<double>(res.ticks_balanced)))
                            : 99.0f;
    res.duty_pct = dbg.duty_pct;
    res.perch_pct = dbg.perch_pct;
    res.final_pos_m = plant.pos;
    return res;
}

inline Result run_drive_scenario(const Surface& surf, const control::Config& cfg, float stick_fwd,
                                 float drive_start_s, float drive_end_s, float duration_s) {
    Plant plant{};
    plant.surf = surf;
    plant.theta = surf.true_eq_deg * control::DEG_TO_RAD;

    imu_fusion::ImuFusion6Dof fusion(static_cast<float>(balance::TICKS_PER_S));
    balance::Robot robot{};
    robot.use_saved_calibration();
    const balance::Gains gains{};

    robot.request(balance::Request::ARM);
    for (int i = 0; i < balance::LEVEL_SETTLE_TICKS + balance::LEVEL_SAMPLE_TICKS + 40; ++i) {
        const auto imu = plant.sample_imu(fusion, balance::TICK_S);
        robot.step(imu, gains, cfg);
        if (robot.state() == balance::State::BALANCING) break;
    }

    Result res{};
    res.entered_balancing = robot.state() == balance::State::BALANCING;
    const int total_ticks = static_cast<int>(duration_s * balance::TICKS_PER_S);
    const int start_t = static_cast<int>(drive_start_s * balance::TICKS_PER_S);
    const int end_t = static_cast<int>(drive_end_s * balance::TICKS_PER_S);
    double sq_pitch = 0.0;

    for (int t = 0; t < total_ticks; ++t) {
        const float fwd = (t >= start_t && t < end_t) ? stick_fwd : 0.0f;
        robot.steer(fwd, 0.0f);
        const auto imu = plant.sample_imu(fusion, balance::TICK_S);
        const auto motors = robot.step(imu, gains, cfg);
        if (robot.state() == balance::State::FALLEN) {
            res.fell = true;
            break;
        }
        const int cmd = (motors.left + motors.right) / 2;
        plant.step(cmd, balance::TICK_S, 0.0f);
        const float p_deg = plant.theta * control::RAD_TO_DEG - surf.true_eq_deg;
        sq_pitch += static_cast<double>(p_deg) * static_cast<double>(p_deg);
        res.max_pitch_deg = std::max(res.max_pitch_deg, std::abs(p_deg));
        res.max_pos_m = std::max(res.max_pos_m, std::abs(plant.pos));
        ++res.ticks_balanced;
    }

    const auto dbg = robot.debug_terms();
    res.rms_pitch_deg = res.ticks_balanced > 0
                            ? static_cast<float>(std::sqrt(sq_pitch / static_cast<double>(res.ticks_balanced)))
                            : 99.0f;
    res.final_eq_deg = dbg.eq_deg;
    res.final_pos_m = plant.pos;
    res.energy_u2 = dbg.energy_u2;
    res.duty_pct = dbg.duty_pct;
    res.perch_pct = dbg.perch_pct;
    return res;
}

}  // namespace sim
