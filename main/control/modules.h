#pragma once

// Modular 5-stage self-balancing control pipeline for BalaX.
// Every stage can be hot-swapped live at runtime between:
//   0 = Classic 2020 M5Stack BalaC Arduino baseline
//   1 = SOTA 2026 modular controller (VQF 6-DOF + 5-State Lag-Compensated LQR/ESO +
//       Event-Triggered Lyapunov Perch + Smooth Stiction Shaper + Energy Swing-Up)
//
// Zero dynamic allocation, zero mutexes, deterministic O(1) float math for Core 1 @ 100 Hz.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string_view>

namespace control {

inline constexpr float PI_F = 3.14159265358979323846f;
inline constexpr float DEG_TO_RAD = PI_F / 180.0f;
inline constexpr float RAD_TO_DEG = 180.0f / PI_F;

enum class EstimatorMode : uint8_t {
    CLASSIC_CF = 0,
    VQF_6DOF = 1,
};

enum class ControllerMode : uint8_t {
    CLASSIC_PID = 0,
    LQR_ESO = 1,
};

enum class PerchMode : uint8_t {
    OFF = 0,
    LYAPUNOV = 1,
};

enum class ShaperMode : uint8_t {
    CLASSIC_PUNCH = 0,
    SMOOTH_STICTION = 1,
};

enum class TakeoffMode : uint8_t {
    CLASSIC_BANG = 0,
    ENERGY_SWINGUP = 1,
};

constexpr std::string_view name(EstimatorMode m) {
    return m == EstimatorMode::VQF_6DOF ? "vqf_6dof" : "classic_cf";
}
constexpr std::string_view name(ControllerMode m) {
    return m == ControllerMode::LQR_ESO ? "lqr_eso" : "classic_pid";
}
constexpr std::string_view name(PerchMode m) {
    return m == PerchMode::LYAPUNOV ? "lyapunov" : "off";
}
constexpr std::string_view name(ShaperMode m) {
    return m == ShaperMode::SMOOTH_STICTION ? "smooth_stiction" : "classic_punch";
}
constexpr std::string_view name(TakeoffMode m) {
    return m == TakeoffMode::ENERGY_SWINGUP ? "energy_swingup" : "classic_bang";
}

struct Config {
    // Module selectors (stored as float 0/1 so they share the existing NVS setting table).
    float est = 0.0f;
    float ctrl = 0.0f;
    float perch = 0.0f;
    float shaper = 0.0f;
    float takeoff = 0.0f;

    // Stage 2: Lag-Compensated 5-State Discrete-Time LQR gains (motor-command units -127..127),
    // calibrated directly against measured BalaC N20 hardware traces (10 ms actuation lag,
    // wn=11.0 rad/s, ma=0.065, couple=10.8 -> 0.70 rad/s^2/PWM, wd=24.0 1/s, plus gear-lash margin).
    // u = u_eq(v_cmd) + lqr_theta*(pitch - eq_deg) + lqr_omega*pitch_dps + lqr_vel*(v - v_cmd)
    //     + lqr_pos*x - lqr_lag*(u_eff_now - u_eq)
    float lqr_theta = 15.5f;
    float lqr_omega = 1.15f;    // kept below 20 Hz N20 gear-lash chatter threshold
    float lqr_vel = 600.0f;
    float lqr_pos = 115.0f;
    float lqr_lag = 0.25f;      // 1-tick (10 ms) I2C/STM32 actuator-delay Smith compensation

    // Exact-ZOH velocity observer + Online Equilibrium / CG Disturbance Observer (ESO).
    float vel_drag = 24.0f;     // 1/s N20 gearmotor back-EMF + viscous drag (tau_m ~ 42 ms)
    float vel_gain = 0.065f;    // m/s^2 wheel accel per unit effective motor drive
    float eso_rate = 0.35f;     // bumpless pos -> eq_deg adaptation rate
    float eq_limit = 8.0f;      // max learned CG / slope offset (deg)

    // Stage 3: Event-Triggered Lyapunov Perch (passive zero-motor-power balance cone).
    float perch_deg = 1.10f;    // entry half-angle around eq_deg (deg)
    float perch_dps = 8.5f;     // entry pitch-rate threshold (deg/s)
    float perch_vel = 0.07f;    // entry wheel-velocity threshold (m/s)
    float perch_ticks = 10.0f;  // consecutive ticks inside cone before cutting motors to 0
    float perch_exit = 1.65f;   // hysteresis multiplier on exit radius

    // Stage 4: Smooth stiction shaper.
    float coast_u = 1.1f;       // inner zero-torque coast zone (eliminates buzz)

    // Stage 5: Energy-shaping stand-up.
    float swing_wn = 11.0f;      // pendulum natural frequency sqrt(g/l_eff) (rad/s)
    float swing_power = 115.0f;  // peak stand-up torque (gentler than 127 bang)

    EstimatorMode estimator_mode() const {
        return est >= 0.5f ? EstimatorMode::VQF_6DOF : EstimatorMode::CLASSIC_CF;
    }
    ControllerMode controller_mode() const {
        return ctrl >= 0.5f ? ControllerMode::LQR_ESO : ControllerMode::CLASSIC_PID;
    }
    PerchMode perch_mode() const {
        return perch >= 0.5f ? PerchMode::LYAPUNOV : PerchMode::OFF;
    }
    ShaperMode shaper_mode() const {
        return shaper >= 0.5f ? ShaperMode::SMOOTH_STICTION : ShaperMode::CLASSIC_PUNCH;
    }
    TakeoffMode takeoff_mode() const {
        return takeoff >= 0.5f ? TakeoffMode::ENERGY_SWINGUP : TakeoffMode::CLASSIC_BANG;
    }

    static constexpr Config classic() { return Config{}; }

    static constexpr Config sota() {
        Config c{};
        c.est = 1.0f;
        c.ctrl = 1.0f;
        c.perch = 1.0f;
        c.shaper = 1.0f;
        c.takeoff = 1.0f;
        return c;
    }
};

// Lightweight runtime state for the SOTA modules (< 72 bytes RAM).
struct State {
    float vel = 0.0f;           // deadband- & drag-aware wheel velocity estimate (m/s)
    float pos = 0.0f;           // position error integral relative to stick velocity (m)
    float eq_deg = 0.0f;        // learned zero-torque CG / slope equilibrium angle (deg)
    float mean_u_eff = 0.0f;    // low-pass filtered applied wheel effort for telemetry
    float last_u = 0.0f;        // average motor command issued on previous tick
    float prev_u_eff = 0.0f;    // effective motor torque from 1 tick prior (10 ms actuator lag alignment)
    float v_cmd_filt = 0.0f;    // slew-rate-limited drive velocity setpoint (m/s)
    float lyapunov_e = 0.0f;    // latest normalized Lyapunov stability energy
    int perch_dwell = 0;        // consecutive ticks inside perch entry cone
    bool perched = false;       // true while motors are intentionally cut to exact 0
    uint32_t balance_ticks = 0; // total ticks in BALANCING since reset
    uint32_t active_ticks = 0;  // ticks with |left| + |right| > 0
    uint32_t perch_ticks = 0;   // ticks spent passively perched (u == 0)
    float energy_u2 = 0.0f;     // cumulative control effort sum((L^2 + R^2)/2 * dt)

    void reset() { *this = State{}; }

    float duty_pct() const {
        return balance_ticks > 0 ? (100.0f * static_cast<float>(active_ticks) / static_cast<float>(balance_ticks))
                                 : 0.0f;
    }
    float perch_pct() const {
        return balance_ticks > 0 ? (100.0f * static_cast<float>(perch_ticks) / static_cast<float>(balance_ticks))
                                 : 0.0f;
    }
};

// Stage 1 (VQF fallback when external 6-DOF quaternion isn't provided):
// Exact atan2 tilt with Gaussian linear-acceleration trust gating.
inline void step_vqf_fallback_estimator(float forward_g, float up_g, float forward_g_offset, float pitch_dps,
                                        float dt_s, float& pitch_deg) {
    const float a_norm = std::hypot(forward_g, up_g);
    const float raw_deg = std::atan2(forward_g, std::max(0.15f, std::abs(up_g))) * RAD_TO_DEG -
                          forward_g_offset * RAD_TO_DEG;
    const float g_err = a_norm - 1.0f;
    const float trust = std::exp(-(g_err * g_err) / (2.0f * 0.14f * 0.14f));
    const float gain = (1.0f - std::exp(-dt_s / 0.8f)) * trust;
    pitch_deg += pitch_dps * dt_s + (raw_deg - pitch_deg) * gain;
}

// Effective wheel torque beyond deadzone.
inline float effective_effort(float applied_u, float deadband) {
    const float mag = std::abs(applied_u);
    if (mag <= deadband) return 0.0f;
    return (applied_u > 0.0f ? 1.0f : -1.0f) * (mag - deadband);
}

// Stage 2: 5-State Lag-Compensated LQR + Exact-ZOH Back-EMF Velocity Observer + Bumpless Online CG ESO.
// v_cmd is target physical wheel speed in m/s (typical full-stick cruise +-0.15 m/s).
inline float step_lqr_eso(State& st, const Config& cfg, float pitch_deg, float pitch_dps, float v_cmd,
                          float deadband, float max_power, float dt_s) {
    const float u_eff_now = st.perched ? 0.0f : effective_effort(st.last_u, deadband);

    // Slew-limit v_cmd so stick transitions lean smoothly instead of delivering an instant step kick
    // that excites N20 gearbox backlash or saturates the wheels.
    constexpr float V_CMD_SLEW_MPS2 = 0.45f;
    const float max_dv = V_CMD_SLEW_MPS2 * dt_s;
    st.v_cmd_filt += std::clamp(v_cmd - st.v_cmd_filt, -max_dv, max_dv);
    const float v_ref = st.v_cmd_filt;

    // Exact zero-order-hold (ZOH) discretization of dv/dt = vel_gain * u_eff - vel_drag * v,
    // aligned to the measured 1-tick (10 ms) I2C/STM32 motor transport lag via prev_u_eff.
    const float wd = std::max(0.0f, cfg.vel_drag);
    const float vg = std::max(1e-4f, cfg.vel_gain);
    const float decay = std::exp(-wd * dt_s);
    const float b_vel = wd > 1e-3f ? (vg / wd) * (1.0f - decay) : (vg * dt_s);
    const bool holding = std::abs(v_cmd) < 0.015f && std::abs(v_ref) < 0.015f;

    if (st.perched) {
        st.vel *= 0.85f;
        st.pos *= 0.95f;
        st.prev_u_eff = 0.0f;
        // While passively perched (u == 0, still), pitch_deg IS the true physical zero-torque equilibrium!
        if (cfg.eso_rate > 0.0f && std::abs(pitch_dps) < 2.5f) {
            st.eq_deg = std::clamp(st.eq_deg + (pitch_deg - st.eq_deg) * (0.8f * dt_s), -cfg.eq_limit, cfg.eq_limit);
        }
    } else {
        st.vel = st.vel * decay + b_vel * st.prev_u_eff;
        if (holding) {
            st.pos = std::clamp(st.pos + st.vel * dt_s, -2.5f, 2.5f);
        } else {
            // Bleed stationary position anchor while actively driving so releasing the stick stops cleanly.
            st.pos *= 0.85f;
        }
    }

    // Bumpless Online Equilibrium / CG Disturbance Observer (ESO):
    // When holding position and unsaturated, transfer steady `st.pos` into `st.eq_deg` via
    // `-(lqr_pos / lqr_theta) * delta_pos`, leaving u(t) invariant while converging eq_deg -> true CG.
    const bool unsaturated = std::abs(st.last_u) < 0.85f * max_power;
    const float alpha_u = 1.0f - std::exp(-dt_s / 0.55f);
    st.mean_u_eff += (u_eff_now - st.mean_u_eff) * alpha_u;
    if (holding && unsaturated && !st.perched && cfg.eso_rate > 0.0f && cfg.lqr_theta > 1.0f) {
        const float transfer_rate = std::min(4.0f, cfg.eso_rate * 9.0f);  // 1/s bumpless transfer
        const float delta_pos = st.pos * (1.0f - std::exp(-transfer_rate * dt_s));
        const float next_eq = std::clamp(st.eq_deg - (cfg.lqr_pos / cfg.lqr_theta) * delta_pos,
                                         -cfg.eq_limit, cfg.eq_limit);
        const float actual_d_eq = next_eq - st.eq_deg;
        st.eq_deg = next_eq;
        if (cfg.lqr_pos > 0.1f) {
            st.pos += (cfg.lqr_theta / cfg.lqr_pos) * actual_d_eq;
        }
    }

    // Equilibrium back-EMF cruise voltage for steady speed v_ref (zero steady tilt while driving!).
    const float u_eq = (wd / vg) * v_ref;
    const float e_theta = pitch_deg - st.eq_deg;
    const float u = u_eq + cfg.lqr_theta * e_theta + cfg.lqr_omega * pitch_dps + cfg.lqr_vel * (st.vel - v_ref) +
                    cfg.lqr_pos * st.pos - cfg.lqr_lag * (u_eff_now - u_eq);
    st.prev_u_eff = u_eff_now;
    return u;
}

// Stage 3: Event-Triggered Lyapunov Perch Gate (returns true when motors should be cut to exact 0).
inline bool step_perch_gate(State& st, const Config& cfg, float pitch_deg, float pitch_dps, float v_cmd,
                            float turn_step, float raw_power, float max_power) {
    if (cfg.perch_mode() != PerchMode::LYAPUNOV) {
        st.perched = false;
        st.perch_dwell = 0;
        st.lyapunov_e = 0.0f;
        return false;
    }

    const float eps_deg = std::max(0.15f, cfg.perch_deg);
    const float eps_dps = std::max(1.0f, cfg.perch_dps);
    const float eps_vel = std::max(0.03f, cfg.perch_vel);
    const float e_theta = pitch_deg - st.eq_deg;

    const float nt = e_theta / eps_deg;
    const float nw = pitch_dps / eps_dps;
    const float nv = st.vel / eps_vel;
    st.lyapunov_e = nt * nt + nw * nw + nv * nv;

    const bool steering = std::abs(v_cmd) > 0.01f || std::abs(st.v_cmd_filt) > 0.01f || std::abs(turn_step) > 1e-4f;
    if (steering) {
        st.perched = false;
        st.perch_dwell = 0;
        return false;
    }

    const float exit_mult = std::max(1.2f, cfg.perch_exit);
    const float exit_e = exit_mult * exit_mult;

    if (st.perched) {
        if (st.lyapunov_e > exit_e || std::abs(raw_power) > 0.45f * max_power) {
            st.perched = false;
            st.perch_dwell = 0;
            return false;
        }
        return true;
    }

    const int need_ticks = std::max(1, static_cast<int>(std::lround(cfg.perch_ticks)));
    if (st.lyapunov_e <= 1.0f) {
        if (++st.perch_dwell >= need_ticks) {
            st.perched = true;
            return true;
        }
    } else {
        st.perch_dwell = 0;
    }
    return false;
}

// Stage 4: Smooth Stiction Shaper.
// Replaces discontinuous deadband jump with an inner zero-torque coast zone + continuous linear torque mapping.
inline int shape_smooth_stiction(float power, float deadband, float punch, float max_power, int& wheel_dir,
                                 int& wheel_ticks, float coast_u) {
    const float mag = std::abs(power);
    if (mag < std::max(0.2f, coast_u)) {
        wheel_dir = 0;
        wheel_ticks = 0;
        return 0;
    }
    const int dir = power > 0.0f ? 1 : -1;
    if (wheel_dir == dir) {
        wheel_ticks = std::min(wheel_ticks + 1, 200);
    } else {
        wheel_dir = dir;
        wheel_ticks = 0;
    }

    // Continuous mapping from [coast_u, max_power] -> [deadband, max_power] with brief breakaway boost.
    const float db = std::max(0.0f, deadband);
    const float mp = std::max(db + 1.0f, max_power);
    const float norm = std::clamp((mag - coast_u) / std::max(1.0f, mp - coast_u), 0.0f, 1.0f);
    float out = db + norm * (mp - db);
    if (wheel_ticks < 2 && mag > coast_u * 2.5f && punch > db) {
        const float boost = std::min(punch - db, 4.0f) * (1.0f - 0.5f * static_cast<float>(wheel_ticks));
        out = std::min(mp, out + boost);
    }
    return dir * static_cast<int>(std::lround(std::clamp(out, 0.0f, mp)));
}

// Stage 5: Energy-Shaping Stand-Up Controller (spong/orbital energy pumping + soft LQR catch).
// Works smoothly from either face (+90 deg or -90 deg) without slamming past vertical.
inline int step_energy_swingup(const Config& cfg, float pitch_deg, float pitch_dps, float max_power,
                               bool& ready_to_catch) {
    const float theta = pitch_deg * DEG_TO_RAD;
    const float omega = pitch_dps * DEG_TO_RAD;
    const float wn = std::max(4.0f, cfg.swing_wn);

    // Upright mechanical energy E = 0.5*omega^2 + wn^2*(cos(theta) - 1); E == 0 at upright rest (theta=0, omega=0).
    const float energy = 0.5f * omega * omega + wn * wn * (std::cos(theta) - 1.0f);

    // Catch window: close to upright with manageable angular rate -> hand off smoothly to LQR/PID.
    if (std::abs(pitch_deg) < 24.0f && std::abs(pitch_dps) < 190.0f) {
        ready_to_catch = true;
        return 0;
    }
    ready_to_catch = false;

    const float peak = std::clamp(cfg.swing_power, 40.0f, max_power);
    const float sign_tilt = pitch_deg >= 0.0f ? 1.0f : -1.0f;

    // When nearly flat (|theta| > 42 deg), kick strongly under the CoM to initiate upright rotation.
    if (std::abs(pitch_deg) > 42.0f) {
        return static_cast<int>(std::lround(sign_tilt * peak));
    }

    // Mid-swing (24..42 deg): modulate wheel torque by energy deficit so it coasts cleanly into the catch window!
    // If energy < 0 (too slow to reach upright), push under the CoM; if energy > 0 (too fast), brake gently.
    const float e_scale = wn * wn * 0.45f;
    const float pump = std::clamp(-energy / std::max(1.0f, e_scale), -0.35f, 1.0f);
    const float cmd = sign_tilt * peak * (0.35f + 0.65f * pump);
    return static_cast<int>(std::lround(std::clamp(cmd, -peak, peak)));
}

}  // namespace control
