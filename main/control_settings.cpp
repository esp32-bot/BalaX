#include "control_settings.h"

#include <iterator>

#include "settings.h"

namespace control_settings {

namespace {

using control::Config;

struct ControlSetting {
    settings::Setting<float> setting;
    float Config::*field;
};

// Keys <= 15 chars for NVS compatibility; defaults match control::Config (all 0 = Classic 2020).
ControlSetting SETTINGS[] = {
    {{"ctrl_est", 0.0f, 0.0f, 1.0f, "Estimator module: 0=classic_cf, 1=vqf_6dof"}, &Config::est},
    {{"ctrl_law", 0.0f, 0.0f, 1.0f, "Balance law: 0=classic_pid, 1=lqr_eso"}, &Config::ctrl},
    {{"ctrl_perch", 0.0f, 0.0f, 1.0f, "Perch gate: 0=off, 1=lyapunov u=0 hold"}, &Config::perch},
    {{"ctrl_shaper", 0.0f, 0.0f, 1.0f, "Motor shaper: 0=classic_punch, 1=smooth_stiction"}, &Config::shaper},
    {{"ctrl_takeoff", 0.0f, 0.0f, 1.0f, "Takeoff module: 0=classic_bang, 1=energy_swingup"}, &Config::takeoff},

    {{"lqr_theta", 15.5f, 1.0f, 120.0f, "LQR pitch gain (motor/deg)"}, &Config::lqr_theta},
    {{"lqr_omega", 1.15f, 0.0f, 20.0f, "LQR pitch-rate gain (motor/(deg/s))"}, &Config::lqr_omega},
    {{"lqr_vel", 600.0f, 0.0f, 1500.0f, "LQR wheel-velocity + back-EMF gain"}, &Config::lqr_vel},
    {{"lqr_pos", 115.0f, 0.0f, 500.0f, "LQR position-hold gain"}, &Config::lqr_pos},
    {{"lqr_lag", 0.25f, 0.0f, 0.8f, "LQR 1-tick motor lag compensation"}, &Config::lqr_lag},

    {{"vel_drag", 24.0f, 0.1f, 60.0f, "Velocity observer viscous/back-EMF drag (1/s)"}, &Config::vel_drag},
    {{"vel_gain", 0.065f, 0.002f, 0.5f, "Velocity observer drive gain (m/s^2/u)"}, &Config::vel_gain},
    {{"eso_rate", 0.35f, 0.0f, 1.0f, "Online CG equilibrium adaptation rate"}, &Config::eso_rate},
    {{"eq_limit", 8.0f, 0.0f, 15.0f, "Max learned CG/slope offset (deg)"}, &Config::eq_limit},

    {{"perch_deg", 1.10f, 0.15f, 4.0f, "Lyapunov perch entry half-angle (deg)"}, &Config::perch_deg},
    {{"perch_dps", 8.5f, 1.0f, 30.0f, "Lyapunov perch entry pitch rate (deg/s)"}, &Config::perch_dps},
    {{"perch_vel", 0.07f, 0.01f, 1.0f, "Lyapunov perch entry velocity (m/s)"}, &Config::perch_vel},
    {{"perch_ticks", 10.0f, 1.0f, 100.0f, "Ticks in cone before cutting motors to 0"}, &Config::perch_ticks},
    {{"perch_exit", 1.65f, 1.2f, 3.0f, "Lyapunov perch exit hysteresis multiplier"}, &Config::perch_exit},

    {{"coast_u", 1.1f, 0.25f, 10.0f, "Smooth shaper zero-torque inner coast band"}, &Config::coast_u},
    {{"swing_wn", 11.0f, 5.0f, 25.0f, "Swing-up natural frequency (rad/s)"}, &Config::swing_wn},
    {{"swing_power", 115.0f, 30.0f, 127.0f, "Energy swing-up peak torque"}, &Config::swing_power},
};
static_assert(std::size(SETTINGS) * sizeof(float) == sizeof(Config), "a setting for every control::Config field");

}  // namespace

control::Config current() {
    Config cfg;
    for (const ControlSetting& item : SETTINGS) {
        cfg.*item.field = item.setting.get();
    }
    return cfg;
}

void apply_preset(bool sota) {
    const float v = sota ? 1.0f : 0.0f;
    for (int i = 0; i < 5; ++i) {
        SETTINGS[i].setting.set(v);
    }
}

bool set_stage(std::string_view stage, int mode) {
    const float v = mode ? 1.0f : 0.0f;
    if (stage == "est" || stage == "estimator") { SETTINGS[0].setting.set(v); return true; }
    if (stage == "ctrl" || stage == "law" || stage == "controller") { SETTINGS[1].setting.set(v); return true; }
    if (stage == "perch") { SETTINGS[2].setting.set(v); return true; }
    if (stage == "shaper") { SETTINGS[3].setting.set(v); return true; }
    if (stage == "takeoff" || stage == "standup") { SETTINGS[4].setting.set(v); return true; }
    return false;
}

}  // namespace control_settings
