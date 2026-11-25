#include <cmath>
#include <cstdio>

#include "check.h"
#include "pendulum_sim.h"

namespace {

void print_row(const char* scenario, const char* label, const sim::Result& r) {
    printf("  [sim] %-18s %-8s fell=%d rms=%4.2fdeg max=%5.2fdeg E=%7.0f steadyE=%6.0f duty=%5.1f%% perch=%5.1f%% eq=%+.2fdeg\n",
           scenario, label, r.fell ? 1 : 0, r.rms_pitch_deg, r.max_pitch_deg, r.energy_u2, r.steady_energy_u2,
           r.duty_pct, r.perch_pct, r.final_eq_deg);
}

TEST(sota_hard_floor_uses_less_energy_than_classic) {
    sim::Surface floor{};
    floor.patch_deg = 0.25f;
    const auto classic = sim::run_balance_scenario(floor, control::Config::classic(), 10.0f, 12.0f);
    const auto sota = sim::run_balance_scenario(floor, control::Config::sota(), 10.0f, 12.0f);
    print_row("hard_floor", "classic", classic);
    print_row("hard_floor", "sota", sota);
    CHECK(classic.entered_balancing);
    CHECK(sota.entered_balancing);
    CHECK(!classic.fell);
    CHECK(!sota.fell);
    CHECK(sota.energy_u2 < classic.energy_u2 * 0.70f);
    CHECK(sota.duty_pct < classic.duty_pct);
}

TEST(sota_soft_bed_perches_passively_at_zero_motor_power) {
    sim::Surface bed{};
    bed.patch_deg = 1.5f;
    bed.wheel_drag = 34.0f;
    const auto classic = sim::run_balance_scenario(bed, control::Config::classic(), 10.0f, 12.0f);
    const auto sota = sim::run_balance_scenario(bed, control::Config::sota(), 10.0f, 12.0f);
    print_row("soft_bed", "classic", classic);
    print_row("soft_bed", "sota", sota);
    CHECK(classic.entered_balancing);
    CHECK(sota.entered_balancing);
    CHECK(!classic.fell);
    CHECK(!sota.fell);
    CHECK(sota.perch_pct >= 65.0f);
    CHECK(sota.steady_energy_u2 < 50.0f);
    CHECK(sota.energy_u2 < classic.energy_u2 * 0.50f);
}

TEST(sota_eso_learns_cg_offset_and_slashes_steady_energy) {
    sim::Surface tilted{};
    tilted.patch_deg = 0.35f;
    tilted.true_eq_deg = 2.5f;  // e.g. camera payload or sloped surface
    const auto classic = sim::run_balance_scenario(tilted, control::Config::classic(), 4.0f, 15.0f);
    const auto sota = sim::run_balance_scenario(tilted, control::Config::sota(), 4.0f, 15.0f);
    print_row("cg_offset_+2.5deg", "classic", classic);
    print_row("cg_offset_+2.5deg", "sota", sota);
    CHECK(classic.entered_balancing);
    CHECK(sota.entered_balancing);
    CHECK(!sota.fell);
    CHECK(std::abs(sota.final_eq_deg - 2.5f) < 1.0f);
    CHECK(sota.steady_energy_u2 < classic.steady_energy_u2 * 0.65f);
}

TEST(sota_recovers_from_push_disturbance_and_reperches) {
    sim::Surface bed{};
    bed.patch_deg = 1.2f;
    bed.wheel_drag = 32.0f;
    const auto classic = sim::run_balance_scenario(bed, control::Config::classic(), 6.0f, 12.0f, 4.0f, 18.0f);
    const auto sota = sim::run_balance_scenario(bed, control::Config::sota(), 6.0f, 12.0f, 4.0f, 18.0f);
    print_row("push_impulse", "classic", classic);
    print_row("push_impulse", "sota", sota);
    CHECK(classic.entered_balancing);
    CHECK(sota.entered_balancing);
    CHECK(!classic.fell);
    CHECK(!sota.fell);
    CHECK(sota.perch_pct > 45.0f);
    CHECK(sota.energy_u2 < classic.energy_u2);
}

TEST(sota_vqf_rejects_thermal_gyro_bias_drift) {
    sim::Surface warm{};
    warm.patch_deg = 0.35f;
    warm.gyro_bias_dps = 1.5f;  // uncalibrated thermal gyro drift while warm
    const auto classic = sim::run_balance_scenario(warm, control::Config::classic(), 6.0f, 12.0f);
    const auto sota = sim::run_balance_scenario(warm, control::Config::sota(), 6.0f, 12.0f);
    print_row("gyro_drift_+1.5dps", "classic", classic);
    print_row("gyro_drift_+1.5dps", "sota", sota);
    CHECK(classic.entered_balancing);
    CHECK(sota.entered_balancing);
    CHECK(!sota.fell);
    CHECK(sota.rms_pitch_deg < 3.0f);
    CHECK(sota.energy_u2 < classic.energy_u2);
}

TEST(sota_energy_swingup_stands_up_from_both_faces_smoothly) {
    sim::Surface floor{};
    floor.patch_deg = 0.3f;
    const auto classic_fwd = sim::run_takeoff_scenario(floor, control::Config::classic(), 82.0f, 8.0f);
    const auto sota_fwd = sim::run_takeoff_scenario(floor, control::Config::sota(), 82.0f, 8.0f);
    const auto sota_back = sim::run_takeoff_scenario(floor, control::Config::sota(), -82.0f, 8.0f);
    print_row("standup_+82deg", "classic", classic_fwd);
    print_row("standup_+82deg", "sota", sota_fwd);
    print_row("standup_-82deg", "sota", sota_back);
    CHECK(sota_fwd.stood_up);
    CHECK(!sota_fwd.fell);
    CHECK(sota_back.stood_up);
    CHECK(!sota_back.fell);
    CHECK(sota_fwd.perch_pct > 50.0f);
    CHECK(sota_back.perch_pct > 50.0f);
    CHECK(classic_fwd.fell || sota_fwd.energy_u2 < classic_fwd.energy_u2);
}

TEST(modular_pipeline_stages_compose_independently) {
    sim::Surface bed{};
    bed.patch_deg = 1.4f;
    // Classic PID + Lyapunov Perch only (swapping just 1 module!)
    control::Config mix = control::Config::classic();
    mix.perch = 1.0f;
    const auto res = sim::run_balance_scenario(bed, mix, 6.0f, 10.0f);
    print_row("mix_pid+perch", "hybrid", res);
    CHECK(res.entered_balancing);
    CHECK(!res.fell);
    CHECK(res.perch_pct > 30.0f);
}

TEST(sota_hardware_measured_plant_self_perches) {
    // Hardware-measured N20/BalaC plant (0.70 rad/s^2/PWM, 10 ms lag, wd=24 1/s)
    sim::Surface hw{};
    hw.motor_accel = 0.065f;
    hw.couple = 10.8f;
    hw.wheel_drag = 24.0f;
    hw.patch_deg = 0.35f;
    const auto floor_res = sim::run_balance_scenario(hw, control::Config::sota(), 10.0f, 12.0f);
    print_row("hw_plant_floor", "sota", floor_res);
    CHECK(floor_res.entered_balancing);
    CHECK(!floor_res.fell);
    CHECK(floor_res.perch_pct >= 80.0f);
    CHECK(floor_res.steady_energy_u2 < 50.0f);

    sim::Surface hw_cg = hw;
    hw_cg.true_eq_deg = -5.5f;  // real robot CG offset observed in hardware trace
    const auto cg_res = sim::run_balance_scenario(hw_cg, control::Config::sota(), 4.0f, 18.0f);
    print_row("hw_plant_cg_-5.5", "sota", cg_res);
    CHECK(cg_res.entered_balancing);
    CHECK(!cg_res.fell);
    CHECK(std::abs(cg_res.final_eq_deg - (-5.5f)) < 1.2f);
}

TEST(sota_drives_forward_backward_and_reperches) {
    sim::Surface hw{};
    hw.motor_accel = 0.065f;
    hw.couple = 10.8f;
    hw.wheel_drag = 24.0f;
    hw.patch_deg = 0.35f;
    const auto fwd = sim::run_drive_scenario(hw, control::Config::sota(), 1.0f, 1.5f, 4.5f, 9.0f);
    const auto rev = sim::run_drive_scenario(hw, control::Config::sota(), -1.0f, 1.5f, 4.5f, 9.0f);
    print_row("drive_fwd_+1", "sota", fwd);
    print_row("drive_rev_-1", "sota", rev);
    CHECK(fwd.entered_balancing);
    CHECK(rev.entered_balancing);
    CHECK(!fwd.fell);
    CHECK(!rev.fell);
    CHECK(fwd.final_pos_m > 0.25f);
    CHECK(rev.final_pos_m < -0.25f);
    CHECK(fwd.perch_pct > 15.0f);
    CHECK(rev.perch_pct > 15.0f);
}

TEST(sota_yaw_gives_balance_priority_and_never_winds_up) {
    balance::Robot robot{};
    robot.use_saved_calibration();
    const balance::Gains gains{};
    const auto cfg = control::Config::sota();

    balance::Imu upright{};
    upright.up_g = -1.0f;
    upright.has_fused = true;
    robot.request(balance::Request::ARM);
    for (int i = 0; i < balance::LEVEL_SETTLE_TICKS + balance::LEVEL_SAMPLE_TICKS + 60; ++i) {
        robot.step(upright, gains, cfg);
        if (robot.state() == balance::State::BALANCING) break;
    }
    CHECK(robot.state() == balance::State::BALANCING);

    // Hold full right turn for 2 seconds even with yaw blocked (stuck wheels): must not wind up or exceed yaw cap.
    robot.steer(0.0f, 1.0f);
    for (int i = 0; i < 200; ++i) {
        const auto m = robot.step(upright, gains, cfg);
        CHECK(m.left > m.right);  // right turn => left wheel forward > right wheel
        CHECK(std::abs(m.left - m.right) <= static_cast<int>(gains.max_power * 0.65f));
    }

    // Sudden strong pitch disturbance while still holding full turn: balance MUST get 100% of both wheels!
    balance::Imu tilted = upright;
    tilted.fused_pitch_deg = 12.0f;
    tilted.fused_pitch_dps = 60.0f;
    const auto rescue = robot.step(tilted, gains, cfg);
    CHECK(rescue.left >= 120);
    CHECK(rescue.right >= 120);

    // Release turn stick: zero residual heading windup -> both motors return symmetric quickly.
    robot.steer(0.0f, 0.0f);
    for (int i = 0; i < 40; ++i) {
        robot.step(upright, gains, cfg);
    }
    const auto settled = robot.step(upright, gains, cfg);
    CHECK(std::abs(settled.left - settled.right) <= 4);
}

}  // namespace
