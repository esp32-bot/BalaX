#include "robot.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>

#include "board.h"
#include "config.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "gain_settings.h"
#include "imu_calibration.h"
#include "imu_fusion.h"
#include "sounds.h"
#include "telemetry_stream.h"

namespace robot {

namespace {

constexpr int MS_PER_S = 1000;
constexpr TickType_t TICK = pdMS_TO_TICKS(MS_PER_S / balance::TICKS_PER_S);
static_assert(TICK * portTICK_PERIOD_MS * balance::TICKS_PER_S == MS_PER_S, "the control tick is whole RTOS ticks");
constexpr int ALARM_BEEP_TICKS = ALARM_BEEP.ms * balance::TICKS_PER_S / MS_PER_S;
constexpr int SHUTDOWN_TICKS = duration_ms(SHUTDOWN_MELODY) * balance::TICKS_PER_S / MS_PER_S;

struct Stick {
    float forward = 0;
    float turn = 0;
    int64_t at_us = 0;
};

QueueHandle_t requests;
portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
// Guarded by `lock`.
Stick stick;
Telemetry shared;
m5::Motion raw_shared{};
uint32_t raw_seq = 0;

std::atomic<bool> pending_apply_saved_cal{false};
std::atomic<bool> pending_reset_fusion{false};
std::atomic<bool> pending_reset_heading{false};

// The robot's axes from the IMU's, as the Arduino firmware mapped them.
balance::Imu to_robot(const m5::Motion& motion) {
    return {
        .pitch_dps = motion.gyro_dps.x,
        .yaw_dps = -motion.gyro_dps.y,
        .forward_g = motion.accel_g.z,
        .up_g = motion.accel_g.y,
    };
}

Stick current_stick(int64_t now_us) {
    taskENTER_CRITICAL(&lock);
    const Stick latest = stick;
    taskEXIT_CRITICAL(&lock);
    return now_us - latest.at_us < STEER_TIMEOUT_US ? latest : Stick{};
}

class Loop {
public:
    Loop() {
        if (imu_cal::is_calibrated()) {
            robot_.use_saved_calibration();
        }
    }

    void tick();
    void overran() { ++overruns_; }

private:
    enum class Shutdown : uint8_t { NONE, RESTART, POWER_OFF };

    void take_requests();
    void read_imu();
    void show_state();
    void check_power_button();
    void shut_down();
    void publish(uint32_t tick_us);

    balance::Robot robot_;
    imu_fusion::ImuFusion6Dof fusion_{static_cast<float>(balance::TICKS_PER_S)};
    m5::Motion motion_{};
    int failed_reads_ = 0;
    uint32_t ticks_ = 0;
    int alarm_ticks_ = 0;
    Shutdown shutdown_ = Shutdown::NONE;
    int shutdown_ticks_ = 0;
    uint32_t window_max_us_ = 0;
    uint32_t max_tick_us_ = 0;
    uint32_t overruns_ = 0;
};

void Loop::tick() {
    const int64_t start_us = esp_timer_get_time();
    ++ticks_;
    take_requests();
    const Stick steering = current_stick(start_us);
    robot_.steer(steering.forward, steering.turn);
    read_imu();
    const balance::Motors motors = shutdown_ == Shutdown::NONE ? robot_.motors() : balance::Motors{};
    board.motors.set(motors.left, motors.right);
    // Everything after the motors, so it doesn't delay them.
    show_state();
    if (ticks_ % POWER_BUTTON_POLL_TICKS == 0) {
        check_power_button();
    }
    if (shutdown_ != Shutdown::NONE && --shutdown_ticks_ <= 0) {
        shut_down();
    }
    publish(esp_timer_get_time() - start_us);
}

void Loop::take_requests() {
    if (pending_apply_saved_cal.exchange(false, std::memory_order_acq_rel)) {
        robot_.use_saved_calibration();
    }
    if (pending_reset_fusion.exchange(false, std::memory_order_acq_rel)) {
        fusion_.reset_online_bias();
        fusion_.reset_heading();
    }
    if (pending_reset_heading.exchange(false, std::memory_order_acq_rel)) {
        fusion_.reset_heading();
    }
    balance::Request request;
    while (xQueueReceive(requests, &request, 0) == pdTRUE) {
        if (shutdown_ == Shutdown::NONE) {
            robot_.request(request);
        }
    }
    if (board.button_a.take_press() && shutdown_ == Shutdown::NONE) {
        robot_.request(balance::Request::TOGGLE_ARM);
    }
}

void Loop::read_imu() {
    m5::Motion raw{};
    if (board.imu.read(raw) == ESP_OK) {
        failed_reads_ = 0;
        taskENTER_CRITICAL(&lock);
        raw_shared = raw;
        ++raw_seq;
        taskEXIT_CRITICAL(&lock);

        motion_ = raw;
        imu_cal::apply(motion_);
        fusion_.update_m5stick(
            motion_.accel_g.x, motion_.accel_g.y, motion_.accel_g.z,
            motion_.gyro_dps.x, motion_.gyro_dps.y, motion_.gyro_dps.z,
            balance::TICK_S);
        robot_.step(to_robot(motion_), current_gains());
    } else if (++failed_reads_ >= SENSOR_FAILURE_TICKS) {
        robot_.sensor_failed();
    }
}

// The LED is lit while the robot must be held still; after a fall it flashes with the alarm.
void Loop::show_state() {
    const balance::State state = robot_.state();
    if (state == balance::State::FALLEN) {
        if (alarm_ticks_ == 0) {
            board.speaker.play(ALARM_BEEP);
        }
        board.led.set(alarm_ticks_ < ALARM_BEEP_TICKS);
        alarm_ticks_ = (alarm_ticks_ + 1) % ALARM_PERIOD_TICKS;
    } else {
        alarm_ticks_ = 0;
        board.led.set(state == balance::State::CALIBRATING || state == balance::State::LEVELING);
    }
}

// A short press restarts the robot, as on the Arduino firmware; a long one powers it off.
// Either way the motors stop first and the shutdown melody plays out.
void Loop::check_power_button() {
    const m5::Axp192::Press press = board.pmic.take_power_button();
    if (press == m5::Axp192::Press::NONE || shutdown_ != Shutdown::NONE) {
        return;
    }
    shutdown_ = press == m5::Axp192::Press::LONG ? Shutdown::POWER_OFF : Shutdown::RESTART;
    shutdown_ticks_ = SHUTDOWN_TICKS;
    robot_.request(balance::Request::DISARM);
    board.speaker.play(SHUTDOWN_MELODY);
}

void Loop::shut_down() {
    if (shutdown_ == Shutdown::POWER_OFF) {
        board.pmic.power_off();
    }
    esp_restart();  // also if powering off failed, or USB power keeps it on
}

void Loop::publish(uint32_t tick_us) {
    window_max_us_ = std::max(window_max_us_, tick_us);
    if (ticks_ % balance::TICKS_PER_S == 0) {
        max_tick_us_ = window_max_us_;
        window_max_us_ = 0;
    }
    const balance::Motors motors = robot_.motors();
    const balance::DebugTerms terms = robot_.debug_terms();
    const imu_fusion::EulerDeg fused = fusion_.euler_deg();
    const Telemetry telemetry = {
        .state = robot_.state(),
        .fault = robot_.fault(),
        .armed = robot_.armed(),
        .motors_live = robot_.motors_live(),
        .base_present = board.motors.present(),
        .pitch_deg = robot_.pitch_deg(),
        .yaw_dps = robot_.yaw_dps(),
        .fused_deg = fused,
        .imu_resting = fusion_.is_resting(),
        .motors = motors,
        .terms = terms,
        .imu_c = motion_.temperature_c,
        .max_tick_us = max_tick_us_,
        .overruns = overruns_,
    };
    // Stream calibrated 6-axis IMU + 6-DOF fused yaw at 100 Hz
    const TraceSample sample = {
        .tick = ticks_,
        .state = telemetry.state,
        .fault = telemetry.fault,
        .motor_l = static_cast<int8_t>(motors.left),
        .motor_r = static_cast<int8_t>(motors.right),
        .pitch_deg = fused.pitch,
        .pitch_dps = terms.pitch_dps,
        .angle_i = terms.angle_i,
        .rate_i = terms.rate_i,
        .speed = terms.speed,
        .distance = terms.distance,
        .power = terms.power,
        .accel_x_g = motion_.accel_g.x,
        .accel_y_g = motion_.accel_g.y,
        .accel_z_g = motion_.accel_g.z,
        .gyro_x_dps = motion_.gyro_dps.x,
        .gyro_y_dps = motion_.gyro_dps.y,
        .gyro_z_dps = motion_.gyro_dps.z,
        .yaw_deg = fused.yaw,
    };
    taskENTER_CRITICAL(&lock);
    shared = telemetry;
    taskEXIT_CRITICAL(&lock);
    // Only buffers in RAM / flash spillover when an active LAN streamer is subscribed.
    telemetry_stream::push_sample(sample);
}

void task(void*) {
    Loop loop;
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        loop.tick();
        if (xTaskDelayUntil(&wake, TICK) == pdFALSE) {
            loop.overran();
            wake = xTaskGetTickCount();
        }
    }
}

}  // namespace

esp_err_t start() {
    requests = xQueueCreate(ROBOT_REQUEST_QUEUE, sizeof(balance::Request));
    if (requests == nullptr) {
        return ESP_ERR_NO_MEM;
    }
    return xTaskCreatePinnedToCore(task, "robot", ROBOT_STACK_BYTES, nullptr, ROBOT_PRIORITY, nullptr, ROBOT_CORE) ==
                   pdPASS
               ? ESP_OK
               : ESP_ERR_NO_MEM;
}

void request(balance::Request request) {
    if (requests != nullptr) {
        xQueueSend(requests, &request, 0);
    }
}

void steer(float forward, float turn) {
    const Stick latest = {forward, turn, esp_timer_get_time()};
    taskENTER_CRITICAL(&lock);
    stick = latest;
    taskEXIT_CRITICAL(&lock);
}

Telemetry telemetry() {
    taskENTER_CRITICAL(&lock);
    const Telemetry latest = shared;
    taskEXIT_CRITICAL(&lock);
    return latest;
}

ZeroImuResult zero_imu() {
    ZeroImuResult res{};
    request(balance::Request::DISARM);
    vTaskDelay(pdMS_TO_TICKS(100));  // let motors stop completely

    constexpr int TARGET_SAMPLES = 100;  // 1.0 s at 100 Hz
    constexpr float MAX_STILL_SPREAD_DPS = 15.0f;

    double sgx = 0, sgy = 0, sgz = 0;
    double sax = 0, say = 0, saz = 0;
    float min_gx = std::numeric_limits<float>::infinity();
    float max_gx = -std::numeric_limits<float>::infinity();
    float min_gy = std::numeric_limits<float>::infinity();
    float max_gy = -std::numeric_limits<float>::infinity();
    float min_gz = std::numeric_limits<float>::infinity();
    float max_gz = -std::numeric_limits<float>::infinity();

    uint32_t last_seq = 0;
    taskENTER_CRITICAL(&lock);
    last_seq = raw_seq;
    taskEXIT_CRITICAL(&lock);

    int collected = 0;
    int wait_ticks = 0;
    while (collected < TARGET_SAMPLES && wait_ticks < 250) {
        vTaskDelay(pdMS_TO_TICKS(10));
        ++wait_ticks;
        taskENTER_CRITICAL(&lock);
        const uint32_t seq = raw_seq;
        const m5::Motion m = raw_shared;
        taskEXIT_CRITICAL(&lock);
        if (seq == last_seq) {
            continue;
        }
        last_seq = seq;
        sgx += m.gyro_dps.x;
        sgy += m.gyro_dps.y;
        sgz += m.gyro_dps.z;
        sax += m.accel_g.x;
        say += m.accel_g.y;
        saz += m.accel_g.z;
        min_gx = std::min(min_gx, m.gyro_dps.x);
        max_gx = std::max(max_gx, m.gyro_dps.x);
        min_gy = std::min(min_gy, m.gyro_dps.y);
        max_gy = std::max(max_gy, m.gyro_dps.y);
        min_gz = std::min(min_gz, m.gyro_dps.z);
        max_gz = std::max(max_gz, m.gyro_dps.z);
        ++collected;
    }

    if (collected < TARGET_SAMPLES / 2) {
        return res;
    }

    const float spread = std::max({max_gx - min_gx, max_gy - min_gy, max_gz - min_gz});
    res.max_gyro_spread_dps = spread;
    if (spread > MAX_STILL_SPREAD_DPS) {
        res.moved = true;
        return res;
    }

    const float inv_n = 1.0f / static_cast<float>(collected);
    const m5::Vector3 g_bias = {
        static_cast<float>(sgx) * inv_n,
        static_cast<float>(sgy) * inv_n,
        static_cast<float>(sgz) * inv_n,
    };

    // If a full 6-face ellipsoid accel calibration is already saved, keep it intact and only
    // refresh the 3-axis gyro bias; otherwise remove radial 1g accel norm bias as a fallback.
    if (imu_cal::has_6face_accel()) {
        if (imu_cal::save_gyro(g_bias) != ESP_OK) {
            return res;
        }
        const imu_cal::Offsets kept = imu_cal::current();
        res.preserved_6face = true;
        res.acc_bx = kept.accel_g.x;
        res.acc_by = kept.accel_g.y;
        res.acc_bz = kept.accel_g.z;
    } else {
        const m5::Vector3 a_mean = {
            static_cast<float>(sax) * inv_n,
            static_cast<float>(say) * inv_n,
            static_cast<float>(saz) * inv_n,
        };
        const float a_norm = std::sqrt(a_mean.x * a_mean.x + a_mean.y * a_mean.y + a_mean.z * a_mean.z);
        m5::Vector3 a_bias = {0, 0, 0};
        if (a_norm > 0.5f) {
            const float radial_scale = 1.0f - (1.0f / a_norm);
            a_bias = {a_mean.x * radial_scale, a_mean.y * radial_scale, a_mean.z * radial_scale};
        }
        if (imu_cal::save(g_bias, a_bias) != ESP_OK) {
            return res;
        }
        res.acc_bx = a_bias.x;
        res.acc_by = a_bias.y;
        res.acc_bz = a_bias.z;
    }

    pending_apply_saved_cal.store(true, std::memory_order_release);
    pending_reset_fusion.store(true, std::memory_order_release);
    board.speaker.play(DEFAULT_BEEP);

    res.ok = true;
    res.gyro_bx = g_bias.x;
    res.gyro_by = g_bias.y;
    res.gyro_bz = g_bias.z;
    return res;
}

void reset_heading() {
    pending_reset_heading.store(true, std::memory_order_release);
}

void reset_fusion() {
    pending_apply_saved_cal.store(true, std::memory_order_release);
    pending_reset_fusion.store(true, std::memory_order_release);
}

}  // namespace robot
