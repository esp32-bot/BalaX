#include "robot.h"

#include <algorithm>

#include "board.h"
#include "config.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "gain_settings.h"
#include "sounds.h"

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
    if (board.imu.read(motion_) == ESP_OK) {
        failed_reads_ = 0;
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
    const Telemetry telemetry = {
        .state = robot_.state(),
        .fault = robot_.fault(),
        .armed = robot_.armed(),
        .motors_live = robot_.motors_live(),
        .base_present = board.motors.present(),
        .pitch_deg = robot_.pitch_deg(),
        .yaw_dps = robot_.yaw_dps(),
        .motors = motors,
        .imu_c = motion_.temperature_c,
        .max_tick_us = max_tick_us_,
        .overruns = overruns_,
    };
    taskENTER_CRITICAL(&lock);
    shared = telemetry;
    taskEXIT_CRITICAL(&lock);
}

void task(void*) {
    Loop loop;
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        loop.tick();
        if (xTaskDelayUntil(&wake, TICK) == pdFALSE) {
            loop.overran();
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

}  // namespace robot
