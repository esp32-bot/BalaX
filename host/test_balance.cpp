#include <algorithm>
#include <cmath>
#include <cstdlib>

#include "balance.h"
#include "check.h"

using namespace balance;

namespace {

const Gains GAINS;
constexpr Imu LYING = {0, 0, 1, 0};  // on its back: gravity along the forward axis
constexpr Imu UPRIGHT = {0, 0, 0, 1};
constexpr int CALIBRATION_TICKS = GYRO_SETTLE_TICKS + GYRO_SAMPLE_TICKS;
constexpr int LEVEL_TICKS = LEVEL_SETTLE_TICKS + LEVEL_SAMPLE_TICKS;

void run(Robot& robot, const Imu& imu, int ticks) {
    for (int i = 0; i < ticks; ++i) {
        robot.step(imu, GAINS);
    }
}

// Calibrated, armed, stood up and leveled.
Robot balancing() {
    Robot robot;
    run(robot, LYING, CALIBRATION_TICKS);
    robot.request(Request::ARM);
    while (robot.state() == State::ARMED) {
        robot.step(UPRIGHT, GAINS);
    }
    run(robot, UPRIGHT, LEVEL_TICKS);
    return robot;
}

}  // namespace

TEST(calibrates_then_waits_disarmed) {
    Robot robot;
    CHECK(robot.state() == State::CALIBRATING);
    run(robot, LYING, CALIBRATION_TICKS - 1);
    CHECK(robot.state() == State::CALIBRATING);
    robot.step(LYING, GAINS);
    CHECK(robot.state() == State::READY);
    CHECK(!robot.armed());
}

TEST(arming_during_calibration_waits_for_it) {
    Robot robot;
    robot.request(Request::ARM);
    CHECK(robot.armed());
    run(robot, LYING, CALIBRATION_TICKS);
    CHECK(robot.state() == State::ARMED);
}

TEST(stands_up_levels_then_balances) {
    Robot robot;
    run(robot, LYING, CALIBRATION_TICKS);
    robot.request(Request::ARM);
    run(robot, LYING, 50);
    CHECK(robot.state() == State::ARMED);
    run(robot, UPRIGHT, 30);
    CHECK(robot.state() == State::LEVELING);
    run(robot, UPRIGHT, LEVEL_TICKS);
    CHECK(robot.state() == State::BALANCING);
    CHECK(robot.pitch_deg() == 0);
    const Motors still = robot.step(UPRIGHT, GAINS);
    CHECK(still.left == 0 && still.right == 0);
}

TEST(leveling_starts_over_when_let_go) {
    Robot robot = balancing();
    robot.request(Request::DISARM);
    robot.request(Request::ARM);
    run(robot, UPRIGHT, 1);
    CHECK(robot.state() == State::LEVELING);
    run(robot, LYING, 10);
    CHECK(robot.state() == State::ARMED);
}

TEST(leans_into_a_forward_tilt) {
    Robot robot = balancing();
    const Imu tipping = {20, 0, 0.1f, 1};
    const Motors motors = robot.step(tipping, GAINS);
    CHECK(motors.left > 0 && motors.right > 0);
    CHECK(motors.left == motors.right);
}

TEST(falls_past_the_max_angle_and_waits_for_acknowledgment) {
    Robot robot = balancing();
    const Imu falling = {400, 0, 0.8f, 0.5f};
    while (robot.state() == State::BALANCING) {
        robot.step(falling, GAINS);
    }
    CHECK(robot.state() == State::FALLEN);
    CHECK(robot.fault() == Fault::TILT);
    CHECK(!robot.armed());
    CHECK(robot.motors().left == 0 && robot.motors().right == 0);
    run(robot, LYING, 500);
    CHECK(robot.state() == State::FALLEN);
    robot.request(Request::TOGGLE_ARM);  // button A
    CHECK(robot.state() == State::CALIBRATING);
    CHECK(robot.fault() == Fault::NONE);
    run(robot, LYING, CALIBRATION_TICKS);
    CHECK(robot.state() == State::READY);
}

TEST(arm_acknowledges_a_fall_and_rearms) {
    Robot robot = balancing();
    robot.sensor_failed();
    CHECK(robot.state() == State::FALLEN);
    CHECK(robot.fault() == Fault::SENSOR);
    robot.request(Request::ARM);
    run(robot, LYING, CALIBRATION_TICKS);
    CHECK(robot.state() == State::ARMED);
}

TEST(stops_when_overpowered) {
    Robot robot = balancing();
    const Imu leaning = {0, 0, 0.5f, 0.9f};  // held at a lean it can't recover from
    int ticks = 0;
    while (robot.state() == State::BALANCING && ticks++ < 10 * TICKS_PER_S) {
        robot.step(leaning, GAINS);
    }
    CHECK(robot.state() == State::FALLEN);
    CHECK(robot.fault() == Fault::OVERPOWER);
}

TEST(takes_off_or_gives_up) {
    Robot robot;
    run(robot, LYING, CALIBRATION_TICKS);
    robot.request(Request::TAKEOFF);
    CHECK(robot.state() == State::TAKEOFF);
    const Motors full = robot.step(LYING, GAINS);
    CHECK(full.left == TAKEOFF_POWER && full.right == TAKEOFF_POWER);
    run(robot, LYING, TAKEOFF_TICKS);
    CHECK(robot.state() == State::READY);

    robot.request(Request::TAKEOFF);
    run(robot, UPRIGHT, 30);
    CHECK(robot.state() == State::BALANCING);
}

TEST(disarming_stops_the_motors) {
    Robot robot = balancing();
    robot.step({20, 0, 0.1f, 1}, GAINS);
    robot.request(Request::TOGGLE_ARM);
    CHECK(robot.state() == State::READY);
    CHECK(robot.motors().left == 0 && robot.motors().right == 0);
}

TEST(removes_the_gyro_offset) {
    Robot robot;
    const Imu biased = {2.5f, -1.5f, 1, 0};
    run(robot, biased, CALIBRATION_TICKS);
    run(robot, biased, 100);
    CHECK(std::abs(robot.yaw_dps()) < 1e-4f);
}

// The Arduino firmware's getGyro() and drive(), with its globals, to check the port computes
// the same wheel power.
namespace arduino {

float varAng, Y_dps, Avg_Z_dps, yawAngle, power, powerR, powerL, varSpd, varDst, varIang, I_y_dps;
float moveTarget, moveRate, spinTarget, spinStep, spinFact = 1;
int punchCountL, punchCountR, motorLdir, motorRdir, counterOverPwr, Lmotor, Rmotor;
const float clk = 0.01;
const float moveStep = 0.2 * clk;
const float cutoff = 0.1, Kang = 65, Komg = 2, KIang = 2400, KI_y_dps = 10, Kyaw = 4, Kdst = 300, Kspd = 9;
const float mechFactL = 0.5, mechFactR = 0.5;
const int punchPwr2 = 5, punchDur = 1, motorDeadband = 5, maxPwr = 126, fbBalance = 0;

float constrain(float v, float lo, float hi) { return v < lo ? lo : v > hi ? hi : v; }

void getGyro(const Imu& imu) {
    Y_dps = imu.pitch_dps;
    Avg_Z_dps = Avg_Z_dps * 0.9 + imu.yaw_dps * 0.1;
    yawAngle += Avg_Z_dps * clk;
    varAng += (Y_dps + ((imu.forward_g - 0) * 57.3 - varAng) * cutoff) * clk;
}

void UpdateRotation(int rotation) {
    float normalizedRotation = rotation / 10.0;
    spinStep = -69.0 * normalizedRotation * clk;
}

void UpdateMovement(int movement) { moveRate = movement / 5.0; }

void drive() {
    if (std::abs(moveRate) > 0.1)
        spinFact = constrain(-(powerR + powerL) / 10.0, -1.0, 1.0);
    else
        spinFact = 1.0;
    spinTarget += spinStep * spinFact;
    moveTarget += moveStep * (moveRate + (float)fbBalance / 100.0);
    varSpd += power * clk;
    varDst += Kdst * (varSpd * clk - moveTarget);
    varIang += KIang * varAng * clk;
    I_y_dps += KI_y_dps * Y_dps * clk;
    power = varIang + varDst + (Kspd * varSpd) + (Kang * varAng) + (Komg * Y_dps) + I_y_dps;
    if (std::abs(power) > 1000.0)
        counterOverPwr += 1;
    else
        counterOverPwr = 0;
    power = constrain(power, -maxPwr, maxPwr);
    float yawPower = (yawAngle - spinTarget) * Kyaw;
    powerR = power - yawPower;
    powerL = power + yawPower;
    int ipowerL = (int16_t)constrain(powerL * mechFactL, -maxPwr, maxPwr);
    if (ipowerL > 0) {
        punchCountL = motorLdir == 1 ? std::min(punchCountL + 1, 100) : 0;
        motorLdir = 1;
        Lmotor = punchCountL < punchDur ? std::max(ipowerL, punchPwr2) : std::max(ipowerL, motorDeadband);
    } else if (ipowerL < 0) {
        punchCountL = motorLdir == -1 ? std::min(punchCountL + 1, 100) : 0;
        motorLdir = -1;
        Lmotor = punchCountL < punchDur ? std::min(ipowerL, -punchPwr2) : std::min(ipowerL, -motorDeadband);
    } else {
        Lmotor = 0;
        motorLdir = 0;
    }
    int ipowerR = (int16_t)constrain(powerR * mechFactR, -maxPwr, maxPwr);
    if (ipowerR > 0) {
        punchCountR = motorRdir == 1 ? std::min(punchCountR + 1, 100) : 0;
        motorRdir = 1;
        Rmotor = punchCountR < punchDur ? std::max(ipowerR, punchPwr2) : std::max(ipowerR, motorDeadband);
    } else if (ipowerR < 0) {
        punchCountR = motorRdir == -1 ? std::min(punchCountR + 1, 100) : 0;
        motorRdir = -1;
        Rmotor = punchCountR < punchDur ? std::min(ipowerR, -punchPwr2) : std::min(ipowerR, -motorDeadband);
    } else {
        Rmotor = 0;
        motorRdir = 0;
    }
}

}  // namespace arduino

TEST(drives_like_the_arduino_firmware) {
    Robot robot = balancing();  // leveled with no offsets: zero tilt, like the reference
    int worst = 0;
    int compared = 0;
    // A wobble, a turn, then driving forward, until the open-loop controller winds up past
    // its power limit (there's no robot moving in answer).
    for (int tick = 0; tick < 3 * TICKS_PER_S; ++tick) {
        const float t = tick * TICK_S;
        const Imu imu = {30 * std::sin(7 * t), 15 * std::cos(3 * t), 0.05f * std::sin(5 * t), 1};
        const int rotation = tick > 30 ? 4 : 0;
        const int movement = tick > 60 ? 5 : 0;
        arduino::UpdateRotation(rotation);
        arduino::UpdateMovement(movement);
        arduino::getGyro(imu);
        arduino::drive();
        robot.steer(movement / 10.0f, rotation / 10.0f);
        const Motors motors = robot.step(imu, GAINS);
        if (robot.state() != State::BALANCING) {
            break;
        }
        worst = std::max({worst, std::abs(motors.left - arduino::Lmotor), std::abs(motors.right - arduino::Rmotor)});
        ++compared;
    }
    CHECK(compared > TICKS_PER_S / 2);
    CHECK(worst <= 1);  // float rounding may tip a truncation by one
}
