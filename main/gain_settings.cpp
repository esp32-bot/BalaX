#include "gain_settings.h"

#include <iterator>

#include "settings.h"

namespace {

using balance::Gains;

struct GainSetting {
    settings::Setting<float> setting;
    float Gains::*gain;
};

// Name (also the key in flash, so at most 15 characters), default, range and help.
GainSetting GAINS[] = {
    {{"k_angle", balance::DEFAULT_K_ANGLE, 0, 1000, "Power per degree of tilt"}, &Gains::k_angle},
    {{"k_rate", balance::DEFAULT_K_RATE, 0, 100, "Power per deg/s of tilting"}, &Gains::k_rate},
    {{"k_angle_i", balance::DEFAULT_K_ANGLE_I, 0, 10000, "Integral of the tilt"}, &Gains::k_angle_i},
    {{"k_rate_i", balance::DEFAULT_K_RATE_I, 0, 1000, "Integral of the tilt rate"}, &Gains::k_rate_i},
    {{"k_yaw", balance::DEFAULT_K_YAW, 0, 100, "Power per degree off the heading"}, &Gains::k_yaw},
    {{"k_distance", balance::DEFAULT_K_DISTANCE, 0, 3000, "Holding position"}, &Gains::k_distance},
    {{"k_speed", balance::DEFAULT_K_SPEED, 0, 100, "Damping wheel speed"}, &Gains::k_speed},
    {{"motor_gain", balance::DEFAULT_MOTOR_GAIN, 0, 2, "Controller power to motor power"}, &Gains::motor_gain},
    {{"deadband", balance::DEFAULT_DEADBAND, 0, 50, "Least motor power that turns a wheel"}, &Gains::deadband},
    {{"punch", balance::DEFAULT_PUNCH, 0, 127, "Least power on a change of direction"}, &Gains::punch},
    {{"punch_ticks", balance::DEFAULT_PUNCH_TICKS, 0, 100, "Ticks that punch lasts"}, &Gains::punch_ticks},
    {{"max_power", balance::DEFAULT_MAX_POWER, 0, 127, "Motor power limit"}, &Gains::max_power},
    {{"trim", balance::DEFAULT_TRIM, -100, 100, "Forward/back balance, % of a full move"}, &Gains::trim},
    {{"filter", balance::DEFAULT_FILTER, 0, 1, "How fast the accelerometer corrects the gyro"}, &Gains::filter},
    {{"max_angle", balance::DEFAULT_MAX_ANGLE, 5, 90, "Tilt in degrees that counts as fallen"}, &Gains::max_angle},
    {{"overpower_ticks", balance::DEFAULT_OVERPOWER_TICKS, 1, 1000, "Ticks over the limit that count as stuck"},
     &Gains::overpower_ticks},
};
static_assert(std::size(GAINS) * sizeof(float) == sizeof(Gains), "a setting for every gain");

}  // namespace

balance::Gains current_gains() {
    Gains gains;
    for (const GainSetting& g : GAINS) {
        gains.*g.gain = g.setting.get();
    }
    return gains;
}
