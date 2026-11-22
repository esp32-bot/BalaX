#pragma once

#include "balac/motors.h"
#include "esp_err.h"
#include "m5stickc_plus/axp192.h"
#include "m5stickc_plus/button.h"
#include "m5stickc_plus/display.h"
#include "m5stickc_plus/led.h"
#include "m5stickc_plus/mpu6886.h"
#include "m5stickc_plus/speaker.h"

// The robot's hardware: the M5StickC Plus, and the BalaC base on its hat connector.
struct Board {
    m5::Axp192 pmic;
    m5::Mpu6886 imu;
    m5::Display display;
    m5::Speaker speaker;
    m5::Led led;
    m5::Button button_a;  // the big one under the display
    balac::Motors motors;
};

extern Board board;

// Powers up and initializes everything on the board. The base may be switched off or
// missing; the motors then find it once it's there.
esp_err_t init_board();
