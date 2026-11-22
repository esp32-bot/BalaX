#pragma once

#include "esp_err.h"

// Adds the robot's console commands: status, beep, play, reboot.
esp_err_t register_app_commands();
