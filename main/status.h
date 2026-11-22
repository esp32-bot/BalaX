#pragma once

// Live readings: temperatures, power, CPU load, memory, uptime, balancing, Wi-Fi and the remote.

#include "esp_err.h"
#include "status_values.h"

// Shows the readings on the display: every STATUS_PERIOD_MS, and at once when the robot's
// state or its links change.
esp_err_t start_status_screen();
// The readings as last shown; safe from any task.
status::Values latest_status();
// Reads everything afresh and prints it as `name value` lines.
void print_status();
