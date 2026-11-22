#pragma once

// The balancing gains as runtime settings, to tune on the running robot: `get`, `set k_angle
// 70`, `save`. Their defaults are in balance_config.h.

#include "balance.h"

// The gains as they're set now; safe from any task.
balance::Gains current_gains();
