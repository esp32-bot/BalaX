#pragma once

// Runtime settings for the modular 5-stage control pipeline (`get`/`set`/`ctrl`/`save`).
// Defaults live in control/modules.h (0 = Classic 2020, 1 = SOTA 2026).

#include <string_view>

#include "control/modules.h"

namespace control_settings {

// Snapshot of the active 5-stage pipeline & LQR/ESO/Perch parameters; safe from any task.
control::Config current();

// Apply `classic` (all 5 stages = 0) or `sota` (all 5 stages = 1) live in RAM.
void apply_preset(bool sota);

// Swap a single named stage ("est", "ctrl", "perch", "shaper", "takeoff") to 0 or 1.
bool set_stage(std::string_view stage, int mode);

}  // namespace control_settings
