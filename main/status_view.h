#pragma once

// Draws the status screen; status_layout.h says where everything goes. Plain C++, so laptop
// builds can render it (tools/bx preview).

#include "gfx/canvas.h"
#include "status_values.h"

namespace status {

// Draws the whole screen. The canvas is the screen's size.
void draw(gfx::Canvas& canvas, const Values& values);

}  // namespace status
