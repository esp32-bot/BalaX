#pragma once

// JoyX remotes, over ESP-NOW (remote_protocol.h has the messages). A remote finds the robot by
// its access point's name and sends to it; the robot pairs with the first remote it hears and
// answers each of its messages. Another remote can take over once the paired one has been
// quiet for REMOTE_TAKEOVER_US.

#include <cstdint>

#include "esp_err.h"

namespace remote {

// After wifi::start.
esp_err_t start();

struct Link {
    bool paired = false;
    bool live = false;  // heard from within REMOTE_LIVE_US
    uint8_t mac[6] = {};
    float forward = 0;  // the stick, as last heard
    float turn = 0;
    uint32_t messages = 0;        // since startup
    uint32_t failed_replies = 0;  // not sent, or not acknowledged by the remote
};
Link link();

}  // namespace remote
