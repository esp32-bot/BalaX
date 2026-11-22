#pragma once

// The ESP-NOW messages between the robot and a JoyX remote (an M5StickC Plus with a JoyC
// joystick hat): short text in a fixed frame, as the Arduino firmware had them, so existing
// remotes keep working. Plain C++, so laptop tests (host/) can run it.
//
// Remote to robot:
//   "c+XX+YY"           stick position, XX and YY 00..99 with 50 in the middle
//   "ARM", "DISARM", "REQUEST_TAKEOFF"
// Robot to remote, for each message:
//   "ARMED", "DISARMED", "TAKING_OFF"   for the commands
//   "c+XX+YY" + yaw + pitch + left + right + core loads, for a stick position: XX and YY
//   echoed back, then signed numbers of fixed width (see format_reply). Every
//   STATUS_EVERY messages it starts "s+A+BB" instead of "c": armed (0/1) and battery %.

#include <cstddef>
#include <cstdint>
#include <string_view>

namespace remote {

inline constexpr size_t TEXT_BYTES = 64;
inline constexpr int STATUS_EVERY = 100;

// What goes over the air, both ways.
struct Frame {
    uint8_t index;  // counts the remote's messages; replies carry the index they answer
    char text[TEXT_BYTES];
};
static_assert(sizeof(Frame) == 1 + TEXT_BYTES, "same layout as the remote's");

enum class Kind : uint8_t { INVALID, STICK, ARM, DISARM, TAKEOFF };

struct Message {
    Kind kind = Kind::INVALID;
    uint8_t x = 0;  // for STICK: 0..99, 50 in the middle
    uint8_t y = 0;
};

Message parse(std::string_view text);

// The stick as -1..1 each way, in steps of 0.1, with about the middle tenth as 0.
struct Steer {
    float forward;
    float turn;
};
Steer steer(const Message& stick);

// What a stick reply reports.
struct Telemetry {
    bool armed;
    int battery_pct;  // -1 when unknown
    float yaw_dps;
    float pitch_deg;
    int left_power;  // motor power, -127..127
    int right_power;
    int core0_load;  // %
    int core1_load;
};

// Writes the reply to `message`, the remote's message number `index`; returns its length,
// 0 for no reply.
size_t format_reply(const Message& message, uint8_t index, const Telemetry& telemetry, char* out, size_t size);

}  // namespace remote
