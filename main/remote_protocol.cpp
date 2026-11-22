#include "remote_protocol.h"

#include <algorithm>
#include <cstdio>

namespace remote {

namespace {

constexpr std::string_view ARM_TEXT = "ARM";
constexpr std::string_view DISARM_TEXT = "DISARM";
constexpr std::string_view TAKEOFF_TEXT = "REQUEST_TAKEOFF";
constexpr char STICK_PREFIX = 'c';
constexpr char STATUS_PREFIX = 's';
constexpr size_t STICK_LENGTH = 7;  // "c+XX+YY"
constexpr size_t X_AT = 1;
constexpr size_t Y_AT = 4;

constexpr int STICK_MIDDLE = 50;
constexpr int STICK_STEPS = 10;  // each way

// Widths of the numbers in a reply, sign included: the remote reads them by position.
constexpr int MAX_3_DIGITS = 999;
constexpr int MAX_2_DIGITS = 99;

// "+NN" at `at`; the sign may be '+' or '-', but the remote only ever sends '+'.
bool parse_axis(std::string_view text, size_t at, uint8_t& out) {
    const char sign = text[at];
    const char tens = text[at + 1];
    const char ones = text[at + 2];
    if ((sign != '+' && sign != '-') || tens < '0' || tens > '9' || ones < '0' || ones > '9') {
        return false;
    }
    out = (tens - '0') * 10 + (ones - '0');
    return true;
}

// A stick axis as -1..1: in steps of a tenth, rounded toward the middle.
float axis(uint8_t value) {
    return static_cast<float>((2 * value - 2 * STICK_MIDDLE) / STICK_STEPS) / STICK_STEPS;
}

}  // namespace

Message parse(std::string_view text) {
    Message message;
    if (text == ARM_TEXT) {
        message.kind = Kind::ARM;
    } else if (text == DISARM_TEXT) {
        message.kind = Kind::DISARM;
    } else if (text == TAKEOFF_TEXT) {
        message.kind = Kind::TAKEOFF;
    } else if (text.size() >= STICK_LENGTH && text[0] == STICK_PREFIX && parse_axis(text, X_AT, message.x) &&
               parse_axis(text, Y_AT, message.y)) {
        message.kind = Kind::STICK;
    }
    return message;
}

Steer steer(const Message& stick) {
    return {-axis(stick.y), axis(stick.x)};  // the remote's y is flipped
}

size_t format_reply(const Message& message, uint8_t index, const Telemetry& t, char* out, size_t size) {
    int length = 0;
    switch (message.kind) {
        case Kind::INVALID:
            break;
        case Kind::ARM:
            length = snprintf(out, size, "ARMED");
            break;
        case Kind::DISARM:
            length = snprintf(out, size, "DISARMED");
            break;
        case Kind::TAKEOFF:
            length = snprintf(out, size, "TAKING_OFF");
            break;
        case Kind::STICK: {
            char head[8];
            if (index % STATUS_EVERY == 0) {
                snprintf(head, sizeof head, "%c%+d%+03d", STATUS_PREFIX, t.armed ? 1 : 0,
                         std::clamp(t.battery_pct, -1, MAX_2_DIGITS));
            } else {
                snprintf(head, sizeof head, "%c", STICK_PREFIX);
            }
            const auto three = [](auto value) {
                using T = decltype(value);
                return value == value ? std::clamp<T>(value, -MAX_3_DIGITS, MAX_3_DIGITS) : T{};  // NaN as 0
            };
            length = snprintf(out, size, "%s+%02d+%02d%+04.0f%+04.0f%+04d%+04d%+03d%+03d", head, message.x, message.y,
                              three(t.yaw_dps), three(t.pitch_deg), three(t.left_power), three(t.right_power),
                              std::clamp(t.core0_load, 0, MAX_2_DIGITS), std::clamp(t.core1_load, 0, MAX_2_DIGITS));
            break;
        }
    }
    return length > 0 ? std::min(static_cast<size_t>(length), size - 1) : 0;
}

}  // namespace remote
