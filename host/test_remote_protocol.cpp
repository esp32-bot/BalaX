#include <cmath>
#include <cstring>
#include <string_view>

#include "check.h"
#include "remote_protocol.h"

using namespace remote;

TEST(parses_commands) {
    CHECK(parse("ARM").kind == Kind::ARM);
    CHECK(parse("DISARM").kind == Kind::DISARM);
    CHECK(parse("REQUEST_TAKEOFF").kind == Kind::TAKEOFF);
    CHECK(parse("ARMED").kind == Kind::INVALID);
    CHECK(parse("").kind == Kind::INVALID);
}

TEST(parses_stick) {
    const Message m = parse("c+07+93");
    CHECK(m.kind == Kind::STICK);
    CHECK(m.x == 7);
    CHECK(m.y == 93);
    CHECK(parse("c+0a+50").kind == Kind::INVALID);
    CHECK(parse("c+50+5").kind == Kind::INVALID);
    CHECK(parse("x+50+50").kind == Kind::INVALID);
}

TEST(steers_like_the_arduino_firmware) {
    // x = int(0.1 * (2X - 100)), y = int(-0.1 * (2Y - 100)), each then / 10.
    const Steer middle = steer(parse("c+50+50"));
    CHECK(middle.forward == 0 && middle.turn == 0);
    const Steer near_middle = steer(parse("c+54+46"));
    CHECK(near_middle.forward == 0 && near_middle.turn == 0);
    const Steer corner = steer(parse("c+99+00"));
    CHECK(corner.turn == 0.9f);
    CHECK(corner.forward == 1.0f);
    const Steer other = steer(parse("c+00+99"));
    CHECK(other.turn == -1.0f);
    CHECK(other.forward == -0.9f);
}

TEST(replies_to_commands) {
    char out[TEXT_BYTES];
    const Telemetry t{};
    CHECK(std::string_view(out, format_reply(parse("ARM"), 1, t, out, sizeof out)) == "ARMED");
    CHECK(std::string_view(out, format_reply(parse("DISARM"), 1, t, out, sizeof out)) == "DISARMED");
    CHECK(std::string_view(out, format_reply(parse("REQUEST_TAKEOFF"), 1, t, out, sizeof out)) == "TAKING_OFF");
    CHECK(format_reply(parse("junk"), 1, t, out, sizeof out) == 0);
}

TEST(replies_with_telemetry_at_fixed_positions) {
    char out[TEXT_BYTES];
    const Telemetry t{true, 72, -3.4f, 12.6f, 45, -100, 7, 11};
    // The example in the JoyX source: c+50+50+007+029+000+000+73+11
    CHECK(std::string_view(out, format_reply(parse("c+50+05"), 1, t, out, sizeof out)) ==
          "c+50+05-003+013+045-100+07+11");
    CHECK(std::string_view(out, format_reply(parse("c+50+05"), 200, t, out, sizeof out)) ==
          "s+1+72+50+05-003+013+045-100+07+11");
    const Telemetry extreme{false, 100, 5000, -5000, 1270, -1270, 100, -3};
    CHECK(std::string_view(out, format_reply(parse("c+99+99"), 1, extreme, out, sizeof out)) ==
          "c+99+99+999-999+999-999+99+00");
    CHECK(std::string_view(out, format_reply(parse("c+99+99"), 0, extreme, out, sizeof out)).starts_with("s+0+99+99"));
    const Telemetry unknown{false, -1, NAN, NAN, 0, 0, 0, 0};
    CHECK(std::string_view(out, format_reply(parse("c+50+50"), 1, unknown, out, sizeof out)) ==
          "c+50+50+000+000+000+000+00+00");
}
