#include "commands.h"

#include <charconv>
#include <cstdio>
#include <cstring>

#include "board.h"
#include "console_commands.h"
#include "esp_system.h"
#include "robot.h"
#include "sounds.h"
#include "status.h"

namespace {

bool parse(const char* text, uint16_t& out) {
    const char* end = text + strlen(text);
    const auto [stop, error] = std::from_chars(text, end, out);
    return error == std::errc{} && stop == end;
}

int status_command(int, char**) {
    print_status();
    return 0;
}

int beep_command(int argc, char** argv) {
    m5::Note note = DEFAULT_BEEP;
    if (argc > 3 || (argc > 1 && !parse(argv[1], note.hz)) || (argc > 2 && !parse(argv[2], note.ms))) {
        printf("Usage: beep [hz] [ms]\n");
        return 1;
    }
    board.speaker.play(note);
    return 0;
}

int play_command(int argc, char** argv) {
    for (const Melody& melody : MELODIES) {
        if (argc == 2 && melody.name == argv[1]) {
            board.speaker.play(melody.notes);
            return 0;
        }
    }
    printf("Usage: play <name>, one of:");
    for (const Melody& melody : MELODIES) {
        printf(" %.*s", static_cast<int>(melody.name.size()), melody.name.data());
    }
    printf("\n");
    return 1;
}

int reboot_command(int, char**) {
    esp_restart();
}

template <balance::Request REQUEST>
int request_command(int, char**) {
    robot::request(REQUEST);
    return 0;
}

}  // namespace

esp_err_t register_app_commands() {
    static constexpr console_commands::Command COMMANDS[] = {
        {"status", nullptr, "Print every reading: power, CPU, memory, balancing, Wi-Fi, the remote.", status_command},
        {"arm", nullptr, "Arm: it balances once stood up. Also acknowledges a fall.", request_command<balance::Request::ARM>},
        {"disarm", nullptr, "Disarm: motors off. Also acknowledges a fall.", request_command<balance::Request::DISARM>},
        {"takeoff", nullptr, "Lying down: drive full power to pop up, then balance.",
         request_command<balance::Request::TAKEOFF>},
        {"beep", "[hz] [ms]", "Play a tone, to try the volume.", beep_command},
        {"play", "<name>", "Play a melody.", play_command},
        {"reboot", nullptr, "Restart the robot.", reboot_command},
    };
    return console_commands::register_all(COMMANDS);
}
