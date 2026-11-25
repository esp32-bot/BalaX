// Renders the status screen fixtures and checks them against the reference frames in
// host/goldens/, which are raw little-endian RGB565, 135 x 240.
//
//   frames verify [--fixture ID] [--goldens DIR] [--negative-control]
//   frames write-goldens [--fixture ID] [--goldens DIR]
//   frames dump --out DIR      raw frames and manifest.json, for the live preview's native mode
//   frames hashes [--sweep]    frame hashes, for the native/Wasm parity check
//
// Prints one JSON line. Exit status: 0 pass, 1 a frame differs, 2 bad arguments, 4 a reference
// frame is missing or a file can't be read or written. A difference wins over a missing frame.
// --negative-control flips one pixel of every render first, and passes only if every flip is
// caught.

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <string>
#include <string_view>

#include "fixtures.h"
#include "frames.h"

#ifndef FRAMES_GOLDEN_DIR
#define FRAMES_GOLDEN_DIR "host/goldens"
#endif

namespace {

constexpr int PASS = 0;
constexpr int FAIL = 1;
constexpr int INVALID = 2;
constexpr int INFRA = 4;
constexpr int SCHEMA_VERSION = 1;

using Bytes = std::array<uint8_t, sizeof(frames::Frame)>;

struct Args {
    std::string_view command;
    std::string_view fixture;  // all of them if empty
    std::string goldens = FRAMES_GOLDEN_DIR;
    std::string out;
    bool negative_control = false;
    bool sweep = false;
};

std::string json_string(std::string_view text) {
    std::string out = "\"";
    for (const char c : text) {
        if (c == '"' || c == '\\') {
            out += '\\';
            out += c;
        } else if (static_cast<unsigned char>(c) < 0x20) {
            char escaped[8];
            snprintf(escaped, sizeof escaped, "\\u%04x", static_cast<unsigned>(c));
            out += escaped;
        } else {
            out += c;
        }
    }
    return out + '"';
}

std::string hex64(uint64_t value) {
    char text[20];
    snprintf(text, sizeof text, "\"%016llx\"", static_cast<unsigned long long>(value));
    return text;
}

std::string hex16(uint16_t value) {
    char text[10];
    snprintf(text, sizeof text, "\"0x%04x\"", value);
    return text;
}

std::string golden_path(const Args& args, const fixtures::Fixture& f) {
    return args.goldens + "/" + f.id + ".rgb565";
}

bool read_frame(const std::string& path, frames::Frame& frame) {
    FILE* file = fopen(path.c_str(), "rb");
    if (!file) {
        return false;
    }
    Bytes bytes;
    const bool whole = fread(bytes.data(), 1, bytes.size(), file) == bytes.size() && fgetc(file) == EOF;
    fclose(file);
    for (size_t i = 0; whole && i < frame.size(); ++i) {
        frame[i] = static_cast<uint16_t>(bytes[2 * i] | bytes[2 * i + 1] << 8);
    }
    return whole;
}

bool write_frame(const std::string& path, const frames::Frame& frame) {
    Bytes bytes;
    for (size_t i = 0; i < frame.size(); ++i) {
        bytes[2 * i] = frame[i] & 0xff;
        bytes[2 * i + 1] = frame[i] >> 8;
    }
    FILE* file = fopen(path.c_str(), "wb");
    if (!file) {
        return false;
    }
    const bool written = fwrite(bytes.data(), 1, bytes.size(), file) == bytes.size();
    return fclose(file) == 0 && written;
}

bool write_text(const std::string& path, const std::string& text) {
    FILE* file = fopen(path.c_str(), "wb");
    if (!file) {
        return false;
    }
    const bool written = fwrite(text.data(), 1, text.size(), file) == text.size();
    return fclose(file) == 0 && written;
}

bool selected(const Args& args, const fixtures::Fixture& f) {
    return args.fixture.empty() || args.fixture == f.id;
}

double elapsed_ms(std::chrono::steady_clock::time_point start) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

// Where actual differs from expected: how many pixels, the first in reading order, and the
// rectangle around them all.
std::string compare(const frames::Frame& expected, const frames::Frame& actual, int& changed) {
    changed = 0;
    int first = -1;
    int left = frames::WIDTH, top = frames::HEIGHT, right = -1, bottom = -1;
    for (int i = 0; i < static_cast<int>(actual.size()); ++i) {
        if (expected[i] == actual[i]) {
            continue;
        }
        if (changed++ == 0) {
            first = i;
        }
        const int x = i % frames::WIDTH;
        const int y = i / frames::WIDTH;
        left = std::min(left, x);
        right = std::max(right, x);
        top = std::min(top, y);
        bottom = std::max(bottom, y);
    }
    if (changed == 0) {
        return "";
    }
    char text[200];
    snprintf(text, sizeof text,
             ",\"changed\":%d,\"first\":{\"x\":%d,\"y\":%d,\"expected\":%s,\"actual\":%s},"
             "\"bbox\":{\"x\":%d,\"y\":%d,\"w\":%d,\"h\":%d}",
             changed, first % frames::WIDTH, first / frames::WIDTH, hex16(expected[first]).c_str(),
             hex16(actual[first]).c_str(), left, top, right - left + 1, bottom - top + 1);
    return text;
}

int verify(const Args& args) {
    const auto start = std::chrono::steady_clock::now();
    bool differs = false;
    bool missing = false;
    bool missed = false;  // a negative control flip that wasn't caught
    std::string list;
    for (const fixtures::Fixture& f : fixtures::all()) {
        if (!selected(args, f)) {
            continue;
        }
        frames::Frame actual;
        frames::render(actual, f.screen());
        if (args.negative_control) {
            actual[frames::HEIGHT / 2 * frames::WIDTH + frames::WIDTH / 2] ^= 1;
        }
        std::string entry = "{\"id\":" + json_string(f.id) + ",\"hash\":" + hex64(frames::hash(actual));
        frames::Frame expected;
        if (!read_frame(golden_path(args, f), expected)) {
            missing = true;
            entry += ",\"status\":\"missing\",\"golden\":" + json_string(golden_path(args, f));
        } else {
            int changed = 0;
            const std::string where = compare(expected, actual, changed);
            const char* status = args.negative_control ? (changed ? "caught" : "missed") : (changed ? "fail" : "pass");
            differs |= changed > 0;
            missed |= changed == 0;
            entry += ",\"status\":\"" + std::string(status) + "\",\"expectedHash\":" + hex64(frames::hash(expected)) + where;
        }
        list += (list.empty() ? "" : ",") + entry + "}";
    }
    const bool failed = args.negative_control ? missed : differs;
    const int exit_code = failed ? FAIL : missing ? INFRA : PASS;
    const char* status = exit_code == PASS ? "pass" : exit_code == FAIL ? "fail" : "infra";
    printf("{\"schemaVersion\":%d,\"command\":\"verify\",\"negativeControl\":%s,\"status\":\"%s\",\"elapsedMs\":%.2f,"
           "\"fixtures\":[%s]}\n",
           SCHEMA_VERSION, args.negative_control ? "true" : "false", status, elapsed_ms(start), list.c_str());
    return exit_code;
}

int write_goldens(const Args& args) {
    std::error_code error;
    std::filesystem::create_directories(args.goldens, error);
    bool ok = !error;
    std::string list;
    for (const fixtures::Fixture& f : fixtures::all()) {
        if (!selected(args, f)) {
            continue;
        }
        frames::Frame frame;
        frames::render(frame, f.screen());
        const bool written = ok && write_frame(golden_path(args, f), frame);
        ok &= written;
        list += (list.empty() ? "" : ",") + std::string("{\"id\":") + json_string(f.id) + ",\"hash\":" +
                hex64(frames::hash(frame)) + ",\"written\":" + (written ? "true" : "false") + "}";
    }
    printf("{\"schemaVersion\":%d,\"command\":\"write-goldens\",\"status\":\"%s\",\"goldens\":%s,\"fixtures\":[%s]}\n",
           SCHEMA_VERSION, ok ? "pass" : "infra", json_string(args.goldens).c_str(), list.c_str());
    return ok ? PASS : INFRA;
}

int dump(const Args& args) {
    std::error_code error;
    std::filesystem::create_directories(args.out, error);
    bool ok = !error;
    std::string list;
    for (const fixtures::Fixture& f : fixtures::all()) {
        frames::Frame frame;
        frames::render(frame, f.screen());
        const std::string file = std::string(f.id) + ".rgb565";
        ok = ok && write_frame(args.out + "/" + file, frame);
        list += (list.empty() ? "" : ",") + std::string("{\"id\":") + json_string(f.id) + ",\"file\":" + json_string(file) +
                ",\"hash\":" + hex64(frames::hash(frame)) + "}";
    }
    char head[96];
    snprintf(head, sizeof head, "{\"schemaVersion\":%d,\"width\":%d,\"height\":%d,\"fixtures\":[", SCHEMA_VERSION,
             frames::WIDTH, frames::HEIGHT);
    const std::string manifest = head + list + "]}\n";
    ok = ok && write_text(args.out + "/manifest.json", manifest);
    printf("%s", ok ? manifest.c_str() : "{\"schemaVersion\":1,\"command\":\"dump\",\"status\":\"infra\"}\n");
    return ok ? PASS : INFRA;
}

int hashes(const Args& args) {
    std::string list;
    frames::Frame frame;
    for (const fixtures::Fixture& f : fixtures::all()) {
        frames::render(frame, f.screen());
        list += (list.empty() ? "" : ",") + std::string("{\"id\":") + json_string(f.id) + ",\"hash\":" +
                hex64(frames::hash(frame)) + "}";
    }
    std::string sweep;
    for (int k = 0; args.sweep && k < frames::SWEEP_COUNT; ++k) {
        frames::render_sweep(frame, k);
        sweep += (k ? "," : "") + hex64(frames::hash(frame));
    }
    printf("{\"schemaVersion\":%d,\"command\":\"hashes\",\"status\":\"pass\",\"fixtures\":[%s],\"sweep\":[%s]}\n",
           SCHEMA_VERSION, list.c_str(), sweep.c_str());
    return PASS;
}

int invalid(const char* program, const std::string& error) {
    printf("{\"schemaVersion\":%d,\"status\":\"invalid\",\"error\":%s}\n", SCHEMA_VERSION, json_string(error).c_str());
    fprintf(stderr,
            "usage: %s verify [--fixture ID] [--goldens DIR] [--negative-control]\n"
            "       %s write-goldens [--fixture ID] [--goldens DIR]\n"
            "       %s dump --out DIR\n"
            "       %s hashes [--sweep]\n",
            program, program, program, program);
    return INVALID;
}

}  // namespace

int main(int argc, char** argv) {
    Args args;
    if (argc < 2) {
        return invalid(argv[0], "no command");
    }
    args.command = argv[1];
    for (int i = 2; i < argc; ++i) {
        const std::string_view arg = argv[i];
        const bool has_value = i + 1 < argc;
        if (arg == "--fixture" && has_value) {
            args.fixture = argv[++i];
        } else if (arg == "--goldens" && has_value) {
            args.goldens = argv[++i];
        } else if (arg == "--out" && has_value) {
            args.out = argv[++i];
        } else if (arg == "--negative-control") {
            args.negative_control = true;
        } else if (arg == "--sweep") {
            args.sweep = true;
        } else {
            return invalid(argv[0], "unknown argument: " + std::string(arg));
        }
    }
    if (!args.fixture.empty()) {
        bool known = false;
        for (const fixtures::Fixture& f : fixtures::all()) {
            known |= args.fixture == f.id;
        }
        if (!known) {
            return invalid(argv[0], "unknown fixture: " + std::string(args.fixture));
        }
    }
    if (args.command == "verify") return verify(args);
    if (args.command == "write-goldens") return write_goldens(args);
    if (args.command == "dump") return args.out.empty() ? invalid(argv[0], "dump needs --out DIR") : dump(args);
    if (args.command == "hashes") return hashes(args);
    return invalid(argv[0], "unknown command: " + std::string(args.command));
}
