#include "commands.h"

#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <iterator>

#include "app_settings.h"
#include "board.h"
#include "console_commands.h"
#include "esp_app_desc.h"
#include "esp_flash.h"
#include "esp_flash_partitions.h"
#include "esp_image_format.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "idle_power.h"
#include "spi_flash_mmap.h"
#include "imu_calibration.h"
#include "control_settings.h"
#include "power_mode.h"
#include "prog_vm.h"
#include "robot.h"
#include "sounds.h"
#include "status.h"
#include "telemetry_stream.h"

namespace {

bool parse(const char* text, uint16_t& out) {
    const char* end = text + strlen(text);
    const auto [stop, error] = std::from_chars(text, end, out);
    return error == std::errc{} && stop == end;
}

bool parse_float(const char* text, float& out) {
    if (text == nullptr || *text == '\0') {
        return false;
    }
    char* end = nullptr;
    const float val = std::strtof(text, &end);
    if (end == text || *end != '\0' || !std::isfinite(val)) {
        return false;
    }
    out = val;
    return true;
}

bool has_force_flag(int argc, char** argv) {
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "force") == 0 || strcmp(argv[i], "--force") == 0) {
            return true;
        }
    }
    return false;
}

int status_command(int argc, char** argv) {
    if (argc > 1) {
        return print_probe(argc, argv);
    }
    print_status();
    return 0;
}

int probe_command(int argc, char** argv) {
    return print_probe(argc, argv);
}


int ctrl_command(int argc, char** argv) {
    if (argc >= 2 && strcmp(argv[1], "classic") == 0) {
        control_settings::apply_preset(false);
        printf("Control preset: classic (all 5 modules = 0). Run 'save' to persist.\n");
        return 0;
    }
    if (argc >= 2 && strcmp(argv[1], "sota") == 0) {
        control_settings::apply_preset(true);
        printf("Control preset: sota (est=vqf_6dof ctrl=lqr_eso perch=lyapunov shaper=smooth_stiction takeoff=energy_swingup). Run 'save' to persist.\n");
        return 0;
    }
    if (argc >= 3) {
        const int mode = atoi(argv[2]) ? 1 : 0;
        if (!control_settings::set_stage(argv[1], mode)) {
            printf("Unknown stage '%s'. Use: est, ctrl, perch, shaper, takeoff (0 or 1), or classic/sota.\n", argv[1]);
            return 1;
        }
        printf("Set %s = %d (live in RAM; run 'save' to persist).\n", argv[1], mode);
    }
    const auto cfg = control_settings::current();
    const auto t = robot::telemetry();
    printf("est       %d (%s)\n", static_cast<int>(cfg.estimator_mode()), control::name(cfg.estimator_mode()).data());
    printf("ctrl      %d (%s)\n", static_cast<int>(cfg.controller_mode()), control::name(cfg.controller_mode()).data());
    printf("perch     %d (%s)\n", static_cast<int>(cfg.perch_mode()), control::name(cfg.perch_mode()).data());
    printf("shaper    %d (%s)\n", static_cast<int>(cfg.shaper_mode()), control::name(cfg.shaper_mode()).data());
    printf("takeoff   %d (%s)\n", static_cast<int>(cfg.takeoff_mode()), control::name(cfg.takeoff_mode()).data());
    printf("perched   %s (perch_pct=%.1f%% duty=%.1f%% energy=%.1f eq_deg=%.2f)\n",
           t.terms.perched ? "yes" : "no", t.terms.perch_pct, t.terms.duty_pct, t.terms.energy_u2, t.terms.eq_deg);
    return 0;
}

int power_command(int, char**) {
    const float usb_v = board.pmic.usb_v();
    const float usb_ma = board.pmic.usb_ma();
    const float pmic_c = board.pmic.temperature_c();
    const float bat_v = board.pmic.battery_v();
    const float bat_ma = board.pmic.battery_ma();
    const bool usb = idle_power::usb_present(usb_v);
    printf("%-13s %s\n", "power_mode", idle_power::name(power_mode::current()));
    printf("%-13s %s\n", "usb_plugged", usb ? "yes" : "no");
    printf("%-13s %.2f\n", "usb_v", usb_v);
    printf("%-13s %.0f\n", "usb_ma", usb_ma);
    printf("%-13s %.1f\n", "pmic_c", pmic_c);
    printf("%-13s %.2f\n", "battery_v", bat_v);
    printf("%-13s %.0f\n", "battery_ma", bat_ma);
    printf("%-13s %lld\n", "idle_s", static_cast<long long>(power_mode::idle_seconds()));
    printf("%-13s %u\n", "idle_sleep_s", static_cast<unsigned>(idle_sleep_s.get()));
    return 0;
}

int sleep_command(int argc, char** argv) {
    const bool force = has_force_flag(argc, argv);
    const idle_power::GuardResult res = power_mode::enter_sleep(force);
    if (!res.ok) {
        printf("%s\n", res.reason);
        return 1;
    }
    printf("Entered wifi-sleep (WIFI_PS_MAX_MODEM + light sleep, faint 4s LED pulse). Run 'wake' or press Button A to resume.\n");
    return 0;
}

int wake_command(int, char**) {
    power_mode::wake();
    printf("Awake (80 MHz, 100 Hz balance loop active).\n");
    return 0;
}

int off_command(int argc, char** argv) {
    const bool force = has_force_flag(argc, argv);
    const idle_power::GuardResult res = power_mode::enter_off(force);
    if (!res.ok) {
        printf("%s\n", res.reason);
        return 1;
    }
    printf("Entering usb-sleep (deep sleep, ultra-dim 10s LED pulse; auto-switches to wifi-sleep if USB unplugged). Wake via USB EN or Button A.\n");
    return 0;
}

int prog_command(int argc, char** argv) {
    if (argc == 2 && strcmp(argv[1], "stop") == 0) {
        prog_vm::stop();
        printf("Mini-program stop requested.\n");
        return 0;
    }
    if (argc == 2 && strcmp(argv[1], "clear") == 0) {
        prog_vm::clear_cache();
        printf("Cleared unpinned DRAM applet cache entries.\n");
        return 0;
    }
    prog_vm::print_status();
    return 0;
}

int imu_command(int argc, char** argv) {
    if (argc >= 2 && (strcmp(argv[1], "zero") == 0 || strcmp(argv[1], "heading") == 0 ||
                      strcmp(argv[1], "accel") == 0 || strcmp(argv[1], "cg") == 0)) {
        power_mode::wake();
    }
    if (argc == 2 && strcmp(argv[1], "zero") == 0) {
        const robot::ZeroImuResult r = robot::zero_imu();
        if (r.ok) {
            printf("IMU zeroed & saved to flash! gyro_bias=(%.2f, %.2f, %.2f) dps, acc_bias=(%.3f, %.3f, %.3f) g%s (spread=%.2f dps)\n",
                   r.gyro_bx, r.gyro_by, r.gyro_bz, r.acc_bx, r.acc_by, r.acc_bz,
                   r.preserved_6face ? " [6-face kept]" : "", r.max_gyro_spread_dps);
            return 0;
        }
        if (r.moved) {
            printf("IMU zero aborted: robot moved (spread=%.2f dps). Hold still and retry.\n", r.max_gyro_spread_dps);
            return 1;
        }
        printf("IMU zero failed (sensor or NVS error).\n");
        return 1;
    }
    if (argc == 2 && strcmp(argv[1], "heading") == 0) {
        robot::reset_heading();
        printf("IMU heading zeroed.\n");
        return 0;
    }
    if (argc == 8 && strcmp(argv[1], "accel") == 0) {
        m5::Vector3 bias{}, scale{};
        if (!parse_float(argv[2], bias.x) || !parse_float(argv[3], bias.y) || !parse_float(argv[4], bias.z) ||
            !parse_float(argv[5], scale.x) || !parse_float(argv[6], scale.y) || !parse_float(argv[7], scale.z)) {
            printf("Usage: imu accel <bx> <by> <bz> <sx> <sy> <sz>\n");
            return 1;
        }
        if (imu_cal::save_accel(bias, scale) != ESP_OK) {
            printf("Failed to save 6-face accel calibration to flash.\n");
            return 1;
        }
        robot::reset_fusion();
        board.speaker.play(DEFAULT_BEEP);
        const imu_cal::Offsets cal = imu_cal::current();
        printf("6-face accel calibration saved to flash! bias=(%.4f, %.4f, %.4f) g scale=(%.4f, %.4f, %.4f)\n",
               cal.accel_g.x, cal.accel_g.y, cal.accel_g.z,
               cal.accel_scale.x, cal.accel_scale.y, cal.accel_scale.z);
        return 0;
    }
    if (argc == 5 && strcmp(argv[1], "cg") == 0) {
        m5::Vector3 lever{};
        if (!parse_float(argv[2], lever.x) || !parse_float(argv[3], lever.y) || !parse_float(argv[4], lever.z)) {
            printf("Usage: imu cg <rx_m> <ry_m> <rz_m>\n");
            return 1;
        }
        if (imu_cal::save_lever_arm(lever) != ESP_OK) {
            printf("Failed to save CG lever-arm offset to flash.\n");
            return 1;
        }
        board.speaker.play(DEFAULT_BEEP);
        const imu_cal::Offsets cal = imu_cal::current();
        printf("CG->IMU lever-arm saved to flash! r=(%.4f, %.4f, %.4f) m\n",
               cal.lever_arm_m.x, cal.lever_arm_m.y, cal.lever_arm_m.z);
        return 0;
    }
    const imu_cal::Offsets cal = imu_cal::current();
    const robot::Telemetry t = robot::telemetry();
    printf("calibrated   %s\n", cal.valid ? "yes (flash)" : "no");
    printf("accel_6face  %s\n", cal.accel_6face ? "yes" : "no");
    printf("gyro_bias    (%.4f, %.4f, %.4f) dps\n", cal.gyro_dps.x, cal.gyro_dps.y, cal.gyro_dps.z);
    printf("accel_bias   (%.4f, %.4f, %.4f) g\n", cal.accel_g.x, cal.accel_g.y, cal.accel_g.z);
    printf("accel_scale  (%.4f, %.4f, %.4f)\n", cal.accel_scale.x, cal.accel_scale.y, cal.accel_scale.z);
    printf("lever_arm_m  (%.4f, %.4f, %.4f) m\n", cal.lever_arm_m.x, cal.lever_arm_m.y, cal.lever_arm_m.z);
    printf("fused_deg    pitch=%.1f roll=%.1f yaw=%.1f (%s)\n",
           t.fused_deg.pitch, t.fused_deg.roll, t.fused_deg.yaw, t.imu_resting ? "REST" : "MOVING");
    return 0;
}

int stream_command(int argc, char** argv) {
    if (argc == 2 && strcmp(argv[1], "stop") == 0) {
        telemetry_stream::stop_and_free();
        printf("Telemetry stream stopped and all RAM/flash buffers freed.\n");
        return 0;
    }
    const auto s = telemetry_stream::stats();
    printf("active            %s\n", s.active ? "yes" : "no");
    printf("latest_tick       %lu\n", static_cast<unsigned long>(s.latest_tick));
    printf("last_ack_tick     %lu\n", static_cast<unsigned long>(s.last_ack_tick));
    printf("ram_chunks        %lu (%lu bytes)\n", static_cast<unsigned long>(s.ram_chunks),
           static_cast<unsigned long>(s.ram_bytes));
    printf("flash_chunks      %lu (%lu bytes)\n", static_cast<unsigned long>(s.flash_chunks),
           static_cast<unsigned long>(s.flash_bytes));
    printf("samples_sent      %lu\n", static_cast<unsigned long>(s.total_samples_sent));
    printf("compressed_bytes  %lu\n", static_cast<unsigned long>(s.total_bytes_compressed));
    return 0;
}

// partitions: the flash layout as the robot booted it, from its own partition table (never a
// partitions.csv on some laptop), for the remote's Flash tab. Only reads headers, so it is
// cheap, but it reads flash, which stalls the other core for a moment: run it on demand.
//
//   flash_size 0x400000
//   # label type subtype offset size image flags state version
//   ota_0 app ota_0 0x020000 0x1e0000 0x0f3a10 running,boot valid 224e4a8-dirty
//
// image: bytes of the region its app (or bootloader) image takes; flags: running, boot (what
// the next reset starts), next (where an OTA update goes), enc, ro; state: the OTA rollback
// state; version: the image's, last as it may hold spaces. "-" where nothing applies.

// The names partitions.csv uses (gen_esp32part.py), or the number for the rest.
const char* type_name(uint8_t type, char (&buf)[8]) {
    switch (type) {
        case ESP_PARTITION_TYPE_APP: return "app";
        case ESP_PARTITION_TYPE_DATA: return "data";
        case ESP_PARTITION_TYPE_BOOTLOADER: return "bootloader";
        case ESP_PARTITION_TYPE_PARTITION_TABLE: return "partition_table";
        default: snprintf(buf, sizeof buf, "0x%02x", type); return buf;
    }
}

const char* subtype_name(uint8_t type, uint8_t subtype, char (&buf)[8]) {
    static constexpr const char* DATA[] = {"ota", "phy", "nvs", "coredump", "nvs_keys", "efuse", "undefined"};
    static constexpr const char* IMAGE[] = {"primary", "ota", "recovery"};  // bootloader, partition table
    if (type == ESP_PARTITION_TYPE_APP) {
        if (subtype == ESP_PARTITION_SUBTYPE_APP_FACTORY) return "factory";
        if (subtype == ESP_PARTITION_SUBTYPE_APP_TEST) return "test";
        if (subtype >= ESP_PARTITION_SUBTYPE_APP_OTA_MIN && subtype < ESP_PARTITION_SUBTYPE_APP_OTA_MAX) {
            snprintf(buf, sizeof buf, "ota_%d", subtype - ESP_PARTITION_SUBTYPE_APP_OTA_MIN);
            return buf;
        }
    } else if (type == ESP_PARTITION_TYPE_DATA) {
        if (subtype < std::size(DATA)) return DATA[subtype];
        switch (subtype) {
            case ESP_PARTITION_SUBTYPE_DATA_ESPHTTPD: return "esphttpd";
            case ESP_PARTITION_SUBTYPE_DATA_FAT: return "fat";
            case ESP_PARTITION_SUBTYPE_DATA_SPIFFS: return "spiffs";
            case ESP_PARTITION_SUBTYPE_DATA_LITTLEFS: return "littlefs";
            default: break;
        }
    } else if (type == ESP_PARTITION_TYPE_BOOTLOADER || type == ESP_PARTITION_TYPE_PARTITION_TABLE) {
        if (subtype < std::size(IMAGE)) return IMAGE[subtype];
    }
    snprintf(buf, sizeof buf, "0x%02x", subtype);
    return buf;
}

const char* ota_state_name(esp_ota_img_states_t state) {
    switch (state) {
        case ESP_OTA_IMG_NEW: return "new";
        case ESP_OTA_IMG_PENDING_VERIFY: return "pending";
        case ESP_OTA_IMG_VALID: return "valid";
        case ESP_OTA_IMG_INVALID: return "invalid";
        case ESP_OTA_IMG_ABORTED: return "aborted";
        case ESP_OTA_IMG_UNDEFINED: return "undefined";
    }
    return "?";
}

// Bytes the image at the start of a region takes, from its headers alone; 0 when there's none.
uint32_t image_bytes(uint32_t offset, uint32_t size) {
    const esp_partition_pos_t pos = {.offset = offset, .size = size};
    esp_image_metadata_t meta{};
    return esp_image_get_metadata(&pos, &meta) == ESP_OK ? meta.image_len : 0;
}

void print_region(const char* label, const char* type, const char* subtype, uint32_t offset, uint32_t size,
                  uint32_t image, const char* flags, const char* state, const char* version) {
    char image_hex[12] = "-";
    if (image != 0) {
        snprintf(image_hex, sizeof image_hex, "0x%06lx", static_cast<unsigned long>(image));
    }
    printf("%-10s %-10s %-8s 0x%06lx 0x%06lx %-8s %-12s %-7s %s\n", label, type, subtype,
           static_cast<unsigned long>(offset), static_cast<unsigned long>(size), image_hex,
           *flags != '\0' ? flags : "-", state, *version != '\0' ? version : "-");
}

bool same(const esp_partition_t* a, const esp_partition_t* b) {
    return a != nullptr && b != nullptr && a->address == b->address;
}

int partitions_command(int, char**) {
    uint32_t flash_size = 0;
    esp_flash_get_size(nullptr, &flash_size);
    printf("flash_size 0x%06lx\n", static_cast<unsigned long>(flash_size));
    printf("# label type subtype offset size image flags state version\n");

    // The bootloader and the table itself only show up in the table when partitions.csv lists them.
    char buf[8], sub[8];
    if (esp_partition_find_first(ESP_PARTITION_TYPE_BOOTLOADER, ESP_PARTITION_SUBTYPE_ANY, nullptr) == nullptr) {
        const uint32_t size = ESP_PARTITION_TABLE_OFFSET - ESP_PRIMARY_BOOTLOADER_OFFSET;
        esp_bootloader_desc_t desc{};
        const bool described = esp_ota_get_bootloader_description(nullptr, &desc) == ESP_OK;
        print_region("bootloader", "bootloader", "primary", ESP_PRIMARY_BOOTLOADER_OFFSET, size,
                     image_bytes(ESP_PRIMARY_BOOTLOADER_OFFSET, size), "", "-", described ? desc.idf_ver : "");
    }
    if (esp_partition_find_first(ESP_PARTITION_TYPE_PARTITION_TABLE, ESP_PARTITION_SUBTYPE_ANY, nullptr) == nullptr) {
        print_region("part_table", "partition_table", "primary", ESP_PARTITION_TABLE_OFFSET,
                     SPI_FLASH_SEC_SIZE, 0, "", "-", "");
    }

    const esp_partition_t* running = esp_ota_get_running_partition();
    const esp_partition_t* boot = esp_ota_get_boot_partition();
    const esp_partition_t* next = esp_ota_get_next_update_partition(nullptr);
    esp_partition_iterator_t it = esp_partition_find(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, nullptr);
    for (; it != nullptr; it = esp_partition_next(it)) {
        const esp_partition_t* p = esp_partition_get(it);
        char flags[40] = "";
        const auto flag = [&](bool set, const char* name) {
            if (set) {
                snprintf(flags + strlen(flags), sizeof flags - strlen(flags), "%s%s", *flags ? "," : "", name);
            }
        };
        flag(same(p, running), "running");
        flag(same(p, boot), "boot");
        flag(same(p, next), "next");
        flag(p->encrypted, "enc");
        flag(p->readonly, "ro");

        uint32_t image = 0;
        const char* state = "-";
        esp_app_desc_t desc{};
        bool described = false;
        if (p->type == ESP_PARTITION_TYPE_APP || p->type == ESP_PARTITION_TYPE_BOOTLOADER) {
            image = image_bytes(p->address, p->size);
        }
        if (p->type == ESP_PARTITION_TYPE_APP) {
            described = esp_ota_get_partition_description(p, &desc) == ESP_OK;
            esp_ota_img_states_t ota_state;
            if (esp_ota_get_state_partition(p, &ota_state) == ESP_OK) {
                state = ota_state_name(ota_state);
            }
        }
        print_region(p->label, type_name(p->type, buf), subtype_name(p->type, p->subtype, sub), p->address,
                     p->size, image, flags, state, described ? desc.version : "");
    }
    esp_partition_iterator_release(it);
    return 0;
}

int beep_command(int argc, char** argv) {
    m5::Note note = DEFAULT_BEEP;
    if (argc > 3 || (argc > 1 && !parse(argv[1], note.hz)) || (argc > 2 && !parse(argv[2], note.ms))) {
        printf("Usage: beep [hz] [ms]\n");
        return 1;
    }
    power_mode::wake();
    board.speaker.play(note);
    return 0;
}

int play_command(int argc, char** argv) {
    for (const Melody& melody : MELODIES) {
        if (argc == 2 && melody.name == argv[1]) {
            power_mode::wake();
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

int drive_command(int argc, char** argv) {
    if (argc != 3) {
        printf("Usage: drive <forward -1..1> <turn -1..1>\n");
        return 1;
    }
    char* end1 = nullptr;
    char* end2 = nullptr;
    const float fwd = std::strtof(argv[1], &end1);
    const float turn = std::strtof(argv[2], &end2);
    if (!end1 || end1 == argv[1] || !end2 || end2 == argv[2] || !std::isfinite(fwd) || !std::isfinite(turn)) {
        printf("Usage: drive <forward -1..1> <turn -1..1>\n");
        return 1;
    }
    const float cf = std::clamp(fwd, -1.0f, 1.0f);
    const float ct = std::clamp(turn, -1.0f, 1.0f);
    if (std::abs(cf) > 0.01f || std::abs(ct) > 0.01f) {
        power_mode::note_activity();
    }
    robot::steer(cf, ct);
    printf("drive %.2f %.2f\n", cf, ct);
    return 0;
}

template <balance::Request REQUEST>
int request_command(int, char**) {
    if constexpr (REQUEST != balance::Request::DISARM) {
        power_mode::wake();
    } else {
        power_mode::note_activity();
    }
    robot::request(REQUEST);
    return 0;
}

}  // namespace

esp_err_t register_app_commands() {
    static constexpr console_commands::Command COMMANDS[] = {
        {"status", "[field...]", "Print every reading (or specific fields): power, CPU, memory, balancing, Wi-Fi.", status_command},
        {"probe", "[field...|list]", "Query one or more live readings by name (e.g. `probe cpu_mhz cpu0_load pmic_c`), or `list`.", probe_command},
        {"ctrl", "[classic|sota|<stage> <0|1>]", "Inspect or hot-swap the 5 modular control stages (est/ctrl/perch/shaper/takeoff).", ctrl_command},
        {"power", nullptr, "Print power mode (awake/wifi-sleep/usb-sleep), USB presence, temps, and idle timer.", power_command},
        {"wifi-sleep", "[force]", "Enter Wi-Fi low-power sleep (WIFI_PS_MAX_MODEM + light sleep, faint 4s LED pulse).", sleep_command},
        {"sleep", "[force]", "Alias for wifi-sleep.", sleep_command},
        {"wake", nullptr, "Wake from wifi-sleep back to active 100 Hz mode.", wake_command},
        {"usb-sleep", "[force]", "Enter USB-charge deep sleep (ultra-dim 10s LED pulse; auto-switches to wifi-sleep on unplug).", off_command},
        {"off", "[force]", "Alias for usb-sleep.", off_command},
        {"prog", "[status|stop|clear]", "Inspect, stop, or clear DRAM cache for the sandboxed mini-program VM.", prog_command},
        {"imu", "[zero|heading|accel|cg|status]", "Zero/calibrate 6-axis IMU & CG lever-arm to flash, or inspect status.", imu_command},
        {"stream", "[stop]", "Print compressed telemetry stream queue stats (RAM/flash spillover), or stop.", stream_command},
        {"partitions", nullptr, "Print the live flash partition table: offsets, sizes, images, running/boot/next slot.",
         partitions_command},
        {"arm", nullptr, "Arm: it balances once stood up. Also acknowledges a fall.", request_command<balance::Request::ARM>},
        {"disarm", nullptr, "Disarm: motors off. Also acknowledges a fall.", request_command<balance::Request::DISARM>},
        {"takeoff", nullptr, "Lying down: drive full power to pop up, then balance.",
         request_command<balance::Request::TAKEOFF>},
        {"drive", "<fwd> <turn>", "Steer the balancing robot (-1..1 forward, -1..1 turn).", drive_command},
        {"beep", "[hz] [ms]", "Play a tone, to try the volume.", beep_command},
        {"play", "<name>", "Play a melody.", play_command},
        {"reboot", nullptr, "Restart the robot.", reboot_command},
    };
    return console_commands::register_all(COMMANDS);
}
