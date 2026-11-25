#include "remote.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>

#include "config.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "power_mode.h"
#include "remote_protocol.h"
#include "robot.h"
#include "status.h"

namespace remote {

namespace {

constexpr char TAG[] = "remote";
constexpr uint8_t BROADCAST_MAC[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
constexpr int64_t CAM_BEACON_TIMEOUT_US = 3000000;  // Stop ESP-NOW attitude TX 3 s after cam stops streaming
constexpr int64_t CAM_ATT_PERIOD_US = 20000;        // 50 Hz wireless attitude stream to balax-cam

struct Received {
    uint8_t mac[ESP_NOW_ETH_ALEN];
    Frame frame;
};

QueueHandle_t received;
portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
// Guarded by `lock`.
Link shared;
int64_t heard_us = 0;

// Demand-driven balax-cam ESP-NOW attitude streaming (zero TX / zero CPU wakeups when camera is idle!)
std::atomic<int64_t> cam_beacon_us{0};
bool bcast_peer_added = false;
uint16_t cam_att_seq = 0;

uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x8000u) ? static_cast<uint16_t>((crc << 1) ^ 0x1021u) : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

int16_t clamp_cdeg(float deg) {
    if (std::isnan(deg)) return 0;
    const long v = std::lround(deg * 100.0f);
    return static_cast<int16_t>(std::clamp<long>(v, -32767, 32767));
}

void write_u16_le(uint8_t* p, uint16_t v) {
    p[0] = static_cast<uint8_t>(v & 0xFFu);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xFFu);
}

void write_u32_le(uint8_t* p, uint32_t v) {
    p[0] = static_cast<uint8_t>(v & 0xFFu);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xFFu);
    p[2] = static_cast<uint8_t>((v >> 16) & 0xFFu);
    p[3] = static_cast<uint8_t>((v >> 24) & 0xFFu);
}

void ensure_broadcast_peer() {
    if (bcast_peer_added) return;
    esp_now_peer_info_t peer{};
    memcpy(peer.peer_addr, BROADCAST_MAC, sizeof peer.peer_addr);
    peer.channel = 0;
    wifi_mode_t mode = WIFI_MODE_STA;
    esp_wifi_get_mode(&mode);
    peer.ifidx = (mode == WIFI_MODE_AP || mode == WIFI_MODE_APSTA) ? WIFI_IF_AP : WIFI_IF_STA;
    peer.encrypt = false;
    const esp_err_t err = esp_now_add_peer(&peer);
    if (err == ESP_OK || err == ESP_ERR_ESPNOW_EXIST) {
        bcast_peer_added = true;
    }
}

void broadcast_cam_attitude(int64_t now_us) {
    if (power_mode::is_sleeping()) return;
    ensure_broadcast_peer();
    if (!bcast_peer_added) return;

    const robot::Telemetry t = robot::telemetry();
    uint8_t flags = 1u << 0;  // kAttitudeFlagValid
    if (t.armed) flags |= (1u << 1);
    if (t.state == balance::State::BALANCING) flags |= (1u << 2);
    if (t.state == balance::State::FALLEN) flags |= (1u << 3);

    uint8_t pkt[20];
    pkt[0] = 0xB5;  // balax_cam::kAttitudeSync0
    pkt[1] = 0x5B;  // balax_cam::kAttitudeSync1
    pkt[2] = 1;     // balax_cam::kAttitudeProtoVersion
    pkt[3] = flags;
    write_u16_le(pkt + 4, ++cam_att_seq);
    write_u32_le(pkt + 6, static_cast<uint32_t>(now_us));
    write_u16_le(pkt + 10, static_cast<uint16_t>(clamp_cdeg(t.fused_deg.pitch)));
    write_u16_le(pkt + 12, static_cast<uint16_t>(clamp_cdeg(t.fused_deg.roll)));
    write_u16_le(pkt + 14, static_cast<uint16_t>(clamp_cdeg(t.terms.pitch_dps)));
    write_u16_le(pkt + 16, static_cast<uint16_t>(clamp_cdeg(t.roll_dps)));
    write_u16_le(pkt + 18, crc16_ccitt(pkt, 18));

    esp_now_send(BROADCAST_MAC, pkt, sizeof(pkt));
}

template <typename Change>
void update(Change change) {
    taskENTER_CRITICAL(&lock);
    change(shared, heard_us);
    taskEXIT_CRITICAL(&lock);
}

// In the Wi-Fi task: hands the message over and gets out of its way.
void on_receive(const esp_now_recv_info_t* info, const uint8_t* data, int size) {
    // Demand-driven beacon from balax-cam ("BCAM"): wakes 50 Hz ESP-NOW attitude broadcast without touching JoyX pairing.
    if (size >= 4 && data[0] == 'B' && data[1] == 'C' && data[2] == 'A' && data[3] == 'M') {
        cam_beacon_us.store(esp_timer_get_time(), std::memory_order_relaxed);
        Received wakeup{};
        memcpy(wakeup.mac, info->src_addr, sizeof wakeup.mac);
        xQueueSend(received, &wakeup, 0);
        return;
    }
    Received message{};
    memcpy(message.mac, info->src_addr, sizeof message.mac);
    memcpy(&message.frame, data, std::clamp<int>(size, 0, sizeof message.frame));
    xQueueSend(received, &message, 0);
}

void on_sent(const esp_now_send_info_t*, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS) {
        update([](Link& link, int64_t&) { ++link.failed_replies; });
    }
}

// Whether to obey `mac`: the paired remote, or a new one when none is paired or the paired
// one has gone quiet.
bool accept(const uint8_t* mac, int64_t now_us) {
    Link link;
    int64_t heard = 0;
    update([&](Link& current, int64_t& current_heard) {
        link = current;
        heard = current_heard;
    });
    if (link.paired && memcmp(link.mac, mac, sizeof link.mac) == 0) {
        return true;
    }
    if (link.paired && now_us - heard < REMOTE_TAKEOVER_US) {
        return false;
    }
    if (link.paired) {
        esp_now_del_peer(link.mac);
    }
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, mac, sizeof peer.peer_addr);
    peer.channel = 0;  // whichever the radio is on
    wifi_mode_t mode = WIFI_MODE_STA;
    esp_wifi_get_mode(&mode);
    peer.ifidx = (mode == WIFI_MODE_AP || mode == WIFI_MODE_APSTA) ? WIFI_IF_AP : WIFI_IF_STA;
    peer.encrypt = false;
    const bool added = esp_now_add_peer(&peer) == ESP_OK;
    update([&](Link& current, int64_t&) {
        current.paired = added;
        memcpy(current.mac, mac, sizeof current.mac);
    });
    if (added) {
        ESP_LOGI(TAG, "paired with " MACSTR, MAC2STR(mac));
    }
    return added;
}

void obey(const Message& message) {
    if (message.kind != Kind::INVALID) {
        if (power_mode::is_sleeping()) {
            power_mode::wake();
        } else {
            power_mode::note_activity();
        }
    }
    switch (message.kind) {
        case Kind::STICK: {
            const Steer stick = steer(message);
            robot::steer(stick.forward, stick.turn);
            update([&](Link& link, int64_t&) {
                link.forward = stick.forward;
                link.turn = stick.turn;
            });
            break;
        }
        case Kind::ARM:
            robot::request(balance::Request::ARM);
            break;
        case Kind::DISARM:
            robot::request(balance::Request::DISARM);
            break;
        case Kind::TAKEOFF:
            robot::request(balance::Request::TAKEOFF);
            break;
        case Kind::INVALID:
            break;
    }
}

int whole(float value, int unknown) {
    return std::isnan(value) ? unknown : static_cast<int>(std::lround(value));
}

void reply(const Message& message, uint8_t index, const uint8_t* mac) {
    const robot::Telemetry robot = robot::telemetry();
    const status::Values values = latest_status();
    const Telemetry telemetry = {
        .armed = robot.armed,
        .battery_pct = whole(values.battery_pct, -1),
        .yaw_dps = robot.yaw_dps,
        .pitch_deg = robot.pitch_deg,
        .left_power = robot.motors.left,
        .right_power = robot.motors.right,
        .core0_load = whole(values.cpu0_load, 0),
        .core1_load = whole(values.cpu1_load, 0),
    };
    Frame frame{};
    frame.index = index;
    if (format_reply(message, index, telemetry, frame.text, sizeof frame.text) == 0) {
        return;
    }
    // The whole frame: the remote copies that much.
    if (esp_now_send(mac, reinterpret_cast<const uint8_t*>(&frame), sizeof frame) != ESP_OK) {
        update([](Link& link, int64_t&) { ++link.failed_replies; });
    }
}

void task(void*) {
    Received message;
    int64_t last_att_tx_us = 0;
    for (;;) {
        const int64_t now0 = esp_timer_get_time();
        const int64_t beacon_t = cam_beacon_us.load(std::memory_order_relaxed);
        const bool cam_active = beacon_t > 0 && (now0 - beacon_t) < CAM_BEACON_TIMEOUT_US && !power_mode::is_sleeping();
        const TickType_t wait_ticks = cam_active ? pdMS_TO_TICKS(20) : portMAX_DELAY;

        if (xQueueReceive(received, &message, wait_ticks) == pdTRUE) {
            const Message parsed = parse({message.frame.text, strnlen(message.frame.text, sizeof message.frame.text)});
            const int64_t now_us = esp_timer_get_time();
            if (parsed.kind != Kind::INVALID && accept(message.mac, now_us)) {
                update([&](Link& link, int64_t& heard) {
                    ++link.messages;
                    heard = now_us;
                });
                obey(parsed);
                reply(parsed, message.frame.index, message.mac);
            }
        }

        const int64_t now1 = esp_timer_get_time();
        const int64_t latest_beacon = cam_beacon_us.load(std::memory_order_relaxed);
        if (latest_beacon > 0 && (now1 - latest_beacon) < CAM_BEACON_TIMEOUT_US &&
            (now1 - last_att_tx_us) >= CAM_ATT_PERIOD_US) {
            broadcast_cam_attitude(now1);
            last_att_tx_us = now1;
        }
    }
}

}  // namespace

esp_err_t start() {
    received = xQueueCreate(REMOTE_QUEUE, sizeof(Received));
    if (received == nullptr) {
        return ESP_ERR_NO_MEM;
    }
    ESP_RETURN_ON_ERROR(esp_now_init(), TAG, "ESP-NOW");
    ESP_RETURN_ON_ERROR(esp_now_register_recv_cb(on_receive), TAG, "receive callback");
    ESP_RETURN_ON_ERROR(esp_now_register_send_cb(on_sent), TAG, "send callback");
    ensure_broadcast_peer();
    return xTaskCreatePinnedToCore(task, "remote", REMOTE_STACK_BYTES, nullptr, REMOTE_PRIORITY, nullptr,
                                   REMOTE_CORE) == pdPASS
               ? ESP_OK
               : ESP_ERR_NO_MEM;
}

Link link() {
    Link current;
    int64_t heard = 0;
    update([&](Link& link, int64_t& last) {
        current = link;
        heard = last;
    });
    current.live = current.paired && esp_timer_get_time() - heard < REMOTE_LIVE_US;
    return current;
}

}  // namespace remote
