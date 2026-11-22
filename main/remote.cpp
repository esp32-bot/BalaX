#include "remote.h"

#include <algorithm>
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
#include "remote_protocol.h"
#include "robot.h"
#include "status.h"

namespace remote {

namespace {

constexpr char TAG[] = "remote";

struct Received {
    uint8_t mac[ESP_NOW_ETH_ALEN];
    Frame frame;
};

QueueHandle_t received;
portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
// Guarded by `lock`.
Link shared;
int64_t heard_us = 0;

template <typename Change>
void update(Change change) {
    taskENTER_CRITICAL(&lock);
    change(shared, heard_us);
    taskEXIT_CRITICAL(&lock);
}

// In the Wi-Fi task: hands the message over and gets out of its way.
void on_receive(const esp_now_recv_info_t* info, const uint8_t* data, int size) {
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
    peer.channel = 0;         // whichever the radio is on
    peer.ifidx = WIFI_IF_AP;  // the remote sent to the access point, so answer from it
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
    for (;;) {
        xQueueReceive(received, &message, portMAX_DELAY);
        const Message parsed = parse({message.frame.text, strnlen(message.frame.text, sizeof message.frame.text)});
        const int64_t now_us = esp_timer_get_time();
        if (parsed.kind == Kind::INVALID || !accept(message.mac, now_us)) {
            continue;
        }
        update([&](Link& link, int64_t& heard) {
            ++link.messages;
            heard = now_us;
        });
        obey(parsed);
        reply(parsed, message.frame.index, message.mac);
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
