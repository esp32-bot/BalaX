#include "wifi.h"

#include <algorithm>
#include <atomic>
#include <cstring>
#include <iterator>
#include <string_view>

#include "config.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "wifi_networks.h"  // generated from secrets/wifi.json; see tools/wifi.py

namespace wifi {

namespace {

constexpr char TAG[] = "wifi";
constexpr uint64_t US_PER_MS = 1000;
// 52 quarter-dBm = +13.0 dBm (20 mW vs 100 mW default): cuts RF TX spike heat ~40% with full indoor range.
constexpr int8_t WIFI_MAX_TX_QDBM = 52;

bool (*may_scan)() = nullptr;
esp_timer_handle_t retry_timer = nullptr;
uint32_t retry_ms = WIFI_RETRY_MIN_MS;  // only touched from the event and timer tasks, one at a time
char ap_ssid[sizeof Status::ssid];
std::atomic<bool> sleep_ps_enabled{false};

portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
Status shared;  // guarded by `lock`

wifi_ps_type_t sta_ps_mode() {
    return sleep_ps_enabled.load(std::memory_order_acquire) ? WIFI_PS_MAX_MODEM : WIFI_PS_MIN_MODEM;
}

template <size_t N>
size_t copy(uint8_t (&out)[N], std::string_view text) {
    const size_t size = std::min(text.size(), N);
    memcpy(out, text.data(), size);
    return size;
}

template <typename Change>
void update(Change change) {
    taskENTER_CRITICAL(&lock);
    change(shared);
    taskEXIT_CRITICAL(&lock);
}

void enter(Phase phase) {
    update([=](Status& s) { s.phase = phase; });
}

esp_err_t start_access_point() {
    wifi_config_t config = {};
    config.ap.ssid_len = copy(config.ap.ssid, WIFI_ACCESS_POINT.ssid);
    copy(config.ap.password, WIFI_ACCESS_POINT.password);
    config.ap.channel = WIFI_AP_CHANNEL;
    config.ap.authmode = WIFI_ACCESS_POINT.password.empty() ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;
    config.ap.max_connection = WIFI_AP_MAX_CLIENTS;
    memcpy(ap_ssid, config.ap.ssid, config.ap.ssid_len);
    return esp_wifi_set_config(WIFI_IF_AP, &config);
}

void set_access_point_enabled(bool enabled) {
    wifi_mode_t current_mode = WIFI_MODE_NULL;
    if (esp_wifi_get_mode(&current_mode) != ESP_OK) {
        return;
    }
    const wifi_mode_t target_mode = enabled ? WIFI_MODE_APSTA : WIFI_MODE_STA;
    if (current_mode != target_mode) {
        if (esp_wifi_set_mode(target_mode) == ESP_OK) {
            if (enabled) {
                start_access_point();
            }
            // In pure STA mode, modem-sleep drops idle Wi-Fi radio current by ~45-55 mA (~200 mW less heat);
            // softAP requires WIFI_PS_NONE so beacons stay on schedule.
            esp_wifi_set_ps(enabled ? WIFI_PS_NONE : sta_ps_mode());
            ESP_LOGI(TAG, "softAP %s", enabled ? "enabled (no Wi-Fi)" : "disabled (connected to Wi-Fi, modem-sleep on)");
        }
    } else {
        esp_wifi_set_ps(enabled ? WIFI_PS_NONE : sta_ps_mode());
    }
    update([=](Status& s) {
        s.ap_ssid = enabled ? ap_ssid : "off";
        if (!enabled) {
            s.ap_clients = 0;
        }
    });
}

void scan() {
    if (may_scan != nullptr && !may_scan()) {
        esp_timer_start_once(retry_timer, WIFI_HOLD_MS * US_PER_MS);
        return;
    }
    enter(Phase::SEARCHING);
    if (esp_wifi_scan_start(nullptr, false) != ESP_OK) {
        esp_timer_start_once(retry_timer, WIFI_HOLD_MS * US_PER_MS);
    }
}

void retry_later() {
    esp_timer_start_once(retry_timer, retry_ms * US_PER_MS);
    retry_ms = std::min(retry_ms * 2, WIFI_RETRY_MAX_MS);
}

// Joins the first known network in the scan, at its strongest access point.
void connect_to_best() {
    size_t best = WIFI_NETWORKS.size();
    wifi_ap_record_t chosen{};
    wifi_ap_record_t record;
    while (esp_wifi_scan_get_ap_record(&record) == ESP_OK) {
        const std::string_view ssid(reinterpret_cast<const char*>(record.ssid));
        const auto known = std::find_if(WIFI_NETWORKS.begin(), WIFI_NETWORKS.end(),
                                        [&](const WifiCredentials& n) { return n.ssid == ssid; });
        const size_t index = known - WIFI_NETWORKS.begin();
        if (index < best || (index == best && index < WIFI_NETWORKS.size() && record.rssi > chosen.rssi)) {
            best = index;
            chosen = record;
        }
    }
    esp_wifi_clear_ap_list();
    if (best == WIFI_NETWORKS.size()) {
        enter(Phase::NOT_FOUND);
        set_access_point_enabled(true);
        retry_later();
        return;
    }
    const WifiCredentials& network = WIFI_NETWORKS[best];
    wifi_config_t config = {};
    copy(config.sta.ssid, network.ssid);
    copy(config.sta.password, network.password);
    // Straight to the access point the scan found, without scanning again.
    config.sta.bssid_set = true;
    memcpy(config.sta.bssid, chosen.bssid, sizeof config.sta.bssid);
    config.sta.channel = chosen.primary;
    config.sta.listen_interval = WIFI_SLEEP_LISTEN_INTERVAL;
    config.sta.threshold.authmode = network.password.empty() ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;
    config.sta.pmf_cfg.capable = true;
    enter(Phase::JOINING);
    if (esp_wifi_set_config(WIFI_IF_STA, &config) != ESP_OK || esp_wifi_connect() != ESP_OK) {
        enter(Phase::DISCONNECTED);
        set_access_point_enabled(true);
        retry_later();
    }
}

bool password_refused(uint8_t reason) {
    switch (reason) {
        case WIFI_REASON_AUTH_FAIL:
        case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
        case WIFI_REASON_HANDSHAKE_TIMEOUT:
            return true;
        default:
            return false;
    }
}

void on_wifi_event(void*, esp_event_base_t, int32_t id, void* data) {
    switch (id) {
        case WIFI_EVENT_STA_START:
            if (!WIFI_NETWORKS.empty()) {
                scan();
            } else {
                set_access_point_enabled(true);
            }
            break;
        case WIFI_EVENT_SCAN_DONE:
            connect_to_best();
            break;
        case WIFI_EVENT_STA_CONNECTED: {
            const auto* event = static_cast<const wifi_event_sta_connected_t*>(data);
            update([&](Status& s) {
                const size_t size = std::min<size_t>(event->ssid_len, sizeof s.ssid - 1);
                memcpy(s.ssid, event->ssid, size);
                s.ssid[size] = '\0';
            });
            break;
        }
        case WIFI_EVENT_STA_DISCONNECTED: {
            const auto* event = static_cast<const wifi_event_sta_disconnected_t*>(data);
            const Phase phase = password_refused(event->reason) ? Phase::REJECTED : Phase::DISCONNECTED;
            update([=](Status& s) {
                s.phase = phase;
                s.ip = 0;
            });
            set_access_point_enabled(true);
            ESP_LOGI(TAG, "%s (reason %d)", describe(phase), event->reason);
            retry_later();
            break;
        }
        case WIFI_EVENT_AP_STACONNECTED:
            update([](Status& s) { ++s.ap_clients; });
            break;
        case WIFI_EVENT_AP_STADISCONNECTED:
            update([](Status& s) { s.ap_clients -= s.ap_clients > 0; });
            break;
        default:
            break;
    }
}

void on_ip_event(void*, esp_event_base_t, int32_t id, void* data) {
    if (id == IP_EVENT_STA_GOT_IP) {
        const auto* event = static_cast<const ip_event_got_ip_t*>(data);
        retry_ms = WIFI_RETRY_MIN_MS;
        update([&](Status& s) {
            s.phase = Phase::CONNECTED;
            s.ip = event->ip_info.ip.addr;
        });
        // Turn off the softAP & enable Wi-Fi modem-sleep when connected to home Wi-Fi.
        set_access_point_enabled(false);
        ESP_LOGI(TAG, "on %s as " IPSTR, status().ssid, IP2STR(&event->ip_info.ip));
    }
}

}  // namespace

esp_err_t start(bool (*may_scan_now)()) {
    may_scan = may_scan_now;
    const esp_timer_create_args_t retry = {
        .callback = [](void*) { scan(); },
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "wifi retry",
        .skip_unhandled_events = true,
    };
    ESP_RETURN_ON_ERROR(esp_timer_create(&retry, &retry_timer), TAG, "timer");
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "netif");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "event loop");
    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    init.nvs_enable = false;  // set up afresh at every boot, from the firmware
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init), TAG, "init");
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG, "storage");
    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_wifi_event, nullptr, nullptr),
                        TAG, "Wi-Fi events");
    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_ip_event, nullptr, nullptr),
                        TAG, "IP events");
    const bool start_with_ap = WIFI_NETWORKS.empty();
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(start_with_ap ? WIFI_MODE_APSTA : WIFI_MODE_STA), TAG, "mode");
    if (start_with_ap) {
        ESP_RETURN_ON_ERROR(start_access_point(), TAG, "access point");
    } else {
        const size_t size = std::min(WIFI_ACCESS_POINT.ssid.size(), sizeof ap_ssid - 1);
        memcpy(ap_ssid, WIFI_ACCESS_POINT.ssid.data(), size);
        ap_ssid[size] = '\0';
    }
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "start");
    ESP_RETURN_ON_ERROR(esp_wifi_set_max_tx_power(WIFI_MAX_TX_QDBM), TAG, "tx power");
    ESP_RETURN_ON_ERROR(esp_wifi_set_ps(start_with_ap ? WIFI_PS_NONE : sta_ps_mode()), TAG, "power save");
    update([=](Status& s) {
        s.ap_ssid = start_with_ap ? ap_ssid : "off";
        if (!WIFI_NETWORKS.empty()) {
            s.phase = Phase::SEARCHING;
        }
    });
    return ESP_OK;
}

void set_sleep_mode(bool sleep) {
    sleep_ps_enabled.store(sleep, std::memory_order_release);
    wifi_mode_t mode = WIFI_MODE_NULL;
    if (esp_wifi_get_mode(&mode) == ESP_OK && mode == WIFI_MODE_STA) {
        esp_wifi_set_ps(sta_ps_mode());
    }
}

Status status() {
    Status s;
    update([&](Status& current) { s = current; });
    wifi_second_chan_t secondary;
    esp_wifi_get_channel(&s.channel, &secondary);
    int rssi = 0;
    if (s.connected() && esp_wifi_sta_get_rssi(&rssi) == ESP_OK) {
        s.rssi = rssi;
    }
    return s;
}

uint32_t ip() {
    uint32_t address = 0;
    update([&](Status& current) { address = current.ip; });
    return address;
}

}  // namespace wifi
