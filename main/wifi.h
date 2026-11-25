#pragma once

// Wi-Fi: the robot's own access point, always on, and a connection to the first known network
// in range (tools/wifi.py manages both). The access point is how JoyX remotes find the robot
// (remote.h), and how to reach it when no known network is in range.
//
// The radio has one channel for both. The access point starts on WIFI_AP_CHANNEL, where
// remotes without a network of their own look for it, and follows the network's channel once
// connected. Looking for networks hops channels for a moment, which a remote would notice, so
// it waits while `may_scan` says no.

#include <cstdint>

#include "esp_err.h"

namespace wifi {

esp_err_t start(bool (*may_scan)());

// Switches pure-STA Wi-Fi power save between WIFI_PS_MAX_MODEM (sleep) and WIFI_PS_MIN_MODEM (awake).
void set_sleep_mode(bool sleep);

enum class Phase : uint8_t {
    NO_NETWORKS,  // none set up
    SEARCHING,
    NOT_FOUND,    // none of them in range
    JOINING,
    REJECTED,     // the network turned the password down
    DISCONNECTED,
    CONNECTED,
};

// For the screen and the console.
constexpr const char* describe(Phase phase) {
    switch (phase) {
        case Phase::NO_NETWORKS: return "none set up";
        case Phase::SEARCHING: return "searching";
        case Phase::NOT_FOUND: return "none in range";
        case Phase::JOINING: return "joining";
        case Phase::REJECTED: return "wrong password?";
        case Phase::DISCONNECTED: return "disconnected";
        case Phase::CONNECTED: return "connected";
    }
    return "?";
}

struct Status {
    Phase phase = Phase::NO_NETWORKS;
    char ssid[33] = "";          // the network, once connected
    uint32_t ip = 0;             // network byte order, as esp_ip4_addr_t
    int8_t rssi = 0;             // dBm
    uint8_t channel = 0;
    const char* ap_ssid = "";
    uint8_t ap_clients = 0;      // phones and laptops on the access point (remotes don't join it)

    bool connected() const { return phase == Phase::CONNECTED; }
};
Status status();

// Just the address (network byte order, 0 until connected): no radio calls, so it is cheap
// enough to ask for every mDNS question.
uint32_t ip();

}  // namespace wifi
