#pragma once

// App-wide constants. Hardware details live in components/m5stickc_plus.

#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"

// The CPU idles at 80 MHz, the lowest speed that keeps the 80 MHz APB clock (and with it
// the PWM and bus clocks) steady, and speeds up while a task holds a power-management lock.
constexpr int CPU_MIN_MHZ = 80;
constexpr int CPU_MAX_MHZ = 240;

// Setting defaults, until a value is saved on the robot: main/Kconfig.projbuild, overridden
// for one robot in sdkconfig.local.
constexpr uint8_t DEFAULT_VOLUME = CONFIG_BALAX_DEFAULT_VOLUME;
constexpr uint8_t DEFAULT_BRIGHTNESS = CONFIG_BALAX_DEFAULT_BRIGHTNESS;

// Console on the USB serial port.
constexpr char CONSOLE_PROMPT[] = "balax>";  // the console adds a space; tools/console.py waits for it
constexpr uint32_t CONSOLE_STACK_BYTES = 4096;
constexpr UBaseType_t CONSOLE_PRIORITY = 2;

// Status screen: a diagram of the robot's parts with their readings; the layout is in
// status_layout.h. It checks on the robot every STATUS_POLL_MS and reads everything else
// every STATUS_PERIOD_MS.
constexpr uint32_t STATUS_PERIOD_MS = 1000;
constexpr uint32_t STATUS_POLL_MS = 100;
constexpr uint32_t STATUS_STACK_BYTES = 6144;
constexpr UBaseType_t STATUS_PRIORITY = 1;
constexpr BaseType_t STATUS_CORE = 0;  // pinned, as the CPU speed comes from a per-core counter

// Control loop (robot.cpp): every balance::TICK_S, alone on the second core and above every
// other task there.
constexpr uint32_t ROBOT_STACK_BYTES = 4096;
constexpr UBaseType_t ROBOT_PRIORITY = 20;
constexpr BaseType_t ROBOT_CORE = 1;
constexpr UBaseType_t ROBOT_REQUEST_QUEUE = 4;
constexpr int POWER_BUTTON_POLL_TICKS = 10;
constexpr int SENSOR_FAILURE_TICKS = 3;       // failed IMU reads in a row that stop the robot
constexpr int64_t STEER_TIMEOUT_US = 500000;  // the stick goes back to the middle when the remote goes quiet
constexpr int ALARM_PERIOD_TICKS = 55;        // after a fall, beeps this often until acknowledged

// Wi-Fi (wifi.cpp). Channel 1 is where JoyX remotes without a network of their own are.
constexpr uint8_t WIFI_AP_CHANNEL = 1;
constexpr uint8_t WIFI_AP_MAX_CLIENTS = 4;
constexpr uint32_t WIFI_RETRY_MIN_MS = 2000;  // after a disconnect, or no known network in range;
constexpr uint32_t WIFI_RETRY_MAX_MS = 60000; // doubling each time up to this
constexpr uint32_t WIFI_HOLD_MS = 5000;       // how often to ask again while scanning must wait

// JoyX remotes over ESP-NOW (remote.cpp): replies go out as soon as a message comes in.
constexpr uint32_t REMOTE_STACK_BYTES = 4096;
constexpr UBaseType_t REMOTE_PRIORITY = 15;
constexpr BaseType_t REMOTE_CORE = 0;           // with the Wi-Fi task
constexpr UBaseType_t REMOTE_QUEUE = 4;
constexpr int64_t REMOTE_LIVE_US = STEER_TIMEOUT_US;  // heard from this recently, the link is live
constexpr int64_t REMOTE_TAKEOVER_US = 5000000;  // another remote may pair once the paired one is this quiet
