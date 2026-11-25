#pragma once

// App-wide constants. Hardware details live in components/m5stickc_plus.

#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"

// Run at 80 MHz (the lowest speed that keeps the 80 MHz APB clock for Wi-Fi, SPI, PWM & I2C)
// to minimize heat inside the sealed M5StickC Plus enclosure and protect the internal LiPo battery.
constexpr int CPU_MIN_MHZ = 80;
constexpr int CPU_MAX_MHZ = 80;
// While in Wi-Fi `sleep` mode (100 Hz loop paused), allow DFS down to 40 MHz XTAL + light sleep.
constexpr int CPU_SLEEP_MIN_MHZ = 40;

// Setting defaults, until a value is saved on the robot: main/Kconfig.projbuild, overridden
// for one robot in sdkconfig.local.
constexpr uint8_t DEFAULT_VOLUME = CONFIG_BALAX_DEFAULT_VOLUME;
constexpr uint8_t DEFAULT_BRIGHTNESS = CONFIG_BALAX_DEFAULT_BRIGHTNESS;

// Console on the USB serial port.
constexpr char CONSOLE_PROMPT[] = "balax>";  // the console adds a space; tools/console.py waits for it
constexpr uint32_t CONSOLE_STACK_BYTES = 4096;
constexpr UBaseType_t CONSOLE_PRIORITY = 2;
constexpr BaseType_t CONSOLE_CORE = 0;  // commands print floats; see "Cores" below

// Cores: the balance loop has core 1 to itself (ROBOT_CORE); everything else stays on core 0
// with Wi-Fi. A task that uses the FPU (floats) gets pinned to the core it first does so on,
// so a task that floats free can end up on core 1 for good: only integer-only helpers float
// free (the mDNS responder in lan_server.cpp, the speaker, the OTA reboot timers). Core 0's
// tick interrupt is also what wakes the balance loop, so nothing may mask interrupts there
// for long (lan_server.cpp's log ring copies with memcpy for that reason).

// Status screen: a diagram of the robot's parts with their readings; the layout is in
// status_layout.h. It checks on the robot every STATUS_POLL_MS and reads everything else
// every STATUS_PERIOD_MS.
constexpr uint32_t STATUS_PERIOD_MS = 1000;
constexpr uint32_t STATUS_POLL_MS = 100;
constexpr uint32_t STATUS_SLEEP_POLL_MS = 2000;  // 0.5 Hz quiet poll while in Wi-Fi sleep
constexpr uint32_t STATUS_STACK_BYTES = 6144;
constexpr UBaseType_t STATUS_PRIORITY = 1;
constexpr BaseType_t STATUS_CORE = 0;  // pinned, as the CPU speed comes from a per-core counter

// Control loop (robot.cpp): every balance::TICK_S, alone on the second core and above every
// other task there.
constexpr uint32_t ROBOT_STACK_BYTES = 6144;
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
constexpr uint16_t WIFI_SLEEP_LISTEN_INTERVAL = 3;  // beacons to sleep across in WIFI_PS_MAX_MODEM (~300 ms)
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
