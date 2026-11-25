# BalaX: BalaC balancing robot

Firmware for a self-balancing robot: an M5StickC Plus (ESP32-PICO-D4, MPU6886 IMU, AXP192
power chip) on an M5Stack BalaC base. Native ESP-IDF C++, no Arduino. Remote control comes
from a [JoyX](https://github.com/Felipegalind0/JoyX) over ESP-NOW.

The Arduino/PlatformIO firmware is on the `arduino` branch.

## Use

1. Lay the robot flat and power on. It calibrates the gyro (screen: CALIBRATING).
2. Stand it up and hold it still until it has measured upright (LEVELING).
3. READY: press button A (or `ARM` from the remote) to arm. It balances once stood up.
4. After a fall (FALLEN, with an alarm) press A to acknowledge. A again to re-arm.

Power button: short press restarts, long press powers off (motors stop, melody plays).

## Develop

```
tools/bx build | flash | run | monitor | test | preview | cmd <command>...
```

Needs ESP-IDF v6.1.1 at `~/esp/v6.1.1/esp-idf` and the [shared ESP32 tools
repository](https://github.com/esp32-tools/tools) checked out at `~/gh/esp32-tools/tools`.
`tools/bx test` runs the C++ tests (clang,
on the laptop) and the Python ones. `tools/bx preview` renders the status screen to
`host/build/preview.png`. Anything that can be tested without the robot lives in plain C++
headers/sources (`balance`, `remote_protocol`, `status_*`) and is built for the laptop too.

Console commands (`tools/bx cmd status`): `status`, `partitions`, `arm`, `disarm`, `takeoff`, `beep`,
`play`, `reboot`, plus `get`/`set`/`save` for settings (volume, brightness and all 16
balance gains, each with a range).

`balax lan-cmd "partitions"` reads the installed flash table and reports the running
image and OTA slot roles. The remote's Flash tab displays the same robot data.

### Wi-Fi

The robot always runs its own access point and joins the first known network in range.
Credentials go in `secrets/wifi.json` (gitignored, never commit; this repo is public):

```
tools/wifi.py add-current   # this laptop's network (asks for Keychain access)
tools/wifi.py add | remove | list | ap
tools/bx build && tools/bx flash
```

`tools/wifi.py` writes a header into `build/` that the firmware includes.

See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for how it is put together,
[docs/HARDWARE.md](docs/HARDWARE.md) for runtime hardware and protocol facts, and
[docs/silicon/README.md](docs/silicon/README.md) for deep silicon references (M5StickC Plus,
BalaC base, AI-Thinker ESP32-CAM, and OV2640).
