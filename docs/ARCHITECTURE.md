# Architecture

## Layout
- `components/`: reusable drivers. `m5stickc_plus` (AXP192, MPU6886, display, LED, button,
  speaker, I2C), `balac` (motor base), `gfx` (canvas), `settings` (NVS-backed values with
  ranges), `console_commands`.
- `main/`: the application.
  - `balance.*`, `balance_config.h`: the controller, plain C++ (tested on the laptop).
    `Robot::step(imu, gains)` returns motor power; states CALIBRATING, READY, LEVELING,
    BALANCING, FALLEN with faults NONE/TILT/OVERPOWER/SENSOR.
  - `robot.*`: the 10 ms control loop on core 1 (priority 20, `xTaskDelayUntil`). Each tick:
    requests and button A, stick (500 ms timeout), IMU, balance step, motors (zero during
    shutdown), then LED/alarm, power button, publish telemetry.
  - `remote.*` and `remote_protocol.*`: ESP-NOW. The receive callback only queues; a task on
    core 0 parses, pairs, obeys and replies. The protocol is plain C++ and tested.
  - `wifi.*`: AP plus station with phase tracking (`wifi::Phase`).
  - `status*`: the screen and `status` command. `status_values.h` is the data,
    `status_layout.h` the geometry and colours, `status_view.cpp` the drawing.
  - `gain_settings.*`: all 16 gains as settings, one table, so adding one is one line.
  - `config.h`: every task parameter and constant. `sounds.h`: melodies.
- `host/`: laptop tests and the screen preview. `tools/`: `bx`, `console.py`, `wifi.py`.

## Conventions
- No magic values inline; constants in `config.h` or the owning module's header.
- No duplication; table-driven where there are many similar items.
- IDF builds use `-Wall -Werror` (watch format-truncation and missing-field-initializers);
  host builds use clang `-std=c++23`.
- Scratch files go in gitignored dirs inside the repo (`build/`, `host/build/`), never `/tmp`.
- Never commit or print Wi-Fi credentials: `secrets/`, `sdkconfig.local`, `src/creds.h` and
  the generated header in `build/` stay out of git. Old credentials exist in git history.

## Measured (on the robot)
Control tick 0.8 ms worst case, 0 overruns, each core about 2% loaded, heap 108 of 219 KB
used.

## Open points
See TODO.md. Balancing works but falls after a while, about as well as the Arduino version;
gains are runtime settings (`set k_angle ...`, `save`) so tuning needs no reflash.
