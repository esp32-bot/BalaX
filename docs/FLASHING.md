# Building & Flashing BalaX

## USB flash from the remote

With BalaX connected by USB, open the remote on the computer that has the
USB cable and select **Firmware OTA → USB Flash**. Select the detected port
and the partition layout, then confirm the write. The tab shows progress
and messages from the local USB tool.

The bridge uses images from `./tools/bx build`. For Compact, also run
`./tools/bx build-stub` from the BalaX checkout if the updater image is missing.

Use **Compact** for one main app slot and a factory recovery stub. This writes
the partition table and both firmware images. The settings partition stays at
the same offset. Keep the USB cable connected until the flash completes. The
recovery stub can take about a minute to hand off to the main firmware on
the first boot.

Use the terminal USB commands below when the web app is closed.

## Quickest way (from any terminal folder)

After running `./tools/bx install-cli` once (already installed in `~/.local/bin`), you don't need `cd` or `./`:

```sh
lan-flash                 # builds & flashes balax.local over Wi-Fi
```

You can also pass flags or run other BalaX commands from any folder:

```sh
lan-flash balax.local     # explicit hostname (default)
lan-flash --force         # override battery (<30%) or temperature safety check
balax lan-cmd "probe pmic_c imu_c battery_pct cpu_mhz"
balax test                # run host C++ and Python unit tests
balax flash               # flash over USB cable instead of Wi-Fi
```

> **What does `./tools/bx` mean?**
> - `.` means **the current directory** (just like `..` means the parent directory).
> - `bx` is short for **BalaX** — the repo's wrapper script in `tools/bx` that activates ESP-IDF automatically and runs build/flash/test commands.
> - Once `balax` and `lan-flash` are installed in `~/.local/bin` (which is on your `$PATH`), you can call them directly from anywhere without `./tools/bx`.

---

## Pre-flight Battery & Thermal Safety Check

Before uploading over Wi-Fi, `lan-flash` queries BalaX's live status and refuses to flash if:
- **Battery is below `30%`** (prevents mid-OTA shutdown), or
- **Internal temps are too hot** (`pmic_c ≥ 46 °C` or `imu_c ≥ 58 °C` — see [`docs/HARDWARE.md`](HARDWARE.md) for M5StickC Plus LiPo thermal safety).

To override intentionally (for example, while plugged into USB power), pass `--force` (or `-f`), or customize the battery threshold with `--min-battery <pct>`:

```sh
lan-flash --force
```

---

## What `lan-flash` actually does

1. **`[1/2] Building BalaX firmware...`**
   Compiles `main/` with ESP-IDF (`idf.py build`) using incremental `ccache`, producing `build/balax.bin`. Verbose Ninja/CMake compiler output is saved to `build/bx-build.log` so your terminal stays clean (set `BX_VERBOSE=1` if you want the full compiler stream).
2. **`[2/2] Flashing over Wi-Fi to balax.local...`**
   - Runs the pre-flight battery (`≥ 30%`) and temperature (`pmic_c < 46 °C`, `imu_c < 58 °C`) check.
   - If the `balax-remote` web app dev server is running (`http://127.0.0.1:5180`), `lan-flash` automatically routes the upload through the bridge so it doesn't collide with the 100 Hz telemetry poller's auth nonces.
   - Stops the live telemetry stream briefly (`stream stop`) to free ~24 KB of heap before starting OTA.
   - Authenticates with a one-time challenge nonce + **HMAC-SHA256** signature (key derived from `secrets/wifi.json`).
   - Streams `build/balax.bin` into the **inactive A/B OTA partition** (`ota_0` or `ota_1`), verifies integrity, switches the active boot slot, and reboots BalaX. If anything fails mid-upload, the previous working firmware stays untouched.

---

## Why `balax.local` instead of an IP address?

BalaX advertises `balax.local` over mDNS on your local Wi-Fi network, so you never have to look up or remember its IP address (`balax.local` is the default everywhere). If mDNS is blocked on a router, you can still override it with `lan-flash <ip>` or `BX_IP=<ip>`.

---

## First-time USB flash (new / erased board)

Over-the-air Wi-Fi flashing requires BalaX firmware to already be running on Wi-Fi. For a brand-new or erased M5StickC Plus, connect USB-C and run:

```sh
balax flash
```
