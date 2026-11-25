#!/usr/bin/env python3
"""USB power control for BalaX on M5StickC Plus.

- `wake`: pulses the ESP32-PICO-D4 auto-reset EN line (`DTR=False, RTS=True` -> `DTR=False, RTS=False`)
  to reboot BalaX out of USB-charge deep sleep (`off` mode) into normal firmware without touching
  the physical buttons. Note: this only wakes deep sleep (`off` mode where AXP192 DC-DC1 stays on);
  if AXP192 DC-DC1 was hard-shut-off via the 4 s power button, a physical button press is required.
- `off`: sends the `off` (or `off force`) console command over USB serial without resetting first,
  putting BalaX into ~1-2 mA USB-charge deep sleep so it charges cool without overheating.
"""

import argparse
import errno
import sys
import time
from pathlib import Path

import serial

TOOLS_DIR = Path(__file__).resolve().parent
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

import console as bx_console  # noqa: E402

RESET_HOLD_S = 0.12
RELEASE_SETTLE_S = 0.05


def usb_wake(port: str, baud: int = 115200, serial_cls=serial.Serial, sleep_fn=time.sleep) -> None:
    """Reboot BalaX over USB by asserting EN LOW (RTS=True, DTR=False) then releasing both."""
    link = serial_cls()
    link.port = port
    link.baudrate = baud
    link.timeout = 0.1
    link.dtr = False
    link.rts = True
    link.open()
    try:
        try:
            # Classic ESP32 2-transistor auto-reset circuit:
            # DTR=False, RTS=True  -> EN=LOW,  IO0=HIGH (hardware reset)
            # DTR=False, RTS=False -> EN=HIGH, IO0=HIGH (normal flash boot)
            link.dtr = False
            link.rts = True
            sleep_fn(RESET_HOLD_S)
            link.rts = False
            link.dtr = False
            sleep_fn(RELEASE_SETTLE_S)
        except OSError as error:
            if error.errno not in (errno.EINVAL, errno.ENOTTY):
                raise
    finally:
        link.close()


def usb_off(port: str, force: bool = False) -> int:
    """Send `off` (or `off force`) to BalaX over USB console without resetting it first."""
    prompt = bx_console.firmware_value(bx_console.CONFIG_H, r'CONSOLE_PROMPT\[\] = "([^"]*)"').encode() + b" "
    baud = int(bx_console.firmware_value(bx_console.SDKCONFIG, r"^CONFIG_ESP_CONSOLE_UART_BAUDRATE=(\d+)$"))
    command = "usb-sleep force" if force else "usb-sleep"
    with bx_console.Console(port, baud, prompt) as con:
        if not con.sync():
            print("The robot's USB console didn't answer. Is BalaX awake?", file=sys.stderr)
            return 1
        output = con.run(command)
        if output is None:
            print(f"No prompt after '{command}'", file=sys.stderr)
            return 1
        text = output.replace(b"\r\n", b"\n").decode(errors="replace")
        print(text, end="" if not text or text.endswith("\n") else "\n")
        return 1 if any(marker in output for marker in bx_console.FAILURE_MARKERS) else 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="action", required=True)

    p_wake = sub.add_parser("wake", help="pulse USB RTS->EN reset to reboot BalaX out of 'off' deep sleep")
    p_wake.add_argument("--port", required=True, help="USB serial port (e.g. /dev/cu.usbserial-...)")

    p_off = sub.add_parser("usb-sleep", aliases=["off"], help="send 'off' command over USB serial console for cool USB charging")
    p_off.add_argument("--port", required=True, help="USB serial port (e.g. /dev/cu.usbserial-...)")
    p_off.add_argument("--force", action="store_true", help="pass 'off force' even if checks would refuse")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.action == "wake":
        usb_wake(args.port)
        print(f"Pulsed USB reset on {args.port}; BalaX is booting.")
        return 0
    if args.action in ("usb-sleep", "off"):
        return usb_off(args.port, force=args.force)
    return 1


if __name__ == "__main__":
    sys.exit(main())
