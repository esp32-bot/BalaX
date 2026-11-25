"""Offline unit tests for tools/usb_power.py (never touches real serial hardware)."""

import unittest

import usb_power


class FakeSerial:
    def __init__(self):
        self.port = None
        self.baudrate = None
        self.timeout = None
        self._dtr = False
        self._rts = False
        self.events: list[tuple] = []

    @property
    def dtr(self):
        return self._dtr

    @dtr.setter
    def dtr(self, value: bool):
        self._dtr = bool(value)
        self.events.append(("dtr", self._dtr))

    @property
    def rts(self):
        return self._rts

    @rts.setter
    def rts(self, value: bool):
        self._rts = bool(value)
        self.events.append(("rts", self._rts))

    def open(self):
        self.events.append(("open", self.port, self.baudrate))

    def close(self):
        self.events.append(("close",))


class UsbPowerTests(unittest.TestCase):
    def test_usb_wake_pulses_rts_with_dtr_low_for_normal_boot(self):
        created: list[FakeSerial] = []
        sleeps: list[float] = []

        def factory():
            inst = FakeSerial()
            created.append(inst)
            return inst

        usb_power.usb_wake("/dev/cu.usbserial-fake", serial_cls=factory, sleep_fn=sleeps.append)

        self.assertEqual(len(created), 1)
        events = created[0].events
        self.assertIn(("open", "/dev/cu.usbserial-fake", 115200), events)
        open_idx = events.index(("open", "/dev/cu.usbserial-fake", 115200))
        after_open = events[open_idx + 1 :]
        # Must hold DTR=False, RTS=True (EN=LOW, IO0=HIGH), then release RTS=False, DTR=False (normal boot)
        self.assertEqual(
            after_open,
            [("dtr", False), ("rts", True), ("rts", False), ("dtr", False), ("close",)],
        )
        self.assertEqual(sleeps, [usb_power.RESET_HOLD_S, usb_power.RELEASE_SETTLE_S])

    def test_parser_requires_port_for_wake_and_off(self):
        parser = usb_power.build_parser()
        args = parser.parse_args(["wake", "--port", "/dev/cu.usbserial-test"])
        self.assertEqual(args.action, "wake")
        self.assertEqual(args.port, "/dev/cu.usbserial-test")

        args_off = parser.parse_args(["off", "--port", "/dev/cu.usbserial-test", "--force"])
        self.assertEqual(args_off.action, "off")
        self.assertTrue(args_off.force)


if __name__ == "__main__":
    unittest.main()
