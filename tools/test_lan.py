"""Unit tests for LAN OTA battery/thermal pre-flight checks and zlib payload preparation."""

import hashlib
import unittest
import zlib

import lan


class TestOtaPreflight(unittest.TestCase):
    SAMPLE_STATUS = """\
state         READY
fault         NONE
battery_v     3.84
battery_pct   41
battery_ma    -68
usb_v         0.00
pmic_c        38.4
imu_c         48.2
cpu_mhz       80
"""

    def test_parse_status_metrics(self) -> None:
        m = lan.parse_status_metrics(self.SAMPLE_STATUS)
        self.assertEqual(m["battery_pct"], 41.0)
        self.assertAlmostEqual(m["pmic_c"], 38.4)
        self.assertAlmostEqual(m["imu_c"], 48.2)
        self.assertEqual(m["cpu_mhz"], 80.0)

    def test_preflight_passes_when_healthy(self) -> None:
        m = lan.parse_status_metrics(self.SAMPLE_STATUS)
        self.assertEqual(lan.check_ota_preflight(m), [])

    def test_preflight_blocks_low_battery(self) -> None:
        blockers = lan.check_ota_preflight({"battery_pct": 29.0, "pmic_c": 37.0, "imu_c": 46.0})
        self.assertEqual(len(blockers), 1)
        self.assertIn("29%", blockers[0])
        self.assertIn("30%", blockers[0])

    def test_preflight_blocks_hot_pmic_and_imu(self) -> None:
        blockers = lan.check_ota_preflight({"battery_pct": 80.0, "pmic_c": 48.0, "imu_c": 60.0})
        self.assertEqual(len(blockers), 2)
        self.assertIn("48.0°C", blockers[0])
        self.assertIn("60.0°C", blockers[1])

    def test_cli_parser_builds_and_parses_force(self) -> None:
        parser = lan.build_parser()
        args = parser.parse_args(["flash", "--force", "--direct"])
        self.assertEqual(args.subcmd, "flash")
        self.assertTrue(args.force)
        self.assertTrue(args.direct)
        self.assertIsNone(args.ip)
        self.assertEqual(args.min_battery, lan.MIN_OTA_BATTERY_PCT)

    def test_prepare_ota_payload_compresses_when_supported(self) -> None:
        raw = (b"BALAX_FIRMWARE_BLOCK_1234567890" * 200)
        wire, hdrs = lan.prepare_ota_payload(raw, {"compressed_ota": True})
        self.assertEqual(hdrs["Content-Type"], "application/x-balax-zlib")
        self.assertEqual(hdrs["X-Raw-Sha256"], hashlib.sha256(raw).hexdigest())
        self.assertLess(len(wire), len(raw))
        self.assertEqual(zlib.decompress(wire), raw)

    def test_prepare_ota_payload_falls_back_to_raw_for_legacy(self) -> None:
        raw = b"BALAX_LEGACY" * 100
        wire, hdrs = lan.prepare_ota_payload(raw, {})
        self.assertEqual(hdrs["Content-Type"], "application/octet-stream")
        self.assertEqual(wire, raw)


if __name__ == "__main__":
    unittest.main()
