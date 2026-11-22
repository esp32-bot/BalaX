#!/usr/bin/env python3
"""Authenticated LAN OTA (compressed zlib + sector-skip), remote console, and live log streaming for BalaX."""

import argparse
import hashlib
import hmac
import json
import os
import re
import socket
import sys
import time
import urllib.error
import urllib.request
import zlib
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SECRETS_FILE = ROOT / "secrets" / "wifi.json"
DEFAULT_BIN = ROOT / "build" / "balax.bin"
BRIDGE_URL = os.environ.get("BX_BRIDGE_URL", "http://127.0.0.1:5180")
PBKDF2_ITERATIONS = 100_000
KEY_BYTES = 32

# Pre-flight OTA safety thresholds for the sealed M5StickC Plus 120 mAh LiPo.
MIN_OTA_BATTERY_PCT = 30.0
MAX_OTA_PMIC_C = 46.0
MAX_OTA_IMU_C = 58.0


def is_host_or_ip(s: str) -> bool:
    if re.match(r"^\d{1,3}(\.\d{1,3}){3}$", s):
        return True
    if s.endswith(".local"):
        return True
    return False


def derive_lan_key(password: str, salt_hex: str) -> bytes:
    salt = bytes.fromhex(salt_hex)
    return hashlib.pbkdf2_hmac("sha256", password.encode("utf-8"), salt, PBKDF2_ITERATIONS, dklen=KEY_BYTES)


def load_lan_key(password_override: str | None = None) -> bytes:
    if not SECRETS_FILE.exists():
        sys.exit(f"Missing {SECRETS_FILE}; create it with wifi credentials and lan_auth.")
    cfg = json.loads(SECRETS_FILE.read_text())
    auth = cfg.get("lan_auth", {})

    if password_override:
        salt_hex = auth.get("salt_hex") or os.urandom(16).hex()
        return derive_lan_key(password_override, salt_hex)

    if "key_hex" in auth and len(auth["key_hex"]) == KEY_BYTES * 2:
        return bytes.fromhex(auth["key_hex"])

    if "password" in auth:
        salt_hex = auth.get("salt_hex")
        if not salt_hex:
            salt_hex = os.urandom(16).hex()
            auth["salt_hex"] = salt_hex
        key = derive_lan_key(auth["password"], salt_hex)
        auth["iterations"] = PBKDF2_ITERATIONS
        auth["key_hex"] = key.hex()
        cfg["lan_auth"] = auth
        SECRETS_FILE.write_text(json.dumps(cfg, indent=2) + "\n")
        return key

    sys.exit(
        f"No `lan_auth` key found in {SECRETS_FILE}.\n"
        'Add `"lan_auth": {"password": "<your-secret>"}` to secrets/wifi.json and re-flash once over USB.'
    )


def load_saved_ip() -> str | None:
    if SECRETS_FILE.exists():
        cfg = json.loads(SECRETS_FILE.read_text())
        return cfg.get("last_ip")
    return None


def save_last_ip(ip: str) -> None:
    if SECRETS_FILE.exists():
        cfg = json.loads(SECRETS_FILE.read_text())
        if cfg.get("last_ip") != ip:
            cfg["last_ip"] = ip
            SECRETS_FILE.write_text(json.dumps(cfg, indent=2) + "\n")


def resolve_ip(ip_arg: str | None) -> str:
    host = ip_arg or os.environ.get("BX_IP") or "balax.local"
    if host.endswith(".local"):
        try:
            return socket.gethostbyname(host)
        except Exception:
            if not ip_arg and not os.environ.get("BX_IP"):
                saved = load_saved_ip()
                if saved:
                    return saved
            return host
    return host


def get_challenge(ip: str, timeout: float = 5.0) -> dict:
    url = f"http://{ip}/api/challenge"
    with urllib.request.urlopen(url, timeout=timeout) as resp:
        return json.loads(resp.read().decode())


def sign_request(key: bytes, nonce_hex: str, payload: bytes = b"") -> str:
    nonce = bytes.fromhex(nonce_hex)
    h = hmac.new(key, nonce, hashlib.sha256)
    if payload:
        h.update(payload)
    return h.hexdigest()


def prepare_ota_payload(raw_firmware: bytes, challenge: dict, *, allow_compress: bool = True) -> tuple[bytes, dict[str, str]]:
    """Compress firmware with zlib-9 when the robot advertises `compressed_ota: true`."""
    raw_sha = hashlib.sha256(raw_firmware).hexdigest()
    if allow_compress and challenge.get("compressed_ota"):
        comp = zlib.compress(raw_firmware, level=9)
        if len(comp) < len(raw_firmware):
            return comp, {
                "Content-Type": "application/x-balax-zlib",
                "Content-Length": str(len(comp)),
                "X-Raw-Size": str(len(raw_firmware)),
                "X-Raw-Sha256": raw_sha,
            }
    return raw_firmware, {
        "Content-Type": "application/octet-stream",
        "Content-Length": str(len(raw_firmware)),
        "X-Raw-Sha256": raw_sha,
    }


def parse_status_metrics(text: str) -> dict[str, float]:
    """Parse numeric key-value fields from `status` or `probe` output."""
    metrics: dict[str, float] = {}
    for line in text.splitlines():
        parts = line.strip().split()
        if len(parts) < 2:
            continue
        key, raw_val = parts[0], parts[1]
        try:
            metrics[key] = float(raw_val)
        except ValueError:
            continue
    return metrics


def check_ota_preflight(
    metrics: dict[str, float],
    *,
    min_battery_pct: float = MIN_OTA_BATTERY_PCT,
    max_pmic_c: float = MAX_OTA_PMIC_C,
    max_imu_c: float = MAX_OTA_IMU_C,
) -> list[str]:
    """Return safety blocker messages when battery is too low or temps are too hot for OTA."""
    blockers: list[str] = []
    bat = metrics.get("battery_pct")
    if bat is not None and bat < min_battery_pct:
        blockers.append(f"battery is {bat:.0f}% (minimum {min_battery_pct:.0f}% required for safe OTA)")
    pmic = metrics.get("pmic_c")
    if pmic is not None and pmic >= max_pmic_c:
        blockers.append(f"PMIC temp is {pmic:.1f}°C (limit < {max_pmic_c:.0f}°C; let BalaX cool down)")
    imu = metrics.get("imu_c")
    if imu is not None and imu >= max_imu_c:
        blockers.append(f"IMU temp is {imu:.1f}°C (limit < {max_imu_c:.0f}°C; let BalaX cool down)")
    return blockers


def try_bridge_cmd(cmd: str) -> str | None:
    try:
        req = urllib.request.Request(
            f"{BRIDGE_URL}/api/bridge/cmd",
            data=json.dumps({"cmd": cmd}).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urllib.request.urlopen(req, timeout=10) as resp:
            if resp.status == 200:
                return resp.read().decode(errors="replace")
    except Exception:
        return None
    return None


def fetch_robot_cmd_output(ip: str, key: bytes, cmd: str, timeout: float = 6.0) -> str:
    bridged = try_bridge_cmd(cmd)
    if bridged is not None:
        return bridged
    chal = get_challenge(ip, timeout=timeout)
    nonce_hex = chal["nonce"]
    payload = cmd.encode("utf-8")
    sig_hex = sign_request(key, nonce_hex, payload)
    req = urllib.request.Request(
        f"http://{ip}/api/cmd",
        data=payload,
        headers={
            "Content-Type": "text/plain; charset=utf-8",
            "X-Nonce": nonce_hex,
            "X-Signature": sig_hex,
        },
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return resp.read().decode(errors="replace")


def run_ota_preflight(ip: str, key: bytes, *, force: bool, min_battery_pct: float) -> None:
    try:
        raw = fetch_robot_cmd_output(ip, key, "status", timeout=5.0)
    except Exception as exc:
        if force:
            print(f"⚠ Could not read robot status before OTA ({exc}); continuing because --force was set.")
            return
        sys.exit(
            f"✗ Cannot verify robot battery/temperature before flashing ({exc}).\n"
            "  Make sure BalaX is powered on and connected to Wi-Fi, or override with: lan-flash --force"
        )

    metrics = parse_status_metrics(raw)
    summary_parts: list[str] = []
    if "battery_pct" in metrics:
        summary_parts.append(f"battery={metrics['battery_pct']:.0f}%")
    if "battery_ma" in metrics:
        summary_parts.append(f"draw={metrics['battery_ma']:+.0f}mA")
    if "pmic_c" in metrics:
        summary_parts.append(f"pmic={metrics['pmic_c']:.1f}°C")
    if "imu_c" in metrics:
        summary_parts.append(f"imu={metrics['imu_c']:.1f}°C")
    if "usb_v" in metrics and metrics["usb_v"] > 4.0:
        summary_parts.append(f"usb={metrics['usb_v']:.2f}V")
    if summary_parts:
        print("Pre-flight check: " + "  ".join(summary_parts))

    blockers = check_ota_preflight(metrics, min_battery_pct=min_battery_pct)
    if not blockers:
        return

    if force:
        print("⚠ Overriding OTA safety check (--force): " + "; ".join(blockers))
        return

    lines = ["✗ Refusing to flash over Wi-Fi:"]
    for b in blockers:
        lines.append(f"  - {b}")
    lines.append("  Charge / cool down BalaX first, or override with: lan-flash --force")
    sys.exit("\n".join(lines))


def try_bridge_ota(bin_path: Path) -> bool:
    """If the local balax-remote Vite bridge is actively polling the robot, serialize OTA through it."""
    if bin_path.resolve() != DEFAULT_BIN.resolve():
        return False
    try:
        with urllib.request.urlopen(f"{BRIDGE_URL}/api/bridge/status", timeout=0.5) as resp:
            if resp.status != 200:
                return False
    except Exception:
        return False

    print(f"Detected active balax-remote bridge at {BRIDGE_URL}; serializing LAN OTA through bridge...")
    t0 = time.monotonic()
    req = urllib.request.Request(f"{BRIDGE_URL}/api/bridge/ota", data=b"", method="POST")
    try:
        with urllib.request.urlopen(req, timeout=75) as resp:
            raw = resp.read().decode(errors="replace").strip()
            result = json.loads(raw) if raw.startswith("{") else {"raw": raw}
    except urllib.error.HTTPError as e:
        body = e.read().decode(errors="replace")
        sys.exit(f"Bridge LAN OTA failed (HTTP {e.code}): {body}")
    dt = time.monotonic() - t0
    size = bin_path.stat().st_size
    kbps = (size / 1024) / max(dt, 0.01)
    part = result.get("partition", "?")
    extra = ""
    if "sectors_skipped" in result:
        extra = f" [erased={result.get('sectors_erased', 0)}, skipped={result.get('sectors_skipped', 0)}]"
    print(f"✓ LAN OTA verified & flashed via bridge to '{part}' in {dt:.1f}s ({kbps:.0f} KB/s){extra}. Robot is rebooting!")
    return True


def enter_factory_stub(ip: str, key: bytes, chal: dict) -> dict:
    """Reboot robot into compact factory ota_stub and wait for it to return on the LAN."""
    nonce_hex = chal["nonce"]
    sig_hex = sign_request(key, nonce_hex, b"")
    req = urllib.request.Request(
        f"http://{ip}/api/ota/enter",
        data=b"",
        headers={"X-Nonce": nonce_hex, "X-Signature": sig_hex},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=5) as resp:
        resp.read()
    print("Entering factory OTA stub... waiting for BalaX stub on Wi-Fi...")
    time.sleep(1.5)
    deadline = time.monotonic() + 20.0
    while time.monotonic() < deadline:
        try:
            c = get_challenge(ip, timeout=2.0)
            if c.get("is_stub") or c.get("partition") == "factory":
                return c
        except Exception:
            pass
        time.sleep(0.6)
    sys.exit("Timed out waiting for factory OTA stub to come online.")


def lan_flash(
    ip: str,
    key: bytes,
    bin_path: Path,
    *,
    force: bool = False,
    min_battery_pct: float = MIN_OTA_BATTERY_PCT,
    direct: bool = False,
) -> int:
    if not bin_path.exists():
        sys.exit(f"Firmware binary not found at {bin_path}. Run `tools/bx build` first.")
    run_ota_preflight(ip, key, force=force, min_battery_pct=min_battery_pct)
    if not direct and try_bridge_ota(bin_path):
        save_last_ip(ip)
        return 0
    # Free telemetry stream RAM before starting OTA
    try:
        lan_cmd(ip, key, ["stream stop"])
    except Exception:
        pass
    firmware = bin_path.read_bytes()
    chal = get_challenge(ip)
    if chal.get("has_factory") and not chal.get("is_stub") and chal.get("partition") != "factory":
        chal = enter_factory_stub(ip, key, chal)

    wire_payload, extra_hdrs = prepare_ota_payload(firmware, chal)
    nonce_hex = chal["nonce"]
    sig_hex = sign_request(key, nonce_hex, wire_payload)
    is_comp = extra_hdrs["Content-Type"] == "application/x-balax-zlib"
    ratio = f" ({len(wire_payload)*100/len(firmware):.1f}% zlib-9)" if is_comp else " (raw)"

    print(
        f"Connected to {ip} (running {chal.get('partition')} @ {chal.get('version')}).\n"
        f"Uploading {len(wire_payload):,} / {len(firmware):,} bytes{ratio} over LAN..."
    )
    t0 = time.monotonic()
    headers = {
        **extra_hdrs,
        "X-Nonce": nonce_hex,
        "X-Signature": sig_hex,
    }
    req = urllib.request.Request(
        f"http://{ip}/api/ota",
        data=wire_payload,
        headers=headers,
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=75) as resp:
            result = json.loads(resp.read().decode())
    except urllib.error.HTTPError as e:
        body = e.read().decode(errors="replace")
        sys.exit(f"LAN OTA rejected (HTTP {e.code}): {body}")

    dt = time.monotonic() - t0
    kbps = (len(firmware) / 1024) / max(dt, 0.01)
    wear = ""
    if "sectors_skipped" in result:
        wear = (
            f" | flash sectors: {result.get('sectors_erased', 0)} erased, "
            f"{result.get('sectors_skipped', 0)} skipped"
        )
    print(
        f"✓ LAN OTA verified & flashed to '{result.get('partition')}' "
        f"in {dt:.1f}s ({kbps:.0f} KB/s eff){wear}. Robot is rebooting!"
    )
    save_last_ip(ip)
    return 0


def lan_cmd(ip: str, key: bytes, commands: list[str]) -> int:
    for cmd in commands:
        bridged = try_bridge_cmd(cmd)
        if bridged is not None:
            print(bridged, end="" if not bridged or bridged.endswith("\n") else "\n")
            continue
        chal = get_challenge(ip)
        nonce_hex = chal["nonce"]
        payload = cmd.encode("utf-8")
        sig_hex = sign_request(key, nonce_hex, payload)
        req = urllib.request.Request(
            f"http://{ip}/api/cmd",
            data=payload,
            headers={
                "Content-Type": "text/plain; charset=utf-8",
                "X-Nonce": nonce_hex,
                "X-Signature": sig_hex,
            },
            method="POST",
        )
        try:
            with urllib.request.urlopen(req, timeout=10) as resp:
                out = resp.read().decode(errors="replace")
                print(out, end="" if not out or out.endswith("\n") else "\n")
        except urllib.error.HTTPError as e:
            body = e.read().decode(errors="replace")
            sys.exit(f"LAN command '{cmd}' rejected (HTTP {e.code}): {body}")
    save_last_ip(ip)
    return 0


def lan_monitor(ip: str, key: bytes, duration: float | None = None) -> int:
    chal = get_challenge(ip)
    print(
        f"--- LAN Monitor connected to {ip} ({chal.get('partition')} @ {chal.get('version')}) ---\n"
        f"--- Press Ctrl+C to quit ---"
    )
    save_last_ip(ip)
    since = 0
    start = time.monotonic()
    try:
        while True:
            if duration is not None and (time.monotonic() - start) >= duration:
                break
            try:
                c = get_challenge(ip, timeout=3.0)
                nonce_hex = c["nonce"]
                sig_hex = sign_request(key, nonce_hex, b"")
                req = urllib.request.Request(
                    f"http://{ip}/api/logs?since={since}",
                    headers={"X-Nonce": nonce_hex, "X-Signature": sig_hex},
                    method="GET",
                )
                with urllib.request.urlopen(req, timeout=3.0) as resp:
                    seq_hdr = resp.headers.get("X-Log-Seq")
                    if seq_hdr:
                        since = int(seq_hdr)
                    text = resp.read().decode(errors="replace")
                    if text:
                        print(text, end="", flush=True)
            except KeyboardInterrupt:
                raise
            except Exception:
                time.sleep(0.5)
            time.sleep(0.25)
    except KeyboardInterrupt:
        print("\n--- LAN Monitor disconnected ---")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--password", help="override secret password (otherwise uses derived key in secrets/wifi.json)")
    sub = parser.add_subparsers(dest="subcmd", required=True)

    p_flash = sub.add_parser("flash", help="flash firmware over LAN with HMAC-SHA256 challenge-response")
    p_flash.add_argument("ip", nargs="?", help="robot IP or mDNS host (defaults to BX_IP or balax.local)")
    p_flash.add_argument("--bin", type=Path, default=DEFAULT_BIN)
    p_flash.add_argument(
        "-f",
        "--force",
        action="store_true",
        help=f"flash even if battery < {MIN_OTA_BATTERY_PCT:.0f}%% or temps exceed safe limits",
    )
    p_flash.add_argument(
        "--direct",
        action="store_true",
        help="bypass local balax-remote Vite bridge and upload directly to robot",
    )
    p_flash.add_argument(
        "--min-battery",
        type=float,
        default=MIN_OTA_BATTERY_PCT,
        help=f"minimum battery percentage required before OTA (default: {MIN_OTA_BATTERY_PCT:.0f})",
    )

    p_cmd = sub.add_parser("cmd", help="run console commands over LAN")
    p_cmd.add_argument("args", nargs="+", help="[ip|balax.local] followed by one or more console commands")

    p_mon = sub.add_parser("monitor", help="stream live logs over LAN")
    p_mon.add_argument("ip", nargs="?", help="robot IP or mDNS host")
    p_mon.add_argument("--duration", type=float, help="stop after N seconds")
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    key = load_lan_key(args.password)

    if args.subcmd == "flash":
        return lan_flash(
            resolve_ip(args.ip),
            key,
            args.bin,
            force=args.force,
            min_battery_pct=args.min_battery,
            direct=args.direct,
        )
    if args.subcmd == "cmd":
        if is_host_or_ip(args.args[0]):
            if len(args.args) < 2:
                sys.exit("Usage: bx lan-cmd [ip|balax.local] <command>...")
            ip = resolve_ip(args.args[0])
            cmds = args.args[1:]
        else:
            ip = resolve_ip(None)
            cmds = args.args
        return lan_cmd(ip, key, cmds)
    if args.subcmd == "monitor":
        return lan_monitor(resolve_ip(args.ip), key, args.duration)
    return 1


if __name__ == "__main__":
    sys.exit(main())
