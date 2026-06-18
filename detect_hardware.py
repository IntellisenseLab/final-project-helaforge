#!/usr/bin/env python3
"""
detect_hardware.py
==================
Auto-detects Kobuki serial port, LD19 LiDAR serial port, and Kinect device
index by reading USB chip IDs directly from the kernel — works with ANY unit
of each hardware type, regardless of serial number.

Usage (standalone):
    python3 detect_hardware.py

Usage (from shell to get env vars):
    eval $(python3 detect_hardware.py --export)

Returns exit code 0 on success, 1 if a required device is missing.
"""

import argparse
import os
import subprocess
import sys

# ── USB chip identity table ──────────────────────────────────────────────────
# Each entry: (vendor_id, product_id, product_name_fragment_or_None)
KOBUKI_IDS = [
    ("0403", "6001", None),        # FTDI FT232RL  — standard iClebo Kobuki
    ("10c4", "ea60", "obuki"),     # CP2102 variant — product string has "Kobuki"
    ("1a86", "7523", "obuki"),     # CH340 variant
]

LIDAR_IDS = [
    ("10c4", "ea60", None),        # CP2102 — LD19 default chip
    ("1a86", "7523", None),        # CH340  — alternative
    ("067b", "2303", None),        # PL2303 — rare variant
]

KINECT_VID = "045e"
KINECT_PIDS = {"02ad", "02ae", "02bf", "02b0", "02c2"}  # all Kinect v1 PIDs


def udevadm_attrs(dev: str) -> dict:
    """Return a dict of ATTRS from `udevadm info --attribute-walk` for dev."""
    try:
        out = subprocess.check_output(
            ["udevadm", "info", "--name", dev, "--attribute-walk"],
            stderr=subprocess.DEVNULL,
            text=True,
        )
    except (subprocess.CalledProcessError, FileNotFoundError):
        return {}

    attrs: dict = {}
    for line in out.splitlines():
        line = line.strip()
        if line.startswith("ATTRS{"):
            key_end = line.index("}")
            key = line[6:key_end]
            val = line[key_end + 2:].strip().strip('"')
            # Only keep the first occurrence (closest device in walk)
            if key not in attrs:
                attrs[key] = val
    return attrs


def lsusb_devices() -> list:
    """Return list of (bus, device, vid, pid, description) from lsusb."""
    try:
        out = subprocess.check_output(
            ["lsusb"], stderr=subprocess.DEVNULL, text=True
        )
    except (subprocess.CalledProcessError, FileNotFoundError):
        return []

    devices = []
    for line in out.splitlines():
        # Bus 002 Device 003: ID 045e:02c2 Microsoft Corp. Kinect ...
        parts = line.split()
        if len(parts) >= 6 and parts[3] == "ID":
            vid_pid = parts[4].split(":")
            if len(vid_pid) == 2:
                devices.append({
                    "vid": vid_pid[0].lower(),
                    "pid": vid_pid[1].lower(),
                    "desc": " ".join(parts[5:]),
                })
    return devices


def find_serial_port(role: str, id_table: list, exclude_roles: list = None) -> str:
    """
    Scan /dev/ttyUSB* and return the first port whose USB attributes match
    any entry in id_table.  If exclude_roles is given, skip ports already
    assigned to those roles in the global _assignments dict.
    """
    exclude_devs = set()
    if exclude_roles:
        for r in exclude_roles:
            v = _assignments.get(r)
            if v:
                exclude_devs.add(v)

    tty_devs = sorted(
        d for d in os.listdir("/dev") if d.startswith("ttyUSB") or d.startswith("ttyACM")
    )

    for tty in tty_devs:
        dev = f"/dev/{tty}"
        if dev in exclude_devs:
            continue

        attrs = udevadm_attrs(dev)
        vid = attrs.get("idVendor", "").lower()
        pid = attrs.get("idProduct", "").lower()
        product = attrs.get("product", "").lower()
        manufacturer = attrs.get("manufacturer", "").lower()

        for (tv, tp, tprod) in id_table:
            if vid != tv.lower() or pid != tp.lower():
                continue
            if tprod is not None:
                if tprod.lower() not in product and tprod.lower() not in manufacturer:
                    continue
            return dev

    return ""


def count_kinects() -> int:
    """Count connected Kinect v1 devices via libfreenect."""
    try:
        result = subprocess.check_output(
            [
                "python3", "-c",
                "import freenect; ctx=freenect.init(); "
                "n=freenect.num_devices(ctx); "
                "freenect.shutdown(ctx); print(n)"
            ],
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=5,
        )
        return int(result.strip())
    except Exception:
        return 0


def kinect_index_from_lsusb() -> int:
    """
    Fallback: count Kinect cameras seen in lsusb.
    Returns 0 if at least one is present, -1 if none.
    """
    devs = lsusb_devices()
    cams = [d for d in devs if d["vid"] == KINECT_VID and d["pid"] in KINECT_PIDS]
    return 0 if cams else -1


# Global role→device mapping (populated by detect())
_assignments: dict = {}


def detect(verbose: bool = True) -> dict:
    """Run full detection. Returns dict with keys: kobuki, lidar, kinect_index."""
    result = {
        "kobuki": "",
        "lidar": "",
        "kinect_index": -1,
        "kinect_count": 0,
    }

    # ── Kobuki ────────────────────────────────────────────────────────────────
    kobuki_port = find_serial_port("kobuki", KOBUKI_IDS)
    if not kobuki_port:
        # Fallback: try /dev/kobuki symlink (from udev rule)
        if os.path.exists("/dev/kobuki"):
            kobuki_port = "/dev/kobuki"
    _assignments["kobuki"] = kobuki_port
    result["kobuki"] = kobuki_port

    # ── LD19 LiDAR ────────────────────────────────────────────────────────────
    lidar_port = find_serial_port("lidar", LIDAR_IDS, exclude_roles=["kobuki"])
    if not lidar_port:
        if os.path.exists("/dev/ld19"):
            lidar_port = "/dev/ld19"
    _assignments["lidar"] = lidar_port
    result["lidar"] = lidar_port

    # ── Kinect ────────────────────────────────────────────────────────────────
    n_kinects = count_kinects()
    if n_kinects == 0:
        # Try lsusb fallback
        idx = kinect_index_from_lsusb()
        result["kinect_count"] = 1 if idx == 0 else 0
        result["kinect_index"] = idx
    else:
        result["kinect_count"] = n_kinects
        result["kinect_index"] = 0  # always use first found

    if verbose:
        print("=" * 52)
        print("  Hardware Auto-Detection")
        print("=" * 52)
        kp = result["kobuki"] or "NOT FOUND ❌"
        lp = result["lidar"] or "NOT FOUND ❌"
        ki = result["kinect_index"]
        kc = result["kinect_count"]
        print(f"  Kobuki  : {kp}")
        print(f"  LD19    : {lp}")
        print(f"  Kinect  : {'device index ' + str(ki) + ' ✅' if ki >= 0 else 'NOT FOUND ❌'} ({kc} found)")
        print("=" * 52)

    return result


def main():
    parser = argparse.ArgumentParser(description="Auto-detect robot hardware")
    parser.add_argument("--export", action="store_true",
                        help="Print shell export statements (for eval)")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args()

    result = detect(verbose=not args.quiet and not args.export)

    missing = []
    if not result["kobuki"]:
        missing.append("Kobuki")
    if not result["lidar"]:
        missing.append("LD19 LiDAR")
    if result["kinect_index"] < 0:
        missing.append("Kinect")

    if args.export:
        # Print shell-sourceable lines
        print(f"export ROBOT_KOBUKI_PORT='{result['kobuki']}'")
        print(f"export ROBOT_LIDAR_PORT='{result['lidar']}'")
        print(f"export ROBOT_KINECT_INDEX='{max(0, result['kinect_index'])}'")
        print(f"export ROBOT_KINECT_COUNT='{result['kinect_count']}'")
        if missing:
            print(f"# WARNING: Not found: {', '.join(missing)}")
        return 0

    if missing:
        print(f"\n⚠️  Missing hardware: {', '.join(missing)}", file=sys.stderr)
        print("   Plug in all devices and re-run.", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
