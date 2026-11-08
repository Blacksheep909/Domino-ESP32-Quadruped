"""Drive the simulated Domino with the real radio (or the keyboard).

The Radiomaster Boxer in EdgeTX "USB Joystick" mode shows up as a HID gamepad
whose 8 axes are CRSF CH1-CH8. The report decoding is a port of Studio's
`boxer-hid.mjs`, and the stick-to-command mapping copies the firmware's
`readPolicyCommand()` in src/main.cpp, so the sim responds like the robot:

    CH2 right stick vertical    forward / reverse
    CH1 right stick horizontal  sideways
    CH4 left stick horizontal   turn (right = clockwise)
    CH3 left stick vertical     ride height 220-280 mm
    CH6 SB fully down           policy on (otherwise it holds the neutral stand)

Keyboard fallback (Isaac Sim window focused): W/S forward, A/D sideways,
Q/E turn, R/F ride height, P toggles the policy, Backspace resets the robot.
"""

from __future__ import annotations

import math
import re
import struct
import threading
import time

BOXER_VENDOR_ID = 0x1209
BOXER_PRODUCT_ID = 0x4F54
TRANSMITTER_PATTERN = re.compile(r"radiomaster|edge\s*tx|open\s*tx|tx16s|boxer|zorro|pocket|jumper|t-?lite|t-?pro", re.I)
AXIS_COUNT = 8
AXIS_MAX = 2048
REPORT_BYTES_WITHOUT_BATTERY = 19

# Firmware constants (src/main.cpp: kPolicyMax*, kGaitStickDeadband, crsf.h SB).
MAX_FORWARD = 0.50
MAX_REVERSE = 0.30
MAX_LATERAL = 0.25
MAX_YAW = 1.50
STICK_DEADBAND = 0.12
RIDE_MIN_MM = 220.0
RIDE_MAX_MM = 280.0
SB_POLICY_ON_US = 1750
SB_POLICY_OFF_US = 1650


def _candidate(report: bytes, offset: int, layout: str):
    if len(report) < offset + REPORT_BYTES_WITHOUT_BATTERY:
        return None
    axes_off = offset + 3 if layout == "buttons-first" else offset
    if len(report) < axes_off + AXIS_COUNT * 2:
        return None
    axes = list(struct.unpack_from("<8H", report, axes_off))
    valid = sum(0 <= a <= AXIS_MAX for a in axes)
    return valid * 100 + (1 if layout == "buttons-first" else 0), valid, axes


def decode_report(report) -> list[int] | None:
    """HID report -> CH1..CH8 in microseconds (1000-2000), or None."""
    report = bytes(report)
    best = None
    for offset in (0, 1):
        for layout in ("buttons-first", "axes-first"):
            c = _candidate(report, offset, layout)
            if c and (best is None or c[0] > best[0]):
                best = c
    if best is None or best[1] != AXIS_COUNT:
        return None
    # floor(x + 0.5) matches JavaScript Math.round in boxer-hid.mjs.
    return [max(1000, min(2000, math.floor(1000 + a / AXIS_MAX * 1000 + 0.5))) for a in best[2]]


def _norm(us: float) -> float:
    return max(-1.0, min(1.0, (us - 1500.0) / 500.0))


def _deadband(value: float) -> float:
    mag = abs(value)
    if mag <= STICK_DEADBAND:
        return 0.0
    scaled = min(1.0, (mag - STICK_DEADBAND) / (1.0 - STICK_DEADBAND))
    return scaled if value > 0 else -scaled


def channels_to_command(ch: list[int], policy_was_on: bool) -> tuple[float, float, float, float, bool]:
    """Same mapping as the firmware. Returns vx, vy, wz, height_m, policy_on."""
    forward = _deadband(_norm(ch[1]))
    lateral = _deadband(_norm(ch[0]))
    turn = _deadband(_norm(ch[3]))
    vx = forward * (MAX_FORWARD if forward >= 0 else MAX_REVERSE)
    vy = -lateral * MAX_LATERAL
    wz = -turn * MAX_YAW
    h = (max(1000, min(2000, ch[2])) - 1000) / 1000.0
    height_mm = RIDE_MIN_MM + h * (RIDE_MAX_MM - RIDE_MIN_MM)
    policy_on = ch[5] >= (SB_POLICY_OFF_US if policy_was_on else SB_POLICY_ON_US)
    return vx, vy, wz, height_mm / 1000.0, policy_on


class BoxerReader:
    """Background HID reader. `channels` is None until a transmitter reports."""

    def __init__(self):
        self.channels: list[int] | None = None
        self.name = None
        self.updated = 0.0
        self._stop = False
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    @staticmethod
    def _find():
        import hid

        for d in hid.enumerate():
            if d.get("usage_page") != 1 or d.get("usage") not in (4, 5):
                continue
            label = f"{d.get('manufacturer_string') or ''} {d.get('product_string') or ''}"
            if (d["vendor_id"], d["product_id"]) == (BOXER_VENDOR_ID, BOXER_PRODUCT_ID) or TRANSMITTER_PATTERN.search(label):
                return d
        return None

    def _run(self):
        try:
            import hid
        except ImportError:
            return
        while not self._stop:
            info = self._find()
            if not info:
                time.sleep(1.0)
                continue
            try:
                dev = hid.device()
                dev.open_path(info["path"])
                dev.set_nonblocking(False)
                self.name = info.get("product_string") or "CRSF transmitter"
                print(f"[remote] {self.name} connected over USB", flush=True)
                while not self._stop:
                    data = dev.read(64, 200)
                    if data:
                        ch = decode_report(data)
                        if ch:
                            self.channels = ch
                            self.updated = time.time()
            except OSError:
                pass
            self.channels = None
            print("[remote] transmitter disconnected", flush=True)
            time.sleep(1.0)

    @property
    def fresh(self) -> bool:
        return self.channels is not None and time.time() - self.updated < 0.5

    def stop(self):
        self._stop = True
