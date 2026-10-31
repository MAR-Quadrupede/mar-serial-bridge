#!/usr/bin/env python3
"""
M.A.R. — USB Makeblock Ultrasonic reader (Arduino Micro/Nano sketch).
Expects lines: US:123.4  or  TLM:US=123.4
"""

from __future__ import annotations

import glob
import re
import threading
import time
from typing import Callable, List, Optional

try:
    import serial
except ImportError:
    serial = None

US_LINE_RE = re.compile(
    r"(?:^US:|(?:^|,)US=)\s*([0-9]+(?:\.[0-9]+)?|nan)",
    re.IGNORECASE,
)


class UltrasonicUsbDriver:
    def __init__(
        self,
        port: str = "",
        baudrate: int = 115200,
        on_range: Optional[Callable[[float], None]] = None,
        exclude_ports: Optional[List[str]] = None,
    ):
        self.port = (port or "").strip()
        self.baudrate = baudrate
        self.on_range = on_range
        self.exclude_ports = set(exclude_ports or [])
        self.serial = None
        self.is_connected = False
        self.port_name = ""
        self.running = False
        self.distance_cm: float = -1.0
        self.last_ok_time: float = 0.0
        self._thread: Optional[threading.Thread] = None

    @staticmethod
    def discover_ports(exclude: Optional[List[str]] = None) -> List[str]:
        exclude = set(exclude or [])
        ports = []
        for pattern in ("/dev/ttyUSB*", "/dev/ttyACM*", "/dev/cu.usbmodem*", "/dev/cu.wchusb*"):
            for p in sorted(glob.glob(pattern)):
                if p not in exclude and p not in ports:
                    ports.append(p)
        return ports

    def connect(self) -> bool:
        if serial is None:
            print("[US-DRIVER] pySerial mancante")
            return False

        candidates = []
        if self.port:
            candidates.append(self.port)
        for p in self.discover_ports(list(self.exclude_ports)):
            if p not in candidates:
                candidates.append(p)

        for p in candidates:
            try:
                ser = serial.Serial(p, self.baudrate, timeout=0.4)
                time.sleep(1.6)  # DTR reset Nano/Micro
                ser.reset_input_buffer()
                # Probe briefly for US: lines / ready banner
                t0 = time.time()
                got = False
                while time.time() - t0 < 1.2:
                    line = ser.readline().decode("utf-8", errors="ignore").strip()
                    if not line:
                        continue
                    if "MAR_US_READY" in line or US_LINE_RE.search(line):
                        got = True
                        break
                if not got and self.port and p == self.port:
                    # Forced port: accept anyway
                    got = True
                if not got:
                    ser.close()
                    continue

                self.serial = ser
                self.port_name = p
                self.is_connected = True
                self.running = True
                self._thread = threading.Thread(target=self._read_loop, daemon=True, name="us-usb")
                self._thread.start()
                print(f"  ✓ [US-DRIVER] Ultrasuoni USB su {p} @ {self.baudrate}")
                return True
            except Exception as e:
                try:
                    if self.serial:
                        self.serial.close()
                except Exception:
                    pass
                self.serial = None
                print(f"  · [US-DRIVER] skip {p}: {e}")

        print("  ⚠️ [US-DRIVER] Nessun Arduino ultrasuoni trovato")
        self.is_connected = False
        return False

    def _read_loop(self):
        while self.running and self.serial:
            try:
                raw = self.serial.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="ignore").strip()
                m = US_LINE_RE.search(line)
                if not m:
                    continue
                token = m.group(1)
                if token.lower() == "nan":
                    continue
                cm = float(token)
                if cm < 2.0 or cm > 400.0:
                    continue
                self.distance_cm = cm
                self.last_ok_time = time.time()
                if self.on_range:
                    self.on_range(cm)
            except Exception:
                time.sleep(0.05)

    def stop(self):
        self.running = False
        self.is_connected = False
        try:
            if self.serial:
                self.serial.close()
        except Exception:
            pass
        self.serial = None
