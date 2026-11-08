#!/usr/bin/env python3
"""
M.A.R. — Identificazione del RUOLO di ogni porta seriale USB.

Sul rover convivono due Arduino sullo stesso bus USB e Linux non garantisce l'ordine
di enumerazione (ttyUSB0/ttyACM0 possono scambiarsi ad ogni boot). Invece di "prendere
la prima porta che risponde", ogni porta viene aperta UNA volta (in esclusiva), si attende
il reset DTR e si classifica dal banner / dalle righe di telemetria:

  motion      → firmware `mar_stereovision_controller` (NON toccare, dev dedicati):
                ARDUINO_READY, MAR_STEREO_CONTROLLER:ONLINE, TLM:STATUS=..., TLM:GAIT_PERIOD=,
                ULTRASONIC:xx.xcm, EMERGENCY_STOP:
  ultrasonic  → sketch `local/ultrasonic_makeblock_a2`: MAR_US_READY, US:xx.x, TLM:US=xx.x
  unknown     → nessun marcatore riconosciuto (es. LiDAR USB o altro device)

Se una porta non emette nulla spontaneamente, si invia `CMD:PING`: entrambi rispondono
PONG, quindi PONG da solo NON basta a classificare (serve almeno un marcatore).
"""

from __future__ import annotations

import glob
import re
import time
from dataclasses import dataclass, field
from typing import List, Optional

try:
    import serial
except ImportError:  # pragma: no cover
    serial = None

MOTION_MARKERS = (
    "ARDUINO_READY",
    "MAR_STEREO_CONTROLLER",
    "TLM:STATUS",
    "TLM:GAIT",
    "TLM:ALERT",
    "ULTRASONIC:",
    "EMERGENCY_STOP",
)
US_MARKERS = ("MAR_US_READY",)
US_LINE_RE = re.compile(r"^(?:US:|TLM:US=)\s*[0-9.]+", re.IGNORECASE)


@dataclass
class PortProbe:
    port: str
    role: str                      # motion | ultrasonic | unknown | busy | error
    lines: List[str] = field(default_factory=list)
    note: str = ""


def classify_lines(lines: List[str]) -> str:
    motion = any(any(m in ln for m in MOTION_MARKERS) for ln in lines)
    if motion:
        return "motion"
    us = any((any(m in ln for m in US_MARKERS) or US_LINE_RE.match(ln)) for ln in lines)
    if us:
        return "ultrasonic"
    return "unknown"


def list_usb_serial_ports(exclude: Optional[List[str]] = None) -> List[str]:
    exclude = set(exclude or [])
    ports: List[str] = []
    for pattern in ("/dev/ttyACM*", "/dev/ttyUSB*", "/dev/cu.usbmodem*", "/dev/cu.wchusb*", "/dev/cu.usbserial*"):
        for p in sorted(glob.glob(pattern)):
            if p not in exclude and p not in ports:
                ports.append(p)
    return ports


def probe_port_role(port: str, baudrate: int = 115200, listen_s: float = 3.0, reset_wait_s: float = 1.8) -> PortProbe:
    """Apre la porta in esclusiva, aspetta il reset DTR, ascolta e classifica. Chiude sempre la porta."""
    if serial is None:
        return PortProbe(port, "error", note="pyserial mancante")
    ser = None
    lines: List[str] = []
    try:
        ser = serial.Serial(port, baudrate, timeout=0.2, exclusive=True)
    except Exception as e:
        msg = str(e).lower()
        role = "busy" if ("exclusive" in msg or "busy" in msg or "lock" in msg or "resource" in msg) else "error"
        return PortProbe(port, role, note=str(e))
    try:
        time.sleep(reset_wait_s)  # reset DTR dell'Arduino → banner
        t0 = time.time()
        pinged = False
        while time.time() - t0 < listen_s:
            raw = ser.readline()
            if raw:
                ln = raw.decode("utf-8", errors="ignore").strip()
                if ln:
                    lines.append(ln)
                    if classify_lines([ln]) != "unknown":
                        break
            elif not pinged and (time.time() - t0) > 1.0:
                try:
                    ser.write(b"CMD:PING\n")
                    ser.flush()
                except Exception:
                    pass
                pinged = True
        role = classify_lines(lines)
        return PortProbe(port, role, lines=lines[-8:])
    except Exception as e:
        return PortProbe(port, "error", lines=lines[-8:], note=str(e))
    finally:
        try:
            if ser is not None:
                ser.close()
        except Exception:
            pass
