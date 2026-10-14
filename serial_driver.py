#!/usr/bin/env python3
"""
M.A.R. — Arduino Serial Communication Driver
Manages thread-safe communication, port discovery, and packet framing.
"""

from typing import Optional, List, Callable
import threading
import time
import glob
import re

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None


class ArduinoSerialDriver:
    """Manages low-level serial communication with Arduino."""

    def __init__(
        self,
        port: str = "",
        baudrate: int = 115200,
        timeout: float = 0.5,
        on_telemetry: Optional[Callable[[str], None]] = None
    ):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.on_telemetry = on_telemetry
        self.serial: Optional[serial.Serial] = None
        self.is_connected = False
        self.port_name: str = ""
        self.running = False
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None

        self.ultrasonic_distance_cm: float = -1.0
        self.last_telemetry_time: float = 0.0
        self.status_string: str = "DISCONNECTED"

    @staticmethod
    def discover_ports() -> List[str]:
        """Scans for available serial ports on Linux and macOS."""
        ports = []
        if serial is not None:
            try:
                for p in serial.tools.list_ports.comports():
                    desc = (p.description or "") + (p.manufacturer or "")
                    if any(k in desc.lower() for k in ["arduino", "usb", "ch340", "ftdi", "giga"]):
                        ports.append(p.device)
            except Exception:
                pass

        candidates = glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")
        for c in candidates:
            if c not in ports:
                ports.append(c)
        return ports

    def connect(self) -> bool:
        """Establishes connection with Arduino."""
        if serial is None:
            print("[SERIAL-DRIVER] ⚠️ pySerial non è installato.")
            return False

        target_port = self.port
        if not target_port:
            candidates = self.discover_ports()
            if candidates:
                target_port = candidates[0]
            else:
                print("[SERIAL-DRIVER] ⚠️ Nessun dispositivo Arduino rilevato.")
                return False

        print(f"[SERIAL-DRIVER] Connessione a {target_port} ({self.baudrate} baud)...")
        try:
            self.serial = serial.Serial(
                target_port,
                self.baudrate,
                timeout=self.timeout
            )
            self.port_name = target_port
            self.is_connected = True
            time.sleep(1.8)  # Wait for DTR auto-reset

            self.running = True
            self._thread = threading.Thread(target=self._read_loop, daemon=True)
            self._thread.start()

            print(f"  ✓ Connessione stabilita con Arduino su {target_port}")
            return True
        except Exception as e:
            print(f"  ✗ Errore connessione a {target_port}: {e}")
            self.is_connected = False
            return False

    def _read_loop(self):
        """Continuously reads incoming lines from Arduino."""
        while self.running and self.serial and self.serial.is_open:
            try:
                if self.serial.in_waiting > 0:
                    raw = self.serial.readline()
                    line = raw.decode("utf-8", errors="ignore").strip()
                    if not line:
                        continue

                    if "ULTRASONIC:" in line:
                        match = re.search(r"ULTRASONIC:([0-9.]+)", line)
                        if match:
                            with self._lock:
                                self.ultrasonic_distance_cm = float(match.group(1))
                                self.last_telemetry_time = time.time()

                    elif "EMERGENCY_STOP" in line:
                        with self._lock:
                            self.status_string = "EMERGENCY_STOP"
                            self.ultrasonic_distance_cm = 12.0
                        print(f"  🚨 [ARDUINO EMERGENCY]: {line}")

                    elif "TLM:" in line or "ARDUINO_READY" in line:
                        with self._lock:
                            self.status_string = line
                            self.last_telemetry_time = time.time()

                    if self.on_telemetry:
                        self.on_telemetry(line)

                time.sleep(0.01)
            except Exception:
                time.sleep(0.1)

    def send_command(self, cmd: str):
        """Transmits raw formatted command with newline."""
        if not self.is_connected or not self.serial or not self.serial.is_open:
            return

        with self._lock:
            try:
                packet = f"{cmd.strip()}\n"
                self.serial.write(packet.encode("utf-8"))
                self.serial.flush()
            except Exception as e:
                print(f"[SERIAL-DRIVER] ✗ Errore trasmissione comando: {e}")

    def emergency_stop(self):
        """Forces immediate standstill."""
        self.send_command("CMD:STOP")

    def close(self):
        """Safely closes connection."""
        self.running = False
        if self.is_connected and self.serial and self.serial.is_open:
            try:
                self.emergency_stop()
                time.sleep(0.1)
                self.serial.close()
            except Exception:
                pass
        self.is_connected = False
