#!/usr/bin/env python3
"""
M.A.R. — MPU-6050 / GY-521 IMU I2C Driver
Hardware abstraction driver for the 6-axis IMU connected to the Raspberry Pi
I2C bus (default /dev/i2c-1, address 0x68).

This module only performs raw acquisition: it wakes the sensor, reads the
14-byte burst (accel + temp + gyro) and streams raw counts to the message bus.
All scaling, Kalman filtering and attitude estimation live in
`mar-decision-engine` (imu_fusion.py), so the brain keeps full control of the
sensor-fusion policy while this daemon stays a thin HAL.
"""

from typing import Callable, Dict, List, Optional, Tuple
import threading
import time

try:
    import smbus2
except ImportError:
    smbus2 = None


# MPU-6050 register map (subset used here)
MPU6050_ADDR_DEFAULT = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
WHO_AM_I = 0x75

# Full-scale defaults (FS_SEL=0 -> ±250 °/s, AFS_SEL=0 -> ±2 g)
GYRO_SCALE_COUNTS = 131.0     # LSB per (°/s)
ACCEL_SCALE_COUNTS = 16384.0  # LSB per g

SampleCallback = Callable[[List[int], List[int]], None]
StatusCallback = Callable[[bool, str], None]


def _to_signed(high: int, low: int) -> int:
    """Recombines a big-endian 16-bit two's complement value."""
    value = (high << 8) | low
    return value - 65536 if value > 32767 else value


class MPU6050Driver:
    """Thread-safe MPU-6050 reader with automatic reconnection."""

    def __init__(
        self,
        bus: int = 1,
        address: int = MPU6050_ADDR_DEFAULT,
        rate_hz: float = 100.0,
        retry_sec: float = 10.0,
        on_sample: Optional[SampleCallback] = None,
        on_status: Optional[StatusCallback] = None
    ):
        self.bus_number = bus
        self.address = address
        self.rate_hz = max(1.0, float(rate_hz))
        self.retry_sec = max(1.0, float(retry_sec))
        self.on_sample = on_sample
        self.on_status = on_status

        self.bus: Optional["smbus2.SMBus"] = None
        self.is_available = False
        self.last_error = "not started"
        self.samples_read = 0
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

    # ------------------------------------------------------------------ I2C
    def open(self) -> bool:
        """Opens the I2C bus and wakes the MPU-6050."""
        if smbus2 is None:
            self._set_status(False, "smbus2 non installato (pip install smbus2)")
            return False

        try:
            bus = smbus2.SMBus(self.bus_number)
            # Wake device: clear SLEEP bit, use internal 8 MHz oscillator
            bus.write_byte_data(self.address, PWR_MGMT_1, 0x00)
            time.sleep(0.05)
            # Sanity check (WHO_AM_I == 0x68); tolerate clones that do not answer
            try:
                who = bus.read_byte_data(self.address, WHO_AM_I)
                if who not in (0x68, 0x69, 0x70, 0x71, 0x72, 0x73, 0x74, 0x98):
                    bus.close()
                    self._set_status(False, f"WHO_AM_I inatteso su 0x{self.address:02X}: 0x{who:02X}")
                    return False
            except Exception:
                pass

            with self._lock:
                self.bus = bus
            self._set_status(True, f"MPU-6050 attiva su /dev/i2c-{self.bus_number} @0x{self.address:02X}")
            return True
        except Exception as e:
            self._set_status(False, f"IMU non rilevata su I2C ({e})")
            return False

    def read_raw(self) -> Optional[Tuple[List[int], List[int]]]:
        """Reads accel + gyro from the 14-byte burst. Returns (gyro, accel) raw counts."""
        with self._lock:
            bus = self.bus
        if bus is None:
            return None

        try:
            block = bus.read_i2c_block_data(self.address, ACCEL_XOUT_H, 14)
        except Exception as e:
            # Rilascia il bus ma NON fermare il thread: la `_loop` riproverà
            # `open()` ogni `retry_sec` → hot-plug / glitch I2C tollerati.
            self._set_status(False, f"Errore lettura I2C ({e}) — riprovo tra {self.retry_sec:.0f}s")
            self._release_bus()
            return None

        acc = [
            _to_signed(block[0], block[1]),
            _to_signed(block[2], block[3]),
            _to_signed(block[4], block[5]),
        ]
        gyro = [
            _to_signed(block[8], block[9]),
            _to_signed(block[10], block[11]),
            _to_signed(block[12], block[13]),
        ]
        self.samples_read += 1
        return gyro, acc

    def sample_payload(self, gyro: List[int], acc: List[int]) -> Dict:
        """Builds the JSON-ready `mar/imu/raw` payload."""
        return {
            "timestamp": time.time(),
            "source": "mpu6050",
            "gyro": gyro,
            "acc": acc,
            "gyro_scale": GYRO_SCALE_COUNTS,
            "accel_scale": ACCEL_SCALE_COUNTS,
            "rate_hz": self.rate_hz,
        }

    # -------------------------------------------------------------- lifecycle
    def start(self):
        """Starts the acquisition thread (opens the sensor lazily)."""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="mpu6050", daemon=True)
        self._thread.start()

    def _loop(self):
        period = 1.0 / self.rate_hz
        next_tick = time.time()

        while self._running:
            if self.bus is None:
                if not self.open():
                    time.sleep(self.retry_sec)
                    continue

            sample = self.read_raw()
            if sample is None:
                time.sleep(self.retry_sec)
                continue

            gyro, acc = sample
            if self.on_sample:
                try:
                    self.on_sample(gyro, acc)
                except Exception as e:
                    print(f"[IMU-DRIVER] Errore callback campione: {e}")

            next_tick += period
            sleep_time = next_tick - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_tick = time.time()

    def _release_bus(self):
        """Releases the I2C handle (keeps the acquisition thread alive)."""
        with self._lock:
            if self.bus is not None:
                try:
                    self.bus.close()
                except Exception:
                    pass
                self.bus = None

    def close(self):
        """Stops the acquisition thread and releases the I2C bus."""
        self._running = False
        self._release_bus()
        if self.is_available:
            self._set_status(False, "IMU disconnessa")

    def stop(self):
        self.close()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)

    def _set_status(self, available: bool, reason: str):
        changed = (available != self.is_available) or (reason != self.last_error)
        self.is_available = available
        self.last_error = reason
        if changed and self.on_status:
            try:
                self.on_status(available, reason)
            except Exception:
                pass
        if changed:
            icon = "✓" if available else "⚠️"
            print(f"  {icon} [IMU-DRIVER] {reason}")
