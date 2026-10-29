#!/usr/bin/env python3
"""
M.A.R. — mar-serial-bridge microservice daemon
Connects RPi 5 to Arduino, sends continuous watchdog heartbeats,
listens to MQTT motion commands, and publishes hardware telemetry.
"""

import os
import sys
import time
import json
import signal
import threading

from serial_driver import ArduinoSerialDriver
from imu_driver import MPU6050Driver

# Optional MQTT import
try:
    import paho.mqtt.client as mqtt
except ImportError:
    mqtt = None


def env_bool(name: str, default: bool = False) -> bool:
    value = os.getenv(name)
    if value is None:
        return default
    return value.strip().lower() in ("true", "1", "yes", "on")


MQTT_BROKER = os.getenv("MQTT_BROKER", "localhost")
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))
SERIAL_PORT = os.getenv("SERIAL_PORT", "")
BAUDRATE = int(os.getenv("SERIAL_BAUD", "115200"))
HEARTBEAT_INTERVAL = float(os.getenv("HEARTBEAT_SEC", "0.2"))  # 5 Hz heartbeat

# IMU GY-521 / MPU-6050 on Raspberry Pi I2C bus
IMU_ENABLED = env_bool("IMU_ENABLED", True)
IMU_I2C_BUS = int(os.getenv("IMU_I2C_BUS", "1"))
IMU_ADDRESS = int(os.getenv("IMU_ADDRESS", "0x68"), 0)
IMU_RATE_HZ = float(os.getenv("IMU_RATE_HZ", "100"))

running = True
driver: ArduinoSerialDriver = None
imu_driver: MPU6050Driver = None
mqtt_client = None


def on_arduino_telemetry(raw_line: str):
    """Callback when a telemetry string arrives from Arduino."""
    print(f"[TELEMETRY RX]: {raw_line}")
    if mqtt_client and mqtt_client.is_connected():
        payload = {
            "timestamp": time.time(),
            "raw": raw_line
        }
        # Parse TLM:STATUS=...
        if raw_line.startswith("TLM:"):
            parts = raw_line[4:].split(",")
            for p in parts:
                if "=" in p:
                    k, v = p.split("=", 1)
                    payload[k.lower()] = v

        mqtt_client.publish("mar/hardware/telemetry", json.dumps(payload))


def on_imu_sample(gyro, acc):
    """Callback for each raw MPU-6050 sample -> `mar/imu/raw` (consumed by the decision engine)."""
    if mqtt_client and mqtt_client.is_connected():
        # Reuse the driver instance to build the payload (keeps scales in sync)
        payload = imu_driver.sample_payload(gyro, acc) if imu_driver else {
            "timestamp": time.time(), "source": "mpu6050", "gyro": gyro, "acc": acc
        }
        mqtt_client.publish("mar/imu/raw", json.dumps(payload))


def on_imu_status(available: bool, reason: str):
    """Publishes IMU availability changes so the dashboard/engine can react."""
    if mqtt_client and mqtt_client.is_connected():
        mqtt_client.publish("mar/imu/status", json.dumps({
            "timestamp": time.time(),
            "available": available,
            "reason": reason
        }))


def on_mqtt_message(client, userdata, msg):
    """Handles incoming motion commands from the Decision Engine or Operator."""
    topic = msg.topic
    try:
        data = json.loads(msg.payload.decode("utf-8"))
    except Exception:
        data = {"cmd": msg.payload.decode("utf-8").strip()}

    if topic == "mar/safety/estop":
        print(f"🚨 [E-STOP RICEVUTO] Motivo: {data.get('reason', 'N/A')}")
        if driver:
            driver.emergency_stop()

    elif topic == "mar/control/motion":
        cmd_type = str(data.get("cmd", "")).strip()
        param = str(data.get("param", "")).strip()

        # Support both direct keywords (AVANTI, STOP_TERRA, etc.) and CMD: structured protocol
        if cmd_type.upper() in ("AVANTI", "STOP", "STOP_TERRA", "GIRA_DX", "GIRA_SX"):
            formatted_cmd = cmd_type.upper()
        elif cmd_type.startswith("CMD:"):
            formatted_cmd = cmd_type
            if param != "":
                formatted_cmd += f",{param}"
        else:
            formatted_cmd = f"CMD:{cmd_type}"
            if param != "":
                formatted_cmd += f",{param}"

        print(f"[FORWARD TO ARDUINO]: {formatted_cmd}")
        if driver:
            driver.send_command(formatted_cmd)


def heartbeat_loop():
    """Sends heartbeat ping every HEARTBEAT_INTERVAL to keep Arduino watchdog active."""
    while running:
        if driver and driver.is_connected:
            driver.send_command("CMD:PING")
        time.sleep(HEARTBEAT_INTERVAL)


def signal_handler(sig, frame):
    global running
    print("\n[SERIAL-BRIDGE] Ricevuto segnale di terminazione...")
    running = False
    if driver:
        driver.close()
    if imu_driver:
        imu_driver.stop()
    sys.exit(0)


def main():
    global driver, imu_driver, mqtt_client, running
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    print("=" * 60)
    print(" 🐾 M.A.R. — SERIAL BRIDGE & HARDWARE ABSTRACTION DAEMON")
    print("=" * 60)

    # 1. Inizializza driver seriale
    driver = ArduinoSerialDriver(
        port=SERIAL_PORT,
        baudrate=BAUDRATE,
        on_telemetry=on_arduino_telemetry
    )
    connected = driver.connect()
    if not connected:
        print("[SERIAL-BRIDGE] ⚠️ Impossibile agganciare Arduino. Riproverò in background...")

    # 2. Avvia Heartbeat Watchdog Thread
    hb_thread = threading.Thread(target=heartbeat_loop, daemon=True)
    hb_thread.start()

    # 3. Connessione MQTT (se disponibile)
    if mqtt:
        try:
            mqtt_client = mqtt.Client(client_id="mar-serial-bridge")
            mqtt_client.on_message = on_mqtt_message
            mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
            mqtt_client.subscribe([("mar/control/motion", 0), ("mar/safety/estop", 2)])
            mqtt_client.loop_start()
            print(f"  ✓ Connesso a MQTT Broker ({MQTT_BROKER}:{MQTT_PORT})")
        except Exception as e:
            print(f"  ⚠️ Connessione MQTT non riuscita ({e}). Modalità autonoma.")

    # 4. IMU GY-521 / MPU-6050: acquisizione raw verso `mar/imu/raw`
    if IMU_ENABLED:
        imu_driver = MPU6050Driver(
            bus=IMU_I2C_BUS,
            address=IMU_ADDRESS,
            rate_hz=IMU_RATE_HZ,
            on_sample=on_imu_sample,
            on_status=on_imu_status
        )
        imu_driver.start()
        print(f"  ✓ IMU: lettura I2C avviata a {IMU_RATE_HZ:.0f} Hz su /dev/i2c-{IMU_I2C_BUS}")
    else:
        print("  ℹ️ IMU disabilitata (IMU_ENABLED=false).")

    print("\n[SERIAL-BRIDGE] Servizio attivo in ascolto. Premi Ctrl+C per uscire.\n")

    while running:
        if not driver.is_connected:
            time.sleep(2.0)
            driver.connect()
        else:
            time.sleep(0.5)


if __name__ == "__main__":
    main()
