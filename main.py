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
from ultrasonic_driver import UltrasonicUsbDriver
from port_roles import list_usb_serial_ports, probe_port_role

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

# Makeblock Ultrasonic V3 on second Arduino (USB) — pin A2 sketch in local/
ULTRASONIC_ENABLED = env_bool("ULTRASONIC_ENABLED", True)
ULTRASONIC_SIMULATE = env_bool("ULTRASONIC_SIMULATE", False)
# Default OFF: una distanza finta di 300 cm non deve mai entrare nella sicurezza del robot
ULTRASONIC_AUTO_FALLBACK = env_bool("ULTRASONIC_AUTO_FALLBACK", False)
ULTRASONIC_PORT = os.getenv("ULTRASONIC_PORT", "")  # empty = auto (ruolo identificato dal banner)
ULTRASONIC_BAUD = int(os.getenv("ULTRASONIC_BAUD", "115200"))
PORT_RESCAN_S = float(os.getenv("PORT_RESCAN_S", "10.0"))
STATUS_HZ = float(os.getenv("HW_STATUS_HZ", "1.0"))

# Ruoli porta assegnati (porta → ruolo) e porte già classificate come "unknown"
port_roles: dict = {}
unknown_ports: dict = {}   # porta → timestamp ultimo probe

running = True
driver: ArduinoSerialDriver = None
imu_driver: MPU6050Driver = None
us_driver: UltrasonicUsbDriver = None
mqtt_client = None


def publish_ultrasonic(cm: float, source: str = "makeblock_v3_a2", simulated: bool = False):
    """Publish frontal ultrasonic range. `simulated=True` → il decision engine lo IGNORA."""
    if not (mqtt_client and mqtt_client.is_connected()):
        return
    payload = {
        "timestamp": time.time(),
        "us_cm": round(float(cm), 1),
        "ultrasonic_cm": round(float(cm), 1),
        "ultrasonic": round(float(cm), 1),
        "source": source,
        "simulated": bool(simulated),
    }
    mqtt_client.publish("mar/ultrasonic/range", json.dumps(payload))
    mqtt_client.publish("mar/hardware/telemetry", json.dumps(payload))


def publish_hw_status():
    """Stato connessioni hardware (1 Hz) → dashboard / decision engine."""
    if not (mqtt_client and mqtt_client.is_connected()):
        return
    mqtt_client.publish("mar/hardware/status", json.dumps({
        "timestamp": time.time(),
        "motion_connected": bool(driver and driver.is_connected),
        "motion_port": (driver.port_name if driver and driver.is_connected else None),
        "motion_status": (driver.status_string if driver else "DISCONNECTED"),
        "ultrasonic_connected": bool(us_driver and us_driver.is_connected and not us_driver.simulated),
        "ultrasonic_port": (us_driver.port_name if us_driver and us_driver.is_connected else None),
        "ultrasonic_simulated": bool(us_driver and us_driver.simulated),
        "ultrasonic_cm": (us_driver.distance_cm if us_driver else None),
        "imu_enabled": IMU_ENABLED,
        "imu_connected": bool(imu_driver and imu_driver.is_available),
        "imu_status": (imu_driver.last_error if imu_driver else "disabled"),
        "imu_samples": (imu_driver.samples_read if imu_driver else 0),
        "port_roles": dict(port_roles),
        "unknown_ports": sorted(unknown_ports.keys()),
    }))


def on_arduino_telemetry(raw_line: str):
    """Callback when a telemetry string arrives from Arduino."""
    if not (raw_line.startswith("US:") or raw_line.startswith("TLM:US=") or raw_line == "PONG"):
        print(f"[TELEMETRY RX]: {raw_line}")
    if mqtt_client and mqtt_client.is_connected():
        payload = {
            "timestamp": time.time(),
            "raw": raw_line
        }

        # 1. Parsing prefisso TLM:...
        if raw_line.startswith("TLM:"):
            parts = raw_line[4:].split(",")
            for p in parts:
                if "=" in p:
                    k, v = p.split("=", 1)
                    payload[k.lower()] = v
                    if k.lower() in ("us", "ultrasonic", "us_cm"):
                        try:
                            publish_ultrasonic(float(v), source="motion_controller")
                        except Exception:
                            pass

        # 2. Parsing formato standard Arduino: "ULTRASONIC:25.4cm" o "US:25.4"
        import re
        us_match = re.search(r"(?:ULTRASONIC:|US:)\s*([0-9.]+)", raw_line, re.IGNORECASE)
        if us_match:
            try:
                cm_val = float(us_match.group(1))
                payload["ultrasonic_cm"] = cm_val
                publish_ultrasonic(cm_val, source="motion_controller")
            except Exception:
                pass

        # 3. Parsing Emergency Stop da sensore ultrasuoni: "EMERGENCY_STOP:12.0cm"
        if "EMERGENCY_STOP" in raw_line:
            stop_match = re.search(r"EMERGENCY_STOP:\s*([0-9.]+)", raw_line)
            em_cm = float(stop_match.group(1)) if stop_match else 12.0
            payload["emergency"] = True
            payload["ultrasonic_cm"] = em_cm
            publish_ultrasonic(em_cm, source="motion_controller")

        # 4. Parsing formato JSON
        if raw_line.startswith("{") and raw_line.endswith("}"):
            try:
                data = json.loads(raw_line)
                for k in ("ultrasonic_cm", "ultrasonic", "us_cm", "us", "distance"):
                    if k in data:
                        cm_val = float(data[k])
                        payload["ultrasonic_cm"] = cm_val
                        publish_ultrasonic(cm_val, source="motion_controller")
                        break
            except Exception:
                pass

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


def on_mqtt_connect(client, userdata, flags, rc, properties=None):
    """(Ri)sottoscrive i topic ad ogni connessione: resiste a riavvii del broker/DNS."""
    code = getattr(rc, "value", rc)
    if code == 0:
        client.subscribe([("mar/control/motion", 0), ("mar/safety/estop", 2)])
        print(f"  ✓ Connesso a MQTT Broker ({MQTT_BROKER}:{MQTT_PORT}) — topic sottoscritti")
        # stato hardware e IMU subito disponibili alla dashboard
        publish_hw_status()
        if imu_driver:
            on_imu_status(imu_driver.is_available, imu_driver.last_error)
    else:
        print(f"  ⚠️ MQTT connect rifiutato (rc={rc})")


def on_mqtt_disconnect(client, userdata, *args):
    print("  ⚠️ MQTT disconnesso: riconnessione automatica in corso...")


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

        if driver and driver.is_connected:
            print(f"[FORWARD TO ARDUINO]: {formatted_cmd}")
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
    if us_driver:
        us_driver.stop()
    sys.exit(0)


def main():
    global driver, imu_driver, us_driver, mqtt_client, running
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    print("=" * 60)
    print(" 🐾 M.A.R. — SERIAL BRIDGE & HARDWARE ABSTRACTION DAEMON")
    print("=" * 60)

    # 1. Driver seriali (le porte vengono assegnate dopo l'identificazione del ruolo)
    driver = ArduinoSerialDriver(
        port=SERIAL_PORT,
        baudrate=BAUDRATE,
        on_telemetry=on_arduino_telemetry
    )

    # 2. Avvia Heartbeat Watchdog Thread
    hb_thread = threading.Thread(target=heartbeat_loop, daemon=True)
    hb_thread.start()

    # 3. Connessione MQTT (se disponibile)
    if mqtt:
        # connect_async + loop_start: paho riprova da solo (anche se al boot il DNS del
        # cluster non risolve ancora `mar-mosquitto`) e ri-sottoscrive in on_connect.
        try:
            mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="mar-serial-bridge")
        except Exception:
            mqtt_client = mqtt.Client(client_id="mar-serial-bridge")
        mqtt_client.on_connect = on_mqtt_connect
        mqtt_client.on_disconnect = on_mqtt_disconnect
        mqtt_client.on_message = on_mqtt_message
        mqtt_client.reconnect_delay_set(min_delay=1, max_delay=10)
        mqtt_client.connect_async(MQTT_BROKER, MQTT_PORT, 60)
        mqtt_client.loop_start()
        print(f"  ✓ MQTT loop avviato verso {MQTT_BROKER}:{MQTT_PORT} (retry automatico)")

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
        print(f"  ✓ IMU: hot-plug I2C su /dev/i2c-{IMU_I2C_BUS} @0x{IMU_ADDRESS:02X}/0x69 "
              f"a {IMU_RATE_HZ:.0f} Hz (ok se assente: si collega da sola all'arrivo)")
    else:
        print("  ℹ️ IMU disabilitata (IMU_ENABLED=false).")

    # 5. Ultrasuoni Makeblock su Arduino USB dedicato
    if ULTRASONIC_ENABLED:
        us_driver = UltrasonicUsbDriver(
            port=ULTRASONIC_PORT,
            baudrate=ULTRASONIC_BAUD,
            on_range=publish_ultrasonic,
            simulated=ULTRASONIC_SIMULATE,
            auto_fallback=False,  # il fallback viene deciso qui sotto, dopo la scansione porte
        )
        if ULTRASONIC_SIMULATE:
            us_driver.connect()
    else:
        print("  ℹ️ Ultrasuoni disabilitati (ULTRASONIC_ENABLED=false).")

    # 6. Identificazione ruolo porte + connessione
    assign_ports(first=True)

    print("\n[SERIAL-BRIDGE] Servizio attivo in ascolto. Premi Ctrl+C per uscire.\n")

    last_rescan = time.time()
    last_status = 0.0
    while running:
        now = time.time()
        need_motion = not driver.is_connected
        need_us = ULTRASONIC_ENABLED and us_driver is not None and not us_driver.is_connected
        if (need_motion or need_us) and (now - last_rescan) >= PORT_RESCAN_S:
            last_rescan = now
            assign_ports(first=False)
        if now - last_status >= 1.0 / max(0.1, STATUS_HZ):
            last_status = now
            publish_hw_status()
        time.sleep(0.25)


def assign_ports(first: bool):
    """Sonda le porte USB non ancora assegnate, ne identifica il ruolo e collega i driver."""
    global port_roles
    # porte preferite da env: provate per prime, ma il ruolo viene comunque verificato dal banner
    # (una config vecchia con SERIAL_PORT=/dev/ttyUSB0 non deve far agganciare l'Arduino ultrasuoni come motion)
    for pref, want in ((SERIAL_PORT, "motion"), (ULTRASONIC_PORT, "ultrasonic")):
        if not pref or not os.path.exists(pref) or pref in port_roles:
            continue
        if want == "motion" and driver.is_connected:
            continue
        if want == "ultrasonic" and (not ULTRASONIC_ENABLED or us_driver is None or us_driver.is_connected):
            continue
        probe = probe_port_role(pref, baudrate=BAUDRATE)
        if probe.role == want or probe.role == "unknown":
            ok = driver.connect(pref) if want == "motion" else us_driver.connect(pref)
            if ok:
                port_roles[pref] = want
                if probe.role == "unknown":
                    print(f"[SERIAL-BRIDGE] {pref}: ruolo non identificato, uso la porta forzata come {want}")
        else:
            print(f"[SERIAL-BRIDGE] ⚠️ {pref} forzata come {want} ma il banner dice '{probe.role}': ignoro la forzatura")
            if probe.role in ("motion", "ultrasonic"):
                # assegna comunque al ruolo corretto
                if probe.role == "motion" and not driver.is_connected and driver.connect(pref):
                    port_roles[pref] = "motion"
                elif probe.role == "ultrasonic" and ULTRASONIC_ENABLED and us_driver and not us_driver.is_connected and us_driver.connect(pref):
                    port_roles[pref] = "ultrasonic"

    # dimentica le porte scomparse
    for p in list(port_roles.keys()):
        if not os.path.exists(p):
            port_roles.pop(p, None)
    for p in list(unknown_ports.keys()):
        if not os.path.exists(p):
            unknown_ports.pop(p, None)

    busy = set(port_roles.keys())
    if driver.is_connected:
        busy.add(driver.port_name)
    if us_driver and us_driver.is_connected and not us_driver.simulated:
        busy.add(us_driver.port_name)

    candidates = [p for p in list_usb_serial_ports(exclude=list(busy))
                  if (time.time() - unknown_ports.get(p, 0.0)) > 60.0]  # riprova le "unknown" ogni 60 s
    if first:
        print(f"[SERIAL-BRIDGE] Porte USB candidate: {candidates or 'nessuna'}")

    for p in candidates:
        if driver.is_connected and (not ULTRASONIC_ENABLED or us_driver is None or us_driver.is_connected):
            break
        probe = probe_port_role(p, baudrate=BAUDRATE)
        print(f"[SERIAL-BRIDGE] {p} → ruolo={probe.role} {('| ' + probe.note) if probe.note else ''} "
              f"{('| ' + ' / '.join(probe.lines[-3:])) if probe.lines else ''}")
        if probe.role == "motion" and not driver.is_connected:
            if driver.connect(p):
                port_roles[p] = "motion"
        elif probe.role == "ultrasonic" and ULTRASONIC_ENABLED and us_driver and not us_driver.is_connected:
            if us_driver.connect(p):
                port_roles[p] = "ultrasonic"
        elif probe.role in ("unknown", "error"):
            unknown_ports[p] = time.time()
        # "busy" → occupata da un altro processo (es. LiDAR): riproveremo al prossimo giro

    if first:
        if not driver.is_connected:
            print("[SERIAL-BRIDGE] ⚠️ Controller motion NON trovato: i comandi CMD: non verranno inviati (riprovo ogni "
                  f"{PORT_RESCAN_S:.0f}s)")
        if ULTRASONIC_ENABLED and us_driver and not us_driver.is_connected:
            if ULTRASONIC_AUTO_FALLBACK:
                us_driver.auto_fallback = True
                us_driver.connect()
            else:
                print("[SERIAL-BRIDGE] ⚠️ Arduino ultrasuoni NON trovato: nessun dato US (nessuna simulazione)")


if __name__ == "__main__":
    main()
