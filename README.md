# Microservizio: `mar-serial-bridge`

**Hardware Abstraction Layer & Watchdog Heartbeat Manager**  
*Parte dell'architettura a microservizi M.A.R. (Multi-legged Autonomous Robot)*

---

## Ruolo nell'Architettura a Microservizi

`mar-serial-bridge` funge da **ponte hardware deterministico** tra il bus di comunicazione ad alto livello (MQTT / WebSockets su Raspberry Pi 5) e il microcontrollore fisico che muove le zampe (`mar-firmware`).

Le sue responsabilità architetturali sono:
1. **Isolamento della Seriale Hardware**:
   - Gestisce la porta USB `/dev/ttyACM*` o `/dev/ttyUSB*` in modo thread-safe.
   - Gestisce la riconnessione automatica in caso di disconnessione del cavo o reset della scheda.
2. **Generatore di Heartbeat (Safety Loop)**:
   - Invia periodicamente un pacchetto di keep-alive (`CMD:PING`) ogni 200 ms ad Arduino.
   - Garantisce che il watchdog del firmware non scatti finché il sistema software RPi è operativo.
3. **Decodifica e Pubblicazione Telemetria**:
   - Riceve le stringhe di stato e telemetria (`TLM:...`, ultrasuoni, correnti, batterie) da Arduino.
   - Le pubblica in formato JSON strutturato sul topic MQTT `mar/hardware/telemetry`.
4. **Instradamento Prioritario dei Comandi di Moto**:
   - Sottoscrive il topic `mar/control/motion` (proveniente da `mar-decision-engine` o dall'operatore).
   - Sottoscrive con priorità assoluta `mar/safety/estop`, arrestando istantaneamente i motori.
5. **Acquisizione IMU (GY-521 / MPU-6050)**:
   - Legge il burst di 14 byte (accelerometro + giroscopio) dal bus I2C (`/dev/i2c-1`, indirizzo `0x68`).
   - Pubblica i valori grezzi su `mar/imu/raw`: la conversione in unità fisiche, la calibrazione dei bias e il filtro di Kalman restano nel `mar-decision-engine`, che è il proprietario della fusione sensoriale.

---

## Interfacce e Topic MQTT

| Canale / Topic | Direzione | Formato Payload | Descrizione |
| :--- | :--- | :--- | :--- |
| `mar/control/motion` | In ingresso (Sub) | `{"cmd": "FORWARD", "param": 60}` | Comando di movimento |
| `mar/safety/estop` | In ingresso (Sub) | `{"reason": "OPERATOR"}` | Arresto immediato prioritario |
| `mar/hardware/telemetry` | In uscita (Pub) | `{"timestamp": ..., "batt": "11.8", "status": "OK"}` | Telemetria hardware per la dashboard |
| `mar/imu/raw` | In uscita (Pub) | `{"timestamp": ..., "gyro": [gx, gy, gz], "acc": [ax, ay, az], "gyro_scale": 131.0, "accel_scale": 16384.0}` | Campioni IMU grezzi (100 Hz) per il Decision Engine |
| `mar/imu/status` | In uscita (Pub) | `{"timestamp": ..., "available": true, "reason": "MPU-6050 attiva"}` | Disponibilità del sensore IMU |
| Seriale USB (`/dev/tty*`) | Bidirezionale | ASCII 115200 baud (`CMD:...` / `TLM:...`) | Collegamento con `mar-firmware` |
| I2C (`/dev/i2c-1`) | In ingresso | MPU-6050 @ `0x68` | Bus del sensore inerziale |

---

## Struttura del Repository

```text
mar-serial-bridge/
├── README.md              # Questo documento
├── requirements.txt       # pyserial, paho-mqtt, smbus2
├── serial_driver.py       # Driver I/O seriale thread-safe e auto-discovery
├── imu_driver.py          # Driver I2C MPU-6050 (lettura raw, riconnessione automatica)
└── main.py                # Demone del microservizio con loop heartbeat, IMU e client MQTT
```

---

## Avvio del Microservizio

### 1. Installazione Dipendenze
```bash
pip install -r requirements.txt
```

### 2. Esecuzione
```bash
# Esecuzione standard (con broker locale)
python main.py

# Personalizzazione porte via variabili d'ambiente
SERIAL_PORT=/dev/ttyACM0 SERIAL_BAUD=115200 MQTT_BROKER=127.0.0.1 python main.py

# Configurazione IMU (GY-521 / MPU-6050 su I2C)
IMU_ENABLED=true IMU_I2C_BUS=1 IMU_ADDRESS=0x68 IMU_RATE_HZ=100 python main.py
```
