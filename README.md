# 🔌 Microservizio: `mar-serial-bridge`

**Hardware Abstraction Layer & Watchdog Heartbeat Manager**  
*Parte dell'architettura a microservizi M.A.R. (Multi-legged Autonomous Robot)*

---

## 🎯 Ruolo nell'Architettura a Microservizi

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

---

## 📡 Interfacce e Topic MQTT

| Canale / Topic | Direzione | Formato Payload | Descrizione |
| :--- | :--- | :--- | :--- |
| `mar/control/motion` | In ingresso (Sub) | `{"cmd": "FORWARD", "param": 60}` | Comando di movimento |
| `mar/safety/estop` | In ingresso (Sub) | `{"reason": "OPERATOR"}` | Arresto immediato prioritario |
| `mar/hardware/telemetry` | In uscita (Pub) | `{"timestamp": ..., "batt": "11.8", "status": "OK"}` | Telemetria hardware per la dashboard |
| Seriale USB (`/dev/tty*`) | Bidirezionale | ASCII 115200 baud (`CMD:...` / `TLM:...`) | Collegamento con `mar-firmware` |

---

## 📂 Struttura della Cartella

```text
mar-serial-bridge/
├── README.md              # Questo documento
├── requirements.txt       # pyserial, paho-mqtt
├── serial_driver.py       # Driver I/O seriale thread-safe e auto-discovery
└── main.py                # Demone del microservizio con loop heartbeat e client MQTT
```

---

## 🚀 Avvio del Microservizio

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
```
