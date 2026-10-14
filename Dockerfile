FROM python:3.11-slim

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY . .

ENV PYTHONUNBUFFERED=1
ENV MQTT_BROKER=mar-mosquitto
ENV MQTT_PORT=1883
ENV SERIAL_PORT=/dev/ttyACM0
ENV SERIAL_BAUD=115200

CMD ["python", "main.py"]
