import json
import time
import paho.mqtt.client as mqtt
from telegram import Bot

# === Konfiguration ===
MQTT_BROKER = "localhost"
MQTT_TOPIC = "radar/movement"
BOT_TOKEN = "DEIN_BOT_TOKEN"
CHAT_ID = -123456789  # Telegram Chat-ID
LOG_FILE = "/home/pi/radar_log.json"

bot = Bot(token=BOT_TOKEN)

# === JSON-Log vorbereiten ===
def log_event(event):
    try:
        with open(LOG_FILE, "r") as f:
            logs = json.load(f)
    except FileNotFoundError:
        logs = []

    logs.append(event)
    # max 7 Tage
    logs = logs[-1000:]
    with open(LOG_FILE, "w") as f:
        json.dump(logs, f)

# === MQTT Callback ===
def on_message(client, userdata, msg):
    payload = msg.payload.decode()
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
    event = {"time": timestamp, "event": payload}
    log_event(event)
    # Telegram-Nachricht senden
    bot.send_message(chat_id=CHAT_ID, text=f"🚨 {payload} erkannt!")

# === MQTT verbinden ===
client = mqtt.Client()
client.on_message = on_message
client.connect(MQTT_BROKER)
client.subscribe(MQTT_TOPIC)
client.loop_forever()
