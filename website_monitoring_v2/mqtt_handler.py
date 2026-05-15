import paho.mqtt.client as mqtt

# Semua topic yang di-publish oleh ESP32 firmware
TOPICS = [
    "kel1/pa/fallStatus",       # "FALL" | "SAFE" | "DISCONNECTED"
    "kel1/pa/heartRateStatus",  # "75.2" | "0"
    "kel1/pa/fallProb",         # "0.823"
    "kel1/pa/ledStatus",        # "ON" | "OFF"
    "kel1/pa/buzzerStatus",     # "ON" | "OFF"
    "kel1/pa/buzzerReason",     # "NONE" | "BPM_LOW:52" | "BPM_HIGH:135" | "HRV:95"
    "kel1/pa/ecgRaw",           # "2048" — ADC 12-bit (0–4095), "0" saat leads off
    "kel1/pa/hrv",              # RMSSD dalam ms, "0" saat leads off
    "kel1/pa/posture",          # "STANDING" | "SITTING" | "LYING" | "UNKNOWN"
]

class MQTTClient:
    def __init__(self, broker, port, callback):
        self.client = mqtt.Client()
        self.broker = broker
        self.port = port
        self.callback = callback

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print(f"[MQTT] Connected to {self.broker}")
            for topic in TOPICS:
                client.subscribe(topic)
                print(f"[MQTT] Subscribed: {topic}")
        else:
            print(f"[MQTT] Connection failed, rc={rc}")

    def on_message(self, client, userdata, msg):
        topic   = msg.topic
        payload = msg.payload.decode("utf-8").strip()
        print(f"[MQTT] {topic} → {payload}")
        self.callback(topic, payload)

    def start(self):
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect(self.broker, self.port, 60)
        self.client.loop_start()