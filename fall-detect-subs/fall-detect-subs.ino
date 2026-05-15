#include <WiFi.h>
#include <PubSubClient.h>
#include <LiquidCrystal_I2C.h>

// ── Konfigurasi ──────────────────────────────────────────────
const char* WIFI_SSID = "Ridho Ruijie";
const char* WIFI_PASSWORD = "ridhosetiawanskom24";
const char* MQTT_BROKER = "broker.hivemq.com";

#define PIN_LED 13
#define PIN_BUZZER 12
#define BUZZER_FREQ 2000

LiquidCrystal_I2C lcd(0x27, 16, 4);
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Timing & State
unsigned long prevMs = 0;
const long interval = 100; // 10Hz

// Variabel Kontrol Utama (Volatile untuk kecepatan akses)
volatile bool stateLED = false;
volatile bool stateBuzzer = false;
char statusFall[17] = "WAITING...";

// ── Callback MQTT (Fokus Hanya 3 Topik) ──────────────────────
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  char message[length + 1];
  memcpy(message, payload, length);
  message[length] = '\0';
  
  String t = String(topic);

  // 1. Fokus Status Jatuh (Tampilan)
  if (t.endsWith("fallStatus")) {
    snprintf(statusFall, sizeof(statusFall), "%s", message);
  }
  // 2. Fokus LED (Aktuasi)
  else if (t.endsWith("ledStatus")) {
    stateLED = (strcmp(message, "ON") == 0);
    digitalWrite(PIN_LED, stateLED ? HIGH : LOW);
  }
  // 3. Fokus Buzzer (Aktuasi)
  else if (t.endsWith("buzzerStatus")) {
    stateBuzzer = (strcmp(message, "ON") == 0);
    if (stateBuzzer) ledcWriteTone(PIN_BUZZER, BUZZER_FREQ);
    else ledcWriteTone(PIN_BUZZER, 0);
  }
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  
  ledcAttach(PIN_BUZZER, BUZZER_FREQ, 8);
  ledcWriteTone(PIN_BUZZER, 0);
  
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("MONITOR ACTIVE");

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  mqttClient.setServer(MQTT_BROKER, 1883);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(256); // Buffer kecil agar cepat di-flush
}

void loop() {
  // Koneksi WiFi & MQTT Non-Blocking
  if (WiFi.status() == WL_CONNECTED) {
    if (!mqttClient.connected()) {
      // Gunakan ID unik agar tidak konflik dengan pengirim
      if (mqttClient.connect("Monitor_fall_detect_Unique")) {
        mqttClient.subscribe("kel1/pa/fallStatus");
        mqttClient.subscribe("kel1/pa/ledStatus");
        mqttClient.subscribe("kel1/pa/buzzerStatus");
      }
    }
    mqttClient.loop();
  }

  // Sinkronisasi Hardware & LCD (10Hz)
  unsigned long now = millis();
  if (now - prevMs >= interval) {
    prevMs = now;
    
    // FORCE SYNC: Memastikan pin fisik selalu sama dengan status terakhir
    digitalWrite(PIN_LED, stateLED ? HIGH : LOW);
    if (stateBuzzer) ledcWriteTone(PIN_BUZZER, BUZZER_FREQ);
    else ledcWriteTone(PIN_BUZZER, 0);

    // Update Layar LCD (Hanya baris kritis)
    lcd.setCursor(0, 0);
    if (strcmp(statusFall, "FALL") == 0) {
      lcd.print("!! FALL DETECT !!");
    } else {
      lcd.print("Status: ");
      lcd.print(statusFall);
      lcd.print("       "); // Clear sisa karakter
    }

    lcd.setCursor(0, 2);
    lcd.print("LED   : ");
    lcd.print(stateLED ? "ON " : "OFF");

    lcd.setCursor(0, 3);
    lcd.print("BUZZER: ");
    lcd.print(stateBuzzer ? "ON " : "OFF");
  }
}