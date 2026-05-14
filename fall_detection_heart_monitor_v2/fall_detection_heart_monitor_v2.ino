// ================================================================
//  ESP32 — GY521 (MPU-6050) + AD8232 (ECG)
// ================================================================

#include "GY521.h"
#include <Wire.h>
#include "WiFi.h"
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ── EdgeNeuron TinyML ───────────────────────────────────────────
#include <EdgeNeuron.h>
#include "model.h" // your .h model file — check the array name inside!

// At the top with your other globals
float global_fall_prob = 0.0f;
bool global_is_fall = false;

// -- Konfigurasi WiFi & MQTT --
const char* ssid = "Ridho Ruijie";
const char* password = "ridhosetiawanskom24";

const char* mqtt_server = "mqtt.antares.id";
const int mqtt_port = 1883;
const char* antares_access_key = "14a1e0a864fbcbf2:59175a776d9717b8"; // dari akun Antares kamu
const char* antares_project = "PA-IOT";
const char* antares_device  = "fall-00";

char mqtt_topic[128];
char led_topic[128];

WiFiClient espClient;
PubSubClient client(espClient);

// Timer untuk throttling pengiriman data (agar tidak spamming broker)
unsigned long lastMsg = 0;
const long interval = 5000; // Kirim data setiap 200ms

// ── Pin definitions ────────────────────────────────────────────
#define ECG_PIN 36
#define LEADS_OFF_P 32
#define LEADS_OFF_N 35

// ── GY521 ──────────────────────────────────────────────────────
GY521 sensor(0x68);

// ── AD8232 — Leads-off debounce ────────────────────────────────
const int LO_DEBOUNCE = 8;
int lo_p_count = LO_DEBOUNCE;
int lo_n_count = LO_DEBOUNCE;
bool lo_p_stable = true;
bool lo_n_stable = true;

void setup_wifi() {
  delay(10);
  Serial.print("\nConnecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
}

bool last_fall_state = false;

void updateLeadsDebounce(bool raw_p, bool raw_n)
{
  if (raw_p) lo_p_count = min(lo_p_count + 1, LO_DEBOUNCE);
  else        lo_p_count = max(lo_p_count - 1, 0);
  lo_p_stable = (lo_p_count >= LO_DEBOUNCE);

  if (raw_n) lo_n_count = min(lo_n_count + 1, LO_DEBOUNCE);
  else        lo_n_count = max(lo_n_count - 1, 0);
  lo_n_stable = (lo_n_count >= LO_DEBOUNCE);
}

// ── BPM detection ──────────────────────────────────────────────
const int BPM_HISTORY = 8;
const unsigned long REFRACTORY = 300;
const float ADAPTIVE_K = 0.55f;
const float OUTLIER_THR = 0.45f;

float peak_max = 30.0f;
float adapt_thresh = 50.0f;
unsigned long peak_max_reset_ms = 0;
float ecg_filtered;

float rr_intervals[BPM_HISTORY];
int rr_idx = 0;
bool rr_filled = false;

unsigned long last_peak_ms = 0;
float bpm = 0.0;
bool above_thresh = false;

// ── IIR band-pass filter ────────────────────────────────────────
struct Biquad {
  float b0, b1, b2, a1, a2;
  float z1 = 0, z2 = 0;
  
  Biquad(float b0, float b1, float b2, float a1, float a2)
    : b0(b0), b1(b1), b2(b2), a1(a1), a2(a2), z1(0), z2(0) {}

  float process(float x) {
    float y = b0 * x + z1;
    z1 = b1 * x - a1 * y + z2;
    z2 = b2 * x - a2 * y;
    return y;
  }
};

Biquad hpf = {0.9950f, -1.9900f, 0.9950f, -1.9900f, 0.9900f};
// Low-pass 15 Hz @ 50 Hz sample rate
Biquad lpf = {0.1367f, 0.2734f, 0.1367f, -0.6952f, 0.2420f};

// Add DC offset removal BEFORE the biquad filters
float filterECG(int rawValue) {
  static float dc = 2048.0f;
  dc = 0.99f * dc + 0.01f * rawValue;  // slow IIR DC tracker
  float x = rawValue - dc;             // remove DC first
  x = lpf.process(x);                  // only use LPF, skip HPF
  return x;
}

float medianOf(float *arr, int n) {
  float tmp[BPM_HISTORY];
  memcpy(tmp, arr, n * sizeof(float));
  for (int i = 0; i < n - 1; i++)
    for (int j = i + 1; j < n; j++)
      if (tmp[j] < tmp[i]) { float t = tmp[i]; tmp[i] = tmp[j]; tmp[j] = t; }
  return (n % 2) ? tmp[n / 2] : (tmp[n / 2 - 1] + tmp[n / 2]) * 0.5f;
}

float weightedBPM() {
  int count = rr_filled ? BPM_HISTORY : rr_idx;
  if (count == 0) return 0.0f;
  float sum_w = 0, sum_rr = 0;
  for (int i = 0; i < count; i++) {
    int idx = (rr_idx - count + i + BPM_HISTORY) % BPM_HISTORY;
    float w = (float)(i + 1);
    sum_rr += rr_intervals[idx] * w;
    sum_w += w;
  }
  return 60000.0f / (sum_rr / sum_w);
}

void detectPeak(float ecgFiltered) {
  unsigned long now = millis();
  if (now - peak_max_reset_ms > 2000) {
    peak_max = adapt_thresh / ADAPTIVE_K * 0.85f;
    peak_max_reset_ms = now;
  }
  if (ecgFiltered > peak_max) peak_max = ecgFiltered;
  adapt_thresh = peak_max * ADAPTIVE_K;

  if (ecgFiltered > adapt_thresh && !above_thresh) {
    above_thresh = true;
    unsigned long rr = now - last_peak_ms;
    if (last_peak_ms != 0 && rr > REFRACTORY && rr < 2000) {
      bool accept = true;
      int count = rr_filled ? BPM_HISTORY : rr_idx;
      if (count >= 3) {
        float med = medianOf(rr_intervals, count);
        if (fabsf((float)rr - med) / med > OUTLIER_THR) accept = false;
      }
      if (accept) {
        rr_intervals[rr_idx % BPM_HISTORY] = (float)rr;
        rr_idx++;
        if (rr_idx >= BPM_HISTORY) rr_filled = true;
        bpm = weightedBPM();
      }
    }
    last_peak_ms = now;
  } else if (ecgFiltered < adapt_thresh * 0.85f) {
    above_thresh = false;
  }
}

// ── Counter ────────────────────────────────────────────────────
uint32_t counter = 0;

// ══════════════════════════════════════════════════════════════
// ── EdgeNeuron — Fall Detection ────────────────────────────────
// ══════════════════════════════════════════════════════════════

#define N_FEATURES 47
#define FALL_THRESHOLD 0.48f
#define ML_WINDOW_SAMPLES 75
#define ML_STRIDE 37

const float scaler_mean[47] = {-0.005458f, 0.130513f, -0.383600f, 0.343179f, 0.726779f, 0.242416f, 0.698473f, 0.226846f, 0.259893f, 1.314582f, 1.054689f, 0.763832f, -0.112413f, 0.156352f, -0.583839f, 0.223832f, 0.807671f, 0.374166f, -0.072624f, 2.557644f, -6.650922f, 6.069404f, 12.720326f, 2.079424f, 0.264943f, 2.144202f, -4.511122f, 4.963250f, 9.474372f, 1.966867f, -0.031826f, 1.731597f, -3.953468f, 3.984985f, 7.938453f, 1.424057f, 1.380413f, 0.547857f, 2.440632f, 1.759238f, 1.048198f, 0.214967f, 5.470347f, 10.556951f, 3.801863f, 2.182314f, 0.096667f};
const float scaler_std[47] = {0.347560f, 0.182964f, 0.860372f, 0.751155f, 1.232485f, 0.283975f, 0.432986f, 0.333587f, 0.767390f, 1.186798f, 1.603372f, 0.352453f, 0.424153f, 0.189090f, 0.933041f, 0.679870f, 1.185326f, 0.261600f, 1.261780f, 3.650707f, 11.064667f, 9.599742f, 19.253976f, 2.748286f, 1.736150f, 2.660111f, 6.892009f, 6.645374f, 12.261871f, 2.321153f, 1.074681f, 2.380688f, 5.974141f, 6.323863f, 11.247587f, 1.952207f, 0.233045f, 1.224126f, 5.529463f, 1.429443f, 0.120236f, 0.320587f, 6.311061f, 14.174370f, 4.312091f, 2.869851f, 0.298456f};

// ── Tensor arena ───────────────────────────
constexpr int kTensorArenaSize = 60 * 1024;  // 60KB
alignas(16) uint8_t tensor_arena[kTensorArenaSize];

// ── Sliding window buffers ──────────────────────────────────────
float ml_ax[ML_WINDOW_SAMPLES], ml_ay[ML_WINDOW_SAMPLES], ml_az[ML_WINDOW_SAMPLES];
float ml_gx[ML_WINDOW_SAMPLES], ml_gy[ML_WINDOW_SAMPLES], ml_gz[ML_WINDOW_SAMPLES];
int ml_idx = 0;
bool ml_window_ready = false;

void ml_push_sample(float ax, float ay, float az, float gx, float gy, float gz) {
  ml_ax[ml_idx] = ax;  ml_ay[ml_idx] = ay;  ml_az[ml_idx] = az;
  ml_gx[ml_idx] = gx;  ml_gy[ml_idx] = gy;  ml_gz[ml_idx] = gz;
  ml_idx++;
  if (ml_idx >= ML_WINDOW_SAMPLES) {
    ml_window_ready = true;
    memmove(ml_ax, ml_ax + ML_STRIDE, (ML_WINDOW_SAMPLES - ML_STRIDE) * sizeof(float));
    memmove(ml_ay, ml_ay + ML_STRIDE, (ML_WINDOW_SAMPLES - ML_STRIDE) * sizeof(float));
    memmove(ml_az, ml_az + ML_STRIDE, (ML_WINDOW_SAMPLES - ML_STRIDE) * sizeof(float));
    memmove(ml_gx, ml_gx + ML_STRIDE, (ML_WINDOW_SAMPLES - ML_STRIDE) * sizeof(float));
    memmove(ml_gy, ml_gy + ML_STRIDE, (ML_WINDOW_SAMPLES - ML_STRIDE) * sizeof(float));
    memmove(ml_gz, ml_gz + ML_STRIDE, (ML_WINDOW_SAMPLES - ML_STRIDE) * sizeof(float));
    ml_idx = ML_WINDOW_SAMPLES - ML_STRIDE;
  }
}

void axis_stats(float* buf, int n, float& mean, float& std_out, float& mn, float& mx,
                float& range, float& abs_mean) {
  mean = 0; abs_mean = 0; mn = buf[0]; mx = buf[0];
  for (int i = 0; i < n; i++) {
    mean += buf[i]; abs_mean += fabsf(buf[i]);
    if (buf[i] < mn) mn = buf[i];
    if (buf[i] > mx) mx = buf[i];
  }
  mean /= n; abs_mean /= n; range = mx - mn;
  std_out = 0;
  for (int i = 0; i < n; i++) std_out += (buf[i] - mean) * (buf[i] - mean);
  std_out = sqrtf(std_out / n);
}

void extract_features(float* features) {
  float ax_mean, ax_std, ax_min, ax_max, ax_range, ax_abs_mean;
  float ay_mean, ay_std, ay_min, ay_max, ay_range, ay_abs_mean;
  float az_mean, az_std, az_min, az_max, az_range, az_abs_mean;

  axis_stats(ml_ax, ML_WINDOW_SAMPLES, ax_mean, ax_std, ax_min, ax_max, ax_range, ax_abs_mean);
  axis_stats(ml_ay, ML_WINDOW_SAMPLES, ay_mean, ay_std, ay_min, ay_max, ay_range, ay_abs_mean);
  axis_stats(ml_az, ML_WINDOW_SAMPLES, az_mean, az_std, az_min, az_max, az_range, az_abs_mean);

  float res[ML_WINDOW_SAMPLES];
  for (int i = 0; i < ML_WINDOW_SAMPLES; i++)
    res[i] = sqrtf(ml_ax[i]*ml_ax[i] + ml_ay[i]*ml_ay[i] + ml_az[i]*ml_az[i]);

  float sma = 0;
  for (int i = 0; i < ML_WINDOW_SAMPLES; i++)
    sma += fabsf(ml_ax[i]) + fabsf(ml_ay[i]) + fabsf(ml_az[i]);
  sma /= ML_WINDOW_SAMPLES;

  float res_mean = 0, res_std = 0, res_peak = 0;
  for (int i = 0; i < ML_WINDOW_SAMPLES; i++) {
    res_mean += res[i];
    if (res[i] > res_peak) res_peak = res[i];
  }
  res_mean /= ML_WINDOW_SAMPLES;
  for (int i = 0; i < ML_WINDOW_SAMPLES; i++)
    res_std += (res[i] - res_mean) * (res[i] - res_mean);
  res_std = sqrtf(res_std / ML_WINDOW_SAMPLES);

  float res_skew = 0, res_kurt = 0;
  for (int i = 0; i < ML_WINDOW_SAMPLES; i++) {
    float norm = (res[i] - res_mean) / (res_std + 1e-9f);
    res_skew += powf(norm, 3);
    res_kurt += powf(norm, 4);
  }
  res_skew /= ML_WINDOW_SAMPLES;
  res_kurt /= ML_WINDOW_SAMPLES;

  features[0]=ax_mean;   features[1]=ax_std;    features[2]=ax_min;    features[3]=ax_max;
  features[4]=ax_range;  features[5]=ax_abs_mean;
  features[6]=ay_mean;   features[7]=ay_std;    features[8]=ay_min;    features[9]=ay_max;
  features[10]=ay_range; features[11]=ay_abs_mean;
  features[12]=az_mean;  features[13]=az_std;   features[14]=az_min;   features[15]=az_max;
  features[16]=az_range; features[17]=az_abs_mean;
  features[18]=sma;
  features[19]=res_skew; features[20]=res_kurt;
  features[21]=res_peak; features[22]=res_mean; features[23]=res_std;

  // ── Gyro per-axis stats (mirror accel pattern) ─────────────
  float gx_mean, gx_std, gx_min, gx_max, gx_range, gx_abs_mean;
  float gy_mean, gy_std, gy_min, gy_max, gy_range, gy_abs_mean;
  float gz_mean, gz_std, gz_min, gz_max, gz_range, gz_abs_mean;

  axis_stats(ml_gx, ML_WINDOW_SAMPLES, gx_mean, gx_std, gx_min, gx_max, gx_range, gx_abs_mean);
  axis_stats(ml_gy, ML_WINDOW_SAMPLES, gy_mean, gy_std, gy_min, gy_max, gy_range, gy_abs_mean);
  axis_stats(ml_gz, ML_WINDOW_SAMPLES, gz_mean, gz_std, gz_min, gz_max, gz_range, gz_abs_mean);

  features[24]=gx_mean;  features[25]=gx_std;   features[26]=gx_min;   features[27]=gx_max;
  features[28]=gx_range; features[29]=gx_abs_mean;
  features[30]=gy_mean;  features[31]=gy_std;   features[32]=gy_min;   features[33]=gy_max;
  features[34]=gy_range; features[35]=gy_abs_mean;
  features[36]=gz_mean;  features[37]=gz_std;   features[38]=gz_min;   features[39]=gz_max;
  features[40]=gz_range; features[41]=gz_abs_mean;

  // ── Gyro combined ──────────────────────────────────────────
  float gyro_res[ML_WINDOW_SAMPLES];
  for (int i = 0; i < ML_WINDOW_SAMPLES; i++)
    gyro_res[i] = sqrtf(ml_gx[i]*ml_gx[i] + ml_gy[i]*ml_gy[i] + ml_gz[i]*ml_gz[i]);

  float gyro_sma = 0, gyro_peak = 0, gyro_mean = 0, gyro_std = 0;
  for (int i = 0; i < ML_WINDOW_SAMPLES; i++) {
    gyro_sma  += fabsf(ml_gx[i]) + fabsf(ml_gy[i]) + fabsf(ml_gz[i]);
    gyro_mean += gyro_res[i];
    if (gyro_res[i] > gyro_peak) gyro_peak = gyro_res[i];
  }
  gyro_sma  /= ML_WINDOW_SAMPLES;
  gyro_mean /= ML_WINDOW_SAMPLES;
  for (int i = 0; i < ML_WINDOW_SAMPLES; i++)
    gyro_std += (gyro_res[i] - gyro_mean) * (gyro_res[i] - gyro_mean);
  gyro_std = sqrtf(gyro_std / ML_WINDOW_SAMPLES);

  features[42]=gyro_sma;
  features[43]=gyro_peak;
  features[44]=gyro_mean;
  features[45]=gyro_std;

  // ── Cross-sensor correlation ───────────────────────────────
  float cov = 0, std_a = 0, std_g = 0;
  for (int i = 0; i < ML_WINDOW_SAMPLES; i++) {
    cov   += (res[i] - res_mean) * (gyro_res[i] - gyro_mean);
    std_a += (res[i] - res_mean) * (res[i] - res_mean);
    std_g += (gyro_res[i] - gyro_mean) * (gyro_res[i] - gyro_mean);
  }
  features[46] = cov / (sqrtf(std_a * std_g) + 1e-9f);
}

void run_inference() {
  float features[N_FEATURES];
  extract_features(features);

  // Normalize and feed inputs
  for (int i = 0; i < N_FEATURES; i++) {
    float normalized = (features[i] - scaler_mean[i]) / scaler_std[i];
    setModelInput(normalized, i);
  }

  if (!runModelInference()) {
    Serial.println("Inference failed!");
    return;
  }

  global_fall_prob = getModelOutput(0);
  global_is_fall = global_fall_prob < FALL_THRESHOLD;

  Serial.print("FALL_PROB:");
  Serial.print(global_fall_prob, 3);
  Serial.print("\tFALL:");
  Serial.println(global_is_fall ? 1 : 0);
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP32-";
    clientId += String(WiFi.macAddress());

    if (client.connect(clientId.c_str())) {  // no username/password
      Serial.println("connected to Antares");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" retry in 5s");
      delay(5000);
    }
  }
}

void sendSensorData(float fall_prob, bool fall_status, int bpm) {
  StaticJsonDocument<128> inner;
  inner["fall_prob"]       = fall_prob;
  inner["fall_status"]     = fall_status ? "FALL" : "SAFE";
  inner["bpm"]             = bpm;
  char con_buf[128];
  serializeJson(inner, con_buf);

  StaticJsonDocument<512> doc;
  doc["m2m:rqp"]["op"]  = 1;
  doc["m2m:rqp"]["to"]  = "/antares-cse/antares-id/PA-IOT/fall-00";
  doc["m2m:rqp"]["fr"]  = antares_access_key;
  doc["m2m:rqp"]["rqi"] = String(millis()).c_str();
  doc["m2m:rqp"]["ty"]  = 4;
  doc["m2m:rqp"]["pc"]["m2m:cin"]["cnf"] = "message";
  doc["m2m:rqp"]["pc"]["m2m:cin"]["con"] = con_buf;

  char buffer[512];
  serializeJson(doc, buffer);
  client.publish(mqtt_topic, buffer);
}

void triggerEmergencyLED(bool is_falling) {
  client.publish(led_topic, is_falling ? "ON" : "OFF");
}

// ══════════════════════════════════════════════════════════════
void setup() {
  // Matikan fitur hemat daya WiFi untuk stabilitas pengiriman data sensor
  WiFi.setSleep(false);

  btStop();
  delay(1000);

  Serial.begin(115200);
  Wire.begin();
  delay(200);

  while (sensor.wakeup() == false) {
    Serial.println("Could not connect to GY521...");
    delay(1000);
  }
  sensor.setAccelSensitivity(3);
  sensor.setGyroSensitivity(0);
  sensor.setThrottle();

  pinMode(LEADS_OFF_P, INPUT_PULLUP);
  pinMode(LEADS_OFF_N, INPUT);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  delay(100);

  snprintf(mqtt_topic, sizeof(mqtt_topic),
  "/oneM2M/req/%s/antares-cse/json", antares_access_key);
  snprintf(led_topic, sizeof(led_topic),
  "/oneM2M/req/%s/%s/emergency_led", antares_access_key, antares_project, antares_device);

  Serial.print("LO_N_RAW:"); Serial.println(digitalRead(LEADS_OFF_N));
  Serial.print("LO_P_RAW:"); Serial.println(digitalRead(LEADS_OFF_P));
  Serial.print("ADC Sanity Check:"); Serial.println(analogRead(ECG_PIN));

  delay(5000);

  for (int i = 0; i < BPM_HISTORY; i++) rr_intervals[i] = 0;

  // ── EdgeNeuron init ──
  // Replace 'fall_detection_model' with whatever your model.h array is named
  if (!initializeModel(fall_detection_model, tensor_arena, kTensorArenaSize)) {
    Serial.println("EdgeNeuron init failed!");
    while (true);
  }

  Serial.println("ESP32 GY521+AD8232 ready");
  Serial.println("counter\tax\tay\taz\tgx\tgy\tgz\ttemp\tecg\tbpm\tlo_p\tlo_n\tleads_off");

  client.setBufferSize(1024);

  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
}

// ══════════════════════════════════════════════════════════════
void loop() {
if (!client.connected()) {
    reconnect();
  }
  client.loop(); // Menangani proses background MQTT

  sensor.read();
  float raw_ax = sensor.getAccelX();
  float raw_ay = sensor.getAccelY();
  float raw_az = sensor.getAccelZ();

  float gx = sensor.getGyroX();
  float gy = sensor.getGyroY();
  float gz = sensor.getGyroZ();
  float t = sensor.getTemperature();

  ml_push_sample(raw_ax, raw_ay, raw_az, gx, gy, gz);
  if (ml_window_ready) {
    run_inference();
    ml_window_ready = false;
  }

  bool raw_lo_p = (digitalRead(LEADS_OFF_P) == HIGH);
  int lo_n_reads = 0;
  for (int i = 0; i < 3; i++) lo_n_reads += digitalRead(LEADS_OFF_N);
  bool raw_lo_n = (lo_n_reads >= 2);

  updateLeadsDebounce(raw_lo_p, raw_lo_n);
  bool leads_off = lo_p_stable || lo_n_stable;

  int ecg_raw = 0;
  if (!leads_off) {
    ecg_raw = analogRead(ECG_PIN);
    ecg_filtered = filterECG(ecg_raw);
    detectPeak(ecg_filtered);
  } else {
    if (lo_p_count >= LO_DEBOUNCE && lo_n_count >= LO_DEBOUNCE) {
      bpm = 0.0; rr_idx = 0; last_peak_ms = 0;
    }
  }

  Serial.print(counter);   Serial.print('\t');
  Serial.print(raw_ax, 3); Serial.print('\t');
  Serial.print(raw_ay, 3); Serial.print('\t');
  Serial.print(raw_az, 3); Serial.print('\t');
  Serial.print(gx, 3);     Serial.print('\t');
  Serial.print(gy, 3);     Serial.print('\t');
  Serial.print(gz, 3);     Serial.print('\t');
  Serial.print(t, 2);      Serial.print('\t');
  Serial.print(ecg_raw);   Serial.print('\t');
  Serial.print(bpm, 1);    Serial.print('\t');
  Serial.print(lo_p_stable ? 1 : 0); Serial.print('\t');
  Serial.print(lo_n_stable ? 1 : 0); Serial.print('\t');
  Serial.println(leads_off ? 1 : 0);

  counter++;

  // Ambil hasil inferensi fall detection
  // Misalkan variabel 'global_is_fall' diupdate di run_inference()
  bool current_fall_state = (getModelOutput(0) < FALL_THRESHOLD);

  // Throttling: Kirim data tanpa memblokir loop utama
  unsigned long now = millis();
  if (now - lastMsg > interval) {
    lastMsg = now;
    
    // Kirim data hanya jika leads terpasang (lo_p_stable & lo_n_stable FALSE)
    if (!leads_off) {
      sendSensorData(global_fall_prob, global_is_fall, bpm);
    } else {
      // Kirim status 'LEADS_OFF' jika sensor tidak menempel di tubuh
      StaticJsonDocument<100> doc;
      doc["status"] = "DISCONNECTED";
      char buffer[100];
      serializeJson(doc, buffer);
      client.publish(mqtt_topic, buffer);
    }
  }

  // Delay minimal untuk stabilitas sampling rate (20ms = 50Hz)
  delay(20);}