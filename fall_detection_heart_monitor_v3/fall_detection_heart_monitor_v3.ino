// ================================================================
//  ESP32 — GY521 (MPU-6050) + AD8232 (ECG)
//  Versi: Fall Confirmation + Posture Detection + Smart Buzzer
// ================================================================

#include "GY521.h"
#include <Wire.h>
#include "WiFi.h"
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ── EdgeNeuron TinyML ───────────────────────────────────────────
#include <EdgeNeuron.h>
#include "model.h"

// ── Konfigurasi WiFi & MQTT ────────────────────────────────────
const char* ssid     = "Ridho Ruijie";
const char* password = "ridhosetiawanskom24";

const char* mqtt_server = "broker.hivemq.com";
const int   mqtt_port   = 1883;

WiFiClient   espClient;
PubSubClient client(espClient);

// ── Pin definitions ────────────────────────────────────────────
#define ECG_PIN      36
#define LEADS_OFF_P  32
#define LEADS_OFF_N  35

// ── GY521 ──────────────────────────────────────────────────────
GY521 sensor(0x68);

// ── Timing ─────────────────────────────────────────────────────
unsigned long lastMsg = 0;
const long    interval = 100;   // publish setiap 100ms (10 Hz)

enum Posture { POSTURE_STANDING, POSTURE_SITTING, POSTURE_LYING, POSTURE_UNKNOWN };

Posture current_posture = POSTURE_UNKNOWN;

// Rata-rata akselerometer dalam window ML terakhir (diisi saat extract_features)
float mean_ax_global = 0, mean_ay_global = 0, mean_az_global = 0;

// ── AD8232 — Leads-off debounce ────────────────────────────────
const int LO_DEBOUNCE = 8;
int  lo_p_count  = LO_DEBOUNCE, lo_n_count  = LO_DEBOUNCE;
bool lo_p_stable = true,        lo_n_stable  = true;

void updateLeadsDebounce(bool raw_p, bool raw_n) {
  lo_p_count = raw_p ? min(lo_p_count + 1, LO_DEBOUNCE) : max(lo_p_count - 1, 0);
  lo_n_count = raw_n ? min(lo_n_count + 1, LO_DEBOUNCE) : max(lo_n_count - 1, 0);
  lo_p_stable = (lo_p_count >= LO_DEBOUNCE);
  lo_n_stable = (lo_n_count >= LO_DEBOUNCE);
}

// ── BPM & HRV detection ────────────────────────────────────────
const int          BPM_HISTORY  = 8;
const unsigned long REFRACTORY  = 300;
const float        ADAPTIVE_K   = 0.55f;
const float        OUTLIER_THR  = 0.45f;

float         peak_max          = 30.0f;
float         adapt_thresh      = 50.0f;
unsigned long peak_max_reset_ms = 0;
float         ecg_filtered      = 0;

float         rr_intervals[BPM_HISTORY] = {0};
int           rr_idx    = 0;
bool          rr_filled = false;
unsigned long last_peak_ms = 0;
float         bpm          = 0.0f;
bool          above_thresh = false;

// ── IIR band-pass filter ────────────────────────────────────────
struct Biquad {
  float b0, b1, b2, a1, a2, z1 = 0, z2 = 0;
  Biquad(float b0,float b1,float b2,float a1,float a2)
    : b0(b0),b1(b1),b2(b2),a1(a1),a2(a2) {}
  float process(float x) {
    float y = b0*x + z1;
    z1 = b1*x - a1*y + z2;
    z2 = b2*x - a2*y;
    return y;
  }
};
Biquad hpf = {0.9950f,-1.9900f,0.9950f,-1.9900f,0.9900f};
Biquad lpf = {0.1367f, 0.2734f,0.1367f,-0.6952f,0.2420f};

float filterECG(int rawValue) {
  static float dc = 2048.0f;
  dc = 0.99f * dc + 0.01f * rawValue;
  float x = rawValue - dc;
  return lpf.process(x);
}

float medianOf(float *arr, int n) {
  float tmp[BPM_HISTORY];
  memcpy(tmp, arr, n * sizeof(float));
  for (int i = 0; i < n-1; i++)
    for (int j = i+1; j < n; j++)
      if (tmp[j] < tmp[i]) { float t=tmp[i]; tmp[i]=tmp[j]; tmp[j]=t; }
  return (n%2) ? tmp[n/2] : (tmp[n/2-1]+tmp[n/2])*0.5f;
}

float weightedBPM() {
  int count = rr_filled ? BPM_HISTORY : rr_idx;
  if (count == 0) return 0.0f;
  float sum_w=0, sum_rr=0;
  for (int i=0; i<count; i++) {
    int idx = (rr_idx - count + i + BPM_HISTORY) % BPM_HISTORY;
    float w = (float)(i+1);
    sum_rr += rr_intervals[idx]*w; sum_w += w;
  }
  return 60000.0f / (sum_rr/sum_w);
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
        if (fabsf((float)rr - med)/med > OUTLIER_THR) accept = false;
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

// ══════════════════════════════════════════════════════════════
// ── HRV Anomaly Detection ──────────────────────────────────────
// ══════════════════════════════════════════════════════════════
float computeRMSSD() {
  int count = rr_filled ? BPM_HISTORY : rr_idx;
  if (count < 3) return 0.0f;
  float sumSq = 0;
  int   pairs = 0;
  for (int i = 1; i < count; i++) {
    int iA = (rr_idx - count + i - 1 + BPM_HISTORY) % BPM_HISTORY;
    int iB = (rr_idx - count + i     + BPM_HISTORY) % BPM_HISTORY;
    float diff = rr_intervals[iB] - rr_intervals[iA];
    sumSq += diff * diff;
    pairs++;
  }
  return (pairs > 0) ? sqrtf(sumSq / pairs) : 0.0f;
}

bool isBpmAbnormal() {
  if (bpm == 0) return false;
  return (bpm < 50.0f || bpm > 120.0f);
}
bool isHrvAnomaly() {
  float rmssd = computeRMSSD();
  return (rmssd > 120.0f || rmssd < 30.0f);
}

// ══════════════════════════════════════════════════════════════
// ── Posture Detection dari Akselerometer ──────────────────────
// ══════════════════════════════════════════════════════════════
Posture detectPosture(float ax, float ay, float az) {
  float mag = sqrtf(ax*ax + ay*ay + az*az);
  if (mag < 0.3f) return POSTURE_UNKNOWN;
  float nx = ax/mag, ny = ay/mag, nz = az/mag;

  float absNz = fabsf(nz);
  float absNy = fabsf(ny);
  float absNx = fabsf(nx);

  if (absNz >= 0.85f) {
    return POSTURE_STANDING;
  } else if (absNy >= 0.80f || absNx >= 0.80f) {
    return POSTURE_LYING;
  } else if (absNz >= 0.50f) {
    return POSTURE_SITTING;
  }
  return POSTURE_UNKNOWN;
}

const char* postureString(Posture p) {
  switch (p) {
    case POSTURE_STANDING: return "STANDING";
    case POSTURE_SITTING:  return "SITTING";
    case POSTURE_LYING:    return "LYING";
    default:               return "UNKNOWN";
  }
}

// ══════════════════════════════════════════════════════════════
// ── Fall Confirmation — debounce false positive ────────────────
// ══════════════════════════════════════════════════════════════
#define FALL_CONFIRM_WINDOW  5
#define FALL_CONFIRM_MIN     3
#define FALL_HOLD_MS      4000

bool fall_history[FALL_CONFIRM_WINDOW] = {false};
int  fall_hist_idx = 0;

unsigned long fall_confirmed_at = 0;
bool          fall_confirmed     = false;

void updateFallHistory(bool raw_fall) {
  fall_history[fall_hist_idx % FALL_CONFIRM_WINDOW] = raw_fall;
  fall_hist_idx++;

  int fall_count = 0;
  for (int i = 0; i < FALL_CONFIRM_WINDOW; i++)
    if (fall_history[i]) fall_count++;

  bool newly_confirmed = (fall_count >= FALL_CONFIRM_MIN);

  if (newly_confirmed) {
    if (!fall_confirmed) fall_confirmed_at = millis();
    fall_confirmed = true;
  } else {
    if (fall_confirmed && (millis() - fall_confirmed_at > FALL_HOLD_MS)) {
      if (current_posture != POSTURE_LYING) {
        fall_confirmed = false;
      }
    }
  }
}

// ══════════════════════════════════════════════════════════════
// ── Smart Status Logic ────────────────────────────────────────
// Menentukan status led dan buzzer untuk dikirim via MQTT
// ══════════════════════════════════════════════════════════════
bool led_on    = false;
bool buzzer_on = false;
char buzzer_reason[32] = "NONE";

void updateStatus() {
  if (fall_confirmed) {
    led_on = true;

    bool bpmAbnormal = isBpmAbnormal();
    bool hrvAnomaly  = isHrvAnomaly();

    if (bpmAbnormal || hrvAnomaly) {
      buzzer_on = true;

      if (bpm < 60.0f && bpm > 0)       snprintf(buzzer_reason, sizeof(buzzer_reason), "BPM_LOW:%.0f", bpm);
      else if (bpm > 120.0f)             snprintf(buzzer_reason, sizeof(buzzer_reason), "BPM_HIGH:%.0f", bpm);
      else if (hrvAnomaly)               snprintf(buzzer_reason, sizeof(buzzer_reason), "HRV:%.0f", computeRMSSD());
    } else {
      buzzer_on = false;
      snprintf(buzzer_reason, sizeof(buzzer_reason), "NONE");
    }
  } else {
    led_on    = false;
    buzzer_on = false;
    snprintf(buzzer_reason, sizeof(buzzer_reason), "NONE");
  }
}

// ══════════════════════════════════════════════════════════════
// ── EdgeNeuron TinyML ──────────════════════════════════════════
// ══════════════════════════════════════════════════════════════
#define N_FEATURES         47
#define FALL_THRESHOLD     0.48f
#define ML_WINDOW_SAMPLES  75
#define ML_STRIDE          37

const float scaler_mean[47] = {-0.005458f,0.130513f,-0.383600f,0.343179f,0.726779f,0.242416f,0.698473f,0.226846f,0.259893f,1.314582f,1.054689f,0.763832f,-0.112413f,0.156352f,-0.583839f,0.223832f,0.807671f,0.374166f,-0.072624f,2.557644f,-6.650922f,6.069404f,12.720326f,2.079424f,0.264943f,2.144202f,-4.511122f,4.963250f,9.474372f,1.966867f,-0.031826f,1.731597f,-3.953468f,3.984985f,7.938453f,1.424057f,1.380413f,0.547857f,2.440632f,1.759238f,1.048198f,0.214967f,5.470347f,10.556951f,3.801863f,2.182314f,0.096667f};
const float scaler_std[47]  = {0.347560f,0.182964f,0.860372f,0.751155f,1.232485f,0.283975f,0.432986f,0.333587f,0.767390f,1.186798f,1.603372f,0.352453f,0.424153f,0.189090f,0.933041f,0.679870f,1.185326f,0.261600f,1.261780f,3.650707f,11.064667f,9.599742f,19.253976f,2.748286f,1.736150f,2.660111f,6.892009f,6.645374f,12.261871f,2.321153f,1.074681f,2.380688f,5.974141f,6.323863f,11.247587f,1.952207f,0.233045f,1.224126f,5.529463f,1.429443f,0.120236f,0.320587f,6.311061f,14.174370f,4.312091f,2.869851f,0.298456f};

constexpr int kTensorArenaSize = 60 * 1024;
alignas(16) uint8_t tensor_arena[kTensorArenaSize];

float ml_ax[ML_WINDOW_SAMPLES], ml_ay[ML_WINDOW_SAMPLES], ml_az[ML_WINDOW_SAMPLES];
float ml_gx[ML_WINDOW_SAMPLES], ml_gy[ML_WINDOW_SAMPLES], ml_gz[ML_WINDOW_SAMPLES];
int  ml_idx          = 0;
bool ml_window_ready = false;

float global_fall_prob = 0.0f;
bool  global_is_fall   = false;

void ml_push_sample(float ax, float ay, float az, float gx, float gy, float gz) {
  ml_ax[ml_idx]=ax; ml_ay[ml_idx]=ay; ml_az[ml_idx]=az;
  ml_gx[ml_idx]=gx; ml_gy[ml_idx]=gy; ml_gz[ml_idx]=gz;
  ml_idx++;
  if (ml_idx >= ML_WINDOW_SAMPLES) {
    ml_window_ready = true;
    memmove(ml_ax, ml_ax+ML_STRIDE, (ML_WINDOW_SAMPLES-ML_STRIDE)*sizeof(float));
    memmove(ml_ay, ml_ay+ML_STRIDE, (ML_WINDOW_SAMPLES-ML_STRIDE)*sizeof(float));
    memmove(ml_az, ml_az+ML_STRIDE, (ML_WINDOW_SAMPLES-ML_STRIDE)*sizeof(float));
    memmove(ml_gx, ml_gx+ML_STRIDE, (ML_WINDOW_SAMPLES-ML_STRIDE)*sizeof(float));
    memmove(ml_gy, ml_gy+ML_STRIDE, (ML_WINDOW_SAMPLES-ML_STRIDE)*sizeof(float));
    memmove(ml_gz, ml_gz+ML_STRIDE, (ML_WINDOW_SAMPLES-ML_STRIDE)*sizeof(float));
    ml_idx = ML_WINDOW_SAMPLES - ML_STRIDE;
  }
}

void axis_stats(float* buf, int n, float& mean, float& std_out, float& mn, float& mx,
                float& range, float& abs_mean) {
  mean=0; abs_mean=0; mn=buf[0]; mx=buf[0];
  for (int i=0;i<n;i++) { mean+=buf[i]; abs_mean+=fabsf(buf[i]); if(buf[i]<mn)mn=buf[i]; if(buf[i]>mx)mx=buf[i]; }
  mean/=n; abs_mean/=n; range=mx-mn;
  std_out=0;
  for (int i=0;i<n;i++) std_out+=(buf[i]-mean)*(buf[i]-mean);
  std_out=sqrtf(std_out/n);
}

void extract_features(float* features) {
  float ax_mean,ax_std,ax_min,ax_max,ax_range,ax_abs_mean;
  float ay_mean,ay_std,ay_min,ay_max,ay_range,ay_abs_mean;
  float az_mean,az_std,az_min,az_max,az_range,az_abs_mean;

  axis_stats(ml_ax,ML_WINDOW_SAMPLES,ax_mean,ax_std,ax_min,ax_max,ax_range,ax_abs_mean);
  axis_stats(ml_ay,ML_WINDOW_SAMPLES,ay_mean,ay_std,ay_min,ay_max,ay_range,ay_abs_mean);
  axis_stats(ml_az,ML_WINDOW_SAMPLES,az_mean,az_std,az_min,az_max,az_range,az_abs_mean);

  mean_ax_global = ax_mean; mean_ay_global = ay_mean; mean_az_global = az_mean;

  float res[ML_WINDOW_SAMPLES];
  for (int i=0;i<ML_WINDOW_SAMPLES;i++)
    res[i]=sqrtf(ml_ax[i]*ml_ax[i]+ml_ay[i]*ml_ay[i]+ml_az[i]*ml_az[i]);

  float sma=0;
  for (int i=0;i<ML_WINDOW_SAMPLES;i++) sma+=fabsf(ml_ax[i])+fabsf(ml_ay[i])+fabsf(ml_az[i]);
  sma/=ML_WINDOW_SAMPLES;

  float res_mean=0,res_std=0,res_peak=0;
  for (int i=0;i<ML_WINDOW_SAMPLES;i++) { res_mean+=res[i]; if(res[i]>res_peak)res_peak=res[i]; }
  res_mean/=ML_WINDOW_SAMPLES;
  for (int i=0;i<ML_WINDOW_SAMPLES;i++) res_std+=(res[i]-res_mean)*(res[i]-res_mean);
  res_std=sqrtf(res_std/ML_WINDOW_SAMPLES);

  float res_skew=0,res_kurt=0;
  for (int i=0;i<ML_WINDOW_SAMPLES;i++) {
    float norm=(res[i]-res_mean)/(res_std+1e-9f);
    res_skew+=powf(norm,3); res_kurt+=powf(norm,4);
  }
  res_skew/=ML_WINDOW_SAMPLES; res_kurt/=ML_WINDOW_SAMPLES;

  features[0]=ax_mean;  features[1]=ax_std;   features[2]=ax_min;   features[3]=ax_max;
  features[4]=ax_range; features[5]=ax_abs_mean;
  features[6]=ay_mean;  features[7]=ay_std;   features[8]=ay_min;   features[9]=ay_max;
  features[10]=ay_range;features[11]=ay_abs_mean;
  features[12]=az_mean; features[13]=az_std;  features[14]=az_min;  features[15]=az_max;
  features[16]=az_range;features[17]=az_abs_mean;
  features[18]=sma;
  features[19]=res_skew;features[20]=res_kurt;
  features[21]=res_peak;features[22]=res_mean;features[23]=res_std;

  float gx_mean,gx_std,gx_min,gx_max,gx_range,gx_abs_mean;
  float gy_mean,gy_std,gy_min,gy_max,gy_range,gy_abs_mean;
  float gz_mean,gz_std,gz_min,gz_max,gz_range,gz_abs_mean;
  axis_stats(ml_gx,ML_WINDOW_SAMPLES,gx_mean,gx_std,gx_min,gx_max,gx_range,gx_abs_mean);
  axis_stats(ml_gy,ML_WINDOW_SAMPLES,gy_mean,gy_std,gy_min,gy_max,gy_range,gy_abs_mean);
  axis_stats(ml_gz,ML_WINDOW_SAMPLES,gz_mean,gz_std,gz_min,gz_max,gz_range,gz_abs_mean);

  features[24]=gx_mean; features[25]=gx_std;  features[26]=gx_min;  features[27]=gx_max;
  features[28]=gx_range;features[29]=gx_abs_mean;
  features[30]=gy_mean; features[31]=gy_std;  features[32]=gy_min;  features[33]=gy_max;
  features[34]=gy_range;features[35]=gy_abs_mean;
  features[36]=gz_mean; features[37]=gz_std;  features[38]=gz_min;  features[39]=gz_max;
  features[40]=gz_range;features[41]=gz_abs_mean;

  float gyro_res[ML_WINDOW_SAMPLES];
  for (int i=0;i<ML_WINDOW_SAMPLES;i++)
    gyro_res[i]=sqrtf(ml_gx[i]*ml_gx[i]+ml_gy[i]*ml_gy[i]+ml_gz[i]*ml_gz[i]);

  float gyro_sma=0,gyro_peak=0,gyro_mean=0,gyro_std=0;
  for (int i=0;i<ML_WINDOW_SAMPLES;i++) {
    gyro_sma+=fabsf(ml_gx[i])+fabsf(ml_gy[i])+fabsf(ml_gz[i]);
    gyro_mean+=gyro_res[i];
    if(gyro_res[i]>gyro_peak)gyro_peak=gyro_res[i];
  }
  gyro_sma/=ML_WINDOW_SAMPLES; gyro_mean/=ML_WINDOW_SAMPLES;
  for (int i=0;i<ML_WINDOW_SAMPLES;i++)
    gyro_std+=(gyro_res[i]-gyro_mean)*(gyro_res[i]-gyro_mean);
  gyro_std=sqrtf(gyro_std/ML_WINDOW_SAMPLES);

  features[42]=gyro_sma; features[43]=gyro_peak; features[44]=gyro_mean; features[45]=gyro_std;

  float cov=0,std_a=0,std_g=0;
  for (int i=0;i<ML_WINDOW_SAMPLES;i++) {
    cov  +=(res[i]-res_mean)*(gyro_res[i]-gyro_mean);
    std_a+=(res[i]-res_mean)*(res[i]-res_mean);
    std_g+=(gyro_res[i]-gyro_mean)*(gyro_res[i]-gyro_mean);
  }
  features[46]=cov/(sqrtf(std_a*std_g)+1e-9f);
}

void run_inference() {
  float features[N_FEATURES];
  extract_features(features);

  for (int i=0;i<N_FEATURES;i++) {
    float normalized = (features[i]-scaler_mean[i])/scaler_std[i];
    setModelInput(normalized, i);
  }
  if (!runModelInference()) { Serial.println("Inference failed!"); return; }

  global_fall_prob = getModelOutput(0);
  global_is_fall   = global_fall_prob < FALL_THRESHOLD;

  current_posture = detectPosture(mean_ax_global, mean_ay_global, mean_az_global);

  updateFallHistory(global_is_fall);
  updateStatus();

  Serial.printf("FALL_PROB:%.3f\tFALL_RAW:%d\tFALL_CONFIRMED:%d\tPOSTURE:%s\tBPM:%.1f\tRMSSD:%.1f\n",
    global_fall_prob, global_is_fall ? 1 : 0,
    fall_confirmed ? 1 : 0,
    postureString(current_posture),
    bpm, computeRMSSD());
}

// ── MQTT ───────────────────────────────────────────────────────
void setup_wifi() {
  delay(10);
  Serial.print("\nConnecting to "); Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nWiFi connected");
}

void reconnect() {
  while (!client.connected()) {
    String clientId = "ESP32-" + String(WiFi.macAddress());
    if (client.connect(clientId.c_str())) {
      Serial.println("Connected to MQTT broker");
    } else {
      Serial.printf("Failed rc=%d, retry in 5s\n", client.state());
      delay(5000);
    }
  }
}

void publishAll(int ecg_raw, bool leads_off) {
  char buf[32];

  if (!leads_off) {
    client.publish("kel1/pa/fallStatus", fall_confirmed ? "FALL" : "SAFE");

    snprintf(buf, sizeof(buf), "%.3f", global_fall_prob);
    client.publish("kel1/pa/fallProb", buf);

    snprintf(buf, sizeof(buf), "%.1f", bpm);
    client.publish("kel1/pa/heartRateStatus", buf);

    snprintf(buf, sizeof(buf), "%.1f", computeRMSSD());
    client.publish("kel1/pa/hrv", buf);

    client.publish("kel1/pa/posture", postureString(current_posture));

    snprintf(buf, sizeof(buf), "%d", ecg_raw);
    client.publish("kel1/pa/ecgRaw", buf);

    client.publish("kel1/pa/ledStatus",    led_on    ? "ON" : "OFF");
    client.publish("kel1/pa/buzzerStatus", buzzer_on ? "ON" : "OFF");
    client.publish("kel1/pa/buzzerReason", buzzer_reason);

  } else {
    client.publish("kel1/pa/fallStatus",      "DISCONNECTED");
    client.publish("kel1/pa/heartRateStatus", "0");
    client.publish("kel1/pa/ecgRaw",          "0");
    client.publish("kel1/pa/hrv",             "0");
    client.publish("kel1/pa/posture",         "UNKNOWN");
    client.publish("kel1/pa/ledStatus",       "OFF");
    client.publish("kel1/pa/buzzerStatus",    "OFF");
    client.publish("kel1/pa/buzzerReason",    "NONE");
  }
}

// ── Counter ────────────────────────────────────────────────────
uint32_t counter = 0;

// ══════════════════════════════════════════════════════════════
void setup() {
  WiFi.setSleep(false);
  btStop();
  delay(1000);

  Serial.begin(115200);
  Wire.begin();
  delay(200);

  while (sensor.wakeup() == false) {
    Serial.println("Could not connect to GY521..."); delay(1000);
  }
  sensor.setAccelSensitivity(3);
  sensor.setGyroSensitivity(0);
  sensor.setThrottle();

  pinMode(LEADS_OFF_P, INPUT_PULLUP);
  pinMode(LEADS_OFF_N, INPUT);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  delay(100);

  for (int i=0; i<BPM_HISTORY; i++) rr_intervals[i] = 0;
  for (int i=0; i<FALL_CONFIRM_WINDOW; i++) fall_history[i] = false;

  if (!initializeModel(fall_detection_model, tensor_arena, kTensorArenaSize)) {
    Serial.println("EdgeNeuron init failed!"); while(true);
  }

  client.setBufferSize(1024);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);

  Serial.println("ESP32 ready — Smart Fall Detection v2");
}

// ══════════════════════════════════════════════════════════════
void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  sensor.read();
  float raw_ax = sensor.getAccelX();
  float raw_ay = sensor.getAccelY();
  float raw_az = sensor.getAccelZ();
  float gx     = sensor.getGyroX();
  float gy     = sensor.getGyroY();
  float gz     = sensor.getGyroZ();

  ml_push_sample(raw_ax, raw_ay, raw_az, gx, gy, gz);
  if (ml_window_ready) {
    run_inference();
    ml_window_ready = false;
  }

  bool raw_lo_p = (digitalRead(LEADS_OFF_P) == HIGH);
  int  lo_n_reads = 0;
  for (int i=0; i<3; i++) lo_n_reads += digitalRead(LEADS_OFF_N);
  bool raw_lo_n = (lo_n_reads >= 2);
  updateLeadsDebounce(raw_lo_p, raw_lo_n);
  bool leads_off = lo_p_stable || lo_n_stable;

  int ecg_raw = 0;
  if (!leads_off) {
    ecg_raw      = analogRead(ECG_PIN);
    ecg_filtered = filterECG(ecg_raw);
    detectPeak(ecg_filtered);
  } else {
    if (lo_p_count >= LO_DEBOUNCE && lo_n_count >= LO_DEBOUNCE) {
      bpm = 0.0; rr_idx = 0; last_peak_ms = 0;
    }
  }

  Serial.printf("%u\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%d\t%.1f\t%d\t%d\n",
    counter, raw_ax, raw_ay, raw_az, gx, gy, gz,
    ecg_raw, bpm, lo_p_stable?1:0, lo_n_stable?1:0);

  counter++;

  unsigned long now = millis();
  if (now - lastMsg > interval) {
    lastMsg = now;
    publishAll(ecg_raw, leads_off);
  }

  delay(20);
}