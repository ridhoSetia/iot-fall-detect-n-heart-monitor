// ================================================================
//  ESP32 — GY521 (MPU-6050) + AD8232 (ECG)
//
//  Pin ESP32:
//    ECG_PIN      → GPIO 36 (VP, input-only, ADC1_CH0)
//    LEADS_OFF_P  → GPIO 25 (LO+, INPUT_PULLUP)
//    LEADS_OFF_N  → GPIO 26 (LO-, INPUT + resistor 10kΩ eksternal ke GND!)
//
//  Output serial (tab-separated, 100 Hz):
//  counter\tax\tay\taz\tgx\tgy\tgz\ttemp\tecg\tbpm\tlo_p\tlo_n\tleads_off
// ================================================================

#include "GY521.h"
#include <Wire.h>
#include "WiFi.h" // diperlukan untuk mematikan WiFi

// ── Pin definitions ────────────────────────────────────────────
#define ECG_PIN 36     // OUTPUT AD8232 → ADC1_CH0 (input-only)
#define LEADS_OFF_P 25 // LO+ → INPUT_PULLUP, aktif HIGH saat elektroda lepas
#define LEADS_OFF_N 26 // LO- → INPUT + 10kΩ eksternal ke GND (pull-down)

// ── GY521 ──────────────────────────────────────────────────────
GY521 sensor(0x68);

const int WINDOW_SIZE = 10;
float buffer_ax[WINDOW_SIZE], buffer_ay[WINDOW_SIZE], buffer_az[WINDOW_SIZE];
int readIndex = 0;

float getSMA(float newValue, float *buffer)
{
  buffer[readIndex] = newValue;
  float sum = 0;
  for (int i = 0; i < WINDOW_SIZE; i++)
    sum += buffer[i];
  return sum / WINDOW_SIZE;
}

// ── AD8232 — Leads-off debounce ────────────────────────────────
const int LO_DEBOUNCE = 8;
int lo_p_count = LO_DEBOUNCE;
int lo_n_count = LO_DEBOUNCE;
bool lo_p_stable = true;
bool lo_n_stable = true;

void updateLeadsDebounce(bool raw_p, bool raw_n)
{
  // LO+
  if (raw_p)
    lo_p_count = min(lo_p_count + 1, LO_DEBOUNCE);
  else
    lo_p_count = max(lo_p_count - 1, 0);
  lo_p_stable = (lo_p_count >= LO_DEBOUNCE);

  // LO-
  if (raw_n)
    lo_n_count = min(lo_n_count + 1, LO_DEBOUNCE);
  else
    lo_n_count = max(lo_n_count - 1, 0);
  lo_n_stable = (lo_n_count >= LO_DEBOUNCE);
}

// ── BPM detection — versi dioptimalkan ────────────────────────
const int BPM_HISTORY = 8; // lebih banyak dari 5
const unsigned long REFRACTORY = 300;
const float ADAPTIVE_K = 0.75f;  // threshold = 75% dari peak max
const float OUTLIER_THR = 0.25f; // tolak RR yang melenceng >25%

// Adaptive threshold
float peak_max = 0.0f;
float adapt_thresh = 1500.0f; // nilai awal aman
unsigned long peak_max_reset_ms = 0;

// RR buffer
float rr_intervals[BPM_HISTORY];
int rr_idx = 0;
bool rr_filled = false;

unsigned long last_peak_ms = 0;
float bpm = 0.0;
bool above_thresh = false;

// ── IIR band-pass filter (Butterworth 2nd order, ~0.5–3.3 Hz @ 100 Hz) ──
// Koefisien dihitung untuk Fs=100 Hz, fl=0.5 Hz, fh=3.3 Hz
// Bisa digenerate ulang di: https://www.earlevel.com/main/2021/09/02/biquad-calculator-v3/
struct Biquad
{
  float b0, b1, b2, a1, a2;
  float z1 = 0, z2 = 0;
  float process(float x)
  {
    float y = b0 * x + z1;
    z1 = b1 * x - a1 * y + z2;
    z2 = b2 * x - a2 * y;
    return y;
  }
};

// High-pass 0.5 Hz (buang baseline drift)
Biquad hpf = {0.9691f, -1.9382f, 0.9691f, -1.9371f, 0.9393f};
// Low-pass 3.3 Hz (buang noise HF, izinkan s.d ~200 BPM)
Biquad lpf = {0.0034f, 0.0068f, 0.0034f, -1.8227f, 0.8363f};

float filterECG(int rawValue)
{
  float x = (float)rawValue;
  x = hpf.process(x);
  x = lpf.process(x);
  return x;
}

// ── Median dari array float ──
float medianOf(float *arr, int n)
{
  float tmp[BPM_HISTORY];
  memcpy(tmp, arr, n * sizeof(float));
  // selection sort sederhana (n kecil, overhead minimal)
  for (int i = 0; i < n - 1; i++)
    for (int j = i + 1; j < n; j++)
      if (tmp[j] < tmp[i])
      {
        float t = tmp[i];
        tmp[i] = tmp[j];
        tmp[j] = t;
      }
  return (n % 2) ? tmp[n / 2] : (tmp[n / 2 - 1] + tmp[n / 2]) * 0.5f;
}

// ── Weighted average RR (bobot = posisi, terbaru = bobot terbesar) ──
float weightedBPM()
{
  int count = rr_filled ? BPM_HISTORY : rr_idx;
  if (count == 0)
    return 0.0f;
  float sum_w = 0, sum_rr = 0;
  for (int i = 0; i < count; i++)
  {
    // indeks dalam ring buffer: (rr_idx - count + i + BPM_HISTORY) % BPM_HISTORY
    int idx = (rr_idx - count + i + BPM_HISTORY) % BPM_HISTORY;
    float w = (float)(i + 1); // bobot 1..count, terbaru = terbesar
    sum_rr += rr_intervals[idx] * w;
    sum_w += w;
  }
  return 60000.0f / (sum_rr / sum_w);
}

void detectPeak(float ecgFiltered)
{
  unsigned long now = millis();

  // ── Update adaptive threshold ──
  // Reset peak_max setiap 2 detik supaya threshold ikut turun kalau sinyal melemah
  if (now - peak_max_reset_ms > 2000)
  {
    peak_max = adapt_thresh / ADAPTIVE_K * 0.85f; // turun perlahan, tidak crash
    peak_max_reset_ms = now;
  }
  if (ecgFiltered > peak_max)
    peak_max = ecgFiltered;
  adapt_thresh = peak_max * ADAPTIVE_K;

  // ── Peak detection ──
  if (ecgFiltered > adapt_thresh && !above_thresh)
  {
    above_thresh = true;
    unsigned long rr = now - last_peak_ms;

    if (last_peak_ms != 0 && rr > REFRACTORY && rr < 2000)
    {
      // ── Outlier rejection ──
      bool accept = true;
      int count = rr_filled ? BPM_HISTORY : rr_idx;
      if (count >= 3)
      { // butuh minimal 3 sample untuk median yang berarti
        float med = medianOf(rr_intervals, count);
        float deviation = fabsf((float)rr - med) / med;
        if (deviation > OUTLIER_THR)
          accept = false; // lompatan >25% dari median → buang
      }

      if (accept)
      {
        rr_intervals[rr_idx % BPM_HISTORY] = (float)rr;
        rr_idx++;
        if (rr_idx >= BPM_HISTORY)
          rr_filled = true;
        bpm = weightedBPM();
      }
    }
    last_peak_ms = now;
  }
  else if (ecgFiltered < adapt_thresh * 0.85f)
  {
    above_thresh = false; // hysteresis 15% biar tidak false-trigger
  }
}

// ── Counter ────────────────────────────────────────────────────
uint32_t counter = 0;

// ══════════════════════════════════════════════════════════════
void setup()
{
  // ── Matikan WiFi & Bluetooth PERTAMA ──
  // WiFi/BT yang aktif menyebabkan ADC1 baca 4095 (interferensi internal)
  WiFi.mode(WIFI_OFF);
  btStop();
  delay(1000);

  Serial.begin(115200);
  Wire.begin();
  delay(200);

  // ── GY521 init ──
  while (sensor.wakeup() == false)
  {
    Serial.println("Could not connect to GY521...");
    delay(1000);
  }
  sensor.setAccelSensitivity(0);
  sensor.setGyroSensitivity(0);
  sensor.setThrottle();
  for (int i = 0; i < WINDOW_SIZE; i++)
  {
    buffer_ax[i] = buffer_ay[i] = buffer_az[i] = 0;
  }

  // ── AD8232 pin init ──
  // GPIO 25 (LO+): pull-up internal cukup, AD8232 open-drain aktif HIGH saat lepas
  pinMode(LEADS_OFF_P, INPUT_PULLUP);

  // GPIO 26 (LO-): WAJIB pasang resistor 10kΩ dari pin ini ke GND secara fisik!
  //                ESP32 tidak punya pull-down internal yang reliable.
  //                Tanpa resistor → pin floating → leads_off tidak akurat.
  pinMode(LEADS_OFF_N, INPUT);

  // GPIO 36 (ECG): input-only, atur atenuasi hanya untuk pin ini
  // ADC_11db = range 0–3.3V (full scale), cocok untuk output AD8232
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db); // per-pin, tidak ganggu channel lain
  delay(100);
  Serial.print(" LO_N_RAW:");
  Serial.println(digitalRead(LEADS_OFF_N));
  Serial.print("LO_P_RAW:");
  Serial.print(digitalRead(LEADS_OFF_P));
  int test = analogRead(ECG_PIN);
  Serial.print("ADC Sanity Check:");
  Serial.println(test);
  delay(5000);

  for (int i = 0; i < BPM_HISTORY; i++)
    rr_intervals[i] = 0;

  Serial.println("ESP32 GY521+AD8232 ready");
  Serial.println("counter\tax\tay\taz\tgx\tgy\tgz\ttemp\tecg\tbpm\tlo_p\tlo_n\tleads_off");
}

// ══════════════════════════════════════════════════════════════
void loop()
{
  // ── GY521 ──
  sensor.read();
  float raw_ax = sensor.getAccelX();
  float raw_ay = sensor.getAccelY();
  float raw_az = sensor.getAccelZ();
  float ax = getSMA(raw_ax, buffer_ax);
  float ay = getSMA(raw_ay, buffer_ay);
  float az = getSMA(raw_az, buffer_az);
  readIndex = (readIndex + 1) % WINDOW_SIZE;

  float gx = sensor.getGyroX();
  float gy = sensor.getGyroY();
  float gz = sensor.getGyroZ();
  float t = sensor.getTemperature();

  // ── AD8232 — baca LO pins ──
  // GPIO 25 (LO+): INPUT_PULLUP → normal = LOW, elektroda lepas = HIGH
  bool raw_lo_p = (digitalRead(LEADS_OFF_P) == HIGH);

  // GPIO 26 (LO-): pull-down eksternal 10kΩ → normal = LOW, elektroda lepas = HIGH
  // majority vote 3 sample untuk filter noise pada pin tanpa pull-down internal
  int lo_n_reads = 0;
  for (int i = 0; i < 3; i++)
    lo_n_reads += digitalRead(LEADS_OFF_N);
  bool raw_lo_n = (lo_n_reads >= 2);

  updateLeadsDebounce(raw_lo_p, raw_lo_n);

  bool leads_off = lo_p_stable || lo_n_stable;

  // ── ECG ──
  int ecg_raw = 0;
  if (!leads_off)
  {
    ecg_raw = analogRead(ECG_PIN);
    float ecg_filtered = filterECG(ecg_raw);
    detectPeak(ecg_filtered);
  }
  else
  {
    // Reset BPM jika kedua leads off stabil
    if (lo_p_count >= LO_DEBOUNCE && lo_n_count >= LO_DEBOUNCE)
    {
      bpm = 0.0;
      rr_idx = 0;
      last_peak_ms = 0;
    }
  }

  // ── Output serial ──
  Serial.print(counter);
  Serial.print('\t');
  Serial.print(ax, 3);
  Serial.print('\t');
  Serial.print(ay, 3);
  Serial.print('\t');
  Serial.print(az, 3);
  Serial.print('\t');
  Serial.print(gx, 3);
  Serial.print('\t');
  Serial.print(gy, 3);
  Serial.print('\t');
  Serial.print(gz, 3);
  Serial.print('\t');
  Serial.print(t, 2);
  Serial.print('\t');
  Serial.print(ecg_raw);
  Serial.print('\t');
  Serial.print(bpm, 1);
  Serial.print('\t');
  Serial.print(lo_p_stable ? 1 : 0);
  Serial.print('\t');
  Serial.print(lo_n_stable ? 1 : 0);
  Serial.print('\t');
  Serial.println(leads_off ? 1 : 0);

  counter++;
  delay(10); // 100 Hz
}