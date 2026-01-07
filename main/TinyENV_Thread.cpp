// Copyright 2025 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/* - Tiny Room Sensor (XIAO ESP32-C6) - Headless
 * - Matter over WiFi (Temp°C + Humidity %RH)
 * - Commissioning info via Serial
 * - BOOT hold >5s to decommission
 */

// Matter Manager
#include <Matter.h>
#include <MatterEndPoint.h>
#include "esp_pm.h"
#include <Wire.h>
#include <Adafruit_SHT4x.h>
#include <MatterEndpoints/MatterTemperatureSensorBattery.h>
#include "esp_openthread.h"
#include <openthread/link.h>
#if !CONFIG_ENABLE_CHIPOBLE
// if the device can be commissioned using BLE, WiFi is not used - save flash space
#include <WiFi.h>
#endif

/* =========== Debug Mode Switch ========*/
static constexpr bool DEBUG_SERIAL = false;

// ------------ Board Pin Defs -----------------
#define SDA_PIN 22
#define SCL_PIN 23
#define VBAT_ADC_PIN A0   // GPIO0 (XIAO ESP32-C6 A0)

// ----------- Matter Endpoints -------------
MatterTemperatureSensorBattery TempSensor;      // °C (Matter spec)
MatterHumiditySensor    HumiditySensor;  // %RH

// CONFIG_ENABLE_CHIPOBLE is enabled when BLE is used to commission the Matter Network
#if !CONFIG_ENABLE_CHIPOBLE
// WiFi is manually set and started
const char *ssid = "your-ssid";          // Change this to your WiFi SSID
const char *password = "your-password";  // Change this to your WiFi password
#endif

// set your board USER BUTTON pin here
const uint8_t BUTTON_PIN = BOOT_PIN;
const uint32_t DECOMMISSION_HOLD_MS = 5000;

// ---------- SHT41 Init ----------
Adafruit_SHT4x sht4;
bool sht_ready = false;

// ---------- Timers ----------
const uint32_t SENSOR_UPDATE_MS = 120000; // 2min: sensor + Matter update
uint32_t last_sensor_ms = 0;

// ---------- Cached readings ----------
float g_lastTempC = 50.0f;
float g_lastRH    = 99.0f;

// ---------- Utils ----------
static inline float C_to_F(float c) { return (c * 9.0f / 5.0f) + 32.0f; }

// ---------- Battery (A0 via 1:2 divider) ----------
static constexpr float VBAT_GAIN = 1.0123f;  // <-- Battery sense conversion factor (1.0123 for my XIAO ESP32-C6 w/ 220,000u resistors)

static inline float readBatteryVoltsA0() {
  uint32_t mv = 0;
  for (int i = 0; i < 16; i++) {
    uint32_t s = analogReadMilliVolts(VBAT_ADC_PIN);
    mv += s;
    delay(2);
  }
  float pin_mv = (mv / 16.0f);
  float vA0    = pin_mv / 1000.0f;            // volts at VBAT_ADC_PIN
  float vbat   = (vA0 * 2.0f) * VBAT_GAIN;    // undo 1:2 divider, then apply calibration
  if (DEBUG_SERIAL) {
    Serial.printf("VBAT_ADC pin avg: %.1f mV | v_pin=%.3f V | vbat=%.3f V\r\n",pin_mv, vA0, vbat);
  }
  return vbat;
}

static inline uint8_t voltsToPct(float v) {
  if (v >= 4.10f) return 100;   // Values above 4.10V registers as 100%
  if (v <= 3.00f) return 0;     // Values below 3.0 register as 0%

  // 3.90–4.10 -> 80–100  (ΔV=0.20, Δ%=20, slope=100 %/V)
  if (v >= 3.90f) return (uint8_t)lroundf(80.0f + (v - 3.90f) * 100.0f);

  // 3.60–3.90 -> 40–80   (ΔV=0.30, Δ%=40, slope=133.333... %/V)
  if (v >= 3.60f) return (uint8_t)lroundf(40.0f + (v - 3.60f) * (40.0f / 0.30f));

  // 3.30–3.60 -> 10–40   (ΔV=0.30, Δ%=30, slope=100 %/V)
  if (v >= 3.30f) return (uint8_t)lroundf(10.0f + (v - 3.30f) * (30.0f / 0.30f));

  // 3.00–3.30 -> 0–10    (ΔV=0.30, Δ%=10, slope=33.333... %/V)
  return (uint8_t)lroundf((v - 3.00f) * (10.0f / 0.30f));
}

// ---------- SHT41 ----------
void sensors_init() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000); // conservative for bring-up

  if (sht4.begin()) {
    sht4.setPrecision(SHT4X_HIGH_PRECISION);
    sht4.setHeater(SHT4X_NO_HEATER);
    sht_ready = true;
    Serial.println("SHT4x detected");
  } else {
    Serial.println("SHT4x not found on I2C!");
  }
}

bool read_sht41(float &tempC, float &rh) {
  if (!sht_ready) return false;

  sensors_event_t hum, temp;
  if (!sht4.getEvent(&hum, &temp)) return false;

  tempC = temp.temperature;
  rh    = hum.relative_humidity;
  return true;
}

// -------- Sensor Update Function---------
static void sensorUpdate() {
  float tC, rh;

  if (read_sht41(tC, rh)) {
    g_lastTempC = tC;
    g_lastRH    = rh;

    float vbat   = readBatteryVoltsA0();
    uint8_t bpct = voltsToPct(vbat);

    TempSensor.setTemperature(tC);

    uint32_t mv_to_matter = (uint32_t) lroundf(vbat * 1000.0f);
    TempSensor.setBatteryVoltageMv(mv_to_matter);
    TempSensor.setBatteryPercent(bpct);

    HumiditySensor.setHumidity(rh);

    if (DEBUG_SERIAL) {
      Serial.print("Sensor update: ");
      Serial.print(C_to_F(tC), 1);
      Serial.print(" F, ");
      Serial.print(rh, 0);
      Serial.print(" %RH, ");
      Serial.print(" VBAT: ");
      Serial.print(vbat, 3);
      Serial.print("V (");
      Serial.print(bpct);
      Serial.println("%)");
    }
  } else {
    if (DEBUG_SERIAL) {
      Serial.println("SHT41 sensor read failed.");
    }
  }
}

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.begin(115200);
  delay(200);

  Serial.println("\nTiny Room Sensor (headless, Matter + SHT41)");
  analogReadResolution(12);
  analogSetPinAttenuation(VBAT_ADC_PIN, ADC_11db);

  sensors_init();

  // Seed cache (ok if read fails; defaults remain)
  float seedC = g_lastTempC;
  float seedRH = g_lastRH;
  if (read_sht41(seedC, seedRH)) {
    g_lastTempC = seedC;
    g_lastRH    = seedRH;
  }

  // Matter endpoints
  TempSensor.begin(g_lastTempC);   // °C
  HumiditySensor.begin(g_lastRH);

  Matter.begin();
  
  // Commission if needed (info via Serial)
  if (!Matter.isDeviceCommissioned()) {
    String manualCode = Matter.getManualPairingCode().c_str();
    String qrUrl      = Matter.getOnboardingQRCodeUrl().c_str();

    Serial.println("--------------------------------------------------");
    Serial.println("Device not commissioned yet.");
    Serial.print("Manual pairing code: ");
    Serial.println(manualCode);
    Serial.print("QR code URL: ");
    Serial.println(qrUrl);
    Serial.println("Use your Matter controller to commission this device.");
    Serial.println("--------------------------------------------------");
  } else {
    Serial.println("Device already commissioned.");
  }

  /* ============ Set Thread Polling Interval ======= */
  otInstance *ot = esp_openthread_get_instance();
  if (ot) {
    otLinkSetPollPeriod(ot, 5000); // 5s to start; try 10s if you can tolerate latency
    Serial.println("Set OT poll period to 10000ms");
  }

  /* ==== Power Saving Features Enable ==== */
  setCpuFrequencyMhz(80);     // Reduce CPU freq (optional but usually helpful)

  esp_pm_config_t pm = {      // Enable dynamic freq scaling + automatic light sleep when idle
    .max_freq_mhz = 80,
    .min_freq_mhz = 40,
    .light_sleep_enable = true
  };
  esp_pm_configure(&pm);

  /* ======== Initial Power-on sensor update =====*/
  sensorUpdate(); // 
  if (read_sht41(tC, rh)) {
    Serial.println("Initial Sensor Read");
    Serial.print("Sensor update: ");
    Serial.print(C_to_F(tC), 1);
    Serial.print(" F, ");
    Serial.print(rh, 0);
    Serial.print(" %RH, ");
    Serial.print(" VBAT: ");
    Serial.print(vbat, 3);
    Serial.print("V (");
    Serial.print(bpct);
    Serial.println("%)");
  } 
  else {
    Serial.println("SHT41 sensor read failed.");
  }
  last_sensor_ms = millis();
}

void loop() {
  const uint32_t now = millis();

  // ---- Periodic sensor read + Matter update ----
  if (now - last_sensor_ms >= SENSOR_UPDATE_MS) {
    last_sensor_ms = now;
    sensorUpdate();
  }
    // ---- Decommission (hold BOOT > 5s) ----
  static bool pressed = false;
  static uint32_t press_ts = 0;

  bool btn_low = (digitalRead(BUTTON_PIN) == LOW);
  if (btn_low && !pressed) {
    pressed = true;
    press_ts = now;
  }
  if (!btn_low && pressed) {
    pressed = false;
  }
  if (pressed && (now - press_ts >= DECOMMISSION_HOLD_MS)) {
    Serial.println("Decommissioning Matter node...");
    Matter.decommission();
    pressed = false;
    press_ts = now;
  }
  vTaskDelay(pdMS_TO_TICKS(1000));

}