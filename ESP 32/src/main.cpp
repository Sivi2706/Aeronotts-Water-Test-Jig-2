#include <Arduino.h>
#include "esp32_gps_rf_tx.h"

// LoRa pins (same as before)
#define LORA_CS     5
#define LORA_RST    14
#define LORA_DIO0   26
#define LORA_FREQ   433E6

// GPS pins (NEO-8M)
#define GPS_RX_PIN  16   // ESP32 RX (connect to GPS TX)
#define GPS_TX_PIN  17   // ESP32 TX (connect to GPS RX) optional
#define GPS_BAUD    9600

static uint32_t lastTxMs = 0;
static const uint32_t TX_PERIOD_MS = 1000;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println("ESP32 GPS -> LoRa TX starting...");

  if (!esp32_gpsrf_init(LORA_FREQ, LORA_CS, LORA_RST, LORA_DIO0,
                        GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD)) {
    Serial.println("Init FAILED. Check LoRa wiring/power/frequency.");
    while (1) delay(1000);
  }

  Serial.println("Init OK.");
  Serial.println("Sending CSV over LoRa at 1 Hz.");
  Serial.println("NOFIX CSV: ms,NOFIX,chars,sentFix,sats,hdop");
  Serial.println("FIX   CSV: ms,FIX,lat,lng,alt_m,spd_kmph,sats,hdop,utc_hhmmss,utc_ddmmyy");
}

void loop() {
  esp32_gpsrf_poll();

  uint32_t now = millis();
  if (now - lastTxMs >= TX_PERIOD_MS) {
    lastTxMs = now;

    bool ok = esp32_gpsrf_send_packet_csv();
    Serial.println(ok ? "\nTX packet sent" : "\nTX send failed");
  }
}