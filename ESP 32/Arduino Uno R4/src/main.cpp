#include <Arduino.h>
#include "r4_ground_lora_rx.h"

static constexpr long LORA_FREQ_HZ = 433E6;   // RA-02 SX1278 = 433MHz

static bool g_loraOk = false;
static uint32_t g_lastHbMs = 0;
static uint32_t g_lastInitTryMs = 0;

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 2500) {}

  Serial.println();
  Serial.println("R4 Ground Station -> LoRa RX starting...");
  Serial.println("Expecting ESP32 lines: ms,fix,lat,lng,alt_m,spd_kmph,hdop,sats,utc_hhmmss,utc_ddmmyy");

  // First init attempt
  g_loraOk = r4LoRaRx_begin(LORA_FREQ_HZ);
  Serial.println(g_loraOk ? "LoRa init OK" : "LoRa init FAILED");
}

void loop() {
  uint32_t now = millis();

  // Heartbeat + LoRa state every 1s
  if (now - g_lastHbMs >= 1000) {
    g_lastHbMs = now;
    Serial.print("HB,");
    Serial.print(now);
    Serial.print(",LORA_OK=");
    Serial.println(g_loraOk ? 1 : 0);
  }

  // If init failed, retry every 2s (so you don’t need to reset)
  if (!g_loraOk && (now - g_lastInitTryMs >= 2000)) {
    g_lastInitTryMs = now;
    Serial.println("Retrying LoRa init...");
    g_loraOk = r4LoRaRx_begin(LORA_FREQ_HZ);
    Serial.println(g_loraOk ? "LoRa init OK" : "LoRa init FAILED");
  }

  // Receive packets
  String line;
  int rssi = 0;
  if (r4LoRaRx_receiveLine(line, rssi)) {
    Serial.print("RX,RSSI=");
    Serial.print(rssi);
    Serial.print(",PAYLOAD=");
    Serial.println(line);
  }
}