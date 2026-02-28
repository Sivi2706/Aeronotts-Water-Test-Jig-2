#include <Arduino.h>
#include "r4_ground_lora_rx.h"

static constexpr long LORA_FREQ_HZ = 915E6; // match ESP32

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("R4 Ground Station -> LoRa RX starting...");
  if (!r4LoRaRx_begin(LORA_FREQ_HZ)) {
    Serial.println("LoRa init FAILED");
    while (1) delay(100);
  }
  Serial.println("LoRa init OK. Waiting for packets...");
}

void loop() {
  String line;
  int rssi = 0;

  if (r4LoRaRx_receiveLine(line, rssi)) {
    Serial.print("RX: ");
    Serial.print(line);
    Serial.print("  (RSSI=");
    Serial.print(rssi);
    Serial.println(" dBm)");
  }
}