#include <Arduino.h>
#include "lora_app.h"

// ===== Pins (UNO R4 -> Ra-02) =====
static const int LORA_CS    = 10; // NSS/CS
static const int LORA_RST   = 9;  // RESET
static const int LORA_DIO0  = 2;  // DIO0

static const long LORA_FREQ = 433E6; // must match the transmitter

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println("UNO R4 LoRa RX: waiting for messages...");

  if (!lora_init(LORA_FREQ, LORA_CS, LORA_RST, LORA_DIO0)) {
    Serial.println("LoRa init FAILED. Check wiring/power/frequency.");
    while (true) delay(1000);
  }

  Serial.println("LoRa init OK.");
}

void loop() {
  String msg;
  int rssi = 0;

  if (lora_receive_line(msg, rssi)) {
    Serial.print("RX: ");
    Serial.print(msg);
    Serial.print("  (RSSI=");
    Serial.print(rssi);
    Serial.println(" dBm)");
  }
}