#include "esp_air_lora_tx.h"
#include "esp_air_pins.h"

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

static bool g_txOk = false;
static uint32_t g_txCount = 0;

bool espLoRaTx_begin(long frequency_hz) {
  g_txOk = false;

  // Set pins (must match your esp_air_pins.h)
  // Ensure these macros exist:
  //   ESP_LORA_CS, ESP_LORA_RST, ESP_LORA_DIO0
  LoRa.setPins(ESP_LORA_CS, ESP_LORA_RST, ESP_LORA_DIO0);

  // Optional: SPI.begin(...) only if you defined custom SPI pins in your project
  // Otherwise LoRa lib uses default VSPI pins on ESP32.
  // SPI.begin();

  for (int attempt = 1; attempt <= 5; attempt++) {
    if (LoRa.begin(frequency_hz)) {
      g_txOk = true;
      break;
    }
    delay(200);
  }
  if (!g_txOk) return false;

  // *** MUST match the R4 receiver settings ***
  LoRa.setSyncWord(0x12);
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.enableCrc();

  // Reasonable TX power for RA-02 (typically up to 20 dBm, but 17 is safer)
  LoRa.setTxPower(17);

  return true;
}

bool espLoRaTx_sendLine(const char* line) {
  if (!g_txOk || !line) return false;

  g_txCount++;

  // Build packet
  LoRa.beginPacket();
  LoRa.print(line);
  int rc = LoRa.endPacket();   // returns 1 on success in this library

  // Local debug (ESP32 serial) so you can confirm RF is really sending
  Serial.print("LORA_TX#");
  Serial.print(g_txCount);
  Serial.print(rc == 1 ? " OK: " : " FAIL: ");
  Serial.println(line);

  return (rc == 1);
}