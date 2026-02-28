#include "esp_air_lora_tx.h"
#include "esp_air_pins.h"
#include <SPI.h>
#include <LoRa.h>

static bool EspLoRaOk = false;

bool espLoRaTx_begin(long frequency_hz) {
  SPI.begin(ESP_LORA_SCK, ESP_LORA_MISO, ESP_LORA_MOSI, ESP_LORA_CS);
  LoRa.setPins(ESP_LORA_CS, ESP_LORA_RST, ESP_LORA_DIO0);

  EspLoRaOk = LoRa.begin(frequency_hz);
  if (!EspLoRaOk) return false;

  // Optional tuning
  LoRa.setTxPower(17);
  LoRa.enableCrc();
  return true;
}

bool espLoRaTx_sendLine(const char* line) {
  if (!EspLoRaOk) return false;
  LoRa.beginPacket();
  LoRa.print(line);
  LoRa.endPacket();
  return true;
}