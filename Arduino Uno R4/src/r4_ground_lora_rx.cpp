#include "r4_ground_lora_rx.h"
#include "r4_ground_pins.h"
#include <SPI.h>
#include <LoRa.h>

static bool R4LoRaOk = false;

bool r4LoRaRx_begin(long frequency_hz) {
  // On Uno-class boards SPI pins are provided by the board.
  LoRa.setPins(R4_LORA_CS, R4_LORA_RST, R4_LORA_DIO0);
  R4LoRaOk = LoRa.begin(frequency_hz);
  if (!R4LoRaOk) return false;

  LoRa.enableCrc();
  return true;
}

bool r4LoRaRx_receiveLine(String& outLine, int& outRssi) {
  if (!R4LoRaOk) return false;

  int packetSize = LoRa.parsePacket();
  if (!packetSize) return false;

  outLine = "";
  while (LoRa.available()) outLine += (char)LoRa.read();
  outRssi = LoRa.packetRssi();
  return true;
}