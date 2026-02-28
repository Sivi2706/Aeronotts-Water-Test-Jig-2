#include "lora_app.h"
#include <SPI.h>
#include <LoRa.h>

static bool s_ok = false;

bool lora_init(long frequency_hz, int cs_pin, int reset_pin, int dio0_pin) {
  // UNO R4: SPI pins are the board's hardware SPI (UNO-style: D13/D12/D11)
  // SPI.begin() uses the board defaults.
  SPI.begin();

  LoRa.setSPI(SPI);
  LoRa.setPins(cs_pin, reset_pin, dio0_pin);

  if (!LoRa.begin(frequency_hz)) {
    s_ok = false;
    return false;
  }

  s_ok = true;
  return true;
}

bool lora_receive_line(String& out_line, int& out_rssi) {
  if (!s_ok) return false;

  int packetSize = LoRa.parsePacket();
  if (packetSize <= 0) return false;

  out_line = "";
  while (LoRa.available()) {
    out_line += (char)LoRa.read();
  }
  out_rssi = LoRa.packetRssi();
  out_line.trim();
  return true;
}