#pragma once
#include <Arduino.h>
#include <LoRa.h>
#include "spi_bus.h"
#include "common_types.h"

class LoRaMgr {
public:
  bool begin(SPIBus& bus, uint8_t cs, uint8_t rst, uint8_t dio0, long freqHz, int txPower, ErrorState& errs);
  void send(SPIBus& bus, const String& payload);

private:
  bool _ok = false;
};