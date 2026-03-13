#include "lora_mgr.h"

bool LoRaMgr::begin(SPIBus& bus, uint8_t cs, uint8_t rst, uint8_t dio0,
                    long freqHz, int txPower, ErrorState& errs) {
  if (!bus.take(200)) {
    errs.lora = ERR_LORA_INIT_FAIL;
    _ok = false;
    return false;
  }

  LoRa.setPins(cs, rst, dio0);
  LoRa.setSPI(bus.spi());

  bool ok = LoRa.begin(freqHz);
  if (ok) {
    LoRa.setTxPower(txPower);
    errs.lora = ERR_NONE;
    _ok = true;
  } else {
    errs.lora = ERR_LORA_INIT_FAIL;
    _ok = false;
  }

  bus.give();
  return _ok;
}

void LoRaMgr::send(SPIBus& bus, const String& payload) {
  if (!_ok) return;
  if (!bus.take(50)) return;

  LoRa.beginPacket();
  LoRa.print(payload);
  LoRa.endPacket(true);

  bus.give();
}