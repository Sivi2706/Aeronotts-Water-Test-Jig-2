#pragma once
#include <Arduino.h>
#include "lora_mgr.h"
#include "spi_bus.h"

class SerialLoRaMirror {
public:
  // Call in setup after LoRa init is done
  void begin(LoRaMgr* lora, SPIBus* bus, bool* loraOkFlag);

  // Thread-safe: can be called from anywhere
  void logLine(const String& line);
  void logf(const char* fmt, ...);

  // Starts the worker task that actually transmits
  void startTask(BaseType_t core, uint32_t stackWords = 4096, UBaseType_t prio = 1);

private:
  static void taskThunk(void* arg);
  void taskLoop();

  LoRaMgr* _lora = nullptr;
  SPIBus* _bus = nullptr;
  bool* _loraOk = nullptr;

  QueueHandle_t _q = nullptr;
};