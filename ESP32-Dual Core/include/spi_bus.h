#pragma once
#include <Arduino.h>
#include <SPI.h>

class SPIBus {
public:
  void begin(uint8_t sck, uint8_t miso, uint8_t mosi);

  // Mutex-protected SPI “session”
  bool take(uint32_t timeout_ms = 50);
  void give();

  SPIClass& spi() { return _vspi; }

private:
  SPIClass _vspi{VSPI};
  SemaphoreHandle_t _mutex = nullptr;
};