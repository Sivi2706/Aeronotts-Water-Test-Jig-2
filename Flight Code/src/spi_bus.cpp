#include "spi_bus.h"

void SPIBus::begin(uint8_t sck, uint8_t miso, uint8_t mosi) {
  _vspi.begin(sck, miso, mosi);
  if (!_mutex) _mutex = xSemaphoreCreateMutex();
}

bool SPIBus::take(uint32_t timeout_ms) {
  if (!_mutex) return true;
  return xSemaphoreTake(_mutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

void SPIBus::give() {
  if (!_mutex) return;
  xSemaphoreGive(_mutex);
}