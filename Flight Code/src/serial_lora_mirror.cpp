#include "serial_lora_mirror.h"
#include <stdarg.h>

void SerialLoRaMirror::begin(LoRaMgr* lora, SPIBus* bus, bool* loraOkFlag) {
  _lora = lora;
  _bus = bus;
  _loraOk = loraOkFlag;

  if (!_q) {
    // Queue holds pointers to heap-allocated Strings (keeps it simple)
    _q = xQueueCreate(20, sizeof(String*));
  }
}

void SerialLoRaMirror::startTask(BaseType_t core, uint32_t stackWords, UBaseType_t prio) {
  xTaskCreatePinnedToCore(taskThunk, "LoRaMirrorTX", stackWords, this, prio, nullptr, core);
}

void SerialLoRaMirror::logLine(const String& line) {
  // Always print to USB serial
  Serial.println(line);

  // Enqueue for LoRa TX
  if (!_q) return;
  String* heapStr = new String(line);
  if (xQueueSend(_q, &heapStr, 0) != pdTRUE) {
    // Queue full: drop oldest then push
    String* old = nullptr;
    if (xQueueReceive(_q, &old, 0) == pdTRUE && old) delete old;
    xQueueSend(_q, &heapStr, 0);
  }
}

void SerialLoRaMirror::logf(const char* fmt, ...) {
  char buf[220]; // keep payload short for SX127x
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  logLine(String(buf));
}

void SerialLoRaMirror::taskThunk(void* arg) {
  static_cast<SerialLoRaMirror*>(arg)->taskLoop();
}

void SerialLoRaMirror::taskLoop() {
  for (;;) {
    String* msg = nullptr;
    if (xQueueReceive(_q, &msg, pdMS_TO_TICKS(100)) == pdTRUE) {
      if (msg) {
        if (_lora && _bus && _loraOk && *_loraOk) {
          _lora->send(*_bus, *msg);
        }
        delete msg;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}