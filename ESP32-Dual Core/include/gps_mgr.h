#pragma once
#include <Arduino.h>
#include <TinyGPSPlus.h>
#include "common_types.h"

class GPSMgr {
public:
  GPSMgr() = default;

  void begin(HardwareSerial& serial, uint32_t baud, int8_t rxPin, int8_t txPin);

  // Non-blocking: call frequently (e.g., every loop iteration on Core0)
  void step();

  // Emit a GPSFrame to a queue at a fixed rate (frameHz).
  // Returns true if a frame was emitted.
  bool emitFrameIfDue(QueueHandle_t outQueue, uint32_t frameHz);

  // Access latest parsed state
  const GPSState& latest() const { return _latest; }

private:
  HardwareSerial* _ser = nullptr;
  TinyGPSPlus _gps;

  GPSState _latest{};
  uint32_t _lastFrameMs = 0;

  static void safeCopy(char* dst, size_t dstLen, const char* src);
  bool readLine(char* lineBuf, size_t lineBufLen);
};