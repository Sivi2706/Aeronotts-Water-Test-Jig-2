#include "gps_mgr.h"
#include <string.h>

void GPSMgr::begin(HardwareSerial& serial, uint32_t baud, int8_t rxPin, int8_t txPin) {
  _ser = &serial;
  _ser->begin(baud, SERIAL_8N1, rxPin, txPin);
}

void GPSMgr::safeCopy(char* dst, size_t dstLen, const char* src) {
  if (!dst || dstLen == 0) return;
  strncpy(dst, src ? src : "", dstLen - 1);
  dst[dstLen - 1] = '\0';
}

bool GPSMgr::readLine(char* lineBuf, size_t lineBufLen) {
  static char buf[128];
  static size_t idx = 0;

  if (!_ser) return false;

  while (_ser->available() > 0) {
    char c = (char)_ser->read();

    // TinyGPS++ needs the bytes too:
    _gps.encode(c);

    if (c == '\r') continue;

    if (c == '\n') {
      buf[idx] = '\0';
      idx = 0;
      safeCopy(lineBuf, lineBufLen, buf);
      return true;
    }

    if (idx < sizeof(buf) - 1) {
      buf[idx++] = c;
    } else {
      // overflow: reset
      idx = 0;
    }
  }
  return false;
}

void GPSMgr::step() {
  if (!_ser) return;

  // Read/parse incoming bytes, and opportunistically capture GGA/RMC lines
  char line[128];
  if (readLine(line, sizeof(line))) {
    if (strstr(line, "GGA")) safeCopy(_latest.lastGGA, sizeof(_latest.lastGGA), line);
    if (strstr(line, "RMC")) safeCopy(_latest.lastRMC, sizeof(_latest.lastRMC), line);
  }

  // Update parsed fields from TinyGPS++
  _latest.hasData = _gps.location.isValid() || _gps.date.isValid() || _gps.time.isValid();

  if (_gps.location.isValid()) {
    _latest.lat = _gps.location.lat();
    _latest.lon = _gps.location.lng();
  }
  if (_gps.altitude.isValid()) {
    _latest.alt_m = _gps.altitude.meters();
  }
  if (_gps.satellites.isValid()) {
    _latest.sats = _gps.satellites.value();
  }
  if (_gps.hdop.isValid()) {
    _latest.hdop = _gps.hdop.hdop();
  }

  _latest.hasFix = _gps.location.isValid() && (_latest.sats > 0);
}

bool GPSMgr::emitFrameIfDue(QueueHandle_t outQueue, uint32_t frameHz) {
  if (!outQueue || frameHz == 0) return false;

  const uint32_t now = millis();
  const uint32_t period = 1000UL / frameHz;

  if (now - _lastFrameMs < period) return false;
  _lastFrameMs = now;

  GPSFrame f{};
  f.rx_ms = now;
  f.gps = _latest;

  // Non-blocking send; if full, drop oldest then send
  if (xQueueSend(outQueue, &f, 0) != pdTRUE) {
    GPSFrame dump;
    xQueueReceive(outQueue, &dump, 0);
    xQueueSend(outQueue, &f, 0);
  }
  return true;
}