#pragma once
#include <Arduino.h>
#include <SD.h>
#include "spi_bus.h"
#include "common_types.h"

class SDLogger {
public:
  bool begin(SPIBus& bus, uint8_t sdCs, ErrorState& errs);

  const String& dir() const { return _datalogDir; }

  // Write one row per tick (timestamps must match)
  void writeTick(uint32_t t_ms, const IMUState& imu, const BaroState& baro, const GPSState& gps, const ErrorState& errs);

  void flushEvery(uint32_t nTicks); // flush every N ticks

private:
  String _datalogDir;
  File _imuFile, _baroFile, _gpsFile, _errFile;
  uint32_t _tickCount = 0;
  uint32_t _flushEvery = 10;

  static String makePath(const String& dir, const char* name);
  bool createNextDatalogDir();
  bool openCSVFiles(ErrorState& errs);

  static void formatDMS(double deg, bool isLat, char* out, size_t outLen);
  static void formatLatLonDMS(double lat, double lon, char* out, size_t outLen);
};