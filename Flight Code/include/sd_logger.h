#pragma once
#include <Arduino.h>
#include <SD.h>
#include "spi_bus.h"
#include "common_types.h"
#include "flight_fsm.h"   // for FlightState / flightStateName

class SDLogger {
public:
  bool begin(SPIBus& bus, uint8_t sdCs, ErrorState& errs);

  const String& dir() const { return _datalogDir; }

  // Write one row per tick across all raw CSV files (timestamps must match)
  void writeTick(uint32_t t_ms, const IMUState& imu, const BaroState& baro,
                 const GPSState& gps, const ErrorState& errs);

  // Write a flight event to SD and return the formatted LoRa packet string.
  // The caller passes the returned string directly to loraMgr.send() so the
  // SD write and RF transmission always carry identical content — if RF is
  // lost the SD card still has the full record.
  String writeEvent(uint32_t t_ms, FlightState event, const char* rfLabel,
                    const GPSState& gps, const BaroState& baro,
                    float accelMag_ms2);

  void flushEvery(uint32_t nTicks);

private:
  String _datalogDir;
  File _imuFile, _baroFile, _gpsFile, _errFile, _evtFile;  // +_evtFile
  uint32_t _tickCount  = 0;
  uint32_t _flushEvery = 10;

  static String makePath(const String& dir, const char* name);
  bool createNextDatalogDir();
  bool openCSVFiles(ErrorState& errs);

  static void formatDMS(double deg, bool isLat, char* out, size_t outLen);
  static void formatLatLonDMS(double lat, double lon, char* out, size_t outLen);
};