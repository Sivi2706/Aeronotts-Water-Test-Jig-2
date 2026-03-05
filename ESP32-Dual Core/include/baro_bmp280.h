#pragma once
#include <Arduino.h>
#include <Adafruit_BMP280.h>
#include "common_types.h"

class BaroBMP280 {
public:
  bool begin(uint8_t addr, float seaLevel_hPa, float& outBaseAlt_m, ErrorState& errs);
  bool read(BaroState& out, float seaLevel_hPa, float baseAlt_m, ErrorState& errs);

private:
  Adafruit_BMP280 _bmp;
};