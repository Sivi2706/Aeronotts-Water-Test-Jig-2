#pragma once
#include <Arduino.h>

struct EspBmpSample {
  bool ok;
  float temp_C;
  float press_hPa;
  float alt_m;
};

bool espBmp_begin();
EspBmpSample espBmp_read(float seaLevel_hPa);