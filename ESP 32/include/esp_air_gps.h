#pragma once
#include <Arduino.h>

struct EspGpsFix {
  bool valid;
  double lat;
  double lng;
  double alt_m;
  double spd_kmph;
  double hdop;
  uint32_t sats;
  uint32_t utc_hhmmss;
  uint32_t utc_ddmmyy;
};

void espGps_begin();
void espGps_poll();                 // call frequently
EspGpsFix espGps_getLatestFix();    // last known fix snapshot