#pragma once
#include <Arduino.h>

struct EspImuRaw {
  bool ok;
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
};

bool espImu_begin();      // init BMI160 over I2C
EspImuRaw espImu_readRaw();