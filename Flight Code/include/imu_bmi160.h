#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <DFRobot_BMI160.h>
#include "common_types.h"

class IMUBMI160 {
public:
  IMUBMI160() = default;

  // DFRobot style
  bool begin(uint8_t i2cAddr, ErrorState& errs);

  // Reads gx,gy,gz, ax,ay,az in one shot
  bool read(IMUState& out, ErrorState& errs);

private:
  DFRobot_BMI160 _bmi;

  // If you already know your accel/gyro scaling from your old code,
  // plug it in here. These defaults are safe placeholders.
  static float accelRawToMS2(int16_t raw);
  static float gyroRawToRadS(int16_t raw);
};