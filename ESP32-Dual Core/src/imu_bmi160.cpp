#include "imu_bmi160.h"
#include <math.h>

static constexpr float G0 = 9.80665f;

float IMUBMI160::accelRawToMS2(int16_t raw) {
  // Common assumption: 16384 LSB/g (for +/-2g)
  // If your BMI160 is configured for another range, adjust:
  // +/-4g => 8192, +/-8g => 4096, +/-16g => 2048
  return (raw / 16384.0f) * G0;
}

float IMUBMI160::gyroRawToRadS(int16_t raw) {
  // Common assumption: 16.4 LSB/(deg/s) (for +/-2000 dps)
  // Adjust if your range differs
  const float dps = raw / 16.4f;
  return dps * 0.01745329252f;
}

bool IMUBMI160::begin(uint8_t i2cAddr, ErrorState& errs) {
  // This is the DFRobot init function used in many examples
  if (_bmi.I2cInit((int8_t)i2cAddr) != BMI160_OK) {
    errs.bmi = ERR_BMI_INIT_FAIL;
    return false;
  }
  errs.bmi = ERR_NONE;
  return true;
}

bool IMUBMI160::read(IMUState& out, ErrorState& errs) {
  int16_t d[6]; // gx,gy,gz, ax,ay,az

  // DFRobot all-in-one read
  if (_bmi.getAccelGyroData(d) != BMI160_OK) {
    out.valid = false;
    errs.bmi = ERR_BMI_READ_FAIL;
    return false;
  }

  // Indices per DFRobot: gx,gy,gz, ax,ay,az
  const int16_t gx = d[0], gy = d[1], gz = d[2];
  const int16_t ax = d[3], ay = d[4], az = d[5];

  out.ax_ms2 = accelRawToMS2(ax);
  out.ay_ms2 = accelRawToMS2(ay);
  out.az_ms2 = accelRawToMS2(az);

  out.gx_rad = gyroRawToRadS(gx);
  out.gy_rad = gyroRawToRadS(gy);
  out.gz_rad = gyroRawToRadS(gz);

  out.amag_ms2 = sqrtf(out.ax_ms2*out.ax_ms2 + out.ay_ms2*out.ay_ms2 + out.az_ms2*out.az_ms2);
  out.valid = true;
  errs.bmi = ERR_NONE;
  return true;
}