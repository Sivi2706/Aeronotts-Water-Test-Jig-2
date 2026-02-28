#include "esp_air_bmi160.h"
#include "esp_air_pins.h"
#include <Wire.h>
#include <DFRobot_BMI160.h>

static DFRobot_BMI160 imu;
static bool imuOk = false;

static bool imu_try_init(uint8_t addr) {
  // Common in DFRobot BMI160: I2cInit(addr)
  int8_t rslt = imu.I2cInit(addr);
  return (rslt == 0);
}

bool espImu_begin() {
  Wire.begin(ESP_I2C_SDA, ESP_I2C_SCL);

  imuOk = imu_try_init(0x69);
  if (!imuOk) imuOk = imu_try_init(0x68);

  return imuOk;
}

EspImuRaw espImu_readRaw() {
  EspImuRaw out{};
  out.ok = false;
  if (!imuOk) return out;

  // ---- Try the most common patterns ----
  // Pattern A: separate getters
  //   getAccelData(int16_t *accel)
  //   getGyroData(int16_t *gyro)
  //
  // Pattern B: one 6-element buffer
  //   getAccelGyroData(int16_t *data)
  // where data = {gx,gy,gz, ax,ay,az} or {ax,ay,az,gx,gy,gz} depending on lib
  //
  // Because we can't detect overloads cleanly here, you must pick the correct block
  // by matching your library header. Start with Pattern B (most common in DFRobot).

  // ========== Pattern B: single buffer ==========
  int16_t data[6] = {0,0,0,0,0,0};
  int8_t rslt = imu.getAccelGyroData(data);  // <-- THIS is likely your actual signature

  if (rslt == 0) {
    // Many DFRobot examples use: data = {gx,gy,gz, ax,ay,az}
    out.gx = data[0]; out.gy = data[1]; out.gz = data[2];
    out.ax = data[3]; out.ay = data[4]; out.az = data[5];
    out.ok = true;
    return out;
  }

  // If Pattern B doesn't work at runtime, you can switch to Pattern A in the header match.
  return out;
}