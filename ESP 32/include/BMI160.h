#pragma once
#include <Arduino.h>
#include <Wire.h>

// ----------------- Return codes -----------------
#ifndef BMI160_OK
#define BMI160_OK 0
#endif
#ifndef BMI160_ERR
#define BMI160_ERR -1
#endif

// Default I2C address (SDO/SDIO to GND => 0x68, to VDDIO => 0x69)
#ifndef BMI160_I2C_ADDR
#define BMI160_I2C_ADDR 0x68
#endif

// ----------------- Data mode types -----------------
enum {
  onlyAccel     = 0,
  onlyGyro      = 1,
  bothAccelGyro = 2
};

class BMI160 {
public:
  BMI160(TwoWire &wire = Wire);

  /**
   * @fn I2cInit
   * @brief set the i2c addr and init the i2c.
   * @param i2c_addr  bmi160 i2c addr (0x68 or 0x69)
   * @return BMI160_OK(0) means success
   */
  int8_t I2cInit(int8_t i2c_addr = BMI160_I2C_ADDR);

  /**
   * @fn getSensorData
   * @brief select mode and save returned data to parameter data.
   * @param type  onlyAccel / onlyGyro / bothAccelGyro
   * @param data  buffer:
   *              onlyAccel -> int16_t[3] ax,ay,az
   *              onlyGyro  -> int16_t[3] gx,gy,gz
   *              both      -> int16_t[6] gx,gy,gz, ax,ay,az  (DFRobot style)
   * @return BMI160_OK(0) means success
   */
  int8_t getSensorData(uint8_t type, int16_t *data);

  int8_t getAccelData(int16_t *data);       // ax,ay,az
  int8_t getGyroData(int16_t *data);        // gx,gy,gz
  int8_t getAccelGyroData(int16_t *data);   // gx,gy,gz,ax,ay,az

  int8_t softReset();

  // Minimal stubs (compile-safe). You can expand later if you need steps/interrupts.
  int8_t setInt(int intNum);
  int8_t setStepCounter();
  int8_t readStepCounter(uint16_t *stepVal);
  int8_t setStepPowerMode(uint8_t model);

  // Helpers: convert raw to physical units (based on configured ranges)
  float accelLSB_to_mps2(int16_t raw) const; // m/s^2
  float gyroLSB_to_dps(int16_t raw) const;   // deg/s

private:
  TwoWire *_wire;
  uint8_t _addr = BMI160_I2C_ADDR;

  // Current configured ranges (kept simple)
  // accel: ±2g, gyro: ±2000 dps by default in this driver
  static constexpr float G0 = 9.80665f;

  // --- Registers (BMI160) ---
  static constexpr uint8_t REG_CHIP_ID    = 0x00;
  static constexpr uint8_t REG_ERR_REG    = 0x02;
  static constexpr uint8_t REG_PMU_STATUS = 0x03;

  static constexpr uint8_t REG_DATA_GYR_X_L = 0x0C; // gyro starts here
  static constexpr uint8_t REG_DATA_ACC_X_L = 0x12; // accel starts here

  static constexpr uint8_t REG_ACC_CONF   = 0x40;
  static constexpr uint8_t REG_ACC_RANGE  = 0x41;
  static constexpr uint8_t REG_GYR_CONF   = 0x42;
  static constexpr uint8_t REG_GYR_RANGE  = 0x43;

  static constexpr uint8_t REG_CMD        = 0x7E;

  // Commands
  static constexpr uint8_t CMD_SOFT_RESET = 0xB6;
  static constexpr uint8_t CMD_ACC_NORMAL = 0x11;
  static constexpr uint8_t CMD_GYR_NORMAL = 0x15;

  // I2C ops
  bool writeReg(uint8_t reg, uint8_t val);
  bool readReg(uint8_t reg, uint8_t &val);
  bool readBytes(uint8_t startReg, uint8_t *buf, size_t len);

  // Setup
  bool configureDefaults();
};