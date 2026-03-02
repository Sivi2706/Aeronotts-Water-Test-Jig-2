#include "BMI160.h"

BMI160::BMI160(TwoWire &wire) : _wire(&wire) {}

int8_t BMI160::I2cInit(int8_t i2c_addr) {
  _addr = (uint8_t)i2c_addr;

  // Basic chip ID check
  uint8_t chip = 0;
  if (!readReg(REG_CHIP_ID, chip)) return BMI160_ERR;

  // BMI160 chip id is typically 0xD1 (we only check it's non-zero and readable)
  if (chip == 0x00 || chip == 0xFF) return BMI160_ERR;

  // Soft reset then configure
  if (softReset() != BMI160_OK) return BMI160_ERR;
  if (!configureDefaults()) return BMI160_ERR;

  return BMI160_OK;
}

int8_t BMI160::softReset() {
  if (!writeReg(REG_CMD, CMD_SOFT_RESET)) return BMI160_ERR;
  delay(50); // datasheet typical reset time
  return BMI160_OK;
}

bool BMI160::configureDefaults() {
  // Put accel + gyro into normal mode
  if (!writeReg(REG_CMD, CMD_ACC_NORMAL)) return false;
  delay(10);
  if (!writeReg(REG_CMD, CMD_GYR_NORMAL)) return false;
  delay(10);

  // Configure accel: ODR ~100 Hz, normal bandwidth
  // ACC_CONF: [7:4]=ODR, [3:2]=BW, [1:0]=US (undersampling)
  // ODR=100Hz -> 0x08 (common mapping), set BW normal (0x02 << 2)
  // Many implementations use 0x28 for 100Hz + normal BW.
  if (!writeReg(REG_ACC_CONF, 0x28)) return false;

  // Accel range: ±2g => 0x03
  if (!writeReg(REG_ACC_RANGE, 0x03)) return false;

  // Gyro: ODR ~100 Hz, normal BW
  // GYR_CONF: similar mapping, many use 0x28 for 100 Hz
  if (!writeReg(REG_GYR_CONF, 0x28)) return false;

  // Gyro range: ±2000 dps => 0x00
  if (!writeReg(REG_GYR_RANGE, 0x00)) return false;

  delay(20);
  return true;
}

int8_t BMI160::getSensorData(uint8_t type, int16_t *data) {
  if (!data) return BMI160_ERR;

  if (type == onlyAccel) {
    return getAccelData(data);
  } else if (type == onlyGyro) {
    return getGyroData(data);
  } else if (type == bothAccelGyro) {
    return getAccelGyroData(data);
  }
  return BMI160_ERR;
}

int8_t BMI160::getAccelData(int16_t *data) {
  uint8_t buf[6];
  if (!readBytes(REG_DATA_ACC_X_L, buf, sizeof(buf))) return BMI160_ERR;

  // little-endian
  data[0] = (int16_t)((buf[1] << 8) | buf[0]);
  data[1] = (int16_t)((buf[3] << 8) | buf[2]);
  data[2] = (int16_t)((buf[5] << 8) | buf[4]);
  return BMI160_OK;
}

int8_t BMI160::getGyroData(int16_t *data) {
  uint8_t buf[6];
  if (!readBytes(REG_DATA_GYR_X_L, buf, sizeof(buf))) return BMI160_ERR;

  data[0] = (int16_t)((buf[1] << 8) | buf[0]);
  data[1] = (int16_t)((buf[3] << 8) | buf[2]);
  data[2] = (int16_t)((buf[5] << 8) | buf[4]);
  return BMI160_OK;
}

int8_t BMI160::getAccelGyroData(int16_t *data) {
  // DFRobot style: gx,gy,gz, ax,ay,az
  uint8_t buf[12];
  if (!readBytes(REG_DATA_GYR_X_L, buf, sizeof(buf))) return BMI160_ERR;

  int16_t gx = (int16_t)((buf[1]  << 8) | buf[0]);
  int16_t gy = (int16_t)((buf[3]  << 8) | buf[2]);
  int16_t gz = (int16_t)((buf[5]  << 8) | buf[4]);

  int16_t ax = (int16_t)((buf[7]  << 8) | buf[6]);
  int16_t ay = (int16_t)((buf[9]  << 8) | buf[8]);
  int16_t az = (int16_t)((buf[11] << 8) | buf[10]);

  data[0] = gx; data[1] = gy; data[2] = gz;
  data[3] = ax; data[4] = ay; data[5] = az;
  return BMI160_OK;
}

// ----------------- Minimal stubs -----------------
int8_t BMI160::setInt(int) { return BMI160_OK; }
int8_t BMI160::setStepCounter() { return BMI160_OK; }
int8_t BMI160::readStepCounter(uint16_t *stepVal) {
  if (stepVal) *stepVal = 0;
  return BMI160_OK;
}
int8_t BMI160::setStepPowerMode(uint8_t) { return BMI160_OK; }

// ----------------- Unit conversion helpers -----------------
// For accel ±2g: 1 g = 16384 LSB
float BMI160::accelLSB_to_mps2(int16_t raw) const {
  const float lsb_per_g = 16384.0f;
  return (raw / lsb_per_g) * G0;
}

// For gyro ±2000 dps: 16.4 LSB/°/s
float BMI160::gyroLSB_to_dps(int16_t raw) const {
  const float lsb_per_dps = 16.4f;
  return raw / lsb_per_dps;
}

// ----------------- I2C low-level -----------------
bool BMI160::writeReg(uint8_t reg, uint8_t val) {
  _wire->beginTransmission(_addr);
  _wire->write(reg);
  _wire->write(val);
  return (_wire->endTransmission() == 0);
}

bool BMI160::readReg(uint8_t reg, uint8_t &val) {
  _wire->beginTransmission(_addr);
  _wire->write(reg);
  if (_wire->endTransmission(false) != 0) return false;

  if (_wire->requestFrom((int)_addr, 1) != 1) return false;
  val = (uint8_t)_wire->read();
  return true;
}

bool BMI160::readBytes(uint8_t startReg, uint8_t *buf, size_t len) {
  _wire->beginTransmission(_addr);
  _wire->write(startReg);
  if (_wire->endTransmission(false) != 0) return false;

  size_t got = _wire->requestFrom((int)_addr, (int)len);
  if (got != len) return false;

  for (size_t i = 0; i < len; i++) buf[i] = (uint8_t)_wire->read();
  return true;
}