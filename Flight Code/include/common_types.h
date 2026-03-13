#pragma once
#include <Arduino.h>

// ======================================================
//                     ERROR LOGGING
// ======================================================
enum ErrorCode : uint16_t {
  ERR_NONE             = 0,

  // Init errors
  ERR_SD_INIT_FAIL      = 100,
  ERR_LORA_INIT_FAIL    = 110,
  ERR_BMP_INIT_FAIL     = 120,
  ERR_BMI_INIT_FAIL     = 130,

  // Runtime read errors
  ERR_BMP_READ_FAIL     = 200,
  ERR_BMI_READ_FAIL     = 210,
  ERR_GPS_NO_FIX        = 220,
  ERR_GPS_NO_DATA       = 221,

  // SD file errors
  ERR_SD_FILE_OPEN_FAIL = 300,
  ERR_SD_WRITE_FAIL     = 310,
};

static inline const char* errorCodeToStr(ErrorCode c) {
  switch (c) {
    case ERR_NONE:              return "OK";
    case ERR_SD_INIT_FAIL:      return "SD_INIT_FAIL";
    case ERR_LORA_INIT_FAIL:    return "LORA_INIT_FAIL";
    case ERR_BMP_INIT_FAIL:     return "BMP_INIT_FAIL";
    case ERR_BMI_INIT_FAIL:     return "BMI_INIT_FAIL";
    case ERR_BMP_READ_FAIL:     return "BMP_READ_FAIL";
    case ERR_BMI_READ_FAIL:     return "BMI_READ_FAIL";
    case ERR_GPS_NO_FIX:        return "GPS_NO_FIX";
    case ERR_GPS_NO_DATA:       return "GPS_NO_DATA";
    case ERR_SD_FILE_OPEN_FAIL: return "SD_FILE_OPEN_FAIL";
    case ERR_SD_WRITE_FAIL:     return "SD_WRITE_FAIL";
    default:                    return "UNKNOWN_ERR";
  }
}

struct ErrorState {
  ErrorCode sd   = ERR_NONE;
  ErrorCode lora = ERR_NONE;
  ErrorCode bmp  = ERR_NONE;
  ErrorCode bmi  = ERR_NONE;
  ErrorCode gps  = ERR_NONE;
};

// ======================================================
//                    DATA STRUCTURES
// ======================================================
struct IMUState {
  float ax_ms2 = 0, ay_ms2 = 0, az_ms2 = 0;
  float gx_rad = 0, gy_rad = 0, gz_rad = 0;
  float amag_ms2 = 0;
  bool valid = false;
};

struct BaroState {
  float temp_C = 0;
  float press_hPa = 0;
  float absAlt_m = 0;
  float relAlt_m = 0;
  bool valid = false;
};

struct GPSState {
  double lat = 0.0;
  double lon = 0.0;
  double alt_m = 0.0;
  uint32_t sats = 0;
  double hdop = 0.0;

  bool hasFix = false;
  bool hasData = false;

  char lastGGA[96] = {0};
  char lastRMC[96] = {0};
};

struct GPSFrame {
  uint32_t rx_ms;
  GPSState gps;
};