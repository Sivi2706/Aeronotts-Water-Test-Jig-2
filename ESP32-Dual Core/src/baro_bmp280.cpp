#include "baro_bmp280.h"
#include <math.h>

bool BaroBMP280::begin(uint8_t addr, float seaLevel_hPa, float& outBaseAlt_m, ErrorState& errs) {
  (void)seaLevel_hPa;

  if (!_bmp.begin(addr)) {
    errs.bmp = ERR_BMP_INIT_FAIL;
    return false;
  }

  _bmp.setSampling(
    Adafruit_BMP280::MODE_NORMAL,
    Adafruit_BMP280::SAMPLING_X2,
    Adafruit_BMP280::SAMPLING_X16,
    Adafruit_BMP280::FILTER_X16,
    Adafruit_BMP280::STANDBY_MS_63
  );

  // Baseline average (~2s)
  const uint32_t start = millis();
  float sum = 0.0f;
  uint32_t n = 0;
  while (millis() - start < 2000) {
    sum += _bmp.readAltitude(seaLevel_hPa);
    n++;
    delay(40);
  }
  outBaseAlt_m = (n > 0) ? (sum / (float)n) : 0.0f;

  errs.bmp = ERR_NONE;
  return true;
}

bool BaroBMP280::read(BaroState& out, float seaLevel_hPa, float baseAlt_m, ErrorState& errs) {
  float t = _bmp.readTemperature();
  float p = _bmp.readPressure();          // Pa
  float a = _bmp.readAltitude(seaLevel_hPa);

  if (!isfinite(t) || !isfinite(p) || !isfinite(a)) {
    out.valid = false;
    errs.bmp = ERR_BMP_READ_FAIL;
    return false;
  }

  out.temp_C = t;
  out.press_hPa = p / 100.0f;
  out.absAlt_m = a;
  out.relAlt_m = a - baseAlt_m;
  out.valid = true;
  errs.bmp = ERR_NONE;
  return true;
}