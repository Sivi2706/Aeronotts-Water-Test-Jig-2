#include "esp_air_bmp280.h"
#include "esp_air_pins.h"
#include <Wire.h>
#include <Adafruit_BMP280.h>

static Adafruit_BMP280 EspBmp;
static bool EspBmpOk = false;

bool espBmp_begin() {
  Wire.begin(ESP_I2C_SDA, ESP_I2C_SCL);

  // BMP280 supports I2C or SPI; we use I2C here. :contentReference[oaicite:1]{index=1}
  EspBmpOk = EspBmp.begin(0x76); // change to 0x77 if needed
  if (!EspBmpOk) return false;

  // Reasonable default sampling
  EspBmp.setSampling(
    Adafruit_BMP280::MODE_NORMAL,
    Adafruit_BMP280::SAMPLING_X2,   // temp
    Adafruit_BMP280::SAMPLING_X16,  // pressure
    Adafruit_BMP280::FILTER_X16,    // IIR filtering supported by BMP280 :contentReference[oaicite:2]{index=2}
    Adafruit_BMP280::STANDBY_MS_63
  );

  return true;
}

EspBmpSample espBmp_read(float seaLevel_hPa) {
  EspBmpSample s{};
  if (!EspBmpOk) { s.ok = false; return s; }

  s.ok = true;
  s.temp_C = EspBmp.readTemperature();
  s.press_hPa = EspBmp.readPressure() / 100.0f;
  s.alt_m = EspBmp.readAltitude(seaLevel_hPa);
  return s;
}