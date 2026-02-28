#include "esp_air_gps.h"
#include "esp_air_pins.h"
#include <TinyGPSPlus.h>

static HardwareSerial EspGpsSerial(2);
static TinyGPSPlus EspGps;

static EspGpsFix EspLast{};

void espGps_begin() {
  EspGpsSerial.begin(ESP_GPS_BAUD, SERIAL_8N1, ESP_GPS_RX, ESP_GPS_TX);
  EspLast.valid = false;
}

void espGps_poll() {
  while (EspGpsSerial.available()) {
    EspGps.encode(EspGpsSerial.read());
  }

  // update snapshot when fields are valid/updated
  if (EspGps.location.isValid()) {
    EspLast.lat = EspGps.location.lat();
    EspLast.lng = EspGps.location.lng();
  }

  if (EspGps.altitude.isValid()) EspLast.alt_m = EspGps.altitude.meters();
  if (EspGps.speed.isValid())    EspLast.spd_kmph = EspGps.speed.kmph();
  if (EspGps.hdop.isValid())     EspLast.hdop = EspGps.hdop.value() / 100.0; // TinyGPS++ often scales HDOP by 100

  if (EspGps.satellites.isValid()) EspLast.sats = EspGps.satellites.value();

  // TinyGPS++ date/time
  if (EspGps.time.isValid()) {
    EspLast.utc_hhmmss =
      (uint32_t)EspGps.time.hour() * 10000UL +
      (uint32_t)EspGps.time.minute() * 100UL +
      (uint32_t)EspGps.time.second();
  }
  if (EspGps.date.isValid()) {
    EspLast.utc_ddmmyy =
      (uint32_t)EspGps.date.day() * 10000UL +
      (uint32_t)EspGps.date.month() * 100UL +
      (uint32_t)(EspGps.date.year() % 100);
  }

  EspLast.valid = EspGps.location.isValid() && EspGps.location.age() < 2000; // simple freshness rule
}

EspGpsFix espGps_getLatestFix() {
  return EspLast;
}