#include "esp_air_gps.h"
#include "esp_air_pins.h"
#include <TinyGPSPlus.h>

static HardwareSerial EspGpsSerial(2);
static TinyGPSPlus EspGps;
static EspGpsFix EspLast{};

// Diagnostics
static uint32_t g_lastDiagMs = 0;
static uint32_t g_bytesThisWindow = 0;
static uint32_t g_totalBytes = 0;

// Raw NMEA sniffer (prints 1 NMEA line/sec)
static uint32_t g_lastRawMs = 0;
static char g_rawBuf[140];
static uint8_t g_rawIdx = 0;

void espGps_begin() {
  EspGpsSerial.begin(ESP_GPS_BAUD, SERIAL_8N1, ESP_GPS_RX, ESP_GPS_TX);

  // Defaults
  EspLast.valid = false;
  EspLast.lat = 0.0;
  EspLast.lng = 0.0;
  EspLast.alt_m = 0.0;
  EspLast.spd_kmph = 0.0;
  EspLast.hdop = 99.99;
  EspLast.sats = 0;
  EspLast.utc_hhmmss = 0;
  EspLast.utc_ddmmyy = 0;

  g_lastDiagMs = millis();
  g_bytesThisWindow = 0;
  g_totalBytes = 0;

  g_lastRawMs = millis();
  g_rawIdx = 0;
  g_rawBuf[0] = '\0';

  Serial.print("GPS UART2 begin. BAUD=");
  Serial.print(ESP_GPS_BAUD);
  Serial.print(" RX=");
  Serial.print(ESP_GPS_RX);
  Serial.print(" TX=");
  Serial.println(ESP_GPS_TX);
}

static void nmeaSnifferFeed(char c) {
  // Build a line excluding '\r', terminate on '\n'
  if (c == '\n' || g_rawIdx >= sizeof(g_rawBuf) - 1) {
    g_rawBuf[g_rawIdx] = '\0';

    uint32_t now = millis();
    // Print at most 1 line per second (and only if it looks non-empty)
    if ((now - g_lastRawMs) >= 1000 && g_rawIdx > 6) {
      g_lastRawMs = now;
      Serial.print("NMEA:");
      Serial.println(g_rawBuf);
    }

    g_rawIdx = 0;
    return;
  }

  if (c == '\r') return;
  g_rawBuf[g_rawIdx++] = c;
}

void espGps_poll() {
  // Read UART, feed parser + sniffer
  while (EspGpsSerial.available()) {
    char c = (char)EspGpsSerial.read();
    g_bytesThisWindow++;
    g_totalBytes++;

    EspGps.encode(c);
    nmeaSnifferFeed(c);
  }

  // Update snapshot fields when valid
  if (EspGps.location.isValid()) {
    EspLast.lat = EspGps.location.lat();
    EspLast.lng = EspGps.location.lng();
  }

  if (EspGps.altitude.isValid()) EspLast.alt_m = EspGps.altitude.meters();
  if (EspGps.speed.isValid())    EspLast.spd_kmph = EspGps.speed.kmph();

  if (EspGps.hdop.isValid()) {
    // TinyGPS++ HDOP often scaled by 100
    EspLast.hdop = EspGps.hdop.value() / 100.0;
  }

  if (EspGps.satellites.isValid()) {
    EspLast.sats = EspGps.satellites.value();
  }

  // Date/time snapshot (UTC)
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

  // Better "fix" heuristic:
  // - recent location update
  // - >= 4 satellites
  // - hdop < 10 (loosen later if you want)
  const bool locRecent = EspGps.location.isValid() && (EspGps.location.age() < 5000);
  const bool satsOk    = (EspLast.sats >= 4);
  const bool hdopOk    = (!isnan(EspLast.hdop) && EspLast.hdop > 0.0 && EspLast.hdop < 10.0);

  EspLast.valid = locRecent && satsOk && hdopOk;

  // 1 Hz diagnostics
  uint32_t now = millis();
  if (now - g_lastDiagMs >= 1000) {
    g_lastDiagMs = now;

    Serial.print("GPSDIAG,ms=");
    Serial.print(now);
    Serial.print(",bytes/s=");
    Serial.print(g_bytesThisWindow);
    Serial.print(",total=");
    Serial.print(g_totalBytes);

    Serial.print(",chars=");
    Serial.print(EspGps.charsProcessed());
    Serial.print(",sentFix=");
    Serial.print(EspGps.sentencesWithFix());
    Serial.print(",csFail=");
    Serial.print(EspGps.failedChecksum());

    Serial.print(",sats=");
    Serial.print(EspLast.sats);
    Serial.print(",hdop=");
    Serial.print(EspLast.hdop, 2);

    Serial.print(",locValid=");
    Serial.print(EspGps.location.isValid() ? 1 : 0);
    Serial.print(",age=");
    Serial.print(EspGps.location.age());

    Serial.print(",FIX=");
    Serial.println(EspLast.valid ? 1 : 0);

    g_bytesThisWindow = 0;
  }
}

EspGpsFix espGps_getLatestFix() {
  return EspLast;
}