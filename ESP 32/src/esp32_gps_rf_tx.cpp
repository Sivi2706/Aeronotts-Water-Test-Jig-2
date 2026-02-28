#include "esp32_gps_rf_tx.h"

#include <SPI.h>
#include <LoRa.h>
#include <TinyGPSPlus.h>

static TinyGPSPlus gps;
static HardwareSerial GPSUART(1);

static bool lora_ok = false;

// Debug counters
static uint32_t gps_bytes = 0;
static uint32_t last_gps_bytes_print_ms = 0;

static bool lora_send_string(const String& s) {
  if (!lora_ok) return false;
  LoRa.beginPacket();
  LoRa.print(s);
  return (LoRa.endPacket() == 1);
}

bool esp32_gpsrf_init(
  long lora_freq_hz,
  int lora_cs, int lora_rst, int lora_dio0,
  int gps_rx_pin, int gps_tx_pin,
  uint32_t gps_baud
) {
  // LoRa
  SPI.begin(18, 19, 23, lora_cs);   // SCK,MISO,MOSI,SS
  LoRa.setSPI(SPI);
  LoRa.setPins(lora_cs, lora_rst, lora_dio0);

  if (!LoRa.begin(lora_freq_hz)) {
    lora_ok = false;
    return false;
  }
  lora_ok = true;

  // GPS UART (NEO-8M default 9600)
  GPSUART.begin(gps_baud, SERIAL_8N1, gps_rx_pin, gps_tx_pin);

  gps_bytes = 0;
  last_gps_bytes_print_ms = millis();
  return true;
}

void esp32_gpsrf_poll() {
  while (GPSUART.available() > 0) {
    char c = (char)GPSUART.read();
    gps_bytes++;
    gps.encode(c);

    // Optional: show raw NMEA in Serial Monitor for debugging.
    // Comment this out once confirmed working.
    Serial.write(c);
  }

  // Print bytes/sec every 1s so you know UART is alive
  uint32_t now = millis();
  if (now - last_gps_bytes_print_ms >= 1000) {
    last_gps_bytes_print_ms = now;
    Serial.print("\nGPS bytes/sec: ");
    Serial.println(gps_bytes);
    gps_bytes = 0;
  }
}

bool esp32_gpsrf_send_packet_csv() {
  if (!lora_ok) return false;

  String msg;
  msg.reserve(160);

  // If no valid fix yet, send a "NOFIX" status packet.
  if (!gps.location.isValid()) {
    // ms,NOFIX,chars,sentFix,sats,hdop
    msg += String(millis());
    msg += ",NOFIX";
    msg += "," + String(gps.charsProcessed());
    msg += "," + String(gps.sentencesWithFix());

    if (gps.satellites.isValid()) msg += "," + String(gps.satellites.value());
    else msg += ",nan";

    if (gps.hdop.isValid()) msg += "," + String(gps.hdop.value());
    else msg += ",nan";

    return lora_send_string(msg);
  }

  // If fix is valid, send full FIX packet:
  // ms,FIX,lat,lng,alt_m,spd_kmph,sats,hdop,utc_hhmmss,utc_ddmmyy
  msg += String(millis());
  msg += ",FIX";
  msg += "," + String(gps.location.lat(), 6);
  msg += "," + String(gps.location.lng(), 6);

  if (gps.altitude.isValid()) msg += "," + String(gps.altitude.meters(), 2);
  else msg += ",nan";

  if (gps.speed.isValid()) msg += "," + String(gps.speed.kmph(), 2);
  else msg += ",nan";

  if (gps.satellites.isValid()) msg += "," + String(gps.satellites.value());
  else msg += ",nan";

  if (gps.hdop.isValid()) msg += "," + String(gps.hdop.value());
  else msg += ",nan";

  if (gps.time.isValid()) {
    char tbuf[16];
    snprintf(tbuf, sizeof(tbuf), "%02d%02d%02d",
             gps.time.hour(), gps.time.minute(), gps.time.second());
    msg += "," + String(tbuf);
  } else msg += ",nan";

  if (gps.date.isValid()) {
    char dbuf[16];
    snprintf(dbuf, sizeof(dbuf), "%02d%02d%02d",
             gps.date.day(), gps.date.month(), gps.date.year() % 100);
    msg += "," + String(dbuf);
  } else msg += ",nan";

  return lora_send_string(msg);
}