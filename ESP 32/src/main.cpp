#include <Arduino.h>
#include "esp_air_pins.h"
#include "esp_air_gps.h"
#include "esp_air_bmi160.h"
#include "esp_air_bmp280.h"
#include "esp_air_sdlog.h"
#include "esp_air_lora_tx.h"
#include "esp_air_time.h"


static constexpr long LORA_FREQ_HZ = 915E6;     // change to 433E6 / 868E6 / 915E6
static constexpr float SEA_LEVEL_HPA = 1013.25f;

static EspSdFiles g_sd;
static uint32_t g_lastStatusMs = 0;

static void espAir_printCsvHeader() {
  Serial.println("ESP32 GPS + IMU + BMP -> LoRa TX + SD log starting...");
  Serial.println("CSV: ms,fix,lat,lng,alt_m,spd_kmph,hdop,sats,utc_hhmmss,utc_ddmmyy");
}

void setup() {
  Serial.begin(115200);
  delay(200);

  espAir_printCsvHeader();

  // Start subsystems
  espGps_begin();

  bool imuOk = espImu_begin();
  bool bmpOk = espBmp_begin();

  if (!espLoRaTx_begin(LORA_FREQ_HZ)) {
    Serial.println("LoRa init FAILED");
  } else {
    Serial.println("LoRa init OK");
  }

  if (!espSd_beginAndCreateLog(g_sd)) {
    Serial.println("SD init FAILED (check wiring/CS)");
  } else {
    Serial.print("SD log folder: ");
    Serial.println(g_sd.folder);
  }

  Serial.print("IMU ok: "); Serial.println(imuOk ? "YES" : "NO");
  Serial.print("BMP ok: "); Serial.println(bmpOk ? "YES" : "NO");

  g_lastStatusMs = millis();
}

void loop() {
  const uint32_t now = millis();

  // Always keep parsing GPS stream
  espGps_poll();

  // 1) Read IMU + BMP at whatever loop speed you get (raw logging)
  // Use one synced timestamp per loop tick:
  char tbuf[16];
  espTime_formatMmSsMs(now, tbuf, sizeof(tbuf));

  // IMU raw line
  EspImuRaw imu = espImu_readRaw();
  if (imu.ok) {
    char line[96];
    snprintf(line, sizeof(line), "%s,%d,%d,%d,%d,%d,%d",
             tbuf, imu.ax, imu.ay, imu.az, imu.gx, imu.gy, imu.gz);
    espSd_appendLine(g_sd.imuPath.c_str(), line);
  }

  // BMP line
  EspBmpSample bmp = espBmp_read(SEA_LEVEL_HPA);
  if (bmp.ok) {
    char line[96];
    snprintf(line, sizeof(line), "%s,%.2f,%.2f,%.2f",
             tbuf, bmp.temp_C, bmp.press_hPa, bmp.alt_m);
    espSd_appendLine(g_sd.bmpPath.c_str(), line);
  }

  // 2) Every 1 second: send status to ground station (GPS + summary)
  if (now - g_lastStatusMs >= 1000) {
    g_lastStatusMs = now;

    EspGpsFix fix = espGps_getLatestFix();

    char payload[160];
    if (!fix.valid) {
      snprintf(payload, sizeof(payload),
               "%lu,NOFIX,0,0,0,0,0,0,0,0",
               (unsigned long)now);
      Serial.println("TX: no valid GPS fix yet");
    } else {
      snprintf(payload, sizeof(payload),
               "%lu,FIX,%.6f,%.6f,%.2f,%.2f,%.2f,%lu,%lu,%lu",
               (unsigned long)now,
               fix.lat, fix.lng, fix.alt_m, fix.spd_kmph, fix.hdop,
               (unsigned long)fix.sats,
               (unsigned long)fix.utc_hhmmss,
               (unsigned long)fix.utc_ddmmyy);

      Serial.print("TX: ");
      Serial.println(payload);
    }

    // Log GPS status once per second too (synced timestamp string + ms)
    {
      char gpsLine[200];
      snprintf(gpsLine, sizeof(gpsLine),
               "%s,%s",
               tbuf, payload);
      espSd_appendLine(g_sd.gpsPath.c_str(), gpsLine);
    }

    // Transmit over LoRa
    espLoRaTx_sendLine(payload);
  }
}