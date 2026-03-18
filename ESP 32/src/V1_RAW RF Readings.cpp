#include <Arduino.h>
#include "esp_air_pins.h"
#include "esp_air_gps.h"
#include "esp_air_bmi160.h"
#include "esp_air_bmp280.h"
#include "esp_air_sdlog.h"
#include "esp_air_lora_tx.h"
#include "esp_air_time.h"

static constexpr long LORA_FREQ_HZ = 433E6; // 433E6 / 868E6 / 915E6 (must match R4)
static constexpr float SEA_LEVEL_HPA   = 1013.25f;

static EspSdFiles g_sd;
static uint32_t g_lastStatusMs = 0;

// Track LoRa init so we don’t “send into the void”
static bool g_loraOk = false;

// -------------------- RF + Serial mirror helpers --------------------
static inline void espAir_rfPrintln(const char* line) {
  Serial.println(line);
  if (g_loraOk) espLoRaTx_sendLine(line);
}

static inline void espAir_rfPrintln(const String& line) {
  Serial.println(line);
  if (g_loraOk) espLoRaTx_sendLine(line.c_str());
}

// Print a “boot/header” line to Serial and also to LoRa
static void espAir_printBootHeader() {
  espAir_rfPrintln("ESP32 GPS + IMU + BMP -> LoRa TX + SD log starting...");
  espAir_rfPrintln("CSV: ms,fix,lat,lng,alt_m,spd_kmph,hdop,sats,utc_hhmmss,utc_ddmmyy");
}

// Build and emit the CSV line that is BOTH printed and transmitted
static void espAir_emitCsvLine(uint32_t now, const EspGpsFix& fix) {
  char csv[200];

  if (!fix.valid) {
    // Keep exact CSV schema even without fix
    // fix field: NOFIX
    snprintf(csv, sizeof(csv),
             "%lu,NOFIX,0,0,0,0,0,0,0,0",
             (unsigned long)now);
  } else {
    // fix field: FIX
    snprintf(csv, sizeof(csv),
             "%lu,FIX,%.6f,%.6f,%.2f,%.2f,%.2f,%lu,%lu,%lu",
             (unsigned long)now,
             fix.lat, fix.lng,
             fix.alt_m, fix.spd_kmph, fix.hdop,
             (unsigned long)fix.sats,
             (unsigned long)fix.utc_hhmmss,
             (unsigned long)fix.utc_ddmmyy);
  }

  // Mirror: Serial output == RF output
  espAir_rfPrintln(csv);

  // Also log GPS status once per second to SD (timestamped)
  char tbuf[16];
  espTime_formatMmSsMs(now, tbuf, sizeof(tbuf));

  char gpsLine[240];
  snprintf(gpsLine, sizeof(gpsLine), "%s,%s", tbuf, csv);
  espSd_appendLine(g_sd.gpsPath.c_str(), gpsLine);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // Start subsystems first
  espGps_begin();

  bool imuOk = espImu_begin();
  bool bmpOk = espBmp_begin();

  // LoRa init (set flag)
  if (!espLoRaTx_begin(LORA_FREQ_HZ)) {
    g_loraOk = false;
    Serial.println("LoRa init FAILED");
  } else {
    g_loraOk = true;
    Serial.println("LoRa init OK");
  }

  // Now that g_loraOk is known, print header and also broadcast it
  espAir_printBootHeader();

  // Mirror subsystem status over RF too
  {
    char msg[96];

    snprintf(msg, sizeof(msg), "IMU ok: %s", imuOk ? "YES" : "NO");
    espAir_rfPrintln(msg);

    snprintf(msg, sizeof(msg), "BMP ok: %s", bmpOk ? "YES" : "NO");
    espAir_rfPrintln(msg);
  }

  // SD init (note: SD logging is local, but we also announce status over RF)
  if (!espSd_beginAndCreateLog(g_sd)) {
    espAir_rfPrintln("SD init FAILED (check wiring/CS)");
  } else {
    Serial.print("SD log folder: ");
    Serial.println(g_sd.folder);

    // Also send folder info to R4 (optional)
    String msg = String("SD log folder: ") + g_sd.folder;
    espAir_rfPrintln(msg);
  }

  g_lastStatusMs = millis();
}

void loop() {
  const uint32_t now = millis();

  // Always keep parsing GPS stream
  espGps_poll();

  // Use one synced timestamp per loop tick:
  char tbuf[16];
  espTime_formatMmSsMs(now, tbuf, sizeof(tbuf));

  // 1) IMU raw logging (fast)
  EspImuRaw imu = espImu_readRaw();
  if (imu.ok) {
    char line[96];
    snprintf(line, sizeof(line), "%s,%d,%d,%d,%d,%d,%d",
             tbuf, imu.ax, imu.ay, imu.az, imu.gx, imu.gy, imu.gz);
    espSd_appendLine(g_sd.imuPath.c_str(), line);
  }

  // 2) BMP logging (fast)
  EspBmpSample bmp = espBmp_read(SEA_LEVEL_HPA);
  if (bmp.ok) {
    char line[96];
    snprintf(line, sizeof(line), "%s,%.2f,%.2f,%.2f",
             tbuf, bmp.temp_C, bmp.press_hPa, bmp.alt_m);
    espSd_appendLine(g_sd.bmpPath.c_str(), line);
  }

  // 3) Every 1 second: print+RF-send the SAME CSV line (mirrored)
  if (now - g_lastStatusMs >= 1000) {
    g_lastStatusMs = now;

    EspGpsFix fix = espGps_getLatestFix();
    espAir_emitCsvLine(now, fix);
  }
}