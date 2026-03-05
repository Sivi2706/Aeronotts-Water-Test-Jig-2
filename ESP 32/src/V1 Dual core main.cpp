// #include <Arduino.h>

// #include "esp_air_pins.h"
// #include "esp_air_gps.h"
// #include "esp_air_bmi160.h"
// #include "esp_air_bmp280.h"
// #include "esp_air_sdlog.h"
// #include "esp_air_lora_tx.h"
// #include "esp_air_time.h"

// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/semphr.h"

// // -------------------- USER SETTINGS --------------------
// static constexpr long  LORA_FREQ_HZ   = 433E6;     // 433E6 / 868E6 / 915E6 (must match R4)
// static constexpr float SEA_LEVEL_HPA  = 1013.25f;

// // Core 0 sensor logging rate:
// // "15200" interpreted as ~15.2ms => ~66Hz
// static constexpr uint32_t SENSOR_PERIOD_MS = 15;

// // GPS is typically 1Hz updates; we will TX when utc_hhmmss changes.
// // If GPS has NO valid time, we fall back to 1Hz based on millis.
// static constexpr uint32_t FALLBACK_TX_PERIOD_MS = 1000;

// // -------------------- GLOBALS --------------------
// static EspSdFiles g_sd;

// // Track LoRa init so we don’t “send into the void”
// static bool g_loraOk = false;

// // SD mutex (because both tasks write to SD)
// static SemaphoreHandle_t g_sdMutex = nullptr;

// // GPS snapshot mutex (GPS task updates, other task could read if needed)
// static SemaphoreHandle_t g_gpsMutex = nullptr;
// static EspGpsFix g_fixShared{};

// // -------------------- HELPERS --------------------
// static inline void sdAppendSafe(const char* path, const char* line) {
//   if (!path || !line) return;
//   if (!g_sdMutex) { espSd_appendLine(path, line); return; }

//   if (xSemaphoreTake(g_sdMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
//     espSd_appendLine(path, line);
//     xSemaphoreGive(g_sdMutex);
//   }
// }

// static inline void sdAppendSafe(const String& path, const char* line) {
//   sdAppendSafe(path.c_str(), line);
// }

// static inline void rfPrintlnMirror(const char* line) {
//   Serial.println(line);
//   if (g_loraOk) espLoRaTx_sendLine(line);
// }

// static inline void rfPrintlnMirror(const String& line) {
//   Serial.println(line);
//   if (g_loraOk) espLoRaTx_sendLine(line.c_str());
// }

// // Print a “boot/header” line to Serial and also to LoRa
// static void printBootHeader() {
//   rfPrintlnMirror("ESP32 Dual-core: Core0=IMU/BMP->SD, Core1=GPS+LoRa TX");
//   rfPrintlnMirror("CSV: ms,fix,lat,lng,alt_m,spd_kmph,hdop,sats,utc_hhmmss,utc_ddmmyy");
// }

// // Build CSV payload (same schema as your current ESP32 output)
// static void buildCsv(char* out, size_t outSz, uint32_t nowMs, const EspGpsFix& fix) {
//   if (!out || outSz == 0) return;

//   if (!fix.valid) {
//     snprintf(out, outSz,
//              "%lu,NOFIX,0,0,0,0,0,0,0,0",
//              (unsigned long)nowMs);
//   } else {
//     snprintf(out, outSz,
//              "%lu,FIX,%.6f,%.6f,%.2f,%.2f,%.2f,%lu,%lu,%lu",
//              (unsigned long)nowMs,
//              fix.lat, fix.lng,
//              fix.alt_m, fix.spd_kmph, fix.hdop,
//              (unsigned long)fix.sats,
//              (unsigned long)fix.utc_hhmmss,
//              (unsigned long)fix.utc_ddmmyy);
//   }
// }

// // Update shared fix with mutex
// static void gpsFixWriteShared(const EspGpsFix& fix) {
//   if (!g_gpsMutex) { g_fixShared = fix; return; }
//   if (xSemaphoreTake(g_gpsMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
//     g_fixShared = fix;
//     xSemaphoreGive(g_gpsMutex);
//   }
// }

// static EspGpsFix gpsFixReadShared() {
//   EspGpsFix f{};
//   if (!g_gpsMutex) return g_fixShared;
//   if (xSemaphoreTake(g_gpsMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
//     f = g_fixShared;
//     xSemaphoreGive(g_gpsMutex);
//   }
//   return f;
// }

// // -------------------- TASK: Core 0 --------------------
// // IMU + BMP280 raw read + SD logging (fast loop)
// static void TaskSensorSD(void* pv) {
//   (void)pv;

//   TickType_t lastWake = xTaskGetTickCount();
//   const TickType_t periodTicks = pdMS_TO_TICKS(SENSOR_PERIOD_MS);

//   for (;;) {
//     const uint32_t now = millis();

//     // Use one synced timestamp string per tick
//     char tbuf[16];
//     espTime_formatMmSsMs(now, tbuf, sizeof(tbuf));

//     // IMU raw line
//     EspImuRaw imu = espImu_readRaw();
//     if (imu.ok) {
//       char line[120];
//       snprintf(line, sizeof(line),
//                "%s,%d,%d,%d,%d,%d,%d",
//                tbuf, imu.ax, imu.ay, imu.az, imu.gx, imu.gy, imu.gz);
//       sdAppendSafe(g_sd.imuPath, line);
//     }

//     // BMP line
//     EspBmpSample bmp = espBmp_read(SEA_LEVEL_HPA);
//     if (bmp.ok) {
//       char line[120];
//       snprintf(line, sizeof(line),
//                "%s,%.2f,%.2f,%.2f",
//                tbuf, bmp.temp_C, bmp.press_hPa, bmp.alt_m);
//       sdAppendSafe(g_sd.bmpPath, line);
//     }

//     vTaskDelayUntil(&lastWake, periodTicks);
//   }
// }

// // -------------------- TASK: Core 1 --------------------
// // GPS polling (UART2 @ 9600) + LoRa transmit synced to GPS seconds + SD GPS logging
// static void TaskGpsRadio(void* pv) {
//   (void)pv;

//   uint32_t lastFallbackTxMs = 0;
//   uint32_t lastUtc = 0;          // last utc_hhmmss sent
//   bool lastUtcValid = false;

//   for (;;) {
//     const uint32_t now = millis();

//     // Keep parsing GPS stream often (this is the important part)
//     espGps_poll();

//     // Get the latest snapshot produced by espGps_poll()
//     EspGpsFix fix = espGps_getLatestFix();
//     gpsFixWriteShared(fix);

//     // Decide when to transmit:
//     // 1) If GPS has valid utc_hhmmss, TX when it changes (new second).
//     // 2) Otherwise, fallback to millis-based 1Hz TX.
//     bool doTx = false;

//     if (fix.valid && fix.utc_hhmmss != 0) {
//       if (!lastUtcValid || fix.utc_hhmmss != lastUtc) {
//         doTx = true;
//         lastUtc = fix.utc_hhmmss;
//         lastUtcValid = true;
//       }
//     } else {
//       if (now - lastFallbackTxMs >= FALLBACK_TX_PERIOD_MS) {
//         doTx = true;
//         lastFallbackTxMs = now;
//       }
//       lastUtcValid = false;
//       lastUtc = 0;
//     }

//     if (doTx) {
//       char payload[220];
//       buildCsv(payload, sizeof(payload), now, fix);

//       // Mirror: Serial output == RF output
//       rfPrintlnMirror(payload);

//       // Also log GPS to SD (timestamped)
//       char tbuf[16];
//       espTime_formatMmSsMs(now, tbuf, sizeof(tbuf));

//       char gpsLine[260];
//       snprintf(gpsLine, sizeof(gpsLine),
//                "%s,%s",
//                tbuf, payload);
//       sdAppendSafe(g_sd.gpsPath, gpsLine);
//     }

//     // Tight loop but not starving other tasks
//     vTaskDelay(pdMS_TO_TICKS(5));
//   }
// }

// // -------------------- SETUP / LOOP --------------------
// void setup() {
//   Serial.begin(115200);
//   delay(200);

//   // Mutexes
//   g_sdMutex  = xSemaphoreCreateMutex();
//   g_gpsMutex = xSemaphoreCreateMutex();

//   // Start subsystems (init in setup, runtime in tasks)
//   espGps_begin();

//   const bool imuOk = espImu_begin();
//   const bool bmpOk = espBmp_begin();

//   if (!espLoRaTx_begin(LORA_FREQ_HZ)) {
//     g_loraOk = false;
//     Serial.println("LoRa init FAILED");
//   } else {
//     g_loraOk = true;
//     Serial.println("LoRa init OK");
//   }

//   printBootHeader();

//   {
//     char msg[96];
//     snprintf(msg, sizeof(msg), "IMU ok: %s", imuOk ? "YES" : "NO");
//     rfPrintlnMirror(msg);
//     snprintf(msg, sizeof(msg), "BMP ok: %s", bmpOk ? "YES" : "NO");
//     rfPrintlnMirror(msg);
//   }

//   if (!espSd_beginAndCreateLog(g_sd)) {
//     rfPrintlnMirror("SD init FAILED (check wiring/CS)");
//   } else {
//     Serial.print("SD log folder: ");
//     Serial.println(g_sd.folder);
//     rfPrintlnMirror(String("SD log folder: ") + g_sd.folder);
//   }

//   // Create tasks pinned to cores
//   // ESP32 core mapping (Arduino):
//   //   core 0 and core 1 are available; WiFi typically lives on core 0,
//   //   but this split usually works fine for sensor/logging vs GPS/radio.

//   // Core 0: Sensor + SD raw logging
//   xTaskCreatePinnedToCore(
//     TaskSensorSD,
//     "TaskSensorSD",
//     8192,
//     nullptr,
//     2,
//     nullptr,
//     0
//   );

//   // Core 1: GPS poll + LoRa TX + GPS SD log
//   xTaskCreatePinnedToCore(
//     TaskGpsRadio,
//     "TaskGpsRadio",
//     8192,
//     nullptr,
//     2,
//     nullptr,
//     1
//   );

//   Serial.println("Tasks started: Core0=Sensors/SD, Core1=GPS/LoRa");
// }

// void loop() {
//   // Everything runs in tasks now.
//   vTaskDelay(pdMS_TO_TICKS(1000));
// }