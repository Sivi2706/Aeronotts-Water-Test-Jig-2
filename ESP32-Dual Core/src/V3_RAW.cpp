// #include <Arduino.h>
// #include <Wire.h>

// #include "esp_air_pins.h"
// #include "common_types.h"
// #include "spi_bus.h"
// #include "imu_bmi160.h"
// #include "baro_bmp280.h"
// #include "gps_mgr.h"
// #include "lora_mgr.h"
// #include "sd_logger.h"
// #include "serial_lora_mirror.h"

// // ======================================================
// //                     CONFIG
// // ======================================================
// static constexpr uint32_t SERIAL_BAUD = 115200;

// // Core assignment
// static constexpr BaseType_t CORE_GPS_RF   = 0; // Core0
// static constexpr BaseType_t CORE_SENS_LOG = 1; // Core1

// // Master logging tick: ALL files written each tick => timestamps match
// static constexpr uint32_t LOG_HZ = 10;
// static constexpr uint32_t LOG_PERIOD_MS = 1000 / LOG_HZ;

// // Sensor internal sampling
// static constexpr uint32_t IMU_HZ  = 50;
// static constexpr uint32_t BARO_HZ = 25;

// // GPS -> Core1 update rate
// static constexpr uint32_t GPS_FRAME_HZ = 5;

// // LoRa
// static constexpr long LORA_FREQ_HZ = 433E6;   // MUST match receiver
// static constexpr int  LORA_TX_PWR  = 17;
// static constexpr bool ENABLE_LORA_TELEMETRY = true;

// // Dedicated transmit period = 1 second
// static constexpr uint32_t RF_SEND_PERIOD_MS = 1000;

// // BMP280
// static constexpr uint8_t BMP_ADDR = 0x76;
// static constexpr float SEA_LEVEL_HPA = 1013.25f;

// // BMI160
// static constexpr uint8_t BMI_ADDR = 0x69;

// // Disable repeated Core1 mirrored print spam over LoRa
// // 0 = disabled
// static constexpr uint32_t CORE1_PRINT_HZ = 0;

// // ======================================================
// //                   GLOBAL MODULES
// // ======================================================
// SPIBus spiBus;

// HardwareSerial GPSSerial(2);
// GPSMgr gpsMgr;
// LoRaMgr loraMgr;

// IMUBMI160 imuMod;
// BaroBMP280 baroMod;

// SDLogger sdlog;

// // Core0->Core1 queue
// QueueHandle_t gpsQ;

// // Shared init state
// static bool sdOK = false;
// static bool loraOK = false;

// // Serial + optional LoRa mirror
// SerialLoRaMirror mirror;

// // Shared latest sensor snapshots for RF packet
// portMUX_TYPE sharedMux = portMUX_INITIALIZER_UNLOCKED;
// static IMUState  g_imuLatest{};
// static BaroState g_baroLatest{};
// static GPSState  g_gpsLatest{};

// // ======================================================
// // Helper: Serial only
// // ======================================================
// static inline void SLOG(const String& s) {
//   Serial.println(s);
// }

// static inline void SLOGF(const char* fmt, ...) {
//   char buf[220];
//   va_list ap;
//   va_start(ap, fmt);
//   vsnprintf(buf, sizeof(buf), fmt, ap);
//   va_end(ap);
//   Serial.println(buf);
// }

// // ======================================================
// // Helper: send one LoRa line directly
// // ======================================================
// static void sendLoRaLine(const String& line) {
//   if (!ENABLE_LORA_TELEMETRY || !loraOK) return;

//   // If your LoRaMgr has a different send function name,
//   // replace this with the correct one from your lora_mgr.h
//   loraMgr.send(line);
// }

// // ======================================================
// //             CORE0 TASK: GPS + LORA
// // ======================================================
// static void taskGPSandRF(void* pv) {
//   (void)pv;

//   uint32_t lastTxMs = 0;

//   SLOG("CORE0: GPS+RF task started");

//   for (;;) {
//     // 1) Non-blocking GPS processing
//     gpsMgr.step();

//     // 2) Push frames to Core1 at fixed rate
//     gpsMgr.emitFrameIfDue(gpsQ, GPS_FRAME_HZ);

//     // 3) Dedicated 1 Hz LoRa packet
//     if (ENABLE_LORA_TELEMETRY && loraOK) {
//       const uint32_t now = millis();

//       if (now - lastTxMs >= RF_SEND_PERIOD_MS) {
//         lastTxMs = now;

//         IMUState  imuSnap{};
//         BaroState baroSnap{};
//         GPSState  gpsSnap{};

//         portENTER_CRITICAL(&sharedMux);
//         imuSnap  = g_imuLatest;
//         baroSnap = g_baroLatest;
//         gpsSnap  = g_gpsLatest;
//         portEXIT_CRITICAL(&sharedMux);

//         String line;
//         line.reserve(180);

//         // Format:
//         // TX1S,time_ms,fix,lat,lon,alt_m,hdop,sats,relAlt_m,accMag
//         line += "TX1S,";
//         line += String(now);
//         line += ",";
//         line += String((int)gpsSnap.hasFix);
//         line += ",";
//         line += String(gpsSnap.lat, 6);
//         line += ",";
//         line += String(gpsSnap.lon, 6);
//         line += ",";
//         line += String(gpsSnap.alt_m, 2);
//         line += ",";
//         line += String(gpsSnap.hdop, 2);
//         line += ",";
//         line += String(gpsSnap.sats);
//         line += ",";
//         line += String(baroSnap.relAlt_m, 2);
//         line += ",";
//         line += String(imuSnap.amag_ms2, 2);

//         sendLoRaLine(line);

//         // Also print locally to Serial so you can verify
//         SLOG(line);
//       }
//     }

//     vTaskDelay(pdMS_TO_TICKS(1));
//   }
// }

// // ======================================================
// //          CORE1 TASK: SENSORS + SD LOGGING
// // ======================================================
// static void taskSensorsAndLogging(void* pv) {
//   (void)pv;

//   IMUState  imuLatest{};
//   BaroState baroLatest{};
//   GPSState  gpsLatest{};
//   ErrorState errs{};

//   float baseAlt = 0.0f;

//   SLOG("CORE1: Sensors+SD task starting...");

//   // Init sensors
//   const bool bmpOK = baroMod.begin(BMP_ADDR, SEA_LEVEL_HPA, baseAlt, errs);
//   const bool bmiOK = imuMod.begin(BMI_ADDR, errs);

//   if (bmpOK) SLOGF("BMP280: OK baseAlt=%.2f m", baseAlt);
//   else       SLOGF("BMP280: FAIL err=%s", errorCodeToStr(errs.bmp));

//   if (bmiOK) SLOG("BMI160: OK");
//   else       SLOGF("BMI160: FAIL err=%s", errorCodeToStr(errs.bmi));

//   uint32_t lastIMUms  = 0;
//   uint32_t lastBAROms = 0;
//   uint32_t lastLOGms  = 0;

//   uint32_t lastCore1PrintMs = 0;
//   const uint32_t core1PrintPeriodMs =
//     (CORE1_PRINT_HZ == 0) ? 0 : (1000UL / CORE1_PRINT_HZ);

//   GPSFrame gf{};

//   for (;;) {
//     const uint32_t now = millis();

//     // Pull latest GPS frames
//     while (xQueueReceive(gpsQ, &gf, 0) == pdTRUE) {
//       gpsLatest = gf.gps;
//     }

//     // Update IMU
//     if (now - lastIMUms >= (1000 / IMU_HZ)) {
//       lastIMUms = now;
//       if (bmiOK) imuMod.read(imuLatest, errs);
//       else {
//         imuLatest.valid = false;
//         errs.bmi = ERR_BMI_INIT_FAIL;
//       }
//     }

//     // Update BARO
//     if (now - lastBAROms >= (1000 / BARO_HZ)) {
//       lastBAROms = now;
//       if (bmpOK) baroMod.read(baroLatest, SEA_LEVEL_HPA, baseAlt, errs);
//       else {
//         baroLatest.valid = false;
//         errs.bmp = ERR_BMP_INIT_FAIL;
//       }
//     }

//     // Update shared snapshots for Core0 RF transmission
//     portENTER_CRITICAL(&sharedMux);
//     g_imuLatest  = imuLatest;
//     g_baroLatest = baroLatest;
//     g_gpsLatest  = gpsLatest;
//     portEXIT_CRITICAL(&sharedMux);

//     // Master log tick
//     if (now - lastLOGms >= LOG_PERIOD_MS) {
//       lastLOGms = now;

//       if (!gpsLatest.hasData) errs.gps = ERR_GPS_NO_DATA;
//       else if (!gpsLatest.hasFix) errs.gps = ERR_GPS_NO_FIX;
//       else errs.gps = ERR_NONE;

//       if (sdOK) {
//         sdlog.writeTick(now, imuLatest, baroLatest, gpsLatest, errs);
//       }

//       // Optional Serial-only diagnostic line
//       if (core1PrintPeriodMs != 0 && (now - lastCore1PrintMs >= core1PrintPeriodMs)) {
//         lastCore1PrintMs = now;

//         SLOGF(
//           "CORE1_LOG,%lu,IMU=%d,BARO=%d,GPSfix=%d,ALTrel=%.2f,AccMag=%.2f,ERR(bmp=%s,bmi=%s,gps=%s)",
//           (unsigned long)now,
//           (int)imuLatest.valid,
//           (int)baroLatest.valid,
//           (int)gpsLatest.hasFix,
//           baroLatest.relAlt_m,
//           imuLatest.amag_ms2,
//           errorCodeToStr(errs.bmp),
//           errorCodeToStr(errs.bmi),
//           errorCodeToStr(errs.gps)
//         );
//       }
//     }

//     vTaskDelay(pdMS_TO_TICKS(1));
//   }
// }

// // ======================================================
// //                      SETUP
// // ======================================================
// void setup() {
//   Serial.begin(SERIAL_BAUD);
//   delay(300);

//   // I2C
//   Wire.begin(ESP_I2C_SDA, ESP_I2C_SCL, 400000);

//   // SPI shared (SD + LoRa)
//   spiBus.begin(ESP_LORA_SCK, ESP_LORA_MISO, ESP_LORA_MOSI);

//   // GPS UART2
//   gpsMgr.begin(GPSSerial, ESP_GPS_BAUD, ESP_GPS_RX, ESP_GPS_TX);

//   // Queue
//   gpsQ = xQueueCreate(6, sizeof(GPSFrame));

//   ErrorState errs{};

//   // SD init
//   sdOK = sdlog.begin(spiBus, ESP_SD_CS, errs);
//   sdlog.flushEvery(10);

//   // LoRa init
//   if (ENABLE_LORA_TELEMETRY) {
//     loraOK = loraMgr.begin(
//       spiBus,
//       ESP_LORA_CS,
//       ESP_LORA_RST,
//       ESP_LORA_DIO0,
//       LORA_FREQ_HZ,
//       LORA_TX_PWR,
//       errs
//     );
//   } else {
//     errs.lora = ERR_NONE;
//     loraOK = true;
//   }

//   // Mirror started only if you still want it available,
//   // but this code does not use it for periodic RF packets.
//   mirror.begin(&loraMgr, &spiBus, &loraOK);
//   mirror.startTask(CORE_GPS_RF);

//   SLOG("=== INIT STATUS ===");
//   SLOGF("SD,%s,DIR=%s", sdOK ? "OK" : "FAIL", sdOK ? sdlog.dir().c_str() : "(none)");
//   SLOGF("LORA,%s,FREQ_MHZ=%.0f", loraOK ? "OK" : "FAIL", (double)LORA_FREQ_HZ / 1e6);

//   // Start tasks
//   xTaskCreatePinnedToCore(
//     taskSensorsAndLogging,
//     "Sensors+SD",
//     12288,
//     nullptr,
//     2,
//     nullptr,
//     CORE_SENS_LOG
//   );

//   xTaskCreatePinnedToCore(
//     taskGPSandRF,
//     "GPS+RF",
//     8192,
//     nullptr,
//     2,
//     nullptr,
//     CORE_GPS_RF
//   );
// }

// void loop() {
//   delay(1000);
// }