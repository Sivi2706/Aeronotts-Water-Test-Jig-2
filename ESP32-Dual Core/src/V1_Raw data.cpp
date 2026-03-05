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

// // Sensor internal sampling (updates “latest” values)
// static constexpr uint32_t IMU_HZ  = 50;
// static constexpr uint32_t BARO_HZ = 25;

// // GPS -> Core1 update rate
// static constexpr uint32_t GPS_FRAME_HZ = 5;
// static constexpr uint32_t GPS_PARSE_YIELD_MS = 2;

// // LoRa
// static constexpr long LORA_FREQ_HZ = 915E6;
// static constexpr int  LORA_TX_PWR  = 17;
// static constexpr bool ENABLE_LORA_TELEMETRY = true;
// static constexpr uint32_t LORA_TLM_HZ = 1;

// // BMP280
// static constexpr uint8_t BMP_ADDR = 0x76;
// static constexpr float SEA_LEVEL_HPA = 1013.25f;

// // BMI160
// static constexpr uint8_t BMI_ADDR = 0x69;

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

// //Serial Mirror Viewer
// SerialLoRaMirror mirror;

// // ======================================================
// //             CORE0 TASK: GPS + LORA
// // ======================================================
// static void taskGPSandRF(void* pv) {
//   (void)pv;

//   uint32_t lastLoRaMs = 0;
//   const uint32_t loraPeriodMs = (LORA_TLM_HZ == 0) ? 1000 : (1000 / LORA_TLM_HZ);

//   for (;;) {
//     // 1) Non-blocking GPS processing
//     gpsMgr.step();

//     // 2) Push frames to Core1 at fixed rate
//     gpsMgr.emitFrameIfDue(gpsQ, GPS_FRAME_HZ);

//     // 3) Optional LoRa telemetry at 1 Hz (or configured rate)
//     if (ENABLE_LORA_TELEMETRY && loraOK) {
//       const uint32_t now = millis();
//       if (now - lastLoRaMs >= loraPeriodMs) {
//         lastLoRaMs = now;

//         const GPSState& g = gpsMgr.latest();

//         // Keep payload short & robust
//         // Example: "T=123456,LAT=2.9465,LON=101.8753,SAT=7,FIX=1"
//         String payload;
//         payload.reserve(80);
//         payload += "T=";   payload += String(now);
//         payload += ",LAT="; payload += String(g.lat, 6);
//         payload += ",LON="; payload += String(g.lon, 6);
//         payload += ",SAT="; payload += String(g.sats);
//         payload += ",FIX="; payload += String((int)g.hasFix);

//         loraMgr.send(spiBus, payload);
//       }
//     }

//     // Yield (keeps GPS responsive without hogging CPU)
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

//   // Init sensors (Core1)
//   const bool bmpOK = baroMod.begin(BMP_ADDR, SEA_LEVEL_HPA, baseAlt, errs);
//   const bool bmiOK = imuMod.begin(BMI_ADDR, errs);

//   uint32_t lastIMUms  = 0;
//   uint32_t lastBAROms = 0;
//   uint32_t lastLOGms  = 0;

//   GPSFrame gf{};

//   for (;;) {
//     const uint32_t now = millis();

//     // Pull latest GPS frames (non-blocking)
//     while (xQueueReceive(gpsQ, &gf, 0) == pdTRUE) {
//       gpsLatest = gf.gps;
//     }

//     // Update IMU at IMU_HZ
//     if (now - lastIMUms >= (1000 / IMU_HZ)) {
//       lastIMUms = now;
//       if (bmiOK) imuMod.read(imuLatest, errs);
//       else { imuLatest.valid = false; errs.bmi = ERR_BMI_INIT_FAIL; }
//     }

//     // Update BARO at BARO_HZ
//     if (now - lastBAROms >= (1000 / BARO_HZ)) {
//       lastBAROms = now;
//       if (bmpOK) baroMod.read(baroLatest, SEA_LEVEL_HPA, baseAlt, errs);
//       else { baroLatest.valid = false; errs.bmp = ERR_BMP_INIT_FAIL; }
//     }

//     // Master log tick
//     if (now - lastLOGms >= LOG_PERIOD_MS) {
//       lastLOGms = now;

//       // Classify GPS error for this tick
//       if (!gpsLatest.hasData) errs.gps = ERR_GPS_NO_DATA;
//       else if (!gpsLatest.hasFix) errs.gps = ERR_GPS_NO_FIX;
//       else errs.gps = ERR_NONE;

//       // Write ALL CSV rows with same timestamp
//       if (sdOK) {
//         sdlog.writeTick(now, imuLatest, baroLatest, gpsLatest, errs);
//       }

//       // Debug heartbeat
//       Serial.printf("Core1 LOG t=%lu | IMU=%d BARO=%d GPSfix=%d | ERR(bmp=%s,bmi=%s,gps=%s)\n",
//         (unsigned long)now,
//         (int)imuLatest.valid, (int)baroLatest.valid, (int)gpsLatest.hasFix,
//         errorCodeToStr(errs.bmp),
//         errorCodeToStr(errs.bmi),
//         errorCodeToStr(errs.gps)
//       );
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
//   sdlog.flushEvery(10); // flush every 1s at 10Hz

//   // LoRa init
//   if (ENABLE_LORA_TELEMETRY) {
//     loraOK = loraMgr.begin(spiBus, ESP_LORA_CS, ESP_LORA_RST, ESP_LORA_DIO0, LORA_FREQ_HZ, LORA_TX_PWR, errs);
//   } else {
//     errs.lora = ERR_NONE;
//     loraOK = true;
//   }

//   Serial.println("=== INIT STATUS ===");
//   Serial.printf("SD:   %s | dir=%s\n", sdOK ? "OK" : "FAIL", sdOK ? sdlog.dir().c_str() : "(none)");
//   Serial.printf("LoRa: %s\n", loraOK ? "OK" : "FAIL");

//   // Start tasks
//   xTaskCreatePinnedToCore(taskSensorsAndLogging, "Sensors+SD", 12288, nullptr, 2, nullptr, CORE_SENS_LOG);
//   xTaskCreatePinnedToCore(taskGPSandRF,         "GPS+RF",      8192,  nullptr, 2, nullptr, CORE_GPS_RF);
// }

// void loop() {
//   delay(1000);
// }