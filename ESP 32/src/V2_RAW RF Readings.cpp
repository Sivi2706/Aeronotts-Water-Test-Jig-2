// #include <Arduino.h>
// #include <SPI.h>
// #include <SD.h>
// #include <Wire.h>

// #include <LoRa.h>
// #include <TinyGPSPlus.h>
// #include <Adafruit_BMP280.h>

// #include "BMI160.h"
// #include "esp_air_pins.h"

// // ======================================================
// //                     CONFIG
// // ======================================================
// static constexpr uint32_t SERIAL_BAUD = 115200;

// // Core assignment
// static constexpr BaseType_t CORE_GPS_RF   = 0; // Core 0
// static constexpr BaseType_t CORE_SENS_LOG = 1; // Core 1

// // Master logging tick (ALL files written each tick => timestamps match)
// static constexpr uint32_t LOG_HZ = 10;                 // 10 Hz common timestamp
// static constexpr uint32_t LOG_PERIOD_MS = 1000 / LOG_HZ;

// // Sensor internal sampling (updates the "latest" values)
// static constexpr uint32_t IMU_HZ  = 50;                // IMU updates faster than logging
// static constexpr uint32_t BARO_HZ = 25;                // baro updates faster than logging

// // GPS parsing / queue update
// static constexpr uint32_t GPS_FRAME_HZ = 5;            // send GPS frame to Core1 at 5Hz
// static constexpr uint32_t GPS_PARSE_YIELD_MS = 2;

// // LoRa (optional)
// static constexpr long LORA_FREQ_HZ = 915E6;            // change if needed
// static constexpr int  LORA_TX_PWR  = 17;
// static constexpr bool ENABLE_LORA_TELEMETRY = true;    // set false if you only want logging
// static constexpr uint32_t LORA_TLM_HZ = 1;             // 1 Hz TX

// // BMP280
// static constexpr uint8_t BMP_ADDR = 0x76;              // change to 0x77 if needed
// static constexpr float SEA_LEVEL_HPA = 1013.25f;

// // BMI160
// static constexpr uint8_t BMI_ADDR = 0x69;

// // Directory prefix
// static const char* DATALOG_PREFIX = "/Datalog_";

// // CSV filenames (inside folder)
// static const char* IMU_CSV_NAME   = "imu_raw.csv";
// static const char* BARO_CSV_NAME  = "baro_raw.csv";
// static const char* GPS_CSV_NAME   = "gps_raw.csv";
// static const char* ERR_CSV_NAME   = "error_log.csv";

// // ======================================================
// //                SHARED RESOURCES
// // ======================================================
// SPIClass vspi(VSPI);
// SemaphoreHandle_t spiMutex;

// // GPS runs on Core0
// HardwareSerial GPSSerial(2);
// TinyGPSPlus gps;

// // Sensors on Core1
// Adafruit_BMP280 bmp;
// BMI160 imu(Wire);

// // SD (Core1 only)
// bool sdOK = false;
// String datalogDir;
// File imuFile, baroFile, gpsFile, errFile;

// // ======================================================
// //                    ERROR LOGGING
// // ======================================================
// enum ErrorCode : uint16_t {
//   ERR_NONE             = 0,

//   // Init errors
//   ERR_SD_INIT_FAIL      = 100,
//   ERR_LORA_INIT_FAIL    = 110,
//   ERR_BMP_INIT_FAIL     = 120,
//   ERR_BMI_INIT_FAIL     = 130,

//   // Runtime read errors
//   ERR_BMP_READ_FAIL     = 200,
//   ERR_BMI_READ_FAIL     = 210,
//   ERR_GPS_NO_FIX        = 220,
//   ERR_GPS_NO_DATA       = 221,

//   // SD file errors
//   ERR_SD_FILE_OPEN_FAIL = 300,
//   ERR_SD_WRITE_FAIL     = 310,
// };

// static const char* errorCodeToStr(ErrorCode c) {
//   switch (c) {
//     case ERR_NONE:              return "OK";
//     case ERR_SD_INIT_FAIL:      return "SD_INIT_FAIL";
//     case ERR_LORA_INIT_FAIL:    return "LORA_INIT_FAIL";
//     case ERR_BMP_INIT_FAIL:     return "BMP_INIT_FAIL";
//     case ERR_BMI_INIT_FAIL:     return "BMI_INIT_FAIL";
//     case ERR_BMP_READ_FAIL:     return "BMP_READ_FAIL";
//     case ERR_BMI_READ_FAIL:     return "BMI_READ_FAIL";
//     case ERR_GPS_NO_FIX:        return "GPS_NO_FIX";
//     case ERR_GPS_NO_DATA:       return "GPS_NO_DATA";
//     case ERR_SD_FILE_OPEN_FAIL: return "SD_FILE_OPEN_FAIL";
//     case ERR_SD_WRITE_FAIL:     return "SD_WRITE_FAIL";
//     default:                    return "UNKNOWN_ERR";
//   }
// }

// // We keep latest error state per component and only "event-log" when it changes,
// // but we still write an error summary row on every master tick.
// struct ErrorState {
//   ErrorCode sd   = ERR_NONE;
//   ErrorCode lora = ERR_NONE;
//   ErrorCode bmp  = ERR_NONE;
//   ErrorCode bmi  = ERR_NONE;
//   ErrorCode gps  = ERR_NONE;
// };

// // ======================================================
// //                    DATA STRUCTURES
// // ======================================================
// struct IMUState {
//   // raw converted values (SI units)
//   float ax_ms2 = 0, ay_ms2 = 0, az_ms2 = 0;
//   float gx_rad = 0, gy_rad = 0, gz_rad = 0;
//   float amag_ms2 = 0;
//   bool valid = false;
// };

// struct BaroState {
//   float temp_C = 0;
//   float press_hPa = 0;
//   float absAlt_m = 0;
//   float relAlt_m = 0;
//   bool valid = false;
// };

// struct GPSState {
//   // Parsed fields
//   double lat = 0.0;
//   double lon = 0.0;
//   double alt_m = 0.0;
//   uint32_t sats = 0;
//   double hdop = 0.0;

//   bool hasFix = false;
//   bool hasData = false;

//   // raw NMEA (latest complete lines captured on Core0)
//   char lastGGA[96] = {0};
//   char lastRMC[96] = {0};
// };

// // Core0 -> Core1 queue
// struct GPSFrame {
//   uint32_t rx_ms;          // time Core0 produced the frame (for debugging)
//   GPSState gps;
// };

// // ======================================================
// //                     QUEUES
// // ======================================================
// QueueHandle_t gpsQ; // Core0 -> Core1 (latest GPS frame)

// // ======================================================
// //                      SPI MUTEX
// // ======================================================
// static bool takeSPI(uint32_t timeout_ms = 50) {
//   if (!spiMutex) return true;
//   return xSemaphoreTake(spiMutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
// }
// static void giveSPI() {
//   if (!spiMutex) return;
//   xSemaphoreGive(spiMutex);
// }

// // ======================================================
// //                GPS DMS FORMATTER
// // ======================================================
// static void formatDMS(double deg, bool isLat, char* out, size_t outLen) {
//   // deg is signed decimal degrees
//   char hemi;
//   if (isLat) hemi = (deg >= 0) ? 'N' : 'S';
//   else       hemi = (deg >= 0) ? 'E' : 'W';

//   double a = fabs(deg);
//   int d = (int)a;
//   double minFloat = (a - d) * 60.0;
//   int m = (int)minFloat;
//   double s = (minFloat - m) * 60.0;

//   // seconds with 1 decimal, like 47.4
//   // Format: 2°56'47.4"N
//   snprintf(out, outLen, "%d%c%02d'%04.1f\"%c", d, 0xB0, m, s, hemi);
// }

// static void formatLatLonDMS(double lat, double lon, char* out, size_t outLen) {
//   char latDMS[32], lonDMS[32];
//   formatDMS(lat, true,  latDMS, sizeof(latDMS));
//   formatDMS(lon, false, lonDMS, sizeof(lonDMS));
//   snprintf(out, outLen, "%s %s", latDMS, lonDMS);
// }

// // ======================================================
// //             SD DIRECTORY + FILE CREATION
// // ======================================================
// static String makePath(const String& dir, const char* name) {
//   return dir + "/" + String(name);
// }

// static bool createNextDatalogDir() {
//   // Finds next available /Datalog_001, /Datalog_002, ...
//   for (int i = 1; i <= 999; i++) {
//     char buf[24];
//     snprintf(buf, sizeof(buf), "%s%03d", DATALOG_PREFIX, i); // "/Datalog_001"
//     String candidate(buf);

//     if (!SD.exists(candidate)) {
//       if (SD.mkdir(candidate)) {
//         datalogDir = candidate;
//         return true;
//       }
//       return false;
//     }
//   }
//   return false;
// }

// static bool openCSVFiles(ErrorState &errs) {
//   // Open all CSV files (append)
//   imuFile  = SD.open(makePath(datalogDir, IMU_CSV_NAME),  FILE_APPEND);
//   baroFile = SD.open(makePath(datalogDir, BARO_CSV_NAME), FILE_APPEND);
//   gpsFile  = SD.open(makePath(datalogDir, GPS_CSV_NAME),  FILE_APPEND);
//   errFile  = SD.open(makePath(datalogDir, ERR_CSV_NAME),  FILE_APPEND);

//   if (!imuFile || !baroFile || !gpsFile || !errFile) {
//     errs.sd = ERR_SD_FILE_OPEN_FAIL;
//     return false;
//   }

//   // Write headers if empty
//   if (imuFile.size() == 0) {
//     imuFile.println("t_ms,ax_ms2,ay_ms2,az_ms2,gx_rad,gy_rad,gz_rad,accMag_ms2,valid");
//     imuFile.flush();
//   }
//   if (baroFile.size() == 0) {
//     baroFile.println("t_ms,temp_C,press_hPa,absAlt_m,relAlt_m,valid");
//     baroFile.flush();
//   }
//   if (gpsFile.size() == 0) {
//     gpsFile.println("t_ms,lat,lon,alt_m,sats,hdop,hasFix,coord_dms,lastGGA,lastRMC");
//     gpsFile.flush();
//   }
//   if (errFile.size() == 0) {
//     errFile.println("t_ms,sd,lora,bmp,bmi,gps");
//     errFile.flush();
//   }

//   return true;
// }

// // ======================================================
// //                 SENSOR INITIALIZATION
// // ======================================================
// static bool initBMP280(float &baseAlt_m, ErrorState &errs) {
//   if (!bmp.begin(BMP_ADDR)) {
//     errs.bmp = ERR_BMP_INIT_FAIL;
//     return false;
//   }

//   bmp.setSampling(
//     Adafruit_BMP280::MODE_NORMAL,
//     Adafruit_BMP280::SAMPLING_X2,
//     Adafruit_BMP280::SAMPLING_X16,
//     Adafruit_BMP280::FILTER_X16,
//     Adafruit_BMP280::STANDBY_MS_63
//   );

//   // Baseline average (2s)
//   const uint32_t start = millis();
//   float sum = 0.0f;
//   uint32_t n = 0;
//   while (millis() - start < 2000) {
//     sum += bmp.readAltitude(SEA_LEVEL_HPA);
//     n++;
//     delay(40);
//   }
//   baseAlt_m = (n > 0) ? (sum / (float)n) : 0.0f;

//   errs.bmp = ERR_NONE;
//   return true;
// }

// static bool initBMI160(ErrorState &errs) {
//   if (imu.I2cInit(BMI_ADDR) != BMI160_OK) {
//     errs.bmi = ERR_BMI_INIT_FAIL;
//     return false;
//   }
//   errs.bmi = ERR_NONE;
//   return true;
// }

// // ======================================================
// //                SENSOR READ (CORE1)
// // ======================================================
// static bool readIMU(IMUState &out, ErrorState &errs) {
//   int16_t d[6]; // gx,gy,gz, ax,ay,az
//   if (imu.getAccelGyroData(d) != BMI160_OK) {
//     out.valid = false;
//     errs.bmi = ERR_BMI_READ_FAIL;
//     return false;
//   }

//   out.ax_ms2 = imu.accelLSB_to_mps2(d[3]);
//   out.ay_ms2 = imu.accelLSB_to_mps2(d[4]);
//   out.az_ms2 = imu.accelLSB_to_mps2(d[5]);

//   const float DEG2RAD = 0.01745329252f;
//   out.gx_rad = imu.gyroLSB_to_dps(d[0]) * DEG2RAD;
//   out.gy_rad = imu.gyroLSB_to_dps(d[1]) * DEG2RAD;
//   out.gz_rad = imu.gyroLSB_to_dps(d[2]) * DEG2RAD;

//   out.amag_ms2 = sqrtf(out.ax_ms2*out.ax_ms2 + out.ay_ms2*out.ay_ms2 + out.az_ms2*out.az_ms2);
//   out.valid = true;
//   errs.bmi = ERR_NONE;
//   return true;
// }

// static bool readBaro(BaroState &out, float baseAlt_m, ErrorState &errs) {
//   // Read temp/pressure/alt
//   float t = bmp.readTemperature();
//   float p = bmp.readPressure(); // Pa
//   float a = bmp.readAltitude(SEA_LEVEL_HPA);

//   if (!isfinite(t) || !isfinite(p) || !isfinite(a)) {
//     out.valid = false;
//     errs.bmp = ERR_BMP_READ_FAIL;
//     return false;
//   }

//   out.temp_C = t;
//   out.press_hPa = p / 100.0f;
//   out.absAlt_m = a;
//   out.relAlt_m = a - baseAlt_m;
//   out.valid = true;
//   errs.bmp = ERR_NONE;
//   return true;
// }

// // ======================================================
// //                  LORA (CORE0)
// // ======================================================
// static bool initLoRa(ErrorState &errs) {
//   if (!takeSPI(200)) {
//     errs.lora = ERR_LORA_INIT_FAIL;
//     return false;
//   }

//   LoRa.setPins(ESP_LORA_CS, ESP_LORA_RST, ESP_LORA_DIO0);
//   LoRa.setSPI(vspi);

//   bool ok = LoRa.begin(LORA_FREQ_HZ);
//   if (ok) {
//     LoRa.setTxPower(LORA_TX_PWR);
//     errs.lora = ERR_NONE;
//   } else {
//     errs.lora = ERR_LORA_INIT_FAIL;
//   }

//   giveSPI();
//   return ok;
// }

// static void sendLoRa(const String& payload) {
//   if (!ENABLE_LORA_TELEMETRY) return;
//   if (!takeSPI(50)) return;
//   LoRa.beginPacket();
//   LoRa.print(payload);
//   LoRa.endPacket(true);
//   giveSPI();
// }

// // ======================================================
// //               CORE0: GPS PARSER + RF
// // ======================================================
// static void safeCopy(char* dst, size_t dstLen, const char* src) {
//   if (!dst || dstLen == 0) return;
//   strncpy(dst, src ? src : "", dstLen - 1);
//   dst[dstLen - 1] = '\0';
// }

// // Capture last complete NMEA lines (GGA/RMC)
// static bool readLineFromGPS(char* lineBuf, size_t lineBufLen) {
//   // Reads bytes until '\n' or buffer full. Non-blocking: returns true if a full line assembled.
//   static char buf[128];
//   static size_t idx = 0;

//   while (GPSSerial.available() > 0) {
//     char c = (char)GPSSerial.read();
//     if (c == '\r') continue;

//     if (c == '\n') {
//       buf[idx] = '\0';
//       idx = 0;

//       // copy out
//       safeCopy(lineBuf, lineBufLen, buf);
//       return true;
//     }

//     if (idx < sizeof(buf) - 1) {
//       buf[idx++] = c;
//     } else {
//       // overflow: reset
//       idx = 0;
//     }
//   }
//   return false;
// }

// void taskGPSandRF(void* pv) {
//   (void)pv;

//   GPSState latest{};
//   char line[128];

//   uint32_t lastFrameMs = 0;
//   uint32_t lastLoRaMs  = 0;

//   for (;;) {
//     // Parse GPS bytes (TinyGPS++)
//     while (GPSSerial.available() > 0) {
//       gps.encode(GPSSerial.read());
//     }

//     // Also capture complete NMEA lines for "raw" logging
//     if (readLineFromGPS(line, sizeof(line))) {
//       // example: "$GPGGA,...."
//       if (strstr(line, "GGA")) safeCopy(latest.lastGGA, sizeof(latest.lastGGA), line);
//       if (strstr(line, "RMC")) safeCopy(latest.lastRMC, sizeof(latest.lastRMC), line);
//     }

//     // Update parsed fields
//     latest.hasData = gps.location.isValid() || gps.date.isValid() || gps.time.isValid();

//     if (gps.location.isValid()) {
//       latest.lat = gps.location.lat();
//       latest.lon = gps.location.lng();
//     }
//     if (gps.altitude.isValid()) {
//       latest.alt_m = gps.altitude.meters();
//     }
//     if (gps.satellites.isValid()) {
//       latest.sats = gps.satellites.value();
//     }
//     if (gps.hdop.isValid()) {
//       latest.hdop = gps.hdop.hdop();
//     }

//     latest.hasFix = gps.location.isValid() && (latest.sats > 0);

//     // Send GPSFrame to Core1 at GPS_FRAME_HZ
//     const uint32_t now = millis();
//     if (now - lastFrameMs >= (1000 / GPS_FRAME_HZ)) {
//       lastFrameMs = now;

//       GPSFrame f{};
//       f.rx_ms = now;
//       f.gps = latest;

//       // keep latest: if full, pop and push
//       if (xQueueSend(gpsQ, &f, 0) != pdTRUE) {
//         GPSFrame dump;
//         xQueueReceive(gpsQ, &dump, 0);
//         xQueueSend(gpsQ, &f, 0);
//       }
//     }

//     // Optional LoRa 1Hz telemetry (only GPS; SD is Core1)
//     if (ENABLE_LORA_TELEMETRY && (now - lastLoRaMs >= (1000 / LORA_TLM_HZ))) {
//       lastLoRaMs = now;

//       String payload;
//       payload.reserve(120);
//       payload += "GPS,";
//       payload += String(now);
//       payload += ",";
//       payload += String(latest.lat, 6);
//       payload += ",";
//       payload += String(latest.lon, 6);
//       payload += ",";
//       payload += String(latest.alt_m, 1);
//       payload += ",";
//       payload += String((unsigned long)latest.sats);
//       sendLoRa(payload);

//       // Debug heartbeat
//       Serial.printf("Core0 GPS frame: lat=%.6f lon=%.6f sats=%lu fix=%d\n",
//                     latest.lat, latest.lon, (unsigned long)latest.sats, (int)latest.hasFix);
//     }

//     vTaskDelay(pdMS_TO_TICKS(GPS_PARSE_YIELD_MS));
//   }
// }

// // ======================================================
// //           CORE1: SENSORS + ALL SD LOGGING
// // ======================================================
// void taskSensorsAndLogging(void* pv) {
//   (void)pv;

//   // Latest values
//   IMUState  imuLatest{};
//   BaroState baroLatest{};
//   GPSState  gpsLatest{};
//   ErrorState errs{};

//   // Baseline
//   float baseAlt = 0.0f;

//   // Init sensors (Core1)
//   bool bmpOK = initBMP280(baseAlt, errs);
//   bool bmiOK = initBMI160(errs);

//   // Master timing
//   uint32_t lastIMUms  = 0;
//   uint32_t lastBAROms = 0;
//   uint32_t lastLOGms  = 0;

//   // For SD flush frequency
//   uint32_t flushCounter = 0;

//   // GPS queue receive
//   GPSFrame gf{};

//   for (;;) {
//     const uint32_t now = millis();

//     // 1) Pull latest GPS frame from Core0 (non-blocking)
//     while (xQueueReceive(gpsQ, &gf, 0) == pdTRUE) {
//       gpsLatest = gf.gps;
//     }

//     // 2) Update IMU at IMU_HZ
//     if (now - lastIMUms >= (1000 / IMU_HZ)) {
//       lastIMUms = now;
//       if (bmiOK) {
//         readIMU(imuLatest, errs);
//       } else {
//         imuLatest.valid = false;
//         errs.bmi = ERR_BMI_INIT_FAIL;
//       }
//     }

//     // 3) Update Baro at BARO_HZ
//     if (now - lastBAROms >= (1000 / BARO_HZ)) {
//       lastBAROms = now;
//       if (bmpOK) {
//         readBaro(baroLatest, baseAlt, errs);
//       } else {
//         baroLatest.valid = false;
//         errs.bmp = ERR_BMP_INIT_FAIL;
//       }
//     }

//     // 4) Master log tick (writes ALL files with the SAME t_ms)
//     if (now - lastLOGms >= LOG_PERIOD_MS) {
//       lastLOGms = now;

//       // GPS error classification for this tick
//       if (!gpsLatest.hasData) errs.gps = ERR_GPS_NO_DATA;
//       else if (!gpsLatest.hasFix) errs.gps = ERR_GPS_NO_FIX;
//       else errs.gps = ERR_NONE;

//       // Write IMU row
//       if (imuFile) {
//         imuFile.printf("%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d\n",
//           (unsigned long)now,
//           imuLatest.ax_ms2, imuLatest.ay_ms2, imuLatest.az_ms2,
//           imuLatest.gx_rad, imuLatest.gy_rad, imuLatest.gz_rad,
//           imuLatest.amag_ms2,
//           (int)imuLatest.valid
//         );
//       }

//       // Write Baro row
//       if (baroFile) {
//         baroFile.printf("%lu,%.3f,%.3f,%.3f,%.3f,%d\n",
//           (unsigned long)now,
//           baroLatest.temp_C,
//           baroLatest.press_hPa,
//           baroLatest.absAlt_m,
//           baroLatest.relAlt_m,
//           (int)baroLatest.valid
//         );
//       }

//       // Write GPS row (raw + processed)
//       if (gpsFile) {
//         char dms[80];
//         formatLatLonDMS(gpsLatest.lat, gpsLatest.lon, dms, sizeof(dms));

//         // Quote raw strings to keep commas safe
//         gpsFile.printf("%lu,%.6f,%.6f,%.2f,%lu,%.2f,%d,\"%s\",\"%s\",\"%s\"\n",
//           (unsigned long)now,
//           gpsLatest.lat, gpsLatest.lon, gpsLatest.alt_m,
//           (unsigned long)gpsLatest.sats,
//           gpsLatest.hdop,
//           (int)gpsLatest.hasFix,
//           dms,
//           gpsLatest.lastGGA,
//           gpsLatest.lastRMC
//         );
//       }

//       // Write error summary row (per tick) => timestamps match
//       if (errFile) {
//         errFile.printf("%lu,%s,%s,%s,%s,%s\n",
//           (unsigned long)now,
//           errorCodeToStr(errs.sd),
//           errorCodeToStr(errs.lora),
//           errorCodeToStr(errs.bmp),
//           errorCodeToStr(errs.bmi),
//           errorCodeToStr(errs.gps)
//         );
//       }

//       // Flush occasionally (not every line; avoids SPI stalls)
//       if (++flushCounter % 10 == 0) { // flush every 1 second at 10Hz
//         if (imuFile)  imuFile.flush();
//         if (baroFile) baroFile.flush();
//         if (gpsFile)  gpsFile.flush();
//         if (errFile)  errFile.flush();
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
// static bool initSDandFiles(ErrorState &errs) {
//   // Start VSPI (shared with LoRa)
//   vspi.begin(ESP_LORA_SCK, ESP_LORA_MISO, ESP_LORA_MOSI);

//   if (!takeSPI(200)) {
//     errs.sd = ERR_SD_INIT_FAIL;
//     return false;
//   }

//   bool ok = SD.begin(ESP_SD_CS, vspi);
//   if (!ok) {
//     giveSPI();
//     errs.sd = ERR_SD_INIT_FAIL;
//     return false;
//   }

//   // Create next datalog directory
//   if (!createNextDatalogDir()) {
//     giveSPI();
//     errs.sd = ERR_SD_INIT_FAIL;
//     return false;
//   }

//   // Open CSVs
//   if (!openCSVFiles(errs)) {
//     giveSPI();
//     return false;
//   }

//   giveSPI();
//   errs.sd = ERR_NONE;
//   return true;
// }

// void setup() {
//   Serial.begin(SERIAL_BAUD);
//   delay(300);

//   spiMutex = xSemaphoreCreateMutex();

//   // I2C
//   Wire.begin(ESP_I2C_SDA, ESP_I2C_SCL, 400000);

//   // GPS UART2
//   GPSSerial.begin(ESP_GPS_BAUD, SERIAL_8N1, ESP_GPS_RX, ESP_GPS_TX);

//   // Queues
//   gpsQ = xQueueCreate(6, sizeof(GPSFrame));

//   // Init status trackers
//   ErrorState errs{};

//   // SD init + folder + files (Core1 will write; init here is fine, but we never touch SD from Core0)
//   sdOK = initSDandFiles(errs);

//   // LoRa init (Core0 uses LoRa at runtime; but init here is okay too)
//   bool loraOK = false;
//   if (ENABLE_LORA_TELEMETRY) {
//     loraOK = initLoRa(errs);
//   } else {
//     errs.lora = ERR_NONE;
//     loraOK = true;
//   }

//   Serial.println("=== INIT STATUS ===");
//   Serial.printf("SD:   %s | dir=%s\n", sdOK ? "OK" : "FAIL", datalogDir.c_str());
//   Serial.printf("LoRa: %s\n", loraOK ? "OK" : "FAIL");

//   // If SD failed, we still run GPS task for debugging, but logging will not work.
//   // You can choose to halt instead.

//   // Start tasks
//   xTaskCreatePinnedToCore(taskSensorsAndLogging, "Sensors+SD", 12288, nullptr, 2, nullptr, CORE_SENS_LOG);
//   xTaskCreatePinnedToCore(taskGPSandRF,         "GPS+RF",      8192,  nullptr, 2, nullptr, CORE_GPS_RF);
// }

// void loop() {
//   delay(1000);
// }