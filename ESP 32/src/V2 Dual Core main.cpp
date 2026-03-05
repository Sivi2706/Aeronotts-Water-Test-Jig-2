// #include <Arduino.h>
// #include <SPI.h>
// #include <SD.h>
// #include <Wire.h>

// #include <LoRa.h>
// #include <TinyGPSPlus.h>
// #include <Adafruit_BMP280.h>

// #include "BMI160.h"          // <-- your library
// #include "esp_air_pins.h"    // <-- your pin header

// // ======================= CONFIG =======================
// static constexpr uint32_t SERIAL_BAUD = 115200;

// // Task core assignment
// static constexpr BaseType_t CORE_GPS_RF   = 0;  // Core 0
// static constexpr BaseType_t CORE_SENS_LOG = 1;  // Core 1

// // Rates
// static constexpr uint32_t SENSOR_HZ            = 50;   // Core 1 sensor loop
// static constexpr uint32_t LOG_HZ               = 25;   // Core 1 SD logging
// static constexpr uint32_t TELEMETRY_HZ         = 1;    // Core 0 LoRa telemetry
// static constexpr uint32_t GPS_PARSE_YIELD_MS   = 2;    // Core 0 yield

// // Flight detection thresholds (tune)
// static constexpr float LAUNCH_ACCEL_MS2        = 15.0f; // ~1.5g
// static constexpr float LAUNCH_ALT_RISE_M       = 5.0f;
// static constexpr float APOGEE_DROP_M           = 2.0f;
// static constexpr uint32_t APOGEE_MIN_ASCENT_MS = 2000;

// // Files
// static const char* LIVE_CSV_PATH  = "/live.csv";
// static const char* EVENT_LOG_PATH = "/events.csv";

// // LoRa config (set to your region/module)
// static constexpr long LORA_FREQ_HZ = 915E6;  // change to 433E6 / 868E6 / 915E6
// static constexpr int  LORA_TX_PWR  = 17;

// // BMI160
// static constexpr uint8_t BMI_ADDR = 0x69; // as in your code

// // ======================================================

// // ---------- Shared SPI + Mutex ----------
// SPIClass vspi(VSPI);
// SemaphoreHandle_t spiMutex;

// // ---------- GPS ----------
// HardwareSerial GPSSerial(2);
// TinyGPSPlus gps;

// // ---------- Sensors ----------
// Adafruit_BMP280 bmp;
// bool bmpOK = false;
// float baseAlt_m = 0.0f;
// bool baselineReady = false;

// // BMI160 using your wrapper
// BMI160 imu(Wire);
// bool imuOK = false;

// // ---------- SD ----------
// bool sdOK = false;
// File liveFile;
// File eventFile;

// // ---------- Flight State ----------
// enum FlightState : uint8_t {
//   FS_IDLE = 0,
//   FS_ASCENT,
//   FS_APOGEE,
//   FS_DESCENT,
//   FS_LANDED
// };

// static const char* flightStateToStr(FlightState s) {
//   switch (s) {
//     case FS_IDLE:    return "IDLE";
//     case FS_ASCENT:  return "ASCENT";
//     case FS_APOGEE:  return "APOGEE";
//     case FS_DESCENT: return "DESCENT";
//     case FS_LANDED:  return "LANDED";
//     default:         return "UNKNOWN";
//   }
// }

// // ---------- Queues ----------
// struct TelemetrySample {
//   uint32_t t_ms;
//   float accelMag_ms2;
//   float relAlt_m;
//   FlightState state;
// };

// struct EventMsg {
//   uint32_t t_ms;
//   char text[48];
// };

// enum CommandType : uint8_t { CMD_NONE=0, CMD_STOP, CMD_RESUME, CMD_PING };

// struct CommandMsg {
//   uint32_t t_ms;
//   CommandType cmd;
// };

// // Core1 -> Core0
// QueueHandle_t telemetryQ;
// QueueHandle_t eventQ;

// // Core0 -> Core1
// QueueHandle_t commandQ;

// // ---------- Core1 control ----------
// volatile bool loggingEnabled = true;

// // ===================== SPI Mutex helpers =====================
// static bool takeSPI(uint32_t timeout_ms=20) {
//   if (!spiMutex) return true;
//   return xSemaphoreTake(spiMutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
// }
// static void giveSPI() {
//   if (!spiMutex) return;
//   xSemaphoreGive(spiMutex);
// }

// // ===================== BMI160 read (RAW accel + gyro) =====================
// static bool readIMU_raw(float &ax_ms2, float &ay_ms2, float &az_ms2,
//                         float &gx_rad, float &gy_rad, float &gz_rad,
//                         float &accMag_ms2)
// {
//   if (!imuOK) return false;

//   int16_t d[6]; // gx,gy,gz, ax,ay,az
//   if (imu.getAccelGyroData(d) != BMI160_OK) {
//     return false;
//   }

//   // Convert using your library helpers
//   ax_ms2 = imu.accelLSB_to_mps2(d[3]);
//   ay_ms2 = imu.accelLSB_to_mps2(d[4]);
//   az_ms2 = imu.accelLSB_to_mps2(d[5]);

//   // gyroLSB_to_dps gives deg/s -> convert to rad/s
//   const float DEG2RAD = 0.01745329252f;
//   gx_rad = imu.gyroLSB_to_dps(d[0]) * DEG2RAD;
//   gy_rad = imu.gyroLSB_to_dps(d[1]) * DEG2RAD;
//   gz_rad = imu.gyroLSB_to_dps(d[2]) * DEG2RAD;

//   accMag_ms2 = sqrtf(ax_ms2*ax_ms2 + ay_ms2*ay_ms2 + az_ms2*az_ms2);
//   return true;
// }

// // ===================== BMP280 relative altitude =====================
// static bool readRelAltitude(float &relAlt_m) {
//   if (!bmpOK || !baselineReady) return false;
//   float absAlt = bmp.readAltitude(1013.25f);
//   relAlt_m = absAlt - baseAlt_m;
//   return true;
// }

// // ===================== Core1: log event helper =====================
// static void logEventCore1(const char* txt) {
//   EventMsg e{};
//   e.t_ms = millis();
//   strncpy(e.text, txt, sizeof(e.text)-1);
//   xQueueSend(eventQ, &e, 0);

//   if (sdOK && loggingEnabled && eventFile) {
//     if (takeSPI(50)) {
//       eventFile.printf("%lu,%s\n", (unsigned long)e.t_ms, e.text);
//       eventFile.flush();
//       giveSPI();
//     }
//   }
// }

// // ===================== CORE 1 TASK: Sensors + Flight + SD =====================
// void taskSensorsAndLogging(void* pv) {
//   (void)pv;

//   const uint32_t sensorPeriodMs = 1000 / SENSOR_HZ;
//   const uint32_t logPeriodMs    = 1000 / LOG_HZ;
//   uint32_t lastSensorMs = 0;
//   uint32_t lastLogMs    = 0;

//   FlightState state = FS_IDLE;
//   float maxAlt_m = -1e9;
//   uint32_t ascentStartMs = 0;
//   bool deployed = false;

//   // Latest readings
//   float ax=0, ay=0, az=0;
//   float gx=0, gy=0, gz=0;
//   float accMag=0;
//   float relAlt=0;

//   CommandMsg cmd{};

//   for (;;) {
//     const uint32_t now = millis();

//     // ----- commands from Core0 -----
//     while (xQueueReceive(commandQ, &cmd, 0) == pdTRUE) {
//       if (cmd.cmd == CMD_STOP)   { loggingEnabled = false; logEventCore1("CMD_STOP"); }
//       if (cmd.cmd == CMD_RESUME) { loggingEnabled = true;  logEventCore1("CMD_RESUME"); }
//       if (cmd.cmd == CMD_PING)   { logEventCore1("CMD_PING"); }
//     }

//     // ----- sensor update -----
//     if (now - lastSensorMs >= sensorPeriodMs) {
//       lastSensorMs = now;

//       bool okIMU = readIMU_raw(ax, ay, az, gx, gy, gz, accMag);
//       bool okALT = readRelAltitude(relAlt);

//       if (!okIMU) accMag = 0.0f;
//       if (!okALT) relAlt = 0.0f;

//       // ----- determine flight state (flowchart) -----
//       switch (state) {
//         case FS_IDLE:
//           if (accMag >= LAUNCH_ACCEL_MS2 || relAlt >= LAUNCH_ALT_RISE_M) {
//             state = FS_ASCENT;
//             ascentStartMs = now;
//             maxAlt_m = relAlt;
//             logEventCore1("LAUNCH");
//           }
//           break;

//         case FS_ASCENT:
//           if (relAlt > maxAlt_m) maxAlt_m = relAlt;

//           if ((now - ascentStartMs) >= APOGEE_MIN_ASCENT_MS &&
//               relAlt <= (maxAlt_m - APOGEE_DROP_M)) {
//             state = FS_APOGEE;
//             logEventCore1("APOGEE");
//           }
//           break;

//         case FS_APOGEE:
//           if (!deployed) {
//             deployed = true;
//             logEventCore1("DEPLOY");
//             // TODO: trigger your deployment pin/servo here
//           }
//           state = FS_DESCENT;
//           break;

//         case FS_DESCENT:
//           if (fabsf(relAlt) < 2.0f) {
//             state = FS_LANDED;
//             logEventCore1("LANDED");
//           }
//           break;

//         case FS_LANDED:
//           break;
//       }

//       // ----- send sensor data to Core0 -----
//       TelemetrySample s{};
//       s.t_ms = now;
//       s.accelMag_ms2 = accMag;
//       s.relAlt_m = relAlt;
//       s.state = state;

//       if (xQueueSend(telemetryQ, &s, 0) != pdTRUE) {
//         TelemetrySample dump;
//         xQueueReceive(telemetryQ, &dump, 0);
//         xQueueSend(telemetryQ, &s, 0);
//       }
//     }

//     // ----- SD continuous CSV logging (flowchart) -----
//     if (sdOK && loggingEnabled && (now - lastLogMs >= logPeriodMs)) {
//       lastLogMs = now;

//       if (liveFile && takeSPI(50)) {
//         liveFile.printf("%lu,%.4f,%.3f,%s\n",
//           (unsigned long)now, accMag, relAlt, flightStateToStr(state));
//         liveFile.flush();
//         giveSPI();
//       }
//     }

//     vTaskDelay(pdMS_TO_TICKS(1));
//   }
// }

// // ===================== CORE 0 TASK: GPS + LoRa =====================
// static void sendLoRaPacket(const String& payload) {
//   if (!takeSPI(50)) return;
//   LoRa.beginPacket();
//   LoRa.print(payload);
//   LoRa.endPacket(true);
//   giveSPI();
// }

// static bool readLoRaLine(String &out) {
//   out = "";
//   if (!takeSPI(10)) return false;

//   int packetSize = LoRa.parsePacket();
//   if (packetSize <= 0) { giveSPI(); return false; }

//   while (LoRa.available()) out += (char)LoRa.read();
//   giveSPI();

//   out.trim();
//   return out.length() > 0;
// }

// static CommandType parseCommand(const String& line) {
//   String s = line;
//   s.toUpperCase();
//   if (s.indexOf("STOP") >= 0)   return CMD_STOP;
//   if (s.indexOf("RESUME") >= 0) return CMD_RESUME;
//   if (s.indexOf("PING") >= 0)   return CMD_PING;
//   return CMD_NONE;
// }

// void taskGPSandRF(void* pv) {
//   (void)pv;

//   TelemetrySample latest{};
//   latest.t_ms = 0;
//   latest.accelMag_ms2 = 0;
//   latest.relAlt_m = 0;
//   latest.state = FS_IDLE;

//   double lat = 0.0, lon = 0.0;
//   double gpsAlt = 0.0;
//   uint32_t sats = 0;

//   const uint32_t telemetryPeriodMs = 1000 / TELEMETRY_HZ;
//   uint32_t lastTelemetryMs = 0;

//   EventMsg ev{};
//   String rxLine;

//   for (;;) {
//     // --- GPS parse ---
//     while (GPSSerial.available() > 0) gps.encode(GPSSerial.read());

//     if (gps.location.isValid()) {
//       lat = gps.location.lat();
//       lon = gps.location.lng();
//     }
//     if (gps.altitude.isValid()) gpsAlt = gps.altitude.meters();
//     if (gps.satellites.isValid()) sats = gps.satellites.value();

//     // --- get latest telemetry from Core1 ---
//     TelemetrySample s{};
//     while (xQueueReceive(telemetryQ, &s, 0) == pdTRUE) latest = s;

//     // --- send event packets immediately ---
//     while (xQueueReceive(eventQ, &ev, 0) == pdTRUE) {
//       String payload = "EVENT,";
//       payload += String(ev.t_ms);
//       payload += ",";
//       payload += String(ev.text);
//       payload += ",";
//       payload += String(lat, 6);
//       payload += ",";
//       payload += String(lon, 6);
//       sendLoRaPacket(payload);
//     }

//     // --- check for ground command ---
//     if (readLoRaLine(rxLine)) {
//       CommandType c = parseCommand(rxLine);
//       if (c != CMD_NONE) {
//         CommandMsg cm{};
//         cm.t_ms = millis();
//         cm.cmd = c;
//         xQueueSend(commandQ, &cm, 0);

//         String ack = "ACK,";
//         ack += (c == CMD_STOP) ? "STOP" : (c == CMD_RESUME) ? "RESUME" : "PING";
//         sendLoRaPacket(ack);
//       }
//     }

//     // --- periodic telemetry (1 Hz) ---
//     const uint32_t now = millis();
//     if (now - lastTelemetryMs >= telemetryPeriodMs) {
//       lastTelemetryMs = now;

//       // TLM,time,lat,lon,gpsAlt,sats,accMag,relAlt,state
//       String payload;
//       payload.reserve(140);

//       payload += "TLM,";
//       payload += String(now);
//       payload += ",";
//       payload += String(lat, 6);
//       payload += ",";
//       payload += String(lon, 6);
//       payload += ",";
//       payload += String(gpsAlt, 1);
//       payload += ",";
//       payload += String((unsigned long)sats);
//       payload += ",";
//       payload += String(latest.accelMag_ms2, 4);
//       payload += ",";
//       payload += String(latest.relAlt_m, 2);
//       payload += ",";
//       payload += flightStateToStr(latest.state);

//       sendLoRaPacket(payload);
//     }

//     vTaskDelay(pdMS_TO_TICKS(GPS_PARSE_YIELD_MS));
//   }
// }

// // ===================== init helpers =====================
// static bool initSD() {
//   if (!takeSPI(200)) return false;

//   bool ok = SD.begin(ESP_SD_CS, vspi);
//   if (!ok) { giveSPI(); return false; }

//   liveFile  = SD.open(LIVE_CSV_PATH, FILE_APPEND);
//   eventFile = SD.open(EVENT_LOG_PATH, FILE_APPEND);

//   if (liveFile && liveFile.size() == 0) {
//     liveFile.println("Time_ms,AccelMag_ms2,RelAlt_m,State");
//     liveFile.flush();
//   }
//   if (eventFile && eventFile.size() == 0) {
//     eventFile.println("Time_ms,Event");
//     eventFile.flush();
//   }

//   giveSPI();
//   return (bool)liveFile && (bool)eventFile;
// }

// static bool initLoRa() {
//   if (!takeSPI(200)) return false;

//   LoRa.setPins(ESP_LORA_CS, ESP_LORA_RST, ESP_LORA_DIO0);
//   LoRa.setSPI(vspi);

//   bool ok = LoRa.begin(LORA_FREQ_HZ);
//   if (ok) LoRa.setTxPower(LORA_TX_PWR);

//   giveSPI();
//   return ok;
// }

// static bool initBMP280Baseline() {
//   if (!bmp.begin(0x76)) {
//     // change to 0x77 if your module uses that
//     return false;
//   }

//   bmp.setSampling(
//     Adafruit_BMP280::MODE_NORMAL,
//     Adafruit_BMP280::SAMPLING_X2,
//     Adafruit_BMP280::SAMPLING_X16,
//     Adafruit_BMP280::FILTER_X16,
//     Adafruit_BMP280::STANDBY_MS_63
//   );

//   // baseline average (2s)
//   const uint32_t start = millis();
//   float sum = 0.0f;
//   uint32_t n = 0;

//   while (millis() - start < 2000) {
//     sum += bmp.readAltitude(1013.25f);
//     n++;
//     delay(40);
//   }

//   baseAlt_m = (n > 0) ? (sum / (float)n) : 0.0f;
//   baselineReady = true;
//   return true;
// }

// static bool initBMI160() {
//   // You wanted: Wire.begin(SDA,SCL,400k) and I2cInit(0x69)
//   if (imu.I2cInit(BMI_ADDR) != BMI160_OK) {
//     return false;
//   }
//   return true;
// }

// // ===================== Arduino setup/loop =====================
// void setup() {
//   Serial.begin(SERIAL_BAUD);
//   delay(300);

//   spiMutex = xSemaphoreCreateMutex();

//   // VSPI (shared by LoRa + SD)
//   vspi.begin(ESP_LORA_SCK, ESP_LORA_MISO, ESP_LORA_MOSI);

//   // I2C (IMU + BMP)
//   Wire.begin(ESP_I2C_SDA, ESP_I2C_SCL, 400000);

//   // GPS UART2
//   GPSSerial.begin(ESP_GPS_BAUD, SERIAL_8N1, ESP_GPS_RX, ESP_GPS_TX);

//   // Init hardware
//   bmpOK = initBMP280Baseline();
//   imuOK = initBMI160();
//   sdOK  = initSD();
//   bool loraOK = initLoRa();

//   Serial.println("=== INIT STATUS ===");
//   Serial.printf("BMP280: %s\n", bmpOK ? "OK" : "FAIL");
//   Serial.printf("BMI160: %s\n", imuOK ? "OK" : "FAIL");
//   Serial.printf("SD:     %s\n", sdOK  ? "OK" : "FAIL");
//   Serial.printf("LoRa:   %s\n", loraOK ? "OK" : "FAIL");

//   // Create queues
//   telemetryQ = xQueueCreate(8, sizeof(TelemetrySample));
//   eventQ     = xQueueCreate(10, sizeof(EventMsg));
//   commandQ   = xQueueCreate(6, sizeof(CommandMsg));

//   // Start tasks (Flowchart mapping)
//   xTaskCreatePinnedToCore(taskSensorsAndLogging, "Sensors+SD", 8192, nullptr, 2, nullptr, CORE_SENS_LOG);
//   xTaskCreatePinnedToCore(taskGPSandRF,         "GPS+LoRa",    8192, nullptr, 2, nullptr, CORE_GPS_RF);

//   // Boot event
//   if (eventQ) {
//     EventMsg e{};
//     e.t_ms = millis();
//     strncpy(e.text, "BOOT", sizeof(e.text)-1);
//     xQueueSend(eventQ, &e, 0);
//   }
// }

// void loop() {
//   delay(1000);
// }