#include <Arduino.h>
#include <Wire.h>

#include "esp_air_pins.h"
#include "common_types.h"
#include "spi_bus.h"
#include "imu_bmi160.h"
#include "baro_bmp280.h"
#include "gps_mgr.h"
#include "lora_mgr.h"
#include "sd_logger.h"
#include "serial_lora_mirror.h"

// ======================================================
//                     CONFIG
// ======================================================
static constexpr uint32_t SERIAL_BAUD = 115200;

// Core assignment
static constexpr BaseType_t CORE_GPS_RF   = 0; // Core0
static constexpr BaseType_t CORE_SENS_LOG = 1; // Core1

// Master logging tick: ALL files written each tick => timestamps match
static constexpr uint32_t LOG_HZ = 10;
static constexpr uint32_t LOG_PERIOD_MS = 1000 / LOG_HZ;

// Sensor internal sampling (updates "latest" values)
static constexpr uint32_t IMU_HZ  = 50;
static constexpr uint32_t BARO_HZ = 25;

// GPS -> Core1 update rate
static constexpr uint32_t GPS_FRAME_HZ = 5;

// LoRa (MUST match Arduino R4)
static constexpr long LORA_FREQ_HZ = 433E6;     // <-- match R4
static constexpr int  LORA_TX_PWR  = 17;
static constexpr bool ENABLE_LORA_TELEMETRY = true;

// How often to send the GPS telemetry line over LoRa — 1 packet/s
static constexpr uint32_t LORA_TLM_HZ = 1;

// BMP280
static constexpr uint8_t BMP_ADDR = 0x76;
static constexpr float SEA_LEVEL_HPA = 1013.25f;

// BMI160
static constexpr uint8_t BMI_ADDR = 0x69;

// Core1 sensor/IMU debug line rate over LoRa.
// Set to 0 to disable entirely so only the GPS line is sent (1 packet/s total).
// Set to 1 to also send 1 CORE1_LOG line/s (giving 2 packets/s total).
static constexpr uint32_t CORE1_PRINT_HZ = 0; // <-- changed from 2 to 0

// ======================================================
//                   GLOBAL MODULES
// ======================================================
SPIBus spiBus;

HardwareSerial GPSSerial(2);
GPSMgr gpsMgr;
LoRaMgr loraMgr;

IMUBMI160 imuMod;
BaroBMP280 baroMod;

SDLogger sdlog;

// Core0->Core1 queue
QueueHandle_t gpsQ;

// Shared init state
static bool sdOK = false;
static bool loraOK = false;

// Serial Mirror (USB Serial + LoRa)
SerialLoRaMirror mirror;

// ======================================================
// Helper: "printf-style" but mirrored to LoRa too
// ======================================================
static inline void MLOG(const String& s) {
  mirror.logLine(s);  // prints to Serial + queues to LoRa
}
static inline void MLOGF(const char* fmt, ...) {
  char buf[220];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  mirror.logLine(String(buf));
}

// ======================================================
//             CORE0 TASK: GPS + LORA
// ======================================================
static void taskGPSandRF(void* pv) {
  (void)pv;

  const uint32_t tlmPeriodMs = (LORA_TLM_HZ == 0) ? 1000 : (1000 / LORA_TLM_HZ);
  uint32_t lastTlmMs = 0;

  MLOG("CORE0: GPS+RF task started");

  for (;;) {
    // 1) Non-blocking GPS processing
    gpsMgr.step();

    // 2) Push frames to Core1 at fixed rate
    gpsMgr.emitFrameIfDue(gpsQ, GPS_FRAME_HZ);

    // 3) GPS telemetry line at LORA_TLM_HZ (1/s)
    if (ENABLE_LORA_TELEMETRY && loraOK) {
      const uint32_t now = millis();
      if (now - lastTlmMs >= tlmPeriodMs) {
        lastTlmMs = now;

        const GPSState& g = gpsMgr.latest();

        // Format: GPS,T(ms),FIX,LAT,LON,ALT,HDOP,SATS
        String line;
        line.reserve(140);
        line += "GPS,";
        line += String(now);
        line += ",";
        line += String((int)g.hasFix);
        line += ",";
        line += String(g.lat, 6);
        line += ",";
        line += String(g.lon, 6);
        line += ",";
        line += String(g.alt_m, 2);
        line += ",";
        line += String(g.hdop, 2);
        line += ",";
        line += String(g.sats);

        MLOG(line);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ======================================================
//          CORE1 TASK: SENSORS + SD LOGGING
// ======================================================
static void taskSensorsAndLogging(void* pv) {
  (void)pv;

  IMUState  imuLatest{};
  BaroState baroLatest{};
  GPSState  gpsLatest{};
  ErrorState errs{};

  float baseAlt = 0.0f;

  MLOG("CORE1: Sensors+SD task starting...");

  // Init sensors (Core1)
  const bool bmpOK = baroMod.begin(BMP_ADDR, SEA_LEVEL_HPA, baseAlt, errs);
  const bool bmiOK = imuMod.begin(BMI_ADDR, errs);

  if (bmpOK) MLOGF("BMP280: OK baseAlt=%.2f m", baseAlt);
  else       MLOGF("BMP280: FAIL err=%s", errorCodeToStr(errs.bmp));

  if (bmiOK) MLOG("BMI160: OK");
  else       MLOGF("BMI160: FAIL err=%s", errorCodeToStr(errs.bmi));

  uint32_t lastIMUms  = 0;
  uint32_t lastBAROms = 0;
  uint32_t lastLOGms  = 0;

  // Rate limit for Core1 debug line over LoRa
  uint32_t lastCore1PrintMs = 0;
  const uint32_t core1PrintPeriodMs =
    (CORE1_PRINT_HZ == 0) ? 0 : (1000UL / CORE1_PRINT_HZ);

  GPSFrame gf{};

  for (;;) {
    const uint32_t now = millis();

    // Pull latest GPS frames (non-blocking)
    while (xQueueReceive(gpsQ, &gf, 0) == pdTRUE) {
      gpsLatest = gf.gps;
    }

    // Update IMU at IMU_HZ
    if (now - lastIMUms >= (1000 / IMU_HZ)) {
      lastIMUms = now;
      if (bmiOK) imuMod.read(imuLatest, errs);
      else { imuLatest.valid = false; errs.bmi = ERR_BMI_INIT_FAIL; }
    }

    // Update BARO at BARO_HZ
    if (now - lastBAROms >= (1000 / BARO_HZ)) {
      lastBAROms = now;
      if (bmpOK) baroMod.read(baroLatest, SEA_LEVEL_HPA, baseAlt, errs);
      else { baroLatest.valid = false; errs.bmp = ERR_BMP_INIT_FAIL; }
    }

    // Master log tick
    if (now - lastLOGms >= LOG_PERIOD_MS) {
      lastLOGms = now;

      // Classify GPS error for this tick
      if (!gpsLatest.hasData) errs.gps = ERR_GPS_NO_DATA;
      else if (!gpsLatest.hasFix) errs.gps = ERR_GPS_NO_FIX;
      else errs.gps = ERR_NONE;

      // Write ALL CSV rows with same timestamp
      if (sdOK) {
        sdlog.writeTick(now, imuLatest, baroLatest, gpsLatest, errs);
      }

      // CORE1_LOG line — only sent if CORE1_PRINT_HZ > 0
      // With CORE1_PRINT_HZ = 0 this block is never entered,
      // keeping LoRa output to exactly 1 packet/s (GPS line only).
      if (core1PrintPeriodMs > 0 &&
          (now - lastCore1PrintMs >= core1PrintPeriodMs)) {
        lastCore1PrintMs = now;

        MLOGF("CORE1_LOG,%lu,IMU=%d,BARO=%d,GPSfix=%d,ALTrel=%.2f,AccMag=%.2f,ERR(bmp=%s,bmi=%s,gps=%s)",
          (unsigned long)now,
          (int)imuLatest.valid,
          (int)baroLatest.valid,
          (int)gpsLatest.hasFix,
          baroLatest.relAlt_m,
          imuLatest.amag_ms2,
          errorCodeToStr(errs.bmp),
          errorCodeToStr(errs.bmi),
          errorCodeToStr(errs.gps)
        );
      }
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ======================================================
//                      SETUP
// ======================================================
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(300);

  // I2C
  Wire.begin(ESP_I2C_SDA, ESP_I2C_SCL, 400000);

  // SPI shared (SD + LoRa)
  spiBus.begin(ESP_LORA_SCK, ESP_LORA_MISO, ESP_LORA_MOSI);

  // GPS UART2
  gpsMgr.begin(GPSSerial, ESP_GPS_BAUD, ESP_GPS_RX, ESP_GPS_TX);

  // Queue
  gpsQ = xQueueCreate(6, sizeof(GPSFrame));

  ErrorState errs{};

  // SD init
  sdOK = sdlog.begin(spiBus, ESP_SD_CS, errs);
  sdlog.flushEvery(10); // flush every 1s at 10Hz

  // LoRa init
  if (ENABLE_LORA_TELEMETRY) {
    loraOK = loraMgr.begin(spiBus, ESP_LORA_CS, ESP_LORA_RST, ESP_LORA_DIO0,
                           LORA_FREQ_HZ, LORA_TX_PWR, errs);
  } else {
    errs.lora = ERR_NONE;
    loraOK = true;
  }

  // Start mirror AFTER LoRa init so it can send immediately
  mirror.begin(&loraMgr, &spiBus, &loraOK);
  mirror.startTask(CORE_GPS_RF); // LoRa TX worker on Core0

  // Everything below is mirrored (Serial + LoRa)
  MLOG("=== INIT STATUS ===");
  MLOGF("SD,%s,DIR=%s", sdOK ? "OK" : "FAIL", sdOK ? sdlog.dir().c_str() : "(none)");
  MLOGF("LORA,%s,FREQ_MHZ=%.0f", loraOK ? "OK" : "FAIL", (double)LORA_FREQ_HZ / 1e6);

  // Start tasks
  xTaskCreatePinnedToCore(taskSensorsAndLogging, "Sensors+SD", 12288, nullptr, 2, nullptr, CORE_SENS_LOG);
  xTaskCreatePinnedToCore(taskGPSandRF,          "GPS+RF",      8192,  nullptr, 2, nullptr, CORE_GPS_RF);
}

void loop() {
  delay(1000);
}
