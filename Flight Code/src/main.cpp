// ============================================================
//  V4_FLIGHT.cpp  —  ESP32 Flight Computer
//  
//  Architecture (matches flowchart):
//    CORE 1 — Sensors (IMU + Baro) + SD logging + Flight FSM
//    CORE 0 — GPS + LoRa RF telemetry + ground commands
//
//  New modules added over V3_RAW:
//    imu_mahony  — Mahony AHRS (quaternion + YPR)
//    flight_fsm  — Apogee detection state machine
//                  (ported from Hand-Test_Treshold.py)
//
//  Servo (parachute deploy) fires on APOGEE detection.
// ============================================================

#include <Arduino.h>
#include <Wire.h>

#include "esp_air_pins.h"
#include "common_types.h"
#include "spi_bus.h"
#include "imu_bmi160.h"
#include "imu_mahony.h"       // NEW: Mahony AHRS
#include "flight_fsm.h"       // NEW: apogee state machine
#include "baro_bmp280.h"
#include "gps_mgr.h"
#include "lora_mgr.h"
#include "sd_logger.h"
#include "serial_lora_mirror.h"

// ============================================================
//                        CONFIG
// ============================================================
static constexpr uint32_t SERIAL_BAUD = 115200;

// Core assignment
static constexpr BaseType_t CORE_GPS_RF   = 0;
static constexpr BaseType_t CORE_SENS_LOG = 1;

// Logging
static constexpr uint32_t LOG_HZ        = 10;
static constexpr uint32_t LOG_PERIOD_MS = 1000 / LOG_HZ;

// IMU / AHRS
static constexpr uint32_t IMU_HZ        = 100;   // Hz — matches Python SAMPLE_HZ
static constexpr uint32_t BARO_HZ       = 25;

// GPS -> Core1 update rate
static constexpr uint32_t GPS_FRAME_HZ  = 5;

// LoRa
static constexpr long LORA_FREQ_HZ      = 433E6;
static constexpr int  LORA_TX_PWR       = 17;
static constexpr bool ENABLE_LORA       = true;
// No periodic 1 Hz spam — TX only on events + slow GPS heartbeat
static constexpr uint32_t GPS_HB_MS     = 10000; // GPS heartbeat every 10s when idle
static constexpr uint32_t MIN_TX_GAP_MS = 500;   // guard: never TX faster than this

// ---- RF Event types ------------------------------------------
// Sent from Core1 -> Core0 via rfEventQ
enum RFEventType : uint8_t {
  RF_EVT_GPS_HB    = 0,   // periodic GPS heartbeat (idle)
  RF_EVT_LAUNCH    = 1,   // launch detected
  RF_EVT_APOGEE    = 2,   // apogee detected
  RF_EVT_LANDED    = 3,   // back to IDLE after apogee
};

struct RFEvent {
  RFEventType type;
  uint32_t    t_ms;
  GPSState    gps;
  BaroState   baro;
  float       accelMag;
  FlightState state;
  char        pkt[180];  // fixed-size — safe to memcpy through FreeRTOS queue
};

// Sensor I2C addresses
static constexpr uint8_t BMP_ADDR       = 0x76;
static constexpr uint8_t BMI_ADDR       = 0x68;
static constexpr float   SEA_LEVEL_HPA  = 1013.25f;

// ---- Servo (parachute deploy) --------------------------------
#define SERVO_PIN           27
#define LEDC_CH             0
#define LEDC_FREQ           50
#define LEDC_BITS           16
#define SERVO_MIN_US        500
#define SERVO_MAX_US        2500
#define SERVO_CLOSE_DEG     180   // stowed / closed position
#define SERVO_OPEN_DEG      90    // deployed / open position

// ---- Apogee failsafe -----------------------------------------
// If apogee is NOT detected within this many seconds of launch,
// force-deploy the servo anyway.
static constexpr float APOGEE_FAILSAFE_S = 10.0f;  // seconds after launch

// ============================================================
//                    GLOBAL MODULES
// ============================================================
SPIBus       spiBus;
HardwareSerial GPSSerial(2);
GPSMgr       gpsMgr;
LoRaMgr      loraMgr;
IMUBMI160    imuMod;
MahonyAHRS   ahrs;           // NEW
FlightFSM    fsm;            // NEW
BaroBMP280   baroMod;
SDLogger     sdlog;
SerialLoRaMirror mirror;

// Core0 <-> Core1 queues
QueueHandle_t gpsQ;
QueueHandle_t cmdQ;          // Core0 -> Core1 ground commands
QueueHandle_t rfEventQ;      // Core1 -> Core0 event-driven RF packets

// Shared init flags
static bool sdOK   = false;
static bool loraOK = false;

// Shared sensor snapshots (for RF packet, written by Core1, read by Core0)
portMUX_TYPE sharedMux = portMUX_INITIALIZER_UNLOCKED;
static IMUState   g_imuLatest{};
static BaroState  g_baroLatest{};
static GPSState   g_gpsLatest{};
static FlightState g_flightState = STATE_INIT;

// ============================================================
//                     SERVO HELPERS
// ============================================================
static uint32_t angleToDuty(int deg) {
  deg = constrain(deg, 0, 180);
  long us = map(deg, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
  return (uint32_t)(us * 65535UL / 20000UL);
}

static void servoInit() {
  ledcSetup(LEDC_CH, LEDC_FREQ, LEDC_BITS);
  ledcAttachPin(SERVO_PIN, LEDC_CH);
  ledcWrite(LEDC_CH, angleToDuty(SERVO_CLOSE_DEG));
}

static void servoSetAngle(int deg) {
  ledcWrite(LEDC_CH, angleToDuty(deg));
}

// ============================================================
//                   SERIAL LOG HELPERS
// ============================================================
static void SLOG(const String& s)         { Serial.println(s); }
static void SLOGF(const char* fmt, ...) {
  char buf[220];
  va_list ap; va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  Serial.println(buf);
}

// ============================================================
//               CORE 0 TASK: GPS + LORA RF
//  TX policy: send ONE packet per event from rfEventQ.
//  Packet format (single line, \n terminated):
//    EVT,<type>,<t_ms>,<fix>,<lat>,<lon>,<alt_m>,<relAlt_m>,<accelMag>,<state>
// ============================================================
static void taskGPSandRF(void* pv) {
  (void)pv;
  uint32_t lastTxMs = 0;
  SLOG("CORE0: GPS+RF task started");

  for (;;) {
    // 1. Non-blocking GPS
    gpsMgr.step();
    gpsMgr.emitFrameIfDue(gpsQ, GPS_FRAME_HZ);

    // 2. Send one RF event if queued
    if (ENABLE_LORA && loraOK) {
      RFEvent evt{};
      if (xQueueReceive(rfEventQ, &evt, 0) == pdTRUE) {
        const uint32_t now = millis();

        // Enforce minimum inter-packet gap to avoid LoRa collisions
        const uint32_t gap = now - lastTxMs;
        if (gap < MIN_TX_GAP_MS) {
          vTaskDelay(pdMS_TO_TICKS(MIN_TX_GAP_MS - gap));
        }

        // Map type to short label
        const char* typeLabel = "UNK";
        switch (evt.type) {
          case RF_EVT_GPS_HB: typeLabel = "GPS_HB";  break;
          case RF_EVT_LAUNCH: typeLabel = "LAUNCH";  break;
          case RF_EVT_APOGEE: typeLabel = "APOGEE";  break;
          case RF_EVT_LANDED: typeLabel = "LANDED";  break;
        }

        // Send the pre-built packet verbatim — same string already on SD
        loraMgr.send(spiBus, String(evt.pkt));
        Serial.print("[TX] ");
        Serial.print(evt.pkt);
        lastTxMs = millis();
      }
    }

    // 3. Ground commands
    uint8_t cmd = 0;
    if (xQueueReceive(cmdQ, &cmd, 0) == pdTRUE) {
      switch (cmd) {
        case 'S': SLOG("CMD: stop logging");   break;
        case 'R': SLOG("CMD: resume logging"); break;
        case 'Q': SLOG("CMD: status query");   break;
        default:  break;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ============================================================
//          CORE 1 TASK: SENSORS + AHRS + FSM + SD
// ============================================================
static void taskSensorsAndLogging(void* pv) {
  (void)pv;

  IMUState   imuLatest{};
  BaroState  baroLatest{};
  GPSState   gpsLatest{};
  ErrorState errs{};

  float baseAlt = 0.0f;
  bool bmpOK = false, bmiOK = false;

  SLOG("CORE1: Sensors+SD task starting...");

  // ---- Init sensors -------------------------------------------
  bmpOK = baroMod.begin(BMP_ADDR, SEA_LEVEL_HPA, baseAlt, errs);
  bmiOK = imuMod.begin(BMI_ADDR, errs);

  if (bmpOK) SLOGF("BMP280: OK  baseAlt=%.2f m", baseAlt);
  else       SLOGF("BMP280: FAIL  err=%s", errorCodeToStr(errs.bmp));

  if (bmiOK) SLOG("BMI160: OK");
  else       SLOGF("BMI160: FAIL  err=%s", errorCodeToStr(errs.bmi));

  // ---- Init Mahony + FSM --------------------------------------
  ahrs.begin((float)IMU_HZ);
  fsm.reset();

  SLOG("AHRS: Mahony filter ready");
  SLOG("FSM:  Collecting baseline (3 s)...");

  // ---- Servo starts closed ------------------------------------
  servoSetAngle(SERVO_CLOSE_DEG);
  bool deployed       = false;
  float inFlightSec   = 0.0f;   // elapsed seconds since launch (for failsafe)

  // ---- Timing -------------------------------------------------
  uint32_t lastIMUms   = 0;
  uint32_t lastBAROms  = 0;
  uint32_t lastLOGms   = 0;
  uint32_t lastPrintMs = 0;
  uint32_t lastGpsHbMs = 0;   // GPS heartbeat timer
  uint32_t lastUs      = micros();

  GPSFrame gf{};

  for (;;) {
    const uint32_t now    = millis();
    const uint32_t nowUs  = micros();
    const float    dt_s   = constrain((nowUs - lastUs) / 1e6f, 0.0001f, 0.05f);
    lastUs = nowUs;

    // ---- Pull GPS frames --------------------------------------
    while (xQueueReceive(gpsQ, &gf, 0) == pdTRUE) {
      gpsLatest = gf.gps;
    }

    // ---- IMU read + AHRS update --------------------------------
    if (now - lastIMUms >= (1000 / IMU_HZ)) {
      lastIMUms = now;

      if (bmiOK && imuMod.read(imuLatest, errs)) {
        // Update Mahony with physical-unit values from imu_bmi160
        ahrs.update(
          imuLatest.gx_rad, imuLatest.gy_rad, imuLatest.gz_rad,
          imuLatest.ax_ms2, imuLatest.ay_ms2, imuLatest.az_ms2,
          dt_s
        );

        // FSM uses raw |accel| from AHRS (already computed)
        bool stateChanged = fsm.update(ahrs.accelMag(), dt_s);

        // ---- Helper: write event to SD and push same string to RF queue ----
        auto fireEvent = [&](RFEventType rfType, const char* rfLabel, FlightState evtState) {
          RFEvent evt{};
          evt.type     = rfType;
          evt.t_ms     = now;
          evt.gps      = gpsLatest;
          evt.baro     = baroLatest;
          evt.accelMag = ahrs.accelMag();
          evt.state    = evtState;

          if (sdOK) {
            // writeEvent builds the packet, writes to SD, returns the string
            String pktStr = sdlog.writeEvent(now, evtState, rfLabel,
                                             gpsLatest, baroLatest, ahrs.accelMag());
            strncpy(evt.pkt, pktStr.c_str(), sizeof(evt.pkt) - 1);
            evt.pkt[sizeof(evt.pkt) - 1] = '\0';
          } else {
            // SD unavailable — build packet directly into the char array
            snprintf(evt.pkt, sizeof(evt.pkt),
              "EVT,%s,%lu,%d,%.6f,%.6f,%.2f,%.2f,%.4f,%s\n",
              rfLabel, (unsigned long)now, (int)gpsLatest.hasFix,
              gpsLatest.lat, gpsLatest.lon, gpsLatest.alt_m,
              baroLatest.relAlt_m, ahrs.accelMag(), flightStateName(evtState));
          }

          // Non-blocking push — drop oldest if full
          if (xQueueSend(rfEventQ, &evt, 0) != pdTRUE) {
            RFEvent dump{};
            xQueueReceive(rfEventQ, &dump, 0);
            xQueueSend(rfEventQ, &evt, 0);
          }
        };

        // ---- Fire events on state transitions -----------------
        if (stateChanged) {
          switch (fsm.state()) {
            case STATE_IN_FLIGHT:
              fireEvent(RF_EVT_LAUNCH, "LAUNCH", STATE_IN_FLIGHT);
              inFlightSec = 0.0f;
              SLOG(">>> LAUNCH detected <<<");
              break;
            case STATE_APOGEE:
              fireEvent(RF_EVT_APOGEE, "APOGEE", STATE_APOGEE);
              break;
            case STATE_IDLE:
              if (deployed) fireEvent(RF_EVT_LANDED, "LANDED", STATE_IDLE);
              inFlightSec = 0.0f;
              break;
            default: break;
          }
        }

        // ---- Accumulate in-flight time for failsafe -----------
        if (fsm.state() == STATE_IN_FLIGHT) {
          inFlightSec += dt_s;
        }

        // ---- APOGEE ACTION: deploy servo ----------------------
        // Triggered by FSM apogee detection OR failsafe timeout
        bool failsafeTriggered = (fsm.state() == STATE_IN_FLIGHT)
                                 && !deployed
                                 && (inFlightSec >= APOGEE_FAILSAFE_S);

        if (!deployed && (
              (stateChanged && fsm.state() == STATE_APOGEE) ||
              failsafeTriggered
            )) {
          deployed = true;
          servoSetAngle(SERVO_OPEN_DEG);

          if (failsafeTriggered) {
            SLOG(">>> FAILSAFE: apogee timeout — servo deployed! <<<");
            fireEvent(RF_EVT_APOGEE, "APOGEE_FAILSAFE", STATE_APOGEE);
          } else {
            SLOG(">>> APOGEE: servo deployed! <<<");
          }
        }

        // ---- Re-arm when back to IDLE -------------------------
        if (fsm.state() == STATE_IDLE && deployed) {
          deployed = false;
          servoSetAngle(SERVO_CLOSE_DEG);
          SLOG("FSM: IDLE — servo closed, re-armed");
        }
      } else if (!bmiOK) {
        imuLatest.valid = false;
        errs.bmi = ERR_BMI_INIT_FAIL;
      }
    }

    // ---- Baro read --------------------------------------------
    if (now - lastBAROms >= (1000 / BARO_HZ)) {
      lastBAROms = now;
      if (bmpOK) baroMod.read(baroLatest, SEA_LEVEL_HPA, baseAlt, errs);
      else { baroLatest.valid = false; errs.bmp = ERR_BMP_INIT_FAIL; }
    }

    // ---- Update shared snapshots for Core0 -------------------
    portENTER_CRITICAL(&sharedMux);
    g_imuLatest   = imuLatest;
    g_baroLatest  = baroLatest;
    g_gpsLatest   = gpsLatest;
    g_flightState = fsm.state();
    portEXIT_CRITICAL(&sharedMux);

    // ---- SD log tick -----------------------------------------
    if (now - lastLOGms >= LOG_PERIOD_MS) {
      lastLOGms = now;

      errs.gps = !gpsLatest.hasData ? ERR_GPS_NO_DATA
               : !gpsLatest.hasFix  ? ERR_GPS_NO_FIX
               : ERR_NONE;

      if (sdOK) sdlog.writeTick(now, imuLatest, baroLatest, gpsLatest, errs);
    }

    // ---- GPS heartbeat RF (slow, only when idle) -------------
    if (fsm.state() == STATE_IDLE && (now - lastGpsHbMs >= GPS_HB_MS)) {
      lastGpsHbMs = now;
      RFEvent evt{};
      evt.type     = RF_EVT_GPS_HB;
      evt.t_ms     = now;
      evt.gps      = gpsLatest;
      evt.baro     = baroLatest;
      evt.accelMag = ahrs.accelMag();
      evt.state    = fsm.state();
      snprintf(evt.pkt, sizeof(evt.pkt),
        "EVT,GPS_HB,%lu,%d,%.6f,%.6f,%.2f,%.2f,%.4f,IDLE\n",
        (unsigned long)now, (int)gpsLatest.hasFix,
        gpsLatest.lat, gpsLatest.lon, gpsLatest.alt_m,
        baroLatest.relAlt_m, ahrs.accelMag());
      xQueueSend(rfEventQ, &evt, 0);
    }

    // ---- Serial diagnostic @ 2 Hz ----------------------------
    if (now - lastPrintMs >= 500) {
      lastPrintMs = now;

      float qw, qx, qy, qz;
      ahrs.getQuaternion(qw, qx, qy, qz);

      SLOGF(
        "DMPSTYLE,%.4f,%.4f,%.4f,%.4f,%.1f,%.1f,%.1f,%.4f,%.4f,%.4f,%.4f",
        qw, qx, qy, qz,
        ahrs.yawDeg(), ahrs.pitchDeg(), ahrs.rollDeg(),
        imuLatest.ax_ms2, imuLatest.ay_ms2, imuLatest.az_ms2,
        ahrs.accelMag()
      );

      SLOGF(
        "FSM  state=%-10s  baseline=%.2f  delta=%.3f  progress=%.0f%%",
        flightStateName(fsm.state()),
        fsm.baseline(),
        fsm.deltaMag(),
        fsm.initProgress() * 100.0f
      );

      SLOGF(
        "BARO relAlt=%.2fm  GPS fix=%d sats=%lu  SD=%s  LoRa=%s",
        baroLatest.relAlt_m,
        (int)gpsLatest.hasFix,
        (unsigned long)gpsLatest.sats,
        sdOK   ? "OK" : "FAIL",
        loraOK ? "OK" : "FAIL"
      );
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ============================================================
//                        SETUP
// ============================================================
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(300);

  SLOG("=== V4 FLIGHT COMPUTER ===");

  // Servo
  servoInit();

  // I2C
  Wire.begin(ESP_I2C_SDA, ESP_I2C_SCL, 400000);

  // SPI (SD + LoRa share bus)
  spiBus.begin(ESP_LORA_SCK, ESP_LORA_MISO, ESP_LORA_MOSI);

  // GPS UART2
  gpsMgr.begin(GPSSerial, ESP_GPS_BAUD, ESP_GPS_RX, ESP_GPS_TX);

  // Queues
  gpsQ     = xQueueCreate(6,  sizeof(GPSFrame));
  cmdQ     = xQueueCreate(4,  sizeof(uint8_t));
  rfEventQ = xQueueCreate(8,  sizeof(RFEvent));   // event-driven RF TX

  ErrorState errs{};

  // SD
  sdOK = sdlog.begin(spiBus, ESP_SD_CS, errs);
  sdlog.flushEvery(10);

  // LoRa
  if (ENABLE_LORA) {
    loraOK = loraMgr.begin(spiBus, ESP_LORA_CS, ESP_LORA_RST, ESP_LORA_DIO0,
                            LORA_FREQ_HZ, LORA_TX_PWR, errs);
  } else {
    loraOK = true;
    errs.lora = ERR_NONE;
  }

  // LoRa mirror task (optional Serial->LoRa bridge)
  mirror.begin(&loraMgr, &spiBus, &loraOK);
  mirror.startTask(CORE_GPS_RF);

  SLOG("=== INIT STATUS ===");
  SLOGF("SD    : %s  dir=%s", sdOK   ? "OK" : "FAIL", sdOK ? sdlog.dir().c_str() : "(none)");
  SLOGF("LoRa  : %s  %.0f MHz", loraOK ? "OK" : "FAIL", (double)LORA_FREQ_HZ / 1e6);
  SLOGF("Servo : pin %d  close=%d deg  open=%d deg  failsafe=%.0fs",
        SERVO_PIN, SERVO_CLOSE_DEG, SERVO_OPEN_DEG, APOGEE_FAILSAFE_S);
  SLOG("===================");

  // Launch tasks
  xTaskCreatePinnedToCore(taskSensorsAndLogging, "Sensors+SD", 12288,
                          nullptr, 2, nullptr, CORE_SENS_LOG);

  xTaskCreatePinnedToCore(taskGPSandRF, "GPS+RF", 8192,
                          nullptr, 2, nullptr, CORE_GPS_RF);
}

void loop() {
  delay(1000);   // all work is in the two tasks
}