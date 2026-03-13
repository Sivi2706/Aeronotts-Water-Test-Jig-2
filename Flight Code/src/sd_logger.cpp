#include "sd_logger.h"
#include <math.h>
#include <string.h>

static const char* DATALOG_PREFIX = "/Datalog_";
static const char* IMU_CSV_NAME   = "imu_raw.csv";
static const char* BARO_CSV_NAME  = "baro_raw.csv";
static const char* GPS_CSV_NAME   = "gps_raw.csv";
static const char* ERR_CSV_NAME   = "error_log.csv";
static const char* EVT_CSV_NAME   = "events.csv";    // NEW

String SDLogger::makePath(const String& dir, const char* name) {
  return dir + "/" + String(name);
}

bool SDLogger::createNextDatalogDir() {
  for (int i = 1; i <= 999; i++) {
    char buf[24];
    snprintf(buf, sizeof(buf), "%s%03d", DATALOG_PREFIX, i);
    String candidate(buf);

    if (!SD.exists(candidate)) {
      if (SD.mkdir(candidate)) {
        _datalogDir = candidate;
        return true;
      }
      return false;
    }
  }
  return false;
}

static void writeHeaderBlock(File& f, const char* columns) {
  // Single clean header row — Excel reads this correctly as column names.
  // Previously had # metadata rows but # is treated as a formula error
  // in Excel CSV mode, causing all columns to appear blank/corrupt.
  f.println(columns);
  f.flush();
}

bool SDLogger::openCSVFiles(ErrorState& errs) {
  _imuFile  = SD.open(makePath(_datalogDir, IMU_CSV_NAME),  FILE_APPEND);
  _baroFile = SD.open(makePath(_datalogDir, BARO_CSV_NAME), FILE_APPEND);
  _gpsFile  = SD.open(makePath(_datalogDir, GPS_CSV_NAME),  FILE_APPEND);
  _errFile  = SD.open(makePath(_datalogDir, ERR_CSV_NAME),  FILE_APPEND);
  _evtFile  = SD.open(makePath(_datalogDir, EVT_CSV_NAME),  FILE_APPEND);

  if (!_imuFile || !_baroFile || !_gpsFile || !_errFile || !_evtFile) {
    errs.sd = ERR_SD_FILE_OPEN_FAIL;
    return false;
  }

  // ========================= IMU CSV =========================
  if (_imuFile.size() == 0) {
    writeHeaderBlock(_imuFile,
      "t_ms,ax_ms2,ay_ms2,az_ms2,gx_rads,gy_rads,gz_rads,accMag_ms2,valid");
  }

  // ========================= BARO CSV =========================
  if (_baroFile.size() == 0) {
    writeHeaderBlock(_baroFile,
      "t_ms,temp_C,press_hPa,absAlt_m,relAlt_m,valid");
  }

  // ========================= GPS CSV =========================
  if (_gpsFile.size() == 0) {
    writeHeaderBlock(_gpsFile,
      "t_ms,lat,lon,alt_m,sats,hdop,hasFix,coord_dms,lastGGA,lastRMC");
  }

  // ========================= ERROR CSV =========================
  if (_errFile.size() == 0) {
    writeHeaderBlock(_errFile,
      "t_ms,sd,lora,bmp,bmi,gps");
  }

  // ========================= EVENTS CSV ========================
  // lora_packet column is quoted so commas inside don't split columns in Excel
  if (_evtFile.size() == 0) {
    writeHeaderBlock(_evtFile,
      "t_ms,event,rf_label,gps_fix,lat,lon,gps_alt_m,baro_relAlt_m,accelMag_ms2,lora_packet");
  }

  return true;
}

void SDLogger::formatDMS(double deg, bool isLat, char* out, size_t outLen) {
  char hemi = isLat ? ((deg >= 0) ? 'N' : 'S') : ((deg >= 0) ? 'E' : 'W');

  double a = fabs(deg);
  int d = (int)a;
  double minFloat = (a - d) * 60.0;
  int m = (int)minFloat;
  double s = (minFloat - m) * 60.0;

  snprintf(out, outLen, "%d%c%02d'%04.1f\"%c", d, 0xB0, m, s, hemi);
}

void SDLogger::formatLatLonDMS(double lat, double lon, char* out, size_t outLen) {
  char latDMS[32], lonDMS[32];
  formatDMS(lat, true, latDMS, sizeof(latDMS));
  formatDMS(lon, false, lonDMS, sizeof(lonDMS));
  snprintf(out, outLen, "%s %s", latDMS, lonDMS);
}

bool SDLogger::begin(SPIBus& bus, uint8_t sdCs, ErrorState& errs) {
  if (!bus.take(200)) {
    errs.sd = ERR_SD_INIT_FAIL;
    return false;
  }

  if (!SD.begin(sdCs, bus.spi())) {
    bus.give();
    errs.sd = ERR_SD_INIT_FAIL;
    return false;
  }

  if (!createNextDatalogDir()) {
    bus.give();
    errs.sd = ERR_SD_INIT_FAIL;
    return false;
  }

  if (!openCSVFiles(errs)) {
    bus.give();
    return false;
  }

  bus.give();
  errs.sd = ERR_NONE;
  return true;
}

void SDLogger::flushEvery(uint32_t nTicks) {
  _flushEvery = (nTicks == 0) ? 1 : nTicks;
}

void SDLogger::writeTick(uint32_t t_ms,
                         const IMUState& imu,
                         const BaroState& baro,
                         const GPSState& gps,
                         const ErrorState& errs) {
  // IMU
  if (_imuFile) {
    _imuFile.printf("%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d\n",
      (unsigned long)t_ms,
      imu.ax_ms2, imu.ay_ms2, imu.az_ms2,
      imu.gx_rad, imu.gy_rad, imu.gz_rad,
      imu.amag_ms2,
      (int)imu.valid
    );
  }

  // Baro
  if (_baroFile) {
    _baroFile.printf("%lu,%.3f,%.3f,%.3f,%.3f,%d\n",
      (unsigned long)t_ms,
      baro.temp_C,
      baro.press_hPa,
      baro.absAlt_m,
      baro.relAlt_m,
      (int)baro.valid
    );
  }

  // GPS
  if (_gpsFile) {
    char dms[80];
    formatLatLonDMS(gps.lat, gps.lon, dms, sizeof(dms));

    _gpsFile.printf("%lu,%.6f,%.6f,%.2f,%lu,%.2f,%d,\"%s\",\"%s\",\"%s\"\n",
      (unsigned long)t_ms,
      gps.lat, gps.lon, gps.alt_m,
      (unsigned long)gps.sats,
      gps.hdop,
      (int)gps.hasFix,
      dms,
      gps.lastGGA,
      gps.lastRMC
    );
  }

  // Error summary
  if (_errFile) {
    _errFile.printf("%lu,%s,%s,%s,%s,%s\n",
      (unsigned long)t_ms,
      errorCodeToStr(errs.sd),
      errorCodeToStr(errs.lora),
      errorCodeToStr(errs.bmp),
      errorCodeToStr(errs.bmi),
      errorCodeToStr(errs.gps)
    );
  }

  // Flush every N ticks
  _tickCount++;
  if (_tickCount % _flushEvery == 0) {
    if (_imuFile)  _imuFile.flush();
    if (_baroFile) _baroFile.flush();
    if (_gpsFile)  _gpsFile.flush();
    if (_errFile)  _errFile.flush();
    if (_evtFile)  _evtFile.flush();
  }
}

String SDLogger::writeEvent(uint32_t t_ms, FlightState event, const char* rfLabel,
                             const GPSState& gps, const BaroState& baro,
                             float accelMag_ms2) {
  // Build the LoRa RF packet — \n terminated as the receiver splits on newlines.
  // Format: EVT,<rf_label>,<t_ms>,<fix>,<lat>,<lon>,<gps_alt_m>,<baro_relAlt_m>,<accelMag>,<fsm_state>
  char pkt[180];
  snprintf(pkt, sizeof(pkt),
    "EVT,%s,%lu,%d,%.6f,%.6f,%.2f,%.2f,%.4f,%s\n",
    rfLabel,
    (unsigned long)t_ms,
    (int)gps.hasFix,
    gps.lat, gps.lon, gps.alt_m,
    baro.relAlt_m,
    accelMag_ms2,
    flightStateName(event)
  );

  // Write to SD events.csv.
  // lora_packet column is quoted so the commas inside don't split into extra
  // columns in Excel. The \n is stripped from pkt before quoting — the newline
  // is only needed for the RF protocol, not inside a CSV field.
  if (_evtFile) {
    // Make a copy without the trailing \n for the CSV field
    char pktNoNl[180];
    strncpy(pktNoNl, pkt, sizeof(pktNoNl) - 1);
    pktNoNl[sizeof(pktNoNl) - 1] = '\0';
    size_t len = strlen(pktNoNl);
    if (len > 0 && pktNoNl[len - 1] == '\n') pktNoNl[len - 1] = '\0';

    _evtFile.printf(
      "%lu,%s,%s,%d,%.6f,%.6f,%.2f,%.2f,%.4f,\"%s\"\n",
      (unsigned long)t_ms,
      flightStateName(event),
      rfLabel,
      (int)gps.hasFix,
      gps.lat, gps.lon, gps.alt_m,
      baro.relAlt_m,
      accelMag_ms2,
      pktNoNl   // quoted, no \n inside — Excel sees one clean column
    );
    _evtFile.flush();  // immediate flush — events are rare and critical
  }

  return String(pkt);  // RF string keeps the \n
}