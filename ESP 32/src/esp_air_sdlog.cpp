#include "esp_air_sdlog.h"
#include "esp_air_pins.h"
#include <SPI.h>
#include <SD.h>

static bool EspSdOk = false;

// Create /DATALOG001, /DATALOG002, ...
static String espSd_nextFolderName() {
  for (int i = 1; i <= 999; i++) {
    char buf[16];
    snprintf(buf, sizeof(buf), "/DATALOG%03d", i);
    if (!SD.exists(buf)) return String(buf);
  }
  return String("/DATALOG999");
}

bool espSd_beginAndCreateLog(EspSdFiles& out) {
  SPI.begin(ESP_LORA_SCK, ESP_LORA_MISO, ESP_LORA_MOSI);

  EspSdOk = SD.begin(ESP_SD_CS);
  if (!EspSdOk) return false;

  out.folder = espSd_nextFolderName();
  SD.mkdir(out.folder);

  out.imuPath = out.folder + "/imu_raw.txt";
  out.gpsPath = out.folder + "/gps.txt";
  out.bmpPath = out.folder + "/bmp_alt.txt";

  // headers (safe if file doesn't exist)
  if (!SD.exists(out.imuPath)) {
    espSd_appendLine(out.imuPath.c_str(), "t_mmssms,ax,ay,az,gx,gy,gz");
  }
  if (!SD.exists(out.gpsPath)) {
    espSd_appendLine(out.gpsPath.c_str(), "t_mmssms,ms,fix,lat,lng,alt_m,spd_kmph,hdop,sats,utc_hhmmss,utc_ddmmyy");
  }
  if (!SD.exists(out.bmpPath)) {
    espSd_appendLine(out.bmpPath.c_str(), "t_mmssms,temp_C,press_hPa,alt_m");
  }

  return true;
}

bool espSd_appendLine(const char* path, const char* line) {
  if (!EspSdOk) return false;
  File f = SD.open(path, FILE_APPEND);
  if (!f) return false;
  f.println(line);
  f.close();
  return true;
}