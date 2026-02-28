#pragma once
#include <Arduino.h>

struct EspSdFiles {
  String folder;
  String imuPath;
  String gpsPath;
  String bmpPath;
};

bool espSd_beginAndCreateLog(EspSdFiles& out);
bool espSd_appendLine(const char* path, const char* line);