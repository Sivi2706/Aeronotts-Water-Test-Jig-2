#pragma once
#include <Arduino.h>

bool r4LoRaRx_begin(long frequency_hz);
bool r4LoRaRx_receiveLine(String& outLine, int& outRssi);