#pragma once
#include <Arduino.h>

bool espLoRaTx_begin(long frequency_hz);
bool espLoRaTx_sendLine(const char* line);