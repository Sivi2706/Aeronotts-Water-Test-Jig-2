#pragma once
#include <Arduino.h>

bool esp32_tx_init(long freq_hz, int csPin, int rstPin, int dio0Pin);
bool esp32_tx_sendText(const String& message);