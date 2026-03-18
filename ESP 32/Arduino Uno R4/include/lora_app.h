#pragma once
#include <Arduino.h>

bool lora_init(long frequency_hz, int cs_pin, int reset_pin, int dio0_pin);
bool lora_receive_line(String& out_line, int& out_rssi);