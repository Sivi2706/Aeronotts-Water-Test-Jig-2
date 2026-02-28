#pragma once
#include <Arduino.h>

bool esp32_gpsrf_init(
  long lora_freq_hz,
  int lora_cs, int lora_rst, int lora_dio0,
  int gps_rx_pin, int gps_tx_pin,
  uint32_t gps_baud
);

void esp32_gpsrf_poll();

// Always sends a packet:
// - If FIX: sends FIX CSV
// - If NO FIX: sends NOFIX status CSV
bool esp32_gpsrf_send_packet_csv();