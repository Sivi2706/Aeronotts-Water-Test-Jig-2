#pragma once
#include <Arduino.h>

// ========== LoRa (ESP32 VSPI) ==========
static constexpr uint8_t ESP_LORA_SCK  = 18;
static constexpr uint8_t ESP_LORA_MISO = 19;
static constexpr uint8_t ESP_LORA_MOSI = 23;
static constexpr uint8_t ESP_LORA_CS   = 5;
static constexpr uint8_t ESP_LORA_RST  = 14;
static constexpr uint8_t ESP_LORA_DIO0 = 26;

// ========== SD card (shared SPI, separate CS) ==========
static constexpr uint8_t ESP_SD_CS     = 13;

// ========== I2C ==========
static constexpr uint8_t ESP_I2C_SDA   = 21;
static constexpr uint8_t ESP_I2C_SCL   = 22;

// ========== GPS (UART2) ==========
static constexpr uint8_t ESP_GPS_RX    = 16; // GPS TX -> ESP RX
static constexpr uint8_t ESP_GPS_TX    = 17; // ESP TX -> GPS RX
static constexpr uint32_t ESP_GPS_BAUD = 9600;