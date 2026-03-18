#include "r4_ground_lora_rx.h"
#include "r4_ground_pins.h"

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

static bool R4LoRaOk = false;

// SX127x register addresses
static constexpr uint8_t REG_VERSION = 0x42;
static constexpr uint8_t EXPECTED_VERSION = 0x12; // typical for SX1278/76 family

static void loraResetPulse() {
  pinMode(R4_LORA_RST, OUTPUT);
  digitalWrite(R4_LORA_RST, LOW);
  delay(10);
  digitalWrite(R4_LORA_RST, HIGH);
  delay(10);
}

static uint8_t sx127x_readReg(uint8_t reg) {
  // MSB=0 for read
  digitalWrite(R4_LORA_CS, LOW);
  SPI.transfer(reg & 0x7F);
  uint8_t v = SPI.transfer(0x00);
  digitalWrite(R4_LORA_CS, HIGH);
  return v;
}

bool r4LoRaRx_begin(long frequency_hz) {
  R4LoRaOk = false;

  // Ensure SPI + CS are sane
  SPI.begin();
  pinMode(R4_LORA_CS, OUTPUT);
  digitalWrite(R4_LORA_CS, HIGH);

  // Reset radio
  loraResetPulse();

  // ---- SPI hardware check: RegVersion ----
  uint8_t ver = sx127x_readReg(REG_VERSION);
  Serial.print("LoRa SPI RegVersion(0x42)=0x");
  if (ver < 16) Serial.print("0");
  Serial.println(ver, HEX);

  if (ver != EXPECTED_VERSION) {
    Serial.println("WARN: SPI check mismatch (wiring/CS/SPI pins/level shifting/power).");
    Serial.println("      Expected ~0x12 for SX1278 family.");
  } else {
    Serial.println("LoRa SPI check OK (SX127x responding).");
  }

  // Library init with retries
  LoRa.setPins(R4_LORA_CS, R4_LORA_RST, R4_LORA_DIO0);

  for (int attempt = 1; attempt <= 5; attempt++) {
    Serial.print("LoRa.begin attempt ");
    Serial.print(attempt);
    Serial.print(" @ ");
    Serial.println((unsigned long)frequency_hz);

    if (LoRa.begin(frequency_hz)) {
      R4LoRaOk = true;
      break;
    }
    delay(250);
  }

  if (!R4LoRaOk) return false;

  // Make settings explicit (match TX!)
  LoRa.setSyncWord(0x12);
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.enableCrc();

  return true;
}

bool r4LoRaRx_receiveLine(String& outLine, int& outRssi) {
  if (!R4LoRaOk) return false;

  int packetSize = LoRa.parsePacket();
  if (!packetSize) return false;

  outLine = "";
  while (LoRa.available()) outLine += (char)LoRa.read();
  outRssi = LoRa.packetRssi();
  return true;
}