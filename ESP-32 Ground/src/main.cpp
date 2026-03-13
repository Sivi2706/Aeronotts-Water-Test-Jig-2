#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

// ======================================================
// ESP32 Ground Station LoRa Receiver
// For SX1278 / RA-02 at 433 MHz
// ======================================================

// ---------------- LoRa frequency ----------------
static constexpr long LORA_FREQ_HZ = 433E6;   // Must match transmitter

// ---------------- ESP32 pin mapping ----------------
// VSPI default pins for common ESP32 DevKit boards
static constexpr int LORA_SCK   = 18;
static constexpr int LORA_MISO  = 19;
static constexpr int LORA_MOSI  = 23;
static constexpr int LORA_CS    = 5;
static constexpr int LORA_RST   = 14;
static constexpr int LORA_DIO0  = 2;

// ---------------- State variables ----------------
static bool g_loraOk = false;
static uint32_t g_lastHbMs = 0;
static uint32_t g_lastInitTryMs = 0;
static uint32_t g_pktCount = 0;

// Optional RSSI printing
static constexpr bool PRINT_RSSI = true;

// Buffer to reconstruct line-based logs
static String g_lineBuf;

// ======================================================
// Helper functions
// ======================================================

static void printBanner() {
  Serial.println();
  Serial.println("==============================================");
  Serial.println("ESP32 Ground Station - LoRa Serial Mirror");
  Serial.println("==============================================");
  Serial.println("This prints ANY text received over LoRa.");
  Serial.println("To truly mirror ESP32 Serial Monitor output,");
  Serial.println("the flight ESP32 must transmit each log line over LoRa.");
  Serial.println();
}

static bool loraBegin(long freqHz) {
  // Start SPI with explicit pins
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);

  // Assign LoRa control pins
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

  // Begin radio
  if (!LoRa.begin(freqHz)) {
    return false;
  }

  // Optional tuning settings
  LoRa.setSpreadingFactor(7);     // typical: 7 to 12
  LoRa.setSignalBandwidth(125E3); // 125 kHz
  LoRa.setCodingRate4(5);         // 4/5
  LoRa.enableCrc();

  return true;
}

static void heartbeat(uint32_t now) {
  Serial.print("HB,");
  Serial.print(now);
  Serial.print(",LORA_OK=");
  Serial.print(g_loraOk ? 1 : 0);
  Serial.print(",PKTS=");
  Serial.println(g_pktCount);
}

static void retryInitIfNeeded(uint32_t now) {
  if (!g_loraOk && (now - g_lastInitTryMs >= 2000)) {
    g_lastInitTryMs = now;
    Serial.println("Retrying LoRa init...");
    g_loraOk = loraBegin(LORA_FREQ_HZ);
    Serial.println(g_loraOk ? "LoRa init OK" : "LoRa init FAILED");
  }
}

static void consumePayloadAndPrint(const String& payload, int rssi) {
  // Accumulate payload into line buffer
  g_lineBuf += payload;

  // Flush complete lines when newline is found
  int nl;
  while ((nl = g_lineBuf.indexOf('\n')) >= 0) {
    String oneLine = g_lineBuf.substring(0, nl);
    g_lineBuf.remove(0, nl + 1);

    // Remove trailing carriage return if present
    if (oneLine.endsWith("\r")) {
      oneLine.remove(oneLine.length() - 1);
    }

    Serial.print("RX#");
    Serial.print(++g_pktCount);

    if (PRINT_RSSI) {
      Serial.print(",RSSI=");
      Serial.print(rssi);
    }

    Serial.print(", ");
    Serial.println(oneLine);
  }

  // If sender never sends newline and buffer grows too large, flush anyway
  if (g_lineBuf.length() > 400) {
    Serial.print("RX#");
    Serial.print(++g_pktCount);

    if (PRINT_RSSI) {
      Serial.print(",RSSI=");
      Serial.print(rssi);
    }

    Serial.print(", ");
    Serial.println(g_lineBuf);

    g_lineBuf = "";
  }
}

static bool receiveLoRaPacket(String& payload, int& rssi) {
  int packetSize = LoRa.parsePacket();
  if (packetSize <= 0) {
    return false;
  }

  payload = "";
  while (LoRa.available()) {
    payload += (char)LoRa.read();
  }

  rssi = LoRa.packetRssi();
  return true;
}

// ======================================================
// Arduino setup / loop
// ======================================================

void setup() {
  Serial.begin(115200);
  delay(500);

  printBanner();

  Serial.print("LoRa init @ ");
  Serial.print((double)LORA_FREQ_HZ / 1e6, 0);
  Serial.println(" MHz ...");

  g_loraOk = loraBegin(LORA_FREQ_HZ);
  Serial.println(g_loraOk ? "LoRa init OK" : "LoRa init FAILED");

  g_lineBuf.reserve(512);
}

void loop() {
  uint32_t now = millis();

  // Heartbeat every 1 second
  if (now - g_lastHbMs >= 1000) {
    g_lastHbMs = now;
    heartbeat(now);
  }

  // Retry init if radio failed
  retryInitIfNeeded(now);

  // Receive packets
  String payload;
  int rssi = 0;

  if (g_loraOk && receiveLoRaPacket(payload, rssi)) {
    consumePayloadAndPrint(payload, rssi);
  }
}