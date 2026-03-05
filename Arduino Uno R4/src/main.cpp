#include <Arduino.h>
#include "r4_ground_lora_rx.h"

// Must match ESP32 LoRa frequency
static constexpr long LORA_FREQ_HZ = 433E6;   // SX1278 (RA-02) 433 MHz

static bool g_loraOk = false;
static uint32_t g_lastHbMs = 0;
static uint32_t g_lastInitTryMs = 0;

static uint32_t g_pktCount = 0;

// Optional: if you want to print RSSI too
static constexpr bool PRINT_RSSI = true;

// If ESP32 sends long logs, you may want to buffer and flush lines cleanly
static String g_lineBuf;

static void printBanner() {
  Serial.println();
  Serial.println("==============================================");
  Serial.println("Arduino R4 Ground Station - LoRa Serial Mirror");
  Serial.println("==============================================");
  Serial.println("This prints ANY text received over LoRa.");
  Serial.println("To truly mirror ESP32 Serial Monitor output,");
  Serial.println("ESP32 must transmit each log line over LoRa.");
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 2500) {}

  printBanner();

  Serial.print("LoRa init @ ");
  Serial.print((double)LORA_FREQ_HZ / 1e6, 0);
  Serial.println(" MHz ...");

  g_loraOk = r4LoRaRx_begin(LORA_FREQ_HZ);
  Serial.println(g_loraOk ? "LoRa init OK" : "LoRa init FAILED");

  g_lineBuf.reserve(512);
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
    g_loraOk = r4LoRaRx_begin(LORA_FREQ_HZ);
    Serial.println(g_loraOk ? "LoRa init OK" : "LoRa init FAILED");
  }
}

// If payload contains newlines, this will print each line cleanly
static void consumePayloadAndPrint(const String& payload, int rssi) {
  // Accumulate
  g_lineBuf += payload;

  // Some senders don't include newline — you can choose to print per-packet too:
  // Uncomment this if you want per-packet raw printing:
  /*
  Serial.print("RX#");
  Serial.print(++g_pktCount);
  if (PRINT_RSSI) {
    Serial.print(",RSSI=");
    Serial.print(rssi);
  }
  Serial.print(", ");
  Serial.println(payload);
  return;
  */

  // Line-based printing: flush complete lines ending with '\n'
  int nl;
  while ((nl = g_lineBuf.indexOf('\n')) >= 0) {
    String oneLine = g_lineBuf.substring(0, nl);
    g_lineBuf.remove(0, nl + 1);

    Serial.print("RX#");
    Serial.print(++g_pktCount);

    if (PRINT_RSSI) {
      Serial.print(",RSSI=");
      Serial.print(rssi);
    }

    Serial.print(", ");
    Serial.println(oneLine);
  }

  // If buffer grows too large (no newlines ever), print it anyway
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

void loop() {
  uint32_t now = millis();

  // Heartbeat every 1s
  if (now - g_lastHbMs >= 1000) {
    g_lastHbMs = now;
    heartbeat(now);
  }

  // Auto retry LoRa init if failed
  retryInitIfNeeded(now);

  // Receive packets
  String payload;
  int rssi = 0;

  // This function should return true when a packet is available
  if (g_loraOk && r4LoRaRx_receiveLine(payload, rssi)) {
    // If ESP32 sends plain text log lines, this will print them.
    // If ESP32 sends CSV telemetry, you'll see it as-is too.
    consumePayloadAndPrint(payload, rssi);
  }
}