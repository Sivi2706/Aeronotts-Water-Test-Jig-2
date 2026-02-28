#include "lora_app.h"
#include <SPI.h>
#include <LoRa.h>

static bool esp32_ready = false;

bool esp32_tx_init(long freq_hz, int csPin, int rstPin, int dio0Pin)
{
    SPI.begin(18, 19, 23, csPin);  // ESP32 VSPI pins
    LoRa.setSPI(SPI);
    LoRa.setPins(csPin, rstPin, dio0Pin);

    if (!LoRa.begin(freq_hz)) {
        esp32_ready = false;
        return false;
    }

    esp32_ready = true;
    return true;
}

bool esp32_tx_sendText(const String& message)
{
    if (!esp32_ready) return false;

    LoRa.beginPacket();
    LoRa.print(message);
    return (LoRa.endPacket() == 1);
}