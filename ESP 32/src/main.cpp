#include <Arduino.h>
#include "lora_app.h"

#define LORA_CS     5
#define LORA_RST    14
#define LORA_DIO0   26
#define LORA_FREQ   433E6

void setup()
{
    Serial.begin(115200);
    while (!Serial);

    Serial.println("ESP32 Transmitter Ready");

    if (!esp32_tx_init(LORA_FREQ, LORA_CS, LORA_RST, LORA_DIO0))
    {
        Serial.println("LoRa INIT FAILED");
        while (1);
    }

    Serial.println("LoRa INIT SUCCESS");
}

void loop()
{
    if (Serial.available())
    {
        String msg = Serial.readStringUntil('\n');
        msg.trim();

        if (msg.length() > 0)
        {
            bool status = esp32_tx_sendText(msg);

            Serial.print("Sent: ");
            Serial.print(msg);
            Serial.print(" [");
            Serial.print(status ? "OK" : "FAIL");
            Serial.println("]");
        }
    }
}