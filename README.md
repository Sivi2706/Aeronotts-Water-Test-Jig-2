# Aeronotts-Water-Test-Jig-2
# PinOuts 
## ESP 32 
### RF Transciever

| LoRa Module Pin | ESP32 GPIO | Purpose                  |     |
| --------------- | ---------- | ------------------------ | --- |
| **VCC**         | 3V3        | 3.3V Power               |     |
| **GND**         | GND        | Ground                   |     |
| **SCK**         | GPIO18     | SPI Clock                |     |
| **MISO**        | GPIO19     | SPI MISO                 |     |
| **MOSI**        | GPIO23     | SPI MOSI                 |     |
| **NSS / CS**    | GPIO5      | Chip Select              |     |
| **RESET**       | GPIO14     | LoRa Reset               |     |
| **DIO0**        | GPIO26     | Interrupt (Packet Ready) |     |
| 3.3V            | 3.3V       |                          |     |
### GPS module

| GPS Pin | ESP32 Pin                                 | Note                           |
| ------- | ----------------------------------------- | ------------------------------ |
| VCC     | 3V3 (or 5V if your GPS board supports it) | Check your GPS breakout        |
| GND     | GND                                       |                                |
| TX      | GPIO16 (ESP32 RX2)                        | GPS TX → ESP32 RX              |
| RX      | GPIO17 (ESP32 TX2)                        | ESP32 TX → GPS RX _(optional)_ |
| 3.3V    | 3.3V                                      |                                |

### BMI-160 Accelerometer + Gyroscope 
| BMI-160 pin | ESP32 Pin | Note |
| ----------- | --------- | ---- |
| SDA         | GPIO 21   | I²C  |
| SCL         | GPIO 22   | I²C  |
| 3.3V        | 3.3V      |      |

### BMP-280 Barometer + Temperature

| BMP-280 pin | ESP32 Pin | Note |
| ----------- | --------- | ---- |
| SDA         | GPIO 21   | I²C  |
| SCL         | GPIO 22   | I²C  |
| 5V          | 5V        |      |

### Micro SD card 
| SD pin | ESP32 Pin | Note                  |     |
| ------ | --------- | --------------------- | --- |
| CLK    | GPIO18    | Shared SPI bus        |     |
| MISO   | GPIO19    | Shared SPI bus        |     |
| MOSI   | GPIO23    | Shared SPI bus        |     |
| CS     | GPIO13    | Separate CS from LoRa |     |
| 3.3V   | 3.3V      |                       |     |

## Ground station 
### RF receiver 

|LoRa Module Pin|UNO R4 Pin|Purpose|
|---|---|---|
|**VCC**|3.3V|3.3V Power|
|**GND**|GND|Ground|
|**SCK**|D13|SPI Clock|
|**MISO**|D12|SPI MISO|
|**MOSI**|D11|SPI MOSI|
|**NSS / CS**|D10|Chip Select|
|**RESET**|D9|LoRa Reset|
|**DIO0**|D2|Interrupt (Packet Ready)|
