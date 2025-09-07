#include <Wire.h>
#include <SoftWire.h>
#include <AsyncDelay.h>
#include "AHT10.h"

// --- SoftWire configuration (custom I2C pins) ---
int sdaPin = 40;
int sclPin = 41;
SoftWire sw(sdaPin, sclPin);
char swTxBuffer[16];
char swRxBuffer[16];

// --- Create two AHT10 sensor objects ---
// One using hardware I2C (Wire), one using software I2C (SoftWire)
AHT10 aht10_hw(0x38, BUS_WIRE, &Wire, nullptr);
AHT10 aht10_sw(0x38, BUS_SOFTWIRE, nullptr, &sw);

AsyncDelay readInterval; // For periodic sensor readings

void setup() {
    Serial.begin(115200);

    // Setup SoftWire buffers and timing BEFORE calling begin
    sw.setTxBuffer(swTxBuffer, sizeof(swTxBuffer));
    sw.setRxBuffer(swRxBuffer, sizeof(swRxBuffer));
    sw.setDelay_us(5);     // Microsecond delay between I2C changes
    sw.setTimeout(1000);   // Timeout in milliseconds

    // Initialize both sensors
    aht10_hw.begin();
    aht10_sw.begin();

    // Start periodic readings every 2 seconds
    readInterval.start(2000, AsyncDelay::MILLIS);

    Serial.println("AHT10 Dual I2C Example: Hardware Wire and SoftWire");
}

void loop() {
    if (readInterval.isExpired()) {
        float hum, temp;

        // --- Read using hardware I2C (Wire) ---
        if (aht10_hw.read(hum, temp)) {
            Serial.print("[Front] Humidity: ");
            Serial.print(hum, 1);
            Serial.print("%, Temperature: ");
            Serial.print(temp, 1);
            Serial.println(" C");
        } else {
            Serial.println("[Front] Read error!");
        }

        // --- Read using software I2C (SoftWire) ---
        if (aht10_sw.read(hum, temp)) {
            Serial.print("[Back] Humidity: ");
            Serial.print(hum, 1);
            Serial.print("%, Temperature: ");
            Serial.print(temp, 1);
            Serial.println(" C");
            Serial.println("\n");
        } else {
            Serial.println("[Back] Read error!");
        }

        // Restart the interval timer
        readInterval.restart();
    }
}
