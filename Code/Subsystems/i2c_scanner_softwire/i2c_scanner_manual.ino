#include <SoftWire.h>

int sdaPin = 40;
int sclPin = 41;

SoftWire sw(sdaPin, sclPin);
char swTxBuffer[16];
char swRxBuffer[16];

void setup() {
  Serial.begin(115200);

  Serial.println("SoftWire I2C Scanner");
  Serial.print("SDA pin: ");
  Serial.println(int(sdaPin));
  Serial.print("SCL pin: ");
  Serial.println(int(sclPin));

  sw.setTxBuffer(swTxBuffer, sizeof(swTxBuffer));
  sw.setRxBuffer(swRxBuffer, sizeof(swRxBuffer));
  sw.setDelay_us(5);     // Adjust delay if needed
  sw.setTimeout(1000);   // Optional: timeout for bus operations
  sw.begin();
}

void loop() {
  Serial.println("Scanning for I2C devices...");

  for (uint8_t address = 1; address < 127; ++address) {
    sw.beginTransmission(address);
    uint8_t error = sw.endTransmission();

    if (error == 0) {
      Serial.print("Found device at 0x");
      if (address < 16)
        Serial.print('0');
      Serial.println(address, HEX);
    }
  }

  Serial.println("Scan complete.\n");
  delay(3000); // Wait before next scan
}
