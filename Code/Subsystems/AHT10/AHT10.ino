#include <Wire.h>
#include <SoftWire.h>
#include <AsyncDelay.h>

#define AHT10_ADDRESS 0x38
#define AHT10_INIT_CMD 0xE1
#define AHT10_MEASURE_CMD 0xAC

int sdaPin = 40;
int sclPin = 41;
SoftWire sw(sdaPin, sclPin);
char swTxBuffer[16];
char swRxBuffer[16];

AsyncDelay readInterval;

void setup() {
#if F_CPU >= 12000000UL
  Serial.begin(115200);
#else
  Serial.begin(9600);
#endif

  // Init Wire
  Wire.begin();
  Wire.beginTransmission(AHT10_ADDRESS);
  Wire.write(AHT10_INIT_CMD);
  Wire.endTransmission();

  // Init SoftWire
  sw.setTxBuffer(swTxBuffer, sizeof(swTxBuffer));
  sw.setRxBuffer(swRxBuffer, sizeof(swRxBuffer));
  sw.setDelay_us(5);
  sw.setTimeout(1000);
  sw.begin();
  sw.beginTransmission(AHT10_ADDRESS);
  sw.write(AHT10_INIT_CMD);
  sw.endTransmission();

  delay(20);
  readInterval.start(2000, AsyncDelay::MILLIS);
}

void readAHT10_Wire() {
  Wire.beginTransmission(AHT10_ADDRESS);
  Wire.write(AHT10_MEASURE_CMD);
  Wire.write(0x33);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(100);

  uint8_t data[6];
  int numBytes = Wire.requestFrom(AHT10_ADDRESS, (uint8_t)6);
  for (int i = 0; i < numBytes; i++) data[i] = Wire.read();

  if (numBytes == 6) {
    unsigned long humidity_raw = ((unsigned long)data[1] << 12) | ((unsigned long)data[2] << 4) | (data[3] >> 4);
    unsigned long temp_raw = (((unsigned long)data[3] & 0x0F) << 16) | ((unsigned long)data[4] << 8) | data[5];
    float humidity = humidity_raw * (100.0 / 1048576.0);
    float temperature = (temp_raw * (200.0 / 1048576.0)) - 50;

    Serial.print("[Wire] Humidity: "); Serial.print(humidity); Serial.print("%, Temperature: "); Serial.print(temperature); Serial.println("C");
  } else {
    Serial.print("[Wire] Read wrong number of bytes: "); Serial.println(numBytes);
  }
}

void readAHT10_SoftWire() {
  sw.beginTransmission(AHT10_ADDRESS);
  sw.write(AHT10_MEASURE_CMD);
  sw.write(0x33);
  sw.write(0x00);
  sw.endTransmission();
  delay(100);

  uint8_t data[6];
  int numBytes = sw.requestFrom(AHT10_ADDRESS, (uint8_t)6);
  for (int i = 0; i < numBytes; i++) data[i] = sw.read();

  if (numBytes == 6) {
    unsigned long humidity_raw = ((unsigned long)data[1] << 12) | ((unsigned long)data[2] << 4) | (data[3] >> 4);
    unsigned long temp_raw = (((unsigned long)data[3] & 0x0F) << 16) | ((unsigned long)data[4] << 8) | data[5];
    float humidity = humidity_raw * (100.0 / 1048576.0);
    float temperature = (temp_raw * (200.0 / 1048576.0)) - 50;

    Serial.print("[SoftWire] Humidity: "); Serial.print(humidity); Serial.print("%, Temperature: "); Serial.print(temperature); Serial.println("C");
  } else {
    Serial.print("[SoftWire] Read wrong number of bytes: "); Serial.println(numBytes);
  }
}

void loop() {
  if (readInterval.isExpired()) {
    Serial.println("-----------");
    readAHT10_Wire();
    readAHT10_SoftWire();
    readInterval.restart();
  }
}
