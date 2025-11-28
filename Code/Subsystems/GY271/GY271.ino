#include <Wire.h>

#define ADDR 0x2C // The address you found

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  delay(100);

  // 1. Initialize the QMC5883P
  // Register 0x0B: Set/Reset Period -> Value 0x01
  Wire.beginTransmission(ADDR);
  Wire.write(0x0B); 
  Wire.write(0x01); 
  Wire.endTransmission();

  // Register 0x0A: Control Register 1 
  // Value 0x1D -> Continuous Mode, 200Hz, 8G Range, OSR 512
  // (Binary: 0001 1101)
  Wire.beginTransmission(ADDR);
  Wire.write(0x0A); 
  Wire.write(0x1D); 
  Wire.endTransmission();

  Serial.println("QMC5883P (0x2C) Initialized.");
}

void loop() {
  // 2. Read Data
  // Start reading from Register 0x01 (Data X LSB)
  Wire.beginTransmission(ADDR);
  Wire.write(0x01); 
  Wire.endTransmission();

  // Request 6 bytes: X(2), Y(2), Z(2)
  Wire.requestFrom(ADDR, 6);

  if (Wire.available() >= 6) {
    // Read low byte first, then high byte
    int x = Wire.read() | (Wire.read() << 8);
    int y = Wire.read() | (Wire.read() << 8);
    int z = Wire.read() | (Wire.read() << 8);

    // 3. Convert to Micro Tesla (uT)
    // For Range 8G, sensitivity is usually 3000 LSB/Gauss
    // 1 Gauss = 100 uT
    float uT_X = (x / 3000.0) * 100.0;
    float uT_Y = (y / 3000.0) * 100.0;
    float uT_Z = (z / 3000.0) * 100.0;

    // 4. Calculate Heading
    float heading = atan2(uT_Y, uT_X) * 180.0 / PI;
    if (heading < 0) heading += 360;

    Serial.print("X: "); Serial.print(uT_X);
    Serial.print(" uT  Y: "); Serial.print(uT_Y);
    Serial.print(" uT  Z: "); Serial.print(uT_Z);
    Serial.print(" uT  Heading: "); Serial.println(heading);
  } else {
    Serial.println("Waiting for data...");
  }
  
  delay(200);
}