
/*
  GY-271 Compass
  Using QMC5883L Compass Library
*/

// I2C Library
#include <Wire.h>
// QMC5883L Compass Library
#include <QMC5883LCompass.h>

QMC5883LCompass compass;

void setup() {
  // Initialize the serial port.
  Serial.begin(115200);
  // Initialize I2C.
  Wire.begin();
  // Initialize the Compass.
  compass.init();
}

void loop() {
  int x, y, z;

  // Read compass values
  compass.read();

  x = compass.getX();
  y = compass.getY();
  z = compass.getZ();

  Serial.print("X: ");
  Serial.print(x);
  Serial.print("   Y: ");
  Serial.print(y);
  Serial.print("   Z: ");
  Serial.println(z);

  delay(100);
}
