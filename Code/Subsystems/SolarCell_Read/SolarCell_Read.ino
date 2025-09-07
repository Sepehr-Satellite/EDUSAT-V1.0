/*
  Analog Input

  Demonstrates analog input by reading an analog sensor on analog pin 0 and
  turning on and off a light emitting diode(LED) connected to digital pin 13.
  The amount of time the LED will be on and off depends on the value obtained
  by analogRead().

  The circuit:
  - potentiometer
    center pin of the potentiometer to the analog input 0
    one side pin (either one) to ground
    the other side pin to +5V
  - LED
    anode (long leg) attached to digital output 13 through 220 ohm resistor
    cathode (short leg) attached to ground

  - Note: because most Arduinos have a built-in LED attached to pin 13 on the
    board, the LED is optional.

  created by David Cuartielles
  modified 30 Aug 2011
  By Tom Igoe

  This example code is in the public domain.

  https://docs.arduino.cc/built-in-examples/analog/AnalogInput/
*/

int sensorPin1 = A0;   // select the input pin for the potentiometer
int sensorPin2 = A1;
int sensorPin3 = A3;   // select the input pin for the potentiometer
int sensorPin4 = A4;
int sensorValue1 = 0;  // variable to store the value coming from the sensor
int sensorValue2 = 0; 
double upperCellVoltage = 0.0;
double lowerCellVoltage = 0.0;

void setup() {
  // declare the ledPin as an OUTPUT:
  Serial.begin(115200);
}

void loop() {
  
  // read the value from the sensor:
  sensorValue1 = analogRead(sensorPin1);
  sensorValue2 = analogRead(sensorPin2);
  upperCellVoltage = (sensorValue1-sensorValue2)/1024.0*5.0;
  lowerCellVoltage = (analogRead(sensorPin3)-analogRead(sensorPin4))/1024.0*5.0;
  Serial.print("upperCellVoltage: ");
  Serial.print(upperCellVoltage);
  Serial.print("\t");
  Serial.print("lowerCellVoltage: ");
  Serial.println(lowerCellVoltage);
  Serial.print("\n");
  delay(1000);
}
