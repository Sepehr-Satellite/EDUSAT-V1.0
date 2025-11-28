#include <Arduino.h>
#include <Wire.h>                 // Standard I2C communication library
#include <SoftWire.h>             // Software I2C for custom pins
#include "AHT10.h"                // AHT10 temperature/humidity sensor (hardware/software I2C)
//#include <QMC5883LCompass.h>    // GY-271 Magnetometer (QMC5883L) - Replaced by direct I2C driver
#include <Adafruit_GFX.h>         // Core graphics library for displays
#include <Adafruit_ST7789.h>      // Hardware-specific library for ST7789 (Display)
#include <Adafruit_MPU6050.h>     // MPU6050 6DOF IMU
#include <Adafruit_Sensor.h>      // Sensor event structure and base class
#include <math.h>                 // Math functions (atan2, etc.) for heading calculation

// ---------------- QMC5883P (GY-271) I2C Address Definition ----------------
// GY-271 Magnetometer (QMC5883P variant) connected via hardware I2C (Wire)
#define QMC_ADDR 0x2C             // I2C address of the detected QMC5883P device

// ---------------- SoftWire configuration (custom I2C pins) ----------------
// Allows using any available pins for I2C, required for multi-sensor applications
// Usage: SoftWire sdaPin, sclPin;
int sdaPin = 40;  // Back AHT10 SDA (can be any GPIO)
int sclPin = 41;  // Back AHT10 SCL (can be any GPIO)
SoftWire sw(sdaPin, sclPin);
char swTxBuffer[16];
char swRxBuffer[16];

// ---------------- ST7789 TFT module connections ----------------
// Pin mapping for Adafruit_ST7789 constructor
#define TFT_DC    8    // Data/Command pin
#define TFT_RST   9    // Reset pin
#define TFT_CS   53    // Chip Select pin
//#define TFT_SCLK 52  //TFT SCK pin is connected to Mega2560 pin52
//#define TFT_MOSI 51  //TFT SDA pin is connected to Mega2560 pin51
// tft.init(WIDTH, HEIGHT, SPI_MODE)
// WIDTH/HEIGHT options: typically 240x240 or 320x240
// SPI_MODE options: SPI_MODE0, SPI_MODE1, SPI_MODE2, SPI_MODE3

// ---------------- Sensor Object Initialization ----------------
// Front AHT10 (Hardware I2C): BUS_WIRE for onboard I2C bus (pins 20/21 on Mega2560)
// Back AHT10 (Software I2C): BUS_SOFTWIRE for custom pin I2C via SoftWire
AHT10 aht10_hw(0x38, BUS_WIRE, &Wire, nullptr);       // Address usually 0x38, 0x39, or 0x40
AHT10 aht10_sw(0x38, BUS_SOFTWIRE, nullptr, &sw);

//QMC5883LCompass gy271;    // GY-271 Magnetometer (QMC5883L) - Removed, replaced with direct I2C access

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST); // ST7789 Display
//Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST); //Option2: Define manual SPI MOSI/SCLK for 

Adafruit_MPU6050 imu;     // MPU6050 6DOF IMU

// ---------------- LED Blinking Variables ----------------
const int led = 13;       // Mega2560 onboard LED (D13)
unsigned long previousMillis = 0;
const long interval = 500; // milliseconds
int ledState = LOW;
bool blinking = false;     // LED blinking status

// ---------------- Motor (A) Pin Connections ----------------
// Use L298N/L293D style pinout for direction/speed control
int enA = 2;   // Enable/speed control
int in1 = 3;   // Direction 1
int in2 = 4;   // Direction 2

// ---------------- Solar Cell Analog Pin Definitions ----------------
// Each cell has two pins for differential measurement (V+ - V-)
int ruCell1 = A0;   // Right Upper Solar Cell Pin-1 (V+)
int ruCell2 = A1;   // Right Upper Solar Cell Pin-2 (V-)
int rlCell1 = A3;   // Right Lower Solar Cell Pin-1 (V+)
int rlCell2 = A4;   // Right Lower Solar Cell Pin-2 (V-)
int luCell1 = A8;   // Left Upper Solar Cell Pin-1 (V+)
int luCell2 = A9;   // Left Upper Solar Cell Pin-2 (V-)
int llCell1 = A10;  // Left Lower Solar Cell Pin-1 (V+)
int llCell2 = A11;  // Left Lower Solar Cell Pin-2 (V-)

// ---------------- Solar Cell Voltage Variables ----------------
// Will be computed as (analogRead(pin1) - analogRead(pin2))/1024.0*5.0 for each cell
double ruCellVoltage = 0.0;     // Right Upper Solar Cell Voltage
double rlCellVoltage = 0.0;     // Right Lower Solar Cell Voltage
double luCellVoltage = 0.0;     // Left Upper Solar Cell Voltage
double llCellVoltage = 0.0;     // Left Lower Solar Cell Voltage

// ---------------- Telecommand List (as string constants) ----------------
// Available commands for remote control and data query
const String LED_ON = "LED_ON";                                     // LED Blinking (On) command 
const String LED_OFF = "LED_OFF";                                   // LED Blinking (Off) command 
const String CAMERA_CAPTURE = "CAMERA_CAPTURE";                     // Capturing the image command
const String FRONT_TEMP = "FRONT_TEMP";                             // Front AHT10 temperature sensor, temperature (Celsius) report command
const String FRONT_HUM = "FRONT_HUM";                               // Front AHT10 temperature sensor, humidity (percent) report command
const String BACK_TEMP = "BACK_TEMP";                               // Back AHT10 temperature sensor, temperature (Celsius) report command
const String BACK_HUM = "BACK_HUM";                                 // Back AHT10 temperature sensor, humidity (percent) report command
const String MAGNETOMETER = "MAGNETOMETER";                         // GY-271 Magnetometer, Magnetic field report command
const String IMU = "IMU";                                           // MPU6050 6-DOF IMU, accelometer, gyro & temperature report command
const String RIGHT_UPPER_CELL_VOLTAGE = "RIGHT_UPPER_CELL_VOLTAGE"; // Right upper solar cell voltage report command
const String RIGHT_LOWER_CELL_VOLTAGE = "RIGHT_LOWER_CELL_VOLTAGE"; // Right lower solar cell voltage report command
const String LEFT_UPPER_CELL_VOLTAGE = "LEFT_UPPER_CELL_VOLTAGE";   // Left upper solar cell voltage report command
const String LEFT_LOWER_CELL_VOLTAGE = "LEFT_LOWER_CELL_VOLTAGE";   // Left lower solar cell voltage report command
const String MOTOR_MAX = "MOTOR_MAX";                               // DC motor max rpm command
const String MOTOR_OFF = "MOTOR_OFF";                               // DC motor turn off command

void setup() {
  // ---------------- Serial Communications Setup ----------------
  Serial.begin(9600);      // For user/PC debug
  Serial2.begin(115200);   // For Payload (ESP32-CAM)
  Serial3.begin(115200);   // For TT&C (ESP8266)

  // ---------------- Pin Initializations ----------------
  pinMode(led, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  digitalWrite(in1, LOW); // Ensure motor off at start
  digitalWrite(in2, LOW);

  // ---------------- Hardware I2C and Software I2C Setup ----------------
  Wire.begin();           // For hardware I2C devices (Front AHT10, GY-271, MPU6050, etc.)

  // Set up SoftWire buffers and timing BEFORE calling begin()
  sw.setTxBuffer(swTxBuffer, sizeof(swTxBuffer));
  sw.setRxBuffer(swRxBuffer, sizeof(swRxBuffer));
  sw.setDelay_us(5);      // microsecond delay (typically 1-10us; increases robustness)
  sw.setTimeout(1000);    // milliseconds

  aht10_hw.begin();       // Initialize hardware I2C AHT10 (front)
  aht10_sw.begin();       // Initialize software I2C AHT10 (back)

  //gy271.init();         // Initialize GY-271 Magnetometer (library-based) - Removed
  initQMC5883P();         // Initialize GY-271 Magnetometer (QMC5883P) via direct I2C registers

  // ---------------- Display Initialization ----------------
  // tft.init(WIDTH, HEIGHT, SPI_MODE)
  // WIDTH: 240 or 320
  // HEIGHT: 240 or 320
  // SPI_MODE options: SPI_MODE0, SPI_MODE1, SPI_MODE2, SPI_MODE3 (ST7789 usually requires SPI_MODE3)
  tft.init(240, 240, SPI_MODE3);
  tft.setRotation(1);     // 0-3 for landscape/portrait
  tft.fillScreen(ST77XX_WHITE);
  tft.setTextSize(3);
  testdrawtext((char*)"Sepehr EDUSAT    v1001", ST77XX_BLUE);

  // ---------------- MPU6050 Initialization and Configuration ----------------
  imu.begin();

  // Set accelerometer range for MPU6050:
  // Options: MPU6050_RANGE_2_G, MPU6050_RANGE_4_G, MPU6050_RANGE_8_G, MPU6050_RANGE_16_G
  imu.setAccelerometerRange(MPU6050_RANGE_8_G); // Chosen: ±8G

  // Set gyroscope range for MPU6050:
  // Options: MPU6050_RANGE_250_DEG, MPU6050_RANGE_500_DEG, MPU6050_RANGE_1000_DEG, MPU6050_RANGE_2000_DEG
  imu.setGyroRange(MPU6050_RANGE_1000_DEG); // Chosen: ±1000 deg/sec

  // Set digital low pass filter bandwidth for MPU6050:
  // Options: MPU6050_BAND_260_HZ, MPU6050_BAND_184_HZ, MPU6050_BAND_94_HZ, MPU6050_BAND_44_HZ, MPU6050_BAND_21_HZ, MPU6050_BAND_10_HZ, MPU6050_BAND_5_HZ
  imu.setFilterBandwidth(MPU6050_BAND_5_HZ); // Chosen: 5 Hz bandwidth for noise filtering
}

void loop() {
  // ---------------- Command Handling from TT&C (Serial3) ----------------
  if (Serial3.available()) {
    String cmd = Serial3.readStringUntil('\n');
    cmd.trim();

    Serial.println(cmd); // Echo for debug

    // -------- LED Blinking Command Handling --------
    if (cmd == LED_ON) {
      blinking = true;
      Serial3.println("LED Blinking: [On]");
    }
    else if (cmd == LED_OFF) {
      blinking = false;
      Serial3.println("LED Blinking: [Off]");
    }

    // -------- ESP32-CAM Picture Capture Command --------
    else if (cmd == CAMERA_CAPTURE) {
      Serial2.println("capture"); // Relay command to ESP32-CAM
      if (Serial2.available()) {
        String cam_resp = Serial2.readStringUntil('\n');
        cam_resp.trim();
        Serial.println(cam_resp); // For debug
      }
    }

    // -------- AHT10 (Front) - Temperature Query --------
    else if (cmd == FRONT_TEMP) {
      float hum, temp;
      if (aht10_hw.read(hum, temp)) {
        Serial3.print("[Front] Temperature: ");
        Serial3.print(temp, 1);
        Serial3.println(" C");
      } else {
        Serial3.println("[Front] Read error!");
      }
    }

    // -------- AHT10 (Front) - Humidity Query --------
    else if (cmd == FRONT_HUM) {
      float hum, temp;
      if (aht10_hw.read(hum, temp)) {
        Serial3.print("[Front] Humidity: ");
        Serial3.print(hum, 1);
        Serial3.println(" %");
      } else {
        Serial3.println("[Front] Read error!");
      }
    }

    // -------- AHT10 (Back) - Temperature Query --------
    else if (cmd == BACK_TEMP) {
      float hum, temp;
      if (aht10_sw.read(hum, temp)) {
        Serial3.print("[Back] Temperature: ");
        Serial3.print(temp, 1);
        Serial3.println(" C");
      } else {
        Serial3.println("[Back] Read error!");
      }
    }

    // -------- AHT10 (Back) - Humidity Query --------
    else if (cmd == BACK_HUM) {
      float hum, temp;
      if (aht10_sw.read(hum, temp)) {
        Serial3.print("[Back] Humidity: ");
        Serial3.print(hum, 1);
        Serial3.println(" %");
      } else {
        Serial3.println("[Back] Read error!");
      }
    }

    // -------- Magnetometer (GY-271 / QMC5883P) Field Query (I2C direct) --------
    else if (cmd == MAGNETOMETER) {
      float uT_X, uT_Y, uT_Z, headingDeg;
      if (readQMC5883P(uT_X, uT_Y, uT_Z, headingDeg)) {
        Serial3.print("X: "); Serial3.print(uT_X, 2); Serial3.print(" uT");
        Serial3.print("   Y: "); Serial3.print(uT_Y, 2); Serial3.print(" uT");
        Serial3.print("   Z: "); Serial3.print(uT_Z, 2); Serial3.print(" uT");
        Serial3.print("   Heading: "); Serial3.print(headingDeg, 1); Serial3.println(" deg");
      } else {
        Serial3.println("Magnetometer: read error or no data");
      }
    }

    // -------- IMU (MPU6050) - Acceleration, Gyro, Temp Query --------
      else if (cmd == IMU) {
      sensors_event_t a, g, temp;
      imu.getEvent(&a, &g, &temp);
      float norm_a = sqrt (a.acceleration.x * a.acceleration.x + a.acceleration.y * a.acceleration.y+ a.acceleration.z * a.acceleration.z);
      float a0 = (a.acceleration.x / norm_a) * 9.806;
      float a1 = (a.acceleration.y / norm_a) * 9.806;
      float a2 = (a.acceleration.z / norm_a) *  9.806;
      Serial3.print("Acceleration X: "); Serial3.print(a0);
      Serial3.print(", Y: "); Serial3.print(a1);
      Serial3.print(", Z: "); Serial3.print(a2);
      Serial3.println(" m/s^2");
      Serial3.print("Rotation X: "); Serial3.print(g.gyro.x);
      Serial3.print(", Y: "); Serial3.print(g.gyro.y);
      Serial3.print(", Z: "); Serial3.print(g.gyro.z);
      Serial3.println(" rad/s");
      Serial3.print("Temperature: "); Serial3.print(temp.temperature);
      Serial3.println(" degC");
      Serial3.println();
    }

    // -------- Solar Cell Voltage Queries --------
    else if (cmd == RIGHT_UPPER_CELL_VOLTAGE) {
      ruCellVoltage = (analogRead(ruCell1) - analogRead(ruCell2)) / 1024.0 * 5.0;
      Serial3.print("Right Upper Cell Voltage: ");
      Serial3.print(ruCellVoltage);
      Serial3.println(" V");
    }
    else if (cmd == RIGHT_LOWER_CELL_VOLTAGE) {
      rlCellVoltage = (analogRead(rlCell1) - analogRead(rlCell2)) / 1024.0 * 5.0;
      Serial3.print("Right Lower Cell Voltage: ");
      Serial3.print(rlCellVoltage);
      Serial3.println(" V");
    }
    else if (cmd == LEFT_UPPER_CELL_VOLTAGE) {
      luCellVoltage = (analogRead(luCell1) - analogRead(luCell2)) / 1024.0 * 5.0;
      Serial3.print("Left Upper Cell Voltage: ");
      Serial3.print(luCellVoltage);
      Serial3.println(" V");
    }
    else if (cmd == LEFT_LOWER_CELL_VOLTAGE) {
      llCellVoltage = (analogRead(llCell1) - analogRead(llCell2)) / 1024.0 * 5.0;
      Serial3.print("Left Lower Cell Voltage: ");
      Serial3.print(llCellVoltage);
      Serial3.println(" V");
    }

    // -------- Motor Commands: Start/Stop --------
    else if (cmd == MOTOR_MAX) {
      digitalWrite(enA, HIGH);    // Set motor A to maximum speed
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      Serial3.println("Motor: Max speed");
    }
    else if (cmd == MOTOR_OFF) {
      digitalWrite(enA, LOW);     // Stop motor
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      Serial3.println("Motor: Off");
    }
  }

  // ---------------- LED Blinking with millis() ----------------
  if (blinking) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      ledState = !ledState;
      digitalWrite(led, ledState);
    }
  } else {
    digitalWrite(led, LOW);
    ledState = LOW;
  }
}

// ---------------- TFT Display Test Text Helper ----------------
// Prints a text string with color on the TFT display. Used for splash screens or status.
void testdrawtext(char *text, uint16_t color) {
  tft.setCursor(5, 70);
  tft.setTextColor(color);
  tft.setTextWrap(true);
  tft.print(text);
}

// ---------------- QMC5883P (GY-271) Magnetometer Helpers ----------------
// Low-level initialization and data readout for the GY-271 module (QMC5883P variant)
// using direct I2C register access (replaces QMC5883LCompass library).

// Initialize QMC5883P at address QMC_ADDR
void initQMC5883P() {
  delay(100);

  // Register 0x0B: Set/Reset Period -> Value 0x01
  Wire.beginTransmission(QMC_ADDR);
  Wire.write(0x0B); 
  Wire.write(0x01); 
  Wire.endTransmission();

  // Register 0x0A: Control Register 1 
  // Value 0x1D -> Continuous Mode, 200Hz, 8G Range, OSR 512
  // (Binary: 0001 1101)
  Wire.beginTransmission(QMC_ADDR);
  Wire.write(0x0A); 
  Wire.write(0x1D); 
  Wire.endTransmission();
}

// Read magnetic field data from QMC5883P and convert to micro Tesla (uT)
// Also computes heading angle in degrees in the XY plane.
// Returns true if valid data was read, false otherwise.
bool readQMC5883P(float &uT_X, float &uT_Y, float &uT_Z, float &headingDeg) {
  // 1. Point to the data registers starting at 0x01 (X LSB)
  Wire.beginTransmission(QMC_ADDR);
  Wire.write(0x01); 
  Wire.endTransmission();

  // 2. Request 6 bytes: X(2), Y(2), Z(2)
  Wire.requestFrom(QMC_ADDR, 6);
  if (Wire.available() < 6) {
    return false; // Not enough data available
  }

  // 3. Read low byte first, then high byte (signed 16-bit, two's complement)
  int16_t rawX = Wire.read() | (Wire.read() << 8);
  int16_t rawY = Wire.read() | (Wire.read() << 8);
  int16_t rawZ = Wire.read() | (Wire.read() << 8);

  // 4. Convert to Micro Tesla (uT)
  // For Range 8G, sensitivity is usually ~3000 LSB/Gauss
  // 1 Gauss = 100 uT
  const float lsb_per_gauss = 3000.0;
  const float gauss_to_uT   = 100.0;

  uT_X = (rawX / lsb_per_gauss) * gauss_to_uT;
  uT_Y = (rawY / lsb_per_gauss) * gauss_to_uT;
  uT_Z = (rawZ / lsb_per_gauss) * gauss_to_uT;

  // 5. Calculate Heading in degrees (XY plane)
  headingDeg = atan2(uT_Y, uT_X) * 180.0 / PI;
  if (headingDeg < 0) headingDeg += 360.0;

  return true;
}