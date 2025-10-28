// Include necessary libraries for hardware interactions and sensor handling
#include <Arduino.h>              // Core Arduino library
#include <Wire.h>                 // Standard I2C communication library
#include <SoftWire.h>             // Software I2C library for custom pins
#include "AHT10.h"                // AHT10 temperature/humidity sensor library (supports hardware/software I2C)
#include <QMC5883LCompass.h>      // Library for GY-271 Magnetometer (QMC5883L)
#include <Adafruit_GFX.h>         // Core graphics library for displays
#include <Adafruit_ST7789.h>      // Hardware-specific library for ST7789 TFT display
#include <Adafruit_MPU6050.h>     // Library for MPU6050 6DOF IMU
#include <Adafruit_Sensor.h>      // Base sensor library for event structures

// SoftWire configuration for custom I2C pins (used for back AHT10 sensor)
// This allows flexible pin assignment for I2C in multi-sensor setups
int sdaPin = 40;  // SDA pin for back AHT10 (configurable GPIO)
int sclPin = 41;  // SCL pin for back AHT10 (configurable GPIO)
SoftWire sw(sdaPin, sclPin);
char swTxBuffer[16];  // Transmit buffer for SoftWire
char swRxBuffer[16];  // Receive buffer for SoftWire

// ST7789 TFT display pin connections
// These pins are used for SPI communication with the display
#define TFT_DC    8    // Data/Command control pin
#define TFT_RST   9    // Reset pin
#define TFT_CS   53    // Chip Select pin

// Sensor object initializations
// Front AHT10 uses hardware I2C (Wire), back uses software I2C (SoftWire)
AHT10 aht10_hw(0x38, BUS_WIRE, &Wire, nullptr);       // Front AHT10 sensor (hardware I2C)
AHT10 aht10_sw(0x38, BUS_SOFTWIRE, nullptr, &sw);     // Back AHT10 sensor (software I2C)
QMC5883LCompass gy271;                                // GY-271 Magnetometer object
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST); // ST7789 TFT display object
Adafruit_MPU6050 imu;                                 // MPU6050 IMU object

// Variables for LED blinking control
// Used to manage onboard LED (pin 13) for status indication
const int led = 13;                   // Onboard LED pin on Mega2560
unsigned long previousMillisLed = 0;  // Timestamp for last LED state change
const long intervalLed = 500;         // Blink interval in milliseconds
int ledState = LOW;                   // Current LED state (LOW/HIGH)
bool blinking = false;                // Flag to enable/disable blinking

// Motor A pin connections (for L298N/L293D driver)
// Controls direction and speed of a DC motor
int enA = 2;   // PWM enable/speed control pin
int in1 = 3;   // Direction control pin 1
int in2 = 4;   // Direction control pin 2

// Solar cell analog input pins
// Each cell uses differential measurement (V+ - V-) for voltage reading
int ruCell1 = A0;   // Right Upper cell V+
int ruCell2 = A1;   // Right Upper cell V-
int rlCell1 = A3;   // Right Lower cell V+
int rlCell2 = A4;   // Right Lower cell V-
int luCell1 = A8;   // Left Upper cell V+
int luCell2 = A9;   // Left Upper cell V-
int llCell1 = A10;  // Left Lower cell V+
int llCell2 = A11;  // Left Lower cell V-

// Variables to store solar cell voltages
// Calculated as (analogRead(V+) - analogRead(V-)) / 1024.0 * 5.0
double ruCellVoltage = 0.0;  // Right Upper voltage
double rlCellVoltage = 0.0;  // Right Lower voltage
double luCellVoltage = 0.0;  // Left Upper voltage
double llCellVoltage = 0.0;  // Left Lower voltage

// Stored sensor data for SENSOR_STORE_AND_SEND mode
// These variables hold the latest readings for on-demand reporting
float frontTemp = 0.0;    // Front AHT10 temperature (Celsius)
float frontHum = 0.0;     // Front AHT10 humidity (percent)
float backTemp = 0.0;     // Back AHT10 temperature (Celsius)
float backHum = 0.0;      // Back AHT10 humidity (percent)
int magX = 0, magY = 0, magZ = 0;  // Magnetometer readings (X, Y, Z in nT)
sensors_event_t imuAccel, imuGyro, imuTemp;  // IMU sensor events (accel, gyro, temp)

// Variables specific to ATTITUDE_HOLD mode
// These are used for PID control and gyro-based attitude stabilization
#define REVERSE_MOTOR false   // Flag to reverse motor direction if needed
#define DEADBAND 0.2          // Deadband threshold in degrees for PID
#define MPU6050_ADDR 0x68     // I2C address of MPU6050
#define ACCEL_CONFIG 0x1C     // Accelerometer config register
#define GYRO_CONFIG  0x1B     // Gyroscope config register
#define PWR_MGMT_1   0x6B     // Power management register 1
#define PWR_MGMT_2   0x6C     // Power management register 2
const float GYRO_SCALE = 65.5f;  // Scale factor for ±500 dps gyro range
uint8_t i2cData[14];             // Buffer for I2C data from MPU6050
int16_t GyZ_raw;                 // Raw Z-axis gyro reading
float gyro_offset = 0.0f;        // Calibrated gyro offset
float angle = 0.0f;              // Integrated angle from gyro
float gyro_rate = 0.0f;          // Current gyro rate (deg/s)
float gyroZfilt = 0.0f;          // Filtered gyro rate
float integral = 0.0f;           // Integral term for PID
float raw_u = 0.0f;              // Raw PID output
float setpoint = 0.0f;           // PID setpoint (target angle)
float Kp = 2.0f;                 // PID proportional gain
float Ki = 0.05f;                // PID integral gain
float Kd = 0.3f;                 // PID derivative gain
float gyro_lpf_alpha = 0.3f;     // Alpha for gyro low-pass filter
const unsigned long loop_time_us = 5000UL;  // Control loop period (5ms, 200Hz)
unsigned long lastMicrosAttitude = 0;       // Timestamp for last attitude loop
unsigned long lastLcdMicros = 0;            // Timestamp for last LCD update
const unsigned long lcd_period_us = 100000UL;  // LCD update period (100ms, 10Hz)
// Color definitions for ST7789 compatibility
#ifndef ST77XX_DARKGREY
  #define ST77XX_DARKGREY  0x7BEF
#endif
#ifndef ST77XX_LIGHTGREY
  #define ST77XX_LIGHTGREY 0xC618
#endif

// List of allowed commands for mode switching and sensor queries
// These are matched against incoming serial commands
const String commands[] = {
  "MODE_IDLE",              // Switch to IDLE mode
  "MODE_SENSOR",            // Switch to SENSOR_STORE_AND_SEND mode
  "MODE_ATTITUDE",          // Switch to ATTITUDE_HOLD mode
  "LED_ON",                 // Enable LED blinking
  "LED_OFF",                // Disable LED blinking
  "CAMERA_CAPTURE",         // Trigger camera capture via ESP32-CAM
  "FRONT_TEMP",             // Report front temperature
  "FRONT_HUM",              // Report front humidity
  "BACK_TEMP",              // Report back temperature
  "BACK_HUM",               // Report back humidity
  "MAGNETOMETER",           // Report magnetometer data
  "IMU",                    // Report IMU data
  "RIGHT_UPPER_CELL_VOLTAGE", // Report right upper solar voltage
  "RIGHT_LOWER_CELL_VOLTAGE", // Report right lower solar voltage
  "LEFT_UPPER_CELL_VOLTAGE",  // Report left upper solar voltage
  "LEFT_LOWER_CELL_VOLTAGE",  // Report left lower solar voltage
  "MOTOR_MAX",                // Set motor to max speed
  "MOTOR_OFF"                 // Turn off motor
};
const int NUM_COMMANDS = 18;  // Total number of commands

// System modes enumeration for state management
// Defines the operational states of the system
enum SystemMode {
  IDLE,                     // Idle state: no active operations
  SENSOR_STORE_AND_SEND,    // Mode to cyclically store sensor data and respond to queries
  ATTITUDE_HOLD             // Mode for gyro-based attitude control with PID
};

SystemMode currentMode = IDLE;  // Initial system mode

// Timing variables for SENSOR_STORE_AND_SEND mode
// Controls periodic sensor readings
unsigned long previousMillisSensor = 0;  // Timestamp for last sensor read
const long intervalSensor = 1000;        // Sensor read interval (1 second)

// Input string buffer for serial commands
// Used to accumulate incoming data until complete
String inputString = "";
bool stringComplete = false;  // Flag indicating a complete command received

// Setup function: initializes hardware and sensors
// Called once at startup
void setup() {
  // Initialize serial ports
  Serial.begin(9600);      // Debug serial (USB/PC)
  Serial2.begin(115200);   // Serial for ESP32-CAM payload
  Serial3.begin(115200);   // Serial for TT&C (ESP8266)

  // Initialize output pins
  pinMode(led, OUTPUT);    // LED pin as output
  pinMode(enA, OUTPUT);    // Motor enable pin
  pinMode(in1, OUTPUT);    // Motor direction pin 1
  pinMode(in2, OUTPUT);    // Motor direction pin 2
  digitalWrite(in1, LOW);  // Ensure motor is off initially
  digitalWrite(in2, LOW);

  // Initialize I2C buses
  Wire.begin();            // Hardware I2C
  sw.setTxBuffer(swTxBuffer, sizeof(swTxBuffer));  // Configure SoftWire buffers
  sw.setRxBuffer(swRxBuffer, sizeof(swRxBuffer));
  sw.setDelay_us(5);       // Set microsecond delay for stability
  sw.setTimeout(1000);     // Set timeout in milliseconds

  // Initialize sensors
  aht10_hw.begin();        // Front AHT10 (hardware I2C)
  aht10_sw.begin();        // Back AHT10 (software I2C)
  gy271.init();            // Magnetometer

  // Initialize TFT display
  tft.init(240, 240, SPI_MODE3);  // 240x240 resolution, SPI mode 3
  tft.setRotation(1);             // Set rotation (landscape/portrait)
  tft.fillScreen(ST77XX_WHITE);   // Clear screen to white
  tft.setTextSize(3);             // Set text size
  testdrawtext((char*)"Sepehr EDUSAT    v1001", ST77XX_BLUE);  // Display splash text

  // Initialize MPU6050 IMU
  imu.begin();
  imu.setAccelerometerRange(MPU6050_RANGE_8_G);    // ±8G range
  imu.setGyroRange(MPU6050_RANGE_1000_DEG);        // ±1000 deg/s range
  imu.setFilterBandwidth(MPU6050_BAND_5_HZ);       // 5Hz low-pass filter

  // Initialize attitude hold specifics
  attitude_setup();

  // Reserve memory for input string to prevent fragmentation
  inputString.reserve(200);
}

// Main loop function: runs repeatedly
// Handles serial input, command processing, and mode execution
void loop() {
  // Check for incoming serial data on Serial3
  checkSerialInput();

  // Process complete command if available
  if (stringComplete) {
    processCommand(inputString);
    inputString = "";        // Reset input string
    stringComplete = false;  // Reset completion flag
  }

  // Execute the current mode's logic
  switch (currentMode) {
    case SENSOR_STORE_AND_SEND:
      runSensorMode();
      break;
    case ATTITUDE_HOLD:
      runAttitudeMode();
      break;
    case IDLE:
      // No operations in idle mode
      break;
  }
}

// Function to read serial input non-blockingly
// Accumulates characters until newline is received
void checkSerialInput() {
  while (Serial3.available()) {
    char inChar = (char)Serial3.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

// Function to process incoming command
// Matches command to predefined list and handles mode switches or queries
void processCommand(String cmd) {
  cmd.trim();  // Remove whitespace
  for (int i = 0; i < NUM_COMMANDS; i++) {
    if (cmd == commands[i]) {
      switch (i) {
        // Mode switch commands
        case 0: currentMode = IDLE; Serial3.println("Mode: IDLE"); return;
        case 1: currentMode = SENSOR_STORE_AND_SEND; Serial3.println("Mode: SENSOR_STORE_AND_SEND"); return;
        case 2: currentMode = ATTITUDE_HOLD; Serial3.println("Mode: ATTITUDE_HOLD"); return;

        // Delegate other commands based on current mode
        default:
          if (currentMode == SENSOR_STORE_AND_SEND) {
            handleSensorCommands(cmd);
            return;
          } else if (currentMode == ATTITUDE_HOLD) {
            handleAttitudeCommands(cmd);
            return;
          }
      }
    }
  }
  // Handle invalid commands
  Serial3.println("Invalid command");
}

// Function to handle commands in SENSOR_STORE_AND_SEND mode
// Processes queries for sensor data and controls
void handleSensorCommands(String cmd) {
  if (cmd == "LED_ON") {
    blinking = true;
    Serial3.println("LED Blinking: [On]");
  } else if (cmd == "LED_OFF") {
    blinking = false;
    Serial3.println("LED Blinking: [Off]");
  } else if (cmd == "CAMERA_CAPTURE") {
    Serial2.println("capture");  // Send capture command to ESP32-CAM
    if (Serial2.available()) {
      String cam_resp = Serial2.readStringUntil('\n');
      cam_resp.trim();
      Serial.println(cam_resp);  // Debug response
    }
  } else if (cmd == "FRONT_TEMP") {
    Serial3.print("[Front] Temperature: ");
    Serial3.print(frontTemp, 1);
    Serial3.println(" C");
  } else if (cmd == "FRONT_HUM") {
    Serial3.print("[Front] Humidity: ");
    Serial3.print(frontHum, 1);
    Serial3.println(" %");
  } else if (cmd == "BACK_TEMP") {
    Serial3.print("[Back] Temperature: ");
    Serial3.print(backTemp, 1);
    Serial3.println(" C");
  } else if (cmd == "BACK_HUM") {
    Serial3.print("[Back] Humidity: ");
    Serial3.print(backHum, 1);
    Serial3.println(" %");
  } else if (cmd == "MAGNETOMETER") {
    Serial3.print("X: "); Serial3.print(magX); Serial3.print(" nT");
    Serial3.print("   Y: "); Serial3.print(magY); Serial3.print(" nT");
    Serial3.print("   Z: "); Serial3.print(magZ); Serial3.println(" nT");
  } else if (cmd == "IMU") {
    Serial3.print("Acceleration X: "); Serial3.print(imuAccel.acceleration.x);
    Serial3.print(", Y: "); Serial3.print(imuAccel.acceleration.y);
    Serial3.print(", Z: "); Serial3.print(imuAccel.acceleration.z);
    Serial3.println(" m/s^2");
    Serial3.print("Rotation X: "); Serial3.print(imuGyro.gyro.x);
    Serial3.print(", Y: "); Serial3.print(imuGyro.gyro.y);
    Serial3.print(", Z: "); Serial3.print(imuGyro.gyro.z);
    Serial3.println(" rad/s");
    Serial3.print("Temperature: "); Serial3.print(imuTemp.temperature);
    Serial3.println(" degC");
  } else if (cmd == "RIGHT_UPPER_CELL_VOLTAGE") {
    Serial3.print("Right Upper Cell Voltage: ");
    Serial3.print(ruCellVoltage);
    Serial3.println(" V");
  } else if (cmd == "RIGHT_LOWER_CELL_VOLTAGE") {
    Serial3.print("Right Lower Cell Voltage: ");
    Serial3.print(rlCellVoltage);
    Serial3.println(" V");
  } else if (cmd == "LEFT_UPPER_CELL_VOLTAGE") {
    Serial3.print("Left Upper Cell Voltage: ");
    Serial3.print(luCellVoltage);
    Serial3.println(" V");
  } else if (cmd == "LEFT_LOWER_CELL_VOLTAGE") {
    Serial3.print("Left Lower Cell Voltage: ");
    Serial3.print(llCellVoltage);
    Serial3.println(" V");
  } else if (cmd == "MOTOR_MAX") {
    digitalWrite(enA, HIGH);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    Serial3.println("Motor: Max speed");
  } else if (cmd == "MOTOR_OFF") {
    digitalWrite(enA, LOW);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    Serial3.println("Motor: Off");
  }
}

// Function to handle commands in ATTITUDE_HOLD mode
// Currently minimal; can be expanded for mode-specific commands
void handleAttitudeCommands(String cmd) {
  // Placeholder for attitude-specific Serial3 commands
  // Motor commands or others can be added here if needed
}

// Function for SENSOR_STORE_AND_SEND mode logic
// Handles LED blinking and periodic sensor data storage
void runSensorMode() {
  unsigned long currentMillis = millis();

  // Manage LED blinking if enabled
  if (blinking) {
    if (currentMillis - previousMillisLed >= intervalLed) {
      previousMillisLed = currentMillis;
      ledState = !ledState;
      digitalWrite(led, ledState);
    }
  } else {
    digitalWrite(led, LOW);
    ledState = LOW;
  }

  // Read and store sensor data periodically
  if (currentMillis - previousMillisSensor >= intervalSensor) {
    previousMillisSensor = currentMillis;

    // Read front AHT10
    float hum, temp;
    if (aht10_hw.read(hum, temp)) {
      frontTemp = temp;
      frontHum = hum;
    }

    // Read back AHT10
    if (aht10_sw.read(hum, temp)) {
      backTemp = temp;
      backHum = hum;
    }

    // Read magnetometer
    gy271.read();
    magX = gy271.getX();
    magY = gy271.getY();
    magZ = gy271.getZ();

    // Read IMU
    imu.getEvent(&imuAccel, &imuGyro, &imuTemp);

    // Read solar cell voltages
    ruCellVoltage = (analogRead(ruCell1) - analogRead(ruCell2)) / 1024.0 * 5.0;
    rlCellVoltage = (analogRead(rlCell1) - analogRead(rlCell2)) / 1024.0 * 5.0;
    luCellVoltage = (analogRead(luCell1) - analogRead(luCell2)) / 1024.0 * 5.0;
    llCellVoltage = (analogRead(llCell1) - analogRead(llCell2)) / 1024.0 * 5.0;
  }
}

// Function for ATTITUDE_HOLD mode logic
// Performs gyro reading, PID computation, motor control, and LCD updates
void runAttitudeMode() {
  unsigned long now = micros();
  float dt = (now - lastMicrosAttitude) * 1e-6f;
  if (dt <= 0.0f) dt = 1e-3f;

  // Run control loop at fixed rate
  if ((now - lastMicrosAttitude) >= loop_time_us) {
    lastMicrosAttitude = now;

    // Calculate current angle from gyro
    angle_calc(dt);

    // Compute PID terms
    float error = angle - setpoint;
    float integral_candidate = integral + error * dt;
    float u_test = -(Kp * error + Kd * gyroZfilt + Ki * integral_candidate);
    if (abs(u_test) <= 255.0f) integral = integral_candidate;
    integral = constrain(integral, -2000.0f, 2000.0f);

    raw_u = -(Kp * error + Kd * gyroZfilt + Ki * integral);
    int pwm_s = (int)constrain(raw_u, -255.0f, 255.0f);

    // Apply deadband
    if (abs(error) < DEADBAND) pwm_s = 0;
    Motor_control(pwm_s);

    // Update LCD at decimated rate
    if ((now - lastLcdMicros) >= lcd_period_us) {
      lastLcdMicros = now;
      lcd_update_values(angle, setpoint, error, gyro_rate, gyroZfilt, integral, pwm_s, dt);
    }
  }

  // Check for tuning commands on debug serial
  checkSerialForTuning();
}

// Helper function for I2C write to MPU6050
// Writes data to specified register
int i2cWrite(uint8_t reg, uint8_t *data, uint8_t len, bool wait) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  for (uint8_t i = 0; i < len; i++) Wire.write(data[i]);
  return Wire.endTransmission(wait);
}

// Helper function for I2C read from MPU6050
// Reads specified number of bytes from register
int i2cRead(uint8_t reg, uint8_t *data, uint8_t nbytes) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)nbytes, (uint8_t)1);
  for (uint8_t i = 0; i < nbytes; i++) data[i] = Wire.read();
  return 0;
}

// Setup function for attitude hold mode
// Initializes MPU6050, calibrates gyro, sets initial setpoint, and configures LCD
void attitude_setup() {
  Wire.setWireTimeout(2500, true);  // Set I2C timeout

  uint8_t d = 0x00;
  while (i2cWrite(PWR_MGMT_1, &d, 1, false));  // Wake up device
  while (i2cWrite(PWR_MGMT_2, &d, 1, false));  // Enable all axes

  d = 0x00; while (i2cWrite(ACCEL_CONFIG, &d, 1, false));  // ±2g accel range
  d = 0x08; while (i2cWrite(GYRO_CONFIG,  &d, 1, false));   // ±500 dps gyro range

  // Calibrate gyro offset
  long sum = 0;
  const int Ngyro = 2000;
  for (int i = 0; i < Ngyro; i++) {
    while (i2cRead(0x3B, i2cData, 14));
    int16_t gz = (int16_t)((i2cData[12] << 8) | i2cData[13]);
    sum += (long)gz;
    delay(2);
  }
  gyro_offset = (float)sum / (float)Ngyro;

  angle = 0.0f;
  gyroZfilt = 0.0f;

  // Set initial setpoint from first angle calculation
  angle_calc(0.001f);
  setpoint = angle;

  // Initialize and draw static LCD content
  lcd_setup();
  lcd_draw_static();
  lcd_show_gains();
}

// Function to calculate angle from gyro data
// Reads raw gyro, applies scale and filter, integrates to angle
void angle_calc(float dt) {
  while (i2cRead(0x3B, i2cData, 14));
  GyZ_raw = (int16_t)((i2cData[12] << 8) | i2cData[13]);

  gyro_rate = ((float)GyZ_raw - gyro_offset) / GYRO_SCALE;
  gyroZfilt = gyro_lpf_alpha * gyro_rate + (1.0f - gyro_lpf_alpha) * gyroZfilt;

  angle += gyro_rate * dt;
}

// Function to control motor based on PWM value
// Handles direction and speed
void Motor_control(int pwm) {
  int abs_pwm = abs(pwm);
  if (REVERSE_MOTOR) pwm = -pwm;

  if (pwm > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, abs_pwm);
  } else if (pwm < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, abs_pwm);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(enA, 0);
  }
}

// Function to check debug serial for PID tuning commands
// Adjusts gains and setpoint based on input strings
void checkSerialForTuning() {
  static String buffer = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      buffer.trim();
      if (buffer == "p+") Kp += 1.0;
      else if (buffer == "p-") Kp -= 1.0;
      else if (buffer == "i+") Ki += 0.01;
      else if (buffer == "i-") Ki -= 0.01;
      else if (buffer == "d+") Kd += 0.25;
      else if (buffer == "d-") Kd -= 0.25;
      else if (buffer == "s+") setpoint += 1.0;
      else if (buffer == "s-") setpoint -= 1.0;

      // Constrain gains to non-negative
      if (Kp < 0) Kp = 0;
      if (Ki < 0) Ki = 0;
      if (Kd < 0) Kd = 0;

      // Update LCD with new gains
      lcd_show_gains();
      buffer = "";
    } else {
      buffer += c;
    }
  }
}

// LCD initialization function
// Sets up display parameters
void lcd_setup() {
  tft.init(240, 240, SPI_MODE3);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextWrap(false);
}

// Helper to draw labels on LCD
void drawLabel(int16_t x, int16_t y, const char* txt) {
  tft.setCursor(x, y);
  tft.setTextColor(ST77XX_CYAN);
  tft.setTextSize(2);
  tft.print(txt);
}

// Function to draw LCD header
void drawHeader() {
  tft.fillRect(0, 0, 240, 24, ST77XX_BLUE);
  tft.setCursor(6, 4);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.print("EDUSAT PID Monitor");
}

// Function to draw static LCD elements (labels, frames)
void lcd_draw_static() {
  tft.fillScreen(ST77XX_BLACK);
  drawHeader();

  int y0 = 28;
  int lh = 20;  // Line height
  drawLabel(6,  y0 + 0*lh, "Angle:");
  drawLabel(6,  y0 + 1*lh, "Setpt:");
  drawLabel(6,  y0 + 2*lh, "Error:");
  drawLabel(6,  y0 + 3*lh, "Gyro :");
  drawLabel(6,  y0 + 4*lh, "G_f :");
  drawLabel(6,  y0 + 5*lh, "Int  :");
  drawLabel(6,  y0 + 6*lh, "PWM :");
  drawLabel(6,  y0 + 7*lh, "dt  :");

  tft.drawRect(0, 190, 240, 50, ST77XX_DARKGREY);
  tft.setCursor(6, 192);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setTextSize(2);
  tft.print("Gains (p+/p-/i+/i-/d+/d-)");
}

// Helper to print float value on LCD, clearing previous area
void printValueAt(int16_t x, int16_t y, float val, uint8_t width, uint8_t prec) {
  tft.fillRect(x, y-2, 120, 18, ST77XX_BLACK);
  char buf[20];
  dtostrf(val, width, prec, buf);
  tft.setCursor(x, y);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.print(buf);
}

// Helper to print integer value on LCD, clearing previous area
void printIntAt(int16_t x, int16_t y, int val) {
  tft.fillRect(x, y-2, 120, 18, ST77XX_BLACK);
  tft.setCursor(x, y);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.print(val);
}

// Function to update dynamic values on LCD
void lcd_update_values(float angle, float setpoint, float error,
                       float gyro_rate, float gyro_filt, float integ,
                       int pwm, float dt) {
  int y0 = 28;
  int lh = 20;
  int xv = 110;  // Value column x-position

  printValueAt(xv, y0 + 0*lh, angle,     7, 2);
  printValueAt(xv, y0 + 1*lh, setpoint,  7, 2);
  printValueAt(xv, y0 + 2*lh, error,     7, 2);
  printValueAt(xv, y0 + 3*lh, gyro_rate, 7, 2);
  printValueAt(xv, y0 + 4*lh, gyro_filt, 7, 2);
  printValueAt(xv, y0 + 5*lh, integ,     7, 2);
  printIntAt  (xv, y0 + 6*lh, pwm);
  printValueAt(xv, y0 + 7*lh, dt*1000.0f, 7, 3);  // dt in ms
}

// Function to display current PID gains on LCD
void lcd_show_gains() {
  tft.fillRect(0, 212, 240, 26, ST77XX_BLACK);
  tft.setCursor(6, 214);
  tft.setTextColor(ST77XX_GREEN);
  tft.setTextSize(2);
  tft.print("Kp:");
  tft.print(Kp, 2);
  tft.print(" Ki:");
  tft.print(Ki, 3);
  tft.print(" Kd:");
  tft.print(Kd, 2);
}

// Helper function for TFT test text display
// Used for initial splash screen
void testdrawtext(char *text, uint16_t color) {
  tft.setCursor(5, 70);
  tft.setTextColor(color);
  tft.setTextWrap(true);
  tft.print(text);
}