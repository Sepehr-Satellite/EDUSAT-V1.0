#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>

// --- Color compatibility for some Adafruit ST77xx versions ---
#ifndef ST77XX_DARKGREY
  #define ST77XX_DARKGREY  0x7BEF  // 16-bit 565
#endif
#ifndef ST77XX_LIGHTGREY
  #define ST77XX_LIGHTGREY 0xC618
#endif


// ===================== Pin Definitions ====================== //
int enA = 2;   // Enable/speed control (PWM)
int in1 = 3;   // Direction 1
int in2 = 4;   // Direction 2

#define REVERSE_MOTOR false // Set to true if motor direction is inverted
#define DEADBAND 0.2        // Deadband in degrees (may need adjustment for rotation)

// ==================== MPU6050 Constants ===================== //
#define MPU6050_ADDR 0x68
#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG  0x1B
#define PWR_MGMT_1   0x6B
#define PWR_MGMT_2   0x6C

// Gyroscope scale factor for ±500 dps
const float GYRO_SCALE = 65.5f;

// ==================== State Variables ======================= //
uint8_t i2cData[14];
int16_t GyZ_raw;
float gyro_offset = 0.0f;
float angle = 0.0f;
float gyro_rate = 0.0f;
float gyroZfilt = 0.0f;
float integral = 0.0f;
float raw_u = 0.0f;
float setpoint = 0.0f; // Setpoint for PID (initial measurement)

// ======================== PID Gains ========================= //
float Kp = 2.0f;  // Proportional gain
float Ki = 0.05f;  // Integral gain
float Kd = 0.3f; // Derivative gain

// ==================== Filter Parameters ===================== //
float gyro_lpf_alpha = 0.3f; // Low-pass filter for gyro

// =========================== Timing ========================= //
const unsigned long loop_time_us = 5000UL; // 5ms = 200Hz
unsigned long lastMicros = 0;

// ============ LCD (ST7789 240x240) Configuration ============ //
// Pins per your sample
#define TFT_DC   8
#define TFT_RST  9
#define TFT_CS   53
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// UI timing: refresh LCD at 10 Hz to avoid flicker / SPI overhead
unsigned long lastLcdMicros = 0;
const unsigned long lcd_period_us = 100000UL; // 100ms

// Predeclare
void lcd_setup();
void lcd_draw_static();
void lcd_update_values(float angle, float setpoint, float error,
                       float gyro_rate, float gyro_filt, float integ,
                       int pwm, float dt);
void lcd_show_gains();

// =============== I2C Helper Functions (unchanged) =============== //
int i2cWrite(uint8_t reg, uint8_t *data, uint8_t len, bool wait) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  for (uint8_t i = 0; i < len; i++) Wire.write(data[i]);
  return Wire.endTransmission(wait);
}

int i2cRead(uint8_t reg, uint8_t *data, uint8_t nbytes) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)nbytes, (uint8_t)1);
  for (uint8_t i = 0; i < nbytes; i++) data[i] = Wire.read();
  return 0;
}

// ================== Sensor Initialization ================== //
void angle_setup() {
  Wire.begin();
  Wire.setWireTimeout(2500, true);

  uint8_t d = 0x00;
  while (i2cWrite(PWR_MGMT_1, &d, 1, false));
  while (i2cWrite(PWR_MGMT_2, &d, 1, false));

  d = 0x00; while (i2cWrite(ACCEL_CONFIG, &d, 1, false)); // ±2g
  d = 0x08; while (i2cWrite(GYRO_CONFIG,  &d, 1, false)); // ±500 dps

  // Gyro calibration (keep still)
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
}

// ================== Angle Calculation ================== //
void angle_calc(float dt) {
  while (i2cRead(0x3B, i2cData, 14));
  GyZ_raw = (int16_t)((i2cData[12] << 8) | i2cData[13]);

  gyro_rate = ((float)GyZ_raw - gyro_offset) / GYRO_SCALE; // deg/s
  gyroZfilt = gyro_lpf_alpha * gyro_rate + (1.0f - gyro_lpf_alpha) * gyroZfilt;

  // Integrate gyro
  angle += gyro_rate * dt;
}

// ================== Motor Control ================== //
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

// ================== Serial Tuning (kept for input) ================== //
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

      if (Kp < 0) Kp = 0;
      if (Ki < 0) Ki = 0;
      if (Kd < 0) Kd = 0;

      // نمایش گین‌ها روی LCD
      lcd_show_gains();
      buffer = "";
    } else {
      buffer += c;
    }
  }
}

// ========================== Setup ========================== //
void setup() {
  Serial.begin(115200);

  // Motor driver pins
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(enA, 0);

  // IMU
  angle_setup();

  // Initial setpoint = first measurement
  angle_calc(0.001f);
  setpoint = angle;

  // LCD
  lcd_setup();
  lcd_draw_static();
  lcd_show_gains(); // نمایش اولیه گین‌ها
}

// =========================== Loop ========================== //
void loop() {
  unsigned long now = micros();
  float dt = (now - lastMicros) * 1e-6f;
  if (dt <= 0.0f) dt = 1e-3f;

  if ((now - lastMicros) >= loop_time_us) {
    lastMicros = now;

    // Estimate angle
    angle_calc(dt);

    // PID
    float error = angle - setpoint;
    float integral_candidate = integral + error * dt;
    float u_test = -(Kp * error + Kd * gyroZfilt + Ki * integral_candidate);
    if (abs(u_test) <= 255.0f) integral = integral_candidate;
    integral = constrain(integral, -2000.0f, 2000.0f);

    raw_u = -(Kp * error + Kd * gyroZfilt + Ki * integral);
    int pwm_s = (int)constrain(raw_u, -255.0f, 255.0f);

    if (abs(error) < DEADBAND) pwm_s = 0;
    Motor_control(pwm_s);

    // LCD update @ 10Hz (decimated)
    if ((now - lastLcdMicros) >= lcd_period_us) {
      lastLcdMicros = now;
      lcd_update_values(angle, setpoint, error, gyro_rate, gyroZfilt, integral, pwm_s, dt);
    }
  }

  // Tuning via serial (no serial prints)
  checkSerialForTuning();
}

/* ======================= LCD Helpers ======================= */

void lcd_setup() {
  tft.init(240, 240, SPI_MODE3); // per your sample
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextWrap(false);
}

void drawLabel(int16_t x, int16_t y, const char* txt) {
  tft.setCursor(x, y);
  tft.setTextColor(ST77XX_CYAN);
  tft.setTextSize(2);
  tft.print(txt);
}

void drawHeader() {
  tft.fillRect(0, 0, 240, 24, ST77XX_BLUE);
  tft.setCursor(6, 4);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.print("EDUSAT PID Monitor");
}

void lcd_draw_static() {
  tft.fillScreen(ST77XX_BLACK);
  drawHeader();

  // Labels (left column)
  int y0 = 28;
  int lh = 20; // line height
  drawLabel(6,  y0 + 0*lh, "Angle:");
  drawLabel(6,  y0 + 1*lh, "Setpt:");
  drawLabel(6,  y0 + 2*lh, "Error:");
  drawLabel(6,  y0 + 3*lh, "Gyro :");
  drawLabel(6,  y0 + 4*lh, "G_f :");
  drawLabel(6,  y0 + 5*lh, "Int  :");
  drawLabel(6,  y0 + 6*lh, "PWM :");
  drawLabel(6,  y0 + 7*lh, "dt  :");

  // Gains area frame
  tft.drawRect(0, 190, 240, 50, ST77XX_DARKGREY);
  tft.setCursor(6, 192);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setTextSize(2);
  tft.print("Gains (p+/p-/i+/i-/d+/d-)");
}

// helper to print a value at x,y—clears previous value area
void printValueAt(int16_t x, int16_t y, float val, uint8_t width, uint8_t prec) {
  // area to clear (value column)
  tft.fillRect(x, y-2, 120, 18, ST77XX_BLACK);
  char buf[20];
  dtostrf(val, width, prec, buf);
  tft.setCursor(x, y);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.print(buf);
}

void printIntAt(int16_t x, int16_t y, int val) {
  tft.fillRect(x, y-2, 120, 18, ST77XX_BLACK);
  tft.setCursor(x, y);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.print(val);
}

void lcd_update_values(float angle, float setpoint, float error,
                       float gyro_rate, float gyro_filt, float integ,
                       int pwm, float dt) {
  int y0 = 28;
  int lh = 20;
  int xv = 110; // value column x

  printValueAt(xv, y0 + 0*lh, angle,     7, 2);
  printValueAt(xv, y0 + 1*lh, setpoint,  7, 2);
  printValueAt(xv, y0 + 2*lh, error,     7, 2);
  printValueAt(xv, y0 + 3*lh, gyro_rate, 7, 2);
  printValueAt(xv, y0 + 4*lh, gyro_filt, 7, 2);
  printValueAt(xv, y0 + 5*lh, integ,     7, 2);
  printIntAt  (xv, y0 + 6*lh, pwm);
  // dt به میلی‌ثانیه برای خوانایی
  printValueAt(xv, y0 + 7*lh, dt*1000.0f, 7, 3);
}

void lcd_show_gains() {
  // Clear gains line
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
