/**************************************************************************
 * 
 * Interfacing Arduino Mega2560 NodeMCU with ST7789 TFT display (240x240 pixel).
 * Graphics test example.
 * This is a free software with NO WARRANTY.
 * https://github.com/Aliken-me
 *
 *************************************************************************/
/**************************************************************************
  This code is modified for Sepehr EDU-Sat Version 1.0
  The Original Code is by Adafruit ST7789 Examples
  Written by Aliken for Sepehr Space Team.
  MIT license, all text above must be included in any redistribution
 *************************************************************************/

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789

// Motor A connections
int enA = 2;
int in1 = 3;
int in2 = 4;

// ST7789 TFT module connections
#define TFT_DC    8     // TFT DC  pin is connected to Mega2560 pin8
#define TFT_RST   9     // TFT RST pin is connected to Mega2560 pin9
#define TFT_CS    53     // TFT CS  pin is connected to Mega2560 pin53
//#define TFT_SCLK 52  //TFT SCK  pin is connected to Mega2560 pin52
//#define TFT_MOSI 51  //TFT SDA  pin is connected to Mega2560 pin51

// Display initialization
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
//Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST); //Option2: Define manual SPI MOSI/SCLK for 

float p = 3.1415926;

void setup(void) {
  Serial.begin(115200);
  
  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);

  Serial.print(F("Hello! ST7789 TFT Test "));

  // if the display has CS pin try with SPI_MODE0, else use SPI_MODE3
  tft.init(240, 240, SPI_MODE3);    // Init ST7789 display 240x240 pixel

  // if the screen is flipped, remove this command
  tft.setRotation(1);

  Serial.println(F("Initialized"));

  // large block of text
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(3);
  testdrawtext("Sepehr EDUSAT v1001", ST77XX_WHITE);
}

void loop() {
  Serial.println("Direction Control Mode");
  directionControl();
  delay(1000);
  Serial.println("Speed Control Mode");
  speedControl();
  delay(1000);
}

// This function lets you control spinning direction of motors
void directionControl() {
  // Set motors to maximum speed
  digitalWrite(enA, HIGH);

  // Turn on motor A & B
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  delay(2000);

  // Now change motor directions
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  delay(2000);

  // Turn off motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}

// This function lets you control speed of the motors
void speedControl() {
  // Turn on motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  // Accelerate from zero to maximum speed
  for (int i = 0; i < 256; i++) {
    analogWrite(enA, i);
    delay(20);
  }

  // Decelerate from maximum speed to zero
  for (int i = 255; i >= 0; --i) {
    analogWrite(enA, i);
    delay(20);
  }

  // Now turn off motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}

void testdrawtext(char *text, uint16_t color) {
  tft.setCursor(0, 0);
  tft.setTextColor(color);
  tft.setTextWrap(true);
  tft.print(text);
}