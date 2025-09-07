#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>

// ========================== WiFi Configuration ===========================
/**
 * @section WiFi
 * WiFi network credentials and host settings for ESP32-CAM.
 */
const char* WIFI_SSID     = "Hack";         // WiFi network SSID
const char* WIFI_PASSWORD = "by56by56";     // WiFi password
const char* MyHostName    = "EDUSAT PAYLOAD"; // Hostname for DHCP

// ====================== Camera Pin Configuration (AI-Thinker) ============
/**
 * @section Camera Pin Mapping
 * Pin definitions for the AI-Thinker ESP32-CAM module.
 * DO NOT modify unless using a custom ESP32-CAM hardware variant.
 */
#define PWDN_GPIO_NUM     32   // Power down
#define RESET_GPIO_NUM    -1   // Reset (not used)
#define XCLK_GPIO_NUM      0   // External clock
#define SIOD_GPIO_NUM     26   // SCCB data
#define SIOC_GPIO_NUM     27   // SCCB clock
#define Y9_GPIO_NUM       35   // D7
#define Y8_GPIO_NUM       34   // D6
#define Y7_GPIO_NUM       39   // D5
#define Y6_GPIO_NUM       36   // D4
#define Y5_GPIO_NUM       21   // D3
#define Y4_GPIO_NUM       19   // D2
#define Y3_GPIO_NUM       18   // D1
#define Y2_GPIO_NUM        5   // D0
#define VSYNC_GPIO_NUM    25   // Vertical sync
#define HREF_GPIO_NUM     23   // Horizontal reference
#define PCLK_GPIO_NUM     22   // Pixel clock

// =========================== LED Configuration ===========================
/**
 * @section LED
 * LED_PIN is an external status LED (active HIGH).
 * LED_BUILTIN_PIN is the onboard LED (may not exist on all ESP32-CAM modules).
 */
#define LED_PIN         33     // External status LED
#define LED_BUILTIN_PIN  4     // On-board LED (optional, not present on all modules)

// ======================== TCP Server Configuration =======================
/**
 * @section TCP Server
 * imageServer: TCP server on port 5000 for image data transmission.
 * imageClient: The currently connected client to the server.
 */
WiFiServer imageServer(5000);
WiFiClient imageClient;

// ======================= Timing & State Variables ========================
/**
 * @section Runtime State
 * Timing and runtime flags for LED indication and image capture control.
 */
unsigned long ledPreviousMillis = 0;      // Stores last time the LED was toggled
const long ledBlinkInterval    = 500;     // Interval for blinking LED when connecting (ms)
const long ledCaptureInterval  = 70;      // Fast LED blink interval during image capture (ms)
bool wifiConnected             = false;   // Flag: WiFi connection status
bool capturing                 = false;   // Flag: Image capture request state

// ===================== Utility & Helper Function Prototypes ===============
void logPrintln(const String& msg);
void wifiBlinkHandle();
void captureBlinkHandle();
void setupCamera();
void waitForCaptureCommand();

// ========================= Utility & Helper Functions =====================

/**
 * @brief   Print a message to Serial (log channel).
 * @param   msg  The message to print.
 */
void logPrintln(const String& msg) {
    Serial.println(msg);
}

/**
 * @brief   Blink the external LED non-blocking while WiFi is not connected.
 *          Useful for visual feedback during network initialization.
 */
void wifiBlinkHandle() {
    unsigned long now = millis();
    static int ledState = LOW;
    if (now - ledPreviousMillis >= ledBlinkInterval) {
        ledPreviousMillis = now;
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState);
    }
}

/**
 * @brief   Fast non-blocking blink of LED during active image capture.
 *          Visually signals that camera is busy capturing or processing.
 */
void captureBlinkHandle() {
    unsigned long now = millis();
    static int ledState = LOW;
    if (now - ledPreviousMillis >= ledCaptureInterval) {
        ledPreviousMillis = now;
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState);
    }
}

/**
 * @brief   Initialize the ESP32-CAM hardware with the correct configuration.
 *          - Sets frame size, quality, and format.
 *          - Maps all camera pins.
 *          - Halts with error if camera setup fails.
 */
void setupCamera() {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;       // 20 MHz external clock
    config.pixel_format = PIXFORMAT_JPEG; // JPEG output format
    config.frame_size = FRAMESIZE_UXGA;   // 1600x1200 pixels (adjust for RAM constraints)
    config.jpeg_quality = 10;             // JPEG quality (lower = better, 0-63)
    config.fb_count = 1;                  // Frame buffer count (1 = minimum RAM usage)

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        logPrintln("Camera initialization failed! System halted.");
        while (1); // Infinite loop on critical failure
    }
    logPrintln("Camera initialized successfully.");
}

/**
 * @brief   Reads text-based commands from Serial.
 *          When the command "capture" (case-insensitive) is received,
 *          sets the capturing flag. Handles line breaks and overflow.
 */
void waitForCaptureCommand() {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd == "capture") {capturing = true;} else {capturing = false;}
        cmd = "";
    }
}

// ============================ Arduino Setup =============================

/**
 * @brief Arduino setup routine.
 * - Initializes Serial interface.
 * - Configures status LEDs.
 * - Connects to WiFi network.
 * - Initializes camera hardware.
 * - Starts TCP server for image transmission.
 */
void setup() {
    // Initialize status LEDs (external and internal, if available)
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    pinMode(LED_BUILTIN_PIN, OUTPUT);
    digitalWrite(LED_BUILTIN_PIN, LOW);

    Serial.begin(115200); // USB-Serial debug/command channel
    delay(10);

    // Set device hostname for network identification
    WiFi.hostname(MyHostName);

    // Begin WiFi connection (station mode)
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    logPrintln("Connecting to WiFi...");
    while (WiFi.status() != WL_CONNECTED) {
        wifiBlinkHandle();
        delay(100);
    }
    wifiConnected = true;
    digitalWrite(LED_PIN, LOW);  // LED ON (solid) = WiFi connected
    logPrintln("WiFi connected! IP address: " + WiFi.localIP().toString());

    // Initialize camera
    setupCamera();

    // Start TCP server for image transfer
    imageServer.begin();
    logPrintln("Image TCP server started on port 5000.");
}

// ============================ Arduino Loop ==============================

/**
 * @brief Main program loop.
 * - Manages status LED for WiFi.
 * - Waits for and processes capture commands over Serial.
 * - Captures an image and streams it to a TCP client on port 5000.
 * - Ensures frame is always up-to-date by discarding previous buffers.
 */
void loop() {
    // If WiFi drops, keep blinking LED until reconnect
    if (!wifiConnected) {
        wifiBlinkHandle();
    }

    // Poll Serial for "capture" command
    waitForCaptureCommand();

    // If "capture" flag set, begin image acquisition and transfer process
    if (capturing) {
        logPrintln("Preparing to capture image...");

        // Provide fast blinking LED as busy indicator during capture
        unsigned long blinkStart = millis();
        const unsigned long blinkDuration = 1600; // ms for user feedback
        while (millis() - blinkStart < blinkDuration) {
            captureBlinkHandle();
        }
        digitalWrite(LED_PIN, LOW); // LED solid ON (ready for next)

        // Wait for TCP client to connect to image server (port 5000)
        logPrintln("Waiting for TCP client on port 5000...");
        imageClient = imageServer.accept();
        unsigned long waitTimeout = millis() + 10000; // 10 seconds timeout
        while (!imageClient && millis() < waitTimeout) {
            imageClient = imageServer.accept();
            delay(10);
        }
        if (!imageClient) {
            logPrintln("No TCP client connected. Capture aborted.");
            capturing = false;
            return;
        }
        logPrintln("TCP client connected. Capturing image...");

        // ======== IMPORTANT: Discard previous frames to ensure up-to-date image ========
        camera_fb_t* fb = NULL;
        for (int i = 0; i < 2; i++) {
            fb = esp_camera_fb_get();
            if (fb) esp_camera_fb_return(fb);
        }
        fb = esp_camera_fb_get(); // Now fb contains the latest image
        if (!fb) {
            logPrintln("Camera capture failed! Client disconnected.");
            imageClient.stop();
            capturing = false;
            return;
        }

        // ==== Send JPEG image size (uint32_t, little-endian) before JPEG data ====
        uint32_t img_len = fb->len;
        imageClient.write((uint8_t*)&img_len, sizeof(img_len));

        // ==== Send JPEG image data over TCP socket ====
        size_t bytes_sent = 0;
        while (bytes_sent < img_len) {
            size_t chunk = imageClient.write(fb->buf + bytes_sent, img_len - bytes_sent);
            if (chunk == 0) {
                delay(1);
                continue;
            }
            bytes_sent += chunk;
        }
        imageClient.clear();          // Ensure all data is sent before disconnect
        delay(50);                   // Short delay for network stability

        logPrintln("Image sent successfully. Bytes transmitted: " + String(img_len));
        esp_camera_fb_return(fb);    // Return frame buffer to driver for reuse
        imageClient.stop();          // Disconnect client after transfer
        logPrintln("TCP client disconnected.");
        capturing = false;

        // Set status LED ON to indicate device ready
        digitalWrite(LED_PIN, LOW);
    }

    // Optionally, keep built-in LED always OFF (if present)
    analogWrite(LED_BUILTIN_PIN, 25);
}
