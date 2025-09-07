#include <Arduino.h>
#include <ESP8266WiFi.h>  // Wi-Fi functionality for ESP8266

// ============================================================
//                    USER CONFIGURATION
// ============================================================

// Wi-Fi Credentials for Station Mode (STA)
const char* ssid      = "Hack";        // [WiFi SSID] Your network name
const char* password  = "by56by56";    // [WiFi Password]

// Hostname for this device on the network
const char* MyHostName = "EDUSAT TT&C";

// ============================================================
//                     TELNET SERVER SETUP
// ============================================================

// Create a Telnet server on port 23 (standard Telnet port)
WiFiServer telnetServer(23);

// Object to store the currently connected Telnet client
WiFiClient telnetClient;

// ============================================================
//                       ARDUINO SETUP
// ============================================================

/**
 * @brief Runs once on boot or reset.
 * Initializes Serial, Wi-Fi connection, and the Telnet server.
 */
void setup() {
  // ----- Serial Debug Initialization -----
  Serial.begin(115200);     // Start serial comms (baud 115200)
  delay(10);
  Serial.println();         // New line for clarity

  // ----- Hostname Assignment -----
  WiFi.hostname(MyHostName);   // Set custom network hostname

  // ----- Wi-Fi Connection (Station Mode) -----
  WiFi.mode(WIFI_STA);         // Set to Station mode (connect to WiFi)
  WiFi.begin(ssid, password);  // Start connecting to WiFi

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected successfully!");
  Serial.print("Device IP address:\t");
  Serial.println(WiFi.localIP());

  // ----- Telnet Server Initialization -----
  telnetServer.begin();           // Start listening for Telnet connections
  telnetServer.setNoDelay(true);  // Disable Nagle's buffering for low-latency
  Serial.println("Telnet server started on port 23.");
}

// ============================================================
//                        MAIN LOOP
// ============================================================

/**
 * @brief Runs continuously after setup().
 * Handles Telnet connections and bridges Serial <-> Telnet.
 */
void loop() {
  // ------------------------------------------------------------
  //      TELNET CLIENT CONNECTION MANAGEMENT
  // ------------------------------------------------------------

  // Handle new Telnet clients attempting to connect
  if (telnetServer.hasClient()) {
    if (telnetClient && telnetClient.connected()) {
      // Reject new clients if one is already connected
      WiFiClient newClient = telnetServer.accept();
      newClient.println("Only one client allowed!"); // Inform
      newClient.stop();                             // Disconnect
    } else {
      // No existing client: accept new client
      telnetClient = telnetServer.accept();
      telnetClient.println("Telnet connected!");
      Serial.println("Telnet client connected!");
    }
  }

  // ------------------------------------------------------------
  //      TELNET COMMAND HANDLING
  // ------------------------------------------------------------

  // Process incoming Telnet commands
  if (telnetClient && telnetClient.connected() && telnetClient.available()) {
    String cmd = telnetClient.readStringUntil('\n');  // Read until newline

    Serial.println(cmd);  // Show received command on Serial (debug)

    // Echo command back to the client
    telnetClient.print("Command received: ");
    telnetClient.println(cmd);

    // ----------- CUSTOM COMMAND PROCESSING HERE --------------
    // Parse and act on 'cmd' (e.g. control hardware)
    // Example: if (cmd == "led on") { digitalWrite(LED_PIN, HIGH); }
    // ---------------------------------------------------------
  }

  // ------------------------------------------------------------
  //      SERIAL TO TELNET BRIDGE (Serial Monitor Forwarding)
  // ------------------------------------------------------------

  // Forward any Serial data to the Telnet client (if connected)
  if (telnetClient && telnetClient.connected() && Serial.available()) {
    String telemetry = Serial.readStringUntil('\n');
    telnetClient.println(telemetry); // Send Serial input to Telnet
  }

  // ------------------------------------------------------------
  // Note: Keep loop() non-blocking for fast, responsive control!
  // ------------------------------------------------------------
}
