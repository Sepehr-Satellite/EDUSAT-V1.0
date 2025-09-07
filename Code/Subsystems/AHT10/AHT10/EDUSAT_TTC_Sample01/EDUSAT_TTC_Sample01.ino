#include <Arduino.h>
#include <ESP8266WiFi.h>  // Provides Wi-Fi capabilities for ESP8266

// ================== USER CONFIGURABLE PARAMETERS ==================

// Set the SSID (network name) and password for the Access Point (AP) mode
const char* ssid = "Sepehr EDUSAT TT&C v1001";   // AP SSID (Wi-Fi network name)
const char* password = "12345678";               // AP password. Set to "" (empty string) for open network

// Set the device's network hostname (useful for identification in LAN scans)
const char* MyHostName = "EDUSAT TT&C";

// ================== TELNET SERVER INITIALIZATION ==================

// Create a Telnet server object to listen on port 23 (standard Telnet port)
WiFiServer telnetServer(23);

// This client object will store the currently connected Telnet client
WiFiClient telnetClient;

/**
 * @brief Arduino standard setup function. Runs once after power-up or reset.
 * Sets up the ESP8266 as a Wi-Fi Access Point and starts the Telnet server.
 */
void setup() {
  // Initialize serial communication at 115200 baud (for debugging)
  Serial.begin(115200);
  delay(10);          // Brief pause to ensure Serial is ready
  Serial.println();   // Print empty line for readability

  // Set the device's network hostname (visible in DHCP clients list)
  WiFi.setHostname(MyHostName);

  // Attempt to start the ESP8266 in Access Point mode with the defined SSID & password
  if (!WiFi.softAP(ssid, password)) {
    Serial.println("Access Point creation failed.");  // Print error if failed
    while (1);   // Halt execution (infinite loop)
  }

  // Access Point started successfully. Show AP info on serial
  Serial.print("Access Point \"");
  Serial.print(ssid);
  Serial.println("\" started");

  // Print the IP address assigned to the AP interface
  Serial.print("IP address:\t");
  Serial.println(WiFi.softAPIP());

  // Initialize the Telnet server (for remote debug and control)
  telnetServer.begin();              // Start listening for clients
  telnetServer.setNoDelay(true);     // Send packets immediately, no TCP buffering
  Serial.println("Telnet server started on port 23."); // Confirmation message
}

/**
 * @brief Arduino main loop function. Repeats indefinitely.
 * Manages Telnet client connections and handles incoming Telnet commands.
 */
void loop() {
  // =========== TELNET CLIENT CONNECTION MANAGEMENT ===========

  // If a new client attempts to connect to the Telnet server...
  if (telnetServer.hasClient()) {
    // If a client is already connected, reject new client
    if (telnetClient && telnetClient.connected()) {
      // !! CHANGED: Use .accept() instead of deprecated .available()
      WiFiClient newClient = telnetServer.accept(); // Get new client object (modern API)
      newClient.println("Only one client allowed!");   // Notify client
      newClient.stop();                               // Close the connection
    } else {
      // No existing client: Accept the incoming client
      // !! CHANGED: Use .accept() instead of deprecated .available()
      telnetClient = telnetServer.accept();
      telnetClient.println("Telnet connected!");       // Greet client
      Serial.println("Telnet client connected!");      // Log the event
    }
  }

  // =========== TELNET COMMAND HANDLING ===========

  // If the Telnet client is connected and has sent data...
  if (telnetClient && telnetClient.connected() && telnetClient.available()) {
    // Read the incoming line (until newline character)
    String cmd = telnetClient.readStringUntil('\n');

    // Print the received command to Serial for debugging
    Serial.print("Received via Telnet: ");
    Serial.println(cmd);

    // (Optional) Echo the command back to the client
    telnetClient.print("Command received: ");
    telnetClient.println(cmd);

    // ========== ADD CUSTOM COMMAND PROCESSING BELOW ==========
    // You can parse the 'cmd' variable and perform actions based on received text.
    // For example: if (cmd == "led on") { digitalWrite(LED_PIN, HIGH); }
    // =========================================================
  }

  // Note: Loop repeats rapidly; avoid blocking operations here.
}