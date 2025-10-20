#include <HardwareSerial.h>  // For Serial1 on Arduino boards that support it

// Define the union for float to byte conversion
union FloatUnion {
  float f;
  uint8_t bytes[4];
};

// Global arrays to store the parsed values for the three SS
float ss_alpha[3] = {0.0f, 0.0f, 0.0f};
float ss_beta[3] = {0.0f, 0.0f, 0.0f};
float ss_gamma[3] = {0.0f, 0.0f, 0.0f};
float ss_max_lum[3] = {0.0f, 0.0f, 0.0f};

// Function to parse the received data for a specific SS id
void SS_Parse(uint8_t id, uint8_t buffer[], uint8_t buf_len) {
  // Parse the received data from the buffer
  // Packet format: header (4 bytes: 88,88,88,88), data (16 bytes: 4 floats), footer (4 bytes: 88,88,13,10)
  // Total packet size: 24 bytes
  // Data is in little-endian format
  uint8_t data[16] = {0};  // 4 floats * 4 bytes
  for (int ii = 0; ii <= buf_len - 24; ii++) {  // Room for full 24 bytes packet
    if (buffer[ii] == 88 && buffer[ii + 1] == 88 && buffer[ii + 2] == 88 && buffer[ii + 3] == 88 &&  // Header: XXXX
        buffer[ii + 20] == 88 && buffer[ii + 21] == 88 && buffer[ii + 22] == 13 && buffer[ii + 23] == 10) {  // Footer: XX\r\n
      // Found the full packet with header and footer
      // Extract the data between header and footer: buffer[ii + 4] to buffer[ii + 19]
      for (int jj = 0; jj < 16; jj++) {
        data[jj] = buffer[ii + 4 + jj];
      }
      // Parse the 16 bytes into 4 floats
      for (int j = 0; j < 4; j++) {
        FloatUnion fu;
        fu.f = 0.0f;
        // Little-endian:
        for (int k = 0; k < 4; k++) {
          fu.bytes[k] = data[j * 4 + k];
        }
        // If big-endian needed, reverse: fu.bytes[k] = data[j * 4 + (3 - k)];
        switch (j) {
          case 0: ss_alpha[id] = fu.f; break;
          case 1: ss_beta[id] = fu.f; break;
          case 2: ss_gamma[id] = fu.f; break;
          case 3: ss_max_lum[id] = fu.f; break;
          default: break;
        }
      }
      return;  // Assume one packet per buffer
    }
  }
}

void setup() {
  // Initialize Serial (to computer) and Serial1 (to sensor device)
  Serial.begin(9600);   // Higher baud rate for faster communication
  Serial1.begin(9600);  // Match baud rate to sensor device (adjust if needed)
  Serial1.setTimeout(50);  // Short timeout for readBytes
  while (!Serial) {}    // Wait for Serial to initialize
  while (!Serial1) {}   // Wait for Serial1 to initialize
}

void loop() {
  // For each SS id (0 to 2), request data by sending the id, receive fixed packet, parse
  for (uint8_t id = 0; id < 3; id++) {
    // Send the id to the sensor device to request data for that SS
    Serial1.write(id);
    
    // Minimal delay to allow the device to start responding
    delay(10);
    
    // Read exactly 24 bytes for the packet
    uint8_t buffer[24] = {0};
    size_t bytesRead = Serial1.readBytes(buffer, 24);
    
    if (bytesRead == 24) {
      // Parse the buffer for this id
      SS_Parse(id, buffer, 24);
    }
  }
  
  // After collecting data for all three SS, send to computer in the specified text format
  Serial.print("\n\n======================== SS ========================= \r\n");
  Serial.print("SS: Alpha: ");
  Serial.print(ss_alpha[0], 6);
  Serial.print(", ");
  Serial.print(ss_alpha[1], 6);
  Serial.print(", ");
  Serial.print(ss_alpha[2], 6);
  Serial.print(" \r\n");
  
  Serial.print("SS: Beta: ");
  Serial.print(ss_beta[0], 6);
  Serial.print(", ");
  Serial.print(ss_beta[1], 6);
  Serial.print(", ");
  Serial.print(ss_beta[2], 6);
  Serial.print(" \r\n");
  
  Serial.print("SS: Gamma: ");
  Serial.print(ss_gamma[0], 6);
  Serial.print(", ");
  Serial.print(ss_gamma[1], 6);
  Serial.print(", ");
  Serial.print(ss_gamma[2], 6);
  Serial.print(" \r\n");
  
  Serial.print("SS: Max Lum: ");
  Serial.print(ss_max_lum[0], 6);
  Serial.print(", ");
  Serial.print(ss_max_lum[1], 6);
  Serial.print(", ");
  Serial.print(ss_max_lum[2], 6);
  Serial.print(" \r\n");
  
  Serial.print("------------------------------------------------------ \r\n");
  
  // Shorter delay before next cycle
  delay(100);
}