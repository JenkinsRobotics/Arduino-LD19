/**
 * LiDAR Package Finder Demonstration
 * -----------------------------------
 *
 * This code demonstrates the detection and processing of data packets from a LiDAR sensor,
 * specifically tailored for the LD19 model. It captures data packets over a hardware serial
 * connection from the LiDAR sensor and identifies the start of new packets based on specific
 * byte sequences. Once a packet is detected, it can be processed or passed through for further
 * action, such as display or analysis.
 *
 * LiDAR Information:
 * - Model: LiDAR Sensor LD19
 * - Purpose: This setup is used to demonstrate the method of identifying and segregating
 *   data packets transmitted by the LD19 LiDAR sensor for applications like mapping,
 *   object detection, or distance measurement.
 * - Operating Principle: The LD19 sensor operates on a time-of-flight principle, measuring
 *   the return time of a laser light to calculate distances.
 *
 * Serial Settings:
 * - Baud Rate: 230400 bits per second
 *   This high baud rate is selected to manage the rapid data rate needed to capture the frequent
 *   data packets emitted by the LD19 sensor.
 * - Data Bits: 8
 *   Each byte of data consists of 8 bits, which is standard for serial data communication.
 * - Stop Bits: 1
 *   Each data packet ends with a single stop bit to signify the end of the data transmission.
 * - Parity: None
 *   No parity bit is used, simplifying the communication protocol and reducing the data overhead.
 * - Flow Control: None
 *   The system does not use hardware or software flow control; it relies on the buffer management
 *   capabilities of the ESP32 and the timing of the LiDAR sensor to handle data flow.
 *
 * Packet Detection Logic:
 * - Packet Header (0x54) and VerLen (0x2C) Detection:
 *   This code continuously monitors the serial stream and detects the sequence `0x54 0x2C`, which
 *   marks the beginning of a new packet according to the LD19 protocol. Upon detecting this sequence,
 *   it assumes that a new packet is starting and begins to process subsequent bytes as part of this packet.
 *
 * Note: This example is part of a library designed to facilitate the use of the LD19 LiDAR sensor with
 * ESP32 or similar microcontrollers. It focuses on raw data acquisition and packet detection, setting the
 * stage for more complex processing in other modules of the library.
 */




#include <Arduino.h>
#include <HardwareSerial.h>

// Define RX and TX pins for the LiDAR connection on Serial1
#define LD19_RX_PIN 32 // RX pin to receive data from LiDAR
#define LD19_TX_PIN 33 // TX pin for LiDAR, not used in read-only mode

// Initialize Serial1 for LiDAR communication
HardwareSerial LD19Serial(1); // Use hardware serial 1 for LiDAR

// Constants for packet parsing
#define POINT_PER_PACK 12
#define HEADER 0x54

// Structure for individual LiDAR measurement points
// Each point includes a distance (14 bits) and an intensity (8 bits)
typedef struct __attribute__((packed)) {
  uint16_t distance ;
  uint8_t intensity;
} LidarPointStructDef;

// Structure for the entire LiDAR data packet
// Includes header, version, speed, angles, measurement points, timestamp, and CRC
typedef struct __attribute__((packed)) {
  uint8_t header;
  uint8_t ver_len;
  uint16_t speed;
  uint16_t start_angle;
  LidarPointStructDef point[POINT_PER_PACK];  // Array of measurement points
  uint16_t end_angle;
  uint16_t timestamp;
  uint8_t crc8;
} LiDARFrameTypeDef;

static const uint8_t CrcTable[256] = {
  // Your CRC table values here...
  0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3,
  0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33,
  0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8,
  0xf5, 0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77,
  0x3a, 0x94, 0xd9, 0x0e, 0x43, 0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55,
  0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4,
  0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f,
  0x62, 0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff,
  0xb2, 0x1c, 0x51, 0x86, 0xcb, 0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2,
  0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12,
  0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99,
  0xd4, 0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14,
  0x59, 0xf7, 0xba, 0x6d, 0x20, 0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36,
  0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9,
  0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72,
  0x3f, 0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2,
  0xef, 0x41, 0x0c, 0xdb, 0x96, 0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1,
  0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71,
  0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa,
  0xb7, 0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35,
  0x78, 0xd6, 0x9b, 0x4c, 0x01, 0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17,
  0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
};

uint8_t CalCRC8(uint8_t *p, uint8_t len) {
  uint8_t crc = 0;
  for (uint16_t i = 0; i < len; i++) {
    crc = CrcTable[(crc ^ *p++) & 0xff];
  }
  return crc;
}

void setup() {
  Serial.begin(230400);  // Initialize USB serial communication with PC
  // The TX pin is set to -1 to disable the TX functionality
  LD19Serial.begin(230400, SERIAL_8N1, LD19_RX_PIN, -1);  
  Serial.println("ESP32 LiDAR Data Processing and Forwarding");
  // Small delay to ensure serial communications are established
  delay(1000);  
}

void loop() {

  // Read and process commands from Serial
    NewPacketFinder();
    // Include a small delay to prevent overwhelming the Serial buffer
    delay(1000);

}

//------------------------------------------------------------------------------
void NewPacketFinder() {
    static bool isBuilding = false; // Flag to indicate if a packet is currently being assembled
    static uint8_t packetBuffer[47]; // Buffer to store incoming packet bytes, size based on expected packet length
    static int bufferIndex = 0; // Index for the current position in the buffer

    while (LD19Serial.available()) {
        uint8_t incomingByte = LD19Serial.read(); // Read the next byte from the serial buffer

        // Check if we are currently assembling a packet
        if (!isBuilding) {
            // Look for the packet start, which is defined by the header byte 0x54
            if (incomingByte == 0x54) {
                // Check the next byte without removing it from the serial buffer
                if (LD19Serial.peek() == 0x2C) { // Verify if the next byte is 0x2C (VerLen)
                    packetBuffer[bufferIndex++] = incomingByte; // Store the header byte in the buffer
                    isBuilding = true; // Start building the packet
                    continue; // Skip to the next loop iteration to process the next byte immediately
                }
            }
        } else {
            // Continue building the packet by storing each subsequent byte
            packetBuffer[bufferIndex++] = incomingByte;

            // Ensure the second byte matches the expected 0x2C value to confirm a valid start
            if (bufferIndex == 2 && incomingByte != 0x2C) {
                // If not, reset the buffer and stop building the current packet
                isBuilding = false;
                bufferIndex = 0;
                continue;
            }

            // Check if the packet buffer is filled indicating potential completion of a packet
            if (bufferIndex >= sizeof(packetBuffer)) {
                // Reset to start detection of a new packet
                isBuilding = false;

                // Print the completed packet in hex format
                Serial.println(); // Start a new line for clarity
                for (int i = 0; i < bufferIndex; i++) {
                    if (packetBuffer[i] < 0x10) Serial.print("0"); // Leading zero for hex digits less than 0x10
                    Serial.print(packetBuffer[i], HEX);
                    Serial.print(" ");
                }
                Serial.println("\n-------------------------------");

                bufferIndex = 0; // Reset the buffer index for the next packet
            }
        }
    }
}
