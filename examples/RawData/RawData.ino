/**
 * LiDAR Data Processing and Serial Pass-through
 * ---------------------------------------------
 * 
 * This code is designed to interface with a specific LiDAR sensor, capturing data packets
 * transmitted over a hardware serial connection and forwarding them for processing and monitoring.
 * It also provides functionality to directly pass data through to a PC via the ESP32's USB connection.
 * 
 * LiDAR Information:
 * - Model: LiDAR Sensor LD19
 * - Purpose: Used to measure distances and intensities for [specific application or purpose].
 * - Operating Principle: [Briefly describe how the LiDAR works, e.g., time-of-flight measurement].
 * 
 * Serial Settings:
 * - Baud Rate: 230400 bits per second
 *   This high baud rate is necessary to accommodate the rapid transmission of data packets from the LiDAR sensor.
 * - Data Bits: 8
 *   Each byte of data consists of 8 bits.
 * - Stop Bits: 1
 *   Each data packet ends with a single stop bit to signify the end of the packet.
 * - Parity: None
 *   No parity bit is used in the data transmission, simplifying data parsing.
 * - Flow Control: None
 *   Flow control mechanisms are not employed, relying on the ESP32 and LiDAR sensor's processing capabilities.
 * 
 * LiDAR Data Packet Format:
 * - Header (1 Byte): Fixed at 0x54, indicating the start of a new data packet.
 * - VerLen (1 Byte): Upper three bits indicate the packet type; lower five bits specify the number of measurement points (12).
 * - Speed (2 Bytes): Indicates the rotation speed of the LiDAR sensor in degrees per second.
 * - Start Angle (2 Bytes): Starting angle of the data packet's measurement points, in units of 0.01 degrees.
 * - Measurement Data (36 Bytes): Contains 12 measurement points, each point represented by 3 bytes.
 * - End Angle (2 Bytes): Ending angle of the data packet's measurement points, in units of 0.01 degrees.
 * - Timestamp (2 Bytes): Specifies the time at which the data packet was sent, in milliseconds.
 * - CRC (1 Byte): A checksum for verifying the integrity of the received data packet.
 * 
 * Note: The detailed interpretation of each measurement point within the 'Measurement Data' section
 * requires further parsing to extract distance and intensity information for each point.
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

// Packet structure and CRC calculation
static const uint8_t CrcTable[256] = {
  // Your CRC table values here...

};

uint8_t CalCRC8(uint8_t *p, uint8_t len) {
  uint8_t crc = 0;
  for (uint16_t i = 0; i < len; i++) {
    crc = CrcTable[(crc ^ *p++) & 0xff];
  }
  return crc;
}

void setup() {
  Serial.begin(230400); // Initialize USB serial communication with PC
  LD19Serial.begin(230400, SERIAL_8N1, LD19_RX_PIN, -1); // Initialize Serial1 for LiDAR communication
  Serial.println("LD19 LiDAR data reading started.");
  delay(1000); // Small delay to ensure serial communications are established
}

void loop() {

    //RawData();
    passthrough();
    // Include a small delay to prevent overwhelming the Serial buffer
    delay(10);

}

//------------------------------------------------------------------------------


/**
 * RawData - Display incoming raw data bytes
 * 
 * This function reads bytes directly from the LiDAR sensor's data stream and prints
 * them to the serial console in hexadecimal format. Each byte is displayed as it is received,
 * providing a continuous stream of raw sensor data.
 */
void RawData() {
  if (LD19Serial.available()) {
    uint8_t incomingByte = LD19Serial.read();
    if (incomingByte < 0x10) Serial.print('0'); // Print leading zero for single-digit hex values
    Serial.print(incomingByte, HEX);
    Serial.print(" "); // Space for readability
  }
}


/**
 * passthrough - Display Full Non-Filtered Data Packets
 * 
 * This function demonstrates how to collect and display full, non-filtered packets of raw data from the LD19 LiDAR sensor.
 * It reads a predefined number of bytes, equivalent to the size of one complete data packet structure, directly from the sensor's data stream.
 * Each packet is then printed in hexadecimal format to the serial console.
 * 
 * The packets are displayed as received without any processing or filtering, providing a direct view of the raw data output from the sensor.
 * This method is particularly useful for understanding the structure and sequence of data packets and for verifying the integrity of data transmission before implementing more complex parsing or processing operations.
 */

void passthrough() {
  if (LD19Serial.available() >= sizeof(LiDARFrameTypeDef)) { // Check if the buffer has enough data for one complete packet
    uint8_t packet[sizeof(LiDARFrameTypeDef)]; // Buffer to hold one packet of data
    LD19Serial.readBytes(packet, sizeof(LiDARFrameTypeDef)); // Read bytes from serial to fill the packet
    Serial.println("\Concept Packet:");
    for (size_t i = 0; i < sizeof(packet); i++) {
      if (packet[i] < 0x10) Serial.print('0'); // Print leading zero for single-digit hex values
      Serial.print(packet[i], HEX);
      Serial.print(" "); // Space for readability
    }
    Serial.println("\n-------------------------------"); // End of packet visual separator
  }
}

