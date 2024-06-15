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

// Packet structure and CRC calculation


volatile bool liveViewActive = true; // Controls the live data stream
bool displayRawData = true; // Add this line - Controls whether to display raw data
const String pauseCommand = "pause";
const String resumeCommand = "resume";
const String sendTestCommand = "sendtest";
const String rawDataCommand = "RawData"; // New command for raw data view
const String parseDataCommand = "ParseData"; // New command for parsed data view


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
  // Configure PWM functionalities
  // The TX pin is set to -1 to disable the TX functionality
  LD19Serial.begin(230400, SERIAL_8N1, LD19_RX_PIN, -1);  
  Serial.println("ESP32 LiDAR Data Processing and Forwarding");
  // Small delay to ensure serial communications are established
  delay(1000);  
}

void loop() {
  PackageBuilderCRC();
  // Include a small delay to prevent overwhelming the Serial buffer
    delay(10);
}

//------------------------------------------------------------------------------



void PackageBuilderCRC() {
  static bool isBuilding = false;
  static uint8_t packetBuffer[47];
  static int bufferIndex = 0;

  while (LD19Serial.available()) {
    uint8_t incomingByte = LD19Serial.read();

    if (!isBuilding && incomingByte == HEADER) {
      if (LD19Serial.peek() == 0x2C) {
        packetBuffer[bufferIndex++] = incomingByte;
        isBuilding = true;
        continue;
      }
    } else if (isBuilding) {
      packetBuffer[bufferIndex++] = incomingByte;

      if (bufferIndex == 2 && incomingByte != 0x2C) {
        isBuilding = false;
        bufferIndex = 0;
        continue;
      }

      if (bufferIndex >= sizeof(packetBuffer)) {
        isBuilding = false;
        uint8_t crc = CalCRC8(packetBuffer, bufferIndex - 1); // Exclude the CRC byte itself
        if (crc == packetBuffer[bufferIndex - 1]) {
          Serial.println("CRC check passed!");
          // Process packet her
          Serial.println("\n-------------------------------");
          printPacketHexDetails(packetBuffer);
        } else {
          Serial.println("CRC check failed!");
        }

        Serial.println(); 
        for (int i = 0; i < bufferIndex; i++) {
          if (packetBuffer[i] < 0x10) Serial.print("0");
          Serial.print(packetBuffer[i], HEX);
          Serial.print(" ");
        }
        Serial.println("\n-------------------------------");

        bufferIndex = 0;
      }
    }
  }
}





//  UPDATED function to correctly wrap around from 355 to 5 degrees through 360 degrees. 
// Function to print the details of a LiDAR packet in hexadecimal format
void printPacketHexDetails(const uint8_t* packet) {
    // Cast the byte array to a structured LiDAR packet for easier access to its fields.
    const LiDARFrameTypeDef* frame = reinterpret_cast<const LiDARFrameTypeDef*>(packet);

    // Header indicates the start of a new data packet.
    Serial.print("Header: 0x");
    Serial.println(frame->header, HEX);

    // VerLen combines version and length info for the packet.
    Serial.print("VerLen: 0x");
    Serial.println(frame->ver_len, HEX);

    // Speed of the LiDAR rotor.
    uint16_t speed = frame->speed;
    Serial.print("Speed: 0x");
    Serial.print(frame->speed, HEX);
    Serial.print(" (");
    Serial.print(speed);
    Serial.println(" degrees per second)");

    // Start angle for the first measurement in this packet.
    float startAngle = frame->start_angle * 0.01;
    Serial.print("Start Angle: 0x");
    Serial.print(frame->start_angle, HEX);
    Serial.print(" (");
    Serial.print(startAngle, 2);
    Serial.println(" degrees)");

    // End angle for the last measurement in this packet.
    float endAngle = frame->end_angle * 0.01;
    Serial.print("End Angle: 0x");
    Serial.print(frame->end_angle, HEX);
    Serial.print(" (");
    Serial.print(endAngle, 2);
    Serial.println(" degrees)");

   // Calculate step size, considering wrapping from 360 to 0 degrees
    float step = ((endAngle < startAngle) ? (360 - startAngle + endAngle) : (endAngle - startAngle)) / (POINT_PER_PACK - 1);

    // Loop through each measurement point and print its distance and intensity.
    Serial.println("Data Points:");
    for (int i = 0; i < POINT_PER_PACK; i++) {
        float angle = startAngle + step * i;
        if (angle >= 360) angle -= 360; // Wrap around if the angle exceeds 360 degrees
       
        Serial.print("Point ");
        Serial.print(i + 1);
        Serial.print(": Angle = "); 
        Serial.print(angle, 2);
        Serial.print(" degrees,");

        uint16_t distance = frame->point[i].distance;
        uint8_t highByte = (distance >> 8) & 0xFF;
        uint8_t lowByte = distance & 0xFF;
        Serial.print(" Distance = 0x");
        if (highByte < 0x10) Serial.print("0");
        Serial.print(highByte, HEX);
        if (lowByte < 0x10) Serial.print("0");
        Serial.print(lowByte, HEX);
        Serial.print(" ("); 
        //Serial.print(distance, HEX);
        Serial.print(distance); 
        Serial.print("mm),");

        // Intensity of the measurement point.
        uint8_t intensity = frame->point[i].intensity;
        Serial.print(" Intensity = 0x");
        if (intensity < 0x10) Serial.print("0");
        Serial.print(intensity, HEX);
        Serial.print(" ("); 
        Serial.print(intensity); 
        Serial.println(")");

    }

    // Timestamp indicating when this packet was generated.
    Serial.print("Timestamp: 0x");
    Serial.print(frame->timestamp, HEX);
    Serial.print(" (");
    Serial.print(frame->timestamp);
    Serial.println(" milliseconds)");
    
    // CRC for validating the integrity of the packet.
    Serial.print("CRC: 0x");
    Serial.println(frame->crc8, HEX);
    Serial.println("-------------------------------");
}


