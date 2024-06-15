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

  // Initialize Serial1 for LiDAR communication using defined RX and TX pins
  // The TX pin is set to -1 to disable the TX functionality
  LD19Serial.begin(230400, SERIAL_8N1, LD19_RX_PIN, -1);  
  Serial.println("ESP32 LiDAR Data Processing and Forwarding");
  // Small delay to ensure serial communications are established
  delay(1000);  
}

void loop() {

  // Read and process commands from Serial
    PackageBuilderCRC();
    // Include a small delay to prevent overwhelming the Serial buffer
    delay(10);

}

//------------------------------------------------------------------------------



void RawData() {
  // Display raw packet data
  if (LD19Serial.available() >= sizeof(LiDARFrameTypeDef)) {
      uint8_t packet[sizeof(LiDARFrameTypeDef)];
      LD19Serial.readBytes(packet, sizeof(LiDARFrameTypeDef));
      for (size_t i = 0; i < sizeof(packet); i++) {
          if (packet[i] < 0x10) Serial.print('0'); // Print leading zero for hex values less than 0x10
          Serial.print(packet[i], HEX);
          Serial.print(" ");
      }
  }
  Serial.println();
}

void passthrough() {
  if (LD19Serial.available() >= 0) {
    uint8_t packet[sizeof(LiDARFrameTypeDef)];
    LD19Serial.readBytes(packet, sizeof(LiDARFrameTypeDef));
    for (size_t i = 0; i < sizeof(packet); i++) {
      if (packet[i] < 0x10) Serial.print('0'); // Print leading zero for hex values less than 0x10
      Serial.print(packet[i], HEX);
      Serial.print(" ");
    }
  } 
  Serial.println();

}





void NewPacketFinder() {
    static bool isHeaderFound = false;
    static bool isVerLenFound = false;
    static uint8_t previousByte = 0x00; // Initialize with a value that won't match the header

    while (LD19Serial.available()) {
        uint8_t incomingByte = LD19Serial.read();

        if (!isHeaderFound) {
            // Looking for the header byte
            if (incomingByte == 0x54) {
                // Header found, now looking for VerLen next
                isHeaderFound = true;
                previousByte = incomingByte; // Remember this byte
                continue; // Move to the next byte
            }
        } else if (isHeaderFound && !isVerLenFound) {
            // Header was found, now checking if this byte is the expected VerLen
            if (previousByte == 0x54 && incomingByte == 0x2C) {
                // Header and VerLen sequence found, this is the start of a new packet
                isVerLenFound = true;
                Serial.println(); // Start a new line for the new packet
                //Serial.print("New Packet: 54 2C "); // Indicate the start of a new packet
            } else {
                // Reset the flags if the sequence is broken
                isHeaderFound = false;
            }
            previousByte = incomingByte; // Remember the current byte for the next iteration
        } else {
            // Reset the flags as we've found a complete header + VerLen sequence
            isHeaderFound = false;
            isVerLenFound = false;
        }

        // If a complete sequence was found, reset for the next packet detection
        if (isHeaderFound && isVerLenFound) {
            isHeaderFound = false;
            isVerLenFound = false;
        }

        // Print other bytes normally
        if (incomingByte < 0x10) Serial.print("0");
        Serial.print(incomingByte, HEX);
        Serial.print(" ");
    }
}




void PackageBuilder2() {
  static bool isHeader = true; // Start with the assumption the first byte is a header for initial alignment
  if (LD19Serial.available()) {
    uint8_t incomingByte = LD19Serial.read();
    // Check if the incoming byte is a header, and if so, start a new line
    if (incomingByte == 0x54 && !isHeader) {
      Serial.println(); // End the previous line
      Serial.println("-------------------------------"); // Visual separator for clarity
      isHeader = true; // Next byte is a header, so we start a new line
    } else {
      isHeader = false; // Not a header, continue the current line
    }
    // Print the byte in HEX format
    if(incomingByte < 0x10) Serial.print("0"); // Print leading zero for single-digit hex values
    Serial.print(incomingByte, HEX);
    Serial.print(" "); // Space for readability
  
  }
  // If there's no data, we do not print "error" continuously to keep the output clean
  else {
      Serial.println("error");
      Serial.println("-------------------------------");
  }
}


void PackageBuilder() {
    static bool isBuilding = false; // Indicates if we are currently building a packet
    static uint8_t packetBuffer[47]; // Buffer to store incoming packet data, adjust size as needed
    static int bufferIndex = 0; // Current index in buffer

    while (LD19Serial.available()) {
        uint8_t incomingByte = LD19Serial.read();

        // Check if we are looking for a new packet start
        if (!isBuilding) {
            // Check if this byte is the start of a new packet header
            if (incomingByte == 0x54 && LD19Serial.peek() == 0x2C) {
                // Confirmed start of a new packet, so start building
                isBuilding = true;
                packetBuffer[bufferIndex++] = incomingByte; // Store the header
                continue; // Move to the next iteration to get the next byte
            }
        } else {
            // If we are building and the next byte is available
            packetBuffer[bufferIndex++] = incomingByte; // Store the current byte

            if (bufferIndex == 2 && incomingByte != 0x2C) {
                // If the second byte isn't 0x2C, reset and look for a new packet start
                isBuilding = false;
                bufferIndex = 0;
                continue;
            }

            if (bufferIndex >= sizeof(packetBuffer)) {
                // If the buffer is full, we assume the packet is complete
                isBuilding = false; // Ready to look for the next packet

                // Print the complete packet
                Serial.println(); // New line for clarity
                Serial.println("Complete Packet:");
                for (int i = 0; i < sizeof(packetBuffer); i++) {
                    if (packetBuffer[i] < 0x10) Serial.print("0"); // Leading zero for single-digit hex
                    Serial.print(packetBuffer[i], HEX);
                    Serial.print(" ");
                }
                Serial.println();
                Serial.println("-------------------------------");

                // Reset for next packet
                bufferIndex = 0;
            }
        }
    }
}
void PackageBuilderCRC() {
    static bool isBuilding = false;
    static uint8_t packetBuffer[47]; // Adjust for your packet size + CRC byte
    static int bufferIndex = 0;

    while (LD19Serial.available()) {
        uint8_t incomingByte = LD19Serial.read();

        if (!isBuilding) {
            if (incomingByte == 0x54 && LD19Serial.peek() == 0x2C) {
                isBuilding = true;
                packetBuffer[bufferIndex++] = incomingByte;
                continue;
            }
        } else {
            packetBuffer[bufferIndex++] = incomingByte;
            if (bufferIndex == 2 && incomingByte != 0x2C) {
                isBuilding = false;
                bufferIndex = 0;
                continue;
            }

            if (bufferIndex >= sizeof(packetBuffer)) {
                isBuilding = false;
                bufferIndex = 0;

                // Calculate CRC of received packet (excluding the CRC byte itself)
                uint8_t calculatedCRC = CalCRC8(packetBuffer, sizeof(packetBuffer) - 1);
                // Get the CRC byte from the packet (last byte)
                uint8_t receivedCRC = packetBuffer[sizeof(packetBuffer) - 1];

                if (calculatedCRC == receivedCRC) {
                    // Packet is valid, print or process it
                    Serial.println("\nComplete and Valid Packet:");
                    for (int i = 0; i < sizeof(packetBuffer); i++) {
                        if (packetBuffer[i] < 0x10) Serial.print("0");
                        Serial.print(packetBuffer[i], HEX);
                        Serial.print(" ");
                    }
                    Serial.println("\n-------------------------------");
                    printPacketHexDetails(packetBuffer);

                } else {
                    // CRC mismatch, packet invalid
                    Serial.println("\nCRC Mismatch - Packet Invalid\n-------------------------------");
                }
            }
        }
    }
}

void passthrough4() {
if (LD19Serial.available() >= sizeof(LiDARFrameTypeDef)) {
    LiDARFrameTypeDef packet;
    LD19Serial.readBytes((char*)&packet, sizeof(LiDARFrameTypeDef));

    // Forward the entire packet to Serial
    Serial.write((const uint8_t*)&packet, sizeof(LiDARFrameTypeDef));

    // Additional newline for clarity in the Serial Monitor
    Serial.println();
    Serial.println("-------------------------------");
  }
  else {
      Serial.println("error");
      Serial.println("-------------------------------");

  }
}




void data() {
  if (LD19Serial.available() >= sizeof(LiDARFrameTypeDef)) {
    LiDARFrameTypeDef packet;
    LD19Serial.readBytes(reinterpret_cast<char*>(&packet), sizeof(LiDARFrameTypeDef));

    if (packet.header == HEADER) {
      Serial.print("Header: 0x"); Serial.println(packet.header, HEX);
      Serial.print("VerLen: 0x"); Serial.println(packet.ver_len, HEX);
      Serial.print("Speed: "); Serial.println(packet.speed);
      Serial.print("Start Angle: "); Serial.println(packet.start_angle);
      
      Serial.println("Data Points:");
      for (int i = 0; i < POINT_PER_PACK; i++) {
        Serial.print("Point "); Serial.print(i + 1); Serial.print(": ");
        Serial.print("0x"); Serial.print(packet.point[i].distance, HEX);
        Serial.print(", Intensity: "); Serial.println(packet.point[i].intensity);
      }
      
      Serial.print("End Angle: "); Serial.println(packet.end_angle);
      Serial.print("Timestamp: "); Serial.println(packet.timestamp);
      Serial.print("CRC: 0x"); Serial.println(packet.crc8, HEX);
      Serial.println("-------------------------------");
    }
  }
  else{
    // If not enough data is available, you might want to handle this differently.
    // For now, it's simply reporting an error.
    Serial.println("Waiting for complete packet...");
    Serial.println("-------------------------------");
  }
}

void BufferInspection() {
  Serial.print("Available Bytes: ");
  Serial.println(LD19Serial.available());
  delay(1000); // Just for testing, adds a 1-second delay between checks
}




void parsing() {
  if (LD19Serial.available() >= sizeof(LiDARFrameTypeDef)) {
    LiDARFrameTypeDef packet;
    LD19Serial.readBytes(reinterpret_cast<char*>(&packet), sizeof(LiDARFrameTypeDef));

    if (packet.header == HEADER) {
      Serial.print("Header: 0x"); Serial.println(packet.header, HEX);
      Serial.print("VerLen: 0x"); Serial.println(packet.ver_len, HEX);
      Serial.print("Speed: "); Serial.println(packet.speed);
      Serial.print("Start Angle: "); Serial.println(packet.start_angle);
      
      Serial.println("Data Points:");
      for (int i = 0; i < POINT_PER_PACK; i++) {
        Serial.print("Point "); Serial.print(i + 1); Serial.print(": ");
        Serial.print("0x"); Serial.print(packet.point[i].distance, HEX);
        Serial.print(", Intensity: "); Serial.println(packet.point[i].intensity);
      }
      
      Serial.print("End Angle: "); Serial.println(packet.end_angle);
      Serial.print("Timestamp: "); Serial.println(packet.timestamp);
      Serial.print("CRC: 0x"); Serial.println(packet.crc8, HEX);
      Serial.println("-------------------------------");
    }
  }
  else{
    // If not enough data is available, you might want to handle this differently.
    // For now, it's simply reporting an error.
    Serial.println("Waiting for complete packet...");
    Serial.println("-------------------------------");
  }
}

//-----------------

void realTimeParsing() {
    if (LD19Serial.available() >= sizeof(LiDARFrameTypeDef)) {
        // Read data only if available from LiDAR
        uint8_t packet[sizeof(LiDARFrameTypeDef)];
        size_t bytesRead = LD19Serial.readBytes(packet, sizeof(LiDARFrameTypeDef));

        LD19Serial.readBytes(reinterpret_cast<char*>(packet), sizeof(LiDARFrameTypeDef));

        // Debugging: Check if the correct amount of bytes are read
        Serial.print("Bytes read: ");
        Serial.println(bytesRead);

        if (bytesRead == sizeof(LiDARFrameTypeDef)) {
            // Process packet only if fully received
            printPacketHexDetails(packet);
            // parseLiDARPacket(packet);
        } else {
            Serial.println("Incomplete packet received.");
        }

    }
}

// Function to test displaying packet details with a hard-coded example
void testExampleDataDetail() {
    // This example packet is predefined with specific values for testing purposes.
    // It mimics a real packet from the LiDAR sensor to validate the parsing logic.
    uint8_t exampleData[] = {
        0x54, 0x2C, 0x68, 0x08, 0xAB, 0x7E, 0xE0, 0x00, 0xE4, 0xDC, 0x00, 0xE2, 
        0xD9, 0x00, 0xE5, 0xD5, 0x00, 0xE3, 0xD3, 0x00, 0xE4, 0xD0, 0x00, 0xE9, 
        0xCD, 0x00, 0xE4, 0xCA, 0x00, 0xE2, 0xC7, 0x00, 0xE9, 0xC5, 0x00, 0xE5, 
        0xC2, 0x00, 0xE5, 0xB0, 0x00, 0xEA, 0xBE, 0x82, 0x3A, 0x1A, 0x50
    };
    Serial.println("Testing with example data packet:");
    printPacketHexDetails(exampleData);
    parseLiDARPacket(exampleData);
}

// Function to print the details of a LiDAR packet in hexadecimal format
void printPacketHexDetails(const uint8_t* packet) {
    // Cast the byte array to a structured LiDAR packet for easier access to its fields.
    const LiDARFrameTypeDef* frame = reinterpret_cast<const LiDARFrameTypeDef*>(packet);
    
    // Uncomment the CRC check if CalCRC8 function is defined and you wish to use it.
    /*
    uint8_t calculatedCRC = CalCRC8(const_cast<uint8_t*>(packet), sizeof(LiDARFrameTypeDef) - 1);
    if (calculatedCRC != frame->crc8) {
        Serial.println("CRC check failed.");
        return;
    }
    */

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

    // For a little-endian system parsing a little-endian sensor output:
    uint16_t rawStartAngle = frame->start_angle;
    float startAngleDegrees = rawStartAngle * 0.1f; // Convert from tenths of a degree to degrees

    // Ensure startAngleDegrees is within 0-360 degrees if needed
    if (startAngleDegrees >= 360.0f) {
        startAngleDegrees -= 360.0f;
    }

    // Loop through each measurement point and print its distance and intensity.
    Serial.println("Data Points:");
    float step = (endAngle - startAngle) / (POINT_PER_PACK - 1);
    for (int i = 0; i < POINT_PER_PACK; i++) {
        // Preparing to print each measurement's distance in little-endian format.
        Serial.print("Point ");
        Serial.print(i + 1);
        // Printing the bytes in the correct order for human readability.
                    
        float angle = startAngle + step * i;
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



void parseLiDARPacket(const uint8_t* packet) {
    const LiDARFrameTypeDef* frame = reinterpret_cast<const LiDARFrameTypeDef*>(packet);
    

    if (frame->header == HEADER) {

        // Displaying start and end angles both as raw values and in HEX, converted to degrees
        float startAngle = frame->start_angle * 0.01;
        float endAngle = frame->end_angle * 0.01;

        // Calculating and displaying distance and intensity for each point
        Serial.println("Data Points:");
        float step = (endAngle - startAngle) / (POINT_PER_PACK - 1);
        for (int i = 0; i < POINT_PER_PACK; i++) {
            float angle = startAngle + step * i;
            uint16_t distance = frame->point[i].distance;
            uint8_t intensity = frame->point[i].intensity;

            Serial.print("Point "); 
            Serial.print(i + 1);
            Serial.print(": Angle = "); 
            Serial.print(angle, 2);
            Serial.print(" degrees, Distance = ");
            Serial.print(distance); 
            Serial.print("mm, Intensity = ");
            Serial.println(intensity); 
        }

        Serial.println("-------------------------------");
    } else {
        Serial.println("Invalid packet header.");
    }
}
