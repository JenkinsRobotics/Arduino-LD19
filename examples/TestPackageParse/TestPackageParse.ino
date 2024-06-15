/**
 * LD19 LiDAR Data Processing and Serial Pass-through
 * --------------------------------------------------
 * This Arduino sketch is designed to interface with the LD19 LiDAR sensor, capturing data packets
 * transmitted over a hardware serial connection and displaying them directly. This setup allows for
 * monitoring the sensor output in real time as well as for debugging and development purposes.
 *
 * The sketch facilitates direct data pass-through to a PC using the ESP32's USB connection, making it 
 * easier to visualize and interpret the raw data coming from the sensor.
 *
 * LiDAR Sensor Information:
 * - Model: LD19
 * - Purpose: To measure distances and intensities, typically used in mapping, robotics, and environmental scanning.
 * - Operating Principle: Uses time-of-flight measurement to determine the distance from the sensor to an object.
 *
 * Serial Communication Settings:
 * - Baud Rate: 230400 bps (necessary to handle the high data throughput of the sensor)
 * - Data Bits: 8 (standard byte size for data transmission)
 * - Stop Bits: 1 (indicates the end of a data packet)
 * - Parity: None (simplifies the data format by eliminating parity checking)
 * - Flow Control: None (relies on the robustness of the ESP32 and sensor processing to manage data flow)
 *
 * LiDAR Data Packet Structure:
 * - Header (1 Byte): 0x54, marks the beginning of a new data packet.
 * - VerLen (1 Byte): Indicates the packet version and length. The upper three bits represent the packet type, 
 *                    while the lower five bits specify the number of measurement points (12).
 * - Speed (2 Bytes): Rotation speed of the LiDAR sensor, expressed in degrees per second.
 * - Start Angle (2 Bytes): Initial angle of measurement from the sensor, scaled by 0.01 degrees.
 * - Measurement Data (36 Bytes): Consists of 12 sets of measurement points, with each set occupying 3 bytes.
 * - End Angle (2 Bytes): Final angle of measurement, also scaled by 0.01 degrees.
 * - Timestamp (2 Bytes): Time at which the packet was sent, measured in milliseconds.
 * - CRC (1 Byte): Checksum used to verify the integrity of the data packet.
 *
 * Note: Parsing the 'Measurement Data' requires additional processing to extract and use the distance and intensity data.
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
  Serial.begin(230400);  // Initialize USB serial communication
  LD19Serial.begin(230400, SERIAL_8N1, LD19_RX_PIN, -1); // Start LiDAR serial communication
  Serial.println("LD19 LiDAR data reading started.");
  delay(1000);  // Short delay after setup
}

void loop() {
  testExampleDataDetail2();  // Display packet details for testing
  delay(10000);  // Loop delay to prevent flooding
}

//------------------------------------------------------------------------------

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
}

// Function to test displaying packet details with a hard-coded example
void testExampleDataDetail2() {
    // This example packet is predefined with specific values for testing purposes.
    // It mimics a real packet from the LiDAR sensor to validate the parsing logic.
    uint8_t exampleData[] = {
        0x54, 0x2C,  // Header and VerLen
        0x68, 0x08,  // Speed (arbitrary example value)
        0x6C, 0x8A,  // Start Angle: 355 degrees in little-endian format
        // Measurement data (12 sets, each 3 bytes: distance and intensity)
        0xE0, 0x00, 0xE4,  // Point 1
        0xDC, 0x00, 0xE2,  // Point 2
        0xD9, 0x00, 0xE5,  // Point 3
        0xD5, 0x00, 0xE3,  // Point 4
        0xD3, 0x00, 0xE4,  // Point 5
        0xD0, 0x00, 0xE9,  // Point 6
        0xCD, 0x00, 0xE4,  // Point 7
        0xCA, 0x00, 0xE2,  // Point 8
        0xC7, 0x00, 0xE9,  // Point 9
        0xC5, 0x00, 0xE5,  // Point 10
        0xC2, 0x00, 0xE5,  // Point 11
        0xB0, 0x00, 0xEA,  // Point 12
        0xF4, 0x01,        // End Angle: 5 degrees in little-endian format
        0xBE, 0x82,        // Timestamp (arbitrary example value)
        0x3A               // CRC (arbitrary example value, should be calculated based on actual data)
    };

    // The actual CRC needs to be calculated based on the content of the packet, excluding the CRC byte itself.

    Serial.println("Testing with example data packet:");
    printPacketHexDetails(exampleData);
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


