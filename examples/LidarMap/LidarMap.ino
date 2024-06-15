/**
 * LD19 LiDAR Sensor Packet Detection
 * -----------------------------------
 * This program captures and processes data packets from the LD19 LiDAR sensor via a hardware serial
 * interface, aiming to detect and segregate packets for applications such as mapping and object detection.
 * 
 * Key Features:
 * - Uses hardware serial to handle high-speed data transmission at 230400 bps.
 * - Detects packet start with byte sequence `0x54 0x2C`, processing subsequent data as packet contents.
 * - Operates on a time-of-flight principle, calculating distance based on laser light return time.
 * 
 * Setup Parameters:
 * - Baud Rate: 230400
 * - Data Bits: 8
 * - Stop Bits: 1
 * - Parity: None
 * - Flow Control: None
 * 
 * The system utilizes the ESP32's buffer management and timing capabilities to ensure efficient data handling
 * without hardware or software flow control. This example serves as part of a comprehensive library for 
 * integrating the LD19 sensor into mapping and object detection systems with ESP32 controllers.
 */


#include <algorithm>  // Include for std::sort
#include <WiFi.h>
#include <WebServer.h>
#include <Arduino.h>
#include <HardwareSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>


// Replace with your network credentials
const char* ssid = "JenkinsWifi";
const char* password = "Jenkins31!";

WebServer server(80);

// LiDAR map and measurement settings
#define OCCUPIED_COLOR "#000000"  // Color for occupied cells in visualization
#define UNOCCUPIED_COLOR "transparent"  // Color for unoccupied cells
#define MAP_SIZE 100  // Dimension of the map in pixels
#define MAX_LIDAR_DISTANCE 200 // Maximum measurable distance in cm
#define MAP_SCALE 1 // Scale for map visualization
#define MAX_DISTANCE_CM 60 // 2 feet in centimeters
#define CENTER (MAP_SIZE / 2) // Center of the map


// Lidar Variables 
// Constants for degree field and resolution
#define TOTAL_DEGREES 360 // Total degrees in a full circle (configurable)
#define DEGREE_RESOLUTION 1 // Resolution of each increment (configurable)

// Calculate number of points based on total degrees and resolution
#define POINTS_COUNT (int)(TOTAL_DEGREES / DEGREE_RESOLUTION)

// Define RX and TX pins for the LiDAR connection on Serial1
#define LD19_RX_PIN 32 // RX pin to receive data from LiDAR
#define LD19_TX_PIN 33 // TX pin for LiDAR, not used in read-only mode
// Constants for serial communication and LiDAR packet parsing
#define POINT_PER_PACK 12 // Number of measurement points per LiDAR packet
#define HEADER 0x54 // Packet header byte
// Data arrays for LiDAR information
int distances[POINTS_COUNT];
int filteredDistances[POINTS_COUNT];
uint8_t intensities[POINTS_COUNT];
bool dataPresent[POINTS_COUNT];
float angles[POINTS_COUNT];  // Array to store angle values
// Global array to track which points have been updated
bool updatedPoints[POINTS_COUNT] = {false};
#define UPDATE_THRESHOLD 0.9 // Threshold for deciding when to process data

// Refresh rate control for printing LiDAR data
unsigned long refreshRate = 300; // Refresh rate in milliseconds (e.g., every 5 seconds)
unsigned long lastPrintTime = 0; // Last time the data was printed

// Initialize Serial1 for LiDAR communication
HardwareSerial LD19Serial(1); // Use hardware serial 1 for LiDAR

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


//-----------------------------------------------
// Filters
//-----------------------------------------------

void noFilter() {
    for (int i = 0; i < POINTS_COUNT; i++) {
        filteredDistances[i] = distances[i];  // Directly copy the original distances to the filtered distances array
    }
}


void filterNoiseWithLinearRegressionAndOutlierRemoval() {
    const int windowSize = 5;  // Define the window size for the linear regression
    const int threshold = 1000;  // Define the threshold for outlier detection
    int halfWindow = windowSize / 2;

    for (int i = 0; i < POINTS_COUNT; i++) {
        float sumX = 0, sumY = 0, sumXY = 0, sumXX = 0;
        int count = 0;

        // First, calculate the mean of the window to identify outliers
        int mean = 0;
        for (int j = -halfWindow; j <= halfWindow; j++) {
            int idx = i + j;
            if (idx >= 0 && idx < POINTS_COUNT) {
                mean += distances[idx];
            }
        }
        mean /= windowSize;

        // Now, collect terms for linear regression, excluding outliers
        for (int j = -halfWindow; j <= halfWindow; j++) {
            int idx = i + j;
            if (idx >= 0 && idx < POINTS_COUNT) {
                if (abs(distances[idx] - mean) < threshold) {  // Check if within threshold
                    float angle = idx * DEGREE_RESOLUTION;  // Compute the angle based on the index
                    sumX += angle;
                    sumY += distances[idx];
                    sumXY += angle * distances[idx];
                    sumXX += angle * angle;
                    count++;
                }
            }
        }

        // Calculate regression line parameters: y = mx + b
        if (count > 1) {  // To ensure we don't divide by zero
            float xBar = sumX / count;
            float yBar = sumY / count;
            float slope = (sumXY - sumX * yBar) / (sumXX - sumX * xBar);
            float intercept = yBar - slope * xBar;

            // Use the regression line to estimate the current point
            filteredDistances[i] = int(slope * (i * DEGREE_RESOLUTION) + intercept);
        } else {
            // If no points are valid, just use the original value
            filteredDistances[i] = distances[i];
        }
    }
}


void filterNoiseInLidarData() {
    // Initialize the filtered distances with the original data
    for (int i = 0; i < POINTS_COUNT; i++) {
        filteredDistances[i] = distances[i];
    }

    // Apply filtering logic, modifying only the filteredDistances array
    for (int i = 1; i < POINTS_COUNT - 1; i++) {
        int prevDist = filteredDistances[i - 1];
        int currDist = filteredDistances[i];
        int nextDist = filteredDistances[i + 1];

        if (abs(currDist - prevDist) > 1000 && abs(currDist - nextDist) > 1000) {
            filteredDistances[i] = (prevDist + nextDist) / 2;
        }
    }

    // Handle boundary conditions for the circular array
    if (abs(filteredDistances[0] - filteredDistances[1]) > 1000 && abs(filteredDistances[0] - filteredDistances[POINTS_COUNT - 1]) > 1000) {
        filteredDistances[0] = (filteredDistances[1] + filteredDistances[POINTS_COUNT - 1]) / 2;
    }
    if (abs(filteredDistances[POINTS_COUNT - 1] - filteredDistances[POINTS_COUNT - 2]) > 1000 && abs(filteredDistances[POINTS_COUNT - 1] - filteredDistances[0]) > 1000) {
        filteredDistances[POINTS_COUNT - 1] = (filteredDistances[POINTS_COUNT - 2] + filteredDistances[0]) / 2;
    }
}


void MedianFilter() {
    const int windowSize = 5;  // This should be an odd number to ensure a single median
    const int threshold = 1000;  // Define the threshold for outlier detection
    int halfWindow = windowSize / 2;
    int window[windowSize];  // Temporary window for storing local values

    for (int i = 0; i < POINTS_COUNT; i++) {
        int windowCount = 0;  // Count how many values are actually in the window

        // Populate the window with values around the current index
        for (int j = -halfWindow; j <= halfWindow; j++) {
            int idx = i + j;
            if (idx >= 0 && idx < POINTS_COUNT) {
                window[windowCount++] = distances[idx];
            }
        }

        // Only proceed if the window is fully populated
        if (windowCount == windowSize) {
            // Sort the window to find the median
            std::sort(window, window + windowSize);
            int median = window[windowCount / 2];

            // Check if the current point is an outlier
            if (abs(distances[i] - median) > threshold) {
                filteredDistances[i] = median;  // Replace the outlier with the median
            } else {
                filteredDistances[i] = distances[i];  // Otherwise, keep the original value
            }
        } else {
            // If window is not full, just use the original value
            filteredDistances[i] = distances[i];
        }
    }
}


void filterNoiseInLidarDataMovingAverage() {
    const int windowSize = 5; // Can be adjusted based on the specific needs
    for (int i = 0; i < POINTS_COUNT; i++) {
        int sum = 0;
        int count = 0;
        for (int j = -windowSize; j <= windowSize; j++) {
            int idx = i + j;
            if (idx >= 0 && idx < POINTS_COUNT) {
                sum += distances[idx];
                count++;
            }
        }
        filteredDistances[i] = sum / count;
    }
}

void LinearRegressionFilter() {
    const int windowSize = 5;  // Adjust the window size to tune the filter's smoothing/stiffness
    float sumX = 0, sumY = 0, sumXY = 0, sumXX = 0;
    int halfWindow = windowSize / 2;

    for (int i = 0; i < POINTS_COUNT; i++) {
        sumX = sumY = sumXY = sumXX = 0;
        int actualWindowSize = 0;
        for (int j = -halfWindow; j <= halfWindow; j++) {
            int idx = i + j;
            if (idx >= 0 && idx < POINTS_COUNT) {
                float angle = angles[idx];
                int distance = distances[idx];
                sumX += angle;
                sumY += distance;
                sumXY += angle * distance;
                sumXX += angle * angle;
                actualWindowSize++;
            }
        }
        float xBar = sumX / actualWindowSize;
        float yBar = sumY / actualWindowSize;
        float slope = (sumXY - sumX * yBar) / (sumXX - sumX * xBar);
        float intercept = yBar - slope * xBar;
        filteredDistances[i] = int(slope * angles[i] + intercept);
    }
}

//-----------------------------------------------
// LIDAR FUNCTIONS
//-----------------------------------------------


void printLidarData() {
    Serial.println("Angle | Distance (mm) | Intensity");
    for (int i = 0; i < POINTS_COUNT; i++) {
        if (dataPresent[i]) {
            Serial.printf("%.2f | %d | %d\n", angles[i], distances[i], intensities[i]);
        }
    }
}


void initializeLidarData() {
    // Initialize the arrays
    for (int i = 0; i < POINTS_COUNT; i++) {
        angles[i] = i * DEGREE_RESOLUTION;  // Initialize angle array
        distances[i] = 0;
        intensities[i] = 0;
        dataPresent[i] = false; // Initially, no data is present
    }
}


void initializeLidar() {
  LD19Serial.begin(230400, SERIAL_8N1, LD19_RX_PIN, -1);
  Serial.println("ESP32 LiDAR Data Processing and Forwarding");
}


//-----------------------------------------------
// ESP32 WEB FUNCTIONS-   ---- Visualise Map
//-----------------------------------------------


const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
  <title>LiDAR Map Visualization</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
      body { font-family: Arial; display: flex; justify-content: center; align-items: center; flex-direction: column; }
      #canvas { border: 1px solid #000; }
      .info { margin: 20px; }
      button { padding: 10px 20px; margin: 5px; }
      table { width: 100%; border-collapse: collapse; }
      th, td { border: 1px solid #ddd; padding: 8px; text-align: left; }
      th { background-color: #4CAF50; color: white; }
  </style>
</head>
<body>
    <h2>LiDAR Map</h2>
    <canvas id="canvas" width="400" height="400"></canvas>
    <div class="info">
        <button onclick="refreshData()">Refresh Data</button>
        <button onclick="changeZoom()">Zoom In/Out</button>
        <p>Zoom: <span id="zoomLevel">1</span>x</p>
        <p>Run Time: <span id="timer">0</span> seconds</p>
    </div>
    <table id="data-table">
        <tr>
            <th>Angle</th>
            <th>Distance (mm)</th>
            <th>Intensity</th>
        </tr>
    </table>
    <script>
        var canvas = document.getElementById('canvas');
        var ctx = canvas.getContext('2d');
        var startTime = Date.now();
        var zoom = 1;
        var zoomFactors = [0.5, 1, 2, 3, 4, 5]; // Zoom levels
        var currentZoomIndex = 1; // Start with 1x zoom

        function drawData() {
            fetch('/data')
            .then(response => response.json())
            .then(data => {
                ctx.clearRect(0, 0, canvas.width, canvas.height); // Clear only if necessary
                drawOverlay(); // Draw static parts if they don't change often
                ctx.beginPath();
                data.forEach((point, index) => {
                    const adjustedX = 200 + (point.x * zoom);
                    const adjustedY = 200 + (point.y * zoom);
                    ctx.lineTo(adjustedX, adjustedY);
                });
                ctx.closePath();
                ctx.strokeStyle = '#000000'; // Set once if always the same
                ctx.stroke();
                updateTable(data);
            });
        }






        function changeZoom() {
            currentZoomIndex = (currentZoomIndex + 1) % zoomFactors.length;
            zoom = zoomFactors[currentZoomIndex];
            document.getElementById('zoomLevel').textContent = zoom + 'x';
            drawData(); // Redraw with new zoom level
        }

        function drawOverlay() {
            for (let i = 0; i < 360; i += 45) {
                let radians = i * Math.PI / 180;
                let isNinetyDegrees = (i === 270); // Only enhance the 90-degree line
                let lengthModifier = isNinetyDegrees ? 1.5 : 1; // 50% longer for the 90-degree line
                let lineWidth = isNinetyDegrees ? 3 : 1; // Thicker line for the 90-degree angle
                ctx.beginPath();
                ctx.moveTo(200, 200);
                ctx.lineTo(200 + 190 * Math.cos(radians) * zoom * lengthModifier, 200 + 190 * Math.sin(radians) * zoom * lengthModifier);
                ctx.strokeStyle = 'rgba(255, 0, 0, 0.5)'; // Red color with 50% opacity
                ctx.lineWidth = lineWidth;
                ctx.stroke();
            }
        }


        function drawCircle() {
            var radiusPixels = (12 * 2.54) * (200 / MAX_LIDAR_DISTANCE) * zoom; // Convert 12 inches to cm and then to pixels
            ctx.beginPath();
            ctx.arc(200, 200, radiusPixels, 0, 2 * Math.PI, false);
            ctx.strokeStyle = '#00FF00'; // Set the circle color to green for visibility
            ctx.stroke();
        }

        function updateTable(data) {
            var table = document.getElementById('data-table');
            // Clear existing table rows except the header
            while(table.rows.length > 1) {
                table.deleteRow(1);
            }
            // Populate table with new data
            data.forEach((point) => {
                var row = table.insertRow(-1);
                var cell1 = row.insertCell(0);
                var cell2 = row.insertCell(1);
                var cell3 = row.insertCell(2);
                cell1.innerHTML = point.angle;
                cell2.innerHTML = point.distance;
                cell3.innerHTML = point.intensity;
            });
        }

        setInterval(() => {
            var runTime = Math.floor((Date.now() - startTime) / 1000);
            document.getElementById('timer').textContent = runTime;
        }, 1000);

        setInterval(drawData, 500); // Refresh every 1 seconds
        drawData(); // Initial draw
    </script>
</body>
</html>
)rawliteral";


String getMapData() {
    String data = "[";
    bool first = true;
    for (int angle = 0; angle < POINTS_COUNT; angle++) {
        if (dataPresent[angle] && filteredDistances[angle] > 0 && intensities[angle] > 10) {
            float radianAngle = angles[angle] * (PI / 180.0); // Ensure 'angles' array is used
            float distanceInCm = filteredDistances[angle] / 10.0; // Ensure conversion from mm to cm if necessary
            int x = int(distanceInCm * cos(radianAngle));
            int y = int(distanceInCm * sin(radianAngle));
            if (!first) data += ", ";
            data += "{";
            data += "\"x\":" + String(x) + ",";
            data += "\"y\":" + String(y) + ",";
            data += "\"angle\":" + String(angles[angle]) + ",";
            data += "\"distance\":" + String(filteredDistances[angle]) + ",";
            data += "\"intensity\":" + String(intensities[angle]);
            data += "}";
            first = false;
        }
    }
    data += "]";
    return data;
}



void setupWiFi() {
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("WiFi connected.");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    // Serve the static HTML page
    server.on("/", HTTP_GET, []() {
        server.send(200, "text/html", index_html);  // Corrected here, removed the function call brackets
    });

    // Endpoint to fetch data dynamically
    server.on("/data", HTTP_GET, []() {
        server.send(200, "application/json", getMapData()); // Ensure getMapData() returns a JSON string
    });

    server.begin();
}



//-----------------------------------------------
// LIDAR FUNCTIONS-   ---- GET ARRAY FOR ANGLE, DISTANCE, INTENSITY
//-----------------------------------------------




void processLiDARPacket(const uint8_t* packet) {
    const LiDARFrameTypeDef* frame = reinterpret_cast<const LiDARFrameTypeDef*>(packet);
    float startAngle = frame->start_angle * 0.01; // Convert to degrees
    float endAngle = frame->end_angle * 0.01; // Convert to degrees

    // Adjust step calculation to handle wrapping from 360 to 0 degrees
    float step = ((endAngle < startAngle) ? (360 - startAngle + endAngle) : (endAngle - startAngle)) / (POINT_PER_PACK - 1);

    for (int i = 0; i < POINT_PER_PACK; i++) {
        float angle = startAngle + step * i;
        if (angle >= 360) {
            angle -= 360; // Wrap around if the angle exceeds 360 degrees
        }
        int index = int(angle / DEGREE_RESOLUTION); // Compute index based on resolution
        if (index >= 0 && index < POINTS_COUNT) {
            distances[index] = frame->point[i].distance;
            intensities[index] = frame->point[i].intensity;
            dataPresent[index] = true; // Mark this angle as having data
            updatedPoints[index] = true; // Mark this point as updated

        }
    }
}

bool checkAndUpdateCompletion() {
    int count = 0;
    for (int i = 0; i < POINTS_COUNT; i++) {
        if (updatedPoints[i]) count++;
    }
    return (count >= (POINTS_COUNT * UPDATE_THRESHOLD)); // Check if 90% of the points are updated
}

void resetUpdateStatus() {
    for (int i = 0; i < POINTS_COUNT; i++) {
        updatedPoints[i] = false;
    }
}



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
            processLiDARPacket(packetBuffer);
            
            if (checkAndUpdateCompletion()) {
                // Apply filters and reset
                // noFilter();
                 MedianFilter();
                // filterNoiseInLidarData();
                // LinearRegressionFilter();
                // filterNoiseInLidarDataMovingAverage();
                // filterNoiseWithLinearRegressionAndOutlierRemoval();
                //printLidarData(); // Print LiDAR data summary
                resetUpdateStatus(); // Reset the update status array
            }
        }else {
          Serial.println("CRC check failed!");
        }
        bufferIndex = 0;
      }
    }
  }
}



//-----------------------------------------------

void lidarTask(void *pvParameters) {
  while (1) {
    PackageBuilderCRC(); // Process incoming packets
    vTaskDelay(pdMS_TO_TICKS(20)); // Scan every 500 milliseconds
  }
}


void mainTask(void *pvParameters) {
  // Loop other functionalities at different rates if needed
  while (1) {
    server.handleClient(); // Handle web server clients  }
    vTaskDelay(10); // Small delay to prevent watchdog timer issues
  }
}


void setup() {
    Serial.begin(115200);
    initializeLidarData();  // Initialize angle, distance, and intensity arrays
    initializeLidar();
    setupWiFi(); 
    // Create tasks for Lidar and other functions
    xTaskCreatePinnedToCore(lidarTask, "Lidar Task", 2048, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(mainTask, "Main Task", 4096, NULL, 1, NULL, 1);
}


void loop() {
  // Empty. Everything is handled by FreeRTOS tasks.

  //PackageBuilderCRC(); // Process incoming packets
  //server.handleClient(); // Handle web server clients  }
}




/**
 * Filter Function Mappings:
 * - LinearRegressionFilter: Implements Linear Regression Filtering.
 *   Uses a linear regression approach over a sliding window to predict distances,
 *   offering a high level of precision for data with underlying linear trends.
 * 
 * - MedianFilter: Implements Median Filter.
 *   Uses a sliding window to calculate the median, replacing the central value with this median
 *   to robustly remove outliers, especially effective in datasets with sporadic, extreme values.
 * 
 * - filterNoiseInLidarDataMovingAverage: Implements Moving Average Filter.
 *   Calculates the average of points within a sliding window, smoothing out fluctuations
 *   and providing a general noise reduction that's simple and effective for minor variations.
 * 
 * - filterNoiseWithLinearRegressionAndOutlierRemoval: Combines Linear Regression with Outlier Removal.
 *   First removes outliers based on a deviation threshold before applying linear regression,
 *   making it highly effective but computationally intensive.
 * 
 * - filterNoiseInLidarData: Implements Basic Outlier Removal.
 *   Directly adjusts data points that significantly deviate from their neighbors,
 *   simple and effective for clear outliers but may distort data with frequent or natural variations.
 */




/**
 * Summary of Filtering Methods:
 * - Linear Regression Filtering: High effectiveness, high complexity. Best for linear trends.
 * - Median Filter: High effectiveness, medium complexity. Excellent for outlier removal.
 * - Moving Average Filter: Medium effectiveness, low complexity. Good for small, uniform noise.
 * - Linear Regression with Outlier Removal: Medium-high effectiveness, very high complexity. Ideal for erratic data with anomalies.
 * - Basic Outlier Removal: Low-medium effectiveness, low complexity. Useful for occasional clear outliers.
 *
 * Ranking (Best to Worst):
 * 1. Median Filter: Most robust for outlier removal, minimal assumptions on data distribution.
 * 2. Linear Regression Filtering: Precise, suitable for linear data, computationally intensive.
 * 3. Linear Regression with Outlier Removal: Comprehensive cleaning, complex, less suitable for real-time.
 * 4. Moving Average Filter: Balances simplicity and smoothing, may oversmooth variable data.
 * 5. Basic Outlier Removal: Simplest, handles clear outliers, may distort with frequent or complex noise patterns.
 *
 * Choose a filter based on data characteristics, noise type, processing capacity, and real-time requirements.
 */
