#include <ESP8266WiFi.h>
#include <WiFiClient.h>

const int baudrate = 115200;

// Define motor control pins for ESP8266
#define IN1 D0  // Right motor IN1
#define IN2 D1  // Right motor IN2
#define IN3 D2  // Left motor IN3
#define IN4 D3  // Left motor IN4


// Replace with your Wi-Fi credentials
const char* ssid = "TRABZON";
const char* password = "10203040A";

// Delay to Turn 10 degrees
int motordelay = 0;  // ~28 ms for a 10-degree turn

struct Delta2GFrame {
  uint8_t frameHeader;
  uint16_t frameLength;
  uint8_t protocolVersion;
  uint8_t frameType;
  uint8_t commandWord;
  uint16_t parameterLength;
  uint8_t* parameters;
  uint16_t checksum;
};

const uint8_t frameHeaderValue = 0xAA;
const uint8_t protocolVersionValue = 0x01;
const uint8_t frameTypeValue = 0x61;
const int scanSteps = 15;

const double rotationSpeedScale = 0.05 * 60;
const double angleScale = 0.01;
const double rangeScale = 0.254 * 0.001;

double* scanSamplesSignalQuality;
double* scanSamplesRange;
int scanSamplesIndex = 0;

boolean debug = 0;


WiFiClient client; // Declare the WiFiClient object globally

int calculateTurnDelay(float angle) {
  return (1000 * angle) / 180;
}


// Function to move forward
void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(motordelay);
  stopMotors();
}

// Function to move backward
void moveBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(motordelay);
  stopMotors();
}

// Function to turn left
void turnLeft() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(motordelay);
  stopMotors();
}

// Function to turn right
void turnRight() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(motordelay);
  stopMotors();
}

// Function to stop the motors
void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void onNewRPMReceiveCallback(int rpm) {
  Serial.print("Current LiDAR RPM: ");
  Serial.println(rpm);
}


// Function to find the best heading based on LiDAR data
float heading_find_best(double distances[], int scanSamples) {
  float bestHeading = 0.0; // Initialize to forward (0 degrees)
  float maxClearDistance = 0.0;

  // Iterate through the distances and determine the clearest direction
  for (int i = 0; i < scanSamples; i++) {
    float angle = (360.0 / scanSamples) * i;
    float distance = distances[i];

    if (angle > 0.0 && angle < 180.0 ) {
      // Check if this direction has the farthest clear distance
      if (distance > maxClearDistance) {
        maxClearDistance = distance;
        bestHeading = angle;
      }
    }

  }

  return bestHeading; // Return the angle with the maximum clear distance
}


// Function to process LiDAR frames and print distances
void LiDARFrameProcessing(Delta2GFrame *frame) {
  int frameIndex;
  String movement = "";

  bool northBlocked = false;
  bool southBlocked = false;
  bool eastBlocked = false;
  bool westBlocked = false;

  if (frame->commandWord == 0xAE) {
    // Get RPM data
    int rpm = frame->parameters[0] * rotationSpeedScale;  // Adjust based on how RPM is stored in the frame
    onNewRPMReceiveCallback(rpm);
  }

  if (frame->commandWord == 0xAD) {
    float rpm = frame->parameters[0] * rotationSpeedScale;
    float offsetAngle = ((frame->parameters[1] << 8) + frame->parameters[2]) * angleScale;
    float startAngle = ((frame->parameters[3] << 8) + frame->parameters[4]) * angleScale;
    int sampleCnt = (frame->parameterLength - 5) / 3;
    frameIndex = int(startAngle / (360.0 / scanSteps));

    if (frameIndex == 0) {
      delete[] scanSamplesRange;  // Clear previous allocations if any
      delete[] scanSamplesSignalQuality;
      scanSamplesIndex = 0;
      scanSamplesRange = new double[sampleCnt * scanSteps];
      scanSamplesSignalQuality = new double[sampleCnt * scanSteps];
    }

    for (int i = 0; i < sampleCnt; i++) {
      uint8_t signalQuality = frame->parameters[5 + (i * 3)];
      uint16_t distance = (frame->parameters[5 + (i * 3) + 1] << 8) + frame->parameters[5 + (i * 3) + 2];
      if (scanSamplesRange != nullptr && scanSamplesIndex < sampleCnt * scanSteps) {
        scanSamplesSignalQuality[scanSamplesIndex] = signalQuality;
        scanSamplesRange[scanSamplesIndex++] = distance * rangeScale;
      }
    }

    //Scan complete
    if (frameIndex == scanSteps - 1) {
     if (scanSamplesRange != nullptr && scanSamplesIndex < sampleCnt * scanSteps) {
        // Call heading_find_best to determine the best direction to move
        float bestHeading = heading_find_best(scanSamplesRange, scanSamplesIndex);

        // Decide movement based on the best heading angle
        if ((bestHeading >= 345 || bestHeading < 15) && !northBlocked) {
          movement = "FORWARD";
        } else if (bestHeading >= 30 && bestHeading < 60 && !eastBlocked) {
          movement = "TURN RIGHT";
        } else if (bestHeading >= 165 && bestHeading < 195 && !southBlocked) {
          movement = "REVERSE";
        } else if (bestHeading >= 285 && bestHeading < 315 && !westBlocked) {
          movement = "TURN LEFT";
        } else {
          movement = "STOP";
        }

        Serial.print("Best Heading: "); Serial.print(bestHeading, 2);
        Serial.print(" degrees and ");
        Serial.println("Movement Decision: " + movement);
      }
    }


  }

  // Execute movement based on determined action
  if (movement == "FORWARD") {
    moveForward();
  } else if (movement == "TURN LEFT") {
    turnLeft();
  } else if (movement == "TURN RIGHT") {
    turnRight();
  } else if (movement == "REVERSE") {
    moveBackward();
  } else {
    stopMotors();
  }
}


void LidarProcess() {
  uint16_t checksum = 0;
  int status = 0;
  int index = 0;
  Delta2GFrame lidarFrame = {0, 0, 0, 0, 0, 0, nullptr, 0};
  uint8_t rxBuffer[2048];

  if (Serial.available() > 0) {

    int rxLength = Serial.readBytes(rxBuffer, sizeof(rxBuffer));

    for (int i = 0; i < rxLength; i++) {
      switch (status) {
        case 0:
          lidarFrame.frameHeader = rxBuffer[i];
          if (lidarFrame.frameHeader == frameHeaderValue) {
            status = 1;
            checksum = 0;
          }
          break;
        case 1:
          lidarFrame.frameLength = (rxBuffer[i] << 8);
          status = 2;
          break;
        case 2:
          lidarFrame.frameLength += rxBuffer[i];
          status = 3;
          break;
        case 3:
          lidarFrame.protocolVersion = rxBuffer[i];
          if (lidarFrame.protocolVersion == protocolVersionValue) {
            status = 4;
          } else {
            status = 0;
          }
          break;
        case 4:
          lidarFrame.frameType = rxBuffer[i];
          if (lidarFrame.frameType == frameTypeValue) {
            status = 5;
          } else {
            status = 0;
          }
          break;
        case 5:
          lidarFrame.commandWord = rxBuffer[i];
          status = 6;
          break;
        case 6:
          lidarFrame.parameterLength = (rxBuffer[i] << 8);
          status = 7;
          break;
        case 7:
          lidarFrame.parameterLength += rxBuffer[i];
          lidarFrame.parameters = new uint8_t[lidarFrame.parameterLength];
          index = 0;
          status = 8;
          break;
        case 8:
          lidarFrame.parameters[index++] = rxBuffer[i];
          if (index == lidarFrame.parameterLength) {
            status = 9;
          }
          break;
        case 9:
          lidarFrame.checksum = (rxBuffer[i] << 8);
          status = 10;
          break;
        case 10:
          lidarFrame.checksum += rxBuffer[i];
          if (lidarFrame.checksum == checksum) {
            LiDARFrameProcessing(&lidarFrame);
          }
          status = 0;
          delete[] lidarFrame.parameters;
          break;
        default:
          status = 0;
          break;
      }
      if (status < 10) {
        checksum += rxBuffer[i];
      }
    }
  }
}

void setup() {

  // Set motor control pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.begin(baudrate);

  // Connect to Wi-Fi
  if (debug) {
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");
  }


  motordelay = calculateTurnDelay(30.0);
  Serial.println("Motordelay : " + motordelay);

  delay(5000);

  for (int i = 0; i < 3; i++) {
    turnLeft();
    delay(1000);
  }

  for (int i = 0; i < 3; i++) {
    turnRight();
    delay(1000);
  }


}

void loop() {
  LidarProcess();
}
