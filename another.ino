#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <AsyncMqttClient.h>

const int baudrate = 115200;
boolean mqttConnected = false;

// Define motor control pins for ESP8266
#define IN1 D0  // Right motor IN1
#define IN2 D1  // Right motor IN2
#define IN3 D2  // Left motor IN3
#define IN4 D3  // Left motor IN4


// Define ultrasound sensor pins for ESP8266
#define TRIGGER D7
#define ECHO    D6

// Replace with your Wi-Fi credentials
const char* ssid = "TRABZON";
const char* password = "10203040A";


// Flespi MQTT credentials
const char* mqtt_server = "mqtt.flespi.io";
const char* mqtt_token = "obvK6D7mG927ZQzh6d3wlnvnMplf47Qlr3AuTyHWqydNyiL5yS7jRO6ah3KT8DPv";  // Flespi MQTT token

const char* mqtt_lidar = "robot-lidar";
const char* mqtt_command = "robot-command";
const char* mqtt_ultrasound = "robot-ultrasound";


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

double* scanSamplesSignalQuality = nullptr;
double* scanSamplesRange = nullptr;
uint16_t* dd = nullptr;
int scanSamplesIndex = 0;

boolean debug = 0;

AsyncMqttClient mqttClient; // Declare the WiFiClient object globally

// Function to read distance in centimeters
float getUltrasonicDistance() {
  // Ensure the trigger pin is low initially
  digitalWrite(TRIGGER, LOW);
  delayMicroseconds(2);

  // Send a 10µs HIGH pulse to trigger the measurement
  digitalWrite(TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER, LOW);

  // Measure the duration of the echo pulse
  long duration = pulseIn(ECHO, HIGH);

  // Convert duration to distance in centimeters
  float distance = (duration * 0.034) / 2;

  return distance;
}

// Ultrasounic Navigation

// Threshold distance in cm to avoid obstacles
const int threshold = 20;

// Function to measure distances in all four directions
void measureAllDirections(long &distanceFront, long &distanceLeft, long &distanceBack, long &distanceRight) {
    distanceFront = getUltrasonicDistance();
    turnLeft(2);
    distanceLeft = getUltrasonicDistance();
    turnLeft(2);
    distanceBack = getUltrasonicDistance();
    turnLeft(2);
    distanceRight = getUltrasonicDistance();
    turnLeft(2);   
}


// Function to determine best movement direction
String decideDirection(long distanceFront, long distanceLeft, long distanceRight, long distanceBack) {
    if (distanceFront > threshold) {
        return "FORWARD";
    } else if (distanceLeft > threshold) {
        return "LEFT";
    } else if (distanceRight > threshold) {
        return "RIGHT";
    } else if (distanceBack > threshold) {
        return "BACKWARD";
    } else {
        return "STOP";
    }
}

void navigateUltrasonic() {
    long distanceFront = getUltrasonicDistance();

    if (distanceFront <= threshold) {
        Serial.println("Obstacle detected in front! Measuring distances...");
        
        long distanceLeft, distanceBack, distanceRight;
        measureAllDirections(distanceFront, distanceLeft, distanceBack, distanceRight);
        
        String command = decideDirection(distanceFront, distanceLeft, distanceRight, distanceBack);
        Serial.print("Move command: ");
        Serial.println(command);
        
        if (command == "FORWARD") {
            // Code to move forward
            moveForward(2);
        } else if (command == "LEFT") {
            turnLeft(2);
            // Code to turn left
        } else if (command == "RIGHT") {
            turnRight(2);
            // Code to turn right
        } else if (command == "BACKWARD") {
            moveBackward(2);
            // Code to move backward
        } else {
            stopMotors();
        }
    }
    delay(100);  // Small delay for stability
}


// Function to send a retained message via MQTT
void sendMQTTMessage(const char *topic, String payload) {
  mqttClient.publish(topic, 1, true, payload.c_str());
  //Serial.println("Retained Message Sent with Timestamp : " + payload);
}


// Function to connect to MQTT server
void connectToMqtt() {
  if (!mqttConnected) {
    Serial.println("Attempting to reconnect to MQTT...");
    mqttClient.connect();
  }
}

// Function to move forward
void moveBackward(int cycle) {

  for (int i = 0; i < cycle; i++) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    delay(100);
  }
  stopMotors();
}

// Function to move backward
void moveForward(int cycle) {
  for (int i = 0; i < cycle; i++) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    delay(100);
  }
  stopMotors();
}

// Function to turn left
void turnLeft(int cycle) {
  for (int i = 0; i < cycle; i++) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    delay(100);
  }

  stopMotors();
}

// Function to turn right
void turnRight(int cycle) {

  for (int i = 0; i < cycle; i++) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    delay(100);
  }

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
    if (debug) {
      for (int i = 0; i < sampleCnt; i++) {
        if (frame->parameters[i] < 0x10) {
          Serial.print("0");  // Add leading zero for single-digit hex values
        }
        Serial.print(frame->parameters[i], HEX);  // Print as two-digit hex
        Serial.print(" ");  // Space between hex values
      }
      Serial.println();
    }

    if (frameIndex == 0) {
      // Clear previous allocations if any
      delete[] dd;
      delete[] scanSamplesRange;
      delete[] scanSamplesSignalQuality;
      scanSamplesIndex = 0;
      scanSamplesRange = new double[sampleCnt * scanSteps];
      scanSamplesSignalQuality = new double[sampleCnt * scanSteps];
      dd = new uint16_t[sampleCnt * scanSteps];
    }

    for (int i = 0; i < sampleCnt; i++) {
      uint8_t signalQuality = frame->parameters[5 + (i * 3)];
      uint16_t distance = (frame->parameters[5 + (i * 3) + 1] << 8) + frame->parameters[5 + (i * 3) + 2];
      if (scanSamplesRange != nullptr && scanSamplesIndex < sampleCnt * scanSteps) {
        scanSamplesSignalQuality[scanSamplesIndex] = signalQuality;
        scanSamplesRange[scanSamplesIndex] = distance * rangeScale;
        dd[scanSamplesIndex] = distance;
        scanSamplesIndex++;
      }
    }

    //Scan complete
    if (frameIndex == scanSteps - 1) {


      float distance = getUltrasonicDistance();
      uint16_t distance_cm = static_cast<uint16_t>(distance);
      char hexString[5]; // 4 characters for the hex value + 1 for null terminator

      // Convert the uint16_t value to a hex string
      sprintf(hexString, " % 04X", distance_cm);

      Serial.print("Distance: ");
      Serial.print(distance_cm);
      Serial.println(" cm");

      sendMQTTMessage(mqtt_ultrasound, hexString);


      if (scanSamplesRange != nullptr && scanSamplesIndex < sampleCnt * scanSteps) {
        String hexString = "";
        for (int i = 0; i < scanSamplesIndex; i++) {
          if (dd[i] < 0x1000) hexString += "0";  // Add leading zero for values < 0x1000
          if (dd[i] < 0x100) hexString += "0";   // Add leading zero for values < 0x100
          if (dd[i] < 0x10) hexString += "0";    // Add leading zero for values < 0x10

          hexString += String(dd[i], HEX);  // Append hex value to the string
          hexString += " ";  // Append a space after each hex value
        }

        hexString.trim();  // Remove the trailing space
        sendMQTTMessage(mqtt_lidar, hexString);
      }
    }
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

void setup_wifi() {

  stopMotors();

  // setup pins for ultrasound
  pinMode(TRIGGER, OUTPUT);
  pinMode(ECHO, INPUT);

  Serial.begin(115200);
  Serial.print("Connecting to WiFi...");

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi");
  delay(1000);
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to Flespi MQTT.");
  mqttConnected = true;
  mqttClient.subscribe(mqtt_command, 1);  // Subscribe with QoS 1
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  mqttConnected = false;
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.print("Subscribed to topic with QoS ");
  Serial.println(qos);
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  Serial.print("Message arrived on topic: ");
  Serial.println(topic);

  // Make sure the payload is null-terminated before using it
  payload[len] = '\0';

  // Convert the payload to uppercase for consistency
  for (size_t i = 0; i < len; i++) {
    payload[i] = toupper(payload[i]);
    delay(0.1);
  }

  // Execute movement based on determined action
  if (strcmp(payload, "FORWARD") == 0) {
    moveForward(2);
  } else if (strcmp(payload, "LEFT") == 0) {
    turnLeft(2);
  } else if (strcmp(payload, "RIGHT") == 0) {
    turnRight(2);
  } else if (strcmp(payload, "BACKWARD") == 0) {
    moveBackward(2);
  } else {
    stopMotors();
  }
  Serial.println(payload);
}

void setup() {

  // Set motor control pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.begin(baudrate);

  setup_wifi();

  // Set up MQTT server
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.setServer(mqtt_server, 1883);
  mqttClient.setKeepAlive(600);
  mqttClient.setCredentials(mqtt_token, "");  // Use token as username, leave password empty

  // Connect to MQTT
  connectToMqtt();
  delay(3000);

}

void loop() {
  // Ensure the MQTT connection is alive
  if (!mqttConnected) {
    connectToMqtt();  // Attempt to reconnect if not connected
  }

  LidarProcess();
 
}
