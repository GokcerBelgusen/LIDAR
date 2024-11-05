#include <Arduino.h>

#define SERIAL_BAUDRATE 115200
#define FRAME_HEADER 0xAA
#define PROTOCOL_VERSION 0x01
#define FRAME_TYPE 0x61
#define SCAN_STEPS 15

#define ROTATION_SPEED_SCALE 3.0
#define ANGLE_SCALE 0.01
#define RANGE_SCALE 0.00025

// Delta-2G Frame structure
struct Delta2GFrame {
  uint8_t frameHeader;
  uint16_t frameLength;
  uint8_t protocolVersion;
  uint8_t frameType;
  uint8_t commandWord;
  uint16_t parameterLength;
  uint8_t parameters[4096];
  uint16_t checksum;
};



// Scan variables
float scanSamplesSignalQuality[4096];
float scanSamplesRange[4096];
int scanIndex = 0;

void cleanDelta2GFrame(struct Delta2GFrame *frame) {
  frame->frameHeader = 0;
  frame->frameLength = 0;
  frame->protocolVersion = 0;
  frame->frameType = 0;
  frame->commandWord = 0;
  frame->parameterLength = 0;
  memset(frame->parameters, 0, sizeof(frame->parameters));
  frame->checksum = 0;
}

// Function to process LiDAR frames and print distances
void LiDARFrameProcessing(struct Delta2GFrame *frame) {
  int frameIndex;
  if (frame->commandWord == 0xAD) {
    float rpm = frame->parameters[0] * ROTATION_SPEED_SCALE;
    float offsetAngle = ((frame->parameters[1] << 8) + frame->parameters[2]) * ANGLE_SCALE;
    float startAngle = ((frame->parameters[3] << 8) + frame->parameters[4]) * ANGLE_SCALE;
    int sampleCnt = (frame->parameterLength - 5) / 3;
    frameIndex = int(startAngle / (360.0 / SCAN_STEPS));

    if (frameIndex == 0) {
      scanIndex = 0;
    }

    for (int i = 0; i < sampleCnt; i++) {
      uint8_t signalQuality = frame->parameters[5 + (i * 3)];
      uint16_t distance = (frame->parameters[5 + (i * 3) + 1] << 8) + frame->parameters[5 + (i * 3) + 2];
      scanSamplesSignalQuality[scanIndex] = signalQuality;
      scanSamplesRange[scanIndex++] = distance * RANGE_SCALE;
    }

    if (frameIndex == (SCAN_STEPS - 1)) {
      // Display distances in serial debug after a complete scan
      Serial.println("LiDAR Scan Distances:");
      for (int i = 0; i < scanIndex; i++) {
        float angle = (360.0 / scanIndex) * i;  // Calculate angle for each distance
        Serial.print("Angle: ");
        Serial.print(angle, 1);  // Print angle with 1 decimal place
        Serial.print("°, Distance: ");
        Serial.print(scanSamplesRange[i], 3);  // Print distance with 3 decimal places
        Serial.println(" m");
      }
    }
  }
}

// Main setup and loop function
void setup() {
  Serial.begin(SERIAL_BAUDRATE);
  Serial.println("LiDAR started...");
}

String hexData16(uint16_t by) {
  String hexData = "0x" + String(by, HEX);
  return hexData;
}

String hexData8(uint8_t by) {
  String hexData = "0x" + String(by, HEX);
  return hexData;
}


void loop() {
  static int status = 0, i = 0;
  static uint16_t checksum = 0;
  static struct Delta2GFrame lidarFrame;

  if (Serial.available()) {
    uint8_t by = Serial.read();

    switch (status) {
      case 0:
        lidarFrame.frameHeader = by;
        if (lidarFrame.frameHeader == FRAME_HEADER) {
          status = 1;
          Serial.println("lidarFrame.frameHeader :" + hexData8(by));
        }
        checksum = 0;
        break;
      case 1:
        lidarFrame.frameLength = (by << 8);
        status = 2;
        break;
      case 2:
        lidarFrame.frameLength += by;
        Serial.println("lidarFrame.frameLength :" + hexData16((lidarFrame.frameLength)));
        status = 3;
        break;
      case 3:
        lidarFrame.protocolVersion = by;
        Serial.println("lidarFrame.protocolVersion :" + hexData8((lidarFrame.protocolVersion)));
        status = 4;
        break;
      case 4:
        lidarFrame.frameType = by;
        Serial.println("lidarFrame.frameType :" + hexData8((lidarFrame.frameType )));
        status = 5;
        break;
      case 5:
        lidarFrame.commandWord = by;
        Serial.println("lidarFrame.commandWord :" + hexData8((lidarFrame.commandWord )));
        status = 6;
        break;
      case 6:
        lidarFrame.parameterLength = (by << 8);
        status = 7;
        break;
      case 7:
        lidarFrame.parameterLength += by;
        Serial.println("lidarFrame.parameterLength :" + hexData16((lidarFrame.parameterLength)));
        memset(lidarFrame.parameters, 0, sizeof(lidarFrame.parameters));
        status = 8;
        i = 0;
        break;
      case 8:
        lidarFrame.parameters[i++] = by;
        if (i - 1 == lidarFrame.parameterLength) {
          status = 9;
        }
        break;
      case 9:
        lidarFrame.checksum = (by << 8);
        status = 10;
        break;
      case 10:
        lidarFrame.checksum += by;
        Serial.println("lidarFrame.checksum :" + hexData16((lidarFrame.checksum)) + " " + hexData16(checksum) );
        if (lidarFrame.checksum == checksum) {
          LiDARFrameProcessing(&lidarFrame);
          cleanDelta2GFrame(&lidarFrame);
        } else {
          Serial.println("ERROR: Frame Checksum Failed");
        }
        status = 0;
        break;
    }
    if (status < 10) {
      checksum = (checksum + by) % 0xFFFF;
    }
  }
}
