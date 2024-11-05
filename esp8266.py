import time
import matplotlib.pyplot as plt
import numpy as np
import websocket

# WebSocket variables
WEBSOCKET_URL = "ws://192.168.1.36:81"  # Replace <ESP8266_IP> with the actual IP of your ESP8266

# Scan variables
scanSamplesSignalQuality = [0.0]
scanSamplesRange = [0.0]

# Delta-2G Frame Characteristics
FRAME_HEADER = 0xAA
PROTOCOL_VERSION = 0x01
FRAME_TYPE = 0x61
SCAN_STEPS = 15

ROTATION_SPEED_SCALE = 0.05 * 60
ANGLE_SCALE = 0.01
RANGE_SCALE = 0.25 * 0.001

# Delta-2G frame structure
class Delta2GFrame:
    frameHeader = 0
    frameLength = 0
    protocolVersion = 0
    frameType = 0
    commandWord = 0
    parameterLength = 0
    parameters = []
    checksum = 0

# Function to process LiDAR frames
def LiDARFrameProcessing(frame: Delta2GFrame):
    global scanSamplesSignalQuality, scanSamplesRange
    match frame.commandWord:
        case 0xAD:
            rpm = frame.parameters[0] * ROTATION_SPEED_SCALE
            offsetAngle = (frame.parameters[1] << 8) + frame.parameters[2]
            offsetAngle = offsetAngle * ANGLE_SCALE
            startAngle = (frame.parameters[3] << 8) + frame.parameters[4]
            startAngle = startAngle * ANGLE_SCALE
            sampleCnt = int((frame.parameterLength - 5) / 3)
            frameIndex = int(startAngle / (360.0 / SCAN_STEPS))

            if frameIndex == 0:
                scanSamplesRange.clear()
                scanSamplesSignalQuality.clear()

            for i in range(sampleCnt):
                signalQuality = frame.parameters[5 + (i * 3)]
                distance = (frame.parameters[5 + (i * 3) + 1] << 8) + frame.parameters[5 + (i * 3) + 2]
                scanSamplesSignalQuality.append(signalQuality)
                scanSamplesRange.append(distance * RANGE_SCALE)

            if frameIndex == (SCAN_STEPS - 1):
                update_map(scanSamplesRange)

# Plot room map in real-time
def update_map(distances):
    plt.clf()
    angles = np.linspace(0, 2 * np.pi, len(distances))
    ax = plt.subplot(111, polar=True)
    ax.plot(angles, distances, 'bo', markersize=4)
    ax.set_rmax(1)  # Adjust this based on expected max range
    ax.grid(True)
    plt.pause(0.001)

# WebSocket message handler
def on_message(ws, message):
    global status, checksum, lidarFrame

    # Print the received message in hexadecimal format
    print("Received message:", message)
    hex_output = ' '.join(f'{ord(c):02x}' for c in message)  # Convert each character to hex
    print("Hex format:", hex_output)

    # Simulating byte-by-byte processing as if reading from serial
    for by in message.encode():  # Assuming message is in byte-like format
        match status:
            case 0:
                lidarFrame.frameHeader = by
                if lidarFrame.frameHeader == FRAME_HEADER:
                    status = 1
                checksum = 0
            case 1:
                lidarFrame.frameLength = (by << 8)
                status = 2
            case 2:
                lidarFrame.frameLength += by
                status = 3
            case 3:
                lidarFrame.protocolVersion = by
                if lidarFrame.protocolVersion == PROTOCOL_VERSION:
                    status = 4
                else:
                    print("ERROR: Frame Protocol Version Failed")
                    status = 0
            case 4:
                lidarFrame.frameType = by
                if lidarFrame.frameType == FRAME_TYPE:
                    status = 5
                else:
                    print("ERROR: Frame Type Failed")
                    status = 0
            case 5:
                lidarFrame.commandWord = by
                status = 6
            case 6:
                lidarFrame.parameterLength = (by << 8)
                status = 7
            case 7:
                lidarFrame.parameterLength += by
                lidarFrame.parameters.clear()
                status = 8
            case 8:
                lidarFrame.parameters.append(by)
                if len(lidarFrame.parameters) == lidarFrame.parameterLength:
                    status = 9
            case 9:
                lidarFrame.checksum = (by << 8)
                status = 10
            case 10:
                lidarFrame.checksum += by
                if lidarFrame.checksum == checksum:
                    LiDARFrameProcessing(lidarFrame)
                else:
                    print("ERROR: Frame Checksum Failed")
                status = 0
        if status < 10:
            checksum = (checksum + by) % 0xFFFF

# WebSocket error handler
def on_error(ws, error):
    print(f"WebSocket error: {error}")

# WebSocket close handler
def on_close(ws):
    print("### WebSocket closed ###")

# WebSocket open handler
def on_open(ws):
    print("WebSocket connection opened")

# Main function
def main():
    global status, checksum, lidarFrame
    status = 0
    checksum = 0
    lidarFrame = Delta2GFrame()

    plt.ion()
    plt.figure()

    # Connect to WebSocket
    ws = websocket.WebSocketApp(
        WEBSOCKET_URL,
        on_message=on_message,
        on_error=on_error,
        on_close=on_close
    )
    ws.on_open = on_open
    ws.run_forever()

if __name__ == "__main__":
    main()
