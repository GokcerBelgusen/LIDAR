
import websocket
import matplotlib.pyplot as plt
import numpy as np

# LiDAR Protocol Constants
FRAME_HEADER = 0xAA
PROTOCOL_VERSION = 0x01
FRAME_TYPE = 0x61
RANGE_SCALE = 0.25 * 0.001  # Convert from mm to meters
SCAN_STEPS = 15

# Storage for scan distances
scan_samples = []
buffer = bytearray()  # Buffer to accumulate incoming messages

def process_lidar_frame(frame):
    global scan_samples
    command_word = frame[6]  # Command word from the frame
    if command_word == 0xAD:  # Check if it is a valid command
        sample_count = (len(frame) - 5) // 3  # Determine sample count
        start_angle = (frame[3] << 8) + frame[4]
        scan_samples.clear()  # Clear previous samples for new scan
        for i in range(sample_count):
            signal_quality = frame[5 + (i * 3)]
            distance = (frame[5 + (i * 3) + 1] << 8) + frame[5 + (i * 3) + 2]
            scan_samples.append(distance * RANGE_SCALE)  # Convert to meters

        if len(scan_samples) == SCAN_STEPS:
            update_plot(scan_samples)

def update_plot(distances):
    plt.clf()
    angles = np.linspace(0, 2 * np.pi, len(distances), endpoint=False)
    ax = plt.subplot(111, polar=True)
    ax.plot(angles, distances, 'bo', markersize=4)
    ax.set_ylim(0, max(distances) + 0.5)  # Adjust this based on expected max range
    ax.set_title("LiDAR Scan Data")
    ax.grid(True)
    plt.pause(0.001)  # Allow for live updating of the plot

def on_message(ws, message):
    global buffer
    # Ensure the message is treated as a bytearray
    byte_data = bytearray(message, 'utf-8')  # or just bytearray(message) if already bytes
    buffer.extend(byte_data)  # Append the received message to the buffer

    # Process until buffer has enough data for at least one frame
    while len(buffer) >= 1000:  # Check for minimum length to process a frame
        # Check for the frame header
        if buffer[0] == FRAME_HEADER:
            frame_length = (buffer[1] << 8) + buffer[2] + 3  # Length includes header and length bytes
            if len(buffer) >= frame_length:  # Check if we have a full frame
                process_lidar_frame(buffer[:frame_length])  # Process the full frame
                buffer = buffer[frame_length:]  # Remove processed frame from buffer
            else:
                break  # Wait for more data if not enough for a full frame
        else:
            # Remove the first byte and continue searching for a valid header
            buffer.pop(0)


def on_error(ws, error):
    print("Error:", error)

def on_close(ws):
    print("WebSocket closed")

def on_open(ws):
    print("WebSocket connection opened")

if __name__ == "__main__":
    plt.ion()  # Turn on interactive mode for live plotting
    # Replace with your ESP8266 IP address
    websocket_url = "ws://192.168.1.36:81"  
    ws = websocket.WebSocketApp(websocket_url,
                                on_message=on_message,
                                on_error=on_error,
                                on_close=on_close)

    ws.on_open = on_open
    ws.run_forever()
