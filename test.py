import serial
import numpy as np
import matplotlib.pyplot as plt

# Serial port variables
SERIAL_PORT = "/dev/tty.usbserial-A50285BI" # Update to your serial port
SERIAL_BAUDRATE = 115200
SCAN_RANGE_THRESHOLD = 50  # cm
SCAN_ANGLE_INCREMENT = 2  # Degrees

# Function to initialize the serial connection
def initialize_serial(port, baudrate):
    try:
        lidar_serial = serial.Serial(port, baudrate, timeout=1)
        return lidar_serial
    except serial.SerialException as e:
        print(f"Serial connection error: {e}")
        return None

# Function to read data from the LiDAR
def read_lidar_data(serial_port):
    try:
        while True:
            if serial_port.in_waiting > 0:
                # Read a byte
                data = serial_port.read(1)
                if data:
                    yield data
    except Exception as e:
        print(f"Error reading from LiDAR: {e}")

# Function to process and visualize LiDAR data
def process_lidar_data(data_stream):
    distances = []
    angles = []
    
    for data in data_stream:
        # Here we need to interpret the incoming data structure
        # This is a placeholder: update according to your data structure
        distance = int.from_bytes(data, byteorder='little')
        angle = len(distances) * SCAN_ANGLE_INCREMENT
        
        if distance < SCAN_RANGE_THRESHOLD:  # filter by distance
            distances.append(distance)
            angles.append(angle)

        # Plot the data
        if len(distances) > 0:
            plt.clf()  # Clear the previous plot
            plt.polar(np.radians(angles), distances, 'ro')  # Polar plot
            plt.title('LDS01RR LiDAR Scan')
            plt.ylim(0, SCAN_RANGE_THRESHOLD)  # Set the limit for the plot
            plt.pause(0.01)

def main():
    lidar_serial = initialize_serial(SERIAL_PORT, SERIAL_BAUDRATE)
    if lidar_serial is not None:
        print("LiDAR connected. Starting data acquisition...")
        
        plt.ion()  # Enable interactive mode for live plotting
        data_stream = read_lidar_data(lidar_serial)
        
        process_lidar_data(data_stream)
        
    lidar_serial.close()

if __name__ == "__main__":
    main()
