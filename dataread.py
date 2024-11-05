import serial
import matplotlib.pyplot as plt
import statistics
import math
import time
from collections import deque
import numpy as np

class LidarData():
    
    def __init__(self):        
        self.DATA_LENGTH = 7        # Adjust based on actual data packet length
        self.MAX_DISTANCE = 3000    # in mm
        self.MIN_DISTANCE = 100      # in mm
        self.port = '/dev/tty.usbserial-A50285BI'  # Specify the serial port 
        self.MAX_DATA_SIZE = 360     # resolution: 1 degree
        self.ser = None
        self.BAUDRATE = 115200

        self.data = {   # sensor data 
            'angles': deque(maxlen=self.MAX_DATA_SIZE),
            'distances': deque(maxlen=self.MAX_DATA_SIZE),
            'speed': [],
            'signal_strength': [],  # TODO: Implement this
            'checksum': [] 
        }
        
        # Setup plot
        self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})
        self.ax.set_rmax(self.MAX_DISTANCE)
        
        if not self.connectSerial(self.port, self.BAUDRATE):  # Setup serial communication
            print("Failed to connect to the LiDAR device.")
            exit(1)

    def connectSerial(self, port: str, baudrate: int) -> bool:
        try: 
            self.ser = serial.Serial(port, baudrate, timeout=1)  # Set a timeout
            self.ser.reset_input_buffer()  # Flush any previous data in the buffer
            print(f'Serial connection established @ {port}')
            return True  # Return True to confirm connection
        except Exception as e: 
            print(f"Connection error: {e}")
            return False
        
    def plotData(self) -> None:  # Plot data on a polar plot
        angles = list(self.data['angles'])
        distances = list(self.data['distances'])
        
        # Outlier filtering with IQR
        q1, q3 = np.percentile(distances, [25, 75])
        iqr = q3 - q1
        lower_bound = q1 - 1.5 * iqr
        upper_bound = q3 + 1.5 * iqr
        
        filtered_angles = [angles[i] for i in range(len(distances)) if lower_bound <= distances[i] <= upper_bound]
        filtered_distances = [distances[i] for i in range(len(distances)) if lower_bound <= distances[i] <= upper_bound]
        
        self.ax.clear()  # Clear current plot
        self.ax.plot(filtered_angles, filtered_distances, ".")  # Plot the points
        self.ax.set_rmax(self.MAX_DISTANCE)
        plt.draw()
        plt.pause(0.001)

    def updateData(self) -> None: 
        while True:
            try:
                if self.ser.in_waiting > 0:  # Check for data in the serial buffer
                    try:
                        raw_data = self.ser.read(self.DATA_LENGTH)  # Read the expected number of bytes
                        print(f"Raw data (bytes): {raw_data}")  # Print raw data for debugging
                        
                        # Parse the binary data according to the protocol
                        if len(raw_data) == self.DATA_LENGTH:
                            # Example parsing logic (adjust based on your protocol)
                            try:
                                angle = raw_data[0] * 4  # Assuming first byte is angle in degrees
                                speed = raw_data[1]       # Assuming second byte is speed in RPM
                                distances = [int.from_bytes(raw_data[i:i+2], 'big') for i in range(2, 6)]  # Read distances
                                checksum = raw_data[-1]    # Last byte as checksum
                                
                                print(f'speed: {speed} RPM, angle: {angle}, distances: {distances}')

                                # Validate and store the data
                                for dist in distances:
                                    if self.MIN_DISTANCE <= dist <= self.MAX_DISTANCE:
                                        # Store angular data in radians
                                        self.data['angles'].append(math.radians(angle))
                                        # Store radial data in mm
                                        self.data['distances'].append(dist)
                                        self.data['checksum'].append(checksum)
                                        self.data['speed'].append(speed)
                                
                                # If enough data is available, plot data
                                if len(self.data['angles']) >= self.MAX_DATA_SIZE:
                                    self.plotData()
                            except Exception as e:
                                print(f"Error parsing data: {e}")
                    except Exception as e:
                        print(f"Error reading data: {e}")  # Print error
            except KeyboardInterrupt:
                print("Exiting program.")
                exit()
    
    def getDistances(self) -> list: 
        return list(self.data['distances'])
    
    def getAngles(self) -> list: 
        return list(self.data['angles'])
    
if __name__ == '__main__':
    sensor = LidarData()
    sensor.updateData()
