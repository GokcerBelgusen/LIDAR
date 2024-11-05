from http.server import BaseHTTPRequestHandler, HTTPServer
import json
import matplotlib.pyplot as plt
import numpy as np
import threading

# Global variable to store LiDAR data
lidar_data = None

class SimpleHTTPRequestHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        global lidar_data
        content_length = int(self.headers['Content-Length'])  # Get the size of the data
        post_data = self.rfile.read(content_length)  # Read the data
        lidar_data = post_data.decode('utf-8')  # Store the data
        print("Received data:", lidar_data)  # Print the data
  
        
def run(server_class=HTTPServer, handler_class=SimpleHTTPRequestHandler, port=8080):
    server_address = ('', port)  # Listen on all available interfaces
    httpd = server_class(server_address, handler_class)
    print(f'Server started at http://0.0.0.0:{port}')
    httpd.serve_forever()  # Start the server

# Plot room map in real-time
def update_map(distances):
    plt.clf()
    angles = np.linspace(0, 5 * np.pi, len(distances))  # Create an array of angles
    ax = plt.subplot(111, polar=True)  # Create a polar subplot
    ax.plot(angles, distances, 'bo', markersize=4)  # Plot the points in polar coordinates
    ax.set_rmax(1)  # Adjust this based on expected max range
    ax.grid(True)  # Enable grid
    plt.pause(0.001)  # Pause to allow for plot update
    

def plot_lidar_data():
    global lidar_data
    plt.ion()  # Enable interactive mode for real-time updating
    plt.figure(figsize=(8, 8))

    while True:
        if lidar_data:
            try:
                # Parse the JSON message
                data = json.loads(lidar_data)
                distances = np.array([entry['distance'] for entry in data['distances']])

                # Update the polar plot
                update_map(distances)
                
            except json.JSONDecodeError:
                print("Error decoding JSON data.")
            except Exception as e:
                print(f"An error occurred: {e}")

        plt.pause(0.5)  # Wait before checking for new data again

if __name__ == "__main__":
    # Start the server in a separate thread
    server_thread = threading.Thread(target=run, args=(HTTPServer, SimpleHTTPRequestHandler, 8080))
    server_thread.start()

    # Start plotting the LiDAR data
    plot_lidar_data()
