import matplotlib.pyplot as plt
import numpy as np
import paho.mqtt.client as mqtt
import time
import keyboard


ROBOT_HEAD  = 270
ROBOT_BACK  = 90
ROBOT_RIGHT = 0
ROBOT_LEFT  = 180

SCALE=4

RANGE_SCALE = 0.0254*SCALE  # Conversion from inches to meters
MAX_DISTANCE_THRESHOLD = 2*SCALE  # Maximum limit for radial distance in meters
CLOSE_DISTANCE_THRESHOLD = 0.3*SCALE  # Threshold for close objects in meters

# Flespi MQTT credentials and topic
mqtt_broker = "mqtt.flespi.io"
mqtt_port = 1883  # Non-SSL port
mqtt_token = "obvK6D7mG927ZQzh6d3wlnvnMplf47Qlr3AuTyHWqydNyiL5yS7jRO6ah3KT8DPv"

# topics
mqtt_lidar = "robot-lidar"
mqtt_ultrasound = "robot-ultrasound"
mqtt_command = "robot-command"

# Initialize the plot
plt.ion()
fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
ax.set_title("Room Mapping with LiDAR (Meters)")
ax.set_theta_zero_location("N")
ax.set_theta_direction(-1)
ax.set_rmax(MAX_DISTANCE_THRESHOLD)  # Adjust radial limit as needed
ax.plot(0, 0, 'go', markersize=10)  # LiDAR position (Robot)

distances = []

 
def on_message(client, userdata, message):
    global distances
    # Check the topic to process messages from different sources
    if message.topic == mqtt_lidar:
       payload = message.payload.decode()
       hex_distances = payload.split()
       distances = [int(d, 16) * RANGE_SCALE / 100 for d in hex_distances]
       # print("Lidar distances:", distances)
        
    elif message.topic == mqtt_ultrasound:
         payload = message.payload.decode()
         # Assuming ultrasound data is received as a single distance value
         distance_cm = int(payload,16)  # Convert to integer directly if it's in cm
         print("Ultrasound distance:", distance_cm)
        
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected successfully.")
        client.subscribe(mqtt_lidar)
        client.subscribe(mqtt_ultrasound)
    else:
        print(f"Failed to connect, return code {rc}")


def decide_direction():
    """
    Decides which direction to move based on LiDAR data using the head-first algorithm.
    Returns the optimal direction angle.
    """
    # Map directions to their angles
    direction_angles = {
        "FORWARD": ROBOT_HEAD,
        "BACKWARD": ROBOT_BACK,
        "LEFT": ROBOT_LEFT,
        "RIGHT": ROBOT_RIGHT
    }

    # Average distances for each direction
    angle_range = np.linspace(0, 2 * np.pi, len(distances))  # Map LiDAR points to full 360 degrees

    # Get average distance in each direction quadrant
    forward_dist = np.mean([d for a, d in zip(angle_range, distances) if 240 <= np.degrees(a) <= 300])
    back_dist = np.mean([d for a, d in zip(angle_range, distances) if 60 <= np.degrees(a) <= 120])
    left_dist = np.mean([d for a, d in zip(angle_range, distances) if 150 <= np.degrees(a) <= 210])
    right_dist = np.mean([d for a, d in zip(angle_range, distances) if 330 <= np.degrees(a) or np.degrees(a) <= 30])

    # Head-first priority order
    if forward_dist > CLOSE_DISTANCE_THRESHOLD:
        return direction_angles["FORWARD"],"FORWARD"
    elif right_dist > CLOSE_DISTANCE_THRESHOLD:
        return direction_angles["RIGHT"],"RIGHT"
    elif left_dist > CLOSE_DISTANCE_THRESHOLD:
        return direction_angles["LEFT"],"LEFT"
    else:
        return direction_angles["BACKWARD"],"BACKWARD"

def send_command(command):
    client.publish(mqtt_command, command)
    print(f"Sent command: {command}")
    
# MQTT setup
client = mqtt.Client()
client.username_pw_set(mqtt_token)
client.on_message = on_message
client.on_connect = on_connect
client.reconnect_delay_set(min_delay=1, max_delay=60)
client.connect(mqtt_broker, mqtt_port)
client.loop_start()

print("Ready...")

try:
    while True:
        
        if keyboard.is_pressed("up"):
            send_command("FORWARD")
        elif keyboard.is_pressed("left"):
            send_command("LEFT")
        elif keyboard.is_pressed("right"):
            send_command("RIGHT")
        elif keyboard.is_pressed("down"):
            send_command("BACKWARD")
        
        if distances:
             
            # Adjust angle for full 360-degree view based on the number of points
            angles = np.linspace(0, 2 * np.pi, len(distances))

            # Clear plot and reset parameters
            ax.cla()
            ax.set_title("Room Mapping with LiDAR (Meters)")
            ax.set_theta_zero_location("N")
            ax.set_theta_direction(-1)
            ax.set_rmax(max(MAX_DISTANCE_THRESHOLD, max(distances, default=MAX_DISTANCE_THRESHOLD)))
            ax.plot(0, 0, 'go', markersize=10)  # LiDAR position (Robot)
            
            # Filter and plot distances
            filtered_distances = [min(d, MAX_DISTANCE_THRESHOLD) for d in distances]
            #print(filtered_distances)
            
            # Plot close distances in red
            for angle, distance in zip(angles, filtered_distances):
                if distance <= CLOSE_DISTANCE_THRESHOLD:
                    ax.plot(angle, distance, 'ro', markersize=5)  # Close object in red
                else:
                    ax.plot(angle, distance, 'bo', markersize=4)  # Other points in blue
            
            # Fill the area around detected objects
            ax.fill_between(angles, 0, filtered_distances, color='lightblue', alpha=0.3)


            # Decide optimal movement direction and draw green arrow
            move_angle,label = decide_direction()
            ax.annotate(label, xy=(np.radians(move_angle), MAX_DISTANCE_THRESHOLD * 0.8),
                        xytext=(0, 0), arrowprops=dict(facecolor='green', shrink=0.05))

            
            plt.draw()
            plt.pause(0.1)

            

        

except KeyboardInterrupt:
    print("Program interrupted")

finally:
    client.loop_stop()
