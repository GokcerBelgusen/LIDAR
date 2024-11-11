import matplotlib.pyplot as plt
import numpy as np
import paho.mqtt.client as mqtt
import time

RANGE_SCALE = 0.0254  # Conversion from inches to meters
MAX_DISTANCE_THRESHOLD = 5  # Maximum limit for radial distance in meters

# Flespi MQTT credentials and topic
mqtt_broker = "mqtt.flespi.io"
mqtt_port = 1883  # Non-SSL port
mqtt_token = "obvK6D7mG927ZQzh6d3wlnvnMplf47Qlr3AuTyHWqydNyiL5yS7jRO6ah3KT8DPv"
mqtt_topic = "robot-lidar"

# Initialize the plot
plt.ion()
fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
ax.set_title("Live LiDAR Distance Data with Best Heading (Meters)")
ax.set_theta_zero_location("N")
ax.set_theta_direction(-1)
ax.set_rmax(MAX_DISTANCE_THRESHOLD)  # Adjust radial limit as needed
ax.plot(0, 0, 'ro', markersize=8)  # LiDAR position (Robot)

distances = []

def on_message(client, userdata, message):
    global distances
    try:
        payload = message.payload.decode()
        hex_distances = payload.split()
        distances = [int(d, 16) * RANGE_SCALE for d in hex_distances]
    except Exception as e:
        print(f"Error parsing message: {e}")

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected successfully.")
        client.subscribe(mqtt_topic)
    else:
        print(f"Failed to connect, return code {rc}")

# Function to find the best heading based on maximum distance
def find_best_heading(angles, distances):
    max_distance = 0
    best_angle = 0
    for angle, distance in zip(angles, distances):
        if distance > max_distance:
            max_distance = distance
            best_angle = angle
    return best_angle, max_distance

# Function to determine the robot's next action based on best heading angle
def determine_next_action(best_angle, threshold_angle=0.1):
    # Assume current heading is 0 radians (facing North)
    current_heading = 0
    angle_difference = best_angle - current_heading

    # Normalize angle difference to (-pi, pi)
    angle_difference = np.arctan2(np.sin(angle_difference), np.cos(angle_difference))

    if abs(angle_difference) < threshold_angle:
        action = "MOVE FORWARD"
    elif angle_difference > 0:
        action = f"TURN LEFT {np.degrees(angle_difference):.1f}°"
    else:
        action = f"TURN RIGHT {np.degrees(-angle_difference):.1f}°"
    
    return action

def detect_obstacles(lidar_data, threshold_distance=1.0):
    """
    Detect obstacles within a specified distance threshold.
    
    Parameters:
        lidar_data (list of tuples): Each tuple contains (angle, distance) readings from LiDAR.
        threshold_distance (float): Distance threshold to detect obstacles in meters.
    
    Returns:
        List of detected obstacles, each represented by (angle, distance).
    """
    obstacles = []
    for angle, distance in lidar_data:
        if distance <= threshold_distance:
            obstacles.append((angle, distance))
    
    return obstacles

def move_robot(obstacles, safe_distance=1.0):
    """
    Move the robot based on detected obstacles.
    
    Parameters:
        obstacles (list of tuples): Detected obstacles with (angle, distance).
        safe_distance (float): Minimum safe distance to avoid obstacles.
        
    Returns:
        str: Direction command ('FORWARD', 'LEFT', 'RIGHT', or 'STOP').
    """
    # Check for obstacles directly ahead (angle close to 0 degrees)
    front_obstacles = [ob for ob in obstacles if -30 <= ob[0] <= 30 and ob[1] < safe_distance]
    
    # Check for obstacles on the left side (angle between 30 and 90 degrees)
    left_obstacles = [ob for ob in obstacles if 30 < ob[0] <= 90 and ob[1] < safe_distance]
    
    # Check for obstacles on the right side (angle between -30 and -90 degrees)
    right_obstacles = [ob for ob in obstacles if -90 <= ob[0] < -30 and ob[1] < safe_distance]
    
    # Determine movement based on obstacle positions
    if not front_obstacles:
        return "FORWARD"  # Safe to move forward
    elif left_obstacles and not right_obstacles:
        return "RIGHT"    # Avoid by moving right
    elif right_obstacles and not left_obstacles:
        return "LEFT"     # Avoid by moving left
    else:
        return "STOP"     # Stop if obstacles are too close on all sides


client = mqtt.Client()
client.username_pw_set(mqtt_token)
client.on_message = on_message
client.on_connect = on_connect
client.reconnect_delay_set(min_delay=1, max_delay=60)
client.connect(mqtt_broker, mqtt_port)
client.loop_start()

try:
      while True:
        if distances:
            angles = np.linspace(0, 2 * np.pi, len(distances))


            # Combine angles and distances into a list of (angle,
            lidar_data = list(zip(angles, distances))
             
            # Detect obstacles within a 1-meter threshold
            obstacles = detect_obstacles(lidar_data, threshold_distance=0.01)
            print("Detected Obstacles:", obstacles)

            # Replace 'left_motor' and 'right_motor' with your actual motor control objects
            movement = move_robot(obstacles)
            speed = 50  # Speed percentage or value, depending on motor control library 
            print("movement :", movement)
            
            # Find the best heading angle
            best_angle, max_distance = find_best_heading(angles, distances)

            # Determine the next action
            next_action = determine_next_action(best_angle)

            # Clear plot and set parameters
            ax.cla()
            ax.set_title("Live LiDAR Distance Data with Best Heading (Meters)")
            ax.set_theta_zero_location("N")
            ax.set_theta_direction(-1)
            ax.set_rmax(max(MAX_DISTANCE_THRESHOLD, max(distances, default=MAX_DISTANCE_THRESHOLD)))
            ax.plot(0, 0, 'ro', markersize=8)  # LiDAR position (Robot)
            
            # Plot LiDAR points
            ax.plot(angles, distances, 'bo', markersize=4)

            # Plot the best heading as a pink arrow
            ax.annotate('', xy=(best_angle, max_distance), xytext=(0, 0),
                        arrowprops=dict(facecolor='pink', edgecolor='pink', width=2.0, headwidth=8))

            # Display the next action as text on the plot
            ax.text(0.5, -0.1, f"Next Action: {next_action}", transform=ax.transAxes, ha='center', fontsize=8, color='purple')

            plt.draw()
            plt.pause(0.1)

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Program interrupted")

finally:
    client.loop_stop()
