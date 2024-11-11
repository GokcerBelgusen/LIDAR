import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import keyboard  # Library to capture keyboard input

# Maze setup (simple walls)
maze = [
    ((1, 1), (8, 1)),  # Bottom wall
    ((8, 1), (8, 8)),  # Right wall
    ((8, 8), (1, 8)),  # Top wall
    ((1, 8), (1, 1)),  # Left wall
    ((3, 3), (3, 7)),  # Inner wall
    ((5, 7), (3, 7)),
]

# Robot setup
robot_position = np.array([2.0, 2.0])
robot_radius = 0.3
move_distance = 0.1
robot_angle = 0  # Robot's facing direction in degrees (0 degrees is to the right)

# LiDAR setup
lidar_range = 2.0  # Max LiDAR range in meters
angles = np.linspace(-90, 90, 5)  # Five LiDAR rays (in degrees)

# Movement functions
def move_robot(direction):
    global robot_position, robot_angle
    if direction == "FORWARD":
        robot_position[1] += move_distance * np.sin(np.radians(robot_angle))
        robot_position[0] += move_distance * np.cos(np.radians(robot_angle))
        print(f"Moving forward. New Position: {robot_position}")
    elif direction == "LEFT":
        robot_angle += 10  # Rotate left
        if robot_angle >= 360:
            robot_angle -= 360
        print(f"Rotating left. New Angle: {robot_angle}")
    elif direction == "RIGHT":
        robot_angle -= 10  # Rotate right
        if robot_angle < 0:
            robot_angle += 360
        print(f"Rotating right. New Angle: {robot_angle}")
    elif direction == "STOP":
        print("Stopping. No movement.")

# LiDAR simulation
def simulate_lidar():
    lidar_data = []
    for angle in angles:
        angle_rad = np.radians(angle + robot_angle)
        ray_direction = np.array([np.cos(angle_rad), np.sin(angle_rad)])  # Direction of the LiDAR ray
        min_distance = lidar_range  # Start with max range

        # Check for intersection with each wall
        for wall_start, wall_end in maze:
            wall_start = np.array(wall_start)
            wall_end = np.array(wall_end)
            wall_vector = wall_end - wall_start
            robot_to_wall_start = wall_start - robot_position

            # Calculate denominator of the intersection formula
            denom = ray_direction[0] * wall_vector[1] - ray_direction[1] * wall_vector[0]
            if abs(denom) < 1e-6:  # Nearly parallel lines (no intersection)
                continue

            # Parametric intersection factors
            t = (robot_to_wall_start[0] * wall_vector[1] - robot_to_wall_start[1] * wall_vector[0]) / denom
            u = (robot_to_wall_start[0] * ray_direction[1] - robot_to_wall_start[1] * ray_direction[0]) / denom

            # t is distance factor along ray, u is position along wall segment
            if 0 <= t <= lidar_range and 0 <= u <= 1:
                intersection_point = robot_position + t * ray_direction
                distance = np.linalg.norm(intersection_point - robot_position)
                min_distance = min(min_distance, distance)

        # Append (angle, distance) tuple
        lidar_data.append((angle, min_distance))
    
    return lidar_data

def detect_obstacles(lidar_data, threshold_distance=1.0):
    obstacles = []
    for angle, distance in lidar_data:
        if distance <= threshold_distance:
            obstacles.append((angle, distance))
    return obstacles

# Autonomous Heading Algorithm to turn until a clear path is found
def find_best_heading(lidar_data):
    best_angle = None
    max_clear_distance = -1

    for angle, distance in lidar_data:
        if distance > max_clear_distance:
            max_clear_distance = distance
            best_angle = angle
    
    # Adjust robot's angle to face the best heading direction
    return best_angle

# Plotting setup
fig, ax = plt.subplots()
ax.set_xlim(0, 10)
ax.set_ylim(0, 10)

# Draw maze walls
for wall_start, wall_end in maze:
    ax.plot([wall_start[0], wall_end[0]], [wall_start[1], wall_end[1]], 'k-')

# Robot and lidar visualization
robot_circle = patches.Circle(robot_position, robot_radius, fc='blue')
ax.add_patch(robot_circle)
lidar_lines = [ax.plot([], [], 'r-')[0] for _ in range(len(angles))]

# Update function for animation
def update(frame):
    lidar_data = simulate_lidar()
    obstacles = detect_obstacles(lidar_data)
    
    # Find the best heading based on LiDAR data
    best_angle = find_best_heading(lidar_data)

    # Check if there is a clear path ahead, and turn the robot until it finds one
    if best_angle is not None:
        angle_diff = best_angle - robot_angle
        if abs(angle_diff) > 50:  # If large difference, rotate
            if angle_diff > 10:
                move_robot("LEFT")
            else:
                move_robot("RIGHT")
        else:  # If small difference, move forward
            move_robot("FORWARD")
    
    # Stop turning if clear path is found ahead
    if best_angle is not None and abs(best_angle - robot_angle) < 5:
        move_robot("FORWARD")

    # Update robot position
    robot_circle.set_center(robot_position)
    
    # Update lidar visualization
    for line, (angle, distance) in zip(lidar_lines, lidar_data):
        angle_rad = np.radians(angle + robot_angle)
        end_x = robot_position[0] + distance * np.cos(angle_rad)
        end_y = robot_position[1] + distance * np.sin(angle_rad)
        line.set_data([robot_position[0], end_x], [robot_position[1], end_y])
    
    return [robot_circle] + lidar_lines

# Run animation
ani = FuncAnimation(fig, update, frames=100, interval=200, blit=True)
plt.show()
