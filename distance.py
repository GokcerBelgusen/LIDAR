import pygame
import serial
import math

# Serial port configuration
SERIAL_PORT = "/dev/tty.usbserial-A50285BI"
BAUD_RATE = 115200

# Frame setup for LDS02RR
FRAME_HEADER = 0xAA
FRAME_TYPE = 0x61

# Constants for scaling
RANGE_SCALE = 0.25  # Modify based on expected range scale
WARNING_DISTANCE_CM = 50.0  # 10 cm for warning
RED_THRESHOLD_CM = 100.0  # Distance threshold for red points

# Initialize pygame display
pygame.init()
WIDTH, HEIGHT = 800, 800
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("LDS02RR LiDAR Mapping")

# Colors and font
WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLACK = (0, 0, 0)
YELLOW = (255, 255, 0)
font = pygame.font.Font(None, 24)

# LiDAR frame structure
class LDS02RRFrame:
    frameHeader = 0
    frameType = 0
    distanceData = []

# Process LiDAR data to update the display
def process_lidar_data(frame):
    screen.fill(BLACK)
    
    warning_detected = False
    warning_distance = float('inf')
    warning_position = (0, 0)
    
    for angle, distance in enumerate(frame.distanceData):
        distance_cm = distance * RANGE_SCALE  # Convert to centimeters
        angle_rad = math.radians(angle)
        
        # Convert polar to Cartesian coordinates
        x = int(math.cos(angle_rad) * distance_cm) + WIDTH // 2
        y = int(math.sin(angle_rad) * distance_cm) + HEIGHT // 2
        
        # Determine color based on distance
        if distance_cm < RED_THRESHOLD_CM:
            color = RED
        else:
            color = GREEN
        
        # Draw the point on pygame screen
        pygame.draw.circle(screen, color, (x, y), 2)

        # Check for distance warning
        if distance_cm < WARNING_DISTANCE_CM:
            warning_detected = True
            warning_distance = distance_cm
            warning_position = (x, y)

    # Show warning if an object is detected within the warning distance
    if warning_detected:
        # Draw warning rectangle
        pygame.draw.rect(screen, YELLOW, (warning_position[0] - 50, warning_position[1] - 30, 100, 60), 2)
        warning_text = font.render(f"Object Detected: {warning_distance:.2f} cm", True, YELLOW)
        screen.blit(warning_text, (warning_position[0] - 45, warning_position[1] - 25))

    pygame.display.flip()

# Main function
def main():
    lidar_serial = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5)
    running = True
    frame = LDS02RRFrame()

    while running:
        data = lidar_serial.read(1000)  # Read data from the LiDAR
        if len(data) > 0:
            for byte in data:
                if byte == FRAME_HEADER:
                    # Collect data from LDS02RR
                    frame.frameHeader = byte
                    frame.frameType = FRAME_TYPE
                    frame.distanceData = [int.from_bytes(data[i:i+2], "little") for i in range(1, len(data)-1, 2)]
                    process_lidar_data(frame)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

    pygame.quit()
    lidar_serial.close()

if __name__ == "__main__":
    main()
