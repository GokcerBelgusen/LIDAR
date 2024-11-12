import paho.mqtt.client as mqtt
import keyboard  # Make sure to install keyboard library with `pip install keyboard`
import time

# MQTT settings
mqtt_broker = "mqtt.flespi.io"
mqtt_port = 1883
mqtt_topic = "robot-command"
mqtt_token = "obvK6D7mG927ZQzh6d3wlnvnMplf47Qlr3AuTyHWqydNyiL5yS7jRO6ah3KT8DPv"

# Setup MQTT client
client = mqtt.Client()
client.username_pw_set(mqtt_token)
client.connect(mqtt_broker, mqtt_port)
client.loop_start()


# Track the state of the keys (whether they are currently pressed)
key_states = {
    "up": False,
    "down": False,
    "left": False,
    "right": False,
    "lidar" : False,
    "ultrasound" : False,
    "stop" : False
}


def send_command(command):
    client.publish(mqtt_topic, command)
    print(f"Sent command: {command}")

print("Press arrow keys for FORWARD, LEFT, RIGHT, BACKWARD, or ESC to exit.")

try:
    while True:

        # Check if 'l' key is pressed
        if keyboard.is_pressed("l") and not key_states["lidar"]:
            send_command("LIDAR")
            key_states["lidar"] = True
        elif not keyboard.is_pressed("l") and key_states["lidar"]:
            key_states["lidar"] = False

        # Check if 'u' key is pressed
        if keyboard.is_pressed("u") and not key_states["ultrasound"]:
            send_command("ULTRASOUND")
            key_states["ultrasound"] = True
        elif not keyboard.is_pressed("u") and key_states["ultrasound"]:
            key_states["ultrasound"] = False

         # Check if 's' key is pressed
        if keyboard.is_pressed("s") and not key_states["stop"]:
            send_command("STOP")
            key_states["stop"] = True
        elif not keyboard.is_pressed("l") and key_states["stop"]:
            key_states["stop"] = False
            
        # Check if 'up' key is pressed
        if keyboard.is_pressed("up") and not key_states["up"]:
            send_command("FORWARD")
            key_states["up"] = True
        elif not keyboard.is_pressed("up") and key_states["up"]:
            key_states["up"] = False

        # Check if 'left' key is pressed
        if keyboard.is_pressed("left") and not key_states["left"]:
            send_command("LEFT")
            key_states["left"] = True
        elif not keyboard.is_pressed("left") and key_states["left"]:
            key_states["left"] = False

        # Check if 'right' key is pressed
        if keyboard.is_pressed("right") and not key_states["right"]:
            send_command("RIGHT")
            key_states["right"] = True
        elif not keyboard.is_pressed("right") and key_states["right"]:
            key_states["right"] = False

        # Check if 'down' key is pressed
        if keyboard.is_pressed("down") and not key_states["down"]:
            send_command("BACKWARD")
            key_states["down"] = True
        elif not keyboard.is_pressed("down") and key_states["down"]:
            key_states["down"] = False

        # Exit on 'esc' key press
        if keyboard.is_pressed("esc"):
            print("Exiting...")
            break

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Program interrupted")

finally:
    client.loop_stop()
    client.disconnect()
    
