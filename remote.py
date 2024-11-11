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
    "right": False
}


def send_command(command):
    client.publish(mqtt_topic, command)
    print(f"Sent command: {command}")

print("Press arrow keys for FORWARD, LEFT, RIGHT, BACKWARD, or ESC to exit.")

try:
    while True:
      

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
    
