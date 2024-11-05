import websocket

def on_open(ws):
    print("WebSocket connection opened.")
    ws.send("Hello")  # Send the message "Hello"
    print("Message sent: Hello")
    ws.close()  # Close the connection after sending

def on_message(ws, message):
    print("Received message:", message)
 

def on_close(ws, close_status_code, close_msg):
    print("WebSocket connection closed.")

if __name__ == "__main__":
    ws_url = "ws://192.168.1.42:8765"  # Change this to your server's IP and port
    ws = websocket.WebSocketApp(ws_url,
                                on_open=on_open,
                                on_message=on_message,
                                on_error=None,
                                on_close=on_close)

    ws.run_forever()
