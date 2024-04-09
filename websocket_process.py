import json
import time
import websocket

FIELDS = ["Sampletime", "Quat1", "Quat2", "Quat3", "Quat4"]

def websocket_process(q, ip_address):
    ws_url = f"ws://{ip_address}:5678/"
    def on_message(ws, message):
        # Put the received message into the queue
        q.put((0, json.loads(message)))

    def on_error(ws, error):
        print("Error: ", error)

    def on_close(ws, close_status_code, close_msg):
        print("### closed ###")

    def on_open(ws):
        print("WebSocket opened")
        sensor_format = [[i, sensor] for sensor in FIELDS for i in range(8)]
        message = json.dumps(sensor_format)
        ws.send(message)
        time.sleep(2)
    def on_message(ws, message):
        q.put((0, json.loads(message)))

    ws = websocket.WebSocketApp(ws_url,
                                on_open=on_open,
                                on_message=on_message,
                                on_error=on_error,
                                on_close=on_close)

    ws.run_forever()