import json
import time
import websocket

class DataStreamClient:
    """
    Client for managing SageMotion data streaming.

    Attributes:
        ip_address (str): The IP address of the SageMotion hub. This could be
            192.168.137.1 for ethernet, 192.168.12.1 for hotspot, or 
            192.168.1.xxx for wifi client.
        port (int): The port on which the WebSocket server is running.
            Defaults to 5678.
        queue (queue.Queue): A queue to store the data received from 
            the server.
        requested_data (list): Data to send to the server upon connection. 
        ws_url (str): The URL for the WebSocket connection.
    """

    def __init__(self, ip_address, queue, port=5678, requested_data=None):
        """
        Initializes the DataStreamClient instance with server details and 
            connection parameters.

        Args:
            ip_address (str): The IP address of the WebSocket server.
            queue (queue.Queue): A queue to store the data received from 
                the server.
            port (int, optional): The port on which the WebSocket server is 
                running. Defaults to 5678.
            requested_data (list): Data to send to the server upon connection.
                This should be a list of lists. The first element in the list 
                will be the sensor (int) and the second will be the text of 
                the value that you want. For example, `[0, 'Accel_x']` will get
                the x axis value of the accelerometer for the first sensor. 
                If you want to get custom parameters, you can use the `None` 
                keyword like `[None, "angle"]`
        """
        self.ip_address = ip_address
        self.port = port
        self.queue = queue
        self.requested_data = requested_data
        self.ws_url = f"ws://{self.ip_address}:{self.port}/"

    def _on_message(self, ws, message):
        """
        Callback for handling messages received from the server.

        Args:
            ws (websocket.WebSocketApp): The WebSocket application instance.
            message (str): The message received from the server.
        """
        data = json.loads(message)
        self.queue.put((0, data))

    def _on_error(self, ws, error):
        """
        Callback for handling errors occurred in the WebSocket connection.

        Args:
            ws (websocket.WebSocketApp): The WebSocket application instance.
            error (str): The error message.
        """
        print("Error: ", error)

    def _on_close(self, ws, close_status_code, close_msg):
        """
        Callback for handling the closure of the WebSocket connection.

        Args:
            ws (websocket.WebSocketApp): The WebSocket application instance.
            close_status_code (int): The status code of the closure.
            close_msg (str): The message regarding the closure.
        """
        print("### closed ###")

    def _on_open(self, ws):
        """
        Callback for handling the opening of the WebSocket connection.

        Sends the requested data to the server upon opening the connection.

        Args:
            ws (websocket.WebSocketApp): The WebSocket application instance.
        """
        print("WebSocket opened")
        message = json.dumps(self.requested_data)
        ws.send(message)
        time.sleep(2)  # Sleep to ensure message is sent before continuing

    def run_forever(self):
        """
        Starts the WebSocket connection using the pre-defined config
            and runs it indefinitely.
        """
        ws = websocket.WebSocketApp(
            self.ws_url,
            on_open=self._on_open,
            on_message=self._on_message,
            on_error=self._on_error,
            on_close=self._on_close,
        )
        ws.run_forever()