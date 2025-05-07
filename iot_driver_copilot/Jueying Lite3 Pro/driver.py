import os
import json
import socket
import struct
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse, parse_qs

# Environment configuration
DEVICE_IP = os.environ.get("DEVICE_IP", "127.0.0.1")
DEVICE_UDP_PORT = int(os.environ.get("DEVICE_UDP_PORT", "8001"))
SERVER_HOST = os.environ.get("SERVER_HOST", "0.0.0.0")
SERVER_PORT = int(os.environ.get("SERVER_PORT", "8080"))

# UDP Client for sending commands (threadsafe)
class UDPClient:
    def __init__(self, ip, port):
        self.addr = (ip, port)
        self.lock = threading.Lock()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(2.0)  # 2 seconds timeout

    def send(self, data: bytes):
        with self.lock:
            self.sock.sendto(data, self.addr)

    def send_and_receive(self, data: bytes, bufsize=4096):
        with self.lock:
            self.sock.sendto(data, self.addr)
            try:
                resp, _ = self.sock.recvfrom(bufsize)
                return resp
            except socket.timeout:
                return b''

# UDP Sensor Data Receiver
class UDPSensorReceiver(threading.Thread):
    def __init__(self, ip, port):
        super().__init__(daemon=True)
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.ip, self.port))
        self.running = True
        self.latest_data = None
        self.lock = threading.Lock()

    def run(self):
        while self.running:
            try:
                data, _ = self.sock.recvfrom(65535)
                with self.lock:
                    self.latest_data = data
            except Exception:
                continue

    def get_latest(self):
        with self.lock:
            if self.latest_data:
                return self.latest_data
            return None

    def stop(self):
        self.running = False
        self.sock.close()

# Helper functions for protocol serialization/deserialization
def build_cmd_vel_payload(linear_x, linear_y, linear_z, angular_x, angular_y, angular_z):
    # Example, pack as struct: 6 floats, network byte order (big endian)
    return struct.pack('!6f', linear_x, linear_y, linear_z, angular_x, angular_y, angular_z)

def parse_sensor_data(raw_data):
    # For demonstration, try to decode as JSON or fallback to hex
    try:
        return json.loads(raw_data.decode())
    except Exception:
        return {"raw": raw_data.hex()}

# UDP ports for sensor data and commands
# We assume:
#   - DEVICE_UDP_PORT: for commands (POSTs)
#   - DEVICE_UDP_SENSOR_PORT: for sensor data (GET /sdata)
DEVICE_UDP_SENSOR_PORT = int(os.environ.get("DEVICE_UDP_SENSOR_PORT", str(DEVICE_UDP_PORT + 1)))

udp_client = UDPClient(DEVICE_IP, DEVICE_UDP_PORT)
sensor_receiver = UDPSensorReceiver("0.0.0.0", DEVICE_UDP_SENSOR_PORT)
sensor_receiver.start()

# HTTP server
class JueyingLite3ProHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        parsed = urlparse(self.path)
        if parsed.path == "/vctrl":
            # Velocity command
            length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(length)
            try:
                req = json.loads(body)
                linear = req.get("linear", {})
                angular = req.get("angular", {})
                payload = build_cmd_vel_payload(
                    float(linear.get("x", 0.0)),
                    float(linear.get("y", 0.0)),
                    float(linear.get("z", 0.0)),
                    float(angular.get("x", 0.0)),
                    float(angular.get("y", 0.0)),
                    float(angular.get("z", 0.0)),
                )
                udp_client.send(payload)
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps({"status": "ok"}).encode())
            except Exception as e:
                self.send_response(400)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps({"error": str(e)}).encode())

        elif parsed.path == "/navopt":
            # Navigation options
            length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(length)
            try:
                # Here, payload format must be defined per device doc. We'll just forward as JSON bytes.
                udp_client.send(body)
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps({"status": "ok"}).encode())
            except Exception as e:
                self.send_response(400)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps({"error": str(e)}).encode())
        else:
            self.send_response(404)
            self.end_headers()

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == "/sdata":
            # Return latest sensor data as JSON
            data = sensor_receiver.get_latest()
            if data:
                parsed_data = parse_sensor_data(data)
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps(parsed_data).encode())
            else:
                self.send_response(204)
                self.end_headers()
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        return  # Silence default console logging

def run_server():
    server = HTTPServer((SERVER_HOST, SERVER_PORT), JueyingLite3ProHandler)
    print(f"HTTP server running at http://{SERVER_HOST}:{SERVER_PORT}/")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        sensor_receiver.stop()
        server.server_close()

if __name__ == "__main__":
    run_server()