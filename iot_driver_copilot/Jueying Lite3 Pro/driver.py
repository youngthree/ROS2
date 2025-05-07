import os
import json
import socket
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse, parse_qs

# Environment Variables Configuration
ROBOT_IP = os.environ.get('ROBOT_IP', '127.0.0.1')
ROBOT_UDP_PORT = int(os.environ.get('ROBOT_UDP_PORT', '9000'))
DRIVER_HTTP_HOST = os.environ.get('DRIVER_HTTP_HOST', '0.0.0.0')
DRIVER_HTTP_PORT = int(os.environ.get('DRIVER_HTTP_PORT', '8080'))

# UDP Protocol Constants (customize as per actual device protocol)
UDP_BUFFER_SIZE = 4096
UDP_TIMEOUT = 2.0

# Supported command types
SUPPORTED_COMMANDS = {"cmd_vel", "simple_cmd", "complex_cmd", "cmd_vel_corrected"}

# UDP Client for sending commands and receiving sensor data
class UdpClient:
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port

    def send_command(self, data: dict):
        payload = json.dumps(data).encode('utf-8')
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.settimeout(UDP_TIMEOUT)
            sock.sendto(payload, (self.ip, self.port))
            try:
                resp, _ = sock.recvfrom(UDP_BUFFER_SIZE)
                return resp.decode('utf-8')
            except socket.timeout:
                return json.dumps({"status": "timeout", "info": "No response from device"})

    def get_sensor(self):
        # Send a special "get_sensor" command (depends on robot's implementation)
        query = {"type": "get_sensor"}
        payload = json.dumps(query).encode('utf-8')
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.settimeout(UDP_TIMEOUT)
            sock.sendto(payload, (self.ip, self.port))
            try:
                resp, _ = sock.recvfrom(UDP_BUFFER_SIZE)
                return resp.decode('utf-8')
            except socket.timeout:
                return json.dumps({"status": "timeout", "info": "No response from device"})

udp_client = UdpClient(ROBOT_IP, ROBOT_UDP_PORT)

class RequestHandler(BaseHTTPRequestHandler):
    def _cors_headers(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')

    def do_OPTIONS(self):
        self.send_response(200)
        self._cors_headers()
        self.end_headers()

    def do_POST(self):
        parsed_path = urlparse(self.path)
        if parsed_path.path == '/cmd':
            content_length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(content_length)
            try:
                data = json.loads(body.decode('utf-8'))
                cmd_type = data.get('type')
                if cmd_type not in SUPPORTED_COMMANDS:
                    self.send_response(400)
                    self._cors_headers()
                    self.send_header('Content-Type', 'application/json')
                    self.end_headers()
                    self.wfile.write(json.dumps({"error": "Invalid command type"}).encode('utf-8'))
                    return

                # Forward the command to the robot and get the response
                resp = udp_client.send_command(data)
                self.send_response(200)
                self._cors_headers()
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(resp.encode('utf-8'))
            except Exception as e:
                self.send_response(400)
                self._cors_headers()
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({"error": str(e)}).encode('utf-8'))
        else:
            self.send_response(404)
            self._cors_headers()
            self.end_headers()

    def do_GET(self):
        parsed_path = urlparse(self.path)
        if parsed_path.path == '/sensor':
            # Retrieve sensor data from the robot
            sensor_data = udp_client.get_sensor()
            self.send_response(200)
            self._cors_headers()
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(sensor_data.encode('utf-8'))
        else:
            self.send_response(404)
            self._cors_headers()
            self.end_headers()

def run_server():
    server_address = (DRIVER_HTTP_HOST, DRIVER_HTTP_PORT)
    httpd = HTTPServer(server_address, RequestHandler)
    print(f"HTTP server listening on {DRIVER_HTTP_HOST}:{DRIVER_HTTP_PORT}")
    httpd.serve_forever()

if __name__ == "__main__":
    run_server()