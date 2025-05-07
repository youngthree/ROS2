import os
import threading
import time
import json
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse, parse_qs

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# Configuration from environment variables
ROBOT_IP = os.environ.get('ROBOT_IP', '127.0.0.1')
ROS_DOMAIN_ID = int(os.environ.get('ROS_DOMAIN_ID', '0'))
HTTP_HOST = os.environ.get('HTTP_HOST', '0.0.0.0')
HTTP_PORT = int(os.environ.get('HTTP_PORT', '8080'))

os.environ['ROS_DOMAIN_ID'] = str(ROS_DOMAIN_ID)

rclpy.init(args=None)

class ImuDataBuffer:
    def __init__(self):
        self.lock = threading.Lock()
        self.latest_imu = None

    def update(self, msg):
        with self.lock:
            self.latest_imu = msg

    def get(self):
        with self.lock:
            return self.latest_imu

imu_buffer = ImuDataBuffer()

class RobotROSNode(Node):
    def __init__(self):
        super().__init__('driver_http_server_node')
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        self.map_trigger_pub = self.create_publisher(
            String,
            '/slam/trigger',
            10
        )
        self.localization_pub = self.create_publisher(
            String,
            '/localization/trigger',
            10
        )

    def imu_callback(self, msg):
        imu_buffer.update(msg)

    def publish_velocity(self, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)

    def trigger_map(self):
        msg = String()
        msg.data = 'start'
        self.map_trigger_pub.publish(msg)

    def trigger_localization(self):
        msg = String()
        msg.data = 'start'
        self.localization_pub.publish(msg)

ros_node = RobotROSNode()

def spin_ros():
    rclpy.spin(ros_node)

ros_thread = threading.Thread(target=spin_ros, daemon=True)
ros_thread.start()

def imu_msg_to_dict(msg):
    return {
        "header": {
            "stamp": {
                "sec": msg.header.stamp.sec,
                "nanosec": msg.header.stamp.nanosec
            },
            "frame_id": msg.header.frame_id
        },
        "orientation": {
            "x": msg.orientation.x,
            "y": msg.orientation.y,
            "z": msg.orientation.z,
            "w": msg.orientation.w
        },
        "orientation_covariance": list(msg.orientation_covariance),
        "angular_velocity": {
            "x": msg.angular_velocity.x,
            "y": msg.angular_velocity.y,
            "z": msg.angular_velocity.z,
        },
        "angular_velocity_covariance": list(msg.angular_velocity_covariance),
        "linear_acceleration": {
            "x": msg.linear_acceleration.x,
            "y": msg.linear_acceleration.y,
            "z": msg.linear_acceleration.z,
        },
        "linear_acceleration_covariance": list(msg.linear_acceleration_covariance)
    }

class DriverRequestHandler(BaseHTTPRequestHandler):
    server_version = "JueyingLite3ProHTTPDriver/1.0"

    def _set_headers(self, status=200, content_type="application/json"):
        self.send_response(status)
        self.send_header('Content-type', content_type)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()

    def do_OPTIONS(self):
        self.send_response(200, "ok")
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self):
        parsed_path = urlparse(self.path)
        if parsed_path.path == '/imu':
            self.handle_imu()
        elif parsed_path.path == '/move':
            self.handle_move(parsed_path)
        else:
            self._set_headers(404)
            self.wfile.write(json.dumps({"error": "Not found"}).encode())

    def do_POST(self):
        parsed_path = urlparse(self.path)
        if parsed_path.path == '/map':
            self.handle_map()
        elif parsed_path.path == '/local':
            self.handle_local()
        else:
            self._set_headers(404)
            self.wfile.write(json.dumps({"error": "Not found"}).encode())

    def handle_imu(self):
        imu = imu_buffer.get()
        if imu is None:
            self._set_headers(503)
            self.wfile.write(json.dumps({"error": "IMU data not available"}).encode())
            return
        self._set_headers()
        self.wfile.write(json.dumps(imu_msg_to_dict(imu)).encode())

    def handle_move(self, parsed_path):
        params = parse_qs(parsed_path.query)
        try:
            linear_x = float(params.get('linear_x', [0.0])[0])
            linear_y = float(params.get('linear_y', [0.0])[0])
            angular_z = float(params.get('angular_z', [0.0])[0])
        except Exception:
            self._set_headers(400)
            self.wfile.write(json.dumps({"error": "Invalid parameters"}).encode())
            return
        ros_node.publish_velocity(linear_x=linear_x, linear_y=linear_y, angular_z=angular_z)
        self._set_headers()
        self.wfile.write(json.dumps({"status": "ok", "linear_x": linear_x, "linear_y": linear_y, "angular_z": angular_z}).encode())

    def handle_map(self):
        ros_node.trigger_map()
        self._set_headers()
        self.wfile.write(json.dumps({"status": "mapping_triggered"}).encode())

    def handle_local(self):
        ros_node.trigger_localization()
        self._set_headers()
        self.wfile.write(json.dumps({"status": "localization_triggered"}).encode())

def run_server():
    server = HTTPServer((HTTP_HOST, HTTP_PORT), DriverRequestHandler)
    try:
        print(f"Starting HTTP server at http://{HTTP_HOST}:{HTTP_PORT}")
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        server.server_close()

if __name__ == '__main__':
    run_server()
