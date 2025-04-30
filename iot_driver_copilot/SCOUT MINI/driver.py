import os
import json
from http.server import BaseHTTPRequestHandler, HTTPServer
import threading

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

# Configuration via environment variables
ROS_MASTER_URI = os.environ.get("ROS_MASTER_URI", "http://localhost:11311")
ROS_IP = os.environ.get("ROS_IP", "127.0.0.1")
DEVICE_NAMESPACE = os.environ.get("DEVICE_NAMESPACE", "")
HTTP_SERVER_HOST = os.environ.get("HTTP_SERVER_HOST", "0.0.0.0")
HTTP_SERVER_PORT = int(os.environ.get("HTTP_SERVER_PORT", "8080"))
ODOM_TOPIC = os.environ.get("ODOM_TOPIC", "/odom")
IMU_TOPIC = os.environ.get("IMU_TOPIC", "/imu")
CMD_VEL_TOPIC = os.environ.get("CMD_VEL_TOPIC", "/cmd_vel")

os.environ["ROS_MASTER_URI"] = ROS_MASTER_URI
os.environ["ROS_IP"] = ROS_IP

# ROS Node initialization in a thread-safe way
rospy_inited = threading.Event()

def ros_spin():
    rospy.init_node('scoutmini_http_driver', anonymous=True, disable_signals=True)
    rospy_inited.set()
    rospy.spin()

threading.Thread(target=ros_spin, daemon=True).start()
rospy_inited.wait(timeout=10)

latest_odom = {}
latest_imu = {}

def odom_callback(msg):
    global latest_odom
    latest_odom = {
        "header": {
            "seq": msg.header.seq,
            "stamp": {
                "secs": msg.header.stamp.secs,
                "nsecs": msg.header.stamp.nsecs
            },
            "frame_id": msg.header.frame_id
        },
        "child_frame_id": msg.child_frame_id,
        "pose": {
            "pose": {
                "position": {
                    "x": msg.pose.pose.position.x,
                    "y": msg.pose.pose.position.y,
                    "z": msg.pose.pose.position.z
                },
                "orientation": {
                    "x": msg.pose.pose.orientation.x,
                    "y": msg.pose.pose.orientation.y,
                    "z": msg.pose.pose.orientation.z,
                    "w": msg.pose.pose.orientation.w
                }
            },
            "covariance": list(msg.pose.covariance)
        },
        "twist": {
            "twist": {
                "linear": {
                    "x": msg.twist.twist.linear.x,
                    "y": msg.twist.twist.linear.y,
                    "z": msg.twist.twist.linear.z
                },
                "angular": {
                    "x": msg.twist.twist.angular.x,
                    "y": msg.twist.twist.angular.y,
                    "z": msg.twist.twist.angular.z
                }
            },
            "covariance": list(msg.twist.covariance)
        }
    }

def imu_callback(msg):
    global latest_imu
    latest_imu = {
        "header": {
            "seq": msg.header.seq,
            "stamp": {
                "secs": msg.header.stamp.secs,
                "nsecs": msg.header.stamp.nsecs
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
            "z": msg.angular_velocity.z
        },
        "angular_velocity_covariance": list(msg.angular_velocity_covariance),
        "linear_acceleration": {
            "x": msg.linear_acceleration.x,
            "y": msg.linear_acceleration.y,
            "z": msg.linear_acceleration.z
        },
        "linear_acceleration_covariance": list(msg.linear_acceleration_covariance)
    }

# Subscribe to topics
rospy.Subscriber(ODOM_TOPIC, Odometry, odom_callback, queue_size=1)
rospy.Subscriber(IMU_TOPIC, Imu, imu_callback, queue_size=1)

cmd_vel_publisher = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=1)

class ScoutMiniHTTPRequestHandler(BaseHTTPRequestHandler):
    def _set_headers(self, status=200, content_type="application/json"):
        self.send_response(status)
        self.send_header('Content-type', content_type)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()

    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header('Access-Control-Allow-Methods', 'POST, GET, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()

    def do_GET(self):
        if self.path == '/odom':
            self._set_headers()
            self.wfile.write(json.dumps(latest_odom).encode('utf-8'))
        elif self.path == '/imu':
            self._set_headers()
            self.wfile.write(json.dumps(latest_imu).encode('utf-8'))
        else:
            self._set_headers(404)
            self.wfile.write(json.dumps({"error": "Not Found"}).encode('utf-8'))

    def do_POST(self):
        if self.path == '/cmd_vel':
            content_length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(content_length)
            try:
                data = json.loads(body)
                twist = Twist()
                # Fill linear
                if "linear" in data:
                    linear = data["linear"]
                    twist.linear.x = float(linear.get("x", 0.0))
                    twist.linear.y = float(linear.get("y", 0.0))
                    twist.linear.z = float(linear.get("z", 0.0))
                # Fill angular
                if "angular" in data:
                    angular = data["angular"]
                    twist.angular.x = float(angular.get("x", 0.0))
                    twist.angular.y = float(angular.get("y", 0.0))
                    twist.angular.z = float(angular.get("z", 0.0))
                cmd_vel_publisher.publish(twist)
                self._set_headers(200)
                self.wfile.write(json.dumps({"status": "ok"}).encode('utf-8'))
            except Exception as e:
                self._set_headers(400)
                self.wfile.write(json.dumps({"error": str(e)}).encode('utf-8'))
        else:
            self._set_headers(404)
            self.wfile.write(json.dumps({"error": "Not Found"}).encode('utf-8'))

def run_server():
    server_address = (HTTP_SERVER_HOST, HTTP_SERVER_PORT)
    httpd = HTTPServer(server_address, ScoutMiniHTTPRequestHandler)
    print(f"Starting HTTP server at http://{HTTP_SERVER_HOST}:{HTTP_SERVER_PORT}")
    httpd.serve_forever()

if __name__ == "__main__":
    run_server()