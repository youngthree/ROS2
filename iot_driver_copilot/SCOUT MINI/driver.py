import os
import threading
import time
import json
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse, parse_qs

# ROS dependencies
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# Environment Variables
SERVER_HOST = os.environ.get('SCOUTMINI_DRIVER_HTTP_HOST', '0.0.0.0')
SERVER_PORT = int(os.environ.get('SCOUTMINI_DRIVER_HTTP_PORT', '8080'))
ROS_MASTER_URI = os.environ.get('SCOUTMINI_DRIVER_ROS_MASTER_URI', 'http://localhost:11311')
ROS_NAMESPACE = os.environ.get('SCOUTMINI_DRIVER_ROS_NAMESPACE', '')  # Optional

# ROS Node Initialization
def ros_spin():
    # Set ROS master URI if needed
    if ROS_MASTER_URI:
        os.environ['ROS_MASTER_URI'] = ROS_MASTER_URI
    rospy.init_node('scoutmini_http_driver', anonymous=True, disable_signals=True)
    rospy.spin()

# Command Publisher (Twist)
class ScoutMiniROSBridge:
    def __init__(self):
        cmd_vel_topic = os.path.join(ROS_NAMESPACE, "cmd_vel") if ROS_NAMESPACE else "cmd_vel"
        odom_topic = os.path.join(ROS_NAMESPACE, "odom") if ROS_NAMESPACE else "odom"
        self.cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
        self.odom_topic = odom_topic
        self.latest_odom = None
        self.odom_lock = threading.Lock()
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)

    def odom_callback(self, msg):
        with self.odom_lock:
            self.latest_odom = msg

    def move(self, linear_x=0.0, angular_z=0.0):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)

    def stop(self):
        self.move(0.0, 0.0)

    def get_latest_odom(self):
        with self.odom_lock:
            return self.latest_odom

bridge = None

# Utility to convert ROS Odometry to dict
def odom_to_dict(odom_msg):
    if not odom_msg:
        return None
    return {
        "header": {
            "seq": odom_msg.header.seq,
            "stamp": odom_msg.header.stamp.to_sec(),
            "frame_id": odom_msg.header.frame_id,
        },
        "child_frame_id": odom_msg.child_frame_id,
        "pose": {
            "position": {
                "x": odom_msg.pose.pose.position.x,
                "y": odom_msg.pose.pose.position.y,
                "z": odom_msg.pose.pose.position.z,
            },
            "orientation": {
                "x": odom_msg.pose.pose.orientation.x,
                "y": odom_msg.pose.pose.orientation.y,
                "z": odom_msg.pose.pose.orientation.z,
                "w": odom_msg.pose.pose.orientation.w,
            },
            "covariance": list(odom_msg.pose.covariance),
        },
        "twist": {
            "linear": {
                "x": odom_msg.twist.twist.linear.x,
                "y": odom_msg.twist.twist.linear.y,
                "z": odom_msg.twist.twist.linear.z,
            },
            "angular": {
                "x": odom_msg.twist.twist.angular.x,
                "y": odom_msg.twist.twist.angular.y,
                "z": odom_msg.twist.twist.angular.z,
            },
            "covariance": list(odom_msg.twist.covariance),
        },
    }

class ScoutMiniHTTPRequestHandler(BaseHTTPRequestHandler):
    def _set_headers(self, status=200):
        self.send_response(status)
        self.send_header('Content-type', 'application/json')
        self.end_headers()

    def _respond(self, data, status=200):
        self._set_headers(status)
        self.wfile.write(json.dumps(data).encode('utf-8'))

    def do_GET(self):
        global bridge
        parsed_path = urlparse(self.path)
        path = parsed_path.path
        params = parse_qs(parsed_path.query)
        # Optional speed parameter for move/forward and move/backward
        speed = float(params.get('speed', [0.2])[0])  # Default speed: 0.2 m/s
        angular_speed = float(params.get('angular_speed', [0.5])[0])  # Default angular speed: 0.5 rad/s

        if path == '/move/forward':
            bridge.move(linear_x=abs(speed), angular_z=0.0)
            self._respond({"result": "Moving forward", "speed": abs(speed)})

        elif path == '/move/backward':
            bridge.move(linear_x=-abs(speed), angular_z=0.0)
            self._respond({"result": "Moving backward", "speed": -abs(speed)})

        elif path == '/turn/left':
            bridge.move(linear_x=0.0, angular_z=abs(angular_speed))
            self._respond({"result": "Turning left", "angular_speed": abs(angular_speed)})

        elif path == '/turn/right':
            bridge.move(linear_x=0.0, angular_z=-abs(angular_speed))
            self._respond({"result": "Turning right", "angular_speed": -abs(angular_speed)})

        elif path == '/stop':
            bridge.stop()
            self._respond({"result": "Stopped"})

        elif path == '/odom':
            odom_msg = bridge.get_latest_odom()
            if odom_msg:
                odom_dict = odom_to_dict(odom_msg)
                self._respond(odom_dict)
            else:
                self._respond({"error": "Odometry data not available"}, status=503)

        else:
            self._respond({"error": "Endpoint not found"}, status=404)

def start_ros_node():
    # To ensure ROS node runs in the background
    thread = threading.Thread(target=ros_spin, daemon=True)
    thread.start()
    # Wait until rospy is ready
    while not rospy.core.is_initialized():
        time.sleep(0.1)
    global bridge
    bridge = ScoutMiniROSBridge()

def run_server():
    httpd = HTTPServer((SERVER_HOST, SERVER_PORT), ScoutMiniHTTPRequestHandler)
    print(f"SCOUT MINI HTTP driver running at http://{SERVER_HOST}:{SERVER_PORT}")
    httpd.serve_forever()

if __name__ == '__main__':
    start_ros_node()
    run_server()