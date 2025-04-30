import os
import json
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
import socketserver
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

# Environment Variables Configuration
ROBOT_MASTER_URI = os.environ.get("ROS_MASTER_URI", "http://localhost:11311")
ROS_HOSTNAME = os.environ.get("ROS_HOSTNAME", "localhost")
SERVER_HOST = os.environ.get("HTTP_SERVER_HOST", "0.0.0.0")
SERVER_PORT = int(os.environ.get("HTTP_SERVER_PORT", "8080"))

# Initialize ROS node (in thread)
def ros_spin():
    rospy.spin()

rospy.init_node('scout_mini_http_driver', anonymous=True, disable_signals=True)

latest_odom = {}
latest_imu = {}
odom_lock = threading.Lock()
imu_lock = threading.Lock()

def odom_callback(msg):
    global latest_odom
    with odom_lock:
        latest_odom = {
            "header": {
                "seq": msg.header.seq,
                "stamp": msg.header.stamp.to_sec(),
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
    with imu_lock:
        latest_imu = {
            "header": {
                "seq": msg.header.seq,
                "stamp": msg.header.stamp.to_sec(),
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

rospy.Subscriber("/odom", Odometry, odom_callback)
rospy.Subscriber("/imu", Imu, imu_callback)
cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

class ThreadedHTTPServer(socketserver.ThreadingMixIn, HTTPServer):
    daemon_threads = True

class ScoutMiniHTTPRequestHandler(BaseHTTPRequestHandler):
    def _set_json_headers(self, status=200):
        self.send_response(status)
        self.send_header('Content-type', 'application/json')
        self.end_headers()

    def do_GET(self):
        if self.path == '/odom':
            with odom_lock:
                data = latest_odom.copy()
            if not data:
                self._set_json_headers(503)
                self.wfile.write(json.dumps({"error": "Odometry data not available"}).encode())
            else:
                self._set_json_headers(200)
                self.wfile.write(json.dumps(data).encode())
        elif self.path == '/imu':
            with imu_lock:
                data = latest_imu.copy()
            if not data:
                self._set_json_headers(503)
                self.wfile.write(json.dumps({"error": "IMU data not available"}).encode())
            else:
                self._set_json_headers(200)
                self.wfile.write(json.dumps(data).encode())
        else:
            self.send_error(404, "Not Found")

    def do_POST(self):
        if self.path == '/cmd_vel':
            content_length = int(self.headers.get('Content-Length', 0))
            if content_length == 0:
                self._set_json_headers(400)
                self.wfile.write(json.dumps({"error": "Empty request body"}).encode())
                return
            try:
                post_data = self.rfile.read(content_length)
                data = json.loads(post_data.decode())
                twist = Twist()
                twist.linear.x = float(data.get("linear", {}).get("x", 0.0))
                twist.linear.y = float(data.get("linear", {}).get("y", 0.0))
                twist.linear.z = float(data.get("linear", {}).get("z", 0.0))
                twist.angular.x = float(data.get("angular", {}).get("x", 0.0))
                twist.angular.y = float(data.get("angular", {}).get("y", 0.0))
                twist.angular.z = float(data.get("angular", {}).get("z", 0.0))
                cmd_vel_pub.publish(twist)
                self._set_json_headers(200)
                self.wfile.write(json.dumps({"status": "ok"}).encode())
            except Exception as e:
                self._set_json_headers(400)
                self.wfile.write(json.dumps({"error": str(e)}).encode())
        else:
            self.send_error(404, "Not Found")

def main():
    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()
    httpd = ThreadedHTTPServer((SERVER_HOST, SERVER_PORT), ScoutMiniHTTPRequestHandler)
    print(f"Scout Mini HTTP driver running at http://{SERVER_HOST}:{SERVER_PORT} ...")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.server_close()

if __name__ == "__main__":
    main()
