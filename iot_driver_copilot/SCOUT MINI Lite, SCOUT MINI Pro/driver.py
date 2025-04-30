import os
import json
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Header

# Read environment variables for ROS and server configuration
ROS_MASTER_URI = os.environ.get('ROS_MASTER_URI', 'http://localhost:11311')
ROS_HOSTNAME = os.environ.get('ROS_HOSTNAME', 'localhost')
SERVER_HOST = os.environ.get('SERVER_HOST', '0.0.0.0')
SERVER_PORT = int(os.environ.get('SERVER_PORT', '8080'))

# Store latest ROS messages
latest_odom = None
latest_imu = None
latest_cmd_vel = None

# Lock for thread safety
odom_lock = threading.Lock()
imu_lock = threading.Lock()
cmd_vel_lock = threading.Lock()

def ros_spin_thread():
    rospy.spin()

def odom_callback(msg):
    global latest_odom
    with odom_lock:
        latest_odom = msg

def imu_callback(msg):
    global latest_imu
    with imu_lock:
        latest_imu = msg

def cmd_vel_callback(msg):
    global latest_cmd_vel
    with cmd_vel_lock:
        latest_cmd_vel = msg

def ros_init():
    if not rospy.core.is_initialized():
        rospy.init_node('scout_mini_lite_http_driver', anonymous=True, disable_signals=True)
    rospy.Subscriber('/odom', Odometry, odom_callback, queue_size=1)
    rospy.Subscriber('/imu', Imu, imu_callback, queue_size=1)
    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback, queue_size=1)

def odom_to_dict(msg):
    return {
        'header': {
            'seq': msg.header.seq,
            'stamp': {'secs': msg.header.stamp.secs, 'nsecs': msg.header.stamp.nsecs},
            'frame_id': msg.header.frame_id
        },
        'child_frame_id': msg.child_frame_id,
        'pose': {
            'pose': {
                'position': {
                    'x': msg.pose.pose.position.x,
                    'y': msg.pose.pose.position.y,
                    'z': msg.pose.pose.position.z
                },
                'orientation': {
                    'x': msg.pose.pose.orientation.x,
                    'y': msg.pose.pose.orientation.y,
                    'z': msg.pose.pose.orientation.z,
                    'w': msg.pose.pose.orientation.w
                }
            },
            'covariance': list(msg.pose.covariance)
        },
        'twist': {
            'twist': {
                'linear': {
                    'x': msg.twist.twist.linear.x,
                    'y': msg.twist.twist.linear.y,
                    'z': msg.twist.twist.linear.z
                },
                'angular': {
                    'x': msg.twist.twist.angular.x,
                    'y': msg.twist.twist.angular.y,
                    'z': msg.twist.twist.angular.z
                }
            },
            'covariance': list(msg.twist.covariance)
        }
    }

def imu_to_dict(msg):
    return {
        'header': {
            'seq': msg.header.seq,
            'stamp': {'secs': msg.header.stamp.secs, 'nsecs': msg.header.stamp.nsecs},
            'frame_id': msg.header.frame_id
        },
        'orientation': {
            'x': msg.orientation.x,
            'y': msg.orientation.y,
            'z': msg.orientation.z,
            'w': msg.orientation.w
        },
        'orientation_covariance': list(msg.orientation_covariance),
        'angular_velocity': {
            'x': msg.angular_velocity.x,
            'y': msg.angular_velocity.y,
            'z': msg.angular_velocity.z
        },
        'angular_velocity_covariance': list(msg.angular_velocity_covariance),
        'linear_acceleration': {
            'x': msg.linear_acceleration.x,
            'y': msg.linear_acceleration.y,
            'z': msg.linear_acceleration.z
        },
        'linear_acceleration_covariance': list(msg.linear_acceleration_covariance)
    }

def twist_to_dict(msg):
    return {
        'linear': {
            'x': msg.linear.x,
            'y': msg.linear.y,
            'z': msg.linear.z
        },
        'angular': {
            'x': msg.angular.x,
            'y': msg.angular.y,
            'z': msg.angular.z
        }
    }

class ScoutMiniHTTPRequestHandler(BaseHTTPRequestHandler):

    def _set_headers(self, code=200, content_type='application/json'):
        self.send_response(code)
        self.send_header('Content-type', content_type)
        self.end_headers()

    def do_GET(self):
        if self.path == '/odom':
            with odom_lock:
                msg = latest_odom
            if msg is None:
                self._set_headers(503)
                self.wfile.write(json.dumps({'error': 'No odometry data yet.'}).encode())
                return
            self._set_headers()
            self.wfile.write(json.dumps(odom_to_dict(msg)).encode())
        elif self.path == '/imu':
            with imu_lock:
                msg = latest_imu
            if msg is None:
                self._set_headers(503)
                self.wfile.write(json.dumps({'error': 'No IMU data yet.'}).encode())
                return
            self._set_headers()
            self.wfile.write(json.dumps(imu_to_dict(msg)).encode())
        elif self.path == '/cmd_vel':
            with cmd_vel_lock:
                msg = latest_cmd_vel
            if msg is None:
                self._set_headers(503)
                self.wfile.write(json.dumps({'error': 'No cmd_vel data yet.'}).encode())
                return
            self._set_headers()
            self.wfile.write(json.dumps(twist_to_dict(msg)).encode())
        else:
            self._set_headers(404)
            self.wfile.write(json.dumps({'error': 'Not Found'}).encode())

    def do_POST(self):
        self._set_headers(405)
        self.wfile.write(json.dumps({'error': 'POST method not allowed'}).encode())

def run():
    os.environ['ROS_MASTER_URI'] = ROS_MASTER_URI
    os.environ['ROS_HOSTNAME'] = ROS_HOSTNAME
    ros_init()
    threading.Thread(target=ros_spin_thread, daemon=True).start()
    server = HTTPServer((SERVER_HOST, SERVER_PORT), ScoutMiniHTTPRequestHandler)
    print(f"Scout Mini HTTP Driver running at http://{SERVER_HOST}:{SERVER_PORT}")
    server.serve_forever()

if __name__ == '__main__':
    run()