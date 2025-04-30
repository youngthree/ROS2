import os
import threading
import time
import struct
from flask import Flask, jsonify, request
import rospy
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

# Load configuration from environment variables
HTTP_HOST = os.environ.get("HTTP_HOST", "0.0.0.0")
HTTP_PORT = int(os.environ.get("HTTP_PORT", "8080"))
ROS_MASTER_URI = os.environ.get("ROS_MASTER_URI", "http://localhost:11311")
ROBOT_NAMESPACE = os.environ.get("ROBOT_NAMESPACE", "")  # Optional

# ROS topic names, can also be set via env vars
TOPIC_CMD_VEL = os.environ.get("TOPIC_CMD_VEL", "/cmd_vel")
TOPIC_ODOM = os.environ.get("TOPIC_ODOM", "/odom")

# Movement parameters
DEFAULT_LINEAR_SPEED = float(os.environ.get("DEFAULT_LINEAR_SPEED", "0.2"))
DEFAULT_ANGULAR_SPEED = float(os.environ.get("DEFAULT_ANGULAR_SPEED", "0.5"))

# ROS initialization flag and odometry cache
ros_initialized = False
odom_cache = {}
odom_lock = threading.Lock()

# Flask app
app = Flask(__name__)

# Ensure ROS is initialized only once and in a background thread for Flask compatibility
def ros_init():
    global ros_initialized
    if not ros_initialized:
        rospy.init_node('scout_mini_http_driver', anonymous=True, disable_signals=True)
        ros_initialized = True

def ros_spin_thread():
    while not rospy.is_shutdown():
        rospy.sleep(0.1)

def start_ros():
    ros_init()
    # Subscribe to odometry
    rospy.Subscriber(TOPIC_ODOM, Odometry, odom_callback)
    # Start a background spinning thread
    t = threading.Thread(target=ros_spin_thread, daemon=True)
    t.start()

def odom_callback(msg):
    with odom_lock:
        odom_cache['seq'] = msg.header.seq
        odom_cache['stamp'] = msg.header.stamp.to_sec()
        odom_cache['frame_id'] = msg.header.frame_id
        odom_cache['child_frame_id'] = msg.child_frame_id

        odom_cache['pose'] = {
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
        }
        odom_cache['twist'] = {
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
        }

def get_cmd_vel_publisher():
    return rospy.Publisher(TOPIC_CMD_VEL, Twist, queue_size=1)

# Utility to send velocity command
def send_velocity(linear_x=0.0, linear_y=0.0, angular_z=0.0, duration=None):
    pub = get_cmd_vel_publisher()
    twist = Twist()
    twist.linear.x = linear_x
    twist.linear.y = linear_y
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = angular_z
    pub.publish(twist)
    if duration:
        time.sleep(duration)
        # Send stop
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

@app.route("/move/forward", methods=["GET"])
def move_forward():
    speed = float(request.args.get("speed", DEFAULT_LINEAR_SPEED))
    duration = float(request.args.get("duration", "0"))
    send_velocity(linear_x=speed, duration=duration if duration > 0 else None)
    return jsonify({"result": "Moving forward", "speed": speed, "duration": duration})

@app.route("/move/backward", methods=["GET"])
def move_backward():
    speed = float(request.args.get("speed", DEFAULT_LINEAR_SPEED))
    duration = float(request.args.get("duration", "0"))
    send_velocity(linear_x=-speed, duration=duration if duration > 0 else None)
    return jsonify({"result": "Moving backward", "speed": speed, "duration": duration})

@app.route("/turn/left", methods=["GET"])
def turn_left():
    angular = float(request.args.get("angular_speed", DEFAULT_ANGULAR_SPEED))
    duration = float(request.args.get("duration", "0"))
    send_velocity(angular_z=angular, duration=duration if duration > 0 else None)
    return jsonify({"result": "Turning left", "angular_speed": angular, "duration": duration})

@app.route("/turn/right", methods=["GET"])
def turn_right():
    angular = float(request.args.get("angular_speed", DEFAULT_ANGULAR_SPEED))
    duration = float(request.args.get("duration", "0"))
    send_velocity(angular_z=-angular, duration=duration if duration > 0 else None)
    return jsonify({"result": "Turning right", "angular_speed": angular, "duration": duration})

@app.route("/stop", methods=["GET"])
def stop():
    send_velocity(0.0, 0.0, 0.0)
    return jsonify({"result": "Stopped"})

@app.route("/odom", methods=["GET"])
def odom():
    with odom_lock:
        if odom_cache:
            return jsonify({"odometry": odom_cache})
    return jsonify({"error": "No odometry data available"}), 503

def main():
    start_ros()
    app.run(host=HTTP_HOST, port=HTTP_PORT, threaded=True)

if __name__ == "__main__":
    main()
